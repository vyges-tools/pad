// SPDX-License-Identifier: Apache-2.0
//! Filling the gaps between pads.
//!
//! Once the pads are placed, whatever is left of each row has to be packed solid with filler
//! cells: an IO ring with a hole in it is not a ring, because the power rails run through the
//! cells rather than over them. Widest filler first, then narrower ones for the remainder.
//!
//! Nothing here touches a database.

/// The name every filler instance starts with.
pub const FILL_PREFIX: &str = "IO_FILL_";

/// A filler cell on offer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Filler {
    pub master: String,
    /// Its size **along the row**, after the row's own orientation.
    pub width: i32,
    /// Whether this one may be placed over something already there.
    pub overlapping: bool,
}

/// One filler to create.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Fill {
    pub name: String,
    pub master: String,
    /// Position along the row, in database units. The caller snaps it to a site.
    pub at: i32,
    pub overlapping: bool,
}

/// Why a row could not be filled. Both are hard errors in the reference.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Unfilled {
    /// The gap is not a whole number of sites, so filling it would leave a sliver.
    Ragged { span: (i32, i32) },
    /// The fillers on offer cannot make up the remaining sites.
    Short { span: (i32, i32) },
}

/// **F1** — what is left of a row once the pads in it are taken out.
///
/// Everything here is **half-open**: `(lower, upper)` with `upper` exclusive, so a width is simply
/// `upper - lower` and a gap begins exactly where the cell before it ends.
///
/// ⚠️ No conversion of any kind. A database rectangle's upper bound is already the exclusive edge
/// — a cell 3000 wide at 458000 runs to 461000 — so nudging the bounds to "make them closed"
/// leaves a one-unit sliver against every pad, and a sliver is not a whole number of sites, so
/// every row is then rejected as ragged.
pub fn gaps(row: (i32, i32), occupied: &[(i32, i32)]) -> Vec<(i32, i32)> {
    let mut taken: Vec<(i32, i32)> = occupied.to_vec();
    taken.sort_unstable();

    let mut out = Vec::new();
    let mut at = row.0;
    let end = row.1;
    for (lo, hi) in taken {
        if lo > at {
            out.push((at, lo.min(end)));
        }
        at = at.max(hi);
    }
    if at < end {
        out.push((at, end));
    }
    out.retain(|&(a, b)| b > a);
    out
}

/// **F2** — pack one row's gaps with fillers.
///
/// Fillers are tried **widest first**, and the order among equal widths is the order given: a
/// stable sort, so a caller listing two same-width cells gets the first one.
///
/// ⚠️ A filler marked *overlapping* keeps being placed after the gap is full. That is the point of
/// marking it: those cells are meant to sit over their neighbours rather than beside them. The
/// loop still ends, because the remaining site count is what stops it.
///
/// `row_start` is the row's own beginning; positions come out as `row_start + site_width * index`,
/// which is why they are in units of **sites** rather than of the row's step.
pub fn fill_row(
    row_name: &str,
    row_span: (i32, i32),
    occupied: &[(i32, i32)],
    row_start: i32,
    site_width: i32,
    site_index_of: &dyn Fn(i32) -> i32,
    fillers: &[Filler],
) -> Result<Vec<Fill>, Unfilled> {
    if site_width <= 0 {
        return Ok(Vec::new());
    }
    let mut sorted: Vec<&Filler> = fillers.iter().collect();
    sorted.sort_by(|a, b| b.width.cmp(&a.width)); // stable: equal widths keep their order

    let mut out = Vec::new();
    for (group, span) in gaps(row_span, occupied).into_iter().enumerate() {
        let width = span.1 - span.0;
        if width % site_width != 0 {
            return Err(Unfilled::Ragged { span });
        }
        let mut sites = width / site_width;
        let start_index = site_index_of(span.0);
        let mut offset = 0;

        for f in &sorted {
            let cells = f.width / site_width;
            if cells <= 0 {
                continue; // a filler narrower than a site would never finish the gap
            }
            while cells <= sites || f.overlapping {
                out.push(Fill {
                    // ⚠️ The offset in the name is the one BEFORE this cell is counted, so the
                    // first cell of every group ends in `_0`.
                    name: format!("{FILL_PREFIX}{row_name}_{group}_{offset}"),
                    master: f.master.clone(),
                    at: row_start + site_width * (start_index + offset),
                    overlapping: f.overlapping,
                });
                offset += cells;
                sites -= cells;
                if sites <= 0 {
                    break;
                }
            }
            if sites <= 0 {
                break;
            }
        }
        if sites > 0 {
            return Err(Unfilled::Short { span });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn filler(master: &str, width: i32) -> Filler {
        Filler { master: master.into(), width, overlapping: false }
    }

    fn index_by(site: i32) -> impl Fn(i32) -> i32 {
        move |p| p / site
    }

    #[test]
    fn an_empty_row_is_one_gap() {
        assert_eq!(gaps((0, 1000), &[]), vec![(0, 1000)]);
    }

    #[test]
    fn a_gap_starts_exactly_where_the_cell_before_it_ends() {
        // ⚠️ A cell at 100 that is 100 wide runs to 200, and the gap after it starts AT 200.
        // Starting it at 201 leaves a one-unit sliver and the row is then unfillable.
        assert_eq!(gaps((0, 1000), &[(100, 200)]), vec![(0, 100), (200, 1000)]);
    }

    #[test]
    fn pads_at_the_ends_leave_no_gap_there() {
        assert_eq!(gaps((0, 1000), &[(0, 100)]), vec![(100, 1000)]);
        assert_eq!(gaps((0, 1000), &[(900, 1000)]), vec![(0, 900)]);
        assert_eq!(gaps((0, 1000), &[(0, 1000)]), Vec::new());
    }

    #[test]
    fn touching_and_overlapping_pads_merge_into_one_run() {
        assert_eq!(gaps((0, 1000), &[(100, 200), (200, 300)]), vec![(0, 100), (300, 1000)]);
        assert_eq!(gaps((0, 1000), &[(100, 300), (150, 200)]), vec![(0, 100), (300, 1000)]);
    }

    #[test]
    fn pads_are_taken_in_position_order_however_they_are_given() {
        let jumbled = gaps((0, 1000), &[(700, 800), (100, 200)]);
        assert_eq!(jumbled, vec![(0, 100), (200, 700), (800, 1000)]);
    }

    #[test]
    fn a_row_is_packed_widest_filler_first() {
        let f = vec![filler("SMALL", 10), filler("BIG", 100)];
        let out =
            fill_row("IO_NORTH", (0, 1000), &[], 0, 10, &index_by(10), &f).unwrap();
        assert_eq!(out.len(), 10, "ten of the wide one, nothing left over");
        assert!(out.iter().all(|c| c.master == "BIG"));
        assert_eq!(out[0].name, "IO_FILL_IO_NORTH_0_0");
        assert_eq!(out[1].name, "IO_FILL_IO_NORTH_0_10", "named by site offset, not by count");
        assert_eq!((out[0].at, out[1].at), (0, 100));
    }

    #[test]
    fn narrower_fillers_take_the_remainder() {
        let f = vec![filler("BIG", 100), filler("SMALL", 10)];
        let out = fill_row("IO_NORTH", (0, 250), &[], 0, 10, &index_by(10), &f).unwrap();
        let big = out.iter().filter(|c| c.master == "BIG").count();
        let small = out.iter().filter(|c| c.master == "SMALL").count();
        assert_eq!((big, small), (2, 5), "two wide, then five narrow for the last 50");
    }

    #[test]
    fn each_gap_gets_its_own_group_number() {
        let f = vec![filler("F", 10)];
        let out = fill_row("IO_WEST", (0, 300), &[(100, 200)], 0, 10, &index_by(10), &f).unwrap();
        assert_eq!(out.first().unwrap().name, "IO_FILL_IO_WEST_0_0");
        assert!(out.iter().any(|c| c.name == "IO_FILL_IO_WEST_1_0"), "the second gap restarts at _0");
    }

    #[test]
    fn the_second_gap_is_positioned_from_its_own_start() {
        let f = vec![filler("F", 100)];
        let out = fill_row("IO_WEST", (0, 300), &[(100, 200)], 0, 100, &index_by(100), &f).unwrap();
        assert_eq!(out.iter().map(|c| c.at).collect::<Vec<_>>(), vec![0, 200]);
    }

    #[test]
    fn a_gap_that_is_not_a_whole_number_of_sites_is_refused() {
        let f = vec![filler("F", 10)];
        // 0..1000 minus a pad ending mid-site.
        let err = fill_row("IO_NORTH", (0, 1000), &[(100, 195)], 0, 10, &index_by(10), &f)
            .unwrap_err();
        assert!(matches!(err, Unfilled::Ragged { .. }), "got {err:?}");
    }

    #[test]
    fn a_gap_the_fillers_cannot_close_is_refused_rather_than_left_open() {
        // ⚠️ A hole in the ring breaks the power rails, so a partial fill is worse than an error.
        let f = vec![filler("WIDE", 300)];
        let err =
            fill_row("IO_NORTH", (0, 100), &[], 0, 10, &index_by(10), &f).unwrap_err();
        assert!(matches!(err, Unfilled::Short { .. }), "got {err:?}");
    }

    #[test]
    fn an_overlapping_filler_keeps_going_past_a_full_gap() {
        // ⚠️ The behaviour that reads like a runaway loop. It is bounded by the site count.
        let f = vec![Filler { master: "OVER".into(), width: 300, overlapping: true }];
        let out = fill_row("IO_NORTH", (0, 100), &[], 0, 10, &index_by(10), &f).unwrap();
        assert_eq!(out.len(), 1, "placed despite being wider than the gap");
        assert!(out[0].overlapping);
    }

    #[test]
    fn equal_widths_keep_the_order_they_were_given() {
        let f = vec![filler("FIRST", 10), filler("SECOND", 10)];
        let out = fill_row("IO_NORTH", (0, 10), &[], 0, 10, &index_by(10), &f).unwrap();
        assert_eq!(out[0].master, "FIRST");
    }

    #[test]
    fn a_filler_narrower_than_a_site_is_passed_over_rather_than_looping_forever() {
        let f = vec![filler("TINY", 4), filler("EXACT", 10)];
        let out = fill_row("IO_NORTH", (0, 20), &[], 0, 10, &index_by(10), &f).unwrap();
        assert!(out.iter().all(|c| c.master == "EXACT"));
    }
}
