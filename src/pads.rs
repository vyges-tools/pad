// SPDX-License-Identifier: Apache-2.0
//! Distributing a list of pads along one side of the ring.
//!
//! `place_pad` puts one cell where the caller says. This puts *many* cells where they will fit,
//! spread evenly, sliding each one clear of whatever is already there.
//!
//! Nothing here touches a database: the caller supplies a conflict test and receives placements.

use crate::clearance::Refusal;
use crate::orient::Orient;
use crate::place::{oriented_size, place_in_row, Edge, Placement, RowGeom};

type Rect = (i32, i32, i32, i32);

/// A row seen as a one-dimensional track: a start, an end, and a site granularity.
#[derive(Debug, Clone)]
pub struct Track {
    pub row: RowGeom,
    pub edge: Edge,
    /// `min(site width, site height)` — the granularity spacing is rounded down to.
    pub site_width: i32,
}

impl Track {
    /// ℹ️ Derived from the edge rather than read back from the row. The two agree by construction:
    /// the ring builder is what set each row's direction, and it set the top and bottom rows
    /// horizontal.
    pub fn horizontal(&self) -> bool {
        matches!(self.edge, Edge::North | Edge::South)
    }

    /// The row's extent along its own length.
    pub fn along(&self, r: Rect) -> (i32, i32) {
        if self.horizontal() {
            (r.0, r.2)
        } else {
            (r.1, r.3)
        }
    }

    pub fn start(&self) -> i32 {
        self.along(self.row.bbox).0
    }

    pub fn end(&self) -> i32 {
        self.along(self.row.bbox).1
    }

    pub fn width(&self) -> i32 {
        self.end() - self.start()
    }

    /// **P1** — the site **index** nearest a position along the row.
    ///
    /// ⚠️ Returns an index, not a coordinate. Rounds to nearest (not down), and clamps into the
    /// row: a position before the row start gives site 0 rather than a negative index.
    pub fn snap_to_site(&self, location: i32) -> i32 {
        let origin = if self.horizontal() { self.row.origin.0 } else { self.row.origin.1 };
        let relative = location - origin;
        let count = (relative as f64 / self.row.spacing as f64).round() as i32;
        count.clamp(0, self.row.site_count)
    }

    pub fn index_to_pos(&self, index: i32) -> i32 {
        let origin = if self.horizontal() { self.row.origin.0 } else { self.row.origin.1 };
        origin + index * self.row.spacing
    }
}

/// A pad waiting to be placed. `size` is the master's own width and height, unrotated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pad {
    pub name: String,
    pub master: String,
    pub size: (i32, i32),
}

/// Why a pad could not be placed. Either is fatal to the whole command, as in the reference.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Refused {
    /// The cell ran off the end of the row.
    OutOfRow { name: String, at: Rect },
    /// The cell hit something and shifting could not clear it.
    ///
    /// ⚠️ Carries **where** it was tried. A refusal that names only the cell and the blocker leaves
    /// the reader unable to tell a placer that chose badly from a check that is too strict — the
    /// two want opposite fixes.
    Blocked { name: String, at: Rect, why: Refusal },
}

/// **P2** — how much of the row's length a pad consumes.
///
/// Measured **after** the row's own orientation, because a row turned on its side swaps the
/// master's width and height.
pub fn pad_width(t: &Track, size: (i32, i32)) -> i32 {
    let (dx, dy) = oriented_size(size.0, size.1, t.row.orient);
    if t.horizontal() {
        dx
    } else {
        dy
    }
}

/// **P3** — the gap left between neighbouring pads.
///
/// The leftover room split evenly into one more gap than there are pads — a gap before the first
/// and after the last as well as between — then **rounded down to a whole site**.
///
/// ⚠️ Computed in `f32`, matching the reference's `float`. At ring dimensions the values run to
/// millions of database units, past `f32`'s 24-bit mantissa, so the rounding is observable: `f64`
/// here would give a different spacing on a large die.
pub fn target_spacing(t: &Track, pads: &[Pad], max_spacing: Option<i32>) -> i32 {
    let total: i32 = pads.iter().map(|p| pad_width(t, p.size)).sum();
    let mut ideal = (t.width() - total) as f32 / (pads.len() + 1) as f32;
    if let Some(m) = max_spacing {
        ideal = ideal.min(m as f32);
    }
    if t.site_width <= 0 {
        return ideal as i32;
    }
    (ideal / t.site_width as f32).floor() as i32 * t.site_width
}

/// **P4** — place one pad at a site index, sliding it along if something is in the way.
///
/// `conflict` is asked about a candidate box and returns where the overlap is. With `allow_shift`,
/// the pad moves to the site at the **start of that overlap** and is asked again; `index + 1`
/// guarantees the loop advances even when the overlap begins behind the pad.
#[allow(clippy::too_many_arguments)]
pub fn place_one(
    t: &Track,
    index: i32,
    pad: &Pad,
    base: Orient,
    allow_overlap: bool,
    allow_shift: bool,
    conflict: &mut dyn FnMut(&str, Rect, Orient) -> Option<Refusal>,
) -> Result<Placement, Refused> {
    let mut index = index;
    loop {
        let (x, y, orient) = place_in_row(index, &t.row, t.edge, pad.size.0, pad.size.1, base);
        let (dx, dy) = oriented_size(pad.size.0, pad.size.1, orient);
        let bbox = (x, y, x + dx, y + dy);

        if !allow_overlap {
            let (lo, hi) = t.along(bbox);
            if lo < t.start() || hi > t.end() {
                return Err(Refused::OutOfRow { name: pad.name.clone(), at: bbox });
            }
        }

        match conflict(&pad.name, bbox, orient) {
            None => {
                return Ok(Placement {
                    name: pad.name.clone(),
                    master: pad.master.clone(),
                    x,
                    y,
                    orient,
                })
            }
            Some(why) => {
                if !allow_shift {
                    if allow_overlap {
                        return Ok(Placement {
                            name: pad.name.clone(),
                            master: pad.master.clone(),
                            x,
                            y,
                            orient,
                        });
                    }
                    return Err(Refused::Blocked { name: pad.name.clone(), at: bbox, why });
                }
                let obstacle = t.along(why.overlap).0;
                index = (index + 1).max(t.snap_to_site(obstacle));
            }
        }
    }
}

/// **P5** — spread every pad along the row, in the order given.
///
/// ⚠️ The running cursor advances by the pad's width and the *target* spacing, **not** by where
/// the pad actually landed. A pad that had to slide does not push its neighbours along. This looks
/// like an oversight and is not: it keeps one obstruction from dragging the whole row out of
/// position, and it is what the reference does.
/// `settled` is called as each pad lands, before the next is tried — a pad already placed is an
/// obstruction to the pads after it, and the caller is what holds that list.
pub fn place_uniform(
    t: &Track,
    pads: &[Pad],
    max_spacing: Option<i32>,
    conflict: &mut dyn FnMut(&str, Rect, Orient) -> Option<Refusal>,
    settled: &mut dyn FnMut(&Placement),
) -> Result<Vec<Placement>, Refused> {
    let spacing = target_spacing(t, pads, max_spacing);
    let mut cursor = t.start() + spacing;
    let mut out = Vec::with_capacity(pads.len());
    for pad in pads {
        let p = place_one(t, t.snap_to_site(cursor), pad, Orient::R0, false, true, conflict)?;
        settled(&p);
        out.push(p);
        cursor += pad_width(t, pad.size) + spacing;
    }
    Ok(out)
}

/// **P6** — do these pads even fit?
///
/// Checked before anything is placed, so an impossible request fails as a request rather than as
/// a pad that will not go in somewhere near the end of the row.
pub fn fits(t: &Track, pads: &[Pad]) -> Result<(), (i32, i32)> {
    let total: i32 = pads.iter().map(|p| pad_width(t, p.size)).sum();
    if total > t.width() {
        return Err((total, t.width()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn track(edge: Edge, bbox: Rect, spacing: i32, sites: i32) -> Track {
        Track {
            row: RowGeom {
                name: "IO_TEST".into(),
                bbox,
                orient: Orient::R0,
                origin: (bbox.0, bbox.1),
                spacing,
                site_count: sites,
            },
            edge,
            site_width: spacing,
        }
    }

    fn pad(name: &str, w: i32, h: i32) -> Pad {
        Pad { name: name.into(), master: "PAD".into(), size: (w, h) }
    }

    fn clear(_: &str, _: Rect, _: Orient) -> Option<Refusal> {
        None
    }

    #[test]
    fn a_row_measures_along_its_own_length() {
        let h = track(Edge::South, (0, 0, 1000, 60), 10, 100);
        assert_eq!((h.start(), h.end(), h.width()), (0, 1000, 1000));
        let v = track(Edge::West, (0, 0, 60, 1000), 10, 100);
        assert_eq!((v.start(), v.end(), v.width()), (0, 1000, 1000));
    }

    #[test]
    fn snapping_rounds_to_the_nearest_site_and_stays_in_the_row() {
        let t = track(Edge::South, (0, 0, 1000, 60), 10, 100);
        assert_eq!(t.snap_to_site(0), 0);
        assert_eq!(t.snap_to_site(14), 1, "rounds down");
        assert_eq!(t.snap_to_site(15), 2, "half rounds away from zero");
        assert_eq!(t.snap_to_site(16), 2, "rounds up");
        // ⚠️ Clamped, not wrapped or negative.
        assert_eq!(t.snap_to_site(-500), 0);
        assert_eq!(t.snap_to_site(99_999), 100);
    }

    #[test]
    fn spacing_is_the_leftover_split_into_one_more_gap_than_pads() {
        let t = track(Edge::South, (0, 0, 1000, 60), 10, 100);
        let pads = vec![pad("a", 100, 60), pad("b", 100, 60), pad("c", 100, 60)];
        // 1000 - 300 = 700 over 4 gaps = 175, floored to a whole 10-unit site = 170.
        assert_eq!(target_spacing(&t, &pads, None), 170);
        // A cap applies before the site rounding.
        assert_eq!(target_spacing(&t, &pads, Some(45)), 40);
        // The cap only ever lowers it.
        assert_eq!(target_spacing(&t, &pads, Some(10_000)), 170);
    }

    #[test]
    fn pads_are_spread_with_a_gap_before_the_first_and_after_the_last() {
        let t = track(Edge::South, (0, 0, 1000, 60), 10, 100);
        let pads = vec![pad("a", 100, 60), pad("b", 100, 60), pad("c", 100, 60)];
        let out = place_uniform(&t, &pads, None, &mut clear, &mut |_| {}).unwrap();
        assert_eq!(out.iter().map(|p| p.x).collect::<Vec<_>>(), vec![170, 440, 710]);
        // A gap after the last one too: 710 + 100 = 810, leaving 190 of the 1000.
        assert!(out.last().unwrap().x + 100 < t.end());
    }

    #[test]
    fn a_pad_slides_clear_of_an_obstruction() {
        let t = track(Edge::South, (0, 0, 1000, 60), 10, 100);
        let pads = vec![pad("a", 100, 60)];
        // Something occupies 200..300. The lone pad would ideally sit at 450.
        let mut hit = |_: &str, b: Rect, _: Orient| {
            let block = (200, 0, 300, 60);
            (b.0 < block.2 && block.0 < b.2)
                .then(|| Refusal {
                    reason: crate::clearance::Reason::Blockage,
                    overlap: (b.0.max(block.0), 0, b.2.min(block.2), 60),
                })
        };
        let out = place_uniform(&t, &pads, Some(150), &mut hit, &mut |_| {}).unwrap();
        assert!(out[0].x >= 300, "slid past the obstruction, got {}", out[0].x);
    }

    #[test]
    fn a_shift_does_not_drag_the_following_pads_along() {
        // ⚠️ The behaviour that reads like a bug. The cursor is ideal, not actual.
        let t = track(Edge::South, (0, 0, 1000, 60), 10, 100);
        let pads = vec![pad("a", 100, 60), pad("b", 100, 60)];
        let ideal = place_uniform(&t, &pads, None, &mut clear, &mut |_| {}).unwrap();
        let mut hit = |_: &str, b: Rect, _: Orient| {
            let block = (250, 0, 400, 60);
            (b.0 < block.2 && block.0 < b.2)
                .then(|| Refusal {
                    reason: crate::clearance::Reason::Blockage,
                    overlap: (b.0.max(block.0), 0, b.2.min(block.2), 60),
                })
        };
        let shifted = place_uniform(&t, &pads, None, &mut hit, &mut |_| {}).unwrap();
        assert_ne!(shifted[0].x, ideal[0].x, "the first pad had to move");
        assert_eq!(shifted[1].x, ideal[1].x, "the second one did not");
    }

    #[test]
    fn running_off_the_end_of_the_row_is_refused_by_name() {
        let t = track(Edge::South, (0, 0, 300, 60), 10, 30);
        let pads = vec![pad("wide", 400, 60)];
        match place_uniform(&t, &pads, None, &mut clear, &mut |_| {}) {
            Err(Refused::OutOfRow { name, .. }) => assert_eq!(name, "wide"),
            other => panic!("expected OutOfRow, got {other:?}"),
        }
    }

    #[test]
    fn a_request_that_cannot_fit_is_rejected_before_anything_moves() {
        let t = track(Edge::South, (0, 0, 300, 60), 10, 30);
        let pads = vec![pad("a", 200, 60), pad("b", 200, 60)];
        assert_eq!(fits(&t, &pads), Err((400, 300)));
        assert_eq!(fits(&t, &pads[..1]), Ok(()));
    }

    #[test]
    fn a_side_row_measures_the_pad_the_other_way_round() {
        // A row turned on its side swaps the master's width and height.
        let mut t = track(Edge::West, (0, 0, 60, 1000), 10, 100);
        t.row.orient = Orient::R0;
        assert_eq!(pad_width(&t, (100, 60)), 60, "vertical row consumes the height");
        let h = track(Edge::South, (0, 0, 1000, 60), 10, 100);
        assert_eq!(pad_width(&h, (100, 60)), 100, "horizontal row consumes the width");
    }
}


// ── Bump-aligned placement ───────────────────────────────────────────────────────────────────

/// A bump a pad connects to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bump {
    pub terminal: String,
    /// Centre of the bump terminal's bounding box.
    pub centre: (i32, i32),
    pub id: u64,
}

/// A pad awaiting bump-aligned placement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BumpPad {
    pub name: String,
    pub id: u64,
    /// Size along the row.
    pub width: i32,
    /// Every bump this pad shares a non-supply net with.
    pub bumps: Vec<Bump>,
}

/// **BA1** — how far a pad sitting at `at` would be from a bump.
///
/// Measured from the pad's **centre** — `at + width / 2` — across to the bump, with the other axis
/// pinned to the row's centre line. ⚠️ Squared, and squared is enough: it is only ever compared.
pub fn pad_bump_distance(
    at: i32,
    width: i32,
    bump: (i32, i32),
    row_centre: (i32, i32),
    horizontal: bool,
) -> i64 {
    let p = if horizontal {
        (at + width / 2, row_centre.1)
    } else {
        (row_centre.0, at + width / 2)
    };
    let (dx, dy) = ((p.0 - bump.0) as i64, (p.1 - bump.1) as i64);
    dx * dx + dy * dy
}

/// **BA2** — the run of pads that share a bump column.
///
/// Walking forward from `start`: each pad takes its **nearest** bump to the current offset, and the
/// run continues only while that bump lies in the same column (or row) as the first pad's.
///
/// ⚠️ A pad with no bump at all **ends the run** rather than being skipped. The run is a contiguous
/// block that will be centred on one bump column, and a pad belonging to no column cannot be in it.
pub fn alignment_group(
    pads: &[BumpPad],
    start: usize,
    offset: i32,
    row_centre: (i32, i32),
    horizontal: bool,
) -> Vec<(usize, Bump)> {
    let along = |c: (i32, i32)| if horizontal { c.0 } else { c.1 };
    let mut out: Vec<(usize, Bump)> = Vec::new();
    for (k, pad) in pads.iter().enumerate().skip(start) {
        if pad.bumps.is_empty() {
            break;
        }
        let best = pad
            .bumps
            .iter()
            .min_by_key(|b| {
                (pad_bump_distance(offset, pad.width, b.centre, row_centre, horizontal), b.id)
            })
            .cloned()
            .expect("checked non-empty");
        if let Some((_, first)) = out.first() {
            if along(first.centre) != along(best.centre) {
                break;
            }
        }
        out.push((k, best));
    }
    out
}

/// **BA3** — where each pad of a group goes, before the travel budget is applied.
///
/// The group is centred on its bump column: it starts half its own width before the column and
/// each pad follows the one before.
///
/// ⚠️ Positions are handed out in **instance id order**, not row order. The reference stores the
/// group in a map keyed by instance and walks it, so a group whose pads were listed in another
/// order comes out laid differently — a property of the reference, not of the geometry.
pub fn group_positions(
    pads: &[BumpPad],
    group: &[(usize, Bump)],
    horizontal: bool,
) -> Vec<(usize, i32)> {
    let along = |c: (i32, i32)| if horizontal { c.0 } else { c.1 };
    let Some((_, first)) = group.first() else { return Vec::new() };
    let total: i32 = group.iter().map(|&(k, _)| pads[k].width).sum();
    let mut by_id: Vec<usize> = group.iter().map(|&(k, _)| k).collect();
    by_id.sort_by_key(|&k| pads[k].id);
    let mut at = along(first.centre) - total / 2;
    by_id
        .into_iter()
        .map(|k| {
            let here = (k, at);
            at += pads[k].width;
            here
        })
        .collect()
}

/// **BA4** — how far along the row a pad may actually go.
///
/// A pad never moves **backwards**, and the row may only move forward by the slack it has. Each
/// pad's move spends from that budget.
///
/// ⚠️ Returns the position **and** the budget left. Not spending it lets every pad take the full
/// slack and the row runs off its own end.
pub fn travel(offset: i32, want: i32, budget: i32) -> (i32, i32) {
    let take = budget.min((want - offset).max(0));
    (offset + take, budget - take)
}

/// **BA5** — is flipping this pad worth it?
///
/// ⚠️ Only when the pad has **more than one** bump connection: with one there is nothing to trade
/// off. The flip is kept unless it makes the total longer, so an exact tie **keeps** it.
pub fn keep_flip(straight: i64, flipped: i64, connections: usize) -> bool {
    connections > 1 && flipped <= straight
}

#[cfg(test)]
mod bump_aligned_tests {
    use super::*;

    fn bump(name: &str, c: (i32, i32), id: u64) -> Bump {
        Bump { terminal: name.into(), centre: c, id }
    }

    fn pad(name: &str, id: u64, width: i32, bumps: Vec<Bump>) -> BumpPad {
        BumpPad { name: name.into(), id, width, bumps }
    }

    #[test]
    fn a_pad_takes_its_nearest_bump() {
        let pads =
            vec![pad("a", 1, 100, vec![bump("far", (900, 500), 1), bump("near", (100, 500), 2)])];
        assert_eq!(alignment_group(&pads, 0, 0, (0, 0), true)[0].1.terminal, "near");
    }

    #[test]
    fn a_run_stops_at_a_different_bump_column() {
        let pads = vec![
            pad("a", 1, 100, vec![bump("x", (100, 500), 1)]),
            pad("b", 2, 100, vec![bump("x2", (100, 900), 2)]),
            pad("c", 3, 100, vec![bump("y", (700, 500), 3)]),
        ];
        assert_eq!(alignment_group(&pads, 0, 0, (0, 0), true).len(), 2);
    }

    #[test]
    fn a_pad_with_no_bump_ends_the_run() {
        // ⚠️ Ends it, rather than being skipped over.
        let pads = vec![
            pad("a", 1, 100, vec![bump("x", (100, 500), 1)]),
            pad("b", 2, 100, vec![]),
            pad("c", 3, 100, vec![bump("x", (100, 500), 2)]),
        ];
        assert_eq!(alignment_group(&pads, 0, 0, (0, 0), true).len(), 1);
        assert!(alignment_group(&pads, 1, 0, (0, 0), true).is_empty(), "and cannot start one");
    }

    #[test]
    fn a_group_is_centred_on_its_bump_column() {
        let pads = vec![
            pad("a", 1, 100, vec![bump("x", (1000, 500), 1)]),
            pad("b", 2, 100, vec![bump("x", (1000, 500), 2)]),
        ];
        let g = alignment_group(&pads, 0, 0, (0, 0), true);
        // The group spans 900..1100, centred on the column; each pad follows the one before.
        assert_eq!(group_positions(&pads, &g, true), vec![(0, 900), (1, 1000)]);
    }

    #[test]
    fn positions_within_a_group_follow_instance_id_order() {
        // ⚠️ Not row order. The reference walks a map keyed by instance.
        let pads = vec![
            pad("a", 9, 100, vec![bump("x", (1000, 500), 1)]),
            pad("b", 2, 100, vec![bump("x", (1000, 500), 2)]),
        ];
        let g = alignment_group(&pads, 0, 0, (0, 0), true);
        assert_eq!(group_positions(&pads, &g, true), vec![(1, 900), (0, 1000)], "lower id first");
    }

    #[test]
    fn a_pad_never_moves_backwards_and_the_budget_is_spent() {
        assert_eq!(travel(500, 900, 1000), (900, 600), "moved 400, 600 left");
        assert_eq!(travel(500, 300, 1000), (500, 1000), "cannot go back, nothing spent");
        assert_eq!(travel(500, 5000, 200), (700, 0), "capped by the budget");
    }

    #[test]
    fn a_flip_is_kept_unless_it_costs_more() {
        assert!(keep_flip(100, 90, 2), "shorter, keep");
        assert!(keep_flip(100, 100, 2), "⚠️ an exact tie keeps the flip");
        assert!(!keep_flip(100, 110, 2), "longer, undo");
        assert!(!keep_flip(100, 10, 1), "⚠️ one connection is never flipped");
    }
}
