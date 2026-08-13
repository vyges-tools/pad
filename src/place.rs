// SPDX-License-Identifier: Apache-2.0
//! Placing cells into the ring — corners, and pads at a given location.
//!
//! The ring says where cells *may* go; this says where they *do*. Two commands share almost all of
//! it: a corner is a cell dropped at a row's own origin, and a pad is a cell dropped at a site
//! index along one.
//!
//! Nothing here touches a database.

use crate::orient::Orient;

/// Which side of the die a row runs along.
///
/// Not stored on the row — **derived from its name**, which is what the reference does. A row named
/// `IO_NORTH_2` is a north row; the ring suffix is part of the name and not part of the question.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edge {
    North,
    South,
    East,
    West,
}

impl Edge {
    /// The edge a row name denotes, or `None` if it is not an IO row.
    ///
    /// ⚠️ Matched as a **prefix**, so `IO_NORTH_0` resolves. And `IO_CORNER_NORTH_WEST` is
    /// deliberately not a north row — it does not start with `IO_NORTH`.
    pub fn of_row(name: &str) -> Option<Edge> {
        for (prefix, edge) in [
            ("IO_NORTH", Edge::North),
            ("IO_SOUTH", Edge::South),
            ("IO_EAST", Edge::East),
            ("IO_WEST", Edge::West),
        ] {
            if name.starts_with(prefix) {
                return Some(edge);
            }
        }
        None
    }

    /// Does this edge's row run left-to-right?
    pub fn is_horizontal(self) -> bool {
        matches!(self, Edge::North | Edge::South)
    }
}

/// The row a cell is being placed into, as this module needs it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RowGeom {
    pub name: String,
    /// The row's bounding box, `(x0, y0, x1, y1)`.
    pub bbox: (i32, i32, i32, i32),
    pub orient: Orient,
    pub origin: (i32, i32),
    pub spacing: i32,
    pub site_count: i32,
}

impl RowGeom {
    pub fn edge(&self) -> Option<Edge> {
        Edge::of_row(&self.name)
    }
}

/// Where a cell ended up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Placement {
    pub name: String,
    pub master: String,
    pub x: i32,
    pub y: i32,
    pub orient: Orient,
}

/// **C1** — the size of a master once oriented.
///
/// A quarter-turn swaps width and height. Everything that positions a cell against the far side of
/// a row needs the **oriented** size, not the master's own — using the master's puts every north
/// and east cell in the wrong place by the difference.
pub fn oriented_size(width: i32, height: i32, orient: Orient) -> (i32, i32) {
    match orient {
        Orient::R90 | Orient::R270 | Orient::MYR90 | Orient::MXR90 => (height, width),
        _ => (width, height),
    }
}

/// **C2** — the site index nearest a location along the row.
///
/// Rounded, not truncated: a location between two sites belongs to the closer one. Clamped to the
/// row, so a location beyond the end lands on the last site rather than off the die.
pub fn snap_to_site(location: i32, row: &RowGeom, edge: Edge) -> i32 {
    if row.spacing <= 0 {
        return 0;
    }
    let start = if edge.is_horizontal() { row.origin.0 } else { row.origin.1 };
    let relative = (location - start) as f64 / row.spacing as f64;
    (relative.round() as i32).clamp(0, row.site_count)
}

/// **C3** — the base orientation `-mirror` asks for.
///
/// ⚠️ On the **side** rows it depends on the SITE's proportions: a site taller than it is wide is
/// mirrored about X, otherwise about Y. The top and bottom rows always mirror about Y. Getting
/// this wrong still produces a legal orientation, just a cell facing the wrong way.
pub fn mirror_base(edge: Edge, site_width: i32, site_height: i32) -> Orient {
    match edge {
        Edge::North | Edge::South => Orient::MY,
        Edge::East | Edge::West => {
            if site_height < site_width {
                Orient::MX
            } else {
                Orient::MY
            }
        }
    }
}

/// **C4** — place a cell at a site index in a row.
///
/// The cell's orientation is the base composed with the **row's own** — the row is already turned
/// to face outward, and the base is whatever the caller asked on top of that.
///
/// ⚠️ **The north and east rows position by the cell's FAR edge**, subtracting its oriented size,
/// because a row hangs inward from the ring's outer bound. South and west position by the near
/// edge and need no subtraction. Treating all four alike puts half the ring one cell-depth out.
pub fn place_in_row(
    index: i32,
    row: &RowGeom,
    edge: Edge,
    master_width: i32,
    master_height: i32,
    base: Orient,
) -> (i32, i32, Orient) {
    let orient = base.concat(row.orient);
    let (dx, dy) = oriented_size(master_width, master_height, orient);
    let offset = index * row.spacing;
    let (x0, y0, x1, y1) = row.bbox;

    let (x, y) = match edge {
        Edge::North => (x0 + offset, y1 - dy),
        Edge::South => (x0 + offset, y0),
        Edge::West => (x0, y0 + offset),
        Edge::East => (x1 - dx, y0 + offset),
    };
    (x, y, orient)
}

/// **C5** — a corner cell: the row's own origin, the row's own orientation.
///
/// The instance is named after the row it fills, so a second call finds and re-places the same cell
/// rather than making another.
pub fn corner_placement(row: &RowGeom, master: &str) -> Placement {
    let (x0, y0, _, _) = row.bbox;
    Placement {
        name: format!("{}_INST", row.name),
        master: master.to_string(),
        x: x0,
        y: y0,
        orient: row.orient,
    }
}

/// The four corner rows, in the order the reference visits them.
///
/// ⚠️ **Not the order the ring is built in.** The ring emits NW, NE, SE, SW; corners are placed
/// NW, NE, SW, SE. It only shows when two corners contend for the same space and the first one
/// there wins.
pub fn corner_row_names(ring_index: i32) -> [String; 4] {
    [
        crate::ring::row_name("IO_CORNER_NORTH_WEST", ring_index),
        crate::ring::row_name("IO_CORNER_NORTH_EAST", ring_index),
        crate::ring::row_name("IO_CORNER_SOUTH_WEST", ring_index),
        crate::ring::row_name("IO_CORNER_SOUTH_EAST", ring_index),
    ]
}

/// **C6** — would a cell of this size at this spot hit something already there?
///
/// Touching is not overlapping: two cells abutting exactly share an edge and no area, which is how
/// a filled row is supposed to look.
pub fn overlaps(at: (i32, i32), size: (i32, i32), blockers: &[(i32, i32, i32, i32)]) -> bool {
    let (x0, y0) = at;
    let (x1, y1) = (x0 + size.0, y0 + size.1);
    blockers.iter().any(|&(bx0, by0, bx1, by1)| x0 < bx1 && bx0 < x1 && y0 < by1 && by0 < y1)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `IO_EAST` as `make_io_sites -offset 15` builds it on the reference's blackparrot floorplan.
    fn east_row() -> RowGeom {
        RowGeom {
            name: "IO_EAST".into(),
            bbox: (5_690_000, 310_000, 5_970_000, 5_690_000),
            orient: Orient::R90,
            origin: (5_690_000, 310_000),
            spacing: 2_000,
            site_count: 2_690,
        }
    }

    fn south_row() -> RowGeom {
        RowGeom {
            name: "IO_SOUTH".into(),
            bbox: (310_000, 30_000, 5_690_000, 310_000),
            orient: Orient::R0,
            origin: (310_000, 30_000),
            spacing: 2_000,
            site_count: 2_690,
        }
    }

    fn north_row() -> RowGeom {
        RowGeom {
            name: "IO_NORTH".into(),
            bbox: (310_000, 5_690_000, 5_690_000, 5_970_000),
            orient: Orient::MX,
            origin: (310_000, 5_690_000),
            spacing: 2_000,
            site_count: 2_690,
        }
    }

    fn west_row() -> RowGeom {
        RowGeom {
            name: "IO_WEST".into(),
            bbox: (30_000, 310_000, 310_000, 5_690_000),
            orient: Orient::MXR90,
            origin: (30_000, 310_000),
            spacing: 2_000,
            site_count: 2_690,
        }
    }

    /// The pad masters the reference cases use: 80um along the row, 280um deep.
    const PAD_W: i32 = 160_000;
    const PAD_H: i32 = 280_000;

    #[test]
    fn a_rows_edge_comes_from_its_name_and_survives_a_ring_suffix() {
        assert_eq!(Edge::of_row("IO_NORTH"), Some(Edge::North));
        assert_eq!(Edge::of_row("IO_NORTH_2"), Some(Edge::North), "a ring suffix is still north");
        assert_eq!(Edge::of_row("IO_EAST"), Some(Edge::East));
        // A corner row is not a side row, and the prefixes must not collide.
        assert_eq!(Edge::of_row("IO_CORNER_NORTH_WEST"), None);
        assert_eq!(Edge::of_row("CORE_ROW_1"), None);
    }

    #[test]
    fn a_quarter_turn_swaps_the_cells_width_and_height() {
        assert_eq!(oriented_size(100, 300, Orient::R0), (100, 300));
        assert_eq!(oriented_size(100, 300, Orient::MX), (100, 300), "a mirror does not transpose");
        assert_eq!(oriented_size(100, 300, Orient::R90), (300, 100));
        assert_eq!(oriented_size(100, 300, Orient::MXR90), (300, 100));
    }

    #[test]
    fn a_location_snaps_to_the_nearest_site_and_cannot_leave_the_row() {
        let row = south_row();
        // 600um on this row is site 445, and 445 * 2000 + 310000 is exactly 1_200_000.
        assert_eq!(snap_to_site(1_200_000, &row, Edge::South), 445);
        // Rounded, not truncated: just past a site belongs to the next one.
        assert_eq!(snap_to_site(310_000 + 1_100, &row, Edge::South), 1);
        assert_eq!(snap_to_site(310_000 + 900, &row, Edge::South), 0);
        // Clamped at both ends rather than running off the die.
        assert_eq!(snap_to_site(0, &row, Edge::South), 0);
        assert_eq!(snap_to_site(i32::MAX / 2, &row, Edge::South), row.site_count);
    }

    #[test]
    fn a_pad_lands_where_the_reference_puts_it_on_every_edge() {
        // `place_pad.defok`, verbatim, for the four unmirrored pads.
        let cases = [
            (east_row(), Edge::East, 1_000_000, (5_690_000, 1_000_000), "W"),
            (west_row(), Edge::West, 1_200_000, (30_000, 1_200_000), "FW"),
            (north_row(), Edge::North, 1_000_000, (1_000_000, 5_690_000), "FS"),
            (south_row(), Edge::South, 1_200_000, (1_200_000, 30_000), "N"),
        ];
        for (row, edge, location, want_pos, want_orient) in cases {
            let idx = snap_to_site(location, &row, edge);
            let (x, y, o) = place_in_row(idx, &row, edge, PAD_W, PAD_H, Orient::R0);
            assert_eq!((x, y), want_pos, "{} position", row.name);
            assert_eq!(o.def(), want_orient, "{} orientation", row.name);
        }
    }

    #[test]
    fn mirroring_gives_the_orientation_the_reference_records_on_each_edge() {
        // `place_pad.defok`'s four mirrored pads. The site here is taller than it is wide, which
        // is what sends the side rows to MY rather than MX.
        let site = (2_000, 280_000);
        let cases = [
            (east_row(), Edge::East, "FE"),
            (west_row(), Edge::West, "E"),
            (north_row(), Edge::North, "S"),
            (south_row(), Edge::South, "FN"),
        ];
        for (row, edge, want) in cases {
            let base = mirror_base(edge, site.0, site.1);
            let (_, _, o) = place_in_row(0, &row, edge, PAD_W, PAD_H, base);
            assert_eq!(o.def(), want, "{} mirrored", row.name);
        }
    }

    #[test]
    fn a_wide_site_mirrors_the_side_rows_the_other_way() {
        // C3's condition, which the reference cases never exercise in this direction.
        assert_eq!(mirror_base(Edge::East, 280_000, 2_000), Orient::MX, "wider than tall");
        assert_eq!(mirror_base(Edge::East, 2_000, 280_000), Orient::MY, "taller than wide");
        assert_eq!(mirror_base(Edge::North, 280_000, 2_000), Orient::MY, "top and bottom always MY");
    }

    #[test]
    fn the_north_and_east_rows_position_by_the_cells_far_edge() {
        // C4's asymmetry. On the north row the cell hangs DOWN from the top of the row, so its
        // origin is the row's top minus its own depth; on the south row it sits on the bottom.
        let north = north_row();
        let (_, ny, _) = place_in_row(0, &north, Edge::North, PAD_W, PAD_H, Orient::R0);
        assert_eq!(ny, north.bbox.3 - PAD_H, "north subtracts the cell depth");
        assert_eq!(ny, north.bbox.1, "which lands it on the row's own origin here");

        let south = south_row();
        let (_, sy, _) = place_in_row(0, &south, Edge::South, PAD_W, PAD_H, Orient::R0);
        assert_eq!(sy, south.bbox.1, "south does not subtract");
    }

    #[test]
    fn a_corner_takes_its_rows_origin_and_orientation_and_is_named_after_it() {
        // `place_corners_avoid_overlap.defok`.
        let row = RowGeom {
            name: "IO_CORNER_NORTH_WEST".into(),
            bbox: (70_000, 5_650_000, 350_000, 5_930_000),
            orient: Orient::MX,
            origin: (70_000, 5_650_000),
            spacing: 2_000,
            site_count: 140,
        };
        let p = corner_placement(&row, "PAD_CORNER");
        assert_eq!(p.name, "IO_CORNER_NORTH_WEST_INST");
        assert_eq!((p.x, p.y), (70_000, 5_650_000));
        assert_eq!(p.orient.def(), "FS");
    }

    #[test]
    fn corners_are_visited_in_a_different_order_than_the_ring_is_built() {
        // It only matters when two corners contend, but it decides who wins when they do.
        let names = corner_row_names(-1);
        assert_eq!(names[2], "IO_CORNER_SOUTH_WEST", "south-west is third, not fourth");
        assert_eq!(names[3], "IO_CORNER_SOUTH_EAST");
        assert_eq!(corner_row_names(1)[0], "IO_CORNER_NORTH_WEST_1", "the ring suffix carries");
    }

    #[test]
    fn touching_is_not_overlapping() {
        // A filled row is meant to have its cells edge to edge; treating that as a collision would
        // refuse every legal fill.
        let blockers = [(100, 100, 200, 200)];
        assert!(overlaps((150, 150), (10, 10), &blockers), "straight through it");
        assert!(!overlaps((200, 100), (50, 50), &blockers), "abutting on the right");
        assert!(!overlaps((50, 100), (50, 50), &blockers), "abutting on the left");
        assert!(!overlaps((300, 300), (10, 10), &blockers), "well clear");
        assert!(!overlaps((150, 150), (10, 10), &[]), "nothing to hit");
    }

    #[test]
    fn a_corner_that_would_land_on_something_is_detected() {
        // `place_corners_avoid_overlap` puts a cell at (80000, 80000); the south-west corner is
        // 280um square from (70000, 70000) and so cannot go there.
        let sw = (70_000, 70_000);
        let corner = (280_000, 280_000);
        let overlap_cell = [(80_000, 80_000, 80_000 + 2_000, 80_000 + 2_000)];
        assert!(overlaps(sw, corner, &overlap_cell));
        // And the north-east corner, far away, is unaffected.
        assert!(!overlaps((5_650_000, 5_650_000), corner, &overlap_cell));
    }
}
