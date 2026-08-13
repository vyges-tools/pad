// SPDX-License-Identifier: Apache-2.0
//! The pad ring — four corners and four edges of IO sites around the die.
//!
//! This is the foundation of the engine. `place_pad`, `place_corners`, `place_io_fill` and
//! `place_io_terminals` all place *into* these rows, so a ring whose geometry differs from the
//! reference's makes every later comparison meaningless — the same argument that put slot
//! generation first in the pin placer.
//!
//! Nothing here touches a database.

use crate::orient::Orient;

/// A site: the unit an IO row is tiled with.
///
/// ⚠️ A pad site is **not square and not upright** — it is long in the direction the row runs and
/// thin across it. The row's pitch is therefore the site's *smaller* dimension and its thickness
/// the *larger*, which is why the code below reads `min` and `max` of the two rather than `width`
/// and `height`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Site {
    pub name: String,
    pub width: i32,
    pub height: i32,
}

impl Site {
    /// The short side — the pitch a row of these advances by.
    pub fn pitch(&self) -> i32 {
        self.width.min(self.height)
    }
    /// The long side — how far the row reaches in from the die edge.
    pub fn depth(&self) -> i32 {
        self.width.max(self.height)
    }
}

/// How far each edge of the ring is inset from the die.
///
/// Four independent values, not one margin: a design can give one edge more room than another.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Offsets {
    pub west: i32,
    pub north: i32,
    pub east: i32,
    pub south: i32,
}

/// Which way a row's sites are laid out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RowDir {
    Horizontal,
    Vertical,
}

/// One row of the ring.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Row {
    pub name: String,
    pub site: String,
    pub x: i32,
    pub y: i32,
    pub orient: Orient,
    pub dir: RowDir,
    /// How many sites the row holds.
    pub sites: i32,
    /// The distance between them.
    pub pitch: i32,
}

/// The user's rotations, applied on top of each row's own orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rotations {
    pub horizontal: Orient,
    pub vertical: Orient,
    pub corner: Orient,
}

impl Default for Rotations {
    fn default() -> Self {
        Rotations { horizontal: Orient::R0, vertical: Orient::R0, corner: Orient::R0 }
    }
}

/// Row names, before any ring suffix.
const CORNER_NW: &str = "IO_CORNER_NORTH_WEST";
const CORNER_NE: &str = "IO_CORNER_NORTH_EAST";
const CORNER_SE: &str = "IO_CORNER_SOUTH_EAST";
const CORNER_SW: &str = "IO_CORNER_SOUTH_WEST";
const ROW_NORTH: &str = "IO_NORTH";
const ROW_EAST: &str = "IO_EAST";
const ROW_SOUTH: &str = "IO_SOUTH";
const ROW_WEST: &str = "IO_WEST";

/// **R0** — a row's name, with the ring index appended only when there is one.
///
/// A negative index means "the only ring", and then the name is bare. Appending `_-1` would be a
/// different row as far as every later command is concerned.
pub fn row_name(base: &str, ring_index: i32) -> String {
    if ring_index < 0 {
        base.to_string()
    } else {
        format!("{base}_{ring_index}")
    }
}

/// **R1–R4** — the whole ring.
///
/// `die` is `(x0, y0, x1, y1)`. `same_site` says whether the caller passed the *same* site object
/// for the horizontal and vertical directions — see [`Row`] and **R4**; it is deliberately not
/// inferred from the site's name or size.
pub fn make_rows(
    die: (i32, i32, i32, i32),
    horizontal: &Site,
    vertical: &Site,
    corner: &Site,
    offsets: Offsets,
    rotations: Rotations,
    ring_index: i32,
    same_site: bool,
) -> Vec<Row> {
    let (dx0, dy0, dx1, dy1) = die;

    // **R1** — the ring is the die inset by four independent offsets.
    let (mut x0, mut y0) = (dx0 + offsets.west, dy0 + offsets.south);
    let (mut x1, mut y1) = (dx1 - offsets.east, dy1 - offsets.north);

    // **R2** — the corner cell sizes the ring, and its WIDTH is not its own width: the horizontal
    // row's depth can be what sets it. A corner narrower than the row it abuts would leave a gap.
    let corner_height = corner.height;
    let corner_width = horizontal.depth().max(corner.width);

    // **R3** — the ring is truncated to WHOLE sites. What does not divide evenly is given up, not
    // rounded out: a partial site at the end of a row is not a site.
    let x_sites = (x1 - x0 - 2 * corner_width).div_euclid(vertical.pitch().max(1));
    x1 = x0 + 2 * corner_width + x_sites * vertical.pitch();
    let y_sites = (y1 - y0 - 2 * corner_height).div_euclid(horizontal.pitch().max(1));
    y1 = y0 + 2 * corner_height + y_sites * horizontal.pitch();

    // The corners sit at the four extremes of the truncated ring, each placed by its lower-left.
    let (cx1, cy1) = (x1 - corner_width, y1 - corner_height);
    let corner_sites = corner_width.div_euclid(corner.width.max(1));

    let corner_row = |base: &str, x: i32, y: i32, own: Orient| Row {
        name: row_name(base, ring_index),
        site: corner.name.clone(),
        x,
        y,
        // The user's corner rotation is applied FIRST, then the corner's own orientation.
        orient: rotations.corner.concat(own),
        dir: RowDir::Horizontal,
        sites: corner_sites,
        pitch: corner.width,
    };

    // **R4** — the west edge's orientation depends on whether the two directions were given the
    // SAME site. ⚠️ Not the same size, and not the same name: the reference compares the objects.
    // Same site → the row is laid on its side (`MXR90`); different sites → it is upright.
    let west_own = if same_site { Orient::MXR90 } else { Orient::R0 };
    let east_own = west_own.flip_y();
    let south_own = Orient::R0;
    let north_own = south_own.flip_x();

    let nw = corner_row(CORNER_NW, x0, cy1, Orient::MX);
    let ne = corner_row(CORNER_NE, cx1, cy1, Orient::R180);
    let se = corner_row(CORNER_SE, cx1, y0, Orient::MY);
    let sw = corner_row(CORNER_SW, x0, y0, Orient::R0);

    // Each edge starts where its corner ends, so the ring closes with no gap and no overlap.
    let corner_span_x = corner_sites * corner.width;
    let (nw_end_x, sw_end_x) = (nw.x + corner_span_x, sw.x + corner_span_x);
    let sw_end_y = sw.y + corner_height;
    let se_end_y = se.y + corner_height;

    let edge = |base: &str, site: &Site, sites: i32, x: i32, y: i32, own: Orient, dir: RowDir| Row {
        name: row_name(base, ring_index),
        site: site.name.clone(),
        x,
        y,
        orient: match dir {
            RowDir::Horizontal => rotations.vertical.concat(own),
            RowDir::Vertical => rotations.horizontal.concat(own),
        },
        dir,
        sites,
        pitch: site.pitch(),
    };

    vec![
        nw.clone(),
        ne,
        se.clone(),
        sw.clone(),
        // ⚠️ The north and east rows are placed by their INNER edge — the row's depth is
        // subtracted, because a row hangs inward from the ring's outer bound.
        edge(ROW_NORTH, vertical, x_sites, nw_end_x, y1 - vertical.depth(), north_own,
             RowDir::Horizontal),
        edge(ROW_EAST, horizontal, y_sites, x1 - horizontal.depth(), se_end_y, east_own,
             RowDir::Vertical),
        edge(ROW_SOUTH, vertical, x_sites, sw_end_x, y0, south_own, RowDir::Horizontal),
        edge(ROW_WEST, horizontal, y_sites, x0, sw_end_y, west_own, RowDir::Vertical),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The site every `IOSITE` case uses: thin and deep.
    fn iosite() -> Site {
        Site { name: "IOSITE".into(), width: 2_000, height: 280_000 }
    }

    /// A die big enough for the reference cases, in DBU at 2000 per micron.
    const DIE: (i32, i32, i32, i32) = (0, 0, 6_000_000, 6_000_000);

    fn find<'a>(rows: &'a [Row], name: &str) -> &'a Row {
        rows.iter().find(|r| r.name == name).unwrap_or_else(|| panic!("no row {name}"))
    }

    /// Every field of a row, in the order a DEF golden prints them.
    fn as_def(r: &Row) -> String {
        let (dx, dy) = match r.dir {
            RowDir::Horizontal => (r.sites, 1),
            RowDir::Vertical => (1, r.sites),
        };
        let (sx, sy) = match r.dir {
            RowDir::Horizontal => (r.pitch, 0),
            RowDir::Vertical => (0, r.pitch),
        };
        format!(
            "ROW {} {} {} {} {} DO {} BY {} STEP {} {} ;",
            r.name,
            r.site,
            r.x,
            r.y,
            r.orient.def(),
            dx,
            dy,
            sx,
            sy
        )
    }

    #[test]
    fn a_site_is_measured_by_its_short_and_long_sides_not_width_and_height() {
        // The distinction the whole module turns on: a pad site is thin across the row and deep
        // into the die, and which of width/height is which depends on the edge.
        let s = iosite();
        assert_eq!(s.pitch(), 2_000);
        assert_eq!(s.depth(), 280_000);
        let sideways = Site { name: "x".into(), width: 280_000, height: 2_000 };
        assert_eq!(sideways.pitch(), 2_000, "orientation of the site does not change its pitch");
        assert_eq!(sideways.depth(), 280_000);
    }

    #[test]
    fn a_ring_index_of_minus_one_leaves_the_name_bare() {
        assert_eq!(row_name("IO_NORTH", -1), "IO_NORTH");
        assert_eq!(row_name("IO_NORTH", 0), "IO_NORTH_0");
        assert_eq!(row_name("IO_NORTH", 3), "IO_NORTH_3");
    }

    #[test]
    fn the_ring_reproduces_the_reference_golden_exactly() {
        // `make_io_sites.defok`, verbatim: one site for all three directions, 15um offset.
        let s = iosite();
        let rows = make_rows(
            DIE,
            &s,
            &s,
            &s,
            Offsets { west: 30_000, north: 30_000, east: 30_000, south: 30_000 },
            Rotations::default(),
            -1,
            true,
        );
        let want = [
            "ROW IO_CORNER_NORTH_WEST IOSITE 30000 5690000 FS DO 140 BY 1 STEP 2000 0 ;",
            "ROW IO_CORNER_NORTH_EAST IOSITE 5690000 5690000 S DO 140 BY 1 STEP 2000 0 ;",
            "ROW IO_CORNER_SOUTH_EAST IOSITE 5690000 30000 FN DO 140 BY 1 STEP 2000 0 ;",
            "ROW IO_CORNER_SOUTH_WEST IOSITE 30000 30000 N DO 140 BY 1 STEP 2000 0 ;",
            "ROW IO_NORTH IOSITE 310000 5690000 FS DO 2690 BY 1 STEP 2000 0 ;",
            "ROW IO_EAST IOSITE 5690000 310000 W DO 1 BY 2690 STEP 0 2000 ;",
            "ROW IO_SOUTH IOSITE 310000 30000 N DO 2690 BY 1 STEP 2000 0 ;",
            "ROW IO_WEST IOSITE 30000 310000 FW DO 1 BY 2690 STEP 0 2000 ;",
        ];
        let got: Vec<String> = rows.iter().map(as_def).collect();
        assert_eq!(got, want);
    }

    #[test]
    fn giving_the_two_directions_different_sites_turns_the_side_rows_upright() {
        // 🔑 `make_io_sites_different_sites.defok`. The west row is FW when both directions share a
        // site and N when they do not — and the east row follows it by a flip. Nothing about the
        // geometry says which; only whether the caller passed the same site.
        let hor = Site { name: "HORIZONTAL".into(), width: 100_000, height: 2_000 };
        let ver = Site { name: "VERTICAL".into(), width: 2_000, height: 100_000 };
        let cor = Site { name: "CORNER".into(), width: 100_000, height: 100_000 };
        let rows = make_rows(
            DIE,
            &hor,
            &ver,
            &cor,
            Offsets { west: 30_000, north: 30_000, east: 30_000, south: 30_000 },
            Rotations::default(),
            -1,
            false,
        );
        let want = [
            "ROW IO_CORNER_NORTH_WEST CORNER 30000 5870000 FS DO 1 BY 1 STEP 100000 0 ;",
            "ROW IO_CORNER_NORTH_EAST CORNER 5870000 5870000 S DO 1 BY 1 STEP 100000 0 ;",
            "ROW IO_CORNER_SOUTH_EAST CORNER 5870000 30000 FN DO 1 BY 1 STEP 100000 0 ;",
            "ROW IO_CORNER_SOUTH_WEST CORNER 30000 30000 N DO 1 BY 1 STEP 100000 0 ;",
            "ROW IO_NORTH VERTICAL 130000 5870000 FS DO 2870 BY 1 STEP 2000 0 ;",
            "ROW IO_EAST HORIZONTAL 5870000 130000 FN DO 1 BY 2870 STEP 0 2000 ;",
            "ROW IO_SOUTH VERTICAL 130000 30000 N DO 2870 BY 1 STEP 2000 0 ;",
            "ROW IO_WEST HORIZONTAL 30000 130000 N DO 1 BY 2870 STEP 0 2000 ;",
        ];
        let got: Vec<String> = rows.iter().map(as_def).collect();
        assert_eq!(got, want);
    }

    #[test]
    fn the_user_rotations_compose_on_top_of_each_rows_own_orientation() {
        // `make_io_sites_rotations.defok`: -rotation_horizontal MXR90, -rotation_vertical MY, with
        // different sites for the two directions.
        let s = iosite();
        let s2 = Site { name: "IOSITE2".into(), ..iosite() };
        let rows = make_rows(
            DIE,
            &s,
            &s2,
            &s,
            Offsets { west: 30_000, north: 30_000, east: 30_000, south: 30_000 },
            Rotations {
                horizontal: Orient::MXR90,
                vertical: Orient::MY,
                corner: Orient::R0,
            },
            -1,
            false,
        );
        assert_eq!(find(&rows, "IO_NORTH").orient.def(), "S", "MY over the north edge's MX");
        assert_eq!(find(&rows, "IO_SOUTH").orient.def(), "FN", "MY over the south edge's R0");
        assert_eq!(find(&rows, "IO_EAST").orient.def(), "W", "MXR90 over the flipped west");
        assert_eq!(find(&rows, "IO_WEST").orient.def(), "FW");
        // The corners take no rotation here and keep their own.
        assert_eq!(find(&rows, "IO_CORNER_NORTH_WEST").orient.def(), "FS");
    }

    #[test]
    fn the_ring_closes_with_no_gap_between_a_corner_and_the_edge_it_abuts() {
        // The property that makes the ring a ring. Every edge starts exactly where its corner ends.
        let s = iosite();
        let rows = make_rows(
            DIE,
            &s,
            &s,
            &s,
            Offsets { west: 30_000, north: 30_000, east: 30_000, south: 30_000 },
            Rotations::default(),
            -1,
            true,
        );
        let corner_span = find(&rows, "IO_CORNER_SOUTH_WEST").sites * s.width;
        assert_eq!(
            find(&rows, "IO_SOUTH").x,
            find(&rows, "IO_CORNER_SOUTH_WEST").x + corner_span,
            "the south row starts where the south-west corner ends"
        );
        assert_eq!(
            find(&rows, "IO_NORTH").x,
            find(&rows, "IO_CORNER_NORTH_WEST").x + corner_span,
        );
        assert_eq!(
            find(&rows, "IO_WEST").y,
            find(&rows, "IO_CORNER_SOUTH_WEST").y + s.height,
        );
    }

    #[test]
    fn a_ring_that_does_not_divide_evenly_gives_up_the_remainder() {
        // R3. A partial site at the end of a row is not a site, so the ring shrinks to fit rather
        // than rounding outward past the offsets it was given.
        let s = iosite();
        let odd = (0, 0, 6_000_001, 6_000_001);
        let rows = make_rows(
            odd,
            &s,
            &s,
            &s,
            Offsets { west: 30_000, north: 30_000, east: 30_000, south: 30_000 },
            Rotations::default(),
            -1,
            true,
        );
        let north = find(&rows, "IO_NORTH");
        let span = north.x + north.sites * north.pitch;
        assert!(span <= odd.2 - 30_000, "the ring reached past its own offset");
        // And it is the same count as the even die: one extra DBU buys nothing.
        assert_eq!(north.sites, 2_690);
    }

    #[test]
    fn each_edge_is_offered_the_site_for_its_own_direction() {
        let hor = Site { name: "H".into(), width: 100_000, height: 2_000 };
        let ver = Site { name: "V".into(), width: 2_000, height: 100_000 };
        let cor = Site { name: "C".into(), width: 100_000, height: 100_000 };
        let rows =
            make_rows(DIE, &hor, &ver, &cor, Offsets::default(), Rotations::default(), -1, false);
        // North and south run horizontally and are tiled with the VERTICAL-pin site; east and west
        // run vertically and take the horizontal one. The naming is the reference's, and it reads
        // backwards until you think of the pin direction rather than the row direction.
        assert_eq!(find(&rows, "IO_NORTH").site, "V");
        assert_eq!(find(&rows, "IO_SOUTH").site, "V");
        assert_eq!(find(&rows, "IO_EAST").site, "H");
        assert_eq!(find(&rows, "IO_WEST").site, "H");
        assert_eq!(find(&rows, "IO_CORNER_NORTH_WEST").site, "C");
    }

    #[test]
    fn asymmetric_offsets_inset_each_edge_independently() {
        let s = iosite();
        let rows = make_rows(
            DIE,
            &s,
            &s,
            &s,
            Offsets { west: 10_000, north: 20_000, east: 30_000, south: 40_000 },
            Rotations::default(),
            -1,
            true,
        );
        assert_eq!(find(&rows, "IO_CORNER_SOUTH_WEST").x, 10_000, "west");
        assert_eq!(find(&rows, "IO_CORNER_SOUTH_WEST").y, 40_000, "south");
        assert_eq!(find(&rows, "IO_SOUTH").y, 40_000);
        assert_eq!(find(&rows, "IO_WEST").x, 10_000);
    }
}
