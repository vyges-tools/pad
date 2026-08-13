// SPDX-License-Identifier: Apache-2.0
//! The redistribution-layer router — grid construction.
//!
//! RDL routing connects bumps to pads across the face of the die on a single thick layer. The
//! search runs over a graph built from that layer's track grid, thinned so that neighbouring wires
//! can never be closer than the requested spacing.
//!
//! ⚠️ **This is the grid only.** The search, obstruction handling and rip-up are separate stages;
//! see the spec. The grid is worth isolating because it is exactly checkable on its own: the
//! reference reports its own vertex count, and vertices are *not* filtered by obstructions.
//!
//! Nothing here touches a database.

/// A point on the routing grid.
pub type Point = (i32, i32);

/// The thinned track grid the router searches over.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Grid {
    pub x: Vec<i32>,
    pub y: Vec<i32>,
}

impl Grid {
    pub fn vertices(&self) -> usize {
        self.x.len() * self.y.len()
    }

    /// Grid points in the order the reference adds them: x outer, y inner.
    pub fn points(&self) -> impl Iterator<Item = Point> + '_ {
        self.x.iter().flat_map(move |&x| self.y.iter().map(move |&y| (x, y)))
    }
}

/// **G1** — thin one axis of a track grid so wires cannot crowd.
///
/// The first track kept is the first at or beyond `width / 2 + 1`, leaving room for half a wire
/// against the edge. After that a track is kept only once it clears the previous one by more than
/// `width + spacing - 1`.
///
/// ⚠️ The comparison is strict and the pitch carries a `- 1`, so a track exactly one pitch away is
/// **kept**. Writing it as `>=` on a full `width + spacing` drops every other track on a grid whose
/// pitch happens to match, and halves the routing resource without any error.
pub fn thin(tracks: &[i32], width: i32, spacing: i32) -> Vec<i32> {
    let pitch = width + spacing - 1;
    let start = width / 2 + 1;
    let mut out: Vec<i32> = Vec::new();
    for &t in tracks {
        let keep = match out.last() {
            None => t >= start,
            Some(&last) => last + pitch < t,
        };
        if keep {
            out.push(t);
        }
    }
    out
}

/// **G2** — the whole grid.
pub fn grid(track_x: &[i32], track_y: &[i32], width: i32, spacing: i32) -> Grid {
    Grid { x: thin(track_x, width, spacing), y: thin(track_y, width, spacing) }
}

/// **G3** — the neighbours of grid position `(i, j)`, as index pairs.
///
/// Four orthogonal neighbours everywhere. With 45-degree routing allowed, four diagonals as well —
/// ⚠️ but **only from positions where both indices are even**. Diagonals from every point would
/// double back on each other; the reference takes every other position in each direction, so the
/// diagonal mesh is half as dense as the orthogonal one.
pub fn neighbours(g: &Grid, i: usize, j: usize, allow45: bool) -> Vec<(usize, usize)> {
    let (nx, ny) = (g.x.len(), g.y.len());
    let mut out = Vec::new();
    // The reference's own order: up, down, left, right. It decides nothing here, but the edge
    // list is compared against the reference's count and later its content.
    if j + 1 < ny {
        out.push((i, j + 1));
    }
    if j != 0 {
        out.push((i, j - 1));
    }
    if i != 0 {
        out.push((i - 1, j));
    }
    if i + 1 < nx {
        out.push((i + 1, j));
    }
    if allow45 && i % 2 == 0 && j % 2 == 0 {
        if i + 1 < nx && j + 1 < ny {
            out.push((i + 1, j + 1));
        }
        if i + 1 < nx && j != 0 {
            out.push((i + 1, j - 1));
        }
        if i != 0 && j + 1 < ny {
            out.push((i - 1, j + 1));
        }
        if i != 0 && j != 0 {
            out.push((i - 1, j - 1));
        }
    }
    out
}

/// **G4** — the distance between two points, **truncated** to a whole unit.
///
/// ⚠️ Truncating collapses distinct geometric distances onto the same integer, which makes ties in
/// the search common rather than rare. That is a property of the reference and has to be kept: a
/// more precise distance is a different router.
pub fn distance(p0: Point, p1: Point) -> i64 {
    let dx = (p0.0 - p1.0) as f64;
    let dy = (p0.1 - p1.1) as f64;
    (dx * dx + dy * dy).sqrt() as i64
}

/// **G5** — an edge's weight.
///
/// ⚠️ A **horizontal** edge costs one unit more than the same-length vertical one. It is not a
/// physical cost: it is a tie-break, and it is what stops two equally short routes from being
/// chosen arbitrarily. Dropping it does not make routes longer, it makes them unstable.
pub fn edge_weight(p0: Point, p1: Point, scale: f32) -> i64 {
    let bias = i64::from(p0.1 == p1.1);
    (scale * distance(p0, p1) as f32) as i64 + bias
}

/// Every edge in the grid, as point pairs, each counted once.
///
/// ⚠️ Deduplicated by the **unordered** pair. Keeping only the pairs that run "forwards" through
/// the grid looks equivalent and is not: a diagonal is generated from even positions only, so
/// `(2,0) - (1,1)` has no counterpart generated from `(1,1)`, and dropping it silently deletes
/// a quarter of the diagonal mesh.
pub fn edges(g: &Grid, allow45: bool) -> Vec<(Point, Point)> {
    let mut seen = std::collections::BTreeSet::new();
    let mut out = Vec::new();
    for i in 0..g.x.len() {
        for j in 0..g.y.len() {
            for (ni, nj) in neighbours(g, i, j, allow45) {
                let key = if (i, j) < (ni, nj) { ((i, j), (ni, nj)) } else { ((ni, nj), (i, j)) };
                if seen.insert(key) {
                    out.push(((g.x[i], g.y[j]), (g.x[ni], g.y[nj])));
                }
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thinning_starts_clear_of_the_edge() {
        // width 4 -> the first track kept is the first at or beyond 3.
        assert_eq!(thin(&[0, 1, 2, 3, 100], 4, 4), vec![3, 100]);
        assert_eq!(thin(&[0, 1, 2], 4, 4), Vec::<i32>::new(), "none far enough in");
    }

    #[test]
    fn a_track_exactly_one_pitch_away_is_kept() {
        // ⚠️ pitch = width + spacing - 1 = 7, and the test is `last + pitch < t`, so a track at
        // last + 8 is kept and one at last + 7 is not.
        assert_eq!(thin(&[10, 17, 18], 4, 4), vec![10, 18]);
    }

    #[test]
    fn thinning_measures_from_the_last_kept_track_not_the_last_seen() {
        // 10 is kept; 12 and 14 are dropped against 10, not against each other.
        assert_eq!(thin(&[10, 12, 14, 30], 4, 4), vec![10, 30]);
    }

    #[test]
    fn a_grid_has_one_vertex_per_crossing() {
        let g = grid(&[10, 30, 50], &[10, 30], 4, 4);
        assert_eq!(g.vertices(), 6);
        assert_eq!(
            g.points().collect::<Vec<_>>(),
            vec![(10, 10), (10, 30), (30, 10), (30, 30), (50, 10), (50, 30)]
        );
    }

    #[test]
    fn orthogonal_edges_are_counted_once_each() {
        // A 3 x 2 grid has 2*3 - 3 - 2 = ... simply: 3 columns x 1 vertical gap + 2 rows x 2
        // horizontal gaps = 3 + 4 = 7.
        let g = grid(&[10, 30, 50], &[10, 30], 4, 4);
        assert_eq!(edges(&g, false).len(), 7);
    }

    #[test]
    fn a_full_square_grid_has_the_expected_edge_count() {
        let coords: Vec<i32> = (0..10).map(|i| 10 + i * 20).collect();
        let g = grid(&coords, &coords, 4, 4);
        assert_eq!(g.vertices(), 100);
        // 2 * n * (n - 1) for an n x n four-connected grid.
        assert_eq!(edges(&g, false).len(), 2 * 10 * 9);
    }

    #[test]
    fn diagonals_are_added_only_from_even_positions() {
        let coords: Vec<i32> = (0..4).map(|i| 10 + i * 20).collect();
        let g = grid(&coords, &coords, 4, 4);
        let plain = edges(&g, false).len();
        let with45 = edges(&g, true).len();
        // ⚠️ Four diagonals from each of the four even-even interior positions, minus those that
        // fall off the grid. Not one per point.
        assert!(with45 > plain, "diagonals were added");
        // Nine diagonals from the four even-even positions of a 4 x 4 grid, once each.
        assert_eq!(with45 - plain, 9, "half-density diagonal mesh");
    }

    #[test]
    fn distance_is_truncated_rather_than_rounded() {
        assert_eq!(distance((0, 0), (3, 4)), 5);
        // ⚠️ 1.41... becomes 1, not 2. Ties in the search follow from this.
        assert_eq!(distance((0, 0), (1, 1)), 1);
        assert_eq!(distance((0, 0), (10, 10)), 14);
    }

    #[test]
    fn a_horizontal_edge_costs_one_more_than_the_same_vertical_one() {
        // ⚠️ The tie-break that makes route choice stable.
        assert_eq!(edge_weight((0, 0), (100, 0), 1.0), 101);
        assert_eq!(edge_weight((0, 0), (0, 100), 1.0), 100);
    }

    #[test]
    fn the_weight_scale_multiplies_the_distance_but_not_the_bias() {
        assert_eq!(edge_weight((0, 0), (100, 0), 2.0), 201);
    }
}
