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
    // ⚠️ Upstream is `const int64_t weight = edge_weight_scale * distance(p0, p1) + direction_bias;`
    // — a `float * int64_t + int64_t` expression evaluated wholly in float and truncated ONCE, at
    // the assignment. Truncating the product first and adding the bias afterwards is a different
    // number whenever the product lands just below an integer, which is exactly what a restored
    // edge's recovered scale produces.
    (scale * distance(p0, p1) as f32 + bias as f32) as i64
}

/// **G5a** — the scale `removeGraphEdge` hands back, recovered by DIVISION.
///
/// ⛔ **Upstream does not store the scale; it divides the weight by the distance to get it back**:
///
/// ```text
/// removeGraphEdge: const float weight = graph_weight_[edge];
///                  return {p0, p1, weight / distance(p0, p1)};
/// uncommitRoute:   addGraphEdge(p0, p1, weight, false, false);   // that quotient, as the SCALE
/// removeTerminalAccess: addGraphEdge(pt0, pt1, weight, true, true);
/// ```
///
/// 🔑 **The round trip does not return the original weight.** A horizontal edge is stored as
/// `d + 1` (the `direction_bias`), so the recovered scale is `(d + 1) / d`, and re-adding gives
/// `(d + 1) + 1 = d + 2`. **Every rip-up or terminal-access cycle makes a horizontal edge one unit
/// dearer**; vertical and 45° edges carry no bias, recover a scale of exactly 1.0, and are
/// unchanged. ⚠️ Measured in the reference's own `Router_edge` log for `rdl_route_45`: **548 edges
/// restored at weight 25602** where a fresh one of that length is 25601.
pub fn restore_scale(p0: Point, p1: Point, weight: i64) -> f32 {
    weight as f32 / distance(p0, p1) as f32
}

/// A rectangle, already bloated by the clearance the router must keep.
pub type Rect = (i32, i32, i32, i32);

/// **G6** — does a segment touch a rectangle?
///
/// ⚠️ **Closed**: a segment that only grazes a corner counts as blocked. The router is deciding
/// whether a wire may run here, and a wire touching an obstruction is a short.
pub fn hits(p0: Point, p1: Point, r: Rect) -> bool {
    // Bounding-box rejection, inclusive on every side.
    if p0.0.max(p1.0) < r.0 || p0.0.min(p1.0) > r.2 {
        return false;
    }
    if p0.1.max(p1.1) < r.1 || p0.1.min(p1.1) > r.3 {
        return false;
    }
    // For an axis-aligned segment the boxes overlapping is the whole answer.
    if p0.0 == p1.0 || p0.1 == p1.1 {
        return true;
    }
    // Otherwise the rectangle must not sit wholly to one side of the segment's line.
    let cross = |x: i32, y: i32| {
        (p1.0 - p0.0) as i64 * (y - p0.1) as i64 - (p1.1 - p0.1) as i64 * (x - p0.0) as i64
    };
    let s = [cross(r.0, r.1), cross(r.2, r.1), cross(r.0, r.3), cross(r.2, r.3)];
    !(s.iter().all(|&v| v > 0) || s.iter().all(|&v| v < 0))
}

/// **G7** — is this segment blocked by anything?
///
/// ℹ️ Obstructions arrive as **rectangles**, including where the reference holds a polygon. That is
/// exact for this question, not an approximation: a segment meets a polygon exactly when it meets
/// one of the rectangles a decomposition of that polygon is made of, and touching counts either
/// way. It is only the *shape* that is decomposed, never the region.
pub fn blocked(p0: Point, p1: Point, obstructions: &[Obstacle]) -> bool {
    obstructions.iter().any(|o| o.hits(p0, p1))
}

/// **G20** — one thing the router must not run into, **at its real shape**.
///
/// ⛔ **A 45° pin is not a rectangle and cannot be decomposed into rectangles.**
/// `populateObstructions` stores each obstruction twice: an enclosing `odb::Rect` that indexes it
/// in the R-tree, and the true `odb::Polygon` (or `odb::Oct`) that decides the hit. We used to keep
/// only rectangles, on the stated grounds that a segment meets a polygon exactly when it meets one
/// of the rectangles the polygon decomposes into.
///
/// ⚠️ **That is true for a rectilinear polygon and FALSE for an octagon.** OpenDB's decomposition
/// goes through `polygon_90`, which cannot represent a 45° edge, so an octagonal bump pad comes
/// back as three rectangles that are not the same shape — over-covering one corner and
/// under-covering the opposite one. The router then refuses tracks where there is no metal and
/// offers tracks where there is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Obstacle {
    Rect(Rect),
    /// A closed ring, with its enclosing rectangle kept alongside as a cheap reject.
    Poly { bbox: Rect, points: Vec<Point> },
}

impl Obstacle {
    /// The enclosing rectangle — what the reference indexes on.
    pub fn bbox(&self) -> Rect {
        match self {
            Obstacle::Rect(r) => *r,
            Obstacle::Poly { bbox, .. } => *bbox,
        }
    }

    /// Does the segment `p0`–`p1` touch this obstacle?
    pub fn hits(&self, p0: Point, p1: Point) -> bool {
        match self {
            Obstacle::Rect(r) => hits(p0, p1, *r),
            // ⚠️ The bounding box is a reject, never an accept: passing it only means the exact
            // test is worth running.
            Obstacle::Poly { bbox, points } => {
                hits(p0, p1, *bbox) && segment_hits_polygon(p0, p1, points)
            }
        }
    }

    /// Build from a closed ring, collapsing an axis-aligned rectangle back to the cheap form.
    pub fn from_ring(points: Vec<Point>) -> Obstacle {
        let (mut x0, mut y0, mut x1, mut y1) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
        for &(x, y) in &points {
            x0 = x0.min(x);
            y0 = y0.min(y);
            x1 = x1.max(x);
            y1 = y1.max(y);
        }
        let bbox = (x0, y0, x1, y1);
        // A ring whose every point is a corner of its own bounding box IS that rectangle, and the
        // rectangle test is both cheaper and exact.
        if points.iter().all(|&(x, y)| (x == x0 || x == x1) && (y == y0 || y == y1)) {
            return Obstacle::Rect(bbox);
        }
        Obstacle::Poly { bbox, points }
    }
}

/// Every edge in the grid, as point pairs, each counted once.
///
/// ⚠️ Deduplicated by the **unordered** pair. Keeping only the pairs that run "forwards" through
/// the grid looks equivalent and is not: a diagonal is generated from even positions only, so
/// `(2,0) - (1,1)` has no counterpart generated from `(1,1)`, and dropping it silently deletes
/// a quarter of the diagonal mesh.
pub fn edges(g: &Grid, allow45: bool) -> Vec<(Point, Point)> {
    edges_clear(g, allow45, &|_, _| false)
}

/// Every edge the router may actually use.
///
/// ⚠️ Vertices are **not** filtered — a grid point inside an obstruction stays in the graph, it
/// simply has no usable edges. Removing it too would renumber everything and change nothing.
pub fn edges_clear(
    g: &Grid,
    allow45: bool,
    obstructed: &dyn Fn(Point, Point) -> bool,
) -> Vec<(Point, Point)> {
    let mut seen = std::collections::BTreeSet::new();
    let mut out = Vec::new();
    for i in 0..g.x.len() {
        for j in 0..g.y.len() {
            for (ni, nj) in neighbours(g, i, j, allow45) {
                let key = if (i, j) < (ni, nj) { ((i, j), (ni, nj)) } else { ((ni, nj), (i, j)) };
                if !seen.insert(key) {
                    continue;
                }
                let (a, b) = ((g.x[i], g.y[j]), (g.x[ni], g.y[nj]));
                if !obstructed(a, b) {
                    out.push((a, b));
                }
            }
        }
    }
    out
}

/// Somewhere the router may start or finish: a pin shape it has to touch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Target {
    pub terminal: String,
    pub centre: Point,
    pub shape: Rect,
    /// Grid points from which this target can be reached, filled in by [`access_points`].
    pub access: Vec<Point>,
    /// ⛔ **The layer this target's PIN metal is on, which is not always the routing layer.**
    /// With `-bump_via` or `-pad_via`, `generateRoutingTargets` accepts pin geometry on the via's
    /// *other* layer and makes the via's enclosure the target. `writeToDb` then keys off exactly
    /// this: `source->layer != layer_` is what makes it drop a via at that end.
    pub layer: String,
}

/// **G9** — the greatest usable grid coordinate strictly below `at`, and the least strictly above.
///
/// ⚠️ Only coordinates that are **not** inside an obstruction take part.
pub fn nearest_tracks(axis: &[i32], at: i32, usable: &dyn Fn(i32) -> bool) -> Vec<i32> {
    let open: Vec<i32> = axis.iter().copied().filter(|&c| usable(c)).collect();
    let mut out = Vec::new();
    if let Some(&below) = open.iter().filter(|&&c| c < at).next_back() {
        out.push(below);
    }
    if let Some(&above) = open.iter().find(|&&c| c > at) {
        out.push(above);
    }
    out
}

/// **G10** — the grid points a target can be entered from.
///
/// Four candidates at most: the nearest usable track on each side, in each axis. A candidate is
/// dropped when the straight run from the target's centre to it would cross an obstruction that
/// belongs to something else.
///
/// ⚠️ An obstruction belonging to **this** terminal does not count. The target sits inside its own
/// pin metal, so every access line starts inside it; treating that as a violation would remove
/// every access point the terminal has and make the net unroutable.
pub fn access_points(
    g: &Grid,
    target: &Target,
    obstructions: &[Obstacle],
    own: &[Obstacle],
) -> Vec<Point> {
    let foreign = |o: &Obstacle| !own.contains(o);
    // ⚠️ **No exemption here.** A candidate track is rejected if it lies inside ANY obstruction,
    // the terminal's own metal included. The exemption applies only to the line test below. Excusing
    // it here picks tracks inside the terminal's own pad, where every grid edge has been filtered
    // away — the access points then attach to dead grid points and the terminal is unreachable,
    // with four perfectly plausible-looking access points to show for it.
    let clear = |p: Point| !obstructions.iter().any(|o| o.hits(p, p));
    let mut out = Vec::new();
    for x in nearest_tracks(&g.x, target.centre.0, &|x| clear((x, target.centre.1))) {
        out.push((x, target.centre.1));
    }
    for y in nearest_tracks(&g.y, target.centre.1, &|y| clear((target.centre.0, y))) {
        out.push((target.centre.0, y));
    }
    out.retain(|&p| !obstructions.iter().any(|o| foreign(o) && o.hits(target.centre, p)));
    out.sort_unstable();
    out.dedup();
    out
}

/// The routing graph: points, and weighted neighbours.
#[derive(Debug, Clone, Default)]
pub struct Graph {
    pub points: Vec<Point>,
    pub index: std::collections::HashMap<Point, usize>,
    pub adj: Vec<Vec<(usize, i64)>>,
}

impl Graph {
    /// Build from a grid and the edges that survived obstruction filtering.
    ///
    /// ⚠️ Every grid point becomes a vertex, including points with no edges at all. The reference
    /// does the same, and vertex numbering follows from it — x outer, y inner.
    pub fn build(g: &Grid, edges: &[(Point, Point)], scale: f32) -> Graph {
        let points: Vec<Point> = g.points().collect();
        let index: std::collections::HashMap<Point, usize> =
            points.iter().enumerate().map(|(i, &p)| (p, i)).collect();
        let mut adj = vec![Vec::new(); points.len()];
        for &(a, b) in edges {
            let (Some(&ia), Some(&ib)) = (index.get(&a), index.get(&b)) else { continue };
            let w = edge_weight(a, b, scale);
            adj[ia].push((ib, w));
            adj[ib].push((ia, w));
        }
        Graph { points, index, adj }
    }
}

impl Graph {
    fn vertex(&mut self, p: Point) -> usize {
        if let Some(&i) = self.index.get(&p) {
            return i;
        }
        let i = self.points.len();
        self.points.push(p);
        self.index.insert(p, i);
        self.adj.push(Vec::new());
        i
    }

    fn join(&mut self, a: usize, b: usize, w: i64) {
        if a != b && !self.adj[a].iter().any(|&(v, _)| v == b) {
            self.adj[a].push((b, w));
            self.adj[b].push((a, w));
        }
    }

    fn cut(&mut self, a: usize, b: usize) {
        self.adj[a].retain(|&(v, _)| v != b);
        self.adj[b].retain(|&(v, _)| v != a);
    }
}

/// **G12** — graft a terminal onto the grid.
///
/// A snap point lies **on** a grid line but between two grid points, so it splits the edge it sits
/// on: that edge is removed, the snap becomes a vertex joined to both former endpoints, and it is
/// joined to the terminal's centre.
///
/// ⚠️ The centre-to-snap edge is added **without an obstruction check**. It runs through the
/// terminal's own pin metal by construction, and checking it would refuse every terminal.
/// Edges to put back when a temporary change is undone.
#[derive(Debug, Clone, Default)]
pub struct Undo {
    /// `(a, b, scale)` — the scale `removeGraphEdge` recovers by division, not the weight.
    pub restore: Vec<(usize, usize, f32)>,
    pub cut: Vec<(usize, usize)>,
}

impl Graph {
    /// Put the graph back as it was before the change `undo` records.
    pub fn undo(&mut self, undo: &Undo) {
        for &(a, b) in &undo.cut {
            self.cut(a, b);
        }
        for &(a, b, scale) in &undo.restore {
            let w = edge_weight(self.points[a], self.points[b], scale);
            self.join(a, b, w);
        }
    }

    pub fn weight_between(&self, a: usize, b: usize) -> Option<i64> {
        self.adj[a].iter().find(|&&(v, _)| v == b).map(|&(_, w)| w)
    }
}

/// ⛔ **`allow45` prunes ACUTE edges here, and the position in the sequence is the rule.** The
/// reference does it inside the per-vertex loop and **after** `addGraphEdge(snap, pt)`, which is
/// why it has to skip the edge it just added (`if (other_pt == snap) continue`). It iterates the
/// edges of the GRID VERTEX, not of the snap, and every removal is recorded so the access can be
/// undone when a route is ripped up.
pub fn insert_access(
    graph: &mut Graph,
    g: &Grid,
    centre: Point,
    snaps: &[Point],
    allow45: bool,
    // ⛔ **The snap-to-neighbour edges are obstruction-checked; the snap-to-CENTRE edge is not.**
    // `addGraphEdge(snap, target.center, 1.0, false)` passes `check_obstructions = false` — the
    // terminal's own metal must not block its own access — while `addGraphEdge(snap, pt)` for each
    // collinear neighbour takes the default and IS checked. Joining both unconditionally hands the
    // router access edges the reference refuses, and it then reaches places the reference cannot.
    obstructed: &dyn Fn(Point, Point) -> bool,
) -> Undo {
    let mut undo = Undo::default();
    let c = graph.vertex(centre);
    for &snap in snaps {
        // The two grid points this snap sits between, on its own line.
        let between = if g.x.binary_search(&snap.0).is_ok() {
            // On a grid column: the neighbours are above and below in y.
            let below = g.y.iter().copied().filter(|&y| y < snap.1).next_back();
            let above = g.y.iter().copied().find(|&y| y > snap.1);
            (below.map(|y| (snap.0, y)), above.map(|y| (snap.0, y)))
        } else {
            let left = g.x.iter().copied().filter(|&x| x < snap.0).next_back();
            let right = g.x.iter().copied().find(|&x| x > snap.0);
            (left.map(|x| (x, snap.1)), right.map(|x| (x, snap.1)))
        };

        let ends: Vec<usize> = [between.0, between.1]
            .into_iter()
            .flatten()
            .filter_map(|p| graph.index.get(&p).copied())
            .collect();
        if let [a, b] = ends[..] {
            if let Some(w) = graph.weight_between(a, b) {
                undo.restore.push((a, b, restore_scale(graph.points[a], graph.points[b], w)));
            }
            graph.cut(a, b);
        }
        let sv = graph.vertex(snap);
        let w = edge_weight(snap, centre, 1.0);
        graph.join(sv, c, w);
        undo.cut.push((sv, c));
        for &e in &ends {
            let pt_e = graph.points[e];
            if obstructed(snap, pt_e) {
                continue;
            }
            let w = edge_weight(snap, pt_e, 1.0);
            graph.join(sv, e, w);
            undo.cut.push((sv, e));

            // ⛔ **The acute prune, in the reference's own position: after the join above.** A
            // diagonal leaving this grid point in the SAME direction the access edge arrives from
            // is a turn a wire cannot make, so the reference deletes it before any search runs.
            //
            // ⚠️ The axis test is not symmetric: `snap_dy == 0` compares x and EVERYTHING ELSE
            // compares y, a diagonal snap included. Transcribed rather than tidied.
            if allow45 {
                let pt = graph.points[e];
                let (snap_dx, snap_dy) = (pt.0 - snap.0, pt.1 - snap.1);
                let doomed: Vec<usize> = graph.adj[e]
                    .iter()
                    .map(|&(o, _)| o)
                    .filter(|&o| {
                        let other = graph.points[o];
                        if other == snap {
                            return false;           // the edge just added
                        }
                        if pt.0 == other.0 || pt.1 == other.1 {
                            return false;           // a right angle stays
                        }
                        let (edge_dx, edge_dy) = (pt.0 - other.0, pt.1 - other.1);
                        if snap_dy == 0 {
                            (snap_dx < 0 && edge_dx < 0) || (snap_dx > 0 && edge_dx > 0)
                        } else {
                            (snap_dy < 0 && edge_dy < 0) || (snap_dy > 0 && edge_dy > 0)
                        }
                    })
                    .collect();
                for o in doomed {
                    if let Some(w) = graph.weight_between(e, o) {
                        undo.restore
                            .push((e, o, restore_scale(graph.points[e], graph.points[o], w)));
                    }
                    graph.cut(e, o);
                }
            }
        }
    }
    undo
}

/// **G21** — does an existing special wire obstruct the router?
///
/// 🔑 **Upstream rule** (`RDLRouter::populateObstructions`), and the narrowness is the point:
///
/// ```text
/// for swire in net->getSWires():
///     if is_routing_net && swire->getWireType() != FIXED:  continue
/// ```
///
/// ⛔ It is **not** "a net's own metal does not obstruct it". `route()` destroys every non-FIXED
/// swire of the nets being routed before it writes, so those wires are about to cease to exist and
/// obstructing against them would be obstructing against nothing. A **FIXED** swire is kept — so it
/// obstructs its own net's router, and every swire of every other net obstructs unconditionally.
pub fn swire_obstructs(is_routing_net: bool, swire_is_fixed: bool) -> bool {
    !(is_routing_net && !swire_is_fixed)
}

/// **G17a** — `odb::Oct::getPoints()`, the plain octagon, before any caller mutates its ring.
///
/// Shared by [`edge_obstruction`] — which is this shape with four ring indices reassigned — and by
/// the obstruction built from an OCTILINEAR special wire, which uses it unaltered.
///
/// ```text
/// Oct::init(p0, p1, width):  high = the point with the LARGER y (a tie makes p1 high)
///                            A = width / 2                      # INTEGER division
/// getDir():  RIGHT if high.x > low.x, else LEFT (UNKNOWN, when the centres coincide, takes LEFT)
/// B = ceil((A * 2) / sqrt2) - A                                 # ceil in f64, then TRUNCATED
/// ```
///
/// ⚠️ `A` and `B` are the two numeric traps: `width / 2` truncates, and `B`'s `ceil` happens in
/// floating point and is then truncated into an integer. Computing either exactly is a different
/// polygon (numeric reference §1).
///
/// Returns the 9-point ring, first point repeated last.
pub fn oct_points(p0: Point, p1: Point, width: i32) -> Vec<Point> {
    let (low, high) = if p0.1 > p1.1 { (p1, p0) } else { (p0, p1) };
    let a = width / 2;
    let b = (((a * 2) as f64) / std::f64::consts::SQRT_2).ceil() as i32 - a;
    let right = high.0 > low.0;

    let mut pts = vec![(0, 0); 9];
    pts[0] = (low.0 - b, low.1 - a);
    pts[8] = pts[0];
    pts[1] = (low.0 + b, low.1 - a);
    pts[4] = (high.0 + b, high.1 + a);
    pts[5] = (high.0 - b, high.1 + a);
    if right {
        pts[2] = (high.0 + a, high.1 - b);
        pts[3] = (high.0 + a, high.1 + b);
        pts[6] = (low.0 - a, low.1 + b);
        pts[7] = (low.0 - a, low.1 - b);
    } else {
        pts[2] = (low.0 + a, low.1 - b);
        pts[3] = (low.0 + a, low.1 + b);
        pts[6] = (high.0 - a, high.1 + b);
        pts[7] = (high.0 - a, high.1 - b);
    }
    pts
}

/// **G17** — the swept octagon a 45° wire occupies, upstream's `RDLSegment::getEdgeObstruction`.
///
/// 🔑 **The whole calculation, transcribed** (`RDLSegment.cpp` + `odb::Oct` in `geom.h`):
///
/// ```text
/// Oct(p0, p1, 2*dist):  high = the point with the LARGER y (a tie makes p1 high); A = width/2
/// dir:                  RIGHT if high.x > low.x, else LEFT
/// B = ceil((A * 2) / sqrt2) - A
/// p0 = p8 = (low.x-B,  low.y-A)      p1 = (low.x+B,  low.y-A)
/// p4      = (high.x+B, high.y+A)     p5 = (high.x-B, high.y+A)
/// RIGHT:  p2=(high.x+A,high.y-B) p3=(high.x+A,high.y+B) p6=(low.x-A,low.y+B)  p7=(low.x-A,low.y-B)
/// LEFT:   p2=(low.x+A, low.y-B)  p3=(low.x+A, low.y+B)  p6=(high.x-A,high.y+B) p7=(high.x-A,high.y-B)
/// then, with A == dist:
/// RIGHT:  p1.x = low.x+dist;  p2.y = high.y-dist;  p5.x = high.x-dist;  p6.y = low.y+dist
/// LEFT:   p3.y = low.y+dist;  p4.x = high.x+dist;  p7.y = high.y-dist;  p8.x = low.x-dist; p0 = p8
/// ```
///
/// ⚠️ **Three numeric details, all load-bearing.** `A = width / 2` is INTEGER division; `B`'s
/// `ceil` happens in `f64` and is then TRUNCATED into an integer; and **only the LEFT branch
/// re-closes the ring** by reassigning `p0` — the RIGHT branch leaves `p0 == p8` because it
/// touches neither. Writing `B` in exact arithmetic, or closing the ring in both branches, is a
/// different polygon.
///
/// Returns the 9-point ring, first point repeated last.
pub fn edge_obstruction(p0: Point, p1: Point, dist: i32) -> Vec<Point> {
    // `Oct(p0, p1, 2 * dist)` — so A is exactly `dist`.
    let (low, high) = if p0.1 > p1.1 { (p1, p0) } else { (p0, p1) };
    let right = high.0 > low.0;
    let mut pts = oct_points(p0, p1, 2 * dist);

    if right {
        pts[1].0 = low.0 + dist;
        pts[2].1 = high.1 - dist;
        pts[5].0 = high.0 - dist;
        pts[6].1 = low.1 + dist;
    } else {
        pts[3].1 = low.1 + dist;
        pts[4].0 = high.0 + dist;
        pts[7].1 = high.1 - dist;
        pts[8].0 = low.0 - dist;
        pts[0] = pts[8];
    }
    pts
}

/// Do the two segments touch? Endpoints and collinear overlap count, as they do in
/// `boost::geometry::intersects`.
///
/// Integer throughout — the cross products are taken in `i64`, so nothing here rounds.
pub fn segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool {
    let cross = |o: Point, p: Point, q: Point| -> i64 {
        (p.0 - o.0) as i64 * (q.1 - o.1) as i64 - (p.1 - o.1) as i64 * (q.0 - o.0) as i64
    };
    let on = |o: Point, p: Point, q: Point| -> bool {
        q.0.min(o.0) <= p.0 && p.0 <= q.0.max(o.0) && q.1.min(o.1) <= p.1 && p.1 <= q.1.max(o.1)
    };
    let (d1, d2) = (cross(p3, p4, p1), cross(p3, p4, p2));
    let (d3, d4) = (cross(p1, p2, p3), cross(p1, p2, p4));
    if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
        return true;
    }
    (d1 == 0 && on(p3, p1, p4))
        || (d2 == 0 && on(p3, p2, p4))
        || (d3 == 0 && on(p1, p3, p2))
        || (d4 == 0 && on(p1, p4, p2))
}

/// **G19** — every pairing of a source target with a destination target, in the order tried.
///
/// 🔑 Upstream rule (`RDLRouter::route`): the cross product, `stable_sort`ed by centre-to-centre
/// distance, then by `target0.x`, `target0.y`, `target1.x`, `target1.y`. Each pair is attempted in
/// turn and the first that routes wins, so this order is the answer whenever two pairings are
/// equally long.
///
/// ⚠️ `distance` is the TRUNCATED Euclidean length, so two pairings that differ by less than one
/// database unit of length tie here and are settled by the coordinates.
pub fn target_pairs(src: &[Point], dst: &[Point]) -> Vec<(usize, usize)> {
    let mut pairs: Vec<(usize, usize)> =
        (0..src.len()).flat_map(|a| (0..dst.len()).map(move |b| (a, b))).collect();
    pairs.sort_by(|&(la, lb), &(ra, rb)| {
        let (l0, l1) = (src[la], dst[lb]);
        let (r0, r1) = (src[ra], dst[rb]);
        distance(l0, l1)
            .cmp(&distance(r0, r1))
            .then(l0.0.cmp(&r0.0))
            .then(l0.1.cmp(&r0.1))
            .then(l1.0.cmp(&r1.0))
            .then(l1.1.cmp(&r1.1))
    });
    pairs
}

/// **G18** — an access line that crosses one of the terminal's own DIAGONAL pin edges is dropped.
///
/// 🔑 Upstream rule (the tail of `populateTerminalAccessPoints`): gather the terminal's polygon
/// geometry on the target's layer, and for every candidate snap point test the line from the
/// target centre against each polygon edge — **skipping the axis-aligned edges explicitly**. Every
/// snap that crosses a diagonal edge is removed.
///
/// ⚠️ The comment above it upstream reads *"if at least one passes a non-rect edge, remove all
/// violating points"*, which is looser than the code: there is no "at least one" condition, every
/// violating point goes. ⚠️ The polygons are the RAW transformed pin shapes, not the shrunken ones
/// target generation walks.
pub fn snaps_clear_of_diagonal_pin_edges(
    centre: Point,
    snaps: &[Point],
    polygons: &[Vec<Point>],
) -> Vec<Point> {
    if polygons.is_empty() {
        return snaps.to_vec();
    }
    snaps
        .iter()
        .copied()
        .filter(|&snap| {
            !polygons.iter().any(|poly| {
                poly.windows(2).any(|w| {
                    if w[0].0 == w[1].0 || w[0].1 == w[1].1 {
                        return false; // an axis-aligned pin edge is not a barrier
                    }
                    segments_intersect(centre, snap, w[0], w[1])
                })
            })
        })
        .collect()
}

/// Does the segment `a`–`b` touch the closed ring `poly`?
///
/// The ring is convex, so a segment misses it only when it misses every edge AND neither end is
/// inside. Both tests are integer; nothing here rounds.
pub fn segment_hits_polygon(a: Point, b: Point, poly: &[Point]) -> bool {
    let segs_cross = segments_intersect;
    for w in poly.windows(2) {
        if segs_cross(a, b, w[0], w[1]) {
            return true;
        }
    }
    // Neither edge crossed: the segment is either wholly inside or wholly outside. A ray cast
    // from one end settles it.
    let inside = |pt: Point| -> bool {
        let mut c = false;
        for w in poly.windows(2) {
            let (i, j) = (w[0], w[1]);
            if (i.1 > pt.1) != (j.1 > pt.1) {
                let t = (pt.1 - i.1) as i64 * (j.0 - i.0) as i64;
                let u = (j.1 - i.1) as i64;
                let x = i.0 as i64 + if u != 0 { t / u } else { 0 };
                if (pt.0 as i64) < x {
                    c = !c;
                }
            }
        }
        c
    };
    inside(a)
}

/// **L6** — take a routed path out of the graph, recording how to put it back.
///
/// Every edge touching a route vertex goes, and so does every edge crossing the route's corridor.
/// ⚠️ Recorded rather than simply deleted: rip-up has to restore exactly these edges, and
/// recomputing which ones they were after the graph has moved on gives a different set.
pub fn commit_route(
    graph: &mut Graph,
    route: &[Point],
    width: i32,
    spacing: i32,
    allow45: bool,
) -> Undo {
    let mut undo = Undo::default();
    let corridor = commit_corridor(route, width, spacing);
    let mut drop: std::collections::BTreeSet<(usize, usize)> = Default::default();

    for p in route {
        if let Some(&v) = graph.index.get(p) {
            for &(o, _) in &graph.adj[v] {
                drop.insert(if v < o { (v, o) } else { (o, v) });
            }
        }
    }
    // ⚠️ An edge can cross the corridor with **both endpoints outside it**, and those must go too.
    // Testing only the edges of points inside the corridor leaves a wire's neighbours connected
    // straight through it — the graph then offers routes that physically overlap an existing one,
    // and the router succeeds where it should have had to find another way.
    //
    // Widening the search by the longest edge span keeps the candidate set a superset of what can
    // reach in, which keeps this exact without visiting every edge in the graph.
    let span = graph
        .points
        .iter()
        .enumerate()
        .flat_map(|(i, &p)| graph.adj[i].iter().map(move |&(o, _)| (p, o)))
        .map(|(p, o)| {
            let q = graph.points[o];
            (p.0 - q.0).abs().max((p.1 - q.1).abs())
        })
        .max()
        .unwrap_or(0);

    for (i, &p) in graph.points.iter().enumerate() {
        let near = corridor
            .iter()
            .any(|&r| hits(p, p, (r.0 - span, r.1 - span, r.2 + span, r.3 + span)));
        if !near {
            continue;
        }
        for &(o, _) in &graph.adj[i] {
            let q = graph.points[o];
            if corridor.iter().any(|&r| hits(p, q, r)) {
                drop.insert(if i < o { (i, o) } else { (o, i) });
            }
        }
    }
    // ⛔ **Under `allow45`, a committed DIAGONAL also blocks what crosses its swept octagon.**
    // This is the third and last of the reference's `allow45` sites, and it runs after the two
    // above because everything accumulates into one set before anything is removed.
    //
    // ⚠️ **The loop starts at 2, so the FIRST segment is never checked.** That is the reference's
    // own off-by-one (`for (std::size_t i = 2; i < route.size(); i++)`), reproduced rather than
    // tidied: starting at 1 removes edges it keeps.
    if allow45 {
        let d = width / 2 + spacing + 1;
        for i in 2..route.len() {
            let (p0, p1) = (route[i - 1], route[i]);
            if p0.0 == p1.0 || p0.1 == p1.1 {
                continue;                       // `is45DegreeEdge` is simply "not axis-aligned"
            }
            let poly = edge_obstruction(p0, p1, d);
            for (vi, &p) in graph.points.iter().enumerate() {
                for &(o, _) in &graph.adj[vi] {
                    if o < vi {
                        continue;               // each undirected edge once
                    }
                    if segment_hits_polygon(p, graph.points[o], &poly) {
                        drop.insert((vi, o));
                    }
                }
            }
        }
    }

    for (a, b) in drop {
        if let Some(w) = graph.weight_between(a, b) {
            undo.restore.push((a, b, restore_scale(graph.points[a], graph.points[b], w)));
            graph.cut(a, b);
        }
    }
    undo
}

/// A 4-ary min-heap keyed on cost alone, matching the reference's queue element for element.
///
/// ⚠️ **The key is the cost and nothing else.** Adding the vertex as a secondary key — the obvious
/// way to make the order deterministic — gives a *different* deterministic order, and among
/// equal-cost paths the search then returns a different one. The tie is settled by where an entry
/// lands in the array, so the array's exact shape is the answer rather than a detail of it.
#[derive(Default)]
struct CostHeap {
    data: Vec<(i64, usize)>,
}

impl CostHeap {
    const ARITY: usize = 4;

    fn parent(i: usize) -> usize {
        (i - 1) / Self::ARITY
    }

    fn first_child(i: usize) -> usize {
        i * Self::ARITY + 1
    }

    fn push(&mut self, v: (i64, usize)) {
        self.data.push(v);
        self.sift_up(self.data.len() - 1);
    }

    fn pop(&mut self) -> Option<(i64, usize)> {
        if self.data.is_empty() {
            return None;
        }
        let top = self.data[0];
        if self.data.len() == 1 {
            self.data.pop();
        } else {
            self.data[0] = *self.data.last().unwrap();
            self.data.pop();
            self.sift_down();
        }
        Some(top)
    }

    /// ⚠️ Strictly less than the parent moves up; **equal stays put**. That single `<` is what
    /// decides which of two equal-cost routes the search settles on.
    fn sift_up(&mut self, orig: usize) {
        if orig == 0 {
            return;
        }
        let moving = self.data[orig];
        let mut index = orig;
        let mut levels = 0;
        while index != 0 {
            let p = Self::parent(index);
            if moving.0 < self.data[p].0 {
                levels += 1;
                index = p;
            } else {
                break;
            }
        }
        let mut index = orig;
        for _ in 0..levels {
            let p = Self::parent(index);
            self.data[index] = self.data[p];
            index = p;
        }
        self.data[index] = moving;
    }

    /// ⚠️ Among equal children the **first** wins: the scan replaces only on strictly less.
    fn sift_down(&mut self) {
        let mut index = 0usize;
        let size = self.data.len();
        loop {
            let first = Self::first_child(index);
            if first >= size {
                break;
            }
            let last = (first + Self::ARITY).min(size);
            let mut best = first;
            for i in (first + 1)..last {
                if self.data[i].0 < self.data[best].0 {
                    best = i;
                }
            }
            if self.data[best].0 < self.data[index].0 {
                self.data.swap(best, index);
                index = best;
            } else {
                break;
            }
        }
    }
}

/// **G11** — the cheapest route between two points.
///
/// A\* with a **path-dependent** heuristic: the estimate for a vertex includes a penalty when
/// arriving there would turn, and "would turn" is judged from the predecessor recorded *so far*.
///
/// ⚠️ That makes the heuristic inadmissible and the search order significant — this mirrors the
/// reference's tree search, which keeps no closed set and may re-expand a vertex when a cheaper
/// way to it turns up. A textbook A\* with a closed set finds a path of the same cost and not
/// necessarily the same path.
pub fn shortest_path(
    graph: &Graph,
    start: Point,
    goal: Point,
    turn_penalty: f32,
) -> Vec<Point> {
    let (Some(&s), Some(&t)) = (graph.index.get(&start), graph.index.get(&goal)) else {
        return Vec::new();
    };
    let n = graph.points.len();
    let mut dist = vec![i64::MAX; n];
    let mut prev = vec![usize::MAX; n];
    let mut heap = CostHeap::default();

    let heuristic = |v: usize, prev: &[usize]| -> i64 {
        let pt = graph.points[v];
        let base = distance(goal, pt);
        let c = prev[v];
        if c == usize::MAX || c == s {
            return base;
        }
        let b = prev[c];
        if b == usize::MAX || b == s {
            return base;
        }
        let (pc, pb) = (graph.points[c], graph.points[b]);
        let incoming = (pc.0 - pb.0, pc.1 - pb.1);
        let outgoing = (pt.0 - pc.0, pt.1 - pc.1);
        if incoming == outgoing {
            base
        } else {
            base + (turn_penalty * distance(pb, pc) as f32) as i64
        }
    };

    dist[s] = 0;
    prev[s] = s;
    // ⚠️ **Ordered by f ALONE**, as the reference's queue is — see `CostHeap`. An earlier comment
    // here claimed the vertex number was a secondary key; it never was, and the heap ignores it.
    // Adding it would give a different deterministic order and a different equal-cost path.
    heap.push((heuristic(s, &prev), s));

    while let Some((_, u)) = heap.pop() {
        if u == t {
            let mut path = vec![graph.points[u]];
            let mut v = u;
            while prev[v] != v {
                v = prev[v];
                path.push(graph.points[v]);
            }
            path.reverse();
            return path;
        }
        for &(v, w) in &graph.adj[u] {
            // ⛔ **`boost::relax` has a SECOND branch, and the graph is `undirectedS`.** When the
            // forward relaxation fails it tries the edge the other way and may improve the vertex
            // being EXPANDED, from its neighbour:
            //
            // ```text
            // if (d[u] + w < d[v])            { d[v] = d[u]+w; p[v] = u; return true; }
            // else if (is_undirected && d[v] + w < d[u]) { d[u] = d[v]+w; p[u] = v; return true; }
            // ```
            //
            // ⚠️ Two consequences, both load-bearing. `p[u]` can change **mid-expansion**, and the
            // heuristic reads `predecessor[predecessor[w]]` — so every later neighbour in this same
            // out-edge loop is scored against a different incoming direction. And the caller pushes
            // the TARGET either way (`w_rank = distance[w] + h(w)`), so the vertex that actually
            // improved is not the one queued.
            //
            // ℹ️ The reverse branch cannot overflow: it is only reached when the forward test
            // failed, which means `dist[v]` is already finite.
            let decreased = if dist[u].saturating_add(w) < dist[v] {
                dist[v] = dist[u].saturating_add(w);
                prev[v] = u;
                true
            } else if dist[v].saturating_add(w) < dist[u] {
                dist[u] = dist[v].saturating_add(w);
                prev[u] = v;
                true
            } else {
                false
            };
            if decreased {
                let f = dist[v].saturating_add(heuristic(v, &prev));
                heap.push((f, v));
            }
        }
    }
    Vec::new()
}

/// A wire piece: a rectangle for a straight run, or a 45-degree centre line with a width.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wire {
    Straight(Rect),
    Diagonal(Point, Point, i32),
}

fn direction(s: Point, t: Point) -> u8 {
    if s.1 == t.1 {
        0
    } else if s.0 == t.0 {
        1
    } else if (s.0 < t.0) == (s.1 < t.1) {
        2
    } else {
        3
    }
}

/// **G13** — collapse a grid path into straight runs, mitring the corners.
///
/// Consecutive points going the same way become one run. ⚠️ At a right-angle turn both runs are
/// **extended by half the wire width** — the incoming one ends half a width *past* the corner and
/// the outgoing one starts half a width *back*. Without it the two rectangles meet corner to
/// corner and leave a notch of bare substrate on the inside of every bend.
pub fn simplify(route: &[Point], width: i32) -> Vec<(Point, Point)> {
    if route.len() < 2 {
        return Vec::new();
    }
    let ext = width / 2;
    let mut wire = vec![(route[0], route[1])];
    let mut dir = direction(route[0], route[1]);
    for &t in &route[2..] {
        let mut s = wire.last().unwrap().1;
        let seg = direction(s, t);
        if seg == dir {
            wire.last_mut().unwrap().1 = t;
            continue;
        }
        if dir == 0 && seg == 1 {
            let prev = wire.last().unwrap().0;
            wire.last_mut().unwrap().1.0 = if prev.0 < s.0 { s.0 + ext } else { s.0 - ext };
            s.1 = if s.1 < t.1 { s.1 - ext } else { s.1 + ext };
        } else if dir == 1 && seg == 0 {
            let prev = wire.last().unwrap().0;
            wire.last_mut().unwrap().1.1 = if prev.1 < s.1 { s.1 + ext } else { s.1 - ext };
            s.0 = if s.0 < t.0 { s.0 - ext } else { s.0 + ext };
        }
        wire.push((s, t));
        dir = seg;
    }
    wire
}

/// **G14** — a straight run's rectangle: the segment, widened sideways only.
pub fn run_rect(s: Point, t: Point, width: i32) -> Rect {
    let half = width / 2;
    if s.0 == t.0 {
        (s.0 - half, s.1.min(t.1), s.0 + half, s.1.max(t.1))
    } else {
        (s.0.min(t.0), s.1 - half, s.0.max(t.0), s.1 + half)
    }
}

/// **G15** — make the first and last run reach right into the pad it serves.
///
/// ⚠️ Only when the run is **wider across** than the target it lands on. A run no wider than the
/// pad is already covered by it, and merging then would stretch the wire to the pad's far edge for
/// no reason.
pub fn correct_end(run: Rect, horizontal: bool, target: Rect) -> Rect {
    let across = |r: Rect| if horizontal { r.3 - r.1 } else { r.2 - r.0 };
    if across(run) <= across(target) {
        return run;
    }
    (run.0.min(target.0), run.1.min(target.1), run.2.max(target.2), run.3.max(target.3))
}

/// **G16** — the wire pieces for one routed path.
pub fn wires(route: &[Point], width: i32, source: Rect, target: Rect) -> Vec<Wire> {
    let runs = simplify(route, width);
    let last = runs.len().saturating_sub(1);
    runs.iter()
        .enumerate()
        .map(|(i, &(s, t))| {
            if s.0 != t.0 && s.1 != t.1 {
                return Wire::Diagonal(s, t, width);
            }
            let mut r = run_rect(s, t, width);
            let horizontal = s.1 == t.1;
            if i == 0 {
                r = correct_end(r, horizontal, source);
            } else if i == last {
                r = correct_end(r, horizontal, target);
            }
            Wire::Straight(r)
        })
        .collect()
}

/// A destination a route may connect to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dest {
    pub terminal: String,
    pub instance: String,
    pub centre: Point,
    pub cover: bool,
    pub id: u64,
}

/// One routing job: a bump terminal and the terminals it may reach, in attempt order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Route {
    pub source: String,
    pub instance: String,
    pub centre: Point,
    pub id: u64,
    pub dests: Vec<Dest>,
    pub next: usize,
    pub priority: i32,
    pub routed: bool,
    /// Set until the route has been taken off the queue at least once.
    pub pending: bool,
    /// The path committed for this route, empty when it is not routed.
    pub points: Vec<Point>,
    /// **`RDLSegment::preprocess` locked this segment**: its terminals already touch, so it is
    /// marked routed before the queue is built and is never attempted.
    pub locked: bool,
    /// The stubs that bridge a sub-spacing gap, written INSTEAD of a route. Empty when the shapes
    /// simply overlap — that case writes no wire at all, not even a swire.
    pub stubs: Vec<Rect>,
}

/// Squared distance — exact in integers, and all the ordering rules need.
fn squared(a: Point, b: Point) -> i64 {
    let (dx, dy) = ((a.0 - b.0) as i64, (a.1 - b.1) as i64);
    dx * dx + dy * dy
}

/// **R1** — the order a route tries its destinations in.
///
/// Destinations on the **same instance** as the source are dropped: a bump does not route to
/// itself. What remains is sorted **non-cover first**, then by distance, then by terminal id.
///
/// ⚠️ Non-cover first means a bump prefers a *pad* over another bump even when the other bump is
/// nearer. Sorting purely by distance chains bumps to each other and leaves the pads unreached.
pub fn order_dests(source_instance: &str, source_centre: Point, dests: &[Dest]) -> Vec<Dest> {
    let mut out: Vec<Dest> =
        dests.iter().filter(|d| d.instance != source_instance).cloned().collect();
    out.sort_by(|a, b| {
        (a.cover, squared(source_centre, a.centre), a.id)
            .cmp(&(b.cover, squared(source_centre, b.centre), b.id))
    });
    out
}

impl Route {
    pub fn has_next(&self) -> bool {
        self.next < self.dests.len()
    }

    pub fn peek(&self) -> Option<&Dest> {
        self.dests.get(self.next)
    }

    /// **R2** — which of two routes is attempted first.
    ///
    /// Higher **priority** first — priority only rises when a route is ripped up, so a route that
    /// has been displaced gets another go before anything new is tried. Among equals, the
    /// **shortest** next connection first. A tie on distance is settled by terminal id.
    ///
    /// ⚠️ A route with no destinations left is ordered by priority alone; asking it for a distance
    /// would read past the end of its list.
    pub fn precedes(&self, other: &Route) -> std::cmp::Ordering {
        use std::cmp::Ordering;
        if self.priority != other.priority {
            return other.priority.cmp(&self.priority);
        }
        let (Some(a), Some(b)) = (self.peek(), other.peek()) else {
            return Ordering::Equal;
        };
        let (da, db) = (squared(self.centre, a.centre), squared(other.centre, b.centre));
        match da.cmp(&db) {
            Ordering::Equal => other.id.cmp(&self.id),
            o => o,
        }
    }
}

/// **L1** — a route has failed when it is off the queue, unrouted, and out of destinations.
///
/// ⚠️ All three. A route still on the queue has not failed, it has not been tried; and one with
/// destinations left is not finished with.
pub fn is_failed(r: &Route) -> bool {
    !r.pending && !r.routed && !r.has_next()
}

/// **L2** — the failures worth acting on.
///
/// ⚠️ A route whose own source terminal was reached by **some other** route is not a failure: the
/// bump is connected, just not by this route. Counting it would rip up working routes to retry
/// something already done.
pub fn failed_routes(routes: &[Route]) -> Vec<usize> {
    let reached: std::collections::BTreeSet<&str> = routes
        .iter()
        .filter(|r| r.routed)
        .flat_map(|r| {
            std::iter::once(r.source.as_str())
                .chain(r.peek().map(|d| d.terminal.as_str()))
        })
        .collect();
    routes
        .iter()
        .enumerate()
        .filter(|(_, r)| is_failed(r) && !reached.contains(r.source.as_str()))
        .map(|(i, _)| i)
        .collect()
}

/// **L3** — the corridor a failed route probes when looking for routes to displace.
///
/// A straight line from the route's source to **one of** its destinations, widened by a margin.
///
/// ⚠️ Two things change with each failure, and they are the whole convergence mechanism: the
/// destination index **rotates** (`priority % count`), so a route that keeps failing probes a
/// different corridor each round rather than the same one; and the margin **widens**
/// (`(priority + 1) * extent`), so it displaces more each time. Fixing either one makes a
/// congested design loop until the iteration limit instead of resolving.
pub fn ripup_probe(r: &Route, extent: i32) -> Option<(Point, Point, i32)> {
    if r.dests.is_empty() {
        return None;
    }
    let idx = (r.priority.max(0) as usize) % r.dests.len();
    Some((r.centre, r.dests[idx].centre, (r.priority + 1) * extent))
}

/// **L4** — may this routed route be displaced for a failure of that priority?
///
/// ⚠️ `>=`, not `>`. A route may displace another of **equal** priority; without that a design
/// where everything has failed once can never rearrange itself.
pub fn allows_ripup(candidate: &Route, failed_priority: i32) -> bool {
    candidate.routed && failed_priority >= candidate.priority
}

/// **L5** — which routed routes a failure would displace.
pub fn ripup_targets(failed: &Route, routes: &[Route], extent: i32) -> Vec<usize> {
    let Some((a, b, margin)) = ripup_probe(failed, extent) else {
        return Vec::new();
    };
    routes
        .iter()
        .enumerate()
        .filter(|(_, r)| allows_ripup(r, failed.priority))
        .filter(|(_, r)| {
            r.points
                .iter()
                .any(|&p| hits(a, b, (p.0 - margin, p.1 - margin, p.0 + margin, p.1 + margin)))
        })
        .map(|(i, _)| i)
        .collect()
}

/// **R3** — the corridor a committed route takes out of the graph.
///
/// Every edge touching a route vertex goes, and so does every edge crossing the box of
/// `width / 2 + spacing + 1` around each vertex. ⚠️ The `+ 1` is the reference's, and it is what
/// leaves a wire one unit clear of its neighbours rather than exactly touching them.
pub fn commit_corridor(route: &[Point], width: i32, spacing: i32) -> Vec<Rect> {
    let d = width / 2 + spacing + 1;
    route.iter().map(|&(x, y)| (x - d, y - d, x + d, y + d)).collect()
}

/// What one completed run produced.
#[derive(Debug, Clone, Default)]
pub struct Routed {
    /// `(net, source terminal, destination terminal, path)` for every route that connected.
    pub paths: Vec<(String, String, String, Vec<Point>)>,
    /// Every attempt in order: `(source terminal, destination terminal, source centre,
    /// destination centre, path length)`. A length of zero is an attempt that found nothing.
    ///
    /// ⚠️ The TERMINALS are here so this can be diffed against the reference's own
    /// `Routing {src} -> {dst}` debug line. Centres alone cannot be: with several targets per
    /// terminal the same pair is attempted from different points.
    pub log: Vec<(String, String, Point, Point, usize)>,
    pub attempts: usize,
    pub iterations: i32,
    pub failed: Vec<String>,
}

/// **L9a** — `RDLNet::isRouted`: are these two terminals ALREADY CONNECTED, through any chain of
/// committed routes?
///
/// ⛔ **It is a transitive search, not a pair lookup.** `updateRoute` records each committed
/// segment's two terminals in `routed_pairs_` **both ways**, and `isRouted` then descends that
/// graph:
///
/// ```text
/// isRouted(source, dest):        if source == dest: true
///                                visited = {};  isRouted(source, dest, visited)
/// isRouted(source, dest, seen):  dests = routed_pairs_[source]  (absent -> false)
///                                if dest in dests: true
///                                for d in dests: if d not in seen: seen += d
///                                                if isRouted(d, dest, seen): true
///                                false
/// ```
///
/// ⚠️ Only segments that actually committed a path contribute: `updateRoute` is reached from
/// `setRoute` and `resetRoute`, never from the `setRouted()` that `preprocess` ends in, so a locked
/// segment is not in this graph.
pub fn routed_pairs(routes: &[Route]) -> std::collections::HashMap<&str, Vec<&str>> {
    let mut adj: std::collections::HashMap<&str, Vec<&str>> = Default::default();
    for r in routes {
        if !r.routed || r.points.len() <= 1 {
            continue;
        }
        let Some(d) = r.dests.get(r.next.saturating_sub(1)) else { continue };
        adj.entry(r.source.as_str()).or_default().push(d.terminal.as_str());
        adj.entry(d.terminal.as_str()).or_default().push(r.source.as_str());
    }
    adj
}

/// **L9b** — the descent itself, over the graph [`routed_pairs`] builds.
pub fn is_routed(adj: &std::collections::HashMap<&str, Vec<&str>>, source: &str, dest: &str) -> bool {
    if source == dest {
        return true;
    }
    let mut seen: std::collections::HashSet<&str> = Default::default();
    let mut stack = vec![source];
    while let Some(n) = stack.pop() {
        let Some(ns) = adj.get(n) else { continue };
        for &m in ns {
            if m == dest {
                return true;
            }
            if seen.insert(m) {
                stack.push(m);
            }
        }
    }
    false
}

/// Does the point lie ON the segment `a`–`b`?
///
/// `RDLSegment::isIntersecting(line, 0)` tests the line against
/// `getPointObstruction(pt, 0)` — a rectangle of side zero, i.e. the point itself — so this is
/// exactly "the line passes through that route vertex", endpoints included.
fn point_on_segment(pt: Point, a: Point, b: Point) -> bool {
    let cross = (b.0 as i64 - a.0 as i64) * (pt.1 as i64 - a.1 as i64)
        - (b.1 as i64 - a.1 as i64) * (pt.0 as i64 - a.0 as i64);
    cross == 0
        && pt.0 >= a.0.min(b.0)
        && pt.0 <= a.0.max(b.0)
        && pt.1 >= a.1.min(b.1)
        && pt.1 <= a.1.max(b.1)
}

/// **L10** — `removeTerminalAccess` puts back what it took, but **with the checks ON**.
///
/// ⛔ **The two undo paths differ, and only in their last two arguments:**
///
/// ```text
/// uncommitRoute:         addGraphEdge(p0,  p1,  scale, false, false);   // always restored
/// removeTerminalAccess:  addGraphEdge(pt0, pt1, scale, true,  true);    // CHECKED
/// ```
///
/// So an edge that was removed to make room for a terminal's access is put back only if it is
/// still clear — of obstructions, and of every committed route. One that has come to conflict is
/// **never restored**, and the graph loses that resource for the rest of the run.
///
/// ⚠️ `check_routes` reaches `RDLSegment::isIntersecting(line, 0)`, which is the line against each
/// route vertex, not against the whole corridor.
pub fn undo_access(graph: &mut Graph, undo: &Undo, blocked_edge: &dyn Fn(Point, Point) -> bool) {
    for &(a, b) in &undo.cut {
        graph.cut(a, b);
    }
    for &(a, b, scale) in &undo.restore {
        let (pa, pb) = (graph.points[a], graph.points[b]);
        if blocked_edge(pa, pb) {
            continue;
        }
        let w = edge_weight(pa, pb, scale);
        graph.join(a, b, w);
    }
}

/// **L8** — is this access point too close to a route already committed?
///
/// Compares the box of `(width + spacing) / 2` around the point against the same-sized box around
/// every point of every committed route.
///
/// ⚠️ Access points are **re-filtered before each attempt**, not computed once. A terminal's
/// approaches are progressively closed off as wires are laid near it, and a router that keeps its
/// original set will find paths that run over its neighbours — succeeding where it should have
/// been forced to try elsewhere.
pub fn access_blocked(point: Point, committed: &[&[Point]], width: i32, spacing: i32) -> bool {
    let e = (width + spacing) / 2;
    let near = (point.0 - e, point.1 - e, point.0 + e, point.1 + e);
    committed.iter().any(|route| {
        route
            .iter()
            .any(|&p| hits((p.0 - e, p.1 - e), (p.0 + e, p.1 + e), near))
    })
}

/// **L7** — run every route to completion, rearranging when one cannot get through.
///
/// The queue is drained in [`Route::precedes`] order. A route that connects is committed and its
/// corridor removed from the graph; one that does not is put back with its next destination. When
/// the queue empties, whatever failed raises its priority, displaces whatever crosses its probe,
/// and everything goes round again.
///
/// ⚠️ Two guards stop it thrashing, and both are the reference's: it gives up after
/// `max_iterations`, and a round that routed **nothing new** rips nothing up — repeating a round
/// that achieved nothing cannot achieve anything the second time either.
#[allow(clippy::too_many_arguments)]
pub fn route_all(
    graph: &mut Graph,
    grid: &Grid,
    routes: &mut [Route],
    // 🔑 Every target of a terminal, not just one: `(net, [(centre, access points)])`. The
    // reference keeps a vector per iterm and searches the **cross product** of the two ends.
    access: &std::collections::HashMap<String, (String, Vec<(Point, Vec<Point>)>)>,
    width: i32,
    spacing: i32,
    turn_penalty: f32,
    max_iterations: i32,
    rebuild: Option<&[(Point, Point)]>,
    allow45: bool,
    obstructed: &dyn Fn(Point, Point) -> bool,
) -> Routed {
    let mut out = Routed::default();
    let mut committed: Vec<Option<Undo>> = vec![None; routes.len()];
    // ⛔ **Upstream seeds the queue with `if (!segment->isRouted()) route_queue.push(...)`.**
    // `preprocess` runs before this and marks a segment routed when its terminals already touch,
    // so such a segment is never attempted — and never reported failed either.
    let mut queue: Vec<usize> = (0..routes.len()).filter(|&i| !routes[i].routed).collect();
    let mut last_done: std::collections::BTreeSet<String> = Default::default();

    loop {
        queue.sort_by(|&a, &b| routes[a].precedes(&routes[b]));
        while let Some(i) = queue.first().copied() {
            queue.remove(0);
            routes[i].pending = false;
            if !routes[i].has_next() {
                continue;
            }
            // **L9** — skip destinations that are already served.
            //
            // ⚠️ A **pad** already reached by another route is skipped: a pad needs one connection,
            // and routing to it twice wastes the attempt and blocks the corridor for whoever still
            // needs it. So is the reverse of a pair already routed — that wire exists, drawn from
            // the other end. Bumps are *not* skipped this way: several may share a net.
            let d = loop {
                if !routes[i].has_next() {
                    break None;
                }
                let cand = routes[i].dests[routes[i].next].clone();
                routes[i].next += 1;
                // ⛔ **A segment `preprocess` LOCKED does NOT count as served, and it is worth
                // saying why the opposite is tempting.** `preprocess` ends in `setRouted()`, which
                // sets the flag and recomputes the bbox and nothing else — `net_->updateRoute` is
                // reached only from `setRoute` and `resetRoute`. So a locked segment never enters
                // `routed_pairs_` or `routed_noncover_terminals_`, and other bumps go on offering
                // its pad as a destination.
                //
                // ⚠️ Measured on `_overlapping_iterms`: counting a locked segment as served makes
                // us skip **nine** attempts at `u_v18_25/DVDD` that the reference makes, and it
                // *improves* the wire count while doing it. A rule that scores better and is not
                // the reference's is the worst kind, because everything downstream is then
                // attributed to it.
                let served = !cand.cover
                    && routes.iter().any(|r| {
                        r.routed && r.points.len() > 1 && r.dests.get(r.next.saturating_sub(1))
                            .is_some_and(|x| x.terminal == cand.terminal)
                    });
                // ⛔ **`net_->isRouted(iterm_, dst)` is TRANSITIVE.** An earlier version tested
                // only the direct reverse pair, which misses a destination this segment can
                // already reach through a chain of other committed routes — and the reference
                // skips those, so we would route a connection it considers already made.
                let adj = routed_pairs(routes);
                let already = is_routed(&adj, &routes[i].source, &cand.terminal);
                if !served && !already {
                    break Some(cand);
                }
            };
            let Some(d) = d else { continue };

            let (Some(src), Some(dst)) = (access.get(&routes[i].source), access.get(&d.terminal))
            else {
                continue;
            };
            out.attempts += 1;
            // 🔑 **Every pairing of the two terminals' targets, in distance order.** A pad with
            // several pin shapes — or one polygon pin, which yields a target per flat side — has
            // more than one place a wire may land, and the reference tries them all: the cross
            // product, `stable_sort`ed by centre-to-centre distance and then by the four centre
            // coordinates, taking the FIRST pair that routes.
            //
            // ⛔ Collapsing a terminal to one target makes this loop a single attempt. It is
            // invisible on a single-shape pin and decides the result on a bump pad.
            let src_centres: Vec<Point> = src.1.iter().map(|t| t.0).collect();
            let dst_centres: Vec<Point> = dst.1.iter().map(|t| t.0).collect();
            let pairs = target_pairs(&src_centres, &dst_centres);

            let mut laid_path: Option<(Point, Point, Vec<Point>)> = None;
            for (si, di) in pairs {
                let (s_centre, s_snaps) = &src.1[si];
                let (d_centre, d_snaps) = &dst.1[di];

                // ⚠️ An experiment, off by default: rebuild the graph from the original edge list
                // and re-apply every committed corridor, so a vertex's edges sit in build order
                // rather than in whatever order removing and restoring them has left. If the two
                // runs differ, the accumulated order is what decides the remaining equal-cost
                // choices.
                if let Some(base) = rebuild {
                    *graph = Graph::build(grid, base, 1.0);
                    let laid_paths: Vec<Vec<Point>> =
                        routes.iter().filter(|r| r.routed).map(|r| r.points.clone()).collect();
                    for pts in &laid_paths {
                        commit_route(graph, pts, width, spacing, allow45);
                    }
                }
                // ⚠️ Filtered against what is already on the die, **per attempt** — the reference
                // calls `insertTerminalAccess` inside this loop, not once before it.
                let laid: Vec<&[Point]> =
                    routes.iter().filter(|r| r.routed).map(|r| r.points.as_slice()).collect();
                let open = |snaps: &[Point]| -> Vec<Point> {
                    snaps
                        .iter()
                        .copied()
                        .filter(|&p| !access_blocked(p, &laid, width, spacing))
                        .collect()
                };
                let (src_open, dst_open) = (open(s_snaps), open(d_snaps));
                let a = insert_access(graph, grid, *s_centre, &src_open, allow45, obstructed);
                let b = insert_access(graph, grid, *d_centre, &dst_open, allow45, obstructed);
                let path = shortest_path(graph, *s_centre, *d_centre, turn_penalty);
                out.log.push((
                    routes[i].source.clone(),
                    d.terminal.clone(),
                    *s_centre,
                    *d_centre,
                    path.len(),
                ));

                if path.is_empty() {
                    graph.undo(&b);
                    graph.undo(&a);
                    continue;
                }
                // ⚠️ The corridor is taken out **while the terminal access is still grafted on**,
                // and only then is the access removed. The route's own access points are route
                // vertices, so committing first removes the edges around them too; undoing the
                // access first leaves those edges in place and the next route may run straight
                // past a terminal another route is already using.
                committed[i] = Some(commit_route(graph, &path, width, spacing, allow45));
                // ⛔ **Terminal access is undone with the CHECKS ON** — see `undo_access`. The
                // route just committed is part of what those checks see, which is why this runs
                // AFTER `commit_route`.
                let settled: Vec<&[Point]> =
                    routes.iter().filter(|r| r.routed).map(|r| r.points.as_slice()).collect();
                let mut settled = settled;
                settled.push(path.as_slice());
                let checked = |p0: Point, p1: Point| -> bool {
                    obstructed(p0, p1)
                        || settled.iter().any(|pts| pts.iter().any(|&pt| point_on_segment(pt, p0, p1)))
                };
                undo_access(graph, &b, &checked);
                undo_access(graph, &a, &checked);
                laid_path = Some((*s_centre, *d_centre, path));
                break;
            }

            let Some((_, _, path)) = laid_path else {
                if routes[i].has_next() {
                    queue.push(i);
                }
                continue;
            };
            routes[i].routed = true;
            routes[i].points = path.clone();
            let _ = &d;
        }

        out.iterations += 1;
        if out.iterations > max_iterations {
            break;
        }
        let done: std::collections::BTreeSet<String> =
            routes.iter().filter(|r| r.routed).map(|r| r.source.clone()).collect();
        if done == last_done {
            break;
        }
        last_done = done;

        let failed = failed_routes(routes);
        if failed.is_empty() {
            break;
        }
        // ⚠️ Choose everything to displace BEFORE displacing anything: ripping up as we go would
        // let a later failure probe a graph the earlier ones have already changed, so which routes
        // move would depend on the order the failures happen to be listed in.
        let mut ripped: std::collections::BTreeSet<usize> = Default::default();
        for &f in &failed {
            for t in ripup_targets(&routes[f], routes, spacing + width) {
                ripped.insert(t);
            }
        }
        for &t in &ripped {
            if let Some(u) = committed[t].take() {
                graph.undo(&u);
            }
            routes[t].routed = false;
            routes[t].points.clear();
            routes[t].next = 0;
            routes[t].pending = true;
            queue.push(t);
        }
        for &f in &failed {
            routes[f].priority += 1;
            routes[f].next = 0;
            routes[f].pending = true;
            queue.push(f);
        }
        if queue.is_empty() {
            break;
        }
    }
    // 🔑 **Written in the order the segments were DECLARED, not the order they routed.**
    // Nothing upstream ever reorders `RDLNet::segments_`: the priority queue decides *when* a
    // segment is attempted, and `writeToDb` is then reached by walking `routes_` and each net's
    // `getSegments()` in the order `buildIntialRouteSet` added them. Emitting as routes complete
    // gives the same wires in a different sequence — an identical set, resequenced.
    //
    // ⚠️ Only meaningful together with the declaration order itself: the callers build one route
    // per COVER terminal in ascending iterm id, which is what `odb::PtrMap` iteration gives.
    //
    // ⚠️ The destination is the one at `next - 1`. The candidate loop advances `next` past every
    // destination it skipped, so the last one it stepped over is the one that routed — the same
    // index the served/reversed checks read.
    out.paths = routes
        .iter()
        // ⚠️ A LOCKED segment is `routed` and has a destination recorded, but it committed no
        // path. It must not appear here: the writer walks `routes` and `paths` in step, so an
        // entry with no wires would leave the two out of alignment for everything after it.
        .filter(|r| r.routed && !r.locked)
        .filter_map(|r| {
            let d = r.dests.get(r.next.checked_sub(1)?)?;
            let net = access.get(&r.source)?.0.clone();
            Some((net, r.source.clone(), d.terminal.clone(), r.points.clone()))
        })
        .collect();
    out.failed = routes.iter().filter(|r| !r.routed).map(|r| r.source.clone()).collect();
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

    fn dest(name: &str, inst: &str, c: Point, cover: bool, id: u64) -> Dest {
        Dest { terminal: name.into(), instance: inst.into(), centre: c, cover, id }
    }

    fn route(c: Point, id: u64, dests: Vec<Dest>) -> Route {
        Route {
            source: "BUMP/PAD".into(),
            instance: "BUMP".into(),
            centre: c,
            id,
            dests,
            next: 0,
            priority: 0,
            routed: false,
            pending: true,
            points: Vec::new(),
            locked: false,
            stubs: Vec::new(),
        }
    }

    #[test]
    fn an_access_point_next_to_a_committed_route_is_closed_off() {
        // ⚠️ Recomputed before every attempt: a terminal's approaches close as wires are laid near
        // it, and keeping the original set finds paths that run over the neighbours.
        let laid = [(1000, 1000), (1000, 1100)];
        let routes: Vec<&[Point]> = vec![&laid];
        assert!(access_blocked((1000, 1050), &routes, 80, 80), "right on top of it");
        assert!(access_blocked((1080, 1000), &routes, 80, 80), "within a width and a spacing");
        assert!(!access_blocked((1400, 1000), &routes, 80, 80), "far enough away");
        assert!(!access_blocked((1000, 1050), &[], 80, 80), "nothing committed yet");
    }

    #[test]
    fn a_route_has_failed_only_when_it_is_off_the_queue_and_out_of_options() {
        let mut r = route((0, 0), 1, vec![dest("p", "P", (10, 0), false, 1)]);
        assert!(!is_failed(&r), "still on the queue");
        r.pending = false;
        assert!(!is_failed(&r), "still has a destination to try");
        r.next = 1;
        assert!(is_failed(&r));
        r.routed = true;
        assert!(!is_failed(&r), "routed is not failed");
    }

    #[test]
    fn a_bump_reached_by_another_route_is_not_a_failure() {
        // ⚠️ Counting it would rip up working routes to retry something already connected.
        let mut mine = route((0, 0), 1, vec![]);
        mine.source = "BUMP_A/PAD".into();
        mine.pending = false;
        let mut other = route((0, 0), 2, vec![dest("BUMP_A/PAD", "BUMP_A", (5, 5), true, 3)]);
        other.routed = true;
        assert!(is_failed(&mine));
        assert!(failed_routes(&[mine.clone(), other]).is_empty(), "already connected");
        assert_eq!(failed_routes(&[mine]).len(), 1, "nobody else reached it");
    }

    #[test]
    fn the_probe_rotates_its_destination_and_widens_each_failure() {
        // ⚠️ The convergence mechanism. Fixing either half makes a congested design loop.
        let mut r = route(
            (0, 0),
            1,
            vec![dest("a", "A", (100, 0), false, 1), dest("b", "B", (0, 100), false, 2)],
        );
        let (_, first, m0) = ripup_probe(&r, 10).unwrap();
        assert_eq!((first, m0), ((100, 0), 10));
        r.priority = 1;
        let (_, second, m1) = ripup_probe(&r, 10).unwrap();
        assert_eq!(second, (0, 100), "a different corridor is probed");
        assert_eq!(m1, 20, "and a wider one");
        r.priority = 2;
        assert_eq!(ripup_probe(&r, 10).unwrap().1, (100, 0), "wraps around");
    }

    #[test]
    fn a_route_may_displace_another_of_equal_priority() {
        // ⚠️ `>=`, not `>`: without it a design where everything has failed once cannot rearrange.
        let mut settled = route((0, 0), 1, vec![]);
        settled.routed = true;
        settled.priority = 2;
        assert!(allows_ripup(&settled, 2));
        assert!(allows_ripup(&settled, 3));
        assert!(!allows_ripup(&settled, 1), "a lower priority cannot displace it");
        settled.routed = false;
        assert!(!allows_ripup(&settled, 5), "an unrouted route has nothing to displace");
    }

    #[test]
    fn only_routes_crossing_the_probe_are_displaced() {
        let failing = route((0, 0), 1, vec![dest("a", "A", (1000, 0), false, 1)]);
        let mut across = route((0, 0), 2, vec![]);
        across.routed = true;
        across.points = vec![(500, 0)];
        let mut clear = route((0, 0), 3, vec![]);
        clear.routed = true;
        clear.points = vec![(500, 10_000)];
        let hit = ripup_targets(&failing, &[across, clear], 10);
        assert_eq!(hit, vec![0], "only the one in the way");
    }

    #[test]
    fn a_pad_is_preferred_over_a_nearer_bump() {
        // ⚠️ The rule that stops bumps chaining to each other and leaving pads unreached.
        let ds = vec![
            dest("B/PAD", "B", (10, 0), true, 1),
            dest("P/PAD", "P", (1000, 0), false, 2),
        ];
        let ordered = order_dests("BUMP", (0, 0), &ds);
        assert_eq!(ordered[0].terminal, "P/PAD", "the far pad comes before the near bump");
    }

    #[test]
    fn destinations_on_the_source_instance_are_dropped() {
        let ds = vec![dest("BUMP/OTHER", "BUMP", (10, 0), true, 1)];
        assert!(order_dests("BUMP", (0, 0), &ds).is_empty(), "a bump does not route to itself");
    }

    #[test]
    fn equal_kind_destinations_go_nearest_first_then_by_id() {
        let ds = vec![
            dest("c", "C", (30, 0), false, 3),
            dest("a", "A", (10, 0), false, 9),
            dest("b", "B", (10, 0), false, 4),
        ];
        let ordered = order_dests("S", (0, 0), &ds);
        let names: Vec<&str> = ordered.iter().map(|d| d.terminal.as_str()).collect();
        assert_eq!(names, vec!["b", "a", "c"], "distance first, then id");
    }

    #[test]
    fn a_ripped_up_route_is_retried_before_anything_new() {
        let far = route((0, 0), 1, vec![dest("p", "P", (10_000, 0), false, 1)]);
        let mut near = route((0, 0), 2, vec![dest("q", "Q", (10, 0), false, 2)]);
        // Normally the near one wins.
        assert_eq!(near.precedes(&far), std::cmp::Ordering::Less);
        // ⚠️ Priority overrides distance entirely, which is what makes rip-up converge.
        near.priority = 0;
        let mut bumped = far.clone();
        bumped.priority = 1;
        assert_eq!(bumped.precedes(&near), std::cmp::Ordering::Less, "priority wins");
    }

    #[test]
    fn a_route_with_nothing_left_to_try_is_ordered_by_priority_alone() {
        // ⚠️ Reading a distance here would run past the end of the destination list.
        let empty = route((0, 0), 1, vec![]);
        let other = route((0, 0), 2, vec![dest("p", "P", (10, 0), false, 1)]);
        assert_eq!(empty.precedes(&other), std::cmp::Ordering::Equal);
    }

    #[test]
    fn a_committed_route_clears_a_corridor_around_itself() {
        let boxes = commit_corridor(&[(100, 100)], 80, 20);
        // ⚠️ half-width + spacing + 1 = 61, not 60: one unit clear, not exactly touching.
        assert_eq!(boxes, vec![(39, 39, 161, 161)]);
    }

    #[test]
    fn a_straight_run_collapses_to_one_segment() {
        let path = [(0, 0), (0, 100), (0, 200), (0, 300)];
        assert_eq!(simplify(&path, 80), vec![((0, 0), (0, 300))]);
    }

    #[test]
    fn a_corner_is_mitred_by_half_the_wire_width() {
        // ⚠️ Both runs reach past the corner. Without it the rectangles meet corner to corner and
        // leave a notch of bare substrate on the inside of the bend.
        let path = [(0, 0), (0, 100), (100, 100)];
        let runs = simplify(&path, 80);
        assert_eq!(runs.len(), 2);
        assert_eq!(runs[0], ((0, 0), (0, 140)), "the vertical run overshoots by 40");
        assert_eq!(runs[1], ((-40, 100), (100, 100)), "the horizontal starts 40 back");
    }

    #[test]
    fn a_diagonal_is_kept_as_a_centre_line() {
        let path = [(0, 0), (100, 100)];
        let w = wires(&path, 80, (0, 0, 0, 0), (0, 0, 0, 0));
        assert_eq!(w, vec![Wire::Diagonal((0, 0), (100, 100), 80)]);
    }

    #[test]
    fn a_run_is_widened_sideways_only() {
        assert_eq!(run_rect((0, 0), (0, 300), 80), (-40, 0, 40, 300));
        assert_eq!(run_rect((0, 0), (300, 0), 80), (0, -40, 300, 40));
        // ⚠️ Not bloated along its own length: a wire is as long as its path, no longer.
    }

    #[test]
    fn an_end_run_reaches_into_a_pad_only_when_it_is_wider_than_it() {
        let run = (0, -40, 300, 40); // 80 across
        // A wider pad already covers it: leave the run alone.
        assert_eq!(correct_end(run, true, (280, -100, 340, 100)), run);
        // A narrower pad does not: stretch to cover it.
        assert_eq!(
            correct_end(run, true, (280, -20, 340, 20)),
            (0, -40, 340, 40),
            "reaches the pad's far edge"
        );
    }

    #[test]
    fn a_path_is_found_across_a_small_grid() {
        let coords: Vec<i32> = (0..5).map(|i| 10 + i * 100).collect();
        let g = grid(&coords, &coords, 4, 4);
        let graph = Graph::build(&g, &edges(&g, false), 1.0);
        let path = shortest_path(&graph, (10, 10), (410, 410), 2.0);
        assert!(!path.is_empty(), "a clear grid must be crossable");
        assert_eq!(path.first(), Some(&(10, 10)));
        assert_eq!(path.last(), Some(&(410, 410)));
        // Monotone staircase: 4 steps in each axis, so 9 points.
        assert_eq!(path.len(), 9, "no wandering");
    }

    #[test]
    fn a_path_routes_around_a_wall() {
        let coords: Vec<i32> = (0..5).map(|i| 10 + i * 100).collect();
        let g = grid(&coords, &coords, 4, 4);
        // A wall across the middle column, open at the top.
        let wall = [Obstacle::Rect((150, 0, 170, 350))];
        let e = edges_clear(&g, false, &|a, b| blocked(a, b, &wall));
        let graph = Graph::build(&g, &e, 1.0);
        let path = shortest_path(&graph, (10, 10), (410, 10), 2.0);
        assert!(!path.is_empty(), "must go around");
        assert!(path.iter().any(|p| p.1 >= 310), "detoured over the wall");
    }

    #[test]
    fn an_unreachable_goal_gives_no_path() {
        let coords: Vec<i32> = (0..5).map(|i| 10 + i * 100).collect();
        let g = grid(&coords, &coords, 4, 4);
        let wall = [Obstacle::Rect((150, -100, 170, 1000))];
        let e = edges_clear(&g, false, &|a, b| blocked(a, b, &wall));
        let graph = Graph::build(&g, &e, 1.0);
        assert!(shortest_path(&graph, (10, 10), (410, 10), 2.0).is_empty());
    }

    #[test]
    fn the_nearest_track_on_each_side_is_taken() {
        let axis = [0, 10, 20, 30, 40];
        assert_eq!(nearest_tracks(&axis, 25, &|_| true), vec![20, 30]);
        // ⚠️ Strictly either side: a target sitting exactly on a track does not use it.
        assert_eq!(nearest_tracks(&axis, 20, &|_| true), vec![10, 30]);
        assert_eq!(nearest_tracks(&axis, -5, &|_| true), vec![0], "nothing below");
        assert_eq!(nearest_tracks(&axis, 100, &|_| true), vec![40], "nothing above");
    }

    #[test]
    fn a_blocked_track_is_passed_over_for_the_next_one() {
        let axis = [0, 10, 20, 30, 40];
        assert_eq!(nearest_tracks(&axis, 25, &|c| c != 20 && c != 30), vec![10, 40]);
    }

    #[test]
    fn a_target_reaches_the_grid_on_four_sides() {
        let g = Grid { x: vec![0, 100, 200], y: vec![0, 100, 200] };
        let t = Target {
            terminal: "u/PAD".into(),
            centre: (150, 150),
            shape: (140, 140, 160, 160),
            access: vec![],
            layer: String::new(),
        };
        let mut pts = access_points(&g, &t, &[], &[]);
        pts.sort_unstable();
        assert_eq!(pts, vec![(100, 150), (150, 100), (150, 200), (200, 150)]);
    }

    #[test]
    fn a_targets_own_metal_does_not_block_the_line_to_its_access() {
        // ⚠️ The target sits inside its own pin, so every access line starts inside it. Counting
        // that as a violation leaves the terminal with no way in at all.
        let g = Grid { x: vec![0, 100, 200], y: vec![0, 100, 200] };
        let own = (140, 140, 160, 160);
        let t = Target { terminal: "u/PAD".into(), centre: (150, 150), shape: own, access: vec![], layer: String::new() };
        assert_eq!(access_points(&g, &t, &[Obstacle::Rect(own)], &[Obstacle::Rect(own)]).len(), 4);
        assert!(access_points(&g, &t, &[Obstacle::Rect(own)], &[]).is_empty(), "not excused, none survive");
    }

    #[test]
    fn a_candidate_track_inside_any_obstruction_is_rejected_even_the_targets_own() {
        // ⚠️ The asymmetry that matters: the exemption applies to the LINE, never to the
        // candidate. A track inside the terminal's own pad is dead grid — every edge there was
        // filtered out — so accepting it yields access points that reach nothing.
        let g = Grid { x: vec![0, 100, 200, 300], y: vec![150] };
        let own = (90, 140, 210, 160);
        let t = Target { terminal: "u/PAD".into(), centre: (150, 150), shape: own, access: vec![], layer: String::new() };
        let pts = access_points(&g, &t, &[Obstacle::Rect(own)], &[Obstacle::Rect(own)]);
        assert!(!pts.contains(&(100, 150)), "inside the pad, rejected as a candidate");
        assert!(!pts.contains(&(200, 150)), "likewise");
        assert!(pts.contains(&(0, 150)) && pts.contains(&(300, 150)), "the live tracks outside");
    }

    #[test]
    /// ⛔ **The acute prune** — `insertTerminalAccess` under `allow45`. A diagonal leaving a grid
    /// point in the SAME direction the access edge arrives from is a turn a wire cannot make, so
    /// the reference deletes it from the graph before any search runs.
    ///
    /// ⚠️ The axis test is deliberately asymmetric: `snap_dy == 0` compares x, and everything else
    /// compares y. Right angles are kept, and the edge just added to the snap is skipped.
    fn an_acute_diagonal_out_of_an_access_point_is_pruned() {
        // A 5x5 grid at 100 pitch; diagonals exist only where both indices are even.
        let g = Grid { x: vec![0, 100, 200, 300, 400], y: vec![0, 100, 200, 300, 400] };
        let build = || Graph::build(&g, &edges(&g, true), 1.0);

        // ⚠️ The snap must sit BETWEEN two columns so its neighbours are x-neighbours, and one
        // of them must be at an even/even index or it has no diagonals to prune at all. (200,200)
        // is i=2, j=2; (200,100) — which a snap ON the column would reach — is j=1 and has none.
        let centre = (150, 250);
        let snap = (150, 200);

        let mut kept = build();
        let before = kept.adj[kept.index[&(200, 200)]].len();
        insert_access(&mut kept, &g, centre, &[snap], false, &|_, _| false);
        let without_prune = kept.adj[kept.index[&(200, 200)]].len();

        let mut pruned = build();
        insert_access(&mut pruned, &g, centre, &[snap], true, &|_, _| false);
        let with_prune = pruned.adj[pruned.index[&(200, 200)]].len();

        assert!(before > 0, "the fixture has edges to prune");
        assert!(
            with_prune < without_prune,
            "allow45 must remove edges: {without_prune} without the prune, {with_prune} with it"
        );

        // ⚠️ A RIGHT-ANGLE neighbour is never pruned — (200,200)-(200,300) shares an x.
        let v = pruned.index[&(200, 200)];
        let right_angle = pruned.index.get(&(200, 300)).copied();
        if let Some(ra) = right_angle {
            assert!(
                pruned.adj[v].iter().any(|&(o, _)| o == ra),
                "a right-angle edge must survive the acute prune"
            );
        }

        // ⛔ **The OTHER axis branch.** Above, `snap_dy == 0` and only the x comparison runs. A
        // snap between two ROWS gives `snap_dy != 0`, which the reference settles on **y** — and
        // the two branches select different edges, so a version that always compares x is a
        // different router. Without this case that asymmetry is unwitnessed: a mutation replacing
        // the whole test with the x rule passed every other test in this suite.
        let (centre_y, snap_y) = ((250, 150), (200, 150));
        let mut y_kept = build();
        insert_access(&mut y_kept, &g, centre_y, &[snap_y], false, &|_, _| false);
        let y_without = y_kept.adj[y_kept.index[&(200, 200)]].len();

        let mut y_pruned = build();
        insert_access(&mut y_pruned, &g, centre_y, &[snap_y], true, &|_, _| false);
        let y_with = y_pruned.adj[y_pruned.index[&(200, 200)]].len();
        assert!(
            y_with < y_without,
            "the y branch must prune too: {y_without} without, {y_with} with"
        );

        // The two branches must not pick the same survivors — that is what makes the asymmetry
        // observable rather than decorative.
        let surv = |gr: &Graph| -> Vec<Point> {
            let vtx = gr.index[&(200, 200)];
            let mut v: Vec<Point> = gr.adj[vtx].iter().map(|&(o, _)| gr.points[o]).collect();
            v.sort_unstable();
            v
        };
        assert_ne!(
            surv(&pruned),
            surv(&y_pruned),
            "the x branch and the y branch must remove different edges"
        );
    }

    #[test]
    fn an_obstruction_in_the_way_removes_that_access_point() {
        let g = Grid { x: vec![0, 100, 200], y: vec![0, 100, 200] };
        let t = Target {
            terminal: "u/PAD".into(),
            centre: (150, 150),
            shape: (140, 140, 160, 160),
            access: vec![],
            layer: String::new(),
        };
        // A wall just left of the centre blocks the westward access only.
        let wall = [Obstacle::Rect((120, 0, 130, 300))];
        let pts = access_points(&g, &t, &wall, &[]);
        assert!(!pts.contains(&(100, 150)), "west is blocked");
        assert!(pts.contains(&(200, 150)), "east is not");
    }

    #[test]
    fn a_segment_touching_an_obstruction_is_blocked() {
        let r = (100, 100, 200, 200);
        assert!(hits((0, 150), (150, 150), r), "runs into it");
        assert!(hits((0, 150), (100, 150), r), "⚠️ touches the edge");
        assert!(hits((0, 0), (100, 100), r), "⚠️ touches a corner");
        assert!(!hits((0, 150), (99, 150), r), "stops one unit short");
        assert!(!hits((0, 0), (99, 99), r));
    }

    #[test]
    fn a_diagonal_passing_beside_a_rectangle_is_not_blocked() {
        // ⚠️ Their bounding boxes overlap, so the box test alone would call this blocked.
        let r = (100, 100, 200, 200);
        assert!(!hits((0, 300), (300, 500), r), "passes above");
        assert!(hits((0, 100), (300, 300), r), "passes through");
    }

    #[test]
    fn obstructed_edges_are_dropped_and_vertices_are_kept() {
        let coords: Vec<i32> = (0..4).map(|i| 10 + i * 20).collect();
        let g = grid(&coords, &coords, 4, 4);
        let all = edges(&g, false).len();
        let blocker = [Obstacle::Rect((25, 0, 35, 1000))];
        let some = edges_clear(&g, false, &|a, b| blocked(a, b, &blocker)).len();
        assert!(some < all, "a wall removes edges");
        assert_eq!(g.vertices(), 16, "and removes no vertices");
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

#[cfg(test)]
mod oct_commit_tests {
    use super::*;

    // 🔑 Upstream rule (`RDLRouter::commitRoute`, third `allow45` block):
    //     `for (std::size_t i = 2; i < route.size(); i++)` — the octagon of the route's FIRST
    // segment is never applied. Reproduced rather than corrected: starting at 1 removes edges the
    // reference keeps, and the router then fails to find paths it should find.
    //
    // Route (0,0) -> (100,100) -> (200,0), width 20, spacing 0, so d = 20/2 + 0 + 1 = 11.
    //   segment 1 octagon spans x -11..111 — would swallow the probe at (40,60)-(60,40)
    //   segment 2 octagon spans x  89..211 — swallows the probe at (140,60)-(160,40)
    // Both probes sit clear of every per-vertex corridor square (±11, widened by the 20-unit edge
    // span to ±31), so steps 1 and 2 cannot account for either verdict.
    #[test]
    fn the_first_segments_octagon_is_never_applied() {
        let grid = Grid { x: vec![40, 60, 140, 160], y: vec![40, 60] };
        let probe1 = ((40, 60), (60, 40));
        let probe2 = ((140, 60), (160, 40));
        let mut graph = Graph::build(&grid, &[probe1, probe2], 1.0);
        let (a1, b1) = (graph.index[&probe1.0], graph.index[&probe1.1]);
        let (a2, b2) = (graph.index[&probe2.0], graph.index[&probe2.1]);
        assert!(graph.weight_between(a1, b1).is_some() && graph.weight_between(a2, b2).is_some());

        commit_route(&mut graph, &[(0, 0), (100, 100), (200, 0)], 20, 0, true);

        assert!(
            graph.weight_between(a1, b1).is_some(),
            "the first segment is skipped, so its octagon must not cut this edge"
        );
        assert!(
            graph.weight_between(a2, b2).is_none(),
            "the second segment IS checked, so its octagon must cut this edge"
        );
    }

    // 🔑 `is45DegreeEdge` is simply "not axis-aligned", so an orthogonal segment contributes NO
    // octagon. It would contribute a large one if the guard were dropped: `Oct` of a horizontal
    // (100,100)-(200,100) at dist 11 covers x 89..211, y 89..111, which swallows this probe.
    #[test]
    fn an_axis_aligned_segment_contributes_no_octagon() {
        let grid = Grid { x: vec![150, 160], y: vec![95, 105] };
        let probe = ((150, 95), (160, 105));
        let mut graph = Graph::build(&grid, &[probe], 1.0);
        let (a, b) = (graph.index[&probe.0], graph.index[&probe.1]);
        commit_route(&mut graph, &[(0, 0), (100, 100), (200, 100)], 20, 0, true);
        assert!(
            graph.weight_between(a, b).is_some(),
            "the horizontal second segment must not build an octagon"
        );
    }

    // ⚠️ The octagon is built at `dist = width/2 + spacing + 1`, the same `d` as the corridor
    // squares — not at half-width, and not at width. For the (100,100)-(200,0) segment that puts
    // the long sides on `x + y = 178` and `x + y = 222`; at `d/2` they would be 190 and 210. This
    // probe sits on `x + y = 215`: inside at the right `dist`, outside at any smaller one.
    #[test]
    fn the_octagon_is_built_at_the_corridor_distance() {
        let grid = Grid { x: vec![160, 165], y: vec![50, 55] };
        let probe = ((160, 55), (165, 50));
        let mut graph = Graph::build(&grid, &[probe], 1.0);
        let (a, b) = (graph.index[&probe.0], graph.index[&probe.1]);
        commit_route(&mut graph, &[(0, 0), (100, 100), (200, 0)], 20, 0, true);
        assert!(
            graph.weight_between(a, b).is_none(),
            "x + y = 215 lies within the 178..222 band the full-distance octagon covers"
        );
    }

    // …and the band's far edge, so a too-WIDE octagon is caught too: `x + y = 235` is outside the
    // 178..222 band at the right `dist`, but inside the 156..244 band `2 * dist` would give.
    #[test]
    fn the_octagon_is_no_wider_than_the_corridor_distance() {
        let grid = Grid { x: vec![170, 175], y: vec![60, 65] };
        let probe = ((170, 65), (175, 60));
        let mut graph = Graph::build(&grid, &[probe], 1.0);
        let (a, b) = (graph.index[&probe.0], graph.index[&probe.1]);
        commit_route(&mut graph, &[(0, 0), (100, 100), (200, 0)], 20, 0, true);
        assert!(graph.weight_between(a, b).is_some(), "x + y = 235 is clear of the octagon");
    }

    // The same route without `allow45` leaves both probes alone — proof the verdict above comes
    // from the octagon block and not from the corridor squares.
    #[test]
    fn without_allow45_no_octagon_is_applied_at_all() {
        let grid = Grid { x: vec![40, 60, 140, 160], y: vec![40, 60] };
        let (probe1, probe2) = (((40, 60), (60, 40)), ((140, 60), (160, 40)));
        let mut graph = Graph::build(&grid, &[probe1, probe2], 1.0);
        let (a2, b2) = (graph.index[&probe2.0], graph.index[&probe2.1]);
        commit_route(&mut graph, &[(0, 0), (100, 100), (200, 0)], 20, 0, false);
        assert!(graph.weight_between(a2, b2).is_some());
    }
}

#[cfg(test)]
mod oct_tests {
    use super::*;

    // 🔑 Upstream rule (`odb::Oct::init` + `RDLSegment::getEdgeObstruction`): the octagon swept by
    // a 45° wire of half-width `dist`. Values below are hand-computed from that arithmetic, NOT
    // read back from this implementation.
    //
    //   A = (2*dist)/2 = dist = 10;  B = ceil(20 / sqrt2) - 10 = ceil(14.142…) - 10 = 15 - 10 = 5
    #[test]
    fn a_rising_diagonal_sweeps_the_right_handed_octagon() {
        let ring = edge_obstruction((0, 0), (100, 100), 10);
        assert_eq!(
            ring,
            vec![
                (-5, -10),   // p0 — ⚠️ NOT re-closed on the RIGHT branch, so it keeps `low.x - B`
                (10, -10),   // p1, x moved to low.x + dist
                (110, 90),   // p2, y moved to high.y - dist
                (110, 105),  // p3
                (105, 110),  // p4
                (90, 110),   // p5, x moved to high.x - dist
                (-10, 10),   // p6, y moved to low.y + dist
                (-10, -5),   // p7
                (-5, -10),   // p8 == p0
            ]
        );
    }

    // A falling diagonal takes the LEFT branch — and the LEFT branch is the only one that
    // reassigns p0 from the mutated p8. Dropping that reassignment leaves a ring whose first and
    // last points disagree.
    #[test]
    fn a_falling_diagonal_sweeps_the_left_handed_octagon_and_recloses_the_ring() {
        let ring = edge_obstruction((100, 0), (0, 100), 10);
        assert_eq!(
            ring,
            vec![
                (90, -10),   // p0 — reassigned from p8 AFTER p8.x moved to low.x - dist
                (105, -10),  // p1
                (110, -5),   // p2
                (110, 10),   // p3, y moved to low.y + dist
                (10, 110),   // p4, x moved to high.x + dist
                (-5, 110),   // p5
                (-10, 105),  // p6
                (-10, 90),   // p7, y moved to high.y - dist
                (90, -10),   // p8
            ]
        );
        assert_eq!(ring[0], ring[8], "the ring must close");
    }

    // ⚠️ `A = width / 2` is INTEGER division and `B`'s ceil happens in f64 then truncates.
    // dist = 3: A = 3, B = ceil(6/sqrt2) - 3 = ceil(4.2426…) - 3 = 5 - 3 = 2. Exact arithmetic
    // would give a different B for any dist where 2A/sqrt2 lands just above an integer.
    #[test]
    fn b_is_a_truncated_double_ceil_not_exact_arithmetic() {
        let ring = edge_obstruction((0, 0), (50, 50), 3);
        assert_eq!(ring[7], (-3, -2), "p7 = (low.x - A, low.y - B) = (-3, -2)");
        assert_eq!(ring[0], (-2, -3), "p0 = (low.x - B, low.y - A) = (-2, -3)");
    }

    // ⚠️ On a y-tie the SECOND point is `high` (`Oct::init` picks the larger y, ties to p2).
    // Unreachable through `commit_route` — `is45DegreeEdge` filters axis-aligned edges out first —
    // but pinned so the transcription cannot silently flip.
    #[test]
    fn a_y_tie_makes_the_second_point_high() {
        // low = (0,0), high = (100,0), RIGHT: p4 = (high.x + B, high.y + A) = (105, 10).
        assert_eq!(edge_obstruction((0, 0), (100, 0), 10)[4], (105, 10));
        // Reversed, low = (100,0) and high = (0,0), so it is LEFT: p4.x moves to high.x + dist.
        assert_eq!(edge_obstruction((100, 0), (0, 0), 10)[4], (10, 10));
    }

    #[test]
    fn a_segment_through_the_octagon_hits_and_one_clear_of_it_misses() {
        let ring = edge_obstruction((0, 0), (100, 100), 10);
        assert!(segment_hits_polygon((0, 100), (100, 0), &ring), "crosses the body");
        assert!(segment_hits_polygon((40, 40), (60, 60), &ring), "lies wholly inside");
        assert!(!segment_hits_polygon((-200, 300), (200, 300), &ring), "clear of it");
        assert!(!segment_hits_polygon((-100, 60), (-50, 60), &ring), "beside it, no crossing");
    }
}

#[cfg(test)]
mod multi_target_tests {
    use super::*;

    #[test]
    fn touching_endpoints_and_collinear_overlap_both_count_as_intersecting() {
        // A clean crossing.
        assert!(segments_intersect((0, 0), (10, 10), (0, 10), (10, 0)));
        // Meeting at a single endpoint — `boost::geometry::intersects` says yes, so must we.
        assert!(segments_intersect((0, 0), (10, 0), (10, 0), (10, 10)));
        // Collinear and overlapping.
        assert!(segments_intersect((0, 0), (10, 0), (5, 0), (15, 0)));
        // Collinear, disjoint.
        assert!(!segments_intersect((0, 0), (10, 0), (11, 0), (20, 0)));
        // Parallel, apart.
        assert!(!segments_intersect((0, 0), (10, 0), (0, 1), (10, 1)));
        // Would cross if extended, but the segments stop short.
        assert!(!segments_intersect((0, 0), (1, 1), (5, 10), (10, 5)));
        // ⚠️ Four T-junctions, one per endpoint, because each is decided by a DIFFERENT clause: the
        // touching endpoint lies on the other segment's interior, so no strict crossing is found
        // and only that endpoint's own collinearity test fires. Dropping any one of the four
        // silently stops detecting a wire that just touches another.
        assert!(segments_intersect((5, 0), (5, 10), (0, 0), (10, 0)), "p1 on p3p4");
        assert!(segments_intersect((5, 10), (5, 0), (0, 0), (10, 0)), "p2 on p3p4");
        assert!(segments_intersect((0, 0), (10, 0), (5, 0), (5, 10)), "p3 on p1p2");
        assert!(segments_intersect((0, 0), (10, 0), (5, 10), (5, 0)), "p4 on p1p2");
    }

    // 🔑 Upstream rule: only the polygon's NON-axis-aligned edges block an access line; the
    // axis-aligned ones are skipped by an explicit test. An octagonal pad's flat sides are
    // therefore transparent and its facets are not.
    #[test]
    fn only_a_diagonal_pin_edge_blocks_an_access_line() {
        // A unit octagon centred on the origin, closed.
        let oct = vec![
            (10, 4),
            (10, -4),
            (4, -10),
            (-4, -10),
            (-10, -4),
            (-10, 4),
            (-4, 10),
            (4, 10),
            (10, 4),
        ];
        let polys = vec![oct];
        // Straight out through the flat right side: crosses only an axis-aligned edge — kept.
        assert_eq!(
            snaps_clear_of_diagonal_pin_edges((0, 0), &[(40, 0)], &polys),
            vec![(40, 0)],
            "an axis-aligned pin edge is not a barrier"
        );
        // Out through a facet — dropped.
        assert!(
            snaps_clear_of_diagonal_pin_edges((0, 0), &[(40, 40)], &polys).is_empty(),
            "a diagonal pin edge blocks the access line"
        );
        // Both offered: only the clear one survives, and the order is preserved.
        assert_eq!(
            snaps_clear_of_diagonal_pin_edges((0, 0), &[(40, 40), (40, 0), (0, -40)], &polys),
            vec![(40, 0), (0, -40)]
        );
    }

    // ⚠️ With no polygon geometry the filter must be a pass-through, not a rejection — nearly every
    // pin in a design is a plain rectangle.
    #[test]
    fn a_terminal_with_no_polygon_geometry_keeps_every_snap() {
        let snaps = [(1, 2), (3, 4), (5, 6)];
        assert_eq!(snaps_clear_of_diagonal_pin_edges((0, 0), &snaps, &[]), snaps.to_vec());
    }

    // 🔑 Upstream rule (`RDLRouter::route`): cross product, sorted by distance then by
    // target0.x, target0.y, target1.x, target1.y.
    #[test]
    fn target_pairs_are_ordered_by_distance_then_by_the_four_coordinates() {
        // Two sources and two destinations: four pairings, three distinct distances.
        let src = [(0, 0), (100, 0)];
        let dst = [(100, 0), (0, 100)];
        // distances: (0,0)->(100,0) = 100; (0,0)->(0,100) = 100; (100,0)->(100,0) = 0;
        //            (100,0)->(0,100) = 141
        assert_eq!(
            target_pairs(&src, &dst),
            vec![(1, 0), (0, 1), (0, 0), (1, 1)],
            "shortest first; the two 100-unit pairings tie on both source coordinates and are \
             settled by target1.x — (0,100) before (100,0)"
        );
    }

    // The tie-break reaches all four coordinates, so a mutant that stops early is caught.
    #[test]
    fn an_exact_tie_is_settled_by_the_destination_coordinates() {
        // One source, two destinations equidistant from it: 3-4-5 both ways.
        let src = [(0, 0)];
        let dst = [(4, 3), (3, 4)];
        assert_eq!(
            target_pairs(&src, &dst),
            vec![(0, 1), (0, 0)],
            "equal distance, so the lower destination x wins: (3,4) before (4,3)"
        );
    }

    // ⚠️ `distance` truncates, so two pairings whose true lengths differ by a fraction of a unit
    // TIE here and fall through to the coordinates. Computing in floating point would order them.
    #[test]
    fn distances_that_differ_only_below_a_unit_tie_rather_than_ordering() {
        assert_eq!(distance((0, 0), (10, 0)), 10);
        assert_eq!(distance((0, 0), (9, 4)), 9, "sqrt(97) = 9.848… truncates to 9");
        let src = [(0, 0)];
        let dst = [(10, 0), (7, 7)];
        // sqrt(98) = 9.899… also truncates to 9, so (7,7) ties with (9,4) — and beats (10,0).
        assert_eq!(distance((0, 0), (7, 7)), 9);
        assert_eq!(target_pairs(&src, &dst), vec![(0, 1), (0, 0)]);
    }
}

#[cfg(test)]
mod write_order_tests {
    use super::*;

    fn dest(term: &str, at: Point, id: u64) -> Dest {
        Dest { terminal: term.into(), instance: term.into(), centre: at, cover: false, id }
    }

    fn route(src: &str, at: Point, to: Dest, id: u64) -> Route {
        Route {
            source: src.into(),
            instance: src.into(),
            centre: at,
            id,
            dests: vec![to],
            next: 0,
            priority: 0,
            routed: false,
            pending: true,
            points: Vec::new(),
            locked: false,
            stubs: Vec::new(),
        }
    }

    /// ⛔ **`removeTerminalAccess` restores with the CHECKS ON.** An edge taken out to make room
    /// for a terminal's access comes back only if it is still clear; one that a committed route now
    /// crosses is never restored, and the graph loses it for the rest of the run.
    ///
    /// 🔑 The two undo paths differ only in those flags — `uncommitRoute` passes
    /// `false, false` and always restores — so a single unconditional `undo` for both is the
    /// mistake this pins.
    #[test]
    fn an_access_edge_a_route_now_crosses_is_never_restored() {
        let mut g = Graph::default();
        let a = g.vertex((0, 0));
        let b = g.vertex((100, 0));
        let c = g.vertex((0, 100));
        let d = g.vertex((100, 100));
        g.join(a, b, 100);
        g.join(c, d, 100);

        // Take both out, the way inserting terminal access does.
        let mut undo = Undo::default();
        for (u, v) in [(a, b), (c, d)] {
            let w = g.weight_between(u, v).unwrap();
            undo.restore.push((u, v, restore_scale(g.points[u], g.points[v], w)));
            g.cut(u, v);
        }
        assert_eq!(g.weight_between(a, b), None);

        // A committed route now runs through (50, 0), which lies on a–b but not on c–d.
        let route = [(50, 0)];
        let blocked = |p0: Point, p1: Point| route.iter().any(|&pt| point_on_segment(pt, p0, p1));
        undo_access(&mut g, &undo, &blocked);

        assert_eq!(g.weight_between(a, b), None, "the crossed edge stays out");
        // ⚠️ **101, not 100** — and that is the other rule, not a slip. c–d is HORIZONTAL, so it
        // was stored with the `direction_bias`; the scale `removeGraphEdge` recovers by division
        // carries that bias, and re-adding applies it a second time. See `restore_scale`.
        assert_eq!(
            g.weight_between(c, d),
            Some(101),
            "the clear one comes back, one unit dearer for being horizontal"
        );
    }

    /// ⛔ **`RDLNet::isRouted` descends the graph; it is not a pair lookup.** A and C are never
    /// joined directly, but A–B and B–C are both committed, so the reference considers A already
    /// connected to C and refuses to route it again.
    ///
    /// 🔑 This is the test that fails if the descent is reduced to the direct pair, which is what
    /// this engine did until the call-graph review found `isRouted` had no transitive counterpart.
    #[test]
    fn two_terminals_joined_through_a_chain_count_as_already_routed() {
        let mut ab = route("A", (0, 0), dest("B", (100, 0), 1), 1);
        ab.routed = true;
        ab.next = 1;
        ab.points = vec![(0, 0), (100, 0)];
        let mut bc = route("B", (100, 0), dest("C", (200, 0), 2), 2);
        bc.routed = true;
        bc.next = 1;
        bc.points = vec![(100, 0), (200, 0)];
        let routes = vec![ab, bc];
        let adj = routed_pairs(&routes);

        assert!(is_routed(&adj, "A", "B"), "the direct pair");
        assert!(is_routed(&adj, "B", "A"), "and its reverse — the graph is undirected");
        assert!(
            is_routed(&adj, "A", "C"),
            "A reaches C THROUGH B; a direct-pair test would miss this and route it again"
        );
        assert!(is_routed(&adj, "C", "A"), "and the same the other way");
        assert!(!is_routed(&adj, "A", "D"), "an unconnected terminal is not routed");
        assert!(is_routed(&adj, "A", "A"), "`if (source == dest) return true`");
    }

    /// ⚠️ A LOCKED segment contributes nothing to the graph. `preprocess` ends in `setRouted()`,
    /// and `net_->updateRoute` is reached only from `setRoute` and `resetRoute` — so the pair it
    /// locked was never recorded, and other segments go on offering that terminal.
    #[test]
    fn a_locked_segment_is_not_in_the_routed_graph() {
        let mut locked = route("A", (0, 0), dest("B", (100, 0), 1), 1);
        locked.routed = true;
        locked.locked = true;
        locked.next = 1; // its destination IS recorded, but no path was committed
        let routes = vec![locked];
        let adj = routed_pairs(&routes);
        assert!(!is_routed(&adj, "A", "B"), "a locked pair never enters routed_pairs_");
    }

    // ⚠️ A route that never connected contributes NOTHING. Upstream guards the write with
    // `if (segment->isRouted())`, so a failed segment is skipped rather than emitted with an empty
    // path — and the surviving entries keep their list order rather than closing up by completion.
    #[test]
    fn a_route_that_failed_is_not_emitted() {
        // One corridor. Whichever route commits first takes the middle out from under the other.
        let grid = Grid { x: vec![0, 10, 20, 30], y: vec![0] };
        let edges = vec![((0, 0), (10, 0)), ((10, 0), (20, 0)), ((20, 0), (30, 0))];
        let mut graph = Graph::build(&grid, &edges, 1.0);

        let mut routes = vec![
            route("A/p", (0, 0), dest("B/p", (30, 0), 2), 1),
            route("C/p", (10, 0), dest("D/p", (20, 0), 4), 3),
        ];
        routes[1].priority = 10; // C/p goes first and blocks the corridor

        let mut access: std::collections::HashMap<String, (String, Vec<(Point, Vec<Point>)>)> =
            Default::default();
        access.insert("A/p".into(), ("N1".into(), vec![((0, 0), vec![(0, 0)])]));
        access.insert("B/p".into(), ("N1".into(), vec![((30, 0), vec![(30, 0)])]));
        access.insert("C/p".into(), ("N1".into(), vec![((10, 0), vec![(10, 0)])]));
        access.insert("D/p".into(), ("N1".into(), vec![((20, 0), vec![(20, 0)])]));

        let out = route_all(&mut graph, &grid, &mut routes, &access, 8, 4, 0.0, 1, None, false, &|_, _| false);
        let sources: Vec<&str> = out.paths.iter().map(|(_, s, _, _)| s.as_str()).collect();
        assert_eq!(sources, vec!["C/p"], "only the route that connected is emitted");
        assert_eq!(out.failed, vec!["A/p".to_string()]);
    }

    // ⚠️ The destination recorded is the one that actually connected, not the first candidate.
    // It decides which pin shape the wire is clipped against at the write, so taking `dests[0]`
    // when the router fell through to `dests[1]` clips against the wrong pin.
    #[test]
    fn the_destination_recorded_is_the_one_that_connected() {
        // A reachable corridor 0..30, plus an isolated vertex at 100 that nothing can reach.
        let grid = Grid { x: vec![0, 10, 20, 30, 100], y: vec![0] };
        let edges = vec![((0, 0), (10, 0)), ((10, 0), (20, 0)), ((20, 0), (30, 0))];
        let mut graph = Graph::build(&grid, &edges, 1.0);

        let mut r = route("A/p", (0, 0), dest("X/p", (100, 0), 2), 1);
        r.dests.push(dest("B/p", (30, 0), 4)); // tried only after X/p fails
        let mut routes = vec![r];

        let mut access: std::collections::HashMap<String, (String, Vec<(Point, Vec<Point>)>)> =
            Default::default();
        access.insert("A/p".into(), ("N1".into(), vec![((0, 0), vec![(0, 0)])]));
        access.insert("X/p".into(), ("N1".into(), vec![((100, 0), vec![(100, 0)])]));
        access.insert("B/p".into(), ("N1".into(), vec![((30, 0), vec![(30, 0)])]));

        let out = route_all(&mut graph, &grid, &mut routes, &access, 2, 0, 0.0, 10, None, false, &|_, _| false);
        let dests: Vec<&str> = out.paths.iter().map(|(_, _, d, _)| d.as_str()).collect();
        assert_eq!(dests, vec!["B/p"], "X/p was unreachable, so B/p is what connected");
    }

    // 🔑 Upstream rule: nothing ever reorders `RDLNet::segments_`. The priority queue decides
    // WHEN a segment is attempted; `writeToDb` is reached by walking each net's `getSegments()`
    // in the order `buildIntialRouteSet` added them. So the emitted order must follow the route
    // list, not the order routes happened to complete.
    //
    // ⚠️ This test is built so the two orders DISAGREE: route 1 is given a higher priority than
    // route 0, so it is attempted first and completes first.
    #[test]
    fn routes_are_emitted_in_list_order_not_completion_order() {
        // Two independent corridors, far enough apart not to interact.
        let grid = Grid { x: vec![0, 10, 20, 30], y: vec![0, 1000] };
        let edges = vec![
            ((0, 0), (10, 0)),
            ((10, 0), (20, 0)),
            ((20, 0), (30, 0)),
            ((0, 1000), (10, 1000)),
            ((10, 1000), (20, 1000)),
            ((20, 1000), (30, 1000)),
        ];
        let mut graph = Graph::build(&grid, &edges, 1.0);

        let mut routes = vec![
            route("A/p", (0, 0), dest("B/p", (30, 0), 2), 1),
            route("C/p", (0, 1000), dest("D/p", (30, 1000), 4), 3),
        ];
        // Route 1 is attempted first — `precedes` puts the higher priority at the front.
        routes[1].priority = 10;

        let mut access: std::collections::HashMap<String, (String, Vec<(Point, Vec<Point>)>)> =
            Default::default();
        access.insert("A/p".into(), ("N1".into(), vec![((0, 0), vec![(0, 0)])]));
        access.insert("B/p".into(), ("N1".into(), vec![((30, 0), vec![(30, 0)])]));
        access.insert("C/p".into(), ("N2".into(), vec![((0, 1000), vec![(0, 1000)])]));
        access.insert("D/p".into(), ("N2".into(), vec![((30, 1000), vec![(30, 1000)])]));
        // Its own access points are grid corners with no path between them.

        let out = route_all(&mut graph, &grid, &mut routes, &access, 2, 0, 0.0, 10, None, false, &|_, _| false);

        let sources: Vec<&str> = out.paths.iter().map(|(_, s, _, _)| s.as_str()).collect();
        assert_eq!(
            sources,
            vec!["A/p", "C/p"],
            "emitted in route-list order even though C/p routed first"
        );
        // …and the attempt log proves the two orders really did disagree.
        assert_eq!(
            out.log.first().map(|(src, ..)| src.as_str()),
            Some("C/p"),
            "C/p was attempted first"
        );
    }
}

#[cfg(test)]
mod obstacle_tests {
    use super::*;

    /// The octagonal bump pad from `passive_tech/bumps.lef` (`BUMP45`), centred on the origin.
    #[test]
    fn a_fixed_swire_on_the_net_being_routed_still_obstructs_it() {
        // ⛔ The load-bearing case, and the one a "skip my own net" reading gets wrong.
        // `rdl_route_keep_existing` places one FIXED rectangle on VDD and then routes VDD; the
        // reference lays 6 more wires than a router that drives straight through it.
        assert!(swire_obstructs(true, true), "a FIXED swire is kept, so it obstructs its own net");
        // The only skip: a wire `route()` is about to destroy and rebuild.
        assert!(!swire_obstructs(true, false));
        // Another net's wires obstruct whatever their type.
        assert!(swire_obstructs(false, false));
        assert!(swire_obstructs(false, true));
    }

    #[test]
    fn the_plain_octagon_keeps_odbs_truncated_b() {
        // `Oct((0,0), (0,100), 40)`: A = 40/2 = 20, B = ceil(40 / sqrt2) - 20 = ceil(28.284) - 20
        // = 29 - 20 = 9. ⚠️ Both steps truncate; computing B exactly gives a different polygon.
        let pts = oct_points((0, 0), (0, 100), 40);
        assert_eq!(pts.len(), 9);
        assert_eq!(pts[0], (-9, -20), "low oct (-B, -A)");
        assert_eq!(pts[8], pts[0], "the ring is closed");
        assert_eq!(pts[1], (9, -20), "low oct (B, -A)");
        assert_eq!(pts[4], (9, 120), "high oct (B, A)");
        assert_eq!(pts[5], (-9, 120), "high oct (-B, A)");
        // ⚠️ Vertical: `high.x > low.x` is false, so `getDir()` is LEFT, not RIGHT.
        assert_eq!(pts[2], (20, -9), "LEFT: low oct (A, -B)");
        assert_eq!(pts[6], (-20, 109), "LEFT: high oct (-A, B)");
        // A tie on y makes the SECOND point `high` — `Oct::init` compares with `>`.
        assert_eq!(oct_points((0, 0), (100, 0), 40)[4], (109, 20));
    }

    fn octagon() -> Vec<Point> {
        vec![
            (12_000, -28_000),
            (-12_000, -28_000),
            (-28_000, -12_000),
            (-28_000, 12_000),
            (-12_000, 28_000),
            (12_000, 28_000),
            (28_000, 12_000),
            (28_000, -12_000),
            (12_000, -28_000),
        ]
    }

    // ⛔ The reason obstructions stopped being rectangles. OpenDB decomposes this octagon through
    // `polygon_90`, which cannot hold a 45° edge, into three rectangles:
    //     x[-28000,12000] y[-28000,-12000],  x[-28000,28000] y[-12000,12000],
    //     x[-12000,28000] y[12000,28000]
    // The corner at (-24000, -24000) is OUTSIDE the octagon but inside that union, and the corner
    // at (24000, -24000) is the mirror case: inside neither, but the octagon's own edge runs much
    // closer there. The two shapes are not the same, and the union is not even a cover.
    #[test]
    fn the_rectangle_decomposition_of_an_octagon_is_not_the_octagon() {
        let oct = Obstacle::from_ring(octagon());
        let decomposed = [
            Obstacle::Rect((-28_000, -28_000, 12_000, -12_000)),
            Obstacle::Rect((-28_000, -12_000, 28_000, 12_000)),
            Obstacle::Rect((-12_000, 12_000, 28_000, 28_000)),
        ];
        // A point in the cut-off bottom-left corner: the decomposition says metal, the octagon
        // says empty space.
        let p = (-24_000, -24_000);
        assert!(decomposed.iter().any(|o| o.hits(p, p)), "the decomposition covers this corner");
        assert!(!oct.hits(p, p), "the octagon does not — it is cut off at 45 degrees");
    }

    #[test]
    fn an_octagon_blocks_a_line_through_its_body_and_not_one_past_its_cut_corner() {
        let oct = Obstacle::from_ring(octagon());
        assert!(oct.hits((-40_000, 0), (40_000, 0)), "straight through the middle");
        assert!(oct.hits((0, -40_000), (0, 40_000)), "and the other way");
        // Along the bottom-left diagonal, clear of the cut corner but inside the bounding box.
        assert!(!oct.hits((-40_000, -22_000), (-22_000, -40_000)), "outside the cut corner");
        assert!(!oct.hits((-40_000, -40_000), (40_000, -40_000)), "well below it");
    }

    // ⚠️ The bounding box is a REJECT, never an accept — a segment inside the box but outside the
    // shape must not count. That is the whole point of keeping the ring.
    #[test]
    fn the_bounding_box_never_decides_a_hit_on_its_own() {
        let oct = Obstacle::from_ring(octagon());
        assert_eq!(oct.bbox(), (-28_000, -28_000, 28_000, 28_000));
        let corner = ((-28_000, -28_000), (-24_000, -24_000));
        assert!(hits(corner.0, corner.1, oct.bbox()), "inside the bounding box");
        assert!(!oct.hits(corner.0, corner.1), "but outside the octagon");
    }

    // A rectilinear ring collapses back to the cheap form, because there the rectangle test IS
    // exact — and it is the overwhelmingly common case.
    #[test]
    fn a_rectangular_ring_collapses_to_a_rectangle() {
        let r = Obstacle::from_ring(vec![(0, 0), (100, 0), (100, 50), (0, 50), (0, 0)]);
        assert_eq!(r, Obstacle::Rect((0, 0, 100, 50)));
        // …and an L-shape does not, even though every edge of it is axis-aligned.
        let l = Obstacle::from_ring(vec![
            (0, 0),
            (100, 0),
            (100, 50),
            (50, 50),
            (50, 100),
            (0, 100),
            (0, 0),
        ]);
        assert!(matches!(l, Obstacle::Poly { .. }), "an L is not its bounding box");
        assert!(!l.hits((60, 60), (90, 90)), "the notch is empty");
        assert!(l.hits((10, 60), (40, 90)), "the arm is not");
    }
}
