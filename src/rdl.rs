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
pub fn blocked(p0: Point, p1: Point, obstructions: &[Rect]) -> bool {
    obstructions.iter().any(|&r| hits(p0, p1, r))
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
pub fn access_points(g: &Grid, target: &Target, obstructions: &[Rect], own: &[Rect]) -> Vec<Point> {
    let foreign = |r: &Rect| !own.contains(r);
    // ⚠️ **No exemption here.** A candidate track is rejected if it lies inside ANY obstruction,
    // the terminal's own metal included. The exemption applies only to the line test below. Excusing
    // it here picks tracks inside the terminal's own pad, where every grid edge has been filtered
    // away — the access points then attach to dead grid points and the terminal is unreachable,
    // with four perfectly plausible-looking access points to show for it.
    let clear = |p: Point| !obstructions.iter().any(|r| hits(p, p, *r));
    let mut out = Vec::new();
    for x in nearest_tracks(&g.x, target.centre.0, &|x| clear((x, target.centre.1))) {
        out.push((x, target.centre.1));
    }
    for y in nearest_tracks(&g.y, target.centre.1, &|y| clear((target.centre.0, y))) {
        out.push((target.centre.0, y));
    }
    out.retain(|&p| !obstructions.iter().any(|r| foreign(r) && hits(target.centre, p, *r)));
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
    pub restore: Vec<(usize, usize, i64)>,
    pub cut: Vec<(usize, usize)>,
}

impl Graph {
    /// Put the graph back as it was before the change `undo` records.
    pub fn undo(&mut self, undo: &Undo) {
        for &(a, b) in &undo.cut {
            self.cut(a, b);
        }
        for &(a, b, w) in &undo.restore {
            self.join(a, b, w);
        }
    }

    fn weight_between(&self, a: usize, b: usize) -> Option<i64> {
        self.adj[a].iter().find(|&&(v, _)| v == b).map(|&(_, w)| w)
    }
}

pub fn insert_access(graph: &mut Graph, g: &Grid, centre: Point, snaps: &[Point]) -> Undo {
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
                undo.restore.push((a, b, w));
            }
            graph.cut(a, b);
        }
        let sv = graph.vertex(snap);
        let w = edge_weight(snap, centre, 1.0);
        graph.join(sv, c, w);
        undo.cut.push((sv, c));
        for &e in &ends {
            let w = edge_weight(snap, graph.points[e], 1.0);
            graph.join(sv, e, w);
            undo.cut.push((sv, e));
        }
    }
    undo
}

/// **L6** — take a routed path out of the graph, recording how to put it back.
///
/// Every edge touching a route vertex goes, and so does every edge crossing the route's corridor.
/// ⚠️ Recorded rather than simply deleted: rip-up has to restore exactly these edges, and
/// recomputing which ones they were after the graph has moved on gives a different set.
pub fn commit_route(graph: &mut Graph, route: &[Point], width: i32, spacing: i32) -> Undo {
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
    for (i, &p) in graph.points.iter().enumerate() {
        if !corridor.iter().any(|&r| hits(p, p, r)) {
            continue;
        }
        for &(o, _) in &graph.adj[i] {
            let q = graph.points[o];
            if corridor.iter().any(|&r| hits(p, q, r)) {
                drop.insert(if i < o { (i, o) } else { (o, i) });
            }
        }
    }
    for (a, b) in drop {
        if let Some(w) = graph.weight_between(a, b) {
            undo.restore.push((a, b, w));
            graph.cut(a, b);
        }
    }
    undo
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
    let mut heap = std::collections::BinaryHeap::new();

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
    // Ordered by (f, vertex) with the sign flipped, so the smallest f leaves first and equal
    // costs are settled by vertex number rather than by whichever happened to be pushed first.
    heap.push((std::cmp::Reverse(heuristic(s, &prev)), std::cmp::Reverse(s)));

    while let Some((std::cmp::Reverse(_), std::cmp::Reverse(u))) = heap.pop() {
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
            let candidate = dist[u].saturating_add(w);
            if candidate < dist[v] {
                dist[v] = candidate;
                prev[v] = u;
                let f = candidate.saturating_add(heuristic(v, &prev));
                heap.push((std::cmp::Reverse(f), std::cmp::Reverse(v)));
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
    pub attempts: usize,
    pub iterations: i32,
    pub failed: Vec<String>,
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
    access: &std::collections::HashMap<String, (Point, Vec<Point>, String)>,
    width: i32,
    spacing: i32,
    turn_penalty: f32,
    max_iterations: i32,
) -> Routed {
    let mut out = Routed::default();
    let mut committed: Vec<Option<Undo>> = vec![None; routes.len()];
    let mut queue: Vec<usize> = (0..routes.len()).collect();
    let mut last_done: std::collections::BTreeSet<String> = Default::default();

    loop {
        queue.sort_by(|&a, &b| routes[a].precedes(&routes[b]));
        while let Some(i) = queue.first().copied() {
            queue.remove(0);
            routes[i].pending = false;
            if !routes[i].has_next() {
                continue;
            }
            let d = routes[i].dests[routes[i].next].clone();
            routes[i].next += 1;

            let (Some(src), Some(dst)) = (access.get(&routes[i].source), access.get(&d.terminal))
            else {
                continue;
            };
            out.attempts += 1;
            let a = insert_access(graph, grid, src.0, &src.1);
            let b = insert_access(graph, grid, dst.0, &dst.1);
            let path = shortest_path(graph, src.0, dst.0, turn_penalty);
            graph.undo(&b);
            graph.undo(&a);

            if path.is_empty() {
                if routes[i].has_next() {
                    queue.push(i);
                }
                continue;
            }
            committed[i] = Some(commit_route(graph, &path, width, spacing));
            routes[i].routed = true;
            routes[i].points = path.clone();
            out.paths.push((src.2.clone(), routes[i].source.clone(), d.terminal.clone(), path));
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
            out.paths.retain(|(_, s, _, _)| *s != routes[t].source);
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
        }
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
        let wall = [(150, 0, 170, 350)];
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
        let wall = [(150, -100, 170, 1000)];
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
        let t = Target { terminal: "u/PAD".into(), centre: (150, 150), shape: own, access: vec![] };
        assert_eq!(access_points(&g, &t, &[own], &[own]).len(), 4);
        assert!(access_points(&g, &t, &[own], &[]).is_empty(), "not excused, none survive");
    }

    #[test]
    fn a_candidate_track_inside_any_obstruction_is_rejected_even_the_targets_own() {
        // ⚠️ The asymmetry that matters: the exemption applies to the LINE, never to the
        // candidate. A track inside the terminal's own pad is dead grid — every edge there was
        // filtered out — so accepting it yields access points that reach nothing.
        let g = Grid { x: vec![0, 100, 200, 300], y: vec![150] };
        let own = (90, 140, 210, 160);
        let t = Target { terminal: "u/PAD".into(), centre: (150, 150), shape: own, access: vec![] };
        let pts = access_points(&g, &t, &[own], &[own]);
        assert!(!pts.contains(&(100, 150)), "inside the pad, rejected as a candidate");
        assert!(!pts.contains(&(200, 150)), "likewise");
        assert!(pts.contains(&(0, 150)) && pts.contains(&(300, 150)), "the live tracks outside");
    }

    #[test]
    fn an_obstruction_in_the_way_removes_that_access_point() {
        let g = Grid { x: vec![0, 100, 200], y: vec![0, 100, 200] };
        let t = Target {
            terminal: "u/PAD".into(),
            centre: (150, 150),
            shape: (140, 140, 160, 160),
            access: vec![],
        };
        // A wall just left of the centre blocks the westward access only.
        let wall = [(120, 0, 130, 300)];
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
        let blocker = [(25, 0, 35, 1000)];
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
