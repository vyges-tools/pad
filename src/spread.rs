// SPDX-License-Identifier: Apache-2.0
//! Force-directed pad placement — the `placer` strategy.
//!
//! ⚠️ **Not simulated annealing, despite what the mode's name suggests.** There is no random
//! number generator anywhere in it. Three deterministic stages: each pad's *ideal* position from
//! the bumps it serves, an isotonic regression that restores their order along the row, and an
//! iterative spring-and-repulsion spread that separates them. The distinction matters because a
//! stochastic placer could only be reproduced by reproducing its generator, and this one cannot
//! diverge that way at all.
//!
//! Nothing here touches a database.

/// Up to this many spreading iterations before the placer gives up.
pub const MAX_ITERATIONS: i32 = 5000;
const SPRING_START: f32 = 0.1;
const REPEL_START: f32 = 0.5;
const REPEL_END: f32 = 0.5;
/// The first iteration at which the spring force begins to fade.
const SPRING_FADE_FROM: i32 = (0.2 * MAX_ITERATIONS as f32) as i32;
/// The iteration by which it has faded entirely.
const SPRING_FADE_TO: i32 = (0.5 * MAX_ITERATIONS as f32) as i32;
/// How much of a computed move is actually taken each iteration.
pub const DAMPER: f32 = 0.2;

/// **SP1** — a pad's ideal position: the mean of the bumps it serves.
///
/// ⚠️ The plain mean, rounded — not the median and not weighted. A pad serving two distant bumps
/// sits midway between them, which is a place neither bump wants; the spreading stage is what
/// resolves that, not this one.
pub fn ideal_position(bump_centres: &[i32]) -> Option<i32> {
    if bump_centres.is_empty() {
        return None;
    }
    let total: i64 = bump_centres.iter().map(|&c| c as i64).sum();
    Some((total as f32 / bump_centres.len() as f32).round() as i32)
}

/// **SP2** — a starting position for a pad that serves no bump.
///
/// ⚠️ Deliberately crude, and order-dependent: the first pad takes the row's start, the last takes
/// its end, the second copies its predecessor, and anything else takes the **average of the two
/// before it**. It only has to be a starting point the regression can order.
pub fn unconstrained_start(index: usize, count: usize, prior: &[i32], row: (i32, i32)) -> i32 {
    if index == 0 {
        row.0
    } else if index + 1 == count {
        row.1
    } else if index == 1 {
        prior[index - 1]
    } else {
        ((prior[index - 1] as i64 + prior[index - 2] as i64) / 2) as i32
    }
}

/// **SP3** — restore order along the row by pooling adjacent violators.
///
/// Where a pad wants to sit before the one ahead of it, the pair is replaced by their weighted
/// mean and their weights combine. Repeated until nothing moves.
///
/// ⚠️ **The weight update is asymmetric, and deliberately reproduced.** The reference writes
/// `w[i] += w[i-1]` and *then* `w[i-1] += w[i]`, so the second uses the value the first just
/// wrote: two unit weights become 2 and 3, not 2 and 2. The symmetric version is what one would
/// write from the algorithm's description, and it pools later violations differently.
pub fn pool_adjacent_violators(positions: &mut [i32], weights: &mut [f32]) -> bool {
    let mut updated = false;
    for i in 1..positions.len() {
        if positions[i] >= positions[i - 1] {
            continue;
        }
        updated = true;
        let total = weights[i] + weights[i - 1];
        let pooled = ((weights[i] * positions[i] as f32
            + weights[i - 1] * positions[i - 1] as f32)
            / total)
            .round() as i32;
        positions[i] = pooled;
        positions[i - 1] = pooled;
        weights[i] += weights[i - 1];
        weights[i - 1] += weights[i];
    }
    updated
}

/// **SP4** — the spring and repulsion strengths at a given iteration.
///
/// Repulsion is constant. The spring — which pulls a pad back towards its ideal — holds at full
/// strength for the first fifth of the run, fades linearly to nothing by the halfway point, and is
/// zero thereafter.
///
/// ⚠️ The comparisons are strictly greater-than, so the fade starts one iteration *after* the
/// threshold. Off by one here shifts every position slightly rather than obviously.
pub fn forces(iteration: i32) -> (f32, f32) {
    let repel =
        REPEL_START + (REPEL_END - REPEL_START) * iteration as f32 / MAX_ITERATIONS as f32;
    let range = (SPRING_FADE_TO - SPRING_FADE_FROM) as f32;
    let faded = if iteration > SPRING_FADE_FROM {
        SPRING_START * (range - (iteration - SPRING_FADE_FROM) as f32) / range
    } else {
        SPRING_START
    };
    let spring = if iteration > SPRING_FADE_TO { 0.0 } else { faded };
    (spring, repel)
}

/// **SP5** — the nearest position at which a pad does not overlap.
///
/// Given the obstruction a pad at `target` would hit, the two ways out are just before it and just
/// after it, each by half the pad's width. The nearer wins.
///
/// ⚠️ **Except at the row's ends**, where only one way out exists: if stepping back would leave the
/// row, the pad must go forward, and the reverse at the far end. Taking the nearer without that
/// check pushes pads off the row at both ends.
pub fn nearest_legal(
    target: i32,
    obstruction: Option<(i32, i32)>,
    half_width: i32,
    row: (i32, i32),
) -> i32 {
    let Some((lo, hi)) = obstruction else { return target };
    let (start, end) = (lo - half_width, hi + half_width);
    if start < row.0 {
        return end;
    }
    if end > row.1 {
        return start;
    }
    if (target - start) < (end - target) {
        start
    } else {
        end
    }
}

/// Where a pad sits: its span and its centre, kept together because the loop needs both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Anchor {
    pub min: i32,
    pub centre: i32,
    pub max: i32,
    pub width: i32,
}

impl Anchor {
    pub fn at(pos: i32, width: i32) -> Anchor {
        Anchor { min: pos, centre: pos + width / 2, max: pos + width, width }
    }

    pub fn set_location(&mut self, pos: i32) {
        *self = Anchor::at(pos, self.width);
    }

    /// **SP6** — how far `other` reaches past this pad's start.
    ///
    /// ⚠️ **One-sided and directional.** It measures only `other.max - self.min`, so it is
    /// meaningful when `other` is the pad *before* this one and meaningless the other way round.
    /// The loop always calls it in that direction; a symmetric overlap would report a collision
    /// between every pair of pads that merely sit near one another.
    pub fn overlap(&self, other: &Anchor) -> i32 {
        if other.max > self.min {
            other.max - self.min
        } else {
            0
        }
    }
}

/// **SP7** — how far a pad moves this iteration.
///
/// The spring pulls it toward its ideal; an overlap with the pad before pushes it forward and one
/// with the pad after pushes it back, each by the overlap **plus a whole site**, so a resolved
/// collision leaves a gap rather than a touch.
///
/// ⚠️ The damped move is rounded **up** to a whole site, so any non-zero force moves at least one
/// site. Rounding down would let a small force compute a sub-site move, round to nothing, and the
/// loop would spin to the iteration limit with the row still overlapping.
#[allow(clippy::too_many_arguments)]
pub fn step_move(
    spring_delta: i32,
    spring: f32,
    overlap_prev: i32,
    overlap_next: i32,
    repel: f32,
    damper: f32,
    site: i32,
) -> i32 {
    let mut force = spring_delta as f32 * spring;
    if overlap_prev > 0 {
        force += (overlap_prev + site) as f32 * repel;
    }
    if overlap_next > 0 {
        force -= (overlap_next + site) as f32 * repel;
    }
    let magnitude = (force * damper).abs();
    let sign = if force < 0.0 { -1 } else { 1 };
    sign * (magnitude / site as f32).ceil() as i32 * site
}

/// **SP8** — one pass over the row.
///
/// Each pad moves in turn and **sees the pads before it already moved** — the pass is sequential,
/// not simultaneous. A pad is clamped between its neighbours' centres, and between the row's ends
/// for the first and last.
///
/// ⚠️ The row's ends are **inset by half the pad's own width**, because everything in this loop is
/// in *centre* coordinates. Clamping a centre against the raw row edge lets the last pad hang half
/// its width past the end of the row — which the placer then refuses to place, reporting a pad
/// that does not fit in a row that has room for it.
///
/// Returns whether any overlap remained, which is what stops the outer loop.
#[allow(clippy::too_many_arguments)]
pub fn spread_pass(
    anchors: &mut [Anchor],
    targets: &[i32],
    row: (i32, i32),
    spring: f32,
    repel: f32,
    damper: f32,
    site: i32,
    snap: &dyn Fn(i32) -> i32,
) -> bool {
    let mut violations = false;
    for i in 0..anchors.len() {
        let curr = anchors[i];
        let half = curr.width / 2;
        let prev_pos = if i == 0 { row.0 + half } else { anchors[i - 1].centre };
        let next_pos = if i + 1 == anchors.len() { row.1 - half } else { anchors[i + 1].centre };

        let overlap_prev = if i == 0 { 0 } else { curr.overlap(&anchors[i - 1]) };
        let overlap_next =
            if i + 1 == anchors.len() { 0 } else { anchors[i + 1].overlap(&curr) };
        if overlap_prev > 0 || overlap_next > 0 {
            violations = true;
        }

        let move_by = step_move(
            targets[i] - curr.centre,
            spring,
            overlap_prev,
            overlap_next,
            repel,
            damper,
            site,
        );
        let want = snap(curr.centre + move_by - half) + half;
        anchors[i].set_location(want.clamp(prev_pos, next_pos) - half);
    }
    violations
}

#[cfg(test)]
mod tests {
    use super::*;

    fn anchors(spec: &[(i32, i32)]) -> Vec<Anchor> {
        spec.iter().map(|&(p, w)| Anchor::at(p, w)).collect()
    }

    #[test]
    fn overlap_is_measured_one_way_only() {
        // ⚠️ `a.overlap(b)` asks how far b reaches past a's start — meaningful only when b is the
        // pad BEFORE a. Reading it symmetrically reports collisions between pads that merely
        // sit near each other.
        let a = Anchor::at(100, 50); // 100..150
        let b = Anchor::at(120, 50); // 120..170
        assert_eq!(a.overlap(&b), 70, "b reaches 70 past a's start");
        assert_eq!(b.overlap(&a), 30, "and the reverse is a different number");
        // A pad genuinely behind and clear of it gives zero.
        let behind = Anchor::at(0, 50); // 0..50, ends before a starts
        assert_eq!(a.overlap(&behind), 0, "clear behind means no push");
        // ⚠️ And a pad AHEAD gives a number that means nothing — which is why the loop only ever
        // asks in the one direction. This assertion exists to pin that hazard, not to bless it.
        let ahead = Anchor::at(400, 50);
        assert_eq!(a.overlap(&ahead), 350, "nonsense, and never asked for");
    }

    #[test]
    fn any_non_zero_force_moves_at_least_one_site() {
        // ⚠️ A tiny force still moves a whole site, because the damped move rounds UP. Rounding
        // down would stall the loop with the row still overlapping.
        assert_eq!(step_move(1, 0.1, 0, 0, 0.5, 0.2, 1000), 1000);
        assert_eq!(step_move(0, 0.1, 0, 0, 0.5, 0.2, 1000), 0, "no force, no move");
    }

    #[test]
    fn an_overlap_pushes_by_the_overlap_plus_a_whole_site() {
        // The pad before overlaps by 500 with a 1000 site: the push is forward.
        assert!(step_move(0, 0.0, 500, 0, 0.5, 0.2, 1000) > 0);
        // The pad after overlaps: the push is backward.
        assert!(step_move(0, 0.0, 0, 500, 0.5, 0.2, 1000) < 0);
    }

    #[test]
    fn a_pass_separates_two_overlapping_pads() {
        let mut a = anchors(&[(1000, 1000), (1500, 1000)]);
        let targets = [1500, 2000];
        let had = spread_pass(&mut a, &targets, (0, 10_000), 0.1, 0.5, 0.2, 100, &|p| p);
        assert!(had, "the pass reports the overlap it found");
        assert!(a[1].min >= a[0].min, "and the order is preserved");
    }

    #[test]
    fn a_settled_row_reports_no_violation() {
        let mut a = anchors(&[(0, 1000), (2000, 1000), (4000, 1000)]);
        let targets = [500, 2500, 4500];
        assert!(!spread_pass(&mut a, &targets, (0, 10_000), 0.0, 0.5, 0.2, 100, &|p| p));
    }

    #[test]
    fn a_pad_is_clamped_between_its_neighbours() {
        // ⚠️ The middle pad cannot pass either neighbour however hard the spring pulls.
        let mut a = anchors(&[(0, 100), (1000, 100), (2000, 100)]);
        let targets = [50, 900_000, 2050];
        spread_pass(&mut a, &targets, (0, 10_000), 1.0, 0.5, 1.0, 10, &|p| p);
        assert!(a[1].centre <= a[2].centre, "never past the pad ahead");
        assert!(a[1].centre >= a[0].centre, "nor behind the one before");
    }

    #[test]
    fn an_ideal_position_is_the_mean_of_the_bumps_served() {
        assert_eq!(ideal_position(&[100, 300]), Some(200));
        assert_eq!(ideal_position(&[100]), Some(100));
        assert_eq!(ideal_position(&[]), None, "a pad serving nothing has no preference");
        // ⚠️ The mean, not the median: two bumps clustered low still leave the pad pulled high.
        assert_eq!(ideal_position(&[0, 10, 800]), Some(270));
    }

    #[test]
    fn an_unconstrained_pad_takes_a_crude_starting_point() {
        let prior = [500, 700, 900];
        assert_eq!(unconstrained_start(0, 4, &prior, (10, 990)), 10, "first takes the row start");
        assert_eq!(unconstrained_start(3, 4, &prior, (10, 990)), 990, "last takes the row end");
        assert_eq!(unconstrained_start(1, 4, &prior, (10, 990)), 500, "second copies the first");
        assert_eq!(unconstrained_start(2, 4, &prior, (10, 990)), 600, "then the average of two");
    }

    #[test]
    fn a_violation_pools_both_pads_to_their_weighted_mean() {
        let mut p = [300, 100];
        let mut w = [1.0, 1.0];
        assert!(pool_adjacent_violators(&mut p, &mut w));
        assert_eq!(p, [200, 200]);
    }

    #[test]
    fn the_weight_update_is_asymmetric_and_that_is_deliberate() {
        // ⚠️ `w[i] += w[i-1]` then `w[i-1] += w[i]` — the second sees the first's result.
        let mut p = [300, 100];
        let mut w = [1.0, 1.0];
        pool_adjacent_violators(&mut p, &mut w);
        assert_eq!(w, [3.0, 2.0], "not [2.0, 2.0], which the description would suggest");
    }

    #[test]
    fn an_ordered_row_is_left_alone() {
        let mut p = [100, 200, 300];
        let mut w = [1.0; 3];
        assert!(!pool_adjacent_violators(&mut p, &mut w));
        assert_eq!(p, [100, 200, 300]);
    }

    #[test]
    fn the_spring_holds_then_fades_then_stops() {
        assert_eq!(forces(0).0, SPRING_START, "full strength at the start");
        assert_eq!(forces(SPRING_FADE_FROM).0, SPRING_START, "⚠️ still full AT the threshold");
        assert!(forces(SPRING_FADE_FROM + 1).0 < SPRING_START, "fading one step later");
        assert!(forces(SPRING_FADE_TO - 1).0 > 0.0, "a trace left just before the end");
        // ⚠️ The linear fade reaches exactly zero AT the threshold, so the `> threshold` test that
        // forces it to zero never actually changes anything. Two mechanisms, one effect.
        assert_eq!(forces(SPRING_FADE_TO).0, 0.0, "already nothing at the threshold");
        assert_eq!(forces(SPRING_FADE_TO + 1).0, 0.0, "and nothing after");
        // Repulsion does not vary: both ends of its schedule are the same.
        assert_eq!(forces(0).1, forces(MAX_ITERATIONS).1);
    }

    #[test]
    fn a_clear_position_is_left_where_it_is() {
        assert_eq!(nearest_legal(500, None, 50, (0, 1000)), 500);
    }

    #[test]
    fn a_blocked_pad_steps_to_whichever_side_is_nearer() {
        // Obstruction 400..600, half width 50: the ways out are 350 and 650.
        assert_eq!(nearest_legal(450, Some((400, 600)), 50, (0, 1000)), 350);
        assert_eq!(nearest_legal(580, Some((400, 600)), 50, (0, 1000)), 650);
    }

    #[test]
    fn at_the_rows_ends_only_one_way_out_exists() {
        // ⚠️ Stepping back would leave the row, so it must go forward however near the other side.
        assert_eq!(nearest_legal(20, Some((0, 100)), 50, (0, 1000)), 150);
        assert_eq!(nearest_legal(980, Some((900, 1000)), 50, (0, 1000)), 850);
    }
}
