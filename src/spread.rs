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
    pool_pass(positions, weights)
}

/// **SP3b** — one round of the regression: pool violators, **then legalise**.
///
/// ⚠️ The legalisation is not an afterthought and not the spreading stage's job. A pooled position
/// can land on an obstruction, and the round is not finished until every position is somewhere a
/// pad may actually sit. Leaving it out hands the spreading stage a row that starts illegal, and
/// the spread has no mechanism to notice — it resolves *overlaps between pads*, not pads sitting on
/// blockages. The result is a pad committed onto an obstruction and then refused at the very last
/// step, which reads as a placer that cannot place rather than a regression that did not finish.
///
/// Returns whether anything moved, which is what ends the outer loop.
/// ⚠️ `legal` is expected to return a position **already bounded to the row for that pad** — the
/// bounds are inset by half the pad's width, so only the caller knows them. An obstruction reaching
/// past both row ends otherwise makes "step to the far side" land outside the die, and the value
/// becomes the spring target for every later iteration.
pub fn pool_round(
    positions: &mut [i32],
    weights: &mut [f32],
    legal: &dyn Fn(usize, i32) -> i32,
) -> bool {
    let mut updated = pool_pass(positions, weights);
    for i in 0..positions.len() {
        let fixed = legal(i, positions[i]);
        if fixed != positions[i] {
            positions[i] = fixed;
            updated = true;
        }
    }
    updated
}

fn pool_pass(positions: &mut [i32], weights: &mut [f32]) -> bool {
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
    nearest_legal_side(target, obstruction, half_width, row, false, false)
}

/// **SP5b** — the same, but able to insist on a side.
///
/// ⚠️ `round_down` and `round_up` override the "nearer wins" rule, and the tunnelling logic uses
/// them to ask a directed question: *where would this pad land if it kept going the way it is
/// already travelling?* Answering with the nearer side instead makes a pad trying to jump an
/// obstruction settle back on the side it came from.
///
/// The row-end checks come **first** and are not overridden: at an end there is only one way out,
/// whichever side was asked for.
pub fn nearest_legal_side(
    target: i32,
    obstruction: Option<(i32, i32)>,
    half_width: i32,
    row: (i32, i32),
    round_down: bool,
    round_up: bool,
) -> i32 {
    let Some((lo, hi)) = obstruction else { return target };
    let (start, end) = (lo - half_width, hi + half_width);
    if start < row.0 {
        return end;
    }
    if end > row.1 {
        return start;
    }
    if round_down {
        return start;
    }
    if round_up {
        return end;
    }
    if (target - start) < (end - target) {
        start
    } else {
        end
    }
}

/// Where a pad ends up when it has to get past an obstruction, and where it *wanted* to be.
///
/// ⚠️ The two differ when the jump could not be completed — the pad is stuck on the near side, and
/// the caller uses the difference to push whatever is in the way. Returning only the position loses
/// the information that a push is needed, and the row never opens up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tunnel {
    pub position: i32,
    pub ideal: i32,
}

impl Tunnel {
    pub fn stuck(&self) -> bool {
        self.position != self.ideal
    }
}

/// **SP9** — get a pad past an obstruction, or work out that it cannot.
///
/// `blocked` reports the obstruction a pad centred at a position would hit. The pad asks where it
/// would land continuing in its current direction. If that is beyond the neighbour bounding it,
/// it checks whether the boundary itself is clear: if not, it settles on the near edge of the
/// obstruction when that is still within bounds, and otherwise **stays exactly where it is**.
#[allow(clippy::too_many_arguments)]
pub fn tunnel_position(
    target: i32,
    moving_up: bool,
    low_bound: i32,
    curr: i32,
    high_bound: i32,
    half_width: i32,
    row: (i32, i32),
    blocked: &dyn Fn(i32) -> Option<(i32, i32)>,
) -> Tunnel {
    let ideal =
        nearest_legal_side(target, blocked(target), half_width, row, !moving_up, moving_up);
    if ideal == target {
        return Tunnel { position: target, ideal: target };
    }
    let (bound, beyond) =
        if moving_up { (high_bound, ideal > high_bound) } else { (low_bound, ideal < low_bound) };
    if beyond && blocked(bound).is_some() {
        let next = nearest_legal_side(target, blocked(target), half_width, row, true, false);
        let reachable = if moving_up { next <= high_bound } else { next >= low_bound };
        if reachable {
            return Tunnel { position: next, ideal };
        }
        return Tunnel { position: curr, ideal };
    }
    Tunnel { position: ideal, ideal }
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
    blocked: &dyn Fn(usize, i32) -> Option<(i32, i32)>,
    watch: &mut dyn FnMut(usize, i32, i32, i32, i32),
) -> bool {
    let mut violations = false;
    // Pads that tried to jump an obstruction and could not, with where they wanted to be.
    let mut stuck: Vec<(usize, i32)> = Vec::new();
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
        // ⚠️ The jump is decided on the CLAMPED move, not the raw one: a pad may only try to get
        // past an obstruction that lies within reach of its neighbours.
        let mut move_by = move_by;
        if move_by != 0 {
            // Same nesting as below, and for the same reason: the bounds can invert.
            let check = prev_pos.max(next_pos.min(curr.centre + move_by));
            let probe = |c: i32| blocked(i, c);
            let t = tunnel_position(
                check,
                move_by > 0,
                prev_pos,
                curr.centre,
                next_pos,
                half,
                row,
                &probe,
            );
            move_by = t.position - curr.centre;
            if t.stuck() {
                stuck.push((i, t.ideal));
            }
        }

        // ⚠️ `max(prev, min(next, x))`, **not** `clamp`. On a congested row a pass can leave a
        // pad's predecessor ahead of its successor, and `clamp` panics when its bounds are
        // inverted. The reference's nesting yields `prev` in that case and carries on — the
        // difference is a crash on a real design versus a placement that keeps going.
        let want = snap(curr.centre + move_by - half) + half;
        anchors[i].set_location(prev_pos.max(next_pos.min(want)) - half);
        watch(i, curr.centre, anchors[i].centre, prev_pos, next_pos);
    }

    // **SP10** — a pad that could not jump pushes the pads in its way.
    //
    // ⚠️ Without this the row deadlocks: the blocked pad has nowhere to go, the pads under the
    // obstruction have no reason to move, and the loop runs to its iteration limit reporting a
    // violation it can never clear.
    for (i, ideal) in stuck {
        let delta = ideal - anchors[i].centre;
        let push = if delta < 0 { -1 } else { 1 }
            * ((damper * delta.abs() as f32) / site as f32).ceil() as i32
            * site;
        let mut last = i;
        let mut targets_to_push: Vec<usize> = Vec::new();
        if delta > 0 {
            for j in (i + 1)..anchors.len() {
                if anchors[j].centre <= ideal {
                    targets_to_push.push(j);
                    last = j;
                }
            }
        } else {
            for j in (0..i).rev() {
                if anchors[j].centre >= ideal {
                    targets_to_push.push(j);
                    last = j;
                }
            }
        }
        let bound = if delta > 0 {
            if last + 1 == anchors.len() { row.1 } else { anchors[last + 1].centre }
        } else if last == 0 {
            row.0
        } else {
            anchors[last - 1].centre
        };
        for j in targets_to_push {
            let want = if push < 0 {
                bound.max(anchors[j].centre + push)
            } else {
                bound.min(anchors[j].centre + push)
            };
            let half = anchors[j].width / 2;
            anchors[j].set_location(snap(want - half));
        }
        violations = true;
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
    fn inverted_neighbour_bounds_do_not_panic() {
        // ⚠️ A congested row can leave a pad's predecessor ahead of its successor. `clamp` panics
        // on inverted bounds; the reference's nesting yields the lower bound and carries on.
        let mut a = anchors(&[(5000, 100), (0, 100), (0, 100)]);
        let targets = [0, 0, 0];
        spread_pass(&mut a, &targets, (0, 10_000), 0.1, 0.5, 0.2, 100, &|p| p, &|_, _| None,
                    &mut |_, _, _, _, _| {});
    }

    #[test]
    fn a_pass_separates_two_overlapping_pads() {
        let mut a = anchors(&[(1000, 1000), (1500, 1000)]);
        let targets = [1500, 2000];
        let had = spread_pass(&mut a, &targets, (0, 10_000), 0.1, 0.5, 0.2, 100, &|p| p, &|_, _| None, &mut |_, _, _, _, _| {});
        assert!(had, "the pass reports the overlap it found");
        assert!(a[1].min >= a[0].min, "and the order is preserved");
    }

    #[test]
    fn a_settled_row_reports_no_violation() {
        let mut a = anchors(&[(0, 1000), (2000, 1000), (4000, 1000)]);
        let targets = [500, 2500, 4500];
        assert!(!spread_pass(&mut a, &targets, (0, 10_000), 0.0, 0.5, 0.2, 100, &|p| p, &|_, _| None, &mut |_, _, _, _, _| {}));
    }

    #[test]
    fn a_pad_is_clamped_between_its_neighbours() {
        // ⚠️ The middle pad cannot pass either neighbour however hard the spring pulls.
        let mut a = anchors(&[(0, 100), (1000, 100), (2000, 100)]);
        let targets = [50, 900_000, 2050];
        spread_pass(&mut a, &targets, (0, 10_000), 1.0, 0.5, 1.0, 10, &|p| p, &|_, _| None, &mut |_, _, _, _, _| {});
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
    fn a_directed_legalisation_insists_on_its_side() {
        // ⚠️ "Nearer wins" would put this back on the low side; the jump needs the high one.
        let obs = Some((400, 600));
        assert_eq!(nearest_legal_side(450, obs, 50, (0, 1000), false, true), 650, "forced up");
        assert_eq!(nearest_legal_side(580, obs, 50, (0, 1000), true, false), 350, "forced down");
        // At a row end there is only one way out, whichever side was asked for.
        assert_eq!(nearest_legal_side(20, Some((0, 100)), 50, (0, 1000), true, false), 150);
    }

    #[test]
    fn a_clear_target_needs_no_tunnel() {
        let t = tunnel_position(500, true, 0, 400, 1000, 50, (0, 1000), &|_| None);
        assert_eq!(t, Tunnel { position: 500, ideal: 500 });
        assert!(!t.stuck());
    }

    #[test]
    fn a_pad_jumps_an_obstruction_when_there_is_room_beyond_it() {
        let obs = |p: i32| if (400..=600).contains(&p) { Some((400, 600)) } else { None };
        let t = tunnel_position(500, true, 0, 300, 900, 50, (0, 1000), &obs);
        assert_eq!(t.position, 650, "landed past it");
        assert!(!t.stuck());
    }

    #[test]
    fn a_pad_blocked_in_by_its_neighbour_stays_put_and_reports_it() {
        // ⚠️ The neighbour is at 620, so the far side of the obstruction (650) is out of reach and
        // the boundary itself is inside the obstruction. The pad must not move, and the caller has
        // to learn that a push is needed.
        let obs = |p: i32| if (400..=700).contains(&p) { Some((400, 600)) } else { None };
        let t = tunnel_position(500, true, 0, 300, 620, 50, (0, 1000), &obs);
        assert!(t.stuck(), "reported as stuck");
        assert_eq!(t.ideal, 650, "and remembers where it wanted to be");
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
