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

#[cfg(test)]
mod tests {
    use super::*;

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
