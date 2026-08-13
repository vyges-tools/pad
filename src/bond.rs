// SPDX-License-Identifier: Apache-2.0
//! Bond pads — the cell that sits on top of a pad and gives the package something to attach to.
//!
//! One bond pad per pad, at a fixed offset, on the pad's own orientation. Where the pad's terminal
//! carries a net, the bond pad joins it and the net gains a block terminal shape on the bond
//! layer: that shape is the design's connection to the outside world.
//!
//! Nothing here touches a database.

use crate::orient::Orient;

type Rect = (i32, i32, i32, i32);

/// The default instance name prefix.
pub const DEFAULT_PREFIX: &str = "IO_BOND_";

/// Where a bond pad goes, and what it is called.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bond {
    pub name: String,
    pub master: String,
    /// ⚠️ The transform **origin**, not the bounding box's lower-left. The two differ for every
    /// orientation but `R0`, and a bond pad is routinely rotated.
    pub origin: (i32, i32),
    pub orient: Orient,
}

/// **D1** — is this master a bond pad?
///
/// ⚠️ **Exact**, where the bump test is a prefix. odb reports `CLASS COVER` as `"COVER"` and
/// `CLASS COVER BUMP` as `"COVER BUMP"`; a bond pad must be the former and must *not* match the
/// latter. The two tests look inconsistent and are not: they are asking different questions of the
/// same field.
pub fn is_bond_master(class: &str) -> bool {
    class.trim() == "COVER"
}

/// **D2** — the bond layer and the shape on it.
///
/// The pin shape on the **highest routing layer** the master declares — that is the face the
/// package meets. Shapes with no routing level are skipped: they are not routable metal.
///
/// ⚠️ Ties keep the **first** shape seen, because the comparison is strictly "higher". A master
/// with two shapes on its top layer contributes only the first, which is what the reference warns
/// about and then does anyway.
pub fn bond_shape(pins: &[(String, i32, Rect)]) -> Option<(String, Rect)> {
    let mut best: Option<(&str, i32, Rect)> = None;
    for (layer, level, rect) in pins {
        if *level == 0 {
            continue;
        }
        if best.is_none_or(|(_, b, _)| b < *level) {
            best = Some((layer, *level, *rect));
        }
    }
    best.map(|(l, _, r)| (l.to_string(), r))
}

/// **D3** — where one bond pad goes.
///
/// The offset is given in the *pad's* frame, so it turns with the pad: a pad rotated a quarter
/// turn carries its bond pad round with it rather than leaving it beside where the pad used to be.
/// The bond pad's own orientation is the pad's, with the requested rotation applied on top.
pub fn place(
    pad_name: &str,
    pad_origin: (i32, i32),
    pad_orient: Orient,
    master: &str,
    offset: (i32, i32),
    rotation: Orient,
    prefix: &str,
) -> Bond {
    let (dx, dy) = pad_orient.apply(offset.0, offset.1);
    Bond {
        name: format!("{prefix}{pad_name}"),
        master: master.to_string(),
        origin: (pad_origin.0 + dx, pad_origin.1 + dy),
        orient: pad_orient.concat(rotation),
    }
}

/// **D6** — the bond shape where the placed bond pad puts it.
///
/// The master rectangle turned by the instance's orientation and moved to its origin — the
/// instance's own transform, applied to a rectangle.
///
/// ⚠️ Rotated about the master's `(0, 0)`, **not** normalised against the master's size the way
/// [`crate::clearance::transform`] does. A bond pad's pin rectangle is centred on the origin and
/// runs negative, so size-normalising it would move it. And shifting it without rotating leaves
/// the shape a quarter turn out on every side row, where the pads face sideways.
pub fn pin_shape(rect: Rect, orient: Orient, origin: (i32, i32)) -> Rect {
    let (ax, ay) = orient.apply(rect.0, rect.1);
    let (bx, by) = orient.apply(rect.2, rect.3);
    (
        ax.min(bx) + origin.0,
        ay.min(by) + origin.1,
        ax.max(bx) + origin.0,
        ay.max(by) + origin.1,
    )
}

/// **D4** — match a list of instance names against the patterns the caller gave.
///
/// ⚠️ `*` and `?` only. Character classes are deliberately not supported: a pattern like
/// `req_msg[0]` is an ordinary instance name in this domain, and reading `[0]` as a class makes it
/// match `req_msg0` and *not* itself — silently selecting the wrong cells.
pub fn matching<'a>(names: &'a [String], patterns: &[String]) -> Vec<&'a String> {
    names.iter().filter(|n| patterns.iter().any(|p| glob(p, n))).collect()
}

fn glob(pattern: &str, name: &str) -> bool {
    let (p, n): (Vec<char>, Vec<char>) = (pattern.chars().collect(), name.chars().collect());
    // Classic two-pointer wildcard match: remember where the last `*` was and backtrack to it.
    let (mut pi, mut ni, mut star, mut mark) = (0, 0, None, 0);
    while ni < n.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == n[ni]) {
            pi += 1;
            ni += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = Some(pi);
            mark = ni;
            pi += 1;
        } else if let Some(s) = star {
            pi = s + 1;
            mark += 1;
            ni = mark;
        } else {
            return false;
        }
    }
    p[pi..].iter().all(|&c| c == '*')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bond_master_is_a_plain_cover_cell() {
        assert!(is_bond_master("COVER"));
        // ⚠️ The inverse of the bump test, on purpose.
        assert!(!is_bond_master("COVER BUMP"), "a bump is not a bond pad");
        assert!(!is_bond_master("PAD"));
    }

    #[test]
    fn the_bond_shape_is_the_highest_routing_layer() {
        let pins = vec![
            ("metal1".to_string(), 1, (0, 0, 10, 10)),
            ("metal10".to_string(), 10, (-70, -35, 70, 35)),
            ("metal5".to_string(), 5, (0, 0, 20, 20)),
        ];
        assert_eq!(bond_shape(&pins), Some(("metal10".into(), (-70, -35, 70, 35))));
    }

    #[test]
    fn a_shape_with_no_routing_level_is_not_metal_and_is_skipped() {
        let pins = vec![
            ("OVERLAP".to_string(), 0, (0, 0, 99, 99)),
            ("metal1".to_string(), 1, (0, 0, 10, 10)),
        ];
        assert_eq!(bond_shape(&pins), Some(("metal1".into(), (0, 0, 10, 10))));
        assert_eq!(bond_shape(&pins[..1]), None, "nothing routable at all");
    }

    #[test]
    fn a_tie_on_the_top_layer_keeps_the_first_shape() {
        let pins = vec![
            ("metal10".to_string(), 10, (0, 0, 10, 10)),
            ("metal10".to_string(), 10, (50, 50, 60, 60)),
        ];
        assert_eq!(bond_shape(&pins), Some(("metal10".into(), (0, 0, 10, 10))));
    }

    #[test]
    fn the_offset_turns_with_the_pad() {
        // ⚠️ The offset is in the pad's frame. On an unrotated pad it is applied as given.
        let b = place("u_pad", (1000, 2000), Orient::R0, "PAD", (-10, 150), Orient::R0, "IO_BOND_");
        assert_eq!(b.origin, (990, 2150));
        assert_eq!(b.name, "IO_BOND_u_pad");
        assert_eq!(b.orient, Orient::R0);
        // A pad turned a quarter turn carries the offset round with it.
        let t = place("u_pad", (1000, 2000), Orient::R90, "PAD", (-10, 150), Orient::R0, "IO_BOND_");
        assert_ne!(t.origin, b.origin, "the same offset lands elsewhere");
        assert_eq!(t.orient, Orient::R90, "and the bond pad faces the same way as the pad");
    }

    #[test]
    fn the_requested_rotation_composes_onto_the_pads_own() {
        let b = place("p", (0, 0), Orient::MX, "PAD", (0, 0), Orient::R90, "B_");
        assert_eq!(b.orient, Orient::MX.concat(Orient::R90));
    }

    #[test]
    fn the_bond_shape_turns_with_the_bond_pad() {
        // A rectangle centred on the master origin, wider than it is tall.
        let r = (-70, -35, 70, 35);
        assert_eq!(pin_shape(r, Orient::R0, (1000, 2000)), (930, 1965, 1070, 2035));
        // ⚠️ A quarter turn makes it taller than it is wide. Shifting without rotating would
        // leave it wide, which is the shape being in the wrong place on every side row.
        assert_eq!(pin_shape(r, Orient::R90, (1000, 2000)), (965, 1930, 1035, 2070));
        // Mirroring a symmetric rectangle about its own centre changes nothing.
        assert_eq!(pin_shape(r, Orient::MY, (1000, 2000)), (930, 1965, 1070, 2035));
    }

    #[test]
    fn the_bond_shape_is_not_normalised_against_the_master_size() {
        // ⚠️ The rectangle runs negative on purpose: it is centred on the origin. Treating it the
        // way an ordinary master shape is treated would slide it by half the cell.
        assert_eq!(pin_shape((-10, -10, 10, 10), Orient::R0, (0, 0)), (-10, -10, 10, 10));
    }

    #[test]
    fn patterns_select_by_star_and_question_mark_only() {
        let names: Vec<String> =
            ["IO_EAST_SIDE", "IO_WEST_SIDE", "u_pad_inner", "u_pad_outer", "other"]
                .iter()
                .map(|s| s.to_string())
                .collect();
        let pick = |p: &str| {
            matching(&names, &[p.to_string()]).iter().map(|s| s.as_str()).collect::<Vec<_>>()
        };
        assert_eq!(pick("IO_*"), vec!["IO_EAST_SIDE", "IO_WEST_SIDE"]);
        assert_eq!(pick("u_*_inner"), vec!["u_pad_inner"]);
        assert_eq!(pick("*"), names.iter().map(|s| s.as_str()).collect::<Vec<_>>());
        assert_eq!(pick("other"), vec!["other"]);
        assert!(pick("IO_?").is_empty());
    }

    #[test]
    fn a_bracket_is_an_ordinary_character_not_a_class() {
        // ⚠️ The trap this exists to avoid: as a character class, `req_msg[0]` matches `req_msg0`
        // and not `req_msg[0]` — the wrong cells, silently.
        let names: Vec<String> =
            ["req_msg[0]", "req_msg0"].iter().map(|s| s.to_string()).collect();
        let picked = matching(&names, &["req_msg[0]".to_string()]);
        assert_eq!(picked, vec!["req_msg[0]"]);
    }

    #[test]
    fn several_patterns_select_the_union_once_each() {
        let names: Vec<String> =
            ["a1", "a2", "b1"].iter().map(|s| s.to_string()).collect();
        let picked = matching(&names, &["a*".to_string(), "*1".to_string()]);
        assert_eq!(picked, vec!["a1", "a2", "b1"], "no duplicates, input order");
    }
}
