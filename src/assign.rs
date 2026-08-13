// SPDX-License-Identifier: Apache-2.0
//! Assigning a net to a bump.
//!
//! A bump carries one net down to the die. Assigning it connects the bump's own terminals to that
//! net, optionally ties a named pad terminal to the same net, and gives the net a block terminal
//! on the bump's top layer — the point the package attaches to.
//!
//! Nothing here touches a database.

type Rect = (i32, i32, i32, i32);

/// One terminal of a bump, with its shapes in **master** coordinates.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BumpTerm {
    pub name: String,
    pub net: Option<String>,
    /// `(layer name, routing level, rectangle)`.
    pub shapes: Vec<(String, i32, Rect)>,
}

/// What assigning implies. Every field is in the order it must be applied.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Assignment {
    /// Bump terminals to attach to the net.
    pub connect: Vec<String>,
    /// The named pad terminal, if it needs attaching too.
    pub terminal: Option<String>,
    /// The block terminal to make: layer and shape, still in **master** coordinates.
    pub bterm: Option<(String, Rect)>,
}

/// Why an assignment was refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Refused {
    /// The named pad terminal already belongs to a different net.
    WrongNet { terminal: String, net: String },
}

/// **B1** — work out what assigning `net` to this bump means.
///
/// `master_box` is the bump master's placement boundary, whose centre picks the shape.
pub fn assign(
    terms: &[BumpTerm],
    net: &str,
    master_box: Rect,
    terminal: Option<(&str, Option<&str>)>,
) -> Result<Assignment, Refused> {
    let mut out = Assignment::default();

    for t in terms {
        if t.net.as_deref() != Some(net) {
            out.connect.push(t.name.clone());
        }
    }

    // ⚠️ The named terminal is tied to the net once, not once per bump terminal. A bump with two
    // pins would otherwise connect it twice.
    if let Some((name, current)) = terminal {
        match current {
            None => out.terminal = Some(name.to_string()),
            Some(n) if n == net => {}
            Some(n) => {
                return Err(Refused::WrongNet { terminal: name.to_string(), net: n.to_string() })
            }
        }
    }

    out.bterm = top_shape(terms, master_box);
    Ok(out)
}

/// **B2** — the shape the block terminal is made from.
///
/// Shapes are gathered from every terminal, keeping those on the highest routing layer seen *so
/// far* — a layer at least as high as the running best is kept and becomes the new best.
///
/// ⚠️ **Shapes from a lower layer already gathered are never dropped.** The reference reassigns its
/// running layer before testing whether to discard, so the discard never fires. Writing the
/// "obvious" version — clear the list when a higher layer appears — is a *different* engine, and
/// on a bump whose pins climb through several layers it picks a different shape.
///
/// Among the gathered shapes, the one containing the master's centre wins, and if several do, the
/// **last** in `(xlo, ylo, xhi, yhi)` order. With none, the **first** in that order.
pub fn top_shape(terms: &[BumpTerm], master_box: Rect) -> Option<(String, Rect)> {
    let mut best_level: Option<i32> = None;
    let mut layer = String::new();
    let mut shapes: Vec<Rect> = Vec::new();

    for t in terms {
        for (name, level, rect) in &t.shapes {
            if best_level.is_none_or(|b| b <= *level) {
                best_level = Some(*level);
                layer = name.clone();
                if !shapes.contains(rect) {
                    shapes.push(*rect);
                }
            }
        }
    }
    best_level?;

    // The set order the reference relies on: lexicographic, and duplicates already removed.
    shapes.sort_unstable();
    let centre = ((master_box.0 + master_box.2) / 2, (master_box.1 + master_box.3) / 2);
    let chosen = shapes
        .iter()
        .filter(|r| contains(**r, centre))
        .next_back()
        .or_else(|| shapes.first())?;
    Some((layer, *chosen))
}

/// Closed containment: a point on the boundary is inside.
fn contains(r: Rect, p: (i32, i32)) -> bool {
    r.0 <= p.0 && p.0 <= r.2 && r.1 <= p.1 && p.1 <= r.3
}

#[cfg(test)]
mod tests {
    use super::*;

    fn term(name: &str, net: Option<&str>, shapes: &[(&str, i32, Rect)]) -> BumpTerm {
        BumpTerm {
            name: name.into(),
            net: net.map(str::to_string),
            shapes: shapes.iter().map(|(l, v, r)| (l.to_string(), *v, *r)).collect(),
        }
    }

    #[test]
    fn every_bump_terminal_not_already_on_the_net_is_connected() {
        let t = vec![
            term("PAD", None, &[("m10", 10, (0, 0, 100, 100))]),
            term("PAD2", Some("VDD"), &[("m10", 10, (0, 0, 100, 100))]),
            term("PAD3", Some("other"), &[("m10", 10, (0, 0, 100, 100))]),
        ];
        let a = assign(&t, "VDD", (0, 0, 100, 100), None).unwrap();
        assert_eq!(a.connect, vec!["PAD", "PAD3"], "PAD2 is already there");
    }

    #[test]
    fn a_free_pad_terminal_joins_the_net() {
        let t = vec![term("PAD", None, &[("m10", 10, (0, 0, 10, 10))])];
        let a = assign(&t, "VDD", (0, 0, 10, 10), Some(("u_pad/PAD", None))).unwrap();
        assert_eq!(a.terminal.as_deref(), Some("u_pad/PAD"));
    }

    #[test]
    fn a_pad_terminal_already_on_the_net_needs_no_second_connection() {
        let t = vec![term("PAD", None, &[("m10", 10, (0, 0, 10, 10))])];
        let a = assign(&t, "VDD", (0, 0, 10, 10), Some(("u_pad/PAD", Some("VDD")))).unwrap();
        assert_eq!(a.terminal, None);
    }

    #[test]
    fn a_pad_terminal_on_another_net_is_refused_rather_than_rewired() {
        let t = vec![term("PAD", None, &[("m10", 10, (0, 0, 10, 10))])];
        let e = assign(&t, "VDD", (0, 0, 10, 10), Some(("u_pad/PAD", Some("VSS")))).unwrap_err();
        assert_eq!(e, Refused::WrongNet { terminal: "u_pad/PAD".into(), net: "VSS".into() });
    }

    #[test]
    fn the_highest_layer_wins() {
        let t = vec![term(
            "PAD",
            None,
            &[("m5", 5, (0, 0, 10, 10)), ("m10", 10, (20, 20, 30, 30))],
        )];
        let (layer, _) = top_shape(&t, (0, 0, 100, 100)).unwrap();
        assert_eq!(layer, "m10");
    }

    #[test]
    fn a_lower_layer_after_a_higher_one_is_ignored() {
        let t = vec![term(
            "PAD",
            None,
            &[("m10", 10, (20, 20, 30, 30)), ("m5", 5, (0, 0, 10, 10))],
        )];
        let (layer, rect) = top_shape(&t, (25, 25, 25, 25)).unwrap();
        assert_eq!((layer.as_str(), rect), ("m10", (20, 20, 30, 30)));
    }

    #[test]
    fn shapes_gathered_from_a_lower_layer_are_kept_when_a_higher_one_arrives() {
        // ⚠️ The behaviour that looks like a bug in the reference and must be reproduced: the
        // earlier, lower-layer shape stays in the running. Here it is the one containing the
        // centre, so it is the one chosen even though the LAYER reported is the higher one.
        let t = vec![term(
            "PAD",
            None,
            &[("m5", 5, (0, 0, 100, 100)), ("m10", 10, (200, 200, 300, 300))],
        )];
        let (layer, rect) = top_shape(&t, (0, 0, 100, 100)).unwrap();
        assert_eq!(layer, "m10", "the layer is the highest seen");
        assert_eq!(rect, (0, 0, 100, 100), "but the shape is the one over the centre");
    }

    #[test]
    fn with_nothing_over_the_centre_the_first_shape_in_order_is_used() {
        let t = vec![term(
            "PAD",
            None,
            &[("m10", 10, (500, 500, 600, 600)), ("m10", 10, (200, 200, 300, 300))],
        )];
        let (_, rect) = top_shape(&t, (0, 0, 10, 10)).unwrap();
        assert_eq!(rect, (200, 200, 300, 300), "lowest by (xlo, ylo, xhi, yhi)");
    }

    #[test]
    fn with_several_shapes_over_the_centre_the_last_in_order_is_used() {
        let t = vec![term(
            "PAD",
            None,
            &[("m10", 10, (0, 0, 100, 100)), ("m10", 10, (10, 10, 90, 90))],
        )];
        let (_, rect) = top_shape(&t, (0, 0, 100, 100)).unwrap();
        assert_eq!(rect, (10, 10, 90, 90));
    }

    #[test]
    fn shapes_from_both_terminals_of_a_two_pin_bump_are_gathered() {
        let t = vec![
            term("PAD1", None, &[("m10", 10, (0, 0, 40, 40))]),
            term("PAD2", None, &[("m10", 10, (60, 60, 100, 100))]),
        ];
        let a = assign(&t, "VDD", (0, 0, 100, 100), None).unwrap();
        assert_eq!(a.connect.len(), 2);
        // The centre (50, 50) is in neither, so the first in order wins.
        assert_eq!(a.bterm.unwrap().1, (0, 0, 40, 40));
    }

    #[test]
    fn a_bump_with_no_shapes_makes_no_terminal() {
        let t = vec![term("PAD", None, &[])];
        assert_eq!(assign(&t, "VDD", (0, 0, 10, 10), None).unwrap().bterm, None);
    }
}
