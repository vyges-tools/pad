// SPDX-License-Identifier: Apache-2.0
//! Whether a cell can go somewhere — and it is not a question about bounding boxes.
//!
//! A pad ring is dense, and two cells whose boxes overlap very often do not conflict at all: the
//! metal that matters is on different layers, or the box is far bigger than the cell's real
//! outline. Judging by boxes alone refuses placements the reference makes.
//!
//! Three tests, in the order the reference applies them, each stricter about a different thing:
//!
//! 1. **Blockages** — box against box. Nothing subtle; a placement blockage blocks.
//! 2. **Fixed instances** — box against box, then *refined by outline* where either side declares
//!    one. A cell whose true shape is an L does not conflict with something in the notch.
//! 3. **Per-layer clearance** — the cell's own shapes, grown by each layer's spacing, against
//!    whatever else sits on that same layer. This is what makes a bump above a corner a conflict
//!    only when they share metal.
//!
//! Nothing here touches a database.

use crate::orient::Orient;

/// An axis-aligned rectangle.
pub type Rect = (i32, i32, i32, i32);

/// One shape belonging to a cell or an obstruction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Shape {
    pub layer: String,
    pub rect: Rect,
    /// The net this shape carries, if any. Two shapes on the **same** net may touch freely.
    pub net: Option<String>,
}

/// Something already in the design that a new cell must respect.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Blocker {
    pub name: String,
    pub bbox: Rect,
    /// The blocker's true outline, when its master declares one. `None` means "use the box".
    pub outline: Vec<Rect>,
    /// Whether this blocker is checked by box (a fixed instance) or only per layer (a bump).
    pub by_box: bool,
    pub shapes: Vec<Shape>,
}

/// Why a placement was refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reason {
    /// A placement blockage. It has no layer and no owner: it forbids cells outright.
    Blockage,
    /// The cell's box hits a fixed instance's box, and no outline saved it.
    Instance(String),
    /// The cell's metal is too close to something else's on the same layer.
    Layer { blocker: String, layer: String },
}

/// A refusal, and **where** the conflict is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refusal {
    pub reason: Reason,
    /// The blocker's **own** rectangle, not the part of it the cell touches.
    ///
    /// ⚠️ The two are different questions and the reference asks both. "Where do I collide?" wants
    /// the intersection; "how far must I move to get clear?" wants the whole obstruction. Answering
    /// the second with the first bounds every escape by the cell's own width, so a cell can never
    /// step past anything wider than itself — and the symptom is a placer that shuffles instead of
    /// jumping.
    pub blocker: Rect,
    /// ⚠️ The **intersection** with the cell's box, not the blocker's own box. Callers that shift
    /// a cell along a row snap to where the overlap *starts*; the blocker's box can begin far
    /// behind the cell, which would shift it backwards rather than clear of the obstacle.
    pub overlap: Rect,
}

/// The shared area of two rectangles. Meaningless unless they intersect.
fn overlap_of(a: Rect, b: Rect) -> Rect {
    (a.0.max(b.0), a.1.max(b.1), a.2.min(b.2), a.3.min(b.3))
}

/// Do two rectangles share area? Touching is not overlapping.
pub fn intersects(a: Rect, b: Rect) -> bool {
    a.0 < b.2 && b.0 < a.2 && a.1 < b.3 && b.1 < a.3
}

/// Does a rectangle meet any of a set?
fn hits_any(r: Rect, set: &[Rect]) -> bool {
    set.iter().any(|&s| intersects(r, s))
}

/// **L1** — move a shape from master coordinates to where a placed cell puts it.
///
/// The cell's origin is its lower-left **after** orientation, which is why the transformed box is
/// normalised rather than offset directly: a mirrored cell's shapes run the other way from its
/// origin, and adding the origin to a raw coordinate would place them outside it.
pub fn transform(rect: Rect, master: (i32, i32), orient: Orient, at: (i32, i32)) -> Rect {
    let (mw, mh) = master;
    let (x0, y0, x1, y1) = rect;
    // Where each corner lands in the oriented cell's own frame, with the cell's lower-left at 0,0.
    let map = |x: i32, y: i32| -> (i32, i32) {
        match orient {
            Orient::R0 => (x, y),
            Orient::MY => (mw - x, y),
            Orient::MX => (x, mh - y),
            Orient::R180 => (mw - x, mh - y),
            Orient::R90 => (mh - y, x),
            Orient::MXR90 => (y, x),
            Orient::R270 => (y, mw - x),
            Orient::MYR90 => (mh - y, mw - x),
        }
    };
    let (ax, ay) = map(x0, y0);
    let (bx, by) = map(x1, y1);
    (
        ax.min(bx) + at.0,
        ay.min(by) + at.1,
        ax.max(bx) + at.0,
        ay.max(by) + at.1,
    )
}

/// **L2** — a cell's outline: its obstructions on OVERLAP-type layers, if it has any.
///
/// Empty means the cell has no outline and its bounding box stands in for it. That is not the same
/// as an empty outline, and treating it as one would let every cell overlap everything.
pub fn outline_of(shapes: &[Shape], is_overlap_layer: &dyn Fn(&str) -> bool) -> Vec<Rect> {
    shapes.iter().filter(|s| is_overlap_layer(&s.layer)).map(|s| s.rect).collect()
}

/// **L3** — would placing a cell here conflict with anything?
///
/// `cell_bbox` and `cell_shapes` describe the cell *as it would be placed*. `spacing_of` gives each
/// layer's required clearance, which the cell's own shapes are grown by before comparison — the
/// reference bloats the moving cell, not the fixed one.
pub fn refuse(
    cell_name: &str,
    cell_bbox: Rect,
    cell_outline: &[Rect],
    cell_shapes: &[Shape],
    blockers: &[Blocker],
    blockages: &[Rect],
    spacing_of: &dyn Fn(&str) -> i32,
) -> Option<Refusal> {
    // ── 1. Placement blockages, first and unconditionally ────────────────────────────────────
    // ⚠️ Order is part of the answer, not a detail. The caller shifts to the first conflict it is
    // told about, so checking instances before blockages would shift to a different site.
    for &g in blockages {
        if intersects(cell_bbox, g) {
            return Some(Refusal {
                reason: Reason::Blockage,
                blocker: g,
                overlap: overlap_of(cell_bbox, g),
            });
        }
    }

    // ── 2. Fixed instances, by box then refined by outline ───────────────────────────────────
    for b in blockers.iter().filter(|b| b.by_box) {
        if b.name == cell_name || !intersects(cell_bbox, b.bbox) {
            continue;
        }
        // ⚠️ An outline only ever *narrows* the conflict. Where both sides have one, both are
        // used; where one does, it is checked against the other's box. Where neither does, the
        // boxes already decided it.
        let refined = match (b.outline.is_empty(), cell_outline.is_empty()) {
            (false, false) => b.outline.iter().any(|&o| hits_any(o, cell_outline)),
            (false, true) => hits_any(cell_bbox, &b.outline),
            (true, false) => hits_any(b.bbox, cell_outline),
            (true, true) => true,
        };
        if refined {
            return Some(Refusal {
                reason: Reason::Instance(b.name.clone()),
                blocker: b.bbox,
                overlap: overlap_of(cell_bbox, b.bbox),
            });
        }
    }

    // ── 3. Per-layer clearance ───────────────────────────────────────────────────────────────
    for s in cell_shapes {
        let grow = spacing_of(&s.layer).max(0);
        let grown = (s.rect.0 - grow, s.rect.1 - grow, s.rect.2 + grow, s.rect.3 + grow);
        for b in blockers {
            if b.name == cell_name {
                continue;
            }
            for other in b.shapes.iter().filter(|o| o.layer == s.layer) {
                // Shapes on the same net are meant to touch — that is what connection by abutment
                // is. Only a mismatch is a conflict, and "no net" never matches "no net".
                let nets_match =
                    s.net.is_some() && other.net.is_some() && s.net == other.net;
                if !nets_match && intersects(grown, other.rect) {
                    return Some(Refusal {
                        blocker: other.rect,
                        // ⚠️ The overlap is measured against the cell's BOX, not its grown shape:
                        // the shape may reach outside the cell, and a shift target taken from
                        // there would move the cell further than the conflict requires.
                        overlap: overlap_of(cell_bbox, other.rect),
                        reason: Reason::Layer {
                            blocker: b.name.clone(),
                            layer: s.layer.clone(),
                        },
                    });
                }
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shape(layer: &str, rect: Rect) -> Shape {
        Shape { layer: layer.into(), rect, net: None }
    }

    fn fixed(name: &str, bbox: Rect) -> Blocker {
        Blocker { name: name.into(), bbox, outline: vec![], by_box: true, shapes: vec![] }
    }

    fn no_spacing(_: &str) -> i32 {
        0
    }

    #[test]
    fn touching_is_not_overlapping() {
        assert!(intersects((0, 0, 10, 10), (5, 5, 15, 15)));
        assert!(!intersects((0, 0, 10, 10), (10, 0, 20, 10)), "abutting");
        assert!(!intersects((0, 0, 10, 10), (20, 20, 30, 30)));
    }

    #[test]
    fn a_fixed_instance_in_the_way_refuses_the_placement() {
        let b = [fixed("OVERLAP", (80, 80, 120, 120))];
        assert_eq!(
            refuse("C", (70, 70, 350, 350), &[], &[], &b, &[], &no_spacing).map(|r| r.reason),
            Some(Reason::Instance("OVERLAP".into()))
        );
        assert_eq!(refuse("C", (500, 500, 600, 600), &[], &[], &b, &[], &no_spacing), None);
    }

    #[test]
    fn a_cell_never_blocks_itself() {
        // Re-placing an instance that already exists must not refuse the position it holds.
        let b = [fixed("C", (0, 0, 100, 100))];
        assert_eq!(refuse("C", (0, 0, 100, 100), &[], &[], &b, &[], &no_spacing), None);
    }

    #[test]
    fn an_outline_narrows_a_conflict_the_boxes_would_have_called() {
        // 🔑 The cell's box covers the blocker, but its true shape is an L with the blocker
        // sitting in the notch. Boxes say conflict; outlines say no.
        let b = [fixed("IN_THE_NOTCH", (10, 10, 40, 40))];
        let cell_box = (0, 0, 200, 200);
        assert!(refuse("C", cell_box, &[], &[], &b, &[], &no_spacing).is_some(), "by box, refused");

        let l_shape = [(0, 50, 200, 200), (50, 0, 200, 200)];
        assert_eq!(
            refuse("C", cell_box, &l_shape, &[], &b, &[], &no_spacing),
            None,
            "by outline, allowed"
        );
    }

    #[test]
    fn an_outline_on_the_blocker_narrows_it_too() {
        let mut b = fixed("HOLLOW", (0, 0, 200, 200));
        b.outline = vec![(0, 0, 20, 200)]; // only its left strip is real
        let blockers = [b];
        assert_eq!(refuse("C", (100, 100, 150, 150), &[], &[], &blockers, &[], &no_spacing), None);
        assert_eq!(
            refuse("C", (10, 100, 15, 150), &[], &[], &blockers, &[], &no_spacing).map(|r| r.reason),
            Some(Reason::Instance("HOLLOW".into()))
        );
    }

    #[test]
    fn no_outline_is_not_an_empty_outline() {
        // The distinction that would let everything overlap everything if collapsed.
        let b = [fixed("SOLID", (0, 0, 100, 100))];
        assert!(refuse("C", (50, 50, 150, 150), &[], &[], &b, &[], &no_spacing).is_some());
    }

    #[test]
    fn metal_conflicts_only_on_a_shared_layer() {
        // 🔑 The rule the bump cases turn on: a cover cell over a corner is in its way only if
        // they have metal on the same layer.
        let bump = Blocker {
            name: "BUMP".into(),
            bbox: (0, 0, 100, 100),
            outline: vec![],
            by_box: false, // a cover cell is not a box-level blocker
            shapes: vec![shape("metal10", (10, 10, 90, 90))],
        };
        let blockers = [bump];

        let on_m4 = [shape("metal4", (20, 20, 80, 80))];
        assert_eq!(refuse("C", (0, 0, 100, 100), &[], &on_m4, &blockers, &[], &no_spacing), None,
                   "different layers cannot conflict");

        let on_m10 = [shape("metal10", (20, 20, 80, 80))];
        assert_eq!(
            refuse("C", (0, 0, 100, 100), &[], &on_m10, &blockers, &[], &no_spacing).map(|r| r.reason),
            Some(Reason::Layer { blocker: "BUMP".into(), layer: "metal10".into() })
        );
    }

    #[test]
    fn the_moving_cells_shapes_are_grown_by_the_layers_spacing() {
        // Clearance, not collision: metal that merely comes too close is still a conflict.
        let other = Blocker {
            name: "OTHER".into(),
            bbox: (0, 0, 0, 0),
            outline: vec![],
            by_box: false,
            shapes: vec![shape("metal1", (100, 0, 200, 100))],
        };
        let blockers = [other];
        let cell = [shape("metal1", (0, 0, 90, 100))];
        assert_eq!(refuse("C", (0, 0, 90, 100), &[], &cell, &blockers, &[], &no_spacing), None,
                   "ten apart, and no spacing required");
        assert!(
            refuse("C", (0, 0, 90, 100), &[], &cell, &blockers, &[], &|_| 20).is_some(),
            "ten apart, twenty required"
        );
    }

    #[test]
    fn shapes_on_the_same_net_may_touch() {
        // Abutment is how a pad ring carries power; treating it as a conflict would refuse the
        // arrangement the design is built around.
        let mk = |net: Option<&str>| Shape {
            layer: "metal1".into(),
            rect: (0, 0, 100, 100),
            net: net.map(str::to_string),
        };
        let same = [Blocker {
            name: "N".into(),
            bbox: (0, 0, 0, 0),
            outline: vec![],
            by_box: false,
            shapes: vec![mk(Some("VDD"))],
        }];
        assert_eq!(refuse("C", (0, 0, 1, 1), &[], &[mk(Some("VDD"))], &same, &[], &no_spacing), None);
        assert!(refuse("C", (0, 0, 1, 1), &[], &[mk(Some("VSS"))], &same, &[], &no_spacing).is_some());
        // ⚠️ Two unconnected shapes do NOT match — "no net" is not a net they share.
        assert!(refuse("C", (0, 0, 1, 1), &[], &[mk(None)], &same, &[], &no_spacing).is_some());
        let unconnected = [Blocker { shapes: vec![mk(None)], ..same[0].clone() }];
        assert!(
            refuse("C", (0, 0, 1, 1), &[], &[mk(None)], &unconnected, &[], &no_spacing).is_some(),
            "both unconnected still conflict"
        );
    }

    #[test]
    fn two_cells_flush_against_each_other_are_legal() {
        // ⚠️ Pads abut. Boxes that only touch do not overlap, so the box test lets this through --
        // but if the neighbour also carried its metal, the per-layer test would refuse it, because
        // metal a spacing apart is within spacing. That is why an ordinary fixed cell is filed by
        // box OR by layer and never both; only a cover cell brings its shapes.
        let flush = (100, 0, 200, 100);
        let by_box_only = vec![Blocker {
            name: "LEFT".into(), bbox: (0, 0, 100, 100), outline: vec![],
            by_box: true, shapes: vec![],
        }];
        assert_eq!(
            refuse("R", flush, &[], &[shape("metal1", flush)], &by_box_only, &[], &|_| 10),
            None,
            "flush against an ordinary cell is legal"
        );
        // The same geometry as a cover cell -- shapes, no box -- does refuse it.
        let by_layer = vec![Blocker {
            name: "BUMP".into(), bbox: (0, 0, 100, 100), outline: vec![],
            by_box: false, shapes: vec![shape("metal1", (0, 0, 100, 100))],
        }];
        assert!(
            refuse("R", flush, &[], &[shape("metal1", flush)], &by_layer, &[], &|_| 10).is_some(),
            "the same geometry as a cover cell is too close"
        );
    }

    #[test]
    fn a_shape_moves_with_the_cell_that_owns_it() {
        // L1. A mirrored cell's shapes run the other way from its origin; adding the origin to a
        // raw master coordinate would put them outside the cell.
        let master = (100, 40);
        let s = (0, 0, 10, 40); // a strip up the cell's left edge
        assert_eq!(transform(s, master, Orient::R0, (1000, 2000)), (1000, 2000, 1010, 2040));
        // Mirrored about Y, the strip is on the RIGHT of the placed cell.
        assert_eq!(transform(s, master, Orient::MY, (1000, 2000)), (1090, 2000, 1100, 2040));
        // A quarter turn swaps the axes and keeps the shape inside the oriented cell.
        let r = transform(s, master, Orient::R90, (0, 0));
        assert_eq!(r, (0, 0, 40, 10));
    }

    #[test]
    fn an_outline_is_the_overlap_layer_shapes_and_nothing_else() {
        let shapes = [shape("metal4", (0, 0, 10, 10)), shape("OVERLAP", (20, 20, 30, 30))];
        let is_overlap = |l: &str| l == "OVERLAP";
        assert_eq!(outline_of(&shapes, &is_overlap), vec![(20, 20, 30, 30)]);
        assert!(outline_of(&shapes[..1], &is_overlap).is_empty(), "no outline declared");
    }
}
