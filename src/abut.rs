// SPDX-License-Identifier: Apache-2.0
//! Connecting the ring by abutment.
//!
//! Pads in a ring are wired by *touching*: a pad's power terminal shares an edge with its
//! neighbour's, and that shared edge is the connection. Nothing routes it. This works out which
//! terminals touch and what nets that implies.
//!
//! ⚠️ **Touching counts here, and only here.** [`crate::clearance`] asks whether two cells are too
//! close and treats a shared edge as clear; this asks whether they meet and treats a shared edge
//! as a connection. Same geometry, opposite predicate — [`touches`] is deliberately not
//! [`crate::clearance::intersects`].
//!
//! Nothing here touches a database.

use std::collections::{HashMap, HashSet};

type Rect = (i32, i32, i32, i32);

/// One terminal of a placed instance, with its shapes already moved onto the die.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Terminal {
    pub name: String,
    /// `(layer number, rectangle)`. Only shapes on the *same* layer can touch.
    pub shapes: Vec<(i64, Rect)>,
    pub net: Option<String>,
    /// Whether this terminal is a supply pin, which decides the net's signal type.
    pub supply: bool,
}

/// A placed pad.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PadInst {
    pub name: String,
    pub bbox: Rect,
    pub terms: Vec<Terminal>,
}

/// Which terminal: `(instance index, terminal index)`.
pub type TermId = (usize, usize);

/// Two terminals that touch but are already on different nets. Always a defect in the input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Conflict {
    pub a: TermId,
    pub b: TermId,
    pub net_a: String,
    pub net_b: String,
}

/// What connecting by abutment implies, in the order it must be applied.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Plan {
    /// Nets that held a single terminal and are removed before anything is joined.
    pub destroy: Vec<String>,
    /// Nets that did not exist and have to be made.
    pub create: Vec<String>,
    /// `(terminal, net)`, in application order.
    pub connect: Vec<(TermId, String)>,
    /// Nets to mark as special, first-seen order.
    pub special: Vec<String>,
}

/// **A1** — do these two rectangles meet?
///
/// ⚠️ **Closed**: sharing only an edge or a corner counts. Abutting pads share exactly an edge, so
/// the strict test would find no connections at all and the ring would come out unwired.
pub fn touches(a: Rect, b: Rect) -> bool {
    a.0 <= b.2 && b.0 <= a.2 && a.1 <= b.3 && b.1 <= a.3
}

/// **A2** — which terminals of two instances touch.
///
/// Shapes only ever meet on the same layer. Returns terminal index pairs, `a`'s first.
pub fn touching_terms(a: &PadInst, b: &PadInst) -> Vec<(usize, usize)> {
    // A cheap rejection first: cells whose boxes do not meet have no terminals that can.
    if !touches(a.bbox, b.bbox) {
        return Vec::new();
    }
    let mut out = Vec::new();
    for (i, ta) in a.terms.iter().enumerate() {
        for (j, tb) in b.terms.iter().enumerate() {
            let meets = ta.shapes.iter().any(|&(la, ra)| {
                tb.shapes.iter().any(|&(lb, rb)| la == lb && touches(ra, rb))
            });
            if meets {
                out.push((i, j));
            }
        }
    }
    out
}

/// **A3** — every touching pair in the design, in instance order.
///
/// ⚠️ The instance list may repeat an instance (it is gathered row by row, and a cell reaching
/// into two rows is collected twice). That is left as it is rather than deduplicated: the pairs it
/// produces are redundant, not wrong, and every step below is idempotent.
pub fn all_touching(insts: &[PadInst]) -> Vec<(TermId, TermId)> {
    let mut pairs = Vec::new();
    for i in 0..insts.len() {
        for j in (i + 1)..insts.len() {
            for (ti, tj) in touching_terms(&insts[i], &insts[j]) {
                pairs.push(((i, ti), (j, tj)));
            }
        }
    }
    pairs
}

/// The evolving net of every terminal, plus how many connections each net has.
struct State {
    of: Vec<Vec<Option<String>>>,
    counts: HashMap<String, u32>,
}

impl State {
    fn net(&self, t: TermId) -> Option<&String> {
        self.of[t.0][t.1].as_ref()
    }

    fn attach(&mut self, t: TermId, net: &str) {
        if let Some(old) = self.of[t.0][t.1].take() {
            *self.counts.entry(old).or_insert(1) -= 1;
        }
        self.of[t.0][t.1] = Some(net.to_string());
        *self.counts.entry(net.to_string()).or_insert(0) += 1;
    }

    /// Forget a net everywhere. A destroyed net leaves its terminals unconnected, not deleted.
    fn forget(&mut self, net: &str) {
        for inst in &mut self.of {
            for slot in inst.iter_mut() {
                if slot.as_deref() == Some(net) {
                    *slot = None;
                }
            }
        }
        self.counts.remove(net);
    }
}

/// **A4** — join up everything that touches.
///
/// Three steps, in this order:
///
/// 1. **A net holding a single terminal is destroyed.** It cannot be a connection, and leaving it
///    would block the merge below — two touching terminals on different nets is an error, and a
///    stray one-terminal net is exactly that.
/// 2. **Merge until nothing changes.** Where one side has a net and the other does not, the other
///    joins it. Where *both* have nets and they differ, the input is contradictory and this stops.
/// 3. **What is still unconnected gets a new net**, named after the first of the two terminals.
///
/// `connections_of` reports how many terminals a net starts with, counting the whole design and
/// not just these pads.
pub fn connect_by_abutment(
    insts: &[PadInst],
    connections_of: &dyn Fn(&str) -> u32,
) -> Result<Plan, Conflict> {
    let pairs = all_touching(insts);
    let mut plan = Plan::default();
    let mut counts = HashMap::new();
    for inst in insts {
        for t in &inst.terms {
            if let Some(n) = &t.net {
                counts.entry(n.clone()).or_insert_with(|| connections_of(n));
            }
        }
    }
    let mut st = State {
        of: insts.iter().map(|i| i.terms.iter().map(|t| t.net.clone()).collect()).collect(),
        counts,
    };

    // ── 1. Nets with a single terminal ───────────────────────────────────────────────────────
    let mut gone = HashSet::new();
    for (a, b) in &pairs {
        for t in [a, b] {
            let Some(net) = st.net(*t).cloned() else { continue };
            if st.counts.get(&net).copied().unwrap_or(0) == 1 && gone.insert(net.clone()) {
                plan.destroy.push(net.clone());
                st.forget(&net);
            }
        }
    }

    // ── 2 and 3, interleaved exactly as the reference does ───────────────────────────────────
    merge(&pairs, &mut st, &mut plan)?;
    for (a, b) in &pairs {
        if st.net(*a).is_some() {
            continue;
        }
        // ⚠️ `<instance>.<terminal>_RING`, from the FIRST terminal of the pair. Taking the second
        // would name the same net differently depending on instance order.
        let name = format!("{}.{}_RING", insts[a.0].name, insts[a.0].terms[a.1].name);
        plan.create.push(name.clone());
        for t in [a, b] {
            st.attach(*t, &name);
            plan.connect.push((*t, name.clone()));
        }
        // ⚠️ A net created here is **not** special on that account. It becomes special only if the
        // merge below then joins something else to it. A pair that touches nothing else stays an
        // ordinary net — and in DEF that is the difference between appearing in NETS and moving
        // to SPECIALNETS, so getting it wrong relocates hundreds of nets.
        //
        // A new net can let two previously separate groups join, so the merge runs again.
        merge(&pairs, &mut st, &mut plan)?;
    }
    Ok(plan)
}

fn merge(pairs: &[(TermId, TermId)], st: &mut State, plan: &mut Plan) -> Result<(), Conflict> {
    loop {
        let mut changed = false;
        for &(a, b) in pairs {
            let (na, nb) = (st.net(a).cloned(), st.net(b).cloned());
            if na == nb {
                continue;
            }
            if let (Some(net_a), Some(net_b)) = (&na, &nb) {
                return Err(Conflict { a, b, net_a: net_a.clone(), net_b: net_b.clone() });
            }
            let target = na.or(nb).expect("one side has a net, or they would be equal");
            for t in [a, b] {
                if st.net(t) != Some(&target) {
                    st.attach(t, &target);
                    plan.connect.push((t, target.clone()));
                    changed = true;
                }
            }
            note_special(plan, &target);
        }
        if !changed {
            return Ok(());
        }
    }
}

fn note_special(plan: &mut Plan, net: &str) {
    if !plan.special.iter().any(|n| n == net) {
        plan.special.push(net.to_string());
    }
}

/// **A5** — the signal type a net takes once it is special.
///
/// ⚠️ A **supply** terminal wins: if any terminal on the net is power or ground, the net becomes
/// that, whatever it was before. The *last* such terminal decides, which only matters for a net
/// that somehow carries both.
pub fn special_sig_type(current: &str, term_types: &[(bool, String)]) -> String {
    let mut sig = current.to_string();
    for (supply, ty) in term_types {
        if *supply {
            sig = ty.clone();
        }
    }
    sig
}

#[cfg(test)]
mod tests {
    use super::*;

    fn term(name: &str, r: Rect, net: Option<&str>) -> Terminal {
        Terminal {
            name: name.into(),
            shapes: vec![(1, r)],
            net: net.map(str::to_string),
            supply: false,
        }
    }

    fn inst(name: &str, bbox: Rect, terms: Vec<Terminal>) -> PadInst {
        PadInst { name: name.into(), bbox, terms }
    }

    /// Two pads flush against each other at x = 100, each with one terminal on the shared edge.
    fn abutting(net_a: Option<&str>, net_b: Option<&str>) -> Vec<PadInst> {
        vec![
            inst("A", (0, 0, 100, 50), vec![term("VDD", (90, 0, 100, 50), net_a)]),
            inst("B", (100, 0, 200, 50), vec![term("VDD", (100, 0, 110, 50), net_b)]),
        ]
    }

    #[test]
    fn a_shared_edge_is_a_connection() {
        // ⚠️ The whole mechanism. The strict overlap test would find nothing here.
        assert!(touches((0, 0, 100, 50), (100, 0, 200, 50)), "edge to edge");
        assert!(touches((0, 0, 100, 50), (100, 50, 200, 100)), "corner to corner");
        assert!(!touches((0, 0, 100, 50), (101, 0, 200, 50)), "one unit apart is apart");
    }

    #[test]
    fn terminals_only_meet_on_the_same_layer() {
        let mut a = inst("A", (0, 0, 100, 50), vec![term("VDD", (90, 0, 100, 50), None)]);
        let b = inst("B", (100, 0, 200, 50), vec![term("VDD", (100, 0, 110, 50), None)]);
        assert_eq!(touching_terms(&a, &b), vec![(0, 0)]);
        a.terms[0].shapes = vec![(2, (90, 0, 100, 50))];
        assert!(touching_terms(&a, &b).is_empty(), "same place, different layer");
    }

    #[test]
    fn two_unconnected_terminals_get_a_new_net_named_after_the_first() {
        let plan = connect_by_abutment(&abutting(None, None), &|_| 0).unwrap();
        assert_eq!(plan.create, vec!["A.VDD_RING"]);
        assert_eq!(plan.connect.len(), 2);
        assert!(plan.destroy.is_empty());
        // ⚠️ Not special: nothing else joined it. Marking it would move it from NETS to
        // SPECIALNETS in the written design.
        assert!(plan.special.is_empty(), "a lone pair is an ordinary net");
    }

    #[test]
    fn a_new_net_becomes_special_once_a_third_terminal_joins_it() {
        let insts = vec![
            inst("A", (0, 0, 100, 50), vec![term("VDD", (90, 0, 100, 50), None)]),
            inst("B", (100, 0, 200, 50), vec![term("VDD", (100, 0, 200, 50), None)]),
            inst("C", (200, 0, 300, 50), vec![term("VDD", (200, 0, 210, 50), None)]),
        ];
        let plan = connect_by_abutment(&insts, &|_| 0).unwrap();
        assert_eq!(plan.create, vec!["A.VDD_RING"]);
        assert_eq!(plan.special, vec!["A.VDD_RING"], "C joining made it special");
    }

    #[test]
    fn an_unconnected_terminal_joins_its_neighbours_net() {
        let plan = connect_by_abutment(&abutting(Some("VDD"), None), &|_| 4).unwrap();
        assert!(plan.create.is_empty(), "no new net needed");
        assert_eq!(plan.connect, vec![((1, 0), "VDD".to_string())]);
        assert_eq!(plan.special, vec!["VDD"]);
    }

    #[test]
    fn a_net_holding_one_terminal_is_destroyed_before_anything_is_joined() {
        // ⚠️ Without this, the pair below would look like two DIFFERENT nets touching, which is an
        // error — so the order of the two steps is the difference between working and failing.
        let plan = connect_by_abutment(&abutting(Some("VDD"), Some("stray")), &|n| {
            if n == "stray" {
                1
            } else {
                4
            }
        })
        .unwrap();
        assert_eq!(plan.destroy, vec!["stray"]);
        assert_eq!(plan.connect, vec![((1, 0), "VDD".to_string())]);
    }

    #[test]
    fn two_real_nets_touching_is_a_conflict_rather_than_a_silent_merge() {
        let err = connect_by_abutment(&abutting(Some("VDD"), Some("VSS")), &|_| 4).unwrap_err();
        assert_eq!((err.net_a.as_str(), err.net_b.as_str()), ("VDD", "VSS"));
    }

    #[test]
    fn a_chain_of_pads_ends_up_on_one_net() {
        let insts = vec![
            inst("A", (0, 0, 100, 50), vec![term("VDD", (90, 0, 100, 50), Some("VDD"))]),
            // B's terminal spans its whole width, so it reaches both neighbours.
            inst("B", (100, 0, 200, 50), vec![term("VDD", (100, 0, 200, 50), None)]),
            inst("C", (200, 0, 300, 50), vec![term("VDD", (200, 0, 210, 50), None)]),
        ];
        // C touches only B, so it reaches VDD only through the repeated merge.
        let plan = connect_by_abutment(&insts, &|_| 4).unwrap();
        assert!(plan.create.is_empty());
        let nets: Vec<&str> = plan.connect.iter().map(|(_, n)| n.as_str()).collect();
        assert_eq!(nets, vec!["VDD", "VDD"], "both joined the existing net");
    }

    #[test]
    fn pads_that_do_not_touch_imply_nothing() {
        let insts = vec![
            inst("A", (0, 0, 100, 50), vec![term("VDD", (0, 0, 100, 50), None)]),
            inst("B", (200, 0, 300, 50), vec![term("VDD", (200, 0, 300, 50), None)]),
        ];
        assert_eq!(connect_by_abutment(&insts, &|_| 0).unwrap(), Plan::default());
    }

    #[test]
    fn a_supply_terminal_decides_the_signal_type() {
        assert_eq!(special_sig_type("SIGNAL", &[(false, "SIGNAL".into())]), "SIGNAL");
        assert_eq!(special_sig_type("SIGNAL", &[(true, "POWER".into())]), "POWER");
        // The last supply terminal wins.
        assert_eq!(
            special_sig_type("SIGNAL", &[(true, "POWER".into()), (true, "GROUND".into())]),
            "GROUND"
        );
    }
}
