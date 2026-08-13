// SPDX-License-Identifier: Apache-2.0
//! The bump array — a grid of package connections over the face of the die.
//!
//! Bumps are not part of the ring. They sit *above* the design on a cover layer, on a regular
//! lattice, and the die's own cells pass underneath them. That is why a bump never blocks a
//! placement by its bounding box (see [`crate::clearance`]) and why this is a grid rather than a
//! row.
//!
//! Nothing here touches a database.

use crate::orient::Orient;
use crate::place::Placement;

/// The default name prefix, when the caller does not give one.
pub const DEFAULT_PREFIX: &str = "BUMP_";

/// What `make_io_bump_array` asks for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Array {
    pub master: String,
    pub prefix: String,
    /// Lower-left bump, in DBU.
    pub origin: (i32, i32),
    pub rows: i32,
    pub columns: i32,
    /// Distance between bumps, in DBU. The two axes are independent.
    pub pitch: (i32, i32),
}

/// **U1** — every bump in the array.
///
/// ⚠️ **The name is `<prefix><column>_<row>`** — column first. It reads like a coordinate pair and
/// is one, but with x before y, so a grid named row-first is a different design as far as every
/// later command and every golden is concerned.
///
/// Emitted column by column, the order the reference creates them in.
pub fn bumps(a: &Array) -> Vec<Placement> {
    let mut out = Vec::new();
    for col in 0..a.columns.max(0) {
        for row in 0..a.rows.max(0) {
            out.push(Placement {
                name: format!("{}{}_{}", a.prefix, col, row),
                master: a.master.clone(),
                x: a.origin.0 + col * a.pitch.0,
                y: a.origin.1 + row * a.pitch.1,
                // A bump takes no rotation: the array is a lattice, not a ring.
                orient: Orient::R0,
            });
        }
    }
    out
}

/// **U2** — is this master a bump?
///
/// ⚠️ A **prefix** test, because odb reports a class the way the LEF spells it: `CLASS COVER BUMP`
/// comes back as `"COVER BUMP"`. The reference requires specifically `COVER_BUMP`, and the two
/// spellings meet here.
pub fn is_bump_master(class: &str) -> bool {
    let c = class.trim();
    c == "COVER BUMP" || c == "COVER_BUMP"
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `bump_array_make.tcl`: 14 x 14 at 200um origin and 200um pitch, on a 2000 DBU/um technology.
    fn reference() -> Array {
        Array {
            master: "DUMMY_BUMP".into(),
            prefix: DEFAULT_PREFIX.into(),
            origin: (400_000, 400_000),
            rows: 14,
            columns: 14,
            pitch: (400_000, 400_000),
        }
    }

    #[test]
    fn the_array_reproduces_the_reference_golden() {
        let b = bumps(&reference());
        assert_eq!(b.len(), 196, "14 x 14");
        let at = |n: &str| b.iter().find(|p| p.name == n).unwrap_or_else(|| panic!("no {n}"));
        // The reference result for `bump_array_make`.
        assert_eq!((at("BUMP_0_0").x, at("BUMP_0_0").y), (400_000, 400_000));
        assert_eq!((at("BUMP_0_1").x, at("BUMP_0_1").y), (400_000, 800_000));
        assert_eq!((at("BUMP_0_10").x, at("BUMP_0_10").y), (400_000, 4_400_000));
        assert_eq!((at("BUMP_0_12").x, at("BUMP_0_12").y), (400_000, 5_200_000));
        assert!(b.iter().all(|p| p.orient == Orient::R0 && p.master == "DUMMY_BUMP"));
    }

    #[test]
    fn the_name_is_column_first() {
        // A grid named row-first is a different design to every later command.
        let a = Array { rows: 2, columns: 3, ..reference() };
        let b = bumps(&a);
        let named: Vec<&str> = b.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(named, vec!["BUMP_0_0", "BUMP_0_1", "BUMP_1_0", "BUMP_1_1", "BUMP_2_0", "BUMP_2_1"]);
        // BUMP_2_0 is the third COLUMN, first row — so it is displaced in x, not y.
        let at = |n: &str| b.iter().find(|p| p.name == n).unwrap();
        assert_eq!(at("BUMP_2_0").x, a.origin.0 + 2 * a.pitch.0);
        assert_eq!(at("BUMP_2_0").y, a.origin.1);
    }

    #[test]
    fn the_two_pitches_are_independent() {
        let a = Array { rows: 2, columns: 2, pitch: (100, 7_000), ..reference() };
        let b = bumps(&a);
        let at = |n: &str| b.iter().find(|p| p.name == n).unwrap();
        assert_eq!(at("BUMP_1_0").x - at("BUMP_0_0").x, 100, "x pitch");
        assert_eq!(at("BUMP_0_1").y - at("BUMP_0_0").y, 7_000, "y pitch");
    }

    #[test]
    fn a_custom_prefix_replaces_the_default_entirely() {
        let a = Array { prefix: "PAD_BUMP_".into(), rows: 1, columns: 1, ..reference() };
        assert_eq!(bumps(&a)[0].name, "PAD_BUMP_0_0");
    }

    #[test]
    fn an_empty_or_negative_array_yields_nothing_rather_than_panicking() {
        for (r, c) in [(0, 5), (5, 0), (0, 0), (-3, 5)] {
            let a = Array { rows: r, columns: c, ..reference() };
            assert!(bumps(&a).is_empty(), "{r} x {c}");
        }
    }

    #[test]
    fn a_bump_master_is_recognised_in_either_spelling() {
        // ⚠️ odb reports the class as the LEF spells it — `CLASS COVER BUMP` is `"COVER BUMP"`.
        assert!(is_bump_master("COVER BUMP"));
        assert!(is_bump_master("COVER_BUMP"), "the enum spelling too");
        assert!(!is_bump_master("COVER"), "a plain cover cell is not a bump");
        assert!(!is_bump_master("PAD AREAIO"));
        assert!(!is_bump_master("CORE"));
    }
}
