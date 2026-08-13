// SPDX-License-Identifier: Apache-2.0
//! Orientations — the eight ways a cell can sit, and how they compose.
//!
//! A pad ring is built by composing orientations: a per-corner orientation on top of a
//! user-supplied rotation, four times, plus a mirrored partner for each edge. Get the composition
//! backwards and every cell in the ring is wrong in a way that still looks plausible — a ring, just
//! not this ring.
//!
//! The eight are the symmetries of a rectangle (the dihedral group of order 8): four rotations and
//! the same four mirrored. Each is a map of the plane, and composing two is composing the maps.
//!
//! # How the algebra was pinned
//!
//! Not from the reference's source, which only says `concat`. The direction of composition, and
//! what `flipX`/`flipY` mean, were **derived from three goldens that disagree with each other under
//! the wrong reading** — see the tests. `concat(a, b)` turns out to apply `a` **first**; the other
//! reading sends one row of the ring to `R270` where the golden says `R90`.

/// One of the eight orientations, in odb's spelling.
///
/// The DEF spelling is different and both appear in the goldens: `N S E W` for the rotations and
/// `FN FS FE FW` for the mirrored ones. [`Orient::def`] converts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Orient {
    R0,
    R90,
    R180,
    R270,
    MY,
    MYR90,
    MX,
    MXR90,
}

use Orient::*;

impl Orient {
    /// Where this orientation sends the point `(x, y)`.
    ///
    /// The whole algebra is derived from this one table, so composition cannot drift out of step
    /// with the geometry the way two hand-written tables would.
    fn apply(self, x: i32, y: i32) -> (i32, i32) {
        match self {
            R0 => (x, y),
            R90 => (-y, x),
            R180 => (-x, -y),
            R270 => (y, -x),
            MY => (-x, y),
            MYR90 => (-y, -x),
            MX => (x, -y),
            MXR90 => (y, x),
        }
    }

    const ALL: [Orient; 8] = [R0, R90, R180, R270, MY, MYR90, MX, MXR90];

    /// The orientation that maps `(1, 2)` the way this composition does.
    ///
    /// `(1, 2)` is enough to identify an element: the eight images are distinct, because no two of
    /// the eight symmetries agree on a point with `|x| != |y|` and both non-zero.
    fn from_image(p: (i32, i32)) -> Orient {
        *Orient::ALL.iter().find(|o| o.apply(1, 2) == p).expect("a symmetry of the rectangle")
    }

    /// **O1** — `a.concat(b)` applies **`a` first**, then `b`.
    ///
    /// ⚠️ The order is the trap. The reference builds a transform from the user's rotation and
    /// *then* concats the per-edge orientation, so the user's rotation is the one applied first.
    /// Reading it the other way puts the east row at `R270` where the golden says `R90`.
    pub fn concat(self, then: Orient) -> Orient {
        let (x, y) = self.apply(1, 2);
        Orient::from_image(then.apply(x, y))
    }

    /// Mirror about the X axis — negate `y` **after** this orientation has been applied.
    pub fn flip_x(self) -> Orient {
        self.concat(MX)
    }

    /// Mirror about the Y axis — negate `x` after this orientation.
    pub fn flip_y(self) -> Orient {
        self.concat(MY)
    }

    /// The DEF spelling, which is what a golden contains.
    pub fn def(self) -> &'static str {
        match self {
            R0 => "N",
            R90 => "W",
            R180 => "S",
            R270 => "E",
            MY => "FN",
            MYR90 => "FE",
            MX => "FS",
            MXR90 => "FW",
        }
    }

    /// Parse odb's spelling, as a command line gives it.
    pub fn parse(s: &str) -> Option<Orient> {
        Some(match s {
            "R0" => R0,
            "R90" => R90,
            "R180" => R180,
            "R270" => R270,
            "MY" => MY,
            "MYR90" => MYR90,
            "MX" => MX,
            "MXR90" => MXR90,
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_orientation_is_a_distinct_symmetry() {
        // The identification trick the whole module rests on: the eight images of (1,2) are
        // distinct, so an orientation is recoverable from where it sends one point.
        let mut seen: Vec<(i32, i32)> = Orient::ALL.iter().map(|o| o.apply(1, 2)).collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), 8);
    }

    #[test]
    fn identity_composes_to_nothing_on_either_side() {
        for o in Orient::ALL {
            assert_eq!(o.concat(R0), o, "{o:?} then R0");
            assert_eq!(R0.concat(o), o, "R0 then {o:?}");
        }
    }

    #[test]
    fn composition_applies_the_first_argument_first() {
        // 🔑 Pinned by `make_io_sites_rotations`, which sets -rotation_horizontal MXR90 and asks
        // for an east row that the golden records as W (R90).
        //
        //   concat(MXR90, MY) = R90   -- apply MXR90, then MY   ← the golden
        //   MXR90 applied to MY       = R270                    ← the other reading
        assert_eq!(MXR90.concat(MY), R90, "the reading the golden requires");
        assert_ne!(MXR90.concat(MY), R270, "and the one it rules out");

        // From the same case: -rotation_vertical MY over the north edge's own MX gives S.
        assert_eq!(MY.concat(MX), R180, "north edge under a vertical rotation");
        assert_eq!(MY.concat(R0), MY, "south edge under the same rotation");
    }

    #[test]
    fn flipping_matches_what_the_ring_goldens_require() {
        // The east edge is the west edge flipped about Y, and both spellings appear in goldens:
        //   same site for both directions  -> west FW (MXR90), east W  (R90)
        //   different sites                -> west N  (R0),    east FN (MY)
        assert_eq!(MXR90.flip_y(), R90);
        assert_eq!(R0.flip_y(), MY);
        // The north edge is the south edge flipped about X: south N (R0), north FS (MX).
        assert_eq!(R0.flip_x(), MX);
    }

    #[test]
    fn flipping_twice_about_the_same_axis_is_a_no_op() {
        for o in Orient::ALL {
            assert_eq!(o.flip_x().flip_x(), o, "{o:?}");
            assert_eq!(o.flip_y().flip_y(), o, "{o:?}");
        }
    }

    #[test]
    fn composition_is_associative_and_every_element_has_an_inverse() {
        // It is a group; if these fail the table is wrong, whatever the goldens happen to agree on.
        for a in Orient::ALL {
            for b in Orient::ALL {
                for c in Orient::ALL {
                    assert_eq!(a.concat(b).concat(c), a.concat(b.concat(c)), "{a:?} {b:?} {c:?}");
                }
            }
            assert!(
                Orient::ALL.iter().any(|&i| a.concat(i) == R0),
                "{a:?} has no inverse"
            );
        }
    }

    #[test]
    fn the_def_spellings_round_trip() {
        // A golden is read in DEF spelling and a command line is written in odb spelling; both
        // appear in this engine and confusing them is silent.
        let pairs = [
            (R0, "N"),
            (R90, "W"),
            (R180, "S"),
            (R270, "E"),
            (MY, "FN"),
            (MYR90, "FE"),
            (MX, "FS"),
            (MXR90, "FW"),
        ];
        for (o, d) in pairs {
            assert_eq!(o.def(), d, "{o:?}");
            assert_eq!(Orient::parse(&format!("{o:?}")), Some(o));
        }
        assert_eq!(Orient::parse("NONSENSE"), None);
    }
}
