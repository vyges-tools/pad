// SPDX-License-Identifier: Apache-2.0
//! `vyges-pad` — IO pad and bump placement.
//!
//! A chip talks to its package through a ring of IO cells around the die edge, and through an array
//! of bumps over its face. This engine builds that ring and places into it.
//!
//! - **[`orient`]** — the eight orientations and how they compose. A ring is built by composing
//!   them, so this is not a utility: get it backwards and every cell is wrong plausibly.
//! - **[`bump`]** — the package connections over the die's face. Not part of the ring at all: a
//!   lattice above the design, which is why a bump never blocks a placement by its box.
//! - **[`clearance`]** — whether a cell may go somewhere, which is not a question about bounding
//!   boxes: a dense ring is full of cells whose boxes overlap and whose metal does not.
//! - **[`place`]** — putting cells into that ring: a corner at a row's own origin, a pad at a site
//!   index along one. The ring says where a cell *may* go; this says where it *does*.
//! - **[`ring`]** — the ring itself: the die inset by four offsets, corners sized from the corner
//!   site, and four edges of whole sites. Everything else places *into* these rows, so a ring that
//!   differs makes every later comparison meaningless.
//!
//! Nothing in this module reads a database; the binary does that and hands values in.

// The crate's own name inside it: the command-line entry ([`cli_main`]) is built into the
// library too, and its `vyges_pad::…` paths resolve here.
extern crate self as vyges_pad;

/// This crate's version, as Cargo knows it — the single number the whole suite is released on.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// The copyright line `--version` prints.
pub const COPYRIGHT: &str = "© 2026 Vyges. All Rights Reserved.  https://vyges.com";

pub mod abut;
pub mod assign;
pub mod bond;
pub mod bump;
pub mod clearance;
pub mod fill;
pub mod orient;
pub mod pads;
pub mod place;
pub mod rdl;
pub mod ring;
pub mod spread;

pub use orient::Orient;
pub use abut::{all_touching, connect_by_abutment, touches, touching_terms, Conflict, PadInst, Plan, Terminal};
pub use assign::{assign, top_shape, Assignment, BumpTerm};
pub use bond::{bond_shape, is_bond_master, matching, Bond};
pub use fill::{fill_row, gaps, Fill, Filler, Unfilled, FILL_PREFIX};
pub use bump::{bumps, is_bump_master, Array, DEFAULT_PREFIX};
pub use clearance::{Reason, intersects, outline_of, refuse, transform, Blocker, Refusal, Shape};
pub use pads::{alignment_group, group_positions, keep_flip, pad_bump_distance, travel, Bump, BumpPad, fits, pad_width, place_one, place_uniform, target_spacing, Pad, Refused, Track};
pub use place::{
    corner_placement, corner_row_names, mirror_base, oriented_size, overlaps, place_in_row,
    snap_to_site, Edge, Placement, RowGeom,
};
pub use rdl::{edges as rdl_edges, grid as rdl_grid, thin, Grid};
pub use spread::{forces, ideal_position, nearest_legal, pool_adjacent_violators, unconstrained_start};
pub use ring::{make_rows, row_name, Offsets, Rotations, Row, RowDir, Site};

/// The `vyges-pad` command-line entry (the binary's `main`), built into the library too so a
/// suite binary can link every engine and run the one it is invoked as.
#[cfg(feature = "suite")]
#[doc(hidden)]
#[path = "main.rs"]
pub mod cli_main;
