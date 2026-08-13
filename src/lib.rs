// SPDX-License-Identifier: Apache-2.0
//! `vyges-pad` — IO pad and bump placement.
//!
//! A chip talks to its package through a ring of IO cells around the die edge, and through an array
//! of bumps over its face. This engine builds that ring and places into it.
//!
//! - **[`orient`]** — the eight orientations and how they compose. A ring is built by composing
//!   them, so this is not a utility: get it backwards and every cell is wrong plausibly.
//! - **[`place`]** — putting cells into that ring: a corner at a row's own origin, a pad at a site
//!   index along one. The ring says where a cell *may* go; this says where it *does*.
//! - **[`ring`]** — the ring itself: the die inset by four offsets, corners sized from the corner
//!   site, and four edges of whole sites. Everything else places *into* these rows, so a ring that
//!   differs makes every later comparison meaningless.
//!
//! Nothing in this module reads a database; the binary does that and hands values in.

pub mod orient;
pub mod place;
pub mod ring;

pub use orient::Orient;
pub use place::{
    corner_placement, corner_row_names, mirror_base, oriented_size, overlaps, place_in_row,
    snap_to_site, Edge, Placement, RowGeom,
};
pub use ring::{make_rows, row_name, Offsets, Rotations, Row, RowDir, Site};
