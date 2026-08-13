# vyges-pad

IO pad and bump placement over an OpenDB database.

A chip reaches its package through a **ring** of IO cells around the die edge and an array of bumps
over its face. This engine builds that ring and places into it.

## Status

Early. The ring geometry and the orientation algebra it is built from are implemented and verified;
pad, corner, filler, bump and terminal placement are not yet.

- **`orient`** — the eight orientations and how they compose. Not a utility: a pad ring is built by
  composing orientations, and getting the composition backwards yields a ring that looks entirely
  plausible and is wrong in every cell.
- **`ring`** — the die inset by four independent offsets, corners sized from the corner site, and
  four edges tiled with whole sites. Everything placed later goes *into* these rows.

## Correctness

The ring reproduces the reference's row output exactly — name, site, origin, orientation, direction,
site count and pitch — on every one of its three ring cases, including the one that rotates the rows
and the one that gives the two directions different sites.

The algebra underneath was **derived from those goldens rather than assumed**: the direction of
orientation composition is pinned by a case that the opposite reading gets wrong.

## Scope

The upstream module also contains a redistribution-layer **router**. That is a routing engine, and
it is deliberately not part of this one.

## Building

```text
cargo build
cargo test
```

## Licence

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
