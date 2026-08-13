// SPDX-License-Identifier: Apache-2.0
//! `vyges-pad` CLI — the IO pad ring over a `.odb`.
//!
//! Today it exposes `make-io-sites`, which builds the ring every later pad command places into.
//! That is deliberately shippable on its own: a ring whose geometry differs from the reference's
//! makes every later comparison meaningless, and one that agrees makes the rest checkable a stage
//! at a time.
//!
//! Exit status: 0 rows created, 1 the design cannot carry a ring, 2 usage/read/write error.

use std::process::ExitCode;
use vyges_pad::clearance::Rect;
use vyges_opendb::Db;
use vyges_pad::{bumps, corner_placement, intersects, corner_row_names, make_rows, mirror_base, oriented_size,
    outline_of, place_in_row, refuse, snap_to_site, transform, Blocker, Offsets, Orient,
    Placement, Row, RowDir, RowGeom, Rotations, Shape, Site};
use vyges_pad::bump::{is_bump_master, Array, DEFAULT_PREFIX};
use vyges_pad::pads::{
    alignment_group, fits, group_positions, keep_flip, place_one, place_uniform, travel, Bump,
    BumpPad, Pad, Refused, Track,
};
use vyges_pad::fill::{fill_row, Filler, Unfilled};
use vyges_pad::rdl;
use vyges_pad::abut::{connect_by_abutment, special_sig_type, touching_terms, PadInst, Terminal};
use vyges_pad::bond::{bond_shape, is_bond_master, matching, pin_shape, place as bond_place};
use vyges_pad::assign::{assign, BumpTerm, Refused as AssignRefused};

const USAGE: &str = "\
vyges loom pad — IO pad and bump placement: the ring around the die, and what sits in it

USAGE:
  vyges loom pad make-io-sites  <design.odb> --horizontal-site S --vertical-site S
                                --corner-site S --offset D [options]
  vyges loom pad place-corners  <design.odb> --master M [--ring-index N] [options]
  vyges loom pad place-pad      <design.odb> --row R --location D [--master M]
                                [--mirror] --inst NAME [options]
  vyges loom pad make-io-bump-array <design.odb> --bump M --origin 'X Y' --rows N
                                   --columns N --pitch 'DX [DY]' [--prefix P] [options]
  vyges loom pad place-pads <design.odb> --row R --insts 'A B C' [--mode M] [options]
  vyges loom pad place-io-fill <design.odb> --row R --masters 'A B C'
                              [--permit-overlaps 'M'] [options]
  vyges loom pad place-io-terminals <design.odb> --pins 'PATTERN...'
                                   [--allow-non-top-layer] [options]
  vyges loom pad rdl-route <design.odb> --layer L [--width W] [--spacing S]
                          [--allow45] [--grid-only] [options]
  vyges loom pad assign-io-bump <design.odb> --bump INST --net N
                               [--terminal INST/PIN] [--dont-route] [options]
  vyges loom pad connect-by-abutment <design.odb> [options]
  vyges loom pad place-bondpad <design.odb> --bond M --insts 'PATTERN...'
                               [--offset 'X Y'] [--rotation R] [--prefix P] [options]
  vyges loom pad remove-io-bump <design.odb> --inst NAME [options]
  vyges loom pad remove-io-bump-array <design.odb> --bump M [options]
  vyges loom pad --describe
  vyges loom pad --help

OPTIONS:
  --horizontal-site S    site tiling the LEFT and RIGHT rows (required)
  --vertical-site S      site tiling the BOTTOM and TOP rows (required)
  --corner-site S        site tiling the four corners (required)
  --offset D             inset from the die on every edge, in MICRONS (required)
  --rotation-horizontal R  rotation applied to the left/right rows (default R0)
  --rotation-vertical R    rotation applied to the bottom/top rows (default R0)
  --rotation-corner R      rotation applied to the corners (default R0)
  --ring-index N         suffix the row names with _N, for a design with several rings
  --master M             the cell to place
  --row R                the IO row to place into (place-pad)
  --location D           where along that row, in MICRONS (place-pad)
  --inst NAME            the instance to place or create (place-pad)
  --mirror               mirror the pad about the row
  --bump M               the bump master (make-io-bump-array)
  --origin 'X Y'         the lower-left bump, in MICRONS
  --rows N / --columns N the shape of the array
  --pitch 'DX [DY]'      spacing in MICRONS; one value means both axes
  --prefix P             instance name prefix (default BUMP_)
  --insts 'A B C'        the pads to spread along a row (place-pads)
  --mode M               uniform | linear | bump_aligned | placer | default
  --out-odb FILE         write the database here (default: IN PLACE, over the input)
  --out-def FILE         also write the result as DEF (for diffing against a golden)
  --dry-run              report the ring, write nothing
  -o FILE                write the report to FILE instead of stdout
  --json                 emit JSON (the default)
  --describe             print a machine-readable JSON description of the command

EXIT STATUS:
  0  ok       the ring was created
  1  refused  the die cannot carry a ring with these sites and offsets
  2  error    usage error, unreadable database, unknown site, or a failed write
";

const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "name": "pad",
  "summary": "IO pad and bump placement: the ring of IO rows around the die, and the cells placed into it",
  "maturity": "partial",
  "provenance_limitations": [
      "input_hash covers the argument vector, not the content of the .odb it names.",
      "SCOPE: this build implements the IO RING (`make-io-sites`), single-pad placement (`place-pad`) and corner placement (`place-corners`). Filler, bond-pad and terminal placement, the bump array, distributed pad placement and connection by abutment are not implemented.",
      "A cell is refused a position by a LAYER-AWARE check, not a bounding-box one: a fixed instance blocks by box refined by its OVERLAP-layer outline where either side declares one, and anything sharing a layer blocks when the moving cell's shapes, grown by that layer's spacing, reach it. A COVER master (a bump) never blocks by box -- only by shared metal.",
      "SIMPLIFICATION: shape nets are not carried, so two shapes on the same net are treated as a conflict. The reference lets them touch. A cell being created has no nets, which is why every supported case is unaffected; a command placing already-connected cells would need them.",
      "The upstream module also contains a redistribution-layer ROUTER. That is a routing engine and is deliberately out of scope for this one; it is not merely unimplemented, it belongs elsewhere.",
      "The ring is the die area inset by the offset, corners sized from the corner site, and four edges truncated to WHOLE sites -- a remainder that does not fill a site is given up rather than rounded out.",
      "A corner's WIDTH is the larger of the corner site's width and the horizontal row's depth, so the row abutting it can be what sets the corner size.",
      "The left and right rows are laid on their side when the horizontal and vertical sites are THE SAME SITE, and upright when they differ. The reference compares the site objects; this command compares the names it was given, which is the same thing for a name that resolves to one site.",
      "MEASURED: the ring reproduces the reference row output exactly -- name, site, origin, orientation, direction, site count and pitch -- on all 26 cases that build one, including three real sky130 designs. Pad and corner placement match on all 6 comparable cases.",
      "Written against the upstream pad sources at pin b5624809f29048e1f9ce9e83eb562620c652e084. The algorithm is reimplemented from the published behavior, not transliterated."
  ],
  "invocation": {
    "args_template": ["make-io-sites", "{odb}"],
    "optional": [
      { "arg": "out", "flag": "-o" },
      { "arg": "out_odb", "flag": "--out-odb" }
    ],
    "emits_json": true
  },
  "inputs": {
    "type": "object",
    "required": ["odb", "horizontal_site", "vertical_site", "corner_site", "offset"],
    "properties": {
      "odb": { "type": "string", "description": "path to the design database (.odb)" },
      "horizontal_site": { "type": "string", "description": "site for the left and right rows" },
      "vertical_site": { "type": "string", "description": "site for the bottom and top rows" },
      "corner_site": { "type": "string", "description": "site for the four corners" },
      "offset": { "type": "string", "description": "inset from the die on every edge, in microns" },
      "out_odb": { "type": "string", "description": "write the database here instead of in place" },
      "out": { "type": "string", "description": "write the report to FILE instead of stdout" }
    }
  },
  "consumes": ["odb"],
  "produces": ["odb"],
  "artifacts": [ { "role": "ring_report", "field": "report_path" } ],
  "assertion": {
    "id": "ring-created",
    "field": "status",
    "pass_when": { "eq": "ok" }
  }
}
"#;

#[derive(Debug, Default)]
struct Opts {
    odb: String,
    keys: Vec<(String, String)>,
    dry_run: bool,
    mirror: bool,
}

impl Opts {
    fn get(&self, k: &str) -> Option<&str> {
        self.keys.iter().find(|(a, _)| a == k).map(|(_, v)| v.as_str())
    }
    fn need(&self, k: &str) -> Result<&str, String> {
        self.get(k).ok_or_else(|| format!("--{k} is required"))
    }
}

fn parse_opts(args: &[String]) -> Result<Opts, String> {
    let mut o = Opts::default();
    let mut odb = None;
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        match a {
            "--json" => {}
            "--dry-run" => o.dry_run = true,
            "--mirror" => o.mirror = true,
            // ⚠️ Flags must be listed here. Anything else beginning with `--` is read as taking a
            // value, so an unlisted flag silently swallows the argument after it.
            "--allow-non-top-layer" => o.keys.push(("allow-non-top-layer".into(), "1".into())),
            "--dont-route" => o.keys.push(("dont-route".into(), "1".into())),
            "--allow45" => o.keys.push(("allow45".into(), "1".into())),
            "--fixed" => o.keys.push(("fixed".into(), "1".into())),
            "--grid-only" => o.keys.push(("grid-only".into(), "1".into())),
            "--wires" => o.keys.push(("wires".into(), "1".into())),
            "--rebuild-each" => o.keys.push(("rebuild-each".into(), "1".into())),
            a if a.starts_with("--") || a == "-o" => {
                i += 1;
                let v = args.get(i).cloned().ok_or_else(|| format!("{a} needs a value"))?;
                o.keys.push((a.trim_start_matches('-').to_string(), v));
            }
            a if a.starts_with('-') => return Err(format!("unknown option `{a}`")),
            // ⚠️ A second bare word is rejected, not taken. Silently overwriting the design path
            // would make `pad make-io-bump-array design.odb 200 --origin ...` read a file named
            // `200` -- an unquoted list is a common way to arrive here, and the failure would look
            // like a missing file rather than a mistyped command.
            a if odb.is_some() => return Err(format!("unexpected argument `{a}`")),
            a => odb = Some(a.to_string()),
        }
        i += 1;
    }
    o.odb = odb.ok_or("a path to a .odb is required")?;
    Ok(o)
}

/// A rotation argument, in odb's spelling.
fn rotation(opts: &Opts, key: &str) -> Result<Orient, String> {
    match opts.get(key) {
        None => Ok(Orient::R0),
        Some(v) => Orient::parse(v)
            .ok_or_else(|| format!("--{key}: `{v}` is not an orientation (R0, R90, MX, MXR90, …)")),
    }
}

/// Read a site's dimensions, failing loudly if the design has no such site.
///
/// A missing site is a usage error, not an empty ring: silently building nothing would look like a
/// design that cannot carry a ring.
fn site(db: &Db, name: &str) -> Result<Site, String> {
    let (w, h) = (db.site_get_width(name), db.site_get_height(name));
    if w <= 0 || h <= 0 {
        return Err(format!("no site named `{name}` in this design"));
    }
    Ok(Site { name: name.to_string(), width: w, height: h })
}

fn def_dir(dir: RowDir) -> &'static str {
    match dir {
        RowDir::Horizontal => "HORIZONTAL",
        RowDir::Vertical => "VERTICAL",
    }
}

/// Read a row back from the database.
///
/// ℹ️ By NAME, which is safe here and is not always: `tap`'s cut rows can share a name, and a
/// by-name accessor then silently returns the first. The ring's row names are unique by
/// construction — one per edge, per corner, per ring index.
fn read_row(db: &Db, name: &str) -> Option<RowGeom> {
    let spacing = db.row_get_spacing(name);
    let sites = db.row_get_site_count(name);
    if spacing <= 0 || sites <= 0 {
        return None;
    }
    Some(RowGeom {
        name: name.to_string(),
        bbox: (
            db.row_get_b_box_x_min(name),
            db.row_get_b_box_y_min(name),
            db.row_get_b_box_x_max(name),
            db.row_get_b_box_y_max(name),
        ),
        orient: Orient::parse(&db.row_get_orient(name)).unwrap_or(Orient::R0),
        origin: (db.row_get_origin_x(name), db.row_get_origin_y(name)),
        spacing,
        site_count: sites,
    })
}

/// A master's shapes, moved to where a cell placed at `at` with this orientation would put them.
///
/// Obstructions and pin shapes together — the per-layer clearance check does not distinguish them,
/// and a cell's metal is its metal whichever collection it came from.
///
/// ℹ️ Nets are not carried. The reference lets two shapes on the SAME net touch; a corner being
/// created has no nets at all, so every shared-layer overlap is a conflict for it either way. A
/// command that places cells already connected to nets would need them.
fn cell_shapes(db: &Db, master: &str, orient: Orient, at: (i32, i32)) -> Vec<Shape> {
    let (mw, mh) = match master_size(db, master) {
        Ok(s) => s,
        Err(_) => return Vec::new(),
    };
    let mut out = Vec::new();
    for src in [db.master_obstruction_boxes(master), db.master_pin_boxes(master)] {
        for (layer, x0, y0, x1, y1) in src.unwrap_or_default() {
            out.push(Shape {
                layer: db.layer_name_by_number(layer),
                rect: transform((x0, y0, x1, y1), (mw, mh), orient, at),
                net: None,
            });
        }
    }
    out
}

/// The OVERLAP-layer part of a cell's shapes — its true outline, where it declares one.
fn cell_outline(db: &Db, shapes: &[Shape]) -> Vec<Rect> {
    outline_of(shapes, &|layer| {
        db.layer_get_type(layer).unwrap_or_default() == "OVERLAP"
    })
}

/// Everything a cell must not land on.
///
/// Two filters, and both matter — without them a correct placement is refused:
///
/// - ⚠️ **Only FIXED instances block.** A merely `PLACED` cell is not yet committed and the
///   reference walks straight past it. Testing "is it placed" instead rejects positions that are
///   perfectly legal.
/// - ⚠️ **A COVER master does not block.** Bumps sit *over* the die, not in the ring, so a bump
///   above a corner is not in its way. The reference routes them into a separate set used for RDL
///   routing instead.
///
/// ℹ️ Routing **obstructions** are deliberately absent: the reference puts those in that same
/// RDL set, not in the placement check. Placement blockages are what would belong here, and no
/// case among the supported ones uses one.
///
/// `skip` is the cell being placed — an instance that already exists must not block itself, or
/// re-running a placement would refuse the position it already holds.
/// Placement blockages, which forbid a cell outright wherever they reach.
///
/// ⚠️ Not the same as a routing obstruction (which [`blockers`] carries): a blockage has no layer,
/// so there is nothing to compare per layer and nothing an outline can refine.
fn blockages(db: &Db) -> Vec<(i32, i32, i32, i32)> {
    db.blockage_boxes().unwrap_or_default()
}

fn blockers(db: &Db, skip: &str) -> Vec<Blocker> {
    let mut out: Vec<Blocker> = Vec::new();
    for name in db.inst_names() {
        if name == skip {
            continue;
        }
        if !matches!(db.inst_get_placement_status(&name).as_str(), "FIRM" | "LOCKED" | "COVER") {
            continue;
        }
        let Ok(b) = db.inst_bbox(&name) else { continue };
        let [x0, y0, x1, y1] = b[..] else { continue };

        let master = db.inst_get_master(&name);
        // ⚠️ **Prefix, not equality.** odb reports a master's class as the LEF spells it, and
        // `CLASS COVER BUMP` comes back as `"COVER BUMP"`. An equality test against `"COVER"`
        // matches nothing, blocks every bump, and fails silently — the same trap `tap` hit with
        // LEF58 endcap types.
        let is_cover =
            db.master_get_type(&master).unwrap_or_default().starts_with("COVER");
        let orient = Orient::parse(&db.inst_get_orient(&name)).unwrap_or(Orient::R0);
        let shapes = cell_shapes(db, &master, orient, (x0, y0));
        out.push(Blocker {
            name,
            bbox: (x0, y0, x1, y1),
            outline: cell_outline(db, &shapes),
            // A cover cell sits OVER the die and is judged only by the metal it shares with the
            // cell being placed — never by its bounding box.
            by_box: !is_cover,
            // ⚠️ **Either box or metal, never both.** The reference files a fixed instance into one
            // of two collections: an ordinary cell by box and outline, a cover cell by its shapes
            // per layer. Carrying both here refuses two ordinary cells placed flush against each
            // other, because their metal is within a layer's spacing even though their boxes only
            // touch. That is legal, and common: pads abut.
            shapes: if is_cover { shapes } else { Vec::new() },
        });
    }
    // Routing obstructions take part in the per-layer check, not the box one.
    for (layer, x0, y0, x1, y1) in db.obstruction_boxes().unwrap_or_default() {
        out.push(Blocker {
            name: format!("obstruction@{layer}"),
            bbox: (x0, y0, x1, y1),
            outline: vec![],
            by_box: false,
            shapes: vec![Shape {
                layer: db.layer_name_by_number(layer),
                rect: (x0, y0, x1, y1),
                net: None,
            }],
        });
    }
    out
}

/// The size of a master, as the placer needs it.
fn master_size(db: &Db, master: &str) -> Result<(i32, i32), String> {
    let (w, h) = (db.master_get_width(master) as i32, db.master_get_height(master) as i32);
    if w <= 0 || h <= 0 {
        return Err(format!("no master named `{master}` in this design"));
    }
    Ok((w, h))
}

/// Write the instance out, creating it only if the design does not already have it.
fn commit(db: &mut Db, p: &Placement, create: bool) -> Result<(), String> {
    if create {
        db.create_inst(&p.master, &p.name).map_err(|e| format!("cannot create {}: {e}", p.name))?;
    }
    db.inst_set_orient(&p.name, &format!("{:?}", p.orient))
        .map_err(|e| format!("cannot orient {}: {e}", p.name))?;
    db.inst_set_location(&p.name, p.x, p.y)
        .map_err(|e| format!("cannot place {}: {e}", p.name))?;
    db.inst_set_placement_status(&p.name, "FIRM")
        .map_err(|e| format!("cannot fix {}: {e}", p.name))?;
    Ok(())
}

/// **C5, C6** — a cell in each of the four corner rows.
///
/// ⚠️ A corner that would land on something already there is **skipped**, not moved: the reference
/// drops it and warns. One reference case places only three corners for exactly this reason, so
/// placing all four unconditionally does not merely differ, it fails.
fn place_corners(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let master = match opts.need("master").map(str::to_string) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("vyges-pad: {e}\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let ring_index: i32 = match opts.get("ring-index") {
        None => -1,
        Some(v) => match v.parse() {
            Ok(n) => n,
            Err(_) => {
                eprintln!("vyges-pad: --ring-index wants a whole number");
                return ExitCode::from(2);
            }
        },
    };
    let size = match master_size(&db, &master) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };

    let mut placed = Vec::new();
    let mut skipped = Vec::new();
    for row_name in corner_row_names(ring_index) {
        let Some(row) = read_row(&db, &row_name) else {
            eprintln!("vyges-pad: no row `{row_name}` to place a corner in");
            return ExitCode::from(2);
        };
        let p = corner_placement(&row, &master);
        let (dx, dy) = oriented_size(size.0, size.1, p.orient);
        let bbox = (p.x, p.y, p.x + dx, p.y + dy);
        let shapes = cell_shapes(&db, &master, p.orient, (p.x, p.y));
        let outline = cell_outline(&db, &shapes);
        if let Some(why) = refuse(
            &p.name,
            bbox,
            &outline,
            &shapes,
            &blockers(&db, &p.name),
            &blockages(&db),
            &|layer| db.layer_get_spacing(layer),
        ) {
            skipped.push(format!("{} ({why:?})", p.name));
            continue;
        }
        if !opts.dry_run {
            if let Err(e) = commit(&mut db, &p, true) {
                eprintln!("vyges-pad: {e}");
                return ExitCode::from(2);
            }
        }
        placed.push(p);
    }
    finish(&opts, &mut db, "corners", &placed, &skipped)
}

/// **C1–C4** — one pad at a location along a row.
fn place_pad(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let dbu = db.dbu_per_micron();

    let built = (|| -> Result<(Placement, (i32, i32), bool), String> {
        let inst = opts.need("inst")?.to_string();
        let row_name = opts.need("row")?.to_string();
        let location_um: f64 =
            opts.need("location")?.parse().map_err(|_| "--location wants microns".to_string())?;
        let location = (location_um * dbu as f64).round() as i32;

        let existing = db.inst_names().iter().any(|n| n == &inst);
        let master = match opts.get("master") {
            Some(m) => m.to_string(),
            None if existing => db.inst_get_master(&inst),
            None => return Err(format!("cannot create {inst} without --master")),
        };
        // The reference refuses a master that contradicts the instance already in the design,
        // rather than silently re-mastering it.
        if existing {
            let have = db.inst_get_master(&inst);
            if !have.is_empty() && have != master {
                return Err(format!("master mismatch for {inst}: it is {have}, not {master}"));
            }
        }

        let row = read_row(&db, &row_name).ok_or(format!("no row named `{row_name}`"))?;
        let edge = row.edge().ok_or(format!("`{row_name}` is not an IO row"))?;
        let size = master_size(&db, &master)?;

        let base = if opts.mirror {
            let site = db.row_get_site(&row_name);
            mirror_base(edge, db.site_get_width(&site), db.site_get_height(&site))
        } else {
            Orient::R0
        };
        let index = snap_to_site(location, &row, edge);
        let (x, y, orient) = place_in_row(index, &row, edge, size.0, size.1, base);
        Ok((Placement { name: inst, master, x, y, orient }, size, !existing))
    })();

    let (p, size, create) = match built {
        Ok(v) => v,
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };
    let _ = size;
    if !opts.dry_run {
        if let Err(e) = commit(&mut db, &p, create) {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    }
    finish(&opts, &mut db, "pad", &[p], &[])
}

/// Parse, open the database, and check it has a scale. Shared by every verb.
fn open(args: &[String]) -> Result<(Opts, Db), ExitCode> {
    let opts = match parse_opts(args) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("vyges-pad: {e}\n\n{USAGE}");
            return Err(ExitCode::from(2));
        }
    };
    let db = match Db::open(&opts.odb) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("vyges-pad: cannot read {}: {e}", opts.odb);
            return Err(ExitCode::from(2));
        }
    };
    if db.dbu_per_micron() <= 0 {
        eprintln!("vyges-pad: no DBU scale");
        return Err(ExitCode::from(2));
    }
    Ok((opts, db))
}

/// Write the database and the report, and pick the exit status.
fn finish(
    opts: &Opts,
    db: &mut Db,
    what: &str,
    placed: &[Placement],
    skipped: &[String],
) -> ExitCode {
    if !opts.dry_run {
        let out_odb = opts.get("out-odb").unwrap_or(&opts.odb).to_string();
        if let Err(e) = db.write(&out_odb) {
            eprintln!("vyges-pad: cannot write {out_odb}: {e}");
            return ExitCode::from(2);
        }
        if let Some(path) = opts.get("out-def") {
            if let Err(e) = db.write_def(path) {
                eprintln!("vyges-pad: cannot write {path}: {e}");
                return ExitCode::from(2);
            }
        }
    }
    emit_placement_events(what, placed, skipped);
    let report = placement_json(what, placed, skipped);
    match opts.get("o") {
        Some(path) => {
            if let Err(e) = std::fs::write(path, format!("{report}\n")) {
                eprintln!("vyges-pad: cannot write {path}: {e}");
                return ExitCode::from(2);
            }
        }
        None => println!("{report}"),
    }
    // A skipped corner is the reference's own behaviour, not a failure of this run.
    if placed.is_empty() {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}

/// Write out and report a command whose result is connections rather than placements.
fn finish_report(
    opts: &Opts,
    db: &mut Db,
    what: &str,
    made: &[String],
    removed: &[String],
) -> ExitCode {
    if !opts.dry_run {
        let out_odb = opts.get("out-odb").unwrap_or(&opts.odb).to_string();
        if let Err(e) = db.write(&out_odb) {
            eprintln!("vyges-pad: cannot write {out_odb}: {e}");
            return ExitCode::from(2);
        }
        if let Some(path) = opts.get("out-def") {
            if let Err(e) = db.write_def(path) {
                eprintln!("vyges-pad: cannot write {path}: {e}");
                return ExitCode::from(2);
            }
        }
    }
    let quoted = |v: &[String]| {
        v.iter().map(|s| format!("\"{}\"", escape(s))).collect::<Vec<_>>().join(", ")
    };
    let report = format!(
        "{{\n  \"tool\": \"vyges-pad\",\n  \"command\": \"{what}\",\n  \"status\": \"ok\",\n  \
         \"connections\": {},\n  \"removed\": [{}],\n  \"made\": [{}]\n}}",
        made.len(),
        quoted(removed),
        quoted(made),
    );
    match opts.get("o") {
        Some(path) => {
            if let Err(e) = std::fs::write(path, format!("{report}\n")) {
                eprintln!("vyges-pad: cannot write {path}: {e}");
                return ExitCode::from(2);
            }
        }
        None => println!("{report}"),
    }
    // ⚠️ Zero connections is a legitimate answer here -- a ring with nothing touching is unusual,
    // not wrong -- so this does not take the empty-result exit code the placement path uses.
    ExitCode::SUCCESS
}

/// ⚠️ Instance names arrive with DEF's own escaping — `u_io\\[10\\]` is one name, backslashes and
/// all. Writing them into JSON unquoted produces a file no JSON parser will read, which is a bug
/// in the report rather than in the design.
fn escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

fn placement_json(what: &str, placed: &[Placement], skipped: &[String]) -> String {
    let list = placed
        .iter()
        .map(|p| {
            format!(
                "    {{\"inst\": \"{}\", \"master\": \"{}\", \"x\": {}, \"y\": {}, \
                 \"orient\": \"{:?}\"}}",
                p.name, p.master, p.x, p.y, p.orient
            )
        })
        .collect::<Vec<_>>()
        .join(",\n");
    let missed =
        skipped.iter().map(|s| format!("\"{}\"", escape(s))).collect::<Vec<_>>().join(", ");
    format!(
        "{{\n  \"tool\": \"vyges-pad\",\n  \"command\": \"{what}\",\n  \"status\": \"{}\",\n  \
         \"placed\": {},\n  \"skipped\": [{missed}],\n  \"placements\": [\n{list}\n  ]\n}}",
        if placed.is_empty() { "refused" } else { "ok" },
        placed.len(),
    )
}

fn emit_placement_events(what: &str, placed: &[Placement], skipped: &[String]) {
    use vyges_events::{Event, Severity};
    for s in skipped {
        // Upstream warns and moves on. Saying so is the difference between "there is no corner
        // there" and "nobody looked".
        vyges_events::emit(
            &Event::new(
                "vyges-pad",
                Severity::Warn,
                format!("skipping {s}"),
            )
            .with_code("PAD-SKIP-OVERLAP")
            .with_objects(vec![format!("inst:{s}")]),
        );
    }
    vyges_events::emit(
        &Event::new(
            "vyges-pad",
            Severity::Info,
            format!("{what}: placed {}, skipped {}", placed.len(), skipped.len()),
        )
        .with_code("PAD-PLACED"),
    );
}

/// A placed cell's size on the die, after its orientation.
fn oriented_size_of(db: &Db, master: &str, orient: Orient) -> (i32, i32) {
    let (w, h) = master_size(db, master).unwrap_or((0, 0));
    oriented_size(w, h, orient)
}

/// Every pad instance, gathered row by row exactly as the reference gathers them.
///
/// Fixed, not a cover cell, and overlapping a row. ⚠️ **All** rows, core rows included — that is
/// what the reference iterates, and narrowing it to the IO rows would be a different census.
fn pad_insts(db: &Db) -> Vec<PadInst> {
    let names = db.inst_names();
    let mut out = Vec::new();
    // ⚠️ **Row by row, not instance by instance.** The two visit the same set and produce a
    // different ORDER, and order decides which terminal names a net created by abutment. Doing a
    // single pass over instances gets identical connectivity and the wrong net names.
    for row in db.row_names().unwrap_or_default() {
        let r = (
            db.row_get_b_box_x_min(&row),
            db.row_get_b_box_y_min(&row),
            db.row_get_b_box_x_max(&row),
            db.row_get_b_box_y_max(&row),
        );
        for name in &names {
            if !matches!(db.inst_get_placement_status(name).as_str(), "FIRM" | "LOCKED" | "COVER")
            {
                continue;
            }
            if db.master_is_cover(&db.inst_get_master(name)) {
                continue;
            }
            let Some(inst) = read_inst(db, name) else { continue };
            // ⚠️ `intersects` here is the strict test: a cell merely grazing a row's edge is not
            // in that row. This is the clearance predicate, not the abutment one.
            if !intersects(inst.bbox, r) {
                continue;
            }
            // ⚠️ A cell reaching into two rows is collected TWICE, deliberately: the reference
            // does the same, and deduplicating would change the order everything below depends on.
            out.push(inst);
        }
    }
    out
}

/// One instance with its terminals' shapes moved onto the die.
///
/// The single place that reads an instance's geometry. Both the abutment census and the bond-pad
/// pairing use it: a second copy would be free to drift from this one, silently.
fn read_inst(db: &Db, name: &str) -> Option<PadInst> {
    let master = db.inst_get_master(name);
    let b = db.inst_bbox(name).ok()?;
    let [x0, y0, x1, y1] = b[..] else { return None };
    let (mw, mh) = master_size(db, &master).ok()?;
    let orient = Orient::parse(&db.inst_get_orient(name)).unwrap_or(Orient::R0);
    let terms = db
        .master_get_m_terms(&master)
        .into_iter()
        .map(|term| {
            let shapes = db
                .mterm_pin_boxes(&master, &term)
                .unwrap_or_default()
                .into_iter()
                .map(|(layer, a, b, c, d)| {
                    (layer, transform((a, b, c, d), (mw, mh), orient, (x0, y0)))
                })
                .collect();
            let net = db.iterm_get_net(name, &term);
            Terminal {
                supply: matches!(db.iterm_get_sig_type(name, &term).as_str(), "POWER" | "GROUND"),
                name: term,
                shapes,
                net: (!net.is_empty()).then_some(net),
            }
        })
        .collect();
    Some(PadInst { name: name.to_string(), bbox: (x0, y0, x1, y1), terms })
}

/// **A6** — mark a net special, and settle its signal type.
fn make_special(db: &mut Db, net: &str) -> Result<(), String> {
    let iterms: Vec<(String, String)> = db
        .net_get_i_terms(net)
        .iter()
        .filter_map(|t| t.rsplit_once('/').map(|(i, p)| (i.to_string(), p.to_string())))
        .collect();
    let types: Vec<(bool, String)> = iterms
        .iter()
        .map(|(i, p)| {
            let ty = db.iterm_get_sig_type(i, p);
            (matches!(ty.as_str(), "POWER" | "GROUND"), ty)
        })
        .collect();
    let sig = special_sig_type(&db.net_get_sig_type(net), &types);

    db.net_set_special(net).map_err(|e| format!("cannot mark {net} special: {e}"))?;
    for (i, p) in &iterms {
        db.iterm_set_special(i, p).map_err(|e| format!("cannot mark {i}/{p} special: {e}"))?;
    }
    for bterm in db.net_get_b_terms(net) {
        db.bterm_set_special(&bterm).map_err(|e| format!("cannot mark {bterm} special: {e}"))?;
        db.bterm_set_sig_type(&bterm, &sig).map_err(|e| format!("cannot type {bterm}: {e}"))?;
    }
    db.net_set_sig_type(net, &sig).map_err(|e| format!("cannot type {net}: {e}"))
}

/// **G9** — the pin shapes the router has to reach, per net.
///
/// One target per pin rectangle on the routing layer, on every **placed** instance the net
/// touches. ⚠️ Unplaced instances are skipped rather than routed to where they are not.
fn rdl_targets(db: &Db, net: &str, layer: &str) -> Vec<rdl::Target> {
    let mut out = Vec::new();
    for iterm in db.net_get_i_terms(net) {
        let Some((inst, term)) = iterm.rsplit_once('/') else { continue };
        if !db.inst_is_placed(inst) {
            continue;
        }
        let master = db.inst_get_master(inst);
        let orient = Orient::parse(&db.inst_get_orient(inst)).unwrap_or(Orient::R0);
        let origin = (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst));
        for (l, x0, y0, x1, y1) in db.mterm_pin_boxes(&master, term).unwrap_or_default() {
            if db.layer_name_by_number(l) != layer {
                continue;
            }
            let shape = pin_shape((x0, y0, x1, y1), orient, origin);
            out.push(rdl::Target {
                terminal: iterm.clone(),
                centre: ((shape.0 + shape.2) / 2, (shape.1 + shape.3) / 2),
                shape,
                access: Vec::new(),
            });
        }
    }
    out
}

/// **G8** — everything the RDL router must not run into, as bloated rectangles.
///
/// Four sources in the reference; the two that placed instances contribute are the ones every
/// design has:
///
/// 1. each master's **obstructions on the routing layer, minus its own pin shapes** — the pins are
///    what the router is trying to reach, so they are carved back out;
/// 2. each instance's pin shapes, which block everything except the net they belong to.
///
/// ⚠️ Bloating happens **before** the subtraction, not after. Bloating the result instead would
/// eat into the pin openings by the clearance and could close them entirely.
fn rdl_obstructions(db: &Db, layer: &str, bloat: i32) -> Vec<(i32, i32, i32, i32)> {
    use vyges_loom::poly90::{Poly90Set, Rect as P90Rect};
    let grow = |r: (i32, i32, i32, i32)| (r.0 - bloat, r.1 - bloat, r.2 + bloat, r.3 + bloat);
    let on_layer = |v: Vec<(i64, i32, i32, i32, i32)>| -> Vec<(i32, i32, i32, i32)> {
        v.into_iter()
            .filter(|(l, ..)| db.layer_name_by_number(*l) == layer)
            .map(|(_, a, b, c, d)| (a, b, c, d))
            .collect()
    };

    let mut cache: std::collections::HashMap<String, Vec<(i32, i32, i32, i32)>> =
        std::collections::HashMap::new();
    let mut out = Vec::new();

    for inst in db.inst_names() {
        if !db.inst_is_placed(&inst) {
            continue;
        }
        let master = db.inst_get_master(&inst);
        let orient = Orient::parse(&db.inst_get_orient(&inst)).unwrap_or(Orient::R0);
        let origin = (db.inst_get_origin_x(&inst), db.inst_get_origin_y(&inst));

        let shape = cache.entry(master.clone()).or_insert_with(|| {
            let add: Vec<P90Rect> = on_layer(db.master_obstruction_boxes(&master).unwrap_or_default())
                .into_iter()
                .map(|r| { let g = grow(r); P90Rect::new(g.0, g.1, g.2, g.3) })
                .collect();
            if add.is_empty() {
                return Vec::new();
            }
            let sub: Vec<P90Rect> = on_layer(db.master_pin_boxes(&master).unwrap_or_default())
                .into_iter()
                .map(|r| { let g = grow(r); P90Rect::new(g.0, g.1, g.2, g.3) })
                .collect();
            let mut set = Poly90Set::from_rects(&add);
            if !sub.is_empty() {
                set = set.difference(&Poly90Set::from_rects(&sub));
            }
            set.rects().into_iter().map(|r| (r.x0, r.y0, r.x1, r.y1)).collect()
        });
        for r in shape.clone() {
            // Already bloated, so it moves onto the die unchanged.
            out.push(pin_shape(r, orient, origin));
        }

        // The instance's own pin metal.
        for term in db.master_get_m_terms(&master) {
            for r in on_layer(db.mterm_pin_boxes(&master, &term).unwrap_or_default()) {
                out.push(grow(pin_shape(r, orient, origin)));
            }
        }
    }
    out
}

/// **G1-G5** — the RDL routing grid.
///
/// ⚠️ Only the grid. The search, obstructions and rip-up are later stages, and asking for a route
/// exits 3 rather than producing one this engine cannot yet stand behind.
fn rdl_route(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let dbu = db.dbu_per_micron();

    let built = (|| -> Result<rdl::Grid, String> {
        let layer = opts.need("layer")?;
        let um = |k: &str| -> Result<i32, String> {
            match opts.get(k) {
                None => Ok(0),
                Some(v) => v
                    .parse::<f64>()
                    .map(|n| (n * dbu as f64).round() as i32)
                    .map_err(|_| format!("--{k} wants a number, got `{v}`")),
            }
        };
        let (width, spacing) = (um("width")?, um("spacing")?);
        let (tx, ty) = db
            .track_grid(layer)
            .map_err(|e| format!("no track grid on `{layer}`: {e}"))?;
        Ok(rdl::grid(&tx, &ty, width, spacing))
    })();

    let g = match built {
        Ok(v) => v,
        Err(e) => return unsupported_or_error(&e),
    };
    let allow45 = opts.get("allow45").is_some();

    let layer = opts.get("layer").unwrap_or_default().to_string();
    let width = opts.get("width").and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
    let spacing = opts.get("spacing").and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
    let to_dbu = |v: f64| (v * dbu as f64).round() as i32;
    let bloat = to_dbu(width) / 2 + to_dbu(spacing);
    let obstructions = rdl_obstructions(&db, &layer, bloat);
    let clear = rdl::edges_clear(&g, allow45, &|a, b| rdl::blocked(a, b, &obstructions));

    // Routing targets, per net, for every net named on the command line.
    let mut nets: Vec<String> = Vec::new();
    if let Some(pats) = opts.get("nets") {
        let pats: Vec<String> = pats.split_whitespace().map(str::to_string).collect();
        nets = matching(&db.net_names(), &pats)
            .into_iter()
            .cloned()
            .collect();
    }
    let mut target_report = String::new();
    let mut total_targets = 0usize;
    for n in &nets {
        let t = rdl_targets(&db, n, &layer);
        let iterms: std::collections::BTreeSet<&String> =
            t.iter().map(|x| &x.terminal).collect();
        total_targets += t.len();
        target_report.push_str(&format!("{n} has {} targets\n", iterms.len()));
    }
    // The order routes would be attempted in, for checking the queue against the reference.
    if let Some(path) = opts.get("order-report") {
        let mut routes: Vec<rdl::Route> = Vec::new();
        let mut ties = 0usize;
        for (i, n) in nets.iter().enumerate() {
            let targets = rdl_targets(&db, n, &layer);
            // One destination per terminal, at its first target's centre.
            let mut seen = std::collections::BTreeSet::new();
            let mut dests = Vec::new();
            for t in &targets {
                if !seen.insert(t.terminal.clone()) {
                    continue;
                }
                let (inst, pin) = t.terminal.rsplit_once('/').unwrap_or(("", ""));
                let (inst, pin) = (inst.to_string(), pin.to_string());
                let cover = db.master_is_cover(&db.inst_get_master(&inst));
                // ⚠️ The ORDERING distance is measured between iterm bounding-box centres, which
                // span every layer's pin shapes — not between the routing layer's target centres.
                // The two coincide for a single-shape pin and diverge for a pad carrying several,
                // which reorders whole groups rather than individual routes.
                let bb = (
                    db.iterm_get_b_box_x_min(&inst, &pin),
                    db.iterm_get_b_box_y_min(&inst, &pin),
                    db.iterm_get_b_box_x_max(&inst, &pin),
                    db.iterm_get_b_box_y_max(&inst, &pin),
                );
                let iterm_id = db.iterm_id(&inst, &pin).unwrap_or(0) as u64;
                dests.push(rdl::Dest {
                    terminal: t.terminal.clone(),
                    instance: inst,
                    centre: ((bb.0 + bb.2) / 2, (bb.1 + bb.3) / 2),
                    cover,
                    // The database's own identifier. It settles more than half the ordering on
                    // this design, so a stand-in is not good enough.
                    id: iterm_id,
                });
            }
            for d in dests.clone() {
                if !d.cover {
                    continue;
                }
                let ordered = rdl::order_dests(&d.instance, d.centre, &dests);
                if ordered.is_empty() {
                    continue;
                }
                routes.push(rdl::Route {
                    source: d.terminal.clone(),
                    instance: d.instance.clone(),
                    centre: d.centre,
                    id: d.id,
                    dests: ordered,
                    next: 0,
                    priority: 0,
                    routed: false,
                    pending: true,
                    points: Vec::new(),
                });
            }
        }
        // ⚠️ Count ties on the DISTANCE, not on the comparator's result. `precedes` settles a
        // distance tie by identifier and therefore never reports `Equal` — counting its `Equal`
        // results measures nothing and will report zero however many ties there are.
        {
            let mut keys: Vec<i64> = routes
                .iter()
                .filter(|r| r.priority == 0)
                .filter_map(|r| {
                    r.peek().map(|d| {
                        let (dx, dy) =
                            ((r.centre.0 - d.centre.0) as i64, (r.centre.1 - d.centre.1) as i64);
                        dx * dx + dy * dy
                    })
                })
                .collect();
            keys.sort_unstable();
            ties = keys.windows(2).filter(|w| w[0] == w[1]).count();
        }
        routes.sort_by(|a, b| a.precedes(b));
        // The reference logs TARGET centres, so report those rather than the ordering centres.
        let target_of: std::collections::HashMap<String, (i32, i32)> = nets
            .iter()
            .flat_map(|n| rdl_targets(&db, n, &layer))
            .map(|t| (t.terminal, t.centre))
            .collect();
        let body: String = routes
            .iter()
            .take(300)
            .map(|r| {
                let d = r.peek().unwrap();
                let s = target_of.get(&r.source).copied().unwrap_or(r.centre);
                let t = target_of.get(&d.terminal).copied().unwrap_or(d.centre);
                format!("{} {} -> {} {}\n", s.0, s.1, t.0, t.1)
            })
            .collect();
        eprintln!("routes: {}  ordering ties hit: {ties}", routes.len());
        if let Err(e) = std::fs::write(path, body) {
            eprintln!("vyges-pad: cannot write {path}: {e}");
            return ExitCode::from(2);
        }
        return ExitCode::SUCCESS;
    }

    // A single route attempt, for checking the search against the reference's own path length.
    if let Some(spec) = opts.get("probe") {
        let n: Vec<i32> = spec.split_whitespace().filter_map(|t| t.parse().ok()).collect();
        let [sx, sy, tx, ty] = n[..] else {
            eprintln!("vyges-pad: --probe wants `sx sy tx ty`");
            return ExitCode::from(2);
        };
        let turn = opts.get("turn-penalty").and_then(|v| v.parse().ok()).unwrap_or(2.0f32);
        let mut graph = rdl::Graph::build(&g, &clear, 1.0);
        // Graft both terminals onto the grid, as the reference does before each attempt.
        for centre in [(sx, sy), (tx, ty)] {
            let own: Vec<(i32, i32, i32, i32)> = obstructions
                .iter()
                .copied()
                .filter(|r| rdl::hits(centre, centre, *r))
                .collect();
            let t = rdl::Target {
                terminal: String::new(),
                centre,
                shape: (centre.0, centre.1, centre.0, centre.1),
                access: Vec::new(),
            };
            let snaps = rdl::access_points(&g, &t, &obstructions, &own);
            rdl::insert_access(&mut graph, &g, centre, &snaps);
        }
        let path = rdl::shortest_path(&graph, (sx, sy), (tx, ty), turn);
        println!("{{\"segments\": {}}}", path.len());
        if opts.get("wires").is_some() {
            // The pin rectangles the two ends land on, so the end runs can reach into them.
            let shape_of = |c: (i32, i32)| {
                nets.iter()
                    .flat_map(|n| rdl_targets(&db, n, &layer))
                    .find(|t| t.centre == c)
                    .map(|t| t.shape)
                    .unwrap_or((c.0, c.1, c.0, c.1))
            };
            let w = to_dbu(width);
            for piece in rdl::wires(&path, w, shape_of((sx, sy)), shape_of((tx, ty))) {
                match piece {
                    rdl::Wire::Straight(r) => {
                        println!("RECT {} {} {} {}", r.0, r.1, r.2, r.3)
                    }
                    rdl::Wire::Diagonal(a, b, w) => {
                        println!("DIAG {} {} {} {} {w}", a.0, a.1, b.0, b.1)
                    }
                }
            }
        }
        return ExitCode::SUCCESS;
    }
    if let Some(path) = opts.get("net-centre-report") {
        let mut body = String::new();
        for n in &nets {
            for t in rdl_targets(&db, n, &layer) {
                body.push_str(&format!("{n} {} {} {}\n", t.centre.0, t.centre.1, t.terminal));
            }
        }
        if let Err(e) = std::fs::write(path, body) {
            eprintln!("vyges-pad: cannot write {path}: {e}");
            return ExitCode::from(2);
        }
    }
    if let Some(path) = opts.get("centre-report") {
        let mut all = std::collections::BTreeSet::new();
        for n in &nets {
            for t in rdl_targets(&db, n, &layer) {
                all.insert(t.centre);
            }
        }
        let body: String =
            all.iter().map(|(x, y)| format!("{x} {y}\n")).collect();
        if let Err(e) = std::fs::write(path, body) {
            eprintln!("vyges-pad: cannot write {path}: {e}");
            return ExitCode::from(2);
        }
    }
    if let Some(path) = opts.get("target-report") {
        if let Err(e) = std::fs::write(path, &target_report) {
            eprintln!("vyges-pad: cannot write {path}: {e}");
            return ExitCode::from(2);
        }
    }

    let report = format!(
        "{{\n  \"tool\": \"vyges-pad\",\n  \"command\": \"rdl-grid\",\n  \"status\": \"ok\",\n  \
         \"vertices\": {},\n  \"edges\": {},\n  \"obstructions\": {},\n  \"nets\": {},\n  \
         \"targets\": {},\n  \"columns\": {},\n  \"rows\": {}\n}}",
        g.vertices(),
        clear.len(),
        obstructions.len(),
        nets.len(),
        total_targets,
        g.x.len(),
        g.y.len(),
    );
    match opts.get("o") {
        Some(path) => {
            if let Err(e) = std::fs::write(path, format!("{report}\n")) {
                eprintln!("vyges-pad: cannot write {path}: {e}");
                return ExitCode::from(2);
            }
        }
        None => println!("{report}"),
    }
    if opts.get("grid-only").is_some() {
        return ExitCode::SUCCESS;
    }

    // ── The run itself ───────────────────────────────────────────────────────────────────────
    let turn = opts.get("turn-penalty").and_then(|v| v.parse().ok()).unwrap_or(2.0f32);
    let max_iters = opts.get("max-iterations").and_then(|v| v.parse().ok()).unwrap_or(10);
    let w = to_dbu(width);
    let sp = to_dbu(spacing);

    let mut access: std::collections::HashMap<String, (rdl::Point, Vec<rdl::Point>, String)> =
        std::collections::HashMap::new();
    let mut shape_of: std::collections::HashMap<String, (i32, i32, i32, i32)> =
        std::collections::HashMap::new();
    let mut routes: Vec<rdl::Route> = Vec::new();

    for n in &nets {
        let targets = rdl_targets(&db, n, &layer);
        let mut dests: Vec<rdl::Dest> = Vec::new();
        let mut seen = std::collections::BTreeSet::new();
        for t in &targets {
            if !seen.insert(t.terminal.clone()) {
                continue;
            }
            let (inst, pin) = t.terminal.rsplit_once('/').unwrap_or(("", ""));
            let (inst, pin) = (inst.to_string(), pin.to_string());
            let bb = (
                db.iterm_get_b_box_x_min(&inst, &pin),
                db.iterm_get_b_box_y_min(&inst, &pin),
                db.iterm_get_b_box_x_max(&inst, &pin),
                db.iterm_get_b_box_y_max(&inst, &pin),
            );
            let own: Vec<(i32, i32, i32, i32)> = obstructions
                .iter()
                .copied()
                .filter(|r| rdl::hits(t.centre, t.centre, *r))
                .collect();
            let snaps = rdl::access_points(&g, t, &obstructions, &own);
            access.insert(t.terminal.clone(), (t.centre, snaps, n.clone()));
            shape_of.insert(t.terminal.clone(), t.shape);
            dests.push(rdl::Dest {
                terminal: t.terminal.clone(),
                instance: inst.clone(),
                centre: ((bb.0 + bb.2) / 2, (bb.1 + bb.3) / 2),
                cover: db.master_is_cover(&db.inst_get_master(&inst)),
                id: db.iterm_id(&inst, &pin).unwrap_or(0) as u64,
            });
        }
        for d in dests.clone().into_iter().filter(|d| d.cover) {
            let ordered = rdl::order_dests(&d.instance, d.centre, &dests);
            if ordered.is_empty() {
                continue;
            }
            routes.push(rdl::Route {
                source: d.terminal.clone(),
                instance: d.instance.clone(),
                centre: d.centre,
                id: d.id,
                dests: ordered,
                next: 0,
                priority: 0,
                routed: false,
                pending: true,
                points: Vec::new(),
            });
        }
    }

    let mut graph = rdl::Graph::build(&g, &clear, 1.0);
    let done =
        rdl::route_all(
            &mut graph,
            &g,
            &mut routes,
            &access,
            w,
            sp,
            turn,
            max_iters,
            opts.get("rebuild-each").map(|_| clear.as_slice()),
        );

    if !opts.dry_run {
        let applied = (|| -> Result<(), String> {
            // The reference replaces its own previous result and leaves fixed wires alone.
            for net in done.paths.iter().map(|(n, ..)| n.clone()).collect::<std::collections::BTreeSet<_>>() {
                db.clear_routed_swires(&net).map_err(|e| format!("{net}: {e}"))?;
            }
            for (net, src, dst, path) in &done.paths {
                let s = shape_of.get(src).copied().unwrap_or_default();
                let t = shape_of.get(dst).copied().unwrap_or_default();
                make_special(&mut db, net)?;
                for piece in rdl::wires(path, w, s, t) {
                    if let rdl::Wire::Straight(r) = piece {
                        db.add_swire_box(net, &layer, r, opts.get("fixed").is_some())
                            .map_err(|e| format!("cannot write a wire on {net}: {e}"))?;
                    }
                }
            }
            Ok(())
        })();
        if let Err(e) = applied {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    }

    if let Some(path) = opts.get("attempt-report") {
        let body: String = done
            .log
            .iter()
            .map(|(s, t, n)| format!("{} {} {} {} {n}\n", s.0, s.1, t.0, t.1))
            .collect();
        if let Err(e) = std::fs::write(path, body) {
            eprintln!("vyges-pad: cannot write {path}: {e}");
            return ExitCode::from(2);
        }
    }
    let made: Vec<String> =
        done.paths.iter().map(|(n, s, d, _)| format!("{n}: {s} -> {d}")).collect();
    eprintln!(
        "routes {} of {}, {} attempts, {} iterations",
        done.paths.len(),
        routes.len(),
        done.attempts,
        done.iterations
    );
    finish_report(&opts, &mut db, "rdl-route", &made, &done.failed)
}

/// **B1** — assign a net to a bump.
fn assign_io_bump(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };

    let prepared = (|| -> Result<(String, String, Vec<BumpTerm>, Rect, Option<String>), String> {
        let bump = opts.need("bump")?.to_string();
        let net = opts.need("net")?.to_string();
        if db.net_get_sig_type(&net).is_empty() && db.net_get_i_term_count(&net) == 0 {
            // A net that exists has a signal type; one that does not would be created silently.
            return Err(format!("no net named `{net}` in this design"));
        }
        let master = db.inst_get_master(&bump);
        if master.is_empty() {
            return Err(format!("no instance named `{bump}` in this design"));
        }
        let class = db.master_get_type(&master).unwrap_or_default();
        if !is_bump_master(&class) {
            return Err(format!("{bump} is a `{class}` cell, not a bump"));
        }

        let terms = db
            .master_get_m_terms(&master)
            .into_iter()
            .map(|term| {
                let shapes = db
                    .mterm_pin_boxes(&master, &term)
                    .unwrap_or_default()
                    .into_iter()
                    .map(|(l, x0, y0, x1, y1)| {
                        let name = db.layer_name_by_number(l);
                        let level = db.layer_get_routing_level(&name);
                        (name, level, (x0, y0, x1, y1))
                    })
                    .collect();
                let n = db.iterm_get_net(&bump, &term);
                BumpTerm { name: term, net: (!n.is_empty()).then_some(n), shapes }
            })
            .collect();

        let (mw, mh) = master_size(&db, &master)
            .map_err(|e| format!("cannot measure master `{master}`: {e}"))?;
        Ok((bump, net, terms, (0, 0, mw, mh), opts.get("terminal").map(str::to_string)))
    })();

    let (bump, net, terms, master_box, terminal) = match prepared {
        Ok(v) => v,
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };

    // ⚠️ `-dont_route` and `-terminal` are mutually exclusive in the reference, and both only feed
    // the RDL router's map. With no router here, `--dont-route` records intent and changes nothing.
    if opts.get("dont-route").is_some() && terminal.is_some() {
        eprintln!("vyges-pad: --dont-route and --terminal cannot be used together");
        return ExitCode::from(2);
    }

    let term_state = terminal.as_ref().and_then(|t| {
        t.rsplit_once('/').map(|(i, p)| {
            let n = db.iterm_get_net(i, p);
            (t.as_str(), (!n.is_empty()).then_some(n))
        })
    });
    let plan = match assign(
        &terms,
        &net,
        master_box,
        term_state.as_ref().map(|(t, n)| (*t, n.as_deref())),
    ) {
        Ok(v) => v,
        Err(AssignRefused::WrongNet { terminal, net: other }) => {
            eprintln!("vyges-pad: {terminal} is connected to {other}, not to {net}");
            return ExitCode::from(2);
        }
    };

    if !opts.dry_run {
        let applied = (|| -> Result<(), String> {
            for term in &plan.connect {
                db.connect(&bump, term, &net)
                    .map_err(|e| format!("cannot connect {bump}/{term} to {net}: {e}"))?;
            }
            if let Some(t) = &plan.terminal {
                let (i, p) = t.rsplit_once('/').ok_or_else(|| format!("{t} is not inst/pin"))?;
                db.connect(i, p, &net).map_err(|e| format!("cannot connect {t} to {net}: {e}"))?;
            }
            if let Some((layer, rect)) = &plan.bterm {
                let orient = Orient::parse(&db.inst_get_orient(&bump)).unwrap_or(Orient::R0);
                let origin = (db.inst_get_origin_x(&bump), db.inst_get_origin_y(&bump));
                make_bterm(&mut db, &net, layer, pin_shape(*rect, orient, origin))?;
            }
            Ok(())
        })();
        if let Err(e) = applied {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    }

    let mut made: Vec<String> = plan.connect.iter().map(|t| format!("{bump}/{t} -> {net}")).collect();
    if let Some(t) = &plan.terminal {
        made.push(format!("{t} -> {net}"));
    }
    finish_report(&opts, &mut db, "bump-assignment", &made, &[])
}

/// **T1** — give each named pad terminal a block terminal on the die's top routing layer.
fn place_io_terminals(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let allow_lower = opts.get("allow-non-top-layer").is_some();

    let patterns: Vec<String> = match opts.need("pins") {
        Ok(v) => v.split_whitespace().map(str::to_string).collect(),
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };

    // The technology's topmost routing layer, which is where a terminal belongs.
    let top_level = db.tech_get_routing_layer_count();
    let top_layer = (0..db.tech_get_layer_count())
        .map(|i| db.layer_name_by_number(i as i64))
        .find(|n| db.layer_get_routing_level(n) == top_level)
        .unwrap_or_default();

    // Every `<instance>/<terminal>` in the design, so the patterns can be matched against it.
    let mut named = Vec::new();
    for inst in db.inst_names() {
        for term in db.master_get_m_terms(&db.inst_get_master(&inst)) {
            named.push(format!("{inst}/{term}"));
        }
    }
    let chosen: Vec<String> = matching(&named, &patterns).into_iter().cloned().collect();
    if chosen.is_empty() {
        eprintln!("vyges-pad: no terminals matched {}", patterns.join(" "));
        return ExitCode::from(2);
    }

    let mut made = Vec::new();
    for id in &chosen {
        let Some((inst, term)) = id.rsplit_once('/') else { continue };
        let net = db.iterm_get_net(inst, term);
        // ⚠️ Three quiet skips, all deliberate: an unconnected terminal has no net to bring out,
        // a floating instance has no settled position, and a cell that is not a PAD is not on the
        // boundary at all.
        if net.is_empty()
            || !matches!(db.inst_get_placement_status(inst).as_str(), "FIRM" | "LOCKED" | "COVER")
            || !db.master_is_pad(&db.inst_get_master(inst))
        {
            continue;
        }

        let master = db.inst_get_master(inst);
        let pins: Vec<(String, i32, Rect)> = db
            .mterm_pin_boxes(&master, term)
            .unwrap_or_default()
            .into_iter()
            .map(|(layer, x0, y0, x1, y1)| {
                let name = db.layer_name_by_number(layer);
                let level = db.layer_get_routing_level(&name);
                (name, level, (x0, y0, x1, y1))
            })
            .collect();
        let Some((layer, rect)) = bond_shape(&pins) else {
            eprintln!("vyges-pad: {inst}/{term} has no shape to make a terminal from");
            return ExitCode::from(2);
        };
        if !allow_lower && layer != top_layer {
            eprintln!(
                "vyges-pad: {inst}/{term} has no shape on {top_layer}, only on {layer}"
            );
            return ExitCode::from(2);
        }

        let orient = Orient::parse(&db.inst_get_orient(inst)).unwrap_or(Orient::R0);
        let origin = (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst));
        if !opts.dry_run {
            if let Err(e) = make_bterm(&mut db, &net, &layer, pin_shape(rect, orient, origin)) {
                eprintln!("vyges-pad: {e}");
                return ExitCode::from(2);
            }
        }
        made.push(format!("{id} -> {net}"));
    }
    finish_report(&opts, &mut db, "io-terminals", &made, &[])
}

/// **F2** — pack the gaps in one row with filler cells.
fn place_io_fill(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };

    let setup = (|| -> Result<(Track, Vec<Filler>), String> {
        let row_name = opts.need("row")?;
        let row = read_row(&db, row_name).ok_or_else(|| format!("no row `{row_name}`"))?;
        let edge = row.edge().ok_or_else(|| format!("{row_name} is not a recognized IO row"))?;
        let site = db.row_get_site(row_name);
        let track = Track {
            site_width: db.site_get_width(&site).min(db.site_get_height(&site)),
            row,
            edge,
        };

        let overlapping: Vec<String> = opts
            .get("permit-overlaps")
            .unwrap_or_default()
            .split_whitespace()
            .map(str::to_string)
            .collect();
        let mut fillers = Vec::new();
        for master in opts.need("masters")?.split_whitespace() {
            let (w, h) = master_size(&db, master)
                .map_err(|e| format!("cannot measure master `{master}`: {e}"))?;
            fillers.push(Filler {
                master: master.to_string(),
                // Along the row, after the row's own orientation.
                width: vyges_pad::pad_width(&track, (w, h)),
                overlapping: overlapping.iter().any(|m| m == master),
            });
        }
        Ok((track, fillers))
    })();

    let (track, fillers) = match setup {
        Ok(v) => v,
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };

    // What is already in this row: fixed, non-cover cells that overlap it.
    let occupied: Vec<(i32, i32)> = db
        .inst_names()
        .iter()
        .filter(|n| {
            matches!(db.inst_get_placement_status(n).as_str(), "FIRM" | "LOCKED" | "COVER")
                && !db.master_is_cover(&db.inst_get_master(n))
        })
        .filter_map(|n| db.inst_bbox(n).ok())
        .filter_map(|b| match b[..] {
            [x0, y0, x1, y1] => Some((x0, y0, x1, y1)),
            _ => None,
        })
        .filter(|&b| intersects(b, track.row.bbox))
        .map(|b| track.along(b))
        .collect();

    let planned = fill_row(
        &track.row.name,
        track.along(track.row.bbox),
        &occupied,
        track.start(),
        track.site_width,
        &|at| track.snap_to_site(at),
        &fillers,
    );
    let planned = match planned {
        Ok(v) => v,
        Err(Unfilled::Ragged { span }) => {
            eprintln!(
                "vyges-pad: filling {} from {} to {} would leave a gap",
                track.row.name, span.0, span.1
            );
            return ExitCode::from(2);
        }
        Err(Unfilled::Short { span }) => {
            eprintln!(
                "vyges-pad: cannot fill {} from {} to {} with the given cells",
                track.row.name, span.0, span.1
            );
            return ExitCode::from(2);
        }
    };

    let mut placed = Vec::new();
    for cell in &planned {
        let (w, h) = match master_size(&db, &cell.master) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("vyges-pad: {e}");
                return ExitCode::from(2);
            }
        };
        let index = track.snap_to_site(cell.at);
        let (x, y, orient) = place_in_row(index, &track.row, track.edge, w, h, Orient::R0);
        let p = Placement { name: cell.name.clone(), master: cell.master.clone(), x, y, orient };
        if !opts.dry_run {
            if let Err(e) = commit(&mut db, &p, true) {
                eprintln!("vyges-pad: {e}");
                return ExitCode::from(2);
            }
        }
        placed.push(p);
    }
    finish(&opts, &mut db, "io-fill", &placed, &[])
}

/// **D3** — a bond pad on top of every selected pad.
fn place_bondpad(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let dbu = db.dbu_per_micron();

    let setup = (|| -> Result<(String, String, Rect, (i32, i32), Orient, String), String> {
        let bond = opts.need("bond")?.to_string();
        let class = db.master_get_type(&bond).unwrap_or_default();
        if class.is_empty() {
            return Err(format!("no master named `{bond}` in this design"));
        }
        if !is_bond_master(&class) {
            return Err(format!("{bond} is `{class}`, not a COVER cell"));
        }

        // The bond layer: the highest routing layer the master puts a pin on.
        let mut pins = Vec::new();
        for term in db.master_get_m_terms(&bond) {
            for (layer, x0, y0, x1, y1) in db.mterm_pin_boxes(&bond, &term).unwrap_or_default() {
                let name = db.layer_name_by_number(layer);
                let level = db.layer_get_routing_level(&name);
                pins.push((name, level, (x0, y0, x1, y1)));
            }
        }
        let (layer, rect) =
            bond_shape(&pins).ok_or_else(|| format!("cannot find the top layer of {bond}"))?;

        let offset = match opts.get("offset") {
            None => (0, 0),
            Some(v) => {
                let n: Vec<f64> = v.split_whitespace().filter_map(|t| t.parse().ok()).collect();
                match n[..] {
                    [x, y] => ((x * dbu as f64).round() as i32, (y * dbu as f64).round() as i32),
                    _ => return Err("--offset must be specified as `x y`".into()),
                }
            }
        };
        let rotation = rotation(&opts, "rotation")?;
        let prefix = opts.get("prefix").unwrap_or(vyges_pad::bond::DEFAULT_PREFIX).to_string();
        Ok((bond, layer, rect, offset, rotation, prefix))
    })();

    let (bond, layer, bond_rect, offset, rotation, prefix) = match setup {
        Ok(v) => v,
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };

    let patterns: Vec<String> = match opts.need("insts") {
        Ok(v) => v.split_whitespace().map(str::to_string).collect(),
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };
    let all = db.inst_names();
    let chosen: Vec<String> = matching(&all, &patterns).into_iter().cloned().collect();
    if chosen.is_empty() {
        eprintln!("vyges-pad: no instances matched {}", patterns.join(" "));
        return ExitCode::from(2);
    }

    let mut placed = Vec::new();
    let mut wired: Vec<(String, String, String)> = Vec::new(); // (bond inst, term, net)
    for pad in &chosen {
        // ⚠️ Only a FIXED pad gets a bond pad. One still floating has no settled place to sit on.
        if !matches!(db.inst_get_placement_status(pad).as_str(), "FIRM" | "LOCKED" | "COVER") {
            continue;
        }
        let pad_orient = Orient::parse(&db.inst_get_orient(pad)).unwrap_or(Orient::R0);
        let origin = (db.inst_get_origin_x(pad), db.inst_get_origin_y(pad));
        let b = bond_place(pad, origin, pad_orient, &bond, offset, rotation, &prefix);

        if !opts.dry_run {
            let made = (|| -> Result<(), String> {
                db.create_inst(&b.master, &b.name).map_err(|e| format!("{}: {e}", b.name))?;
                db.inst_set_orient(&b.name, &format!("{:?}", b.orient))
                    .map_err(|e| format!("{}: {e}", b.name))?;
                // ⚠️ `set_origin`, not `set_location`: the reference sets the transform origin, and
                // the two differ for every orientation but R0.
                db.inst_set_origin(&b.name, b.origin.0, b.origin.1)
                    .map_err(|e| format!("{}: {e}", b.name))?;
                db.inst_set_placement_status(&b.name, "FIRM")
                    .map_err(|e| format!("{}: {e}", b.name))
            })();
            if let Err(e) = made {
                eprintln!("vyges-pad: cannot create {e}");
                return ExitCode::from(2);
            }
            for (term, net) in touching_nets(&db, pad, &b.name) {
                wired.push((b.name.clone(), term, net));
            }
        }
        placed.push(Placement {
            name: b.name,
            master: b.master,
            x: b.origin.0,
            y: b.origin.1,
            orient: b.orient,
        });
    }

    if !opts.dry_run {
        let joined = (|| -> Result<(), String> {
            for (inst, term, net) in &wired {
                db.connect(inst, term, net)
                    .map_err(|e| format!("cannot connect {inst}/{term} to {net}: {e}"))?;
                // The shape that makes this net reachable from outside the die, placed by the
                // bond pad's OWN transform -- orientation and origin, not its bounding box.
                let orient = Orient::parse(&db.inst_get_orient(inst)).unwrap_or(Orient::R0);
                let origin = (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst));
                let shape = pin_shape(bond_rect, orient, origin);
                make_bterm(&mut db, net, &layer, shape)?;
            }
            Ok(())
        })();
        if let Err(e) = joined {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    }
    finish(&opts, &mut db, "bondpad", &placed, &[])
}

/// The nets a bond pad picks up from the pad underneath it, by terminals that touch.
fn touching_nets(db: &Db, pad: &str, bond: &str) -> Vec<(String, String)> {
    let (Some(a), Some(b)) = (read_inst(db, pad), read_inst(db, bond)) else {
        return Vec::new();
    };
    touching_terms(&a, &b)
        .into_iter()
        .filter_map(|(i, j)| {
            // The net comes from the PAD's terminal; the bond pad is what joins it.
            a.terms[i].net.clone().map(|net| (b.terms[j].name.clone(), net))
        })
        .collect()
}

/// **D5** — give a net a block terminal shape, making one if the net has none.
///
/// ℹ️ The create-a-terminal path is not exercised by any reference case: every net that reaches
/// here already has one. It is implemented rather than left to fail, and said to be untested.
fn make_bterm(db: &mut Db, net: &str, layer: &str, shape: Rect) -> Result<(), String> {
    let mut bterm = db.net_get1st_b_term(net);
    if bterm.is_empty() {
        db.create_bterm(net, net).map_err(|e| format!("cannot make a terminal for {net}: {e}"))?;
        bterm = db.net_get1st_b_term(net);
    }
    let sig = db.net_get_sig_type(net);
    db.bterm_set_sig_type(&bterm, &sig).map_err(|e| format!("cannot type {bterm}: {e}"))?;
    let idx = db
        .create_bterm_pin(&bterm, layer, shape)
        .map_err(|e| format!("cannot add a pin to {bterm}: {e}"))?;
    db.bpin_set_placement_status(&bterm, idx, "FIRM")
        .map_err(|e| format!("cannot fix {bterm} pin {idx}: {e}"))?;
    // ⚠️ The reference makes the net special here, inside this step, not at the call site. Both
    // callers depend on it happening.
    make_special(db, net)
}

/// **A4** — wire the ring by abutment.
fn connect_ring(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };

    let insts = pad_insts(&db);
    let plan = match connect_by_abutment(&insts, &|n| db.net_get_i_term_count(n) + db.net_get_b_term_count(n)) {
        Ok(p) => p,
        Err(c) => {
            eprintln!(
                "vyges-pad: {}/{} ({}) and {}/{} ({}) are touching but on different nets",
                insts[c.a.0].name,
                insts[c.a.0].terms[c.a.1].name,
                c.net_a,
                insts[c.b.0].name,
                insts[c.b.0].terms[c.b.1].name,
                c.net_b
            );
            return ExitCode::from(2);
        }
    };

    if !opts.dry_run {
        let apply = (|| -> Result<(), String> {
            for net in &plan.destroy {
                db.destroy_net(net).map_err(|e| format!("cannot remove {net}: {e}"))?;
            }
            for net in &plan.create {
                db.create_net(net).map_err(|e| format!("cannot create {net}: {e}"))?;
            }
            for ((i, t), net) in &plan.connect {
                let (inst, term) = (&insts[*i].name, &insts[*i].terms[*t].name);
                db.connect(inst, term, net)
                    .map_err(|e| format!("cannot connect {inst}/{term} to {net}: {e}"))?;
            }
            for net in &plan.special {
                make_special(&mut db, net)?;
            }
            Ok(())
        })();
        if let Err(e) = apply {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    }

    let touched: Vec<String> = plan
        .connect
        .iter()
        .map(|((i, t), net)| format!("{}/{} -> {net}", insts[*i].name, insts[*i].terms[*t].name))
        .collect();
    finish_report(&opts, &mut db, "abutment", &touched, &plan.destroy)
}

/// Marks an error as "this engine does not implement that", which exits 3 rather than 2.
///
/// A caller can then tell a feature that is missing from a design that is wrong — a distinction
/// worth an exit code, because the two want opposite responses.
const UNSUPPORTED: &str = "not implemented: ";

fn unsupported_or_error(message: &str) -> ExitCode {
    eprintln!("vyges-pad: {message}");
    ExitCode::from(if message.starts_with(UNSUPPORTED) { 3 } else { 2 })
}

/// The bumps each pad shares a **non-supply** net with, in terminal id order.
///
/// ⚠️ Supply nets are skipped: a pad and a bump both on `VDD` meet through the power grid, and
/// aligning to that would drag the whole row onto the power bumps.
fn bump_pads(db: &Db, track: &Track, pads: &[Pad]) -> Vec<BumpPad> {
    pads.iter()
        .map(|p| {
            let mut bumps: Vec<Bump> = Vec::new();
            for iterm in db.inst_get_i_terms(&p.name) {
                let Some((_, pin)) = iterm.rsplit_once('/') else { continue };
                let net = db.iterm_get_net(&p.name, pin);
                if net.is_empty()
                    || matches!(db.net_get_sig_type(&net).as_str(), "POWER" | "GROUND")
                {
                    continue;
                }
                for other in db.net_get_i_terms(&net) {
                    let Some((owner, opin)) = other.rsplit_once('/') else { continue };
                    if !db.master_is_cover(&db.inst_get_master(owner)) {
                        continue;
                    }
                    let bb = (
                        db.iterm_get_b_box_x_min(owner, opin),
                        db.iterm_get_b_box_y_min(owner, opin),
                        db.iterm_get_b_box_x_max(owner, opin),
                        db.iterm_get_b_box_y_max(owner, opin),
                    );
                    bumps.push(Bump {
                        terminal: other.clone(),
                        centre: ((bb.0 + bb.2) / 2, (bb.1 + bb.3) / 2),
                        id: db.iterm_id(owner, opin).unwrap_or(0) as u64,
                    });
                }
            }
            bumps.sort_by_key(|b| b.id);
            bumps.dedup_by_key(|b| b.terminal.clone());
            BumpPad {
                name: p.name.clone(),
                id: db.inst_id(&p.name).unwrap_or(0) as u64,
                width: vyges_pad::pad_width(track, p.size),
                bumps,
            }
        })
        .collect()
}

/// **SP1-SP8** — place a row by spreading pads out from where their bumps want them.
#[allow(clippy::too_many_arguments)]
fn place_force_directed(
    track: &Track,
    pads: &[Pad],
    aligned: &[BumpPad],
    conflict: &mut dyn FnMut(&str, Rect, Orient) -> Option<vyges_pad::Refusal>,
    conflict_probe: &dyn Fn(&str, Rect) -> Option<Rect>,
    settled: &mut dyn FnMut(&Placement),
) -> Result<Vec<Placement>, Refused> {
    use vyges_pad::spread::{forces, ideal_position, spread_pass, Anchor, DAMPER, MAX_ITERATIONS};
    let horizontal = track.horizontal();
    let row = (track.start(), track.end());
    let along = |c: (i32, i32)| if horizontal { c.0 } else { c.1 };

    let snap = |p: i32| track.index_to_pos(track.snap_to_site(p));
    // What **this** pad, centred here, would run into — as an extent along the row.
    //
    // ⚠️ Per pad, not one probe for the row. Pads differ in width and each is excused its own
    // metal, so asking on another pad's behalf gives an obstruction that is the wrong size and in
    // the wrong place, and the jump logic then aims at a gap that is not there.
    let blocked = |i: usize, centre: i32| -> Option<(i32, i32)> {
        let pad = &pads[i];
        let (w, h) = pad.size;
        let (dx, dy) = oriented_size(w, h, track.row.orient);
        let (half_x, half_y) = (dx / 2, dy / 2);
        let bbox = if horizontal {
            (centre - half_x, track.row.bbox.1, centre + half_x, track.row.bbox.1 + dy)
        } else {
            (track.row.bbox.0, centre - half_y, track.row.bbox.0 + dx, centre + half_y)
        };
        conflict_probe(&pad.name, bbox).map(|r| if horizontal { (r.0, r.2) } else { (r.1, r.3) })
    };

    // ── Ideal positions, and a crude start for pads that serve no bump ───────────────────────
    let mut targets: Vec<i32> = Vec::with_capacity(pads.len());
    for (i, a) in aligned.iter().enumerate() {
        let centres: Vec<i32> = a.bumps.iter().map(|b| along(b.centre)).collect();
        // ⚠️ The row bounds here are inset by half the pad, as in the reference: everything in
        // this stage is a centre coordinate.
        let inset = (row.0 + a.width / 2, row.1 - a.width / 2);
        // ⚠️ The ideal is **legalised as it is computed**, not later. A mean of two bump centres
        // can land squarely on an obstruction, and every stage after this takes it as the thing to
        // pull towards. Legalising only at the end asks the spread to undo a target it was told to
        // aim at.
        let t = ideal_position(&centres)
            .map(|m| {
                let half = vyges_pad::pad_width(track, pads[i].size) / 2;
                vyges_pad::spread::nearest_legal(m, blocked(i, m), half, row)
            })
            .unwrap_or_else(|| {
                vyges_pad::spread::unconstrained_start(i, pads.len(), &targets, inset)
            });
        targets.push(t);
    }


    // ── Restore order along the row ──────────────────────────────────────────────────────────
    let mut ordered = targets.clone();
    let mut weights = vec![1.0f32; ordered.len()];
    let legalise = |i: usize, pos: i32| {
        let half = vyges_pad::pad_width(track, pads[i].size) / 2;
        vyges_pad::spread::nearest_legal(pos, blocked(i, pos), half, row)
    };
    for _ in 0..ordered.len() {
        if !vyges_pad::spread::pool_round(&mut ordered, &mut weights, &legalise) {
            break;
        }
    }

    // ── Spread until nothing overlaps ────────────────────────────────────────────────────────
    let site = track.site_width.max(1);
    // ⚠️ Anchors start **snapped to a site**. The regression works in continuous coordinates and
    // the spread moves in whole sites, so an unsnapped start leaves every position permanently
    // offset by a fraction of a site — small enough to look like rounding, large enough to put a
    // pad on the wrong side of an obstruction edge.
    let mut anchors: Vec<Anchor> = ordered
        .iter()
        .zip(aligned)
        .map(|(&pos, a)| Anchor::at(snap(pos - a.width / 2), a.width))
        .collect();
    // ⚠️ A trace in the reference's own shape, so the two can be diffed line for line rather than
    // reasoned about. This is the facility that closed the RDL attempt-order question.
    let watched = std::env::var("VYGES_PAD_TRACE").ok();
    let mut iterations = 0;
    for k in 0..MAX_ITERATIONS {
        let (spring, repel) = forces(k);
        let names = &pads;
        let w = watched.clone();
        let mut watch = |i: usize, from: i32, to: i32, lo: i32, hi: i32| {
            if let Some(name) = &w {
                if names[i].name == *name && from != to {
                    eprintln!("{k} / {name}: {from} -> {to} ({lo}, {from}, {hi})");
                }
            }
        };
        let more = spread_pass(
            &mut anchors, &ordered, row, spring, repel, DAMPER, site, &snap, &blocked, &mut watch,
        );
        iterations = k + 1;
        if !more {
            break;
        }
    }
    if let Some(name) = &watched {
        if let Some(i) = pads.iter().position(|p| p.name == *name) {
            eprintln!("final / {name}: centre {} after {iterations} iterations", anchors[i].centre);
            eprintln!("target / {name}: pooled {} (ideal {})", ordered[i], targets[i]);
        }
    }

    // ── Commit ───────────────────────────────────────────────────────────────────────────────
    let mut out = Vec::new();
    for (i, a) in anchors.iter().enumerate() {
        // ⚠️ Placed with shifting DISALLOWED: the spreading stage is what resolves overlap, and a
        // placer that also shifts here would hide a spread that had not converged.
        let p = place_one(
            track,
            track.snap_to_site(a.min),
            &pads[i],
            Orient::R0,
            false,
            false,
            conflict,
        )?;
        settled(&p);
        out.push(p);
    }
    Ok(out)
}

/// **BA1-BA5** — place a row so its pads sit under the bumps they serve.
fn place_bump_aligned(
    track: &Track,
    pads: &[Pad],
    aligned: &[BumpPad],
    conflict: &mut dyn FnMut(&str, Rect, Orient) -> Option<vyges_pad::Refusal>,
    settled: &mut dyn FnMut(&Placement),
) -> Result<Vec<Placement>, Refused> {
    let horizontal = track.horizontal();
    let bbox = track.row.bbox;
    let row_centre = ((bbox.0 + bbox.2) / 2, (bbox.1 + bbox.3) / 2);
    let total: i32 = aligned.iter().map(|p| p.width).sum();
    let mut budget = track.width() - total;
    let mut offset = track.start();
    let mut out = Vec::new();
    let mut k = 0usize;

    while k < pads.len() {
        let group = alignment_group(aligned, k, offset, row_centre, horizontal);
        // A pad with no bump simply takes the next place going.
        let wanted: Vec<(usize, i32)> = if group.is_empty() {
            vec![(k, 0)]
        } else {
            group_positions(aligned, &group, horizontal)
        };
        let step = wanted.len();
        for (i, want) in wanted {
            let (at, left) = travel(offset, want, budget);
            budget = left;
            let p = place_one(
                track,
                track.snap_to_site(at),
                &pads[i],
                Orient::R0,
                false,
                true,
                conflict,
            )?;
            settled(&p);
            // ⚠️ The cursor follows where the pad actually LANDED, unlike the uniform placer's
            // ideal cursor: an aligned row is a chain, and a pad that slid moves its successors.
            let (_, end) = track.along((p.x, p.y, p.x + 1, p.y + 1));
            offset = end.max(at) + vyges_pad::pad_width(track, pads[i].size);
            out.push(p);
        }
        k += step.max(1);
    }
    Ok(out)
}

/// **P5** — spread a list of pads along one side of the ring.
fn place_pads(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };

    let prepared = (|| -> Result<(Track, Vec<Pad>, Option<i32>), String> {
        let row_name = opts.need("row")?;
        let row = read_row(&db, row_name).ok_or_else(|| format!("no row `{row_name}`"))?;
        let edge = row.edge().ok_or_else(|| format!("{row_name} is not a recognized IO row"))?;
        let site = db.row_get_site(row_name);
        let track = Track {
            site_width: db.site_get_width(&site).min(db.site_get_height(&site)),
            row,
            edge,
        };

        let names: Vec<String> =
            opts.need("insts")?.split_whitespace().map(str::to_string).collect();
        if names.is_empty() {
            return Err("place-pads requires a list of instances".into());
        }
        let mut pads = Vec::new();
        for name in names {
            let master = db.inst_get_master(&name);
            if master.is_empty() {
                return Err(format!("no instance named `{name}` in this design"));
            }
            let size = master_size(&db, &master)
                .map_err(|e| format!("cannot measure master `{master}`: {e}"))?;
            pads.push(Pad { name, master, size });
        }

        // The strategy. ⚠️ `bump_aligned` is the DEFAULT once any pad connects to a bump, and
        // degrades to uniform when none do — which is why a case that asks for it by name can
        // still be a uniform case. Anything that genuinely needs a different placer is refused
        // rather than quietly placed by this one.
        let mode = opts.get("mode").unwrap_or("default");
        let connected = pads.iter().any(|p| connects_to_a_bump(&db, &p.name));
        let max_spacing = match (mode, connected) {
            ("uniform", _) | ("default", false) => None,
            ("linear", _) => Some(0),
            ("bump_aligned", false) => {
                eprintln!("vyges-pad: no pad connects to a bump, placing uniformly instead");
                None
            }
            ("default" | "bump_aligned", true) => Some(i32::MIN), // marker: bump-aligned
            // The force-directed placer. ⚠️ Named `placer`, and not an annealer: no RNG.
            ("placer", _) => Some(i32::MIN + 1),
            (other, _) => return Err(format!("`{other}` is not a placement mode")),
        };

        if let Err((total, room)) = fits(&track, &pads) {
            return Err(format!(
                "the pads total {total} units and {} has room for {room}",
                track.row.name
            ));
        }
        Ok((track, pads, max_spacing))
    })();

    let (track, pads, max_spacing) = match prepared {
        Ok(v) => v,
        Err(e) => return unsupported_or_error(&e),
    };

    // ⚠️ Everything already in the way, once -- **and each pad joins the list as it lands**. A pad
    // that had to slide is an obstruction to the pads after it, so leaving this out places the
    // first shifted pad correctly and then walks the rest straight through it.
    let mut start = blockers(&db, "");
    start.retain(|b| intersects(b.bbox, track.row.bbox) || !b.by_box);
    let fixed = std::cell::RefCell::new(start);
    let stops = blockages(&db);
    let shapes_at = |name: &str, bbox: (i32, i32, i32, i32), orient: Orient| {
        let master = db.inst_get_master(name);
        let shapes = cell_shapes(&db, &master, orient, (bbox.0, bbox.1));
        let outline = cell_outline(&db, &shapes);
        (shapes, outline)
    };
    let mut conflict = |name: &str, bbox: (i32, i32, i32, i32), orient: Orient| {
        let (shapes, outline) = shapes_at(name, bbox, orient);
        refuse(name, bbox, &outline, &shapes, &fixed.borrow(), &stops, &|l| {
            db.layer_get_spacing(l)
        })
    };
    let mut settle = |p: &Placement| {
        let (dx, dy) = oriented_size_of(&db, &p.master, p.orient);
        let bbox = (p.x, p.y, p.x + dx, p.y + dy);
        let (_, outline) = shapes_at(&p.name, bbox, p.orient);
        // A pad is not a cover cell: it blocks by box and outline only. See `blockers`.
        fixed.borrow_mut().push(Blocker {
            name: p.name.clone(),
            bbox,
            outline,
            by_box: true,
            shapes: Vec::new(),
        });
    };
    // ⚠️ `i32::MIN` is the marker set above for bump-aligned mode, not a spacing.
    let result = if max_spacing == Some(i32::MIN + 1) {
        let aligned = bump_pads(&db, &track, &pads);
        {
            // ⚠️ The row's own orientation, not `R0`. A pad in a side row is turned, and asking
            // where its metal would be if it were NOT turned puts every shape in the wrong place —
            // so the probe reports clear ground, the spread walks onto an obstruction, and the
            // final placement refuses a position the placer itself chose.
            let probe_orient = track.row.orient;
            let probe = |name: &str, bbox: (i32, i32, i32, i32)| {
                let (shapes, outline) = shapes_at(name, bbox, probe_orient);
                refuse(name, bbox, &outline, &shapes, &fixed.borrow(), &stops, &|l| {
                    db.layer_get_spacing(l)
                })
                // ⚠️ The blocker's own rectangle. See `Refusal::blocker`: the intersection would
                // bound every jump by the pad's own width.
                .map(|r| r.blocker)
            };
            place_force_directed(&track, &pads, &aligned, &mut conflict, &probe, &mut settle)
        }
    } else if max_spacing == Some(i32::MIN) {
        let aligned = bump_pads(&db, &track, &pads);
        place_bump_aligned(&track, &pads, &aligned, &mut conflict, &mut settle)
    } else {
        place_uniform(&track, &pads, max_spacing, &mut conflict, &mut settle)
    };

    let placed = match result {
        Ok(v) => v,
        Err(Refused::OutOfRow { name, at }) => {
            eprintln!("vyges-pad: {name} at {at:?} does not fit inside {}", track.row.name);
            return ExitCode::from(2);
        }
        Err(Refused::Blocked { name, at, why }) => {
            eprintln!(
                "vyges-pad: cannot place {name} at {at:?}: {:?} overlapping {:?}",
                why.reason, why.overlap
            );
            return ExitCode::from(2);
        }
    };

    if !opts.dry_run {
        for p in &placed {
            if let Err(e) = commit(&mut db, p, false) {
                eprintln!("vyges-pad: {e}");
                return ExitCode::from(2);
            }
        }
    }
    finish(&opts, &mut db, "pads", &placed, &[])
}

/// Does this pad share a non-supply net with a bump?
///
/// ⚠️ Supply nets do not count: a pad and a bump on `VDD` are expected to meet through the power
/// grid, and treating that as an alignment request would align the whole ring to the power bumps.
fn connects_to_a_bump(db: &Db, inst: &str) -> bool {
    for iterm in db.inst_get_i_terms(inst) {
        // ⚠️ An iterm reads `<instance>/<terminal>`, and the accessors below want the TERMINAL on
        // its own. Passing the whole string as the terminal matches nothing, returns no net, and
        // reports "no pad connects to a bump" for every design — a check that can only fail one
        // way, whose failure looks exactly like a correct negative answer.
        let Some((_, pin)) = iterm.rsplit_once('/') else { continue };
        let net = db.iterm_get_net(inst, pin);
        if net.is_empty() || matches!(db.net_get_sig_type(&net).as_str(), "POWER" | "GROUND") {
            continue;
        }
        for term in db.net_get_i_terms(&net) {
            // ⚠️ Split on the LAST slash: an iterm reads `<instance>/<pin>` and an instance name
            // can itself be hierarchical.
            let Some((owner, _)) = term.rsplit_once('/') else { continue };
            if db.master_is_cover(&db.inst_get_master(owner)) {
                return true;
            }
        }
    }
    false
}

/// **U1** — a grid of bumps over the die.
fn make_io_bump_array(args: &[String]) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let dbu = db.dbu_per_micron();

    let built = (|| -> Result<Vec<Placement>, String> {
        let master = opts.need("bump")?.to_string();
        // The reference refuses anything that is not a bump, rather than placing it on the cover
        // layer and leaving the surprise for whoever reads the result.
        let class = db.master_get_type(&master).unwrap_or_default();
        if class.is_empty() {
            return Err(format!("no master named `{master}` in this design"));
        }
        if !is_bump_master(&class) {
            return Err(format!("{master} is `{class}`, not a COVER BUMP"));
        }

        // ⚠️ The two look alike and are not: an origin must be a PAIR, while a pitch may be a
        // single value meaning the same spacing on both axes. The reference rejects `-origin 200`
        // and accepts `-pitch 200`, and has a distinct diagnostic for each.
        let nums = |key: &str| -> Result<Vec<f64>, String> {
            Ok(opts
                .need(key)?
                .split_whitespace()
                .filter_map(|t| t.parse::<f64>().ok())
                .collect())
        };
        let um = |v: f64| (v * dbu as f64).round() as i32;
        let origin = match nums("origin")?[..] {
            [x, y] => (x, y),
            _ => return Err("--origin must be specified as `x y`".into()),
        };
        let pitch = match nums("pitch")?[..] {
            [d] => (d, d),                   // one pitch means the same on both axes
            [dx, dy] => (dx, dy),
            _ => return Err("--pitch must be specified as `deltax deltay` or `delta`".into()),
        };
        let count = |key: &str| -> Result<i32, String> {
            opts.need(key)?.parse().map_err(|_| format!("--{key} wants a whole number"))
        };

        Ok(bumps(&Array {
            master,
            prefix: opts.get("prefix").unwrap_or(DEFAULT_PREFIX).to_string(),
            origin: (um(origin.0), um(origin.1)),
            rows: count("rows")?,
            columns: count("columns")?,
            pitch: (um(pitch.0), um(pitch.1)),
        }))
    })();

    let placed = match built {
        Ok(v) => v,
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };
    if !opts.dry_run {
        for p in &placed {
            if let Err(e) = commit(&mut db, p, true) {
                eprintln!("vyges-pad: {e}");
                return ExitCode::from(2);
            }
        }
    }
    finish(&opts, &mut db, "bump-array", &placed, &[])
}

/// **U3** — take bumps back out, one instance or a whole master's worth.
///
/// ⚠️ Guarded by the same master-type check as creation, and for the same reason: `remove_io_bump`
/// names an instance, and without the guard a mistyped name that happens to hit a real cell would
/// delete a pad or a macro instead of a bump.
fn remove_io_bump(args: &[String], whole_array: bool) -> ExitCode {
    let (opts, mut db) = match open(args) {
        Ok(v) => v,
        Err(code) => return code,
    };

    let chosen = (|| -> Result<Vec<String>, String> {
        if whole_array {
            let master = opts.need("bump")?;
            let class = db.master_get_type(master).unwrap_or_default();
            if class.is_empty() {
                return Err(format!("no master named `{master}` in this design"));
            }
            if !is_bump_master(&class) {
                return Err(format!("{master} is `{class}`, not a COVER BUMP"));
            }
            Ok(db.inst_names().into_iter().filter(|i| db.inst_master(i) == master).collect())
        } else {
            let inst = opts.need("inst")?.to_string();
            let master = db.inst_master(&inst);
            if master.is_empty() {
                return Err(format!("no instance named `{inst}` in this design"));
            }
            let class = db.master_get_type(&master).unwrap_or_default();
            if !is_bump_master(&class) {
                return Err(format!("{inst} is a `{class}` cell, not a bump"));
            }
            Ok(vec![inst])
        }
    })();

    let chosen = match chosen {
        Ok(v) => v,
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };
    if !opts.dry_run {
        for inst in &chosen {
            if let Err(e) = db.destroy_inst(inst) {
                eprintln!("vyges-pad: cannot remove {inst}: {e}");
                return ExitCode::from(2);
            }
        }
    }
    // Reported as `skipped` -- the field for instances named but not placed by this run.
    finish(&opts, &mut db, "bump-removal", &[], &chosen)
}

fn make_io_sites(args: &[String]) -> ExitCode {
    let opts = match parse_opts(args) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("vyges-pad: {e}\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };

    let mut db = match Db::open(&opts.odb) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("vyges-pad: cannot read {}: {e}", opts.odb);
            return ExitCode::from(2);
        }
    };
    let dbu = db.dbu_per_micron();
    if dbu <= 0 {
        eprintln!("vyges-pad: no DBU scale");
        return ExitCode::from(2);
    }

    let built = (|| -> Result<Vec<Row>, String> {
        let hor_name = opts.need("horizontal-site")?.to_string();
        let ver_name = opts.need("vertical-site")?.to_string();
        let cor_name = opts.need("corner-site")?.to_string();
        let offset_um: f64 = opts
            .need("offset")?
            .parse()
            .map_err(|_| "--offset wants a number of microns".to_string())?;
        let offset = (offset_um * dbu as f64).round() as i32;

        let horizontal = site(&db, &hor_name)?;
        let vertical = site(&db, &ver_name)?;
        let corner = site(&db, &cor_name)?;

        let ring_index: i32 = match opts.get("ring-index") {
            None => -1,
            Some(v) => v.parse().map_err(|_| "--ring-index wants a whole number".to_string())?,
        };

        let die = (
            db.block_get_die_area_x_min(),
            db.block_get_die_area_y_min(),
            db.block_get_die_area_x_max(),
            db.block_get_die_area_y_max(),
        );

        Ok(make_rows(
            die,
            &horizontal,
            &vertical,
            &corner,
            Offsets { west: offset, north: offset, east: offset, south: offset },
            Rotations {
                horizontal: rotation(&opts, "rotation-horizontal")?,
                vertical: rotation(&opts, "rotation-vertical")?,
                corner: rotation(&opts, "rotation-corner")?,
            },
            ring_index,
            // The reference compares the two SITE OBJECTS. A name resolves to one site, so
            // comparing the names given is the same test at this level — and it is the test, not
            // a comparison of their sizes, which are equal in the case that distinguishes them.
            hor_name == ver_name,
        ))
    })();

    let rows = match built {
        Ok(r) => r,
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
    };

    // A ring with no sites on an edge is not a ring; say so rather than writing a broken one.
    let empty: Vec<&Row> = rows.iter().filter(|r| r.sites <= 0).collect();

    if !opts.dry_run && empty.is_empty() {
        for r in &rows {
            if let Err(e) = db.create_row(
                &r.name,
                &r.site,
                r.x,
                r.y,
                &format!("{:?}", r.orient),
                def_dir(r.dir),
                r.sites,
                r.pitch,
            ) {
                eprintln!("vyges-pad: cannot create row {}: {e}", r.name);
                return ExitCode::from(2);
            }
        }
        let out_odb = opts.get("out-odb").unwrap_or(&opts.odb);
        if let Err(e) = db.write(out_odb) {
            eprintln!("vyges-pad: cannot write {out_odb}: {e}");
            return ExitCode::from(2);
        }
        if let Some(p) = opts.get("out-def") {
            if let Err(e) = db.write_def(p) {
                eprintln!("vyges-pad: cannot write {p}: {e}");
                return ExitCode::from(2);
            }
        }
    }

    emit_events(&rows, &empty);
    let report = report_json(&rows, &empty, dbu);
    match opts.get("o") {
        Some(path) => {
            if let Err(e) = std::fs::write(path, format!("{report}\n")) {
                eprintln!("vyges-pad: cannot write {path}: {e}");
                return ExitCode::from(2);
            }
        }
        None => println!("{report}"),
    }

    if empty.is_empty() {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

fn report_json(rows: &[Row], empty: &[&Row], dbu: i32) -> String {
    let list = rows
        .iter()
        .map(|r| {
            format!(
                "    {{\"name\": \"{}\", \"site\": \"{}\", \"x\": {}, \"y\": {}, \
                 \"orient\": \"{:?}\", \"direction\": \"{}\", \"sites\": {}, \"pitch\": {}}}",
                r.name,
                r.site,
                r.x,
                r.y,
                r.orient,
                def_dir(r.dir),
                r.sites,
                r.pitch
            )
        })
        .collect::<Vec<_>>()
        .join(",\n");
    format!(
        "{{\n  \"tool\": \"vyges-pad\",\n  \"status\": \"{}\",\n  \"dbu_per_micron\": {dbu},\n  \
         \"rows_total\": {},\n  \"rows\": [\n{list}\n  ]\n}}",
        if empty.is_empty() { "ok" } else { "refused" },
        rows.len(),
    )
}

fn emit_events(rows: &[Row], empty: &[&Row]) {
    use vyges_events::{Event, Severity};
    for r in empty {
        // An edge with no room for a single site means the offsets have eaten the die. Reporting
        // it is the difference between "no ring" and "a ring with a missing side".
        vyges_events::emit(
            &Event::new(
                "vyges-pad",
                Severity::Error,
                format!("row {} has no room for a single site", r.name),
            )
            .with_code("PAD-ROW-EMPTY")
            .with_objects(vec![format!("row:{}", r.name)]),
        );
    }
    let sites: i32 = rows.iter().map(|r| r.sites).sum();
    vyges_events::emit(
        &Event::new(
            "vyges-pad",
            if empty.is_empty() { Severity::Info } else { Severity::Error },
            format!("IO ring: {} row(s), {sites} site(s)", rows.len()),
        )
        .with_code("PAD-RING"),
    );
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("--describe") => {
            println!("{DESCRIBE}");
            ExitCode::SUCCESS
        }
        Some("--help") | Some("-h") | None => {
            println!("{USAGE}");
            ExitCode::SUCCESS
        }
        Some("make-io-sites") => make_io_sites(&args[1..]),
        Some("place-corners") => place_corners(&args[1..]),
        Some("place-pad") => place_pad(&args[1..]),
        Some("place-pads") => place_pads(&args[1..]),
        Some("connect-by-abutment") => connect_ring(&args[1..]),
        Some("place-bondpad") => place_bondpad(&args[1..]),
        Some("place-io-fill") => place_io_fill(&args[1..]),
        Some("place-io-terminals") => place_io_terminals(&args[1..]),
        Some("assign-io-bump") => assign_io_bump(&args[1..]),
        Some("rdl-route") => rdl_route(&args[1..]),
        Some("make-io-bump-array") => make_io_bump_array(&args[1..]),
        Some("remove-io-bump") => remove_io_bump(&args[1..], false),
        Some("remove-io-bump-array") => remove_io_bump(&args[1..], true),
        Some(other) => {
            eprintln!("vyges-pad: unknown command `{other}`\n\n{USAGE}");
            ExitCode::from(2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_three_sites_and_the_offset_are_all_required() {
        assert!(parse_opts(&[]).is_err(), "a .odb is required");
        let o = parse_opts(&["d.odb".to_string()]).unwrap();
        for k in ["horizontal-site", "vertical-site", "corner-site", "offset"] {
            assert!(o.need(k).is_err(), "{k} should be required");
        }
        let dangling = ["d.odb", "--offset"].map(String::from);
        assert!(parse_opts(&dangling).unwrap_err().contains("--offset"));
        assert!(parse_opts(&["-x".to_string()]).unwrap_err().contains("unknown"));
    }

    #[test]
    fn a_rotation_defaults_to_none_and_a_bad_one_is_refused() {
        let o = parse_opts(&["d.odb".to_string()]).unwrap();
        assert_eq!(rotation(&o, "rotation-corner").unwrap(), Orient::R0);
        let set = parse_opts(&["d.odb", "--rotation-vertical", "MXR90"].map(String::from)).unwrap();
        assert_eq!(rotation(&set, "rotation-vertical").unwrap(), Orient::MXR90);
        let bad = parse_opts(&["d.odb", "--rotation-vertical", "sideways"].map(String::from)).unwrap();
        assert!(rotation(&bad, "rotation-vertical").is_err());
    }

    #[test]
    fn the_report_states_refused_when_an_edge_holds_no_sites() {
        // An offset that eats the die must not read as a successful empty ring.
        let row = |sites| Row {
            name: "IO_NORTH".into(),
            site: "S".into(),
            x: 0,
            y: 0,
            orient: Orient::R0,
            dir: RowDir::Horizontal,
            sites,
            pitch: 10,
        };
        let ok = [row(5)];
        assert!(report_json(&ok, &[], 1000).contains("\"status\": \"ok\""));
        let bad = [row(0)];
        let empty: Vec<&Row> = bad.iter().collect();
        assert!(report_json(&bad, &empty, 1000).contains("\"status\": \"refused\""));
    }

    #[test]
    fn the_descriptor_is_valid_json_and_states_what_is_out_of_scope() {
        let d: serde_json::Value = serde_json::from_str(DESCRIBE).expect("valid JSON");
        assert_eq!(d["name"], "pad");
        let limits = d["provenance_limitations"].as_array().expect("an array");
        assert!(
            limits.iter().any(|l| l.as_str().unwrap_or("").contains("ROUTER")),
            "the descriptor must say the RDL router is out of scope, not merely absent"
        );
        assert!(
            limits.iter().any(|l| l.as_str().unwrap_or("").contains("MEASURED")),
            "the descriptor must state what was measured"
        );
    }
}
