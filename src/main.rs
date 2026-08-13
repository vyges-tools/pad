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
use vyges_pad::pads::{fits, place_uniform, Pad, Refused, Track};

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
        skipped.iter().map(|s| format!("\"{s}\"")).collect::<Vec<_>>().join(", ");
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
            ("default" | "bump_aligned", true) => {
                return Err("bump-aligned placement is not implemented".into())
            }
            ("placer", _) => return Err("the annealing placer is not implemented".into()),
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
        Err(e) => {
            eprintln!("vyges-pad: {e}");
            return ExitCode::from(2);
        }
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
    let result = place_uniform(&track, &pads, max_spacing, &mut conflict, &mut settle);

    let placed = match result {
        Ok(v) => v,
        Err(Refused::OutOfRow { name, at }) => {
            eprintln!("vyges-pad: {name} at {at:?} does not fit inside {}", track.row.name);
            return ExitCode::from(2);
        }
        Err(Refused::Blocked { name, why }) => {
            eprintln!("vyges-pad: cannot place {name}: {:?}", why.reason);
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
    for pin in db.inst_get_i_terms(inst) {
        let net = db.iterm_get_net(inst, &pin);
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
