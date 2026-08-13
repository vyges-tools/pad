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
use vyges_opendb::Db;
use vyges_pad::{make_rows, Offsets, Orient, Row, RowDir, Rotations, Site};

const USAGE: &str = "\
vyges loom pad — IO pad and bump placement: the ring around the die, and what sits in it

USAGE:
  vyges loom pad make-io-sites <design.odb> --horizontal-site S --vertical-site S
                               --corner-site S --offset D [options]
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
      "SCOPE: this build implements the IO RING only -- `make-io-sites`. Pad, corner, filler, bond-pad and terminal placement, the bump array, and connection by abutment are not implemented. The ring is the foundation the rest place into, which is why it is first.",
      "The upstream module also contains a redistribution-layer ROUTER. That is a routing engine and is deliberately out of scope for this one; it is not merely unimplemented, it belongs elsewhere.",
      "The ring is the die area inset by the offset, corners sized from the corner site, and four edges truncated to WHOLE sites -- a remainder that does not fill a site is given up rather than rounded out.",
      "A corner's WIDTH is the larger of the corner site's width and the horizontal row's depth, so the row abutting it can be what sets the corner size.",
      "The left and right rows are laid on their side when the horizontal and vertical sites are THE SAME SITE, and upright when they differ. The reference compares the site objects; this command compares the names it was given, which is the same thing for a name that resolves to one site.",
      "MEASURED: the ring reproduces the reference row output exactly -- name, site, origin, orientation, direction, site count and pitch -- on all three of its ring cases, including the rotated one and the one giving the two directions different sites.",
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
            a if a.starts_with("--") || a == "-o" => {
                i += 1;
                let v = args.get(i).cloned().ok_or_else(|| format!("{a} needs a value"))?;
                o.keys.push((a.trim_start_matches('-').to_string(), v));
            }
            a if a.starts_with('-') => return Err(format!("unknown option `{a}`")),
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
