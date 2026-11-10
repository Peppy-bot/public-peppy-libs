//! Render the dual-arm hull model (and optionally the source meshes) at a
//! configuration into a self-contained interactive HTML scene.
//!
//! ```sh
//! cargo run --release --example visualize -- \
//!     --urdf tests/fixtures/openarm_v10.urdf --meshes ../openarm_description/assets/meshes \
//!     --left-base openarm_left_link0 --left-tip openarm_left_link7 \
//!     --right-base openarm_right_link0 --right-tip openarm_right_link7 \
//!     --yoked-joints 2 \
//!     --finger openarm_left_left_finger,0 --finger openarm_left_right_finger,0 \
//!     --finger openarm_right_left_finger,0 --finger openarm_right_right_finger,0 \
//!     --left 0,0,1.2,0.4,0,0,0 --right 0,0,-1.2,0.4,0,0,0 \
//!     --gripper 0.0 --wireframes -o scene.html
//! ```
//!
//! Each `--finger <link>,<shut>` names a finger the chains' links carry and the
//! position of its joint at which it is shut; the model refuses a URDF whose
//! fingers the arguments leave out, naming the link. `--yoked-joints` is the most
//! moving joints two bodies of one arm lie apart and still go unchecked.
//!
//! `--gripper <f>` (or per side `--left-gripper` / `--right-gripper`) sets the
//! gripper opening as a fraction in `[0, 1]` (0 = shut, 1 = fully open, the
//! default). The finger hulls, and their `--wireframes` source meshes, are placed
//! at that opening, so the scene shows the true finger positions rather than the
//! full swept envelope.
//!
//! A circumscribing fit carries no rounding, so a piece draws as its bare
//! polytope faces, which is exactly the surface the distance query measures
//! against. A piece that does carry a rounding radius additionally gets cylinders
//! along its edges and spheres at its vertices, standing in for the fillets of
//! the hull's Minkowski sum with a ball; those are tessellated, so that surface is
//! drawn to within the tessellation while GJK measures the true one.
//! `--wireframes` overlays the source
//! meshes underneath, so the gap between mesh and piece surface is the fit's
//! slack, visible directly. The closest pair is highlighted with the GJK/EPA
//! witness segment; the HUD shows the signed minimum distance against the band
//! thresholds.
use std::collections::HashSet;

use chain_kinematics::{Chain, ChainSpec, JointSelection};
use collision_geometry::urdf_collision::UrdfCollisions;
use openarm_description::{ARM_DOF, HardwareVersion};
use self_collision_model::nalgebra::{Point3, Vector3};
use self_collision_model::{Finger, PairSpec, Placed, SelfCollisionModel};

/// The generation whose torso regions the scene fits the torso in, when the URDF holds
/// its torso body.
const TORSO_VERSION: HardwareVersion = HardwareVersion::V1;

/// Wireframes are decimated to about this many triangles per body.
const MAX_WIRE_TRIS: usize = 2000;

fn main() {
    if let Err(e) = run() {
        eprintln!("visualize: {e}");
        std::process::exit(1);
    }
}

/// One arm's joints.
type Q = [f64; ARM_DOF];

struct Args {
    urdf: String,
    meshes: String,
    left_base: String,
    left_tip: String,
    right_base: String,
    right_tip: String,
    yoked_joints: Option<usize>,
    /// Each finger the arms carry, by its link, with its joint position at which it
    /// is shut.
    fingers: Vec<(String, f64)>,
    left: Q,
    right: Q,
    /// Gripper opening per side as a fraction in `[0, 1]` (0 = closed, 1 = open).
    left_gripper: f64,
    right_gripper: f64,
    out: String,
    wireframes: bool,
    d_stop: f64,
    d_safe: f64,
    exclude: Vec<PairSpec>,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        urdf: String::new(),
        meshes: String::new(),
        left_base: String::new(),
        left_tip: String::new(),
        right_base: String::new(),
        right_tip: String::new(),
        yoked_joints: None,
        fingers: Vec::new(),
        left: [0.0; ARM_DOF],
        right: [0.0; ARM_DOF],
        left_gripper: 1.0,
        right_gripper: 1.0,
        out: "scene.html".into(),
        wireframes: false,
        d_stop: 0.01,
        d_safe: 0.03,
        exclude: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        let fraction = |s: &str| -> Result<f64, String> {
            let f: f64 = s
                .parse()
                .map_err(|e| format!("bad gripper fraction '{s}': {e}"))?;
            (0.0..=1.0)
                .contains(&f)
                .then_some(f)
                .ok_or(format!("gripper fraction must be in [0, 1], got {f}"))
        };
        match flag.as_str() {
            "--urdf" => args.urdf = value()?,
            "--meshes" | "-m" => args.meshes = value()?,
            "--left-base" => args.left_base = value()?,
            "--left-tip" => args.left_tip = value()?,
            "--right-base" => args.right_base = value()?,
            "--right-tip" => args.right_tip = value()?,
            "--yoked-joints" => {
                args.yoked_joints = Some(value()?.parse().map_err(|e| format!("{e}"))?)
            }
            "--finger" | "-f" => {
                let v = value()?;
                let (link, shut) = v
                    .split_once(',')
                    .ok_or(format!("--finger wants 'link,shut', got '{v}'"))?;
                let shut: f64 = shut
                    .trim()
                    .parse()
                    .map_err(|e| format!("bad shut position '{shut}': {e}"))?;
                args.fingers.push((link.trim().to_string(), shut));
            }
            "--left" | "-l" => args.left = parse_joints(&value()?)?,
            "--right" | "-r" => args.right = parse_joints(&value()?)?,
            "--gripper" | "-g" => {
                let f = fraction(&value()?)?;
                args.left_gripper = f;
                args.right_gripper = f;
            }
            "--left-gripper" => args.left_gripper = fraction(&value()?)?,
            "--right-gripper" => args.right_gripper = fraction(&value()?)?,
            "--out" | "-o" => args.out = value()?,
            "--wireframes" | "-w" => args.wireframes = true,
            "--d-stop" => args.d_stop = value()?.parse().map_err(|e| format!("{e}"))?,
            "--d-safe" => args.d_safe = value()?.parse().map_err(|e| format!("{e}"))?,
            "--exclude" | "-x" => {
                let v = value()?;
                let (a, b) = v
                    .split_once(',')
                    .ok_or(format!("--exclude wants 'link_a,link_b', got '{v}'"))?;
                args.exclude.push(PairSpec::new(a.trim(), b.trim()));
            }
            other => return Err(format!("unknown argument '{other}'")),
        }
    }
    if args.urdf.is_empty()
        || args.meshes.is_empty()
        || args.left_base.is_empty()
        || args.left_tip.is_empty()
        || args.right_base.is_empty()
        || args.right_tip.is_empty()
        || args.yoked_joints.is_none()
    {
        return Err(
            "required: --urdf <file> --meshes <dir> --left-base <link> --left-tip <link> \
             --right-base <link> --right-tip <link> --yoked-joints <n>"
                .into(),
        );
    }
    Ok(args)
}

/// The arms' chains: each from its base to its tip, left then right.
fn chains(args: &Args) -> [ChainSpec<'_>; 2] {
    [
        (&args.left_base, &args.left_tip),
        (&args.right_base, &args.right_tip),
    ]
    .map(|(base, tip)| ChainSpec {
        base_link: Some(base.as_str()),
        tip_link: tip.as_str(),
        joints: JointSelection::PathOrder,
    })
}

fn parse_joints(s: &str) -> Result<Q, String> {
    let vals: Vec<f64> = s
        .split(',')
        .map(|p| {
            p.trim()
                .parse::<f64>()
                .map_err(|e| format!("bad joint '{p}': {e}"))
        })
        .collect::<Result<_, _>>()?;
    vals.try_into()
        .map_err(|v: Vec<f64>| format!("expected {ARM_DOF} joints, got {}", v.len()))
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;
    // The model is a pure distance oracle; the HUD colors by the caller's own
    // d_stop/d_safe (the --d-stop / --d-safe args), independent of any band.
    // Supply the tight torso proxy we ship whenever this is the OpenArm robot,
    // so the scene shows the real fit; any other URDF falls through to auto-fit.
    let urdf = std::fs::read_to_string(&args.urdf)
        .map_err(|e| format!("read urdf '{}': {e}", args.urdf))?;
    let fingers: Vec<Finger> = args
        .fingers
        .iter()
        .map(|(link, shut)| Finger {
            link: link.as_str(),
            shut: *shut,
        })
        .collect();
    let yoked = args
        .yoked_joints
        .expect("parse_args requires --yoked-joints");
    let mut builder =
        SelfCollisionModel::<ARM_DOF, 2>::builder(&urdf, &args.meshes, chains(&args), yoked)
            .fingers(&fingers)
            .exclude(&args.exclude);
    if UrdfCollisions::from_file(&args.urdf)?
        .collision_link_names()
        .iter()
        .any(|n| n == TORSO_VERSION.torso_body())
    {
        builder = builder.regions(TORSO_VERSION.torso_body(), TORSO_VERSION.torso_regions());
    }
    let mut model = builder.build()?;
    // Place the fingers at the requested opening; every query below then sees the
    // fingers where they actually are, not their full swept envelope.
    model.set_gripper_openings(&[args.left_gripper, args.right_gripper]);

    let q = [args.left, args.right];
    let proximity = model.min_distance(&q)?;
    let (distance, witness) = (
        proximity.distance,
        [point_json(&proximity.on_a), point_json(&proximity.on_b)],
    );
    let (link_a, link_b) = (proximity.link_a.to_string(), proximity.link_b.to_string());

    let bodies: Vec<serde_json::Value> = model
        .world_pieces(&q)?
        .into_iter()
        .flat_map(|(name, pieces)| {
            let side = if name.contains("_left_") {
                "left"
            } else if name.contains("_right_") {
                "right"
            } else {
                "fixed"
            };
            let hit = name == link_a || name == link_b;
            pieces
                .into_iter()
                .map(move |p| rounded_piece_json(&p, side, hit))
                .collect::<Vec<_>>()
        })
        .collect();

    let meshes = if args.wireframes {
        mesh_wireframes(&args)?
    } else {
        Vec::new()
    };

    let data = serde_json::json!({
        "dStop": args.d_stop,
        "dSafe": args.d_safe,
        "distance": round(distance),
        "pair": format!("{link_a}  vs  {link_b}"),
        "witness": witness,
        "bodies": bodies,
        "meshes": meshes,
    });
    // Escape "</script>" / "<!--" in any name so it cannot end the script early.
    let html = TEMPLATE.replace("__DATA__", &data.to_string().replace('<', "\\u003c"));
    std::fs::write(&args.out, html).map_err(|e| format!("write {}: {e}", args.out))?;
    println!(
        "wrote {} (d={distance:+.4} m, {link_a} vs {link_b})",
        args.out
    );
    Ok(())
}

/// Decimated source-mesh triangles per body in world frame, from the same URDF
/// and STL pipeline the fit used. Fingers are drawn at the requested opening
/// ([`Args::left_gripper`] / [`Args::right_gripper`]), matching the live finger
/// hulls rather than a swept envelope.
fn mesh_wireframes(args: &Args) -> Result<Vec<serde_json::Value>, String> {
    let urdf = UrdfCollisions::from_file(&args.urdf).map_err(|e| e.to_string())?;
    let robot = urdf_rs::read_file(&args.urdf).map_err(|e| e.to_string())?;
    let arms: Vec<Chain<ARM_DOF>> = chains(args)
        .iter()
        .map(|spec| Chain::<ARM_DOF>::from_urdf(&robot, spec).map_err(|e| e.to_string()))
        .collect::<Result<_, _>>()?;
    let chain_links: Vec<String> = arms
        .iter()
        .flat_map(|arm| {
            let posed = arm.at(&[0.0; ARM_DOF]);
            (0..ARM_DOF)
                .map(|i| posed.link_name(i).to_string())
                .collect::<Vec<_>>()
        })
        .collect();
    let attached: Vec<String> = chain_links
        .iter()
        .flat_map(|l| urdf.children_of(l))
        .filter(|c| !chain_links.contains(c))
        .collect();

    let mut out = Vec::new();
    for name in urdf.collision_link_names() {
        if chain_links.contains(&name) || attached.contains(&name) {
            continue;
        }
        let surfaces = urdf
            .fixed_surfaces_in_root(&name, &args.meshes)
            .map_err(|e| e.to_string())?;
        out.push(wire_json(decimate(&surfaces.concat())));
    }
    for (arm, q, opening) in [
        (&arms[0], &args.left, args.left_gripper),
        (&arms[1], &args.right, args.right_gripper),
    ] {
        let posed = arm.at(q);
        for i in 0..ARM_DOF {
            let name = posed.link_name(i);
            let pose = posed.link_pose_world(i);
            let mut verts: Vec<Point3<f64>> = urdf
                .link_surfaces(name, &args.meshes)?
                .concat()
                .iter()
                .map(|v| pose * v)
                .collect();
            // Fixed children ride with the link; movable children are the
            // gripper fingers, drawn at the requested opening, each opening from
            // its named shut position toward its joint's other limit, as the
            // model places its hull.
            let mut fingers: Vec<(String, f64, f64)> = Vec::new();
            for child in urdf.children_of(name) {
                if chain_links.contains(&child) || urdf.collisions_of(&child).is_empty() {
                    continue;
                }
                let joint = urdf
                    .parent_joint(&child)
                    .expect("children_of implies a parent joint");
                if joint.is_fixed() {
                    verts.extend(
                        urdf.child_surfaces_in_parent(&child, joint.lower_limit, &args.meshes)?
                            .concat()
                            .iter()
                            .map(|v| pose * v),
                    );
                } else {
                    let shut = args
                        .fingers
                        .iter()
                        .find(|(link, _)| *link == child)
                        .map(|(_, shut)| *shut)
                        .ok_or(format!("finger '{child}' needs a --finger {child},<shut>"))?;
                    let open = if shut == joint.lower_limit {
                        joint.upper_limit
                    } else {
                        joint.lower_limit
                    };
                    fingers.push((child, shut, open));
                }
            }
            for (child, closed, open) in &fingers {
                let qc = closed + opening * (open - closed);
                verts.extend(
                    urdf.child_surfaces_in_parent(child, qc, &args.meshes)?
                        .concat()
                        .iter()
                        .map(|v| pose * v),
                );
            }
            out.push(wire_json(decimate(&verts)));
        }
    }
    Ok(out)
}

/// Keep every k-th triangle so each body stays under [`MAX_WIRE_TRIS`].
fn decimate(verts: &[Point3<f64>]) -> Vec<Point3<f64>> {
    let (triangles, _) = verts.as_chunks::<3>();
    let step = triangles.len().div_ceil(MAX_WIRE_TRIS).max(1);
    triangles.iter().step_by(step).flatten().copied().collect()
}

/// One hull piece as render data: the faces offset outward by any rounding
/// radius (the flat caps), the bare vertices (sphere centers), and the unique
/// edges (cylinder axes). At `radius == 0` the caps alone are the surface the
/// distance query sees, drawn exactly and with no fillets. Otherwise the browser
/// sweeps a ball of `radius` over the edges and vertices, so caps + edge
/// cylinders + vertex spheres union into that surface up to the tessellation of
/// the spheres and cylinders.
fn rounded_piece_json(p: &Placed, side: &str, hit: bool) -> serde_json::Value {
    let vertices: Vec<Point3<f64>> = p.vertices().collect();
    let (faces, radius) = (p.hull().faces(), p.hull().inflation());
    let centroid = Point3::from(
        vertices.iter().fold(Vector3::zeros(), |a, v| a + v.coords) / vertices.len() as f64,
    );
    let mut caps: Vec<f64> = Vec::new();
    for f in faces {
        let (a, b, c) = (vertices[f[0]], vertices[f[1]], vertices[f[2]]);
        let mut n = (b - a).cross(&(c - a));
        // Orient outward by the centroid; the hull's own winding is not relied on.
        let face_center = Point3::from((a.coords + b.coords + c.coords) / 3.0);
        if n.dot(&(face_center - centroid)) < 0.0 {
            n = -n;
        }
        let offset = if n.norm_squared() > 1e-20 {
            n.normalize() * radius
        } else {
            Vector3::zeros()
        };
        for v in [a, b, c] {
            caps.extend([
                round(v.x + offset.x),
                round(v.y + offset.y),
                round(v.z + offset.z),
            ]);
        }
    }
    let verts = flat(&vertices);
    let mut seen: HashSet<(usize, usize)> = HashSet::new();
    let mut edges: Vec<f64> = Vec::new();
    for f in faces {
        for (i, j) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            let edge = if i < j { (i, j) } else { (j, i) };
            if seen.insert(edge) {
                edges.extend(flat(&[vertices[edge.0], vertices[edge.1]]));
            }
        }
    }
    serde_json::json!({ "side": side, "hit": hit, "radius": round(radius), "caps": caps, "verts": verts, "edges": edges })
}

fn flat(verts: &[Point3<f64>]) -> Vec<f64> {
    verts
        .iter()
        .flat_map(|p| [round(p.x), round(p.y), round(p.z)])
        .collect()
}

fn wire_json(verts: Vec<Point3<f64>>) -> serde_json::Value {
    serde_json::json!({ "positions": flat(&verts) })
}

fn point_json(p: &Point3<f64>) -> serde_json::Value {
    serde_json::json!([round(p.x), round(p.y), round(p.z)])
}

fn round(x: f64) -> f64 {
    (x * 10_000.0).round() / 10_000.0
}

const TEMPLATE: &str = r##"<!DOCTYPE html>
<html><head><meta charset="utf-8"/><title>self_collision_model scene</title>
<style>
  body { margin:0; background:#15171c; color:#d7dae0; font:13px/1.4 system-ui, sans-serif; }
  #hud { position:fixed; top:10px; left:10px; background:rgba(21,23,28,.85); padding:10px 12px; border:1px solid #333; border-radius:6px; }
  #hud b.bad{color:#ff5a5a} #hud b.warn{color:#ffb054} #hud b.ok{color:#7dd87d}
  .dot{display:inline-block;width:.7em;height:.7em;border-radius:50%;margin-right:.35em}
  .legend span{margin-right:1em}
</style>
<script type="importmap">
{ "imports": { "three":"https://unpkg.com/three@0.160.0/build/three.module.js",
               "three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/" } }
</script>
</head><body><div id="hud"></div>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const DATA = __DATA__;
const renderer = new THREE.WebGLRenderer({ antialias:true });
renderer.setSize(innerWidth, innerHeight); document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x15171c);
scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const sun = new THREE.DirectionalLight(0xffffff, 1.2); sun.position.set(2, -1.5, 3); scene.add(sun);
const camera = new THREE.PerspectiveCamera(50, innerWidth/innerHeight, 0.01, 50);
camera.up.set(0,0,1); camera.position.set(1.4,-1.2,1.0);
const controls = new OrbitControls(camera, renderer.domElement); controls.target.set(0,0,0.45);
const grid = new THREE.GridHelper(2,20,0x3a3f4a,0x262a33); grid.rotation.x = Math.PI/2; scene.add(grid);
scene.add(new THREE.AxesHelper(0.25));
const COL = { left:0x4f8fde, right:0x53b97a, fixed:0x8a8f9c };
const UP = new THREE.Vector3(0, 1, 0);
// Each piece is its collision surface: faces offset outward by any radius (caps),
// cylinders along the edges, and spheres at the vertices fill the fillets.
for (const b of DATA.bodies) {
  const color = b.hit ? (DATA.distance <= 0 ? 0xe03c3c : 0xe2902f) : COL[b.side];
  // Faint enough to read the source mesh through the proxy: the gap between the
  // two is the whole point of the scene, so the proxy tints the geometry rather
  // than hiding it.
  const mat = new THREE.MeshStandardMaterial({ color, transparent:true, opacity: b.hit ? 0.35 : 0.20, depthWrite:false, roughness:0.6, side:THREE.DoubleSide });
  if (b.caps.length) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(b.caps, 3));
    g.computeVertexNormals();
    scene.add(new THREE.Mesh(g, mat));
  }
  const nv = b.verts.length / 3;
  if (nv && b.radius > 0) {
    const im = new THREE.InstancedMesh(new THREE.SphereGeometry(b.radius, 12, 8), mat, nv);
    const m = new THREE.Matrix4();
    for (let i = 0; i < nv; i++) { im.setMatrixAt(i, m.makeTranslation(b.verts[3*i], b.verts[3*i+1], b.verts[3*i+2])); }
    im.instanceMatrix.needsUpdate = true;
    scene.add(im);
  }
  const ne = b.edges.length / 6;
  if (ne && b.radius > 0) {
    const im = new THREE.InstancedMesh(new THREE.CylinderGeometry(b.radius, b.radius, 1, 10, 1, true), mat, ne);
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), a = new THREE.Vector3(), bb = new THREE.Vector3(), d = new THREE.Vector3(), mid = new THREE.Vector3();
    for (let i = 0; i < ne; i++) {
      a.set(b.edges[6*i], b.edges[6*i+1], b.edges[6*i+2]);
      bb.set(b.edges[6*i+3], b.edges[6*i+4], b.edges[6*i+5]);
      d.subVectors(bb, a); const len = d.length(); d.normalize();
      q.setFromUnitVectors(UP, d);
      mid.addVectors(a, bb).multiplyScalar(0.5);
      im.setMatrixAt(i, m.compose(mid, q, new THREE.Vector3(1, len, 1)));
    }
    im.instanceMatrix.needsUpdate = true;
    scene.add(im);
  }
}
for (const w of DATA.meshes) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(w.positions, 3));
  scene.add(new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color:0xc2c8d2, wireframe:true })));
}
{ const [a,b] = DATA.witness.map(p => new THREE.Vector3(...p));
  scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([a,b]), new THREE.LineBasicMaterial({ color:0xff5a5a })));
  for (const p of [a,b]) { const d = new THREE.Mesh(new THREE.SphereGeometry(0.008), new THREE.MeshBasicMaterial({ color:0xff5a5a })); d.position.copy(p); scene.add(d); } }
const cls = DATA.distance <= DATA.dStop ? 'bad' : (DATA.distance < DATA.dSafe ? 'warn' : 'ok');
document.getElementById('hud').innerHTML =
  `<div>min distance <b class="${cls}">${DATA.distance.toFixed(4)} m</b> (${DATA.pair})</div>` +
  `<div class="legend"><span><i class="dot" style="background:#4f8fde"></i>left</span>` +
  `<span><i class="dot" style="background:#53b97a"></i>right</span>` +
  `<span><i class="dot" style="background:#8a8f9c"></i>fixed</span>` +
  `<span><i class="dot" style="background:#e2902f"></i>closest pair</span>` +
  `<span>solid = collision surface</span>` +
  `${DATA.meshes.length ? '<span>wireframe = source mesh</span>' : ''}</div>`;
addEventListener('resize', () => { camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script></body></html>
"##;
