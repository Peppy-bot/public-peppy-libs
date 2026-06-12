//! Render the dual-arm capsule model (and optionally the source meshes) at a
//! configuration into a self-contained interactive HTML scene.
//!
//! ```sh
//! cargo run --release --bin visualize -- \
//!     --urdf tests/fixtures/openarm_v10.urdf --meshes tests/fixtures/meshes \
//!     --left-base openarm_left_link0 --right-base openarm_right_link0 \
//!     --left -0.45,-0.1,0,0.5,0,-0.3,0 --right 0.4,0.1,0,0.7,0,-0.2,0 \
//!     --wireframes -o scene.html
//! ```
//!
//! The page loads three.js from a CDN; everything else (capsules, meshes,
//! witness segment) is embedded. Capsules are colored by side, the closest
//! pair is highlighted, and the margin-adjusted distance is shown. Use it to
//! eyeball fit quality against the mesh wireframes and to review scenarios.

use collision_model::DualArmCollisionModel;
use collision_model::config::CollisionConfig;
use collision_model::nalgebra::Point3;
use collision_model::urdf_collision::UrdfCollisions;
use srs_model::{ARM_DOF, Arm, JointVec};

/// Wireframes are decimated to about this many triangles per body; fit
/// eyeballing needs shape, not the full decimation-resistant mesh.
const MAX_WIRE_TRIS: usize = 2500;

fn main() {
    if let Err(e) = run() {
        eprintln!("visualize: {e}");
        std::process::exit(1);
    }
}

struct Args {
    urdf: String,
    config: String,
    left_base: String,
    right_base: String,
    left: JointVec,
    right: JointVec,
    out: String,
    meshes: Option<String>,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        urdf: String::new(),
        config: String::new(),
        left_base: String::new(),
        right_base: String::new(),
        left: [0.0; ARM_DOF],
        right: [0.0; ARM_DOF],
        out: "scene.html".into(),
        meshes: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--urdf" => args.urdf = value()?,
            "--config" => args.config = value()?,
            "--left-base" => args.left_base = value()?,
            "--right-base" => args.right_base = value()?,
            "--left" | "-l" => args.left = parse_joints(&value()?)?,
            "--right" | "-r" => args.right = parse_joints(&value()?)?,
            "--out" | "-o" => args.out = value()?,
            "--meshes" | "-m" => args.meshes = Some(value()?),
            other => return Err(format!("unknown argument '{other}'")),
        }
    }
    if args.urdf.is_empty() || args.config.is_empty() || args.left_base.is_empty() || args.right_base.is_empty() {
        return Err("required: --urdf <file> --config <json> --left-base <link> --right-base <link>".into());
    }
    Ok(args)
}

fn parse_joints(s: &str) -> Result<JointVec, String> {
    let vals: Vec<f64> = s
        .split(',')
        .map(|p| p.trim().parse::<f64>().map_err(|e| format!("bad joint value '{p}': {e}")))
        .collect::<Result<_, _>>()?;
    vals.try_into().map_err(|v: Vec<f64>| format!("expected {ARM_DOF} joint values, got {}", v.len()))
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let config = CollisionConfig::from_file(&args.config)?.parse()?;
    let mut model =
        DualArmCollisionModel::from_urdf_file(&args.urdf, &args.left_base, &args.right_base, &config)?;
    let proximity = model.min_distance(&args.left, &args.right)?;
    let (witness_a, witness_b) = (proximity.on_a, proximity.on_b);
    let (link_a, link_b) = (proximity.link_a.to_string(), proximity.link_b.to_string());
    let distance = proximity.distance;

    let mut bodies = Vec::new();
    for (name, capsules) in model.world_capsules(&args.left, &args.right)? {
        let side = if name.contains("_left_") { "left" } else if name.contains("_right_") { "right" } else { "fixed" };
        let caps: Vec<serde_json::Value> = capsules
            .iter()
            .map(|c| {
                serde_json::json!({
                    "a": point_json(&c.a),
                    "b": point_json(&c.b),
                    "r": round(c.radius),
                })
            })
            .collect();
        bodies.push(serde_json::json!({
            "name": name,
            "side": side,
            "hit": name == link_a || name == link_b,
            "capsules": caps,
        }));
    }

    let meshes = match &args.meshes {
        Some(dir) => mesh_wireframes(&args, dir)?,
        None => Vec::new(),
    };

    let data = serde_json::json!({
        "title": format!("left {:?}  right {:?}", args.left, args.right),
        "distance": round(distance),
        "pair": format!("{link_a}  vs  {link_b}"),
        "witness": [point_json(&witness_a), point_json(&witness_b)],
        "bodies": bodies,
        "meshes": meshes,
    });

    // Escape for embedding inside a <script> block: a literal "</script>"
    // (or "<!--") in any name would end the script early.
    let json = data.to_string().replace('<', "\\u003c");
    let html = TEMPLATE.replace("__DATA__", &json);
    std::fs::write(&args.out, html).map_err(|e| format!("write {}: {e}", args.out))?;
    println!("wrote {} (d={distance:+.4} m, {link_a} vs {link_b})", args.out);
    Ok(())
}

/// World-frame decimated triangle soup per body, from the same URDF + STL
/// pipeline the fit used. Fingers are drawn at full open, matching the
/// worst-case capsules baked into the wrist.
fn mesh_wireframes(args: &Args, meshes_dir: &str) -> Result<Vec<serde_json::Value>, String> {
    let urdf = UrdfCollisions::from_file(&args.urdf)?;
    let config = CollisionConfig::from_file(&args.config)?.parse()?;

    let mut out = Vec::new();
    for (name, _) in &config.fixed {
        let vertices = urdf.fixed_vertices_in_root(name, meshes_dir)?;
        out.push(wire_json(name, decimate(&vertices)));
    }
    for (base, q) in [(&args.left_base, &args.left), (&args.right_base, &args.right)] {
        let mut arm = Arm::from_urdf_file(&args.urdf, base)?;
        let posed = arm.at(q);
        for i in 0..ARM_DOF {
            let name = posed.link_name(i);
            let pose = posed.link_pose_world(i);
            let mut vertices: Vec<Point3<f64>> =
                urdf.link_vertices(&name, meshes_dir)?.iter().map(|v| pose * v).collect();
            // Attached collision-bearing children (gripper fingers) drawn at
            // full extension, matching the worst-case capsules.
            for child in urdf.children_of(&name) {
                if urdf.collisions_of(&child).is_empty() {
                    continue;
                }
                let open = urdf.parent_joint(&child).map(|j| j.upper_limit).unwrap_or(0.0);
                vertices.extend(urdf.child_vertices_in_parent(&child, open, meshes_dir)?.iter().map(|v| pose * v));
            }
            out.push(wire_json(&name, decimate(&vertices)));
        }
    }
    Ok(out)
}

/// Keep every k-th triangle so each body stays under [`MAX_WIRE_TRIS`].
fn decimate(vertices: &[Point3<f64>]) -> Vec<Point3<f64>> {
    let tris = vertices.len() / 3;
    let step = tris.div_ceil(MAX_WIRE_TRIS).max(1);
    vertices.chunks_exact(3).step_by(step).flatten().copied().collect()
}

fn wire_json(name: &str, vertices: Vec<Point3<f64>>) -> serde_json::Value {
    let flat: Vec<f64> = vertices.iter().flat_map(|p| [round(p.x), round(p.y), round(p.z)]).collect();
    serde_json::json!({ "name": name, "positions": flat })
}

fn point_json(p: &Point3<f64>) -> serde_json::Value {
    serde_json::json!([round(p.x), round(p.y), round(p.z)])
}

/// 0.1 mm grid keeps the embedded JSON compact.
fn round(x: f64) -> f64 {
    (x * 10_000.0).round() / 10_000.0
}

const TEMPLATE: &str = r##"<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>collision_model scene</title>
<style>
  body { margin: 0; background: #15171c; color: #d7dae0; font: 13px/1.4 system-ui, sans-serif; }
  #hud { position: fixed; top: 10px; left: 10px; background: rgba(21,23,28,.85); padding: 10px 12px;
         border: 1px solid #333; border-radius: 6px; max-width: 46em; }
  #hud b.bad { color: #ff5a5a; } #hud b.warn { color: #ffb054; } #hud b.ok { color: #7dd87d; }
  .legend span { display: inline-block; margin-right: 1em; }
  .dot { display: inline-block; width: .7em; height: .7em; border-radius: 50%; margin-right: .35em; }
</style>
<script type="importmap">
{ "imports": { "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
               "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/" } }
</script>
</head>
<body>
<div id="hud"></div>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const DATA = __DATA__;

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x15171c);
const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.01, 50);
camera.up.set(0, 0, 1);
camera.position.set(1.4, -1.2, 1.0);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.45);

scene.add(new THREE.AmbientLight(0xffffff, 0.45));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(2, -1.5, 3);
scene.add(sun);

const grid = new THREE.GridHelper(2, 20, 0x3a3f4a, 0x262a33);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(0.25));

const SIDE_COLORS = { left: 0x4f8fde, right: 0x53b97a, fixed: 0x8a8f9c };

function addCapsule(c, color, hit) {
  const a = new THREE.Vector3(...c.a), b = new THREE.Vector3(...c.b);
  const dir = b.clone().sub(a), len = dir.length();
  const geo = new THREE.CapsuleGeometry(c.r, len, 8, 24);
  const mat = new THREE.MeshStandardMaterial({
    color: hit ? (DATA.distance <= 0 ? 0xe03c3c : 0xe2902f) : color,
    transparent: true, depthWrite: false, opacity: hit ? 0.45 : 0.15, roughness: 0.6,
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.copy(a.clone().add(b).multiplyScalar(0.5));
  if (len > 1e-9) mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  scene.add(mesh);
}

for (const body of DATA.bodies)
  for (const c of body.capsules) addCapsule(c, SIDE_COLORS[body.side], body.hit);

for (const wire of DATA.meshes) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(wire.positions, 3));
  // Opaque on purpose: a transparent wireframe joins the sorted transparent
  // pass, where camera-dependent draw order against the capsules makes whole
  // meshes wash in and out. Opaque renders first, always. The dim color
  // stands in for opacity.
  scene.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x9aa3b2, wireframe: true })));
}

{ // Witness segment between the closest pair, with endpoint markers.
  const [wa, wb] = DATA.witness.map(p => new THREE.Vector3(...p));
  const lineGeo = new THREE.BufferGeometry().setFromPoints([wa, wb]);
  scene.add(new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0xff5a5a })));
  for (const p of [wa, wb]) {
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.008),
      new THREE.MeshBasicMaterial({ color: 0xff5a5a }));
    dot.position.copy(p);
    scene.add(dot);
  }
}

const cls = DATA.distance <= 0 ? 'bad' : (DATA.distance < 0.08 ? 'warn' : 'ok');
document.getElementById('hud').innerHTML =
  `<div>min adjusted distance <b class="${cls}">${DATA.distance.toFixed(4)} m</b> &mdash; ${DATA.pair}</div>` +
  `<div>${DATA.title}</div>` +
  `<div class="legend"><span><i class="dot" style="background:#4f8fde"></i>left</span>` +
  `<span><i class="dot" style="background:#53b97a"></i>right</span>` +
  `<span><i class="dot" style="background:#8a8f9c"></i>fixed</span>` +
  `<span><i class="dot" style="background:#e2902f"></i>closest pair</span>` +
  `${DATA.meshes.length ? '<span>wireframe = source meshes</span>' : ''}</div>`;

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script>
</body>
</html>
"##;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_joints_accepts_seven_and_rejects_other_counts() {
        let q = parse_joints("0, -0.5 ,1.25,0,0,0,0.1").expect("seven values");
        assert_eq!(q[1], -0.5);
        assert_eq!(q[6], 0.1);
        assert!(parse_joints("1,2,3").is_err());
        assert!(parse_joints("1,2,3,4,5,6,7,8").is_err());
        assert!(parse_joints("1,2,x,4,5,6,7").is_err());
    }

    #[test]
    fn decimate_keeps_whole_triangles_under_the_cap() {
        let tri = |k: f64| {
            [Point3::new(k, 0.0, 0.0), Point3::new(k, 1.0, 0.0), Point3::new(k, 0.0, 1.0)]
        };
        let big: Vec<Point3<f64>> = (0..(MAX_WIRE_TRIS * 3)).flat_map(|k| tri(k as f64)).collect();
        let out = decimate(&big);
        assert_eq!(out.len() % 3, 0, "whole triangles only");
        assert!(out.len() / 3 <= MAX_WIRE_TRIS);
        // Every kept triangle is one of the originals, not a re-stitch.
        assert_eq!(out[0..3].iter().map(|p| p.x).collect::<Vec<_>>(), vec![0.0, 0.0, 0.0]);
        let small: Vec<Point3<f64>> = (0..10).flat_map(|k| tri(k as f64)).collect();
        assert_eq!(decimate(&small).len(), 30, "under the cap nothing is dropped");
    }

    #[test]
    fn embedded_json_escapes_script_breakers() {
        let json = serde_json::json!({"name": "</script><b>"}).to_string().replace('<', "\\u003c");
        assert!(!json.contains("</script>"));
        assert!(json.contains("\\u003c/script>"));
    }
}
