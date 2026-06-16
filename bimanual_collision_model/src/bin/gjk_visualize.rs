//! Render the convex-hull + GJK scene: each body's hull wireframe, the closest
//! pair highlighted, the GJK witness segment, and the capsule versus hull
//! minimum distance in the HUD so the tightness gain is visible.
//!
//! ```sh
//! cargo run --release --bin gjk_visualize -- --left ..7.. --right ..7.. -o scene.html
//! ```
use std::collections::HashMap;

use bimanual_collision_model::gjk::{self, Hull, Support};
use bimanual_collision_model::nalgebra::{Isometry3, Point3, Vector3};
use bimanual_collision_model::{BimanualCollisionModel, Capsule, GovernorBand, MarginPolicy};
use srs_model::{ARM_DOF, JointVec};

#[path = "shared/eval_common.rs"]
mod common;

struct PosedHull<'a> {
    hull: &'a Hull,
    iso: Isometry3<f64>,
}

impl Support for PosedHull<'_> {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        self.iso * self.hull.core_support(&self.iso.inverse_transform_vector(dir))
    }
}

fn main() {
    if let Err(e) = run() {
        eprintln!("gjk_visualize: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let (left, right, out) = parse_args()?;
    let policy = MarginPolicy { band: GovernorBand::new(0.01, 0.03).unwrap(), references: vec![[0.0; 7]] };
    let mut model = BimanualCollisionModel::from_urdf_file(common::URDF, common::MESHES, common::FIXED[1], common::FIXED[2], &policy)?;
    let pairs: Vec<(String, String)> = model.checked_pairs().iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();

    let convex = common::fit_hulls();
    let hulls: HashMap<String, Vec<Hull>> =
        convex.iter().map(|(k, pieces)| (k.clone(), pieces.iter().map(|(ch, r)| Hull::new(ch, *r).unwrap()).collect())).collect();
    let iso = common::body_isometries(&left, &right);
    let posed: HashMap<String, Vec<PosedHull>> =
        hulls.iter().map(|(k, hs)| (k.clone(), hs.iter().map(|h| PosedHull { hull: h, iso: iso[k] }).collect())).collect();

    // Raw capsule minimum for the HUD comparison.
    let placed: HashMap<String, Vec<Capsule>> =
        model.world_capsules(&left, &right)?.into_iter().map(|(n, c)| (n.to_string(), c)).collect();
    let cap_dist = |a: &str, b: &str| placed[a].iter().flat_map(|ca| placed[b].iter().map(move |cb| ca.distance_to(cb).distance)).fold(f64::INFINITY, f64::min);
    let cap_min = pairs.iter().map(|(a, b)| cap_dist(a, b)).fold(f64::INFINITY, f64::min);

    // GJK minimum over piece pairs, with the witness segment and closest pair.
    let (mut hull_min, mut witness, mut hit) = (f64::INFINITY, (Point3::origin(), Point3::origin()), (String::new(), String::new()));
    for (a, b) in &pairs {
        for x in &posed[a] {
            for y in &posed[b] {
                let r = gjk::distance(x, y);
                if r.distance < hull_min {
                    (hull_min, witness, hit) = (r.distance, (r.on_a, r.on_b), (a.clone(), b.clone()));
                }
            }
        }
    }

    // World-placed hull triangles, one entry per piece.
    let mut bodies = Vec::new();
    for (name, pieces) in &convex {
        let m = iso[name];
        let side = common::side(name);
        let is_hit = *name == hit.0 || *name == hit.1;
        for (ch, _radius) in pieces {
            let positions: Vec<f64> = ch
                .faces
                .iter()
                .flatten()
                .flat_map(|&vi| {
                    let p = m * ch.vertices[vi];
                    [round(p.x), round(p.y), round(p.z)]
                })
                .collect();
            bodies.push(serde_json::json!({ "side": side, "hit": is_hit, "positions": positions }));
        }
    }

    // Source mesh wireframes (decimated), placed in world frame, to eyeball
    // how the hull pieces sit against the real geometry.
    let meshes: Vec<serde_json::Value> = common::body_vertices()
        .iter()
        .map(|(name, verts)| {
            let m = iso[name];
            let positions: Vec<f64> = decimate(verts).iter().flat_map(|v| {
                let p = m * v;
                [round(p.x), round(p.y), round(p.z)]
            }).collect();
            serde_json::json!({ "positions": positions })
        })
        .collect();

    let data = serde_json::json!({
        "capMin": round(cap_min),
        "hullMin": round(hull_min),
        "pair": format!("{}  vs  {}", hit.0, hit.1),
        "witness": [point_json(&witness.0), point_json(&witness.1)],
        "dStop": 0.01,
        "dSafe": 0.03,
        "bodies": bodies,
        "meshes": meshes,
    });
    let json = data.to_string().replace('<', "\\u003c");
    std::fs::write(&out, TEMPLATE.replace("__DATA__", &json)).map_err(|e| format!("write {out}: {e}"))?;
    println!("wrote {out} (capsule {cap_min:+.4} m, hull {hull_min:+.4} m, {} vs {})", hit.0, hit.1);
    Ok(())
}

fn parse_args() -> Result<(JointVec, JointVec, String), String> {
    let mut left: JointVec = [-0.45, -0.1, 0.0, 0.5, 0.0, -0.3, 0.0];
    let mut right: JointVec = [0.4, 0.1, 0.0, 0.7, 0.0, -0.2, 0.0];
    let mut out = "scene.html".to_string();
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--left" | "-l" => left = parse_joints(&value()?)?,
            "--right" | "-r" => right = parse_joints(&value()?)?,
            "--out" | "-o" => out = value()?,
            other => return Err(format!("unknown argument '{other}'")),
        }
    }
    Ok((left, right, out))
}

fn parse_joints(s: &str) -> Result<JointVec, String> {
    let vals: Vec<f64> = s.split(',').map(|p| p.trim().parse::<f64>().map_err(|e| format!("bad joint '{p}': {e}"))).collect::<Result<_, _>>()?;
    vals.try_into().map_err(|v: Vec<f64>| format!("expected {ARM_DOF} joints, got {}", v.len()))
}

fn round(x: f64) -> f64 {
    (x * 10_000.0).round() / 10_000.0
}

fn point_json(p: &Point3<f64>) -> serde_json::Value {
    serde_json::json!([round(p.x), round(p.y), round(p.z)])
}

/// Keep every k-th triangle so each mesh stays under this many for the overlay.
const MAX_WIRE_TRIS: usize = 2000;

fn decimate(verts: &[Point3<f64>]) -> Vec<Point3<f64>> {
    let step = (verts.len() / 3).div_ceil(MAX_WIRE_TRIS).max(1);
    verts.chunks_exact(3).step_by(step).flatten().copied().collect()
}

const TEMPLATE: &str = r##"<!DOCTYPE html>
<html><head><meta charset="utf-8"/><title>hull + GJK scene</title>
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
for (const b of DATA.bodies) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(b.positions, 3));
  g.computeVertexNormals();
  const color = b.hit ? (DATA.hullMin <= 0 ? 0xe03c3c : 0xe2902f) : COL[b.side];
  // Solid translucent surfaces (not wireframe): far fewer primitives, so the
  // scene stays light to orbit.
  scene.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color, transparent:true, opacity: b.hit ? 0.55 : 0.22, depthWrite:false, roughness:0.6, side:THREE.DoubleSide })));
}
for (const w of DATA.meshes) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(w.positions, 3));
  scene.add(new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color:0x6b7280, wireframe:true })));
}
{ const [a,b] = DATA.witness.map(p => new THREE.Vector3(...p));
  scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([a,b]), new THREE.LineBasicMaterial({ color:0xff5a5a })));
  for (const p of [a,b]) { const d = new THREE.Mesh(new THREE.SphereGeometry(0.008), new THREE.MeshBasicMaterial({ color:0xff5a5a })); d.position.copy(p); scene.add(d); } }
const cls = DATA.hullMin <= DATA.dStop ? 'bad' : (DATA.hullMin < DATA.dSafe ? 'warn' : 'ok');
document.getElementById('hud').innerHTML =
  `<div>hull+GJK min <b class="${cls}">${DATA.hullMin.toFixed(4)} m</b> (${DATA.pair})</div>` +
  `<div>capsule min ${DATA.capMin.toFixed(4)} m; the gap is capsule looseness the hull recovers.</div>` +
  `<div class="legend"><span><i class="dot" style="background:#4f8fde"></i>left</span>` +
  `<span><i class="dot" style="background:#53b97a"></i>right</span>` +
  `<span><i class="dot" style="background:#8a8f9c"></i>fixed</span>` +
  `<span><i class="dot" style="background:#e2902f"></i>closest pair</span>` +
  `<span>solid = hull pieces, wireframe = source mesh</span></div>`;
addEventListener('resize', () => { camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script></body></html>
"##;
