//! Render the collision pieces of a robot model robot_description bundles (and
//! optionally the source meshes) at a configuration into a self-contained interactive
//! HTML scene.
//!
//! ```sh
//! cargo run --release --example visualize -- --model openarm_v1 \
//!     --joints left_arm=0,0,1.2,0.4,0,0,0 --joints right_arm=0,0,-1.2,0.4,0,0,0 \
//!     --gripper 0.0 --wireframes -o scene.html
//! ```
//!
//! `--model <id>` names the model, whose collision meshes the scene writes to a
//! directory of the system's temporary directory. `--joints <arm>=<positions>` sets the
//! joint positions of an arm, named as its SRDF group, base to tip; an arm the arguments
//! leave out stands at its `home` state.
//!
//! `--gripper <f>` sets the opening of every gripper as a fraction in `[0, 1]` (0 =
//! closed, 1 = fully open, the default), and `--gripper <arm>=<f>` that of one arm's
//! gripper. The finger hulls, and their `--wireframes` source meshes, are placed at
//! that opening, so the scene shows the true finger positions.
//!
//! A circumscribing fit carries no rounding, so a piece draws as its bare
//! polytope faces, which is exactly the surface the distance query measures
//! against. A piece that does carry a rounding radius additionally gets cylinders
//! along its edges and spheres at its vertices, standing in for the fillets of
//! the hull's Minkowski sum with a ball; those are tessellated, so that surface is
//! drawn to within the tessellation while GJK measures the true one.
//! `--wireframes` overlays the source
//! meshes underneath, so the gap between mesh and piece surface is the fit's
//! slack, visible directly. The pieces are colored by the arm they ride; the closest
//! pair is highlighted with the GJK/EPA witness segment; the HUD shows the signed
//! minimum distance against the band thresholds.
use std::collections::HashSet;

use chain_kinematics::{Chain, ChainSpec, JointSelection};
use collision_geometry::urdf_collision::UrdfCollisions;
use robot_description::{Arm, OnShape, RobotDescription, on_shape, states};
use self_collision_model::nalgebra::{Point3, Vector3};
use self_collision_model::{Joints, Placed, SelfCollisionModel};

/// Wireframes are decimated to about this many triangles per body.
const MAX_WIRE_TRIS: usize = 2000;

/// The opening of a gripper the arguments give none: fully open.
const DEFAULT_OPENING: f64 = 1.0;

fn main() {
    if let Err(e) = run() {
        eprintln!("visualize: {e}");
        std::process::exit(1);
    }
}

struct Args {
    model: String,
    /// The joint positions of each arm the arguments name, by the arm's name.
    joints: Vec<(String, Vec<f64>)>,
    /// The opening of every gripper as a fraction in `[0, 1]` (0 = closed, 1 = open).
    gripper: f64,
    /// The opening of each arm's gripper the arguments name, by the arm's name.
    arm_grippers: Vec<(String, f64)>,
    out: String,
    wireframes: bool,
    d_stop: f64,
    d_safe: f64,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        model: String::new(),
        joints: Vec::new(),
        gripper: DEFAULT_OPENING,
        arm_grippers: Vec::new(),
        out: "scene.html".into(),
        wireframes: false,
        d_stop: 0.01,
        d_safe: 0.03,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--model" => args.model = value()?,
            "--joints" => {
                let v = value()?;
                let (arm, positions) = v
                    .split_once('=')
                    .ok_or(format!("--joints wants '<arm>=<positions>', got '{v}'"))?;
                args.joints
                    .push((arm.trim().to_string(), parse_joints(positions)?));
            }
            "--gripper" | "-g" => {
                let v = value()?;
                match v.split_once('=') {
                    Some((arm, f)) => args
                        .arm_grippers
                        .push((arm.trim().to_string(), fraction(f)?)),
                    None => args.gripper = fraction(&v)?,
                }
            }
            "--out" | "-o" => args.out = value()?,
            "--wireframes" | "-w" => args.wireframes = true,
            "--d-stop" => args.d_stop = value()?.parse().map_err(|e| format!("{e}"))?,
            "--d-safe" => args.d_safe = value()?.parse().map_err(|e| format!("{e}"))?,
            other => return Err(format!("unknown argument '{other}'")),
        }
    }
    if args.model.is_empty() {
        let ids: Vec<&str> = robot_description::models()
            .map(|model| model.id())
            .collect();
        return Err(format!("required: --model <id>, one of {}", ids.join(", ")));
    }
    Ok(args)
}

fn parse_joints(s: &str) -> Result<Vec<f64>, String> {
    s.split(',')
        .map(|p| {
            p.trim()
                .parse::<f64>()
                .map_err(|e| format!("bad joint '{p}': {e}"))
        })
        .collect()
}

/// A gripper opening, a fraction in `[0, 1]`.
fn fraction(s: &str) -> Result<f64, String> {
    let f: f64 = s
        .trim()
        .parse()
        .map_err(|e| format!("bad gripper fraction '{s}': {e}"))?;
    (0.0..=1.0)
        .contains(&f)
        .then_some(f)
        .ok_or(format!("gripper fraction must be in [0, 1], got {f}"))
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;
    let bundled = robot_description::model(&args.model)?;
    let description = bundled.description();
    let meshes = std::env::temp_dir().join(format!("self_collision_model-{}-meshes", bundled.id()));
    bundled
        .write_meshes_to(&meshes)
        .map_err(|e| format!("write the meshes to {}: {e}", meshes.display()))?;
    let meshes = meshes
        .to_str()
        .ok_or("the system's temporary directory is not UTF-8")?;
    on_shape(
        &description,
        Render {
            args: &args,
            description: &description,
            meshes,
        },
    )
    .map_err(|error| format!("the model {}: {error}", args.model))?
}

/// The scene of a robot at the shape of its arms.
struct Render<'a> {
    args: &'a Args,
    description: &'a RobotDescription,
    meshes: &'a str,
}

impl OnShape for Render<'_> {
    type Output = Result<(), Box<dyn std::error::Error>>;

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) -> Self::Output {
        render::<N, K>(self.args, self.description, self.meshes)
    }
}

/// The chain of `arm`, from its base link to its grasp point.
fn chain_spec(arm: &Arm) -> ChainSpec<'_> {
    ChainSpec {
        base_link: Some(&arm.base_link),
        tip_link: &arm.grasp_link,
        joints: JointSelection::PathOrder,
    }
}

/// The joint positions of `arm`: those the arguments give it, else its `home` state.
fn arm_joints<const N: usize>(
    args: &Args,
    description: &RobotDescription,
    arm: &Arm,
) -> Result<[f64; N], String> {
    let positions = match args.joints.iter().rfind(|(name, _)| *name == arm.name) {
        Some((_, positions)) => positions.as_slice(),
        None => description.state(&arm.name, states::HOME).ok_or(format!(
            "{} has no home state; give --joints {}=<positions>",
            arm.name, arm.name
        ))?,
    };
    positions.try_into().map_err(|_| {
        format!(
            "--joints {}= takes {N} positions, got {}",
            arm.name,
            positions.len()
        )
    })
}

/// The opening of the gripper of `arm`: the one the arguments give it, else that of
/// every gripper.
fn arm_opening(args: &Args, arm: &Arm) -> f64 {
    args.arm_grippers
        .iter()
        .rfind(|(name, _)| *name == arm.name)
        .map_or(args.gripper, |(_, opening)| *opening)
}

/// Writes the scene of the robot `description` describes, a robot of `K` arms of `N`
/// joints each, at the joint positions and the openings the arguments give.
fn render<const N: usize, const K: usize>(
    args: &Args,
    description: &RobotDescription,
    meshes: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let arms = description.arms();
    let unknown = args
        .joints
        .iter()
        .map(|(arm, _)| arm)
        .chain(args.arm_grippers.iter().map(|(arm, _)| arm))
        .find(|name| !arms.iter().any(|arm| arm.name == **name));
    if let Some(unknown) = unknown {
        let names: Vec<&str> = arms.iter().map(|arm| arm.name.as_str()).collect();
        return Err(format!(
            "the model {} has no arm '{unknown}'; name one of {}",
            args.model,
            names.join(", ")
        )
        .into());
    }
    let mut model = SelfCollisionModel::<N, K>::from_description(description, meshes)?;
    let postures = arms
        .iter()
        .map(|arm| arm_joints::<N>(args, description, arm))
        .collect::<Result<Vec<_>, _>>()?;
    let q: Joints<N, K> = postures
        .try_into()
        .map_err(|_| "the model holds a chain per arm")?;
    let openings: [f64; K] = std::array::from_fn(|k| arm_opening(args, &arms[k]));
    // Place the fingers at the requested opening; every query below then sees the
    // fingers where they actually are.
    model.set_gripper_openings(&openings);

    let chains: Vec<Chain<N>> = arms
        .iter()
        .map(|arm| Chain::from_urdf(description.robot(), &chain_spec(arm)))
        .collect::<Result<_, _>>()?;
    let chain_links: Vec<Vec<String>> = chains
        .iter()
        .map(|chain| {
            let posed = chain.at(&[0.0; N]);
            (0..N).map(|i| posed.link_name(i).to_string()).collect()
        })
        .collect();
    // The arm a body rides: a link of its chain or a finger of its gripper. None for a
    // fixed body.
    let arm_of = |body: &str| {
        arms.iter().zip(&chain_links).position(|(arm, links)| {
            links.iter().any(|link| link == body)
                || arm.gripper.fingers.iter().any(|finger| finger.link == body)
        })
    };

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
            let arm = arm_of(name);
            let hit = name == link_a || name == link_b;
            pieces
                .into_iter()
                .map(move |p| rounded_piece_json(&p, arm, hit))
                .collect::<Vec<_>>()
        })
        .collect();

    let wireframes = if args.wireframes {
        mesh_wireframes(description, meshes, &chains, &chain_links, &q, &openings)?
    } else {
        Vec::new()
    };

    let arm_names: Vec<&str> = arms.iter().map(|arm| arm.name.as_str()).collect();
    let data = serde_json::json!({
        "dStop": args.d_stop,
        "dSafe": args.d_safe,
        "distance": round(distance),
        "pair": format!("{link_a}  vs  {link_b}"),
        "witness": witness,
        "arms": arm_names,
        "bodies": bodies,
        "meshes": wireframes,
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
/// and STL pipeline the fit used. Each finger is drawn at its arm's opening, between
/// where its gripper's closed and open states put it, matching its live hull.
fn mesh_wireframes<const N: usize, const K: usize>(
    description: &RobotDescription,
    meshes: &str,
    chains: &[Chain<N>],
    chain_links: &[Vec<String>],
    q: &Joints<N, K>,
    openings: &[f64; K],
) -> Result<Vec<serde_json::Value>, String> {
    let urdf = UrdfCollisions::from_urdf(description.urdf()).map_err(|e| e.to_string())?;
    let on_a_chain = |link: &str| chain_links.iter().flatten().any(|name| name == link);
    let attached: Vec<String> = chain_links
        .iter()
        .flatten()
        .flat_map(|link| urdf.children_of(link))
        .filter(|child| !on_a_chain(child))
        .collect();

    let mut out = Vec::new();
    for name in urdf.collision_link_names() {
        if on_a_chain(&name) || attached.contains(&name) {
            continue;
        }
        let surfaces = urdf
            .fixed_surfaces_in_root(&name, meshes)
            .map_err(|e| e.to_string())?;
        out.push(wire_json(decimate(&surfaces.concat())));
    }
    for (((arm, chain), q), opening) in description.arms().iter().zip(chains).zip(q).zip(openings) {
        let posed = chain.at(q);
        for i in 0..N {
            let name = posed.link_name(i);
            let pose = posed.link_pose_world(i);
            let mut verts: Vec<Point3<f64>> = urdf
                .link_surfaces(name, meshes)?
                .concat()
                .iter()
                .map(|v| pose * v)
                .collect();
            // Fixed children ride with the link; movable children are the gripper's
            // fingers, each drawn where the arm's opening places its hull.
            for child in urdf.children_of(name) {
                if on_a_chain(&child) || urdf.collisions_of(&child).is_empty() {
                    continue;
                }
                let joint = urdf
                    .parent_joint(&child)
                    .expect("children_of implies a parent joint");
                let position = if joint.is_fixed() {
                    0.0
                } else {
                    let finger = arm
                        .gripper
                        .fingers
                        .iter()
                        .find(|finger| finger.link == child)
                        .ok_or(format!(
                            "'{child}' rides a moving joint and {} names no finger of it",
                            arm.gripper.name
                        ))?;
                    finger.closed + opening * (finger.open - finger.closed)
                };
                verts.extend(
                    urdf.child_surfaces_in_parent(&child, position, meshes)?
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
/// the spheres and cylinders. `arm` is the index of the arm whose chain the piece's
/// body rides, none for a fixed body.
fn rounded_piece_json(p: &Placed, arm: Option<usize>, hit: bool) -> serde_json::Value {
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
    serde_json::json!({ "arm": arm, "hit": hit, "radius": round(radius), "caps": caps, "verts": verts, "edges": edges })
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
const ARM_COLORS = [0x4f8fde, 0x53b97a, 0xb07fd8, 0xd8c35a];
const FIXED_COLOR = 0x8a8f9c;
const colorOf = arm => arm === null ? FIXED_COLOR : ARM_COLORS[arm % ARM_COLORS.length];
const css = color => '#' + color.toString(16).padStart(6, '0');
const UP = new THREE.Vector3(0, 1, 0);
// Each piece is its collision surface: faces offset outward by any radius (caps),
// cylinders along the edges, and spheres at the vertices fill the fillets.
for (const b of DATA.bodies) {
  const color = b.hit ? (DATA.distance <= 0 ? 0xe03c3c : 0xe2902f) : colorOf(b.arm);
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
  `<div class="legend">` +
  DATA.arms.map((name, arm) => `<span><i class="dot" style="background:${css(colorOf(arm))}"></i>${name}</span>`).join('') +
  `<span><i class="dot" style="background:${css(FIXED_COLOR)}"></i>fixed</span>` +
  `<span><i class="dot" style="background:#e2902f"></i>closest pair</span>` +
  `<span>solid = collision surface</span>` +
  `${DATA.meshes.length ? '<span>wireframe = source mesh</span>' : ''}</div>`;
addEventListener('resize', () => { camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script></body></html>
"##;
