//! Integration tests against a virtual CAN interface.
//!
//! Setup (once, requires root):
//! ```text
//! sudo modprobe vcan
//! sudo ip link add dev vcan0 mtu 72 type vcan
//! sudo ip link set up vcan0
//! ```
//! then run with `OPENARM_CAN_TEST_IFACE=vcan0 cargo test --test vcan`.
//! Without the environment variable every test skips, so CI needs no
//! privileges. The fixture-replay tests additionally skip until
//! `tests/fixtures/` holds captures from the C++-backed FFI build.

#[path = "common/sweep.rs"]
mod sweep;

use std::sync::Mutex;
use std::time::Duration;

use openarm_can::{ArmCan, GripperCan, JointVec, Mit, PosForce, v10, v20};
use socketcan::id::FdFlags;
use socketcan::{CanAnyFrame, CanFdSocket, EmbeddedFrame, Frame, Socket};

/// All tests share one bus, so they must not interleave traffic.
static BUS_LOCK: Mutex<()> = Mutex::new(());

fn test_iface() -> Option<String> {
    std::env::var("OPENARM_CAN_TEST_IFACE").ok()
}

/// Records every frame on the interface. Open it before the traffic starts:
/// a socket only receives frames sent after it subscribes.
struct Recorder(CanFdSocket);

impl Recorder {
    fn open(iface: &str) -> Self {
        Self(CanFdSocket::open(iface).expect("open recorder socket"))
    }

    /// Reads until the bus stays quiet for 200ms, returning fixture lines.
    fn drain_lines(&self) -> Vec<String> {
        let mut lines = Vec::new();
        while let Ok(frame) = self.0.read_frame_timeout(Duration::from_millis(200)) {
            let (fd, flags, id, data) = match &frame {
                CanAnyFrame::Normal(f) => (false, 0, f.raw_id(), f.data().to_vec()),
                CanAnyFrame::Fd(f) => (
                    true,
                    (f.flags() & (FdFlags::BRS | FdFlags::ESI)).bits(),
                    f.raw_id(),
                    f.data().to_vec(),
                ),
                CanAnyFrame::Remote(_) | CanAnyFrame::Error(_) => continue,
            };
            lines.push(sweep::format_frame(fd, flags, id, &data));
        }
        lines
    }
}

fn splat(value: f64) -> JointVec {
    [value; 7]
}

/// Drives the arm through the shared sweep. The capture harness runs the
/// FFI build through the identical sequence.
fn drive_arm_sweep(iface: &str) {
    let mut arm = ArmCan::open(iface, true).expect("open arm");
    arm.enable_all().expect("enable");
    for &(kp, kd, q, dq, tau) in sweep::ARM_MIT_SWEEP {
        arm.mit_control(&splat(kp), &splat(kd), &splat(q), &splat(dq), &splat(tau))
            .expect("mit_control");
    }
    let [kp, kd, q, dq, tau] = sweep::ARM_MIT_INDEXED;
    arm.mit_control(&kp, &kd, &q, &dq, &tau).expect("indexed");
    arm.disable_all().expect("disable");
}

fn drive_gripper_mit_sweep(iface: &str) {
    let mut gripper = GripperCan::<Mit>::open_mit(
        iface,
        true,
        v10::GRIPPER_MOTOR_TYPE,
        v10::GRIPPER_SEND_ID,
        v10::GRIPPER_RECV_ID,
    )
    .expect("open gripper");
    gripper.enable_all().expect("enable");
    for &(kp, kd, q, dq, tau) in sweep::GRIPPER_MIT_SWEEP {
        gripper.mit_control(kp, kd, q, dq, tau).expect("mit");
    }
    gripper.disable_all().expect("disable");
}

fn drive_gripper_pos_force_sweep(iface: &str) {
    let mut gripper = GripperCan::<PosForce>::open_pos_force(
        iface,
        true,
        v20::GRIPPER_MOTOR_TYPE,
        v20::GRIPPER_SEND_ID,
        v20::GRIPPER_RECV_ID,
    )
    .expect("open gripper");
    gripper.enable_all().expect("enable");
    for &(q, speed, torque) in sweep::POS_FORCE_SWEEP {
        gripper
            .set_position(q, speed, torque)
            .expect("set_position");
    }
    gripper.disable_all().expect("disable");
}

fn replay_against_fixture(fixture_name: &str, drive: fn(&str)) {
    let Some(iface) = test_iface() else {
        eprintln!("skipped: OPENARM_CAN_TEST_IFACE not set");
        return;
    };
    let path = format!(
        "{}/tests/fixtures/{fixture_name}",
        env!("CARGO_MANIFEST_DIR")
    );
    let Ok(fixture) = std::fs::read_to_string(&path) else {
        eprintln!("skipped: no fixture at {path} (run the capture harness first)");
        return;
    };
    let expected: Vec<&str> = fixture.lines().collect();

    let _guard = BUS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let recorder = Recorder::open(&iface);
    drive(&iface);
    let recorded = recorder.drain_lines();
    assert_eq!(recorded, expected, "TX frames diverge from the C++ capture");
}

#[test]
fn arm_sweep_matches_ffi_fixture() {
    replay_against_fixture("arm_mit.txt", drive_arm_sweep);
}

#[test]
fn gripper_mit_sweep_matches_ffi_fixture() {
    replay_against_fixture("gripper_mit.txt", drive_gripper_mit_sweep);
}

#[test]
fn gripper_pos_force_sweep_matches_ffi_fixture() {
    replay_against_fixture("gripper_pos_force.txt", drive_gripper_pos_force_sweep);
}
