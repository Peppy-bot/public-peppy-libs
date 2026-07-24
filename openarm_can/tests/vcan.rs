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
use std::time::{Duration, Instant};

use openarm_can::{
    ARM_MOTOR_TYPES, ARM_RECV_IDS, ArmCan, GripperCan, JointVec, Mit, PosForce, v10, v20,
};
use socketcan::id::FdFlags;
use socketcan::{
    CanAnyFrame, CanFdFrame, CanFdSocket, CanFrame, CanSocket, EmbeddedFrame, Frame, Socket,
    StandardId,
};

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

/// A full-scale state frame: q at +p_max, dq at -v_max, tau at +t_max. These
/// quantizer endpoints decode to the exact limit values.
const FULL_SCALE_STATE: [u8; 8] = [0x00, 0xFF, 0xFF, 0x00, 0x0F, 0xFF, 0x30, 0x28];

#[test]
fn state_frames_decode_into_arm_state() {
    let Some(iface) = test_iface() else {
        eprintln!("skipped: OPENARM_CAN_TEST_IFACE not set");
        return;
    };
    let _guard = BUS_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let mut arm = ArmCan::open(&iface, true).expect("open arm");
    let tx = CanFdSocket::open(&iface).expect("open sender");
    for &recv_id in &ARM_RECV_IDS {
        let id = StandardId::new(recv_id as u16).unwrap();
        let frame = CanFdFrame::with_flags(id, &FULL_SCALE_STATE, FdFlags::BRS).unwrap();
        tx.write_frame(&frame).expect("send state");
    }
    arm.recv_all(100_000).expect("recv_all");

    let state = arm.get_state();
    for (i, motor_type) in ARM_MOTOR_TYPES.iter().enumerate() {
        // Full-scale limits per motor model (Damiao MOTOR_LIMIT_PARAMS).
        let (v_max, t_max) = match motor_type {
            openarm_can::MotorType::DM8009 => (45.0, 54.0),
            openarm_can::MotorType::DM4340 => (8.0, 28.0),
            openarm_can::MotorType::DM4310 => (30.0, 10.0),
            other => panic!("unexpected arm motor type {other:?}"),
        };
        assert_eq!(state.positions[i], 12.5, "joint {i}");
        assert_eq!(state.velocities[i], -v_max, "joint {i}");
        assert_eq!(state.torques[i], t_max, "joint {i}");
    }
}

#[test]
fn classic_state_frame_decodes_on_fd_socket() {
    // The C++ implementation drops classic frames arriving on an FD socket;
    // the native driver accepts both read sizes.
    let Some(iface) = test_iface() else {
        eprintln!("skipped: OPENARM_CAN_TEST_IFACE not set");
        return;
    };
    let _guard = BUS_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let mut arm = ArmCan::open(&iface, true).expect("open arm");
    let tx = CanSocket::open(&iface).expect("open classic sender");
    let id = StandardId::new(ARM_RECV_IDS[0] as u16).unwrap();
    let frame = CanFrame::new(id, &FULL_SCALE_STATE).unwrap();
    tx.write_frame(&frame).expect("send state");
    arm.recv_all(100_000).expect("recv_all");

    assert_eq!(arm.get_state().positions[0], 12.5);
}

#[test]
fn gripper_open_drains_the_ctrl_mode_echo() {
    // The motor echoes a parameter write on the same recv id as state frames;
    // if the echo leaked into the state cache it would decode as a garbage
    // position (~-12.47 rad here). Opening must consume it.
    let Some(iface) = test_iface() else {
        eprintln!("skipped: OPENARM_CAN_TEST_IFACE not set");
        return;
    };
    let _guard = BUS_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let responder = CanFdSocket::open(&iface).expect("open responder");
    let echo_thread = std::thread::spawn(move || {
        // Wait for the control-mode write, then echo it back on the recv id.
        loop {
            let frame = responder
                .read_frame_timeout(Duration::from_secs(2))
                .expect("ctrl-mode write never arrived");
            let CanAnyFrame::Fd(f) = frame else { continue };
            if f.raw_id() == 0x7FF && f.data().get(2) == Some(&0x55) {
                let mut echo = [0u8; 8];
                echo.copy_from_slice(&f.data()[..8]);
                let id = StandardId::new(v20::GRIPPER_RECV_ID as u16).unwrap();
                let reply = CanFdFrame::with_flags(id, &echo, FdFlags::BRS).unwrap();
                responder.write_frame(&reply).expect("send echo");
                return;
            }
        }
    });

    let gripper = GripperCan::<PosForce>::open_pos_force(
        &iface,
        true,
        v20::GRIPPER_MOTOR_TYPE,
        v20::GRIPPER_SEND_ID,
        v20::GRIPPER_RECV_ID,
    )
    .expect("open gripper");
    echo_thread.join().expect("responder thread");

    let state = gripper.get_state();
    assert_eq!(
        state.position, 0.0,
        "param echo leaked into the state cache"
    );
    assert_eq!(state.velocity, 0.0);
    assert_eq!(state.torque, 0.0);
}

#[test]
fn recv_all_times_out_on_a_quiet_bus() {
    let Some(iface) = test_iface() else {
        eprintln!("skipped: OPENARM_CAN_TEST_IFACE not set");
        return;
    };
    let _guard = BUS_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let mut arm = ArmCan::open(&iface, true).expect("open arm");
    let start = Instant::now();
    arm.recv_all(50_000).expect("recv_all");
    let elapsed = start.elapsed();
    assert!(
        elapsed >= Duration::from_millis(40),
        "returned too early: {elapsed:?}"
    );
    assert!(elapsed < Duration::from_secs(1), "hung: {elapsed:?}");
}
