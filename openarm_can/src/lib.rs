//! Pure-Rust driver for OpenArm's Damiao DM motors over Linux SocketCAN.
//!
//! [`ArmCan`] and [`GripperCan`] take `&mut self` for every bus operation;
//! wrap them in `Arc<Mutex<_>>` for cross-task sharing.
//!
//! The arm motor lineup and CAN addressing are identical across OpenArm v1.0 and v2.0
//! (same DM motors, same bus IDs), so they live once at the crate root ([`ARM_MOTOR_TYPES`],
//! [`ARM_SEND_IDS`], [`ARM_RECV_IDS`]). Only the gripper differs per generation: [`v10`]
//! is the prismatic parallel-jaw gripper (MIT position control); [`v20`] is the revolute
//! pinch gripper (POS_FORCE control with a commanded force limit).
//!
//! State readback is poll-based: a `recv_all` pass decodes pending state
//! frames into a cache that `get_state` snapshots.

mod bus;
mod protocol;

use std::marker::PhantomData;

use bus::{MotorBus, MotorSlot};
pub use protocol::MotorType;
use protocol::TorquePu;

/// Degrees of freedom of the arm. Both generations are 7-DOF SRS.
pub const ARM_DOF: usize = 7;

/// A fixed-length array of one `f64` per arm joint.
pub type JointVec = [f64; ARM_DOF];

/// Per-joint arm motor models, j1..j7. Identical across v1.0 and v2.0.
pub const ARM_MOTOR_TYPES: [MotorType; ARM_DOF] = [
    MotorType::DM8009,
    MotorType::DM8009,
    MotorType::DM4340,
    MotorType::DM4340,
    MotorType::DM4310,
    MotorType::DM4310,
    MotorType::DM4310,
];
/// Per-joint arm command (send) CAN ids. Identical across v1.0 and v2.0.
pub const ARM_SEND_IDS: [u32; ARM_DOF] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
/// Per-joint arm state (recv) CAN ids. Identical across v1.0 and v2.0.
pub const ARM_RECV_IDS: [u32; ARM_DOF] = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17];

/// Gripper hardware constants for the OpenArm v1.0 platform: a prismatic parallel-jaw
/// gripper on a single DM4310, run in MIT position control.
pub mod v10 {
    use super::MotorType;

    pub const GRIPPER_MOTOR_TYPE: MotorType = MotorType::DM4310;
    pub const GRIPPER_SEND_ID: u32 = 0x08;
    pub const GRIPPER_RECV_ID: u32 = 0x18;

    // joint=0 m (closed) ↔ motor=0 rad, joint=GRIPPER_OPEN_M ↔ motor=GRIPPER_OPEN_RAD.
    // Values match ROS2 openarm/v10_simple_hardware.
    pub const GRIPPER_OPEN_M: f64 = 0.044;
    #[allow(clippy::approx_constant)]
    pub const GRIPPER_OPEN_RAD: f64 = -1.0472;
}

/// Gripper hardware constants for the OpenArm v2.0 platform: a revolute pinch gripper on a
/// single DM4310, run in POS_FORCE control (position command with a speed + force limit).
pub mod v20 {
    use super::MotorType;

    pub const GRIPPER_MOTOR_TYPE: MotorType = MotorType::DM4310;
    pub const GRIPPER_SEND_ID: u32 = 0x08;
    pub const GRIPPER_RECV_ID: u32 = 0x18;

    // The gripper motor closes at 0 rad and opens toward GRIPPER_OPEN_RAD. This is the
    // motor-frame open angle used by enactic's POS_FORCE reference (test_gripper_posforce
    // commands 0..π/2); each finger joint travels π/2 rad (the right hand's URDF range
    // mirrored to -π/2..0), so the motor↔finger ratio is 1:1.
    pub const GRIPPER_OPEN_RAD: f64 = std::f64::consts::FRAC_PI_2;
}

/// Receive window for the motor's reply to a control-mode write during
/// gripper bring-up, matching the enactic reference timing (demo.cpp waits
/// 2000us after enable and parameter round-trips).
const CTRL_MODE_ECHO_TIMEOUT_US: u32 = 2000;

/// State of the gripper motor from the most recent `recv_all`.
#[derive(Debug, Clone, Copy, Default)]
pub struct GripperState {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
}

/// State of all arm joints from the most recent `recv_all`.
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmState {
    pub positions: JointVec,
    pub velocities: JointVec,
    pub torques: JointVec,
}

#[derive(Debug, thiserror::Error)]
pub enum CanError {
    #[error("failed to open CAN interface '{interface}'")]
    Open {
        interface: String,
        source: std::io::Error,
    },
    #[error("CAN I/O failed")]
    Io(#[from] std::io::Error),
    #[error("CAN id {0:#x} exceeds the 11-bit standard range")]
    InvalidCanId(u32),
    #[error("torque_pu must be per-unit in 0..=1, got {0}")]
    TorqueOutOfRange(f64),
}

pub type Result<T> = std::result::Result<T, CanError>;

/// 7-DOF arm on one CAN interface. Open with [`ArmCan::open`], then
/// `enable_all` before commanding.
pub struct ArmCan(MotorBus);

impl ArmCan {
    /// Opens `can_interface` and registers the seven arm motors
    /// ([`ARM_MOTOR_TYPES`] on [`ARM_SEND_IDS`] / [`ARM_RECV_IDS`]).
    pub fn open(can_interface: &str, enable_fd: bool) -> Result<Self> {
        let slots = (0..ARM_DOF)
            .map(|i| MotorSlot::new(ARM_MOTOR_TYPES[i], ARM_SEND_IDS[i], ARM_RECV_IDS[i], 0))
            .collect::<Result<Vec<_>>>()?;
        Ok(Self(MotorBus::open(can_interface, enable_fd, slots)?))
    }

    pub fn enable_all(&mut self) -> Result<()> {
        self.0.enable_all()
    }

    pub fn disable_all(&mut self) -> Result<()> {
        self.0.disable_all()
    }

    /// Receives pending state frames into the cache: waits up to
    /// `first_timeout_us` for the first frame, then drains without waiting.
    pub fn recv_all(&mut self, first_timeout_us: u32) -> Result<()> {
        self.0.recv_all(first_timeout_us)
    }

    /// Receives and discards pending frames (bring-up replies that must not
    /// land in the state cache).
    pub fn drain(&mut self, first_timeout_us: u32) -> Result<()> {
        self.0.drain(first_timeout_us)
    }

    /// MIT-mode command to all joints: PD to `q`/`dq` plus feedforward `tau`.
    pub fn mit_control(
        &mut self,
        kp: &JointVec,
        kd: &JointVec,
        q: &JointVec,
        dq: &JointVec,
        tau: &JointVec,
    ) -> Result<()> {
        for i in 0..ARM_DOF {
            let frame = protocol::mit_frame(
                ARM_MOTOR_TYPES[i],
                ARM_SEND_IDS[i],
                kp[i],
                kd[i],
                q[i],
                dq[i],
                tau[i],
            );
            self.0.send(&frame)?;
        }
        Ok(())
    }

    /// Snapshot of joint state from the most recent [`recv_all`](Self::recv_all).
    pub fn get_state(&self) -> ArmState {
        let mut state = ArmState::default();
        for (i, slot) in self.0.slots().iter().enumerate() {
            let motor = slot.state();
            state.positions[i] = motor.position;
            state.velocities[i] = motor.velocity;
            state.torques[i] = motor.torque;
        }
        state
    }
}

/// Marker for the MIT control mode (v1.0 prismatic gripper).
pub enum Mit {}
/// Marker for the POS_FORCE control mode (v2.0 pinch gripper).
pub enum PosForce {}

mod sealed {
    use crate::protocol::ControlMode;

    pub trait Sealed {
        const CONTROL_MODE: ControlMode;
    }
    impl Sealed for super::Mit {
        const CONTROL_MODE: ControlMode = ControlMode::Mit;
    }
    impl Sealed for super::PosForce {
        const CONTROL_MODE: ControlMode = ControlMode::PosForce;
    }
}

/// Gripper control mode, fixed at open time: [`Mit`] or [`PosForce`].
pub trait Mode: sealed::Sealed {}
impl Mode for Mit {}
impl Mode for PosForce {}

/// 1-DOF gripper on one CAN interface. The control mode is part of the type:
/// open with [`GripperCan::open_mit`] (v1.0) or
/// [`GripperCan::open_pos_force`] (v2.0), then `enable_all` before
/// commanding. Opening writes the motor's control-mode parameter and consumes
/// the reply, so the bus is quiet when it returns.
pub struct GripperCan<M: Mode> {
    bus: MotorBus,
    _mode: PhantomData<M>,
}

impl GripperCan<Mit> {
    /// Opens the gripper motor in MIT mode; command it with
    /// [`mit_control`](Self::mit_control).
    pub fn open_mit(
        can_interface: &str,
        enable_fd: bool,
        motor_type: MotorType,
        send_id: u32,
        recv_id: u32,
    ) -> Result<Self> {
        Self::open(can_interface, enable_fd, motor_type, send_id, recv_id, 0)
    }

    /// MIT-mode command: PD to `q`/`dq` plus feedforward `tau`.
    pub fn mit_control(&mut self, kp: f64, kd: f64, q: f64, dq: f64, tau: f64) -> Result<()> {
        let slot = &self.bus.slots()[0];
        let frame = protocol::mit_frame(slot.motor_type(), slot.send_id(), kp, kd, q, dq, tau);
        self.bus.send(&frame)
    }
}

impl GripperCan<PosForce> {
    /// Opens the gripper motor in POS_FORCE mode; command it with
    /// [`set_position`](Self::set_position).
    pub fn open_pos_force(
        can_interface: &str,
        enable_fd: bool,
        motor_type: MotorType,
        send_id: u32,
        recv_id: u32,
    ) -> Result<Self> {
        Self::open(
            can_interface,
            enable_fd,
            motor_type,
            send_id,
            recv_id,
            protocol::POS_FORCE_ID_OFFSET,
        )
    }

    /// POS_FORCE-mode command: drive to motor angle `q_rad` with an absolute
    /// speed limit `speed_rad_s` (`0..=100` rad/s, clamped) and a
    /// torque-current limit `torque_pu` (per-unit, `0..=1`; rejected outside
    /// that range). The commanded force is the grip force cap; measured
    /// torque comes back via [`get_state`](Self::get_state).
    pub fn set_position(&mut self, q_rad: f64, speed_rad_s: f64, torque_pu: f64) -> Result<()> {
        let torque = TorquePu::new(torque_pu)?;
        let send_id = self.bus.slots()[0].send_id();
        let frame = protocol::pos_force_frame(send_id, q_rad, speed_rad_s, torque);
        self.bus.send(&frame)
    }
}

impl<M: Mode> GripperCan<M> {
    fn open(
        can_interface: &str,
        enable_fd: bool,
        motor_type: MotorType,
        send_id: u32,
        recv_id: u32,
        extra_send_offset: u32,
    ) -> Result<Self> {
        let slot = MotorSlot::new(motor_type, send_id, recv_id, extra_send_offset)?;
        let mut bus = MotorBus::open(can_interface, enable_fd, vec![slot])?;
        bus.set_control_mode(M::CONTROL_MODE)?;
        bus.drain(CTRL_MODE_ECHO_TIMEOUT_US)?;
        Ok(Self {
            bus,
            _mode: PhantomData,
        })
    }

    pub fn enable_all(&mut self) -> Result<()> {
        self.bus.enable_all()
    }

    pub fn disable_all(&mut self) -> Result<()> {
        self.bus.disable_all()
    }

    /// Receives pending state frames into the cache: waits up to
    /// `first_timeout_us` for the first frame, then drains without waiting.
    pub fn recv_all(&mut self, first_timeout_us: u32) -> Result<()> {
        self.bus.recv_all(first_timeout_us)
    }

    /// Receives and discards pending frames (bring-up replies that must not
    /// land in the state cache).
    pub fn drain(&mut self, first_timeout_us: u32) -> Result<()> {
        self.bus.drain(first_timeout_us)
    }

    /// Snapshot of gripper state from the most recent
    /// [`recv_all`](Self::recv_all). The `torque` field is the measured grip
    /// force feedback.
    pub fn get_state(&self) -> GripperState {
        let motor = self.bus.slots()[0].state();
        GripperState {
            position: motor.position,
            velocity: motor.velocity,
            torque: motor.torque,
        }
    }
}

#[cfg(test)]
#[allow(clippy::assertions_on_constants)]
mod tests {
    use super::*;

    #[test]
    fn arm_arrays_have_consistent_length() {
        assert_eq!(ARM_MOTOR_TYPES.len(), ARM_DOF);
        assert_eq!(ARM_SEND_IDS.len(), ARM_DOF);
        assert_eq!(ARM_RECV_IDS.len(), ARM_DOF);
    }

    #[test]
    fn v1_gripper_mapping_signs_oppose() {
        // Linear mapping: 0 m → 0 rad, GRIPPER_OPEN_M → GRIPPER_OPEN_RAD. The open
        // direction is negative in the motor frame; a sign flip sends the gripper
        // the wrong way.
        assert!(v10::GRIPPER_OPEN_M > 0.0);
        assert!(v10::GRIPPER_OPEN_RAD < 0.0);
    }

    #[test]
    fn v2_gripper_opens_positive() {
        // The v2 pinch gripper closes at 0 rad and opens toward a positive motor angle.
        assert!(v20::GRIPPER_OPEN_RAD > 0.0);
    }

    #[test]
    fn gripper_can_ids_do_not_collide_with_arm() {
        for (send, recv) in [
            (v10::GRIPPER_SEND_ID, v10::GRIPPER_RECV_ID),
            (v20::GRIPPER_SEND_ID, v20::GRIPPER_RECV_ID),
        ] {
            assert!(!ARM_SEND_IDS.contains(&send));
            assert!(!ARM_RECV_IDS.contains(&recv));
        }
    }
}
