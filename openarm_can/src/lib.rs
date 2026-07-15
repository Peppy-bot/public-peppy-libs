//! Rust wrapper around the openarm_can C++ library.
//!
//! [`ArmCan`] and [`GripperCan`] must be wrapped in `Arc<Mutex<_>>` for
//! cross-task sharing — they are `Send` but not `Sync`.
//!
//! The arm motor lineup and CAN addressing are identical across OpenArm v1.0 and v2.0
//! (same DM motors, same bus IDs), so they live once at the crate root ([`ARM_MOTOR_TYPES`],
//! [`ARM_SEND_IDS`], [`ARM_RECV_IDS`]). Only the gripper differs per generation: [`v10`]
//! is the prismatic parallel-jaw gripper (MIT position control); [`v20`] is the revolute
//! pinch gripper (POS_FORCE control with a commanded force limit).

use std::ffi::CString;

/// Damiao motor model; mirrors `openarm::damiao_motor::MotorType` — do not reorder.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorType {
    DM3507 = 0,
    DM4310 = 1,
    DM4310_48V = 2,
    DM4340 = 3,
    DM4340_48V = 4,
    DM6006 = 5,
    DM8006 = 6,
    DM8009 = 7,
    DM10010L = 8,
    DM10010 = 9,
    DMH3510 = 10,
    DMH6215 = 11,
    DMG6220 = 12,
}

/// Damiao motor control mode; mirrors `openarm::damiao_motor::ControlMode`; the values
/// are the on-wire mode ids, do not renumber. The arm and the v1 gripper run [`Mit`];
/// the v2 pinch gripper runs [`PosForce`] (position command with a speed + force limit).
///
/// [`Mit`]: ControlMode::Mit
/// [`PosForce`]: ControlMode::PosForce
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Mit = 1,
    PosVel = 2,
    Vel = 3,
    PosForce = 4,
}

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
    // commands 0..π/2); the URDF finger joints travel the same 0..π/2 rad, so the
    // motor↔finger ratio is 1:1.
    #[allow(clippy::approx_constant)]
    pub const GRIPPER_OPEN_RAD: f64 = std::f64::consts::FRAC_PI_2;
}

/// Damiao motor callback mode. Controls which CAN frames the firmware emits.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CallbackMode {
    State = 0,
    Param = 1,
    Ignore = 2,
}

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
    #[error("CAN interface name '{0}' contains an interior NUL byte")]
    InvalidInterface(String),
    #[error("failed to open CAN interface '{0}'")]
    OpenFailed(String),
}

pub type Result<T> = std::result::Result<T, CanError>;

// FFI to the openarm_can C++ wrapper (see wrapper.h). The surface is small and uses only
// primitive types, so the `extern "C"` block is declared by hand rather than via bindgen.
// build.rs compiles wrapper.cpp and sets `openarm_sdk` when the C++ SDK is present.
#[cfg(openarm_sdk)]
mod inner {
    use std::os::raw::{c_char, c_void};

    pub type OpenArmHandle = *mut c_void;

    unsafe extern "C" {
        pub fn openarm_create(can_interface: *const c_char, enable_fd: bool) -> OpenArmHandle;
        pub fn openarm_destroy(h: OpenArmHandle);
        pub fn openarm_init_arm_motors(
            h: OpenArmHandle,
            motor_types: *const u8,
            send_can_ids: *const u32,
            recv_can_ids: *const u32,
            count: i32,
        );
        pub fn openarm_enable_all(h: OpenArmHandle);
        pub fn openarm_disable_all(h: OpenArmHandle);
        pub fn openarm_recv_all(h: OpenArmHandle, first_timeout_us: i32);
        pub fn openarm_refresh_all(h: OpenArmHandle);
        pub fn openarm_set_callback_mode_all(h: OpenArmHandle, mode: i32);
        pub fn openarm_arm_mit_control(
            h: OpenArmHandle,
            kp: *const f64,
            kd: *const f64,
            q: *const f64,
            dq: *const f64,
            tau: *const f64,
            count: i32,
        );
        pub fn openarm_arm_get_state(
            h: OpenArmHandle,
            positions: *mut f64,
            velocities: *mut f64,
            torques: *mut f64,
            count: i32,
        );
        pub fn openarm_init_gripper_motor(
            h: OpenArmHandle,
            motor_type: u8,
            send_can_id: u32,
            recv_can_id: u32,
        );
        pub fn openarm_init_gripper_motor_mode(
            h: OpenArmHandle,
            motor_type: u8,
            send_can_id: u32,
            recv_can_id: u32,
            control_mode: u8,
        );
        pub fn openarm_gripper_mit_control(
            h: OpenArmHandle,
            kp: f64,
            kd: f64,
            q: f64,
            dq: f64,
            tau: f64,
        );
        pub fn openarm_gripper_pos_force_control(h: OpenArmHandle, q: f64, dq: f64, i: f64);
        pub fn openarm_gripper_get_state(
            h: OpenArmHandle,
            position: *mut f64,
            velocity: *mut f64,
            torque: *mut f64,
        );
    }
}

// Stub FFI used when the openarm_can C++ SDK is absent (dev machines / CI without
// the hardware library; see build.rs). `openarm_create` returns null so
// `CanHandle::new` fails with `CanError::OpenFailed`, which makes the other entry
// points unreachable. They exist only so the crate links and the pure-Rust API
// and tests compile. Build against the real SDK for hardware support. Signatures
// mirror the `openarm_sdk` extern block above so the wrapper code compiles against both.
#[cfg(not(openarm_sdk))]
mod inner {
    #![allow(dead_code, unused_variables)]
    use std::os::raw::{c_char, c_void};

    pub type OpenArmHandle = *mut c_void;

    pub unsafe fn openarm_create(_iface: *const c_char, _enable_fd: bool) -> OpenArmHandle {
        std::ptr::null_mut()
    }
    pub unsafe fn openarm_destroy(h: OpenArmHandle) {}
    pub unsafe fn openarm_enable_all(h: OpenArmHandle) {}
    pub unsafe fn openarm_disable_all(h: OpenArmHandle) {}
    pub unsafe fn openarm_recv_all(h: OpenArmHandle, first_timeout_us: i32) {}
    pub unsafe fn openarm_refresh_all(h: OpenArmHandle) {}
    pub unsafe fn openarm_set_callback_mode_all(h: OpenArmHandle, mode: i32) {}
    pub unsafe fn openarm_init_arm_motors(
        h: OpenArmHandle,
        motor_types: *const u8,
        send_can_ids: *const u32,
        recv_can_ids: *const u32,
        count: i32,
    ) {
    }
    pub unsafe fn openarm_arm_mit_control(
        h: OpenArmHandle,
        kp: *const f64,
        kd: *const f64,
        q: *const f64,
        dq: *const f64,
        tau: *const f64,
        count: i32,
    ) {
    }
    pub unsafe fn openarm_arm_get_state(
        h: OpenArmHandle,
        positions: *mut f64,
        velocities: *mut f64,
        torques: *mut f64,
        count: i32,
    ) {
    }
    pub unsafe fn openarm_init_gripper_motor(
        h: OpenArmHandle,
        motor_type: u8,
        send_can_id: u32,
        recv_can_id: u32,
    ) {
    }
    pub unsafe fn openarm_init_gripper_motor_mode(
        h: OpenArmHandle,
        motor_type: u8,
        send_can_id: u32,
        recv_can_id: u32,
        control_mode: u8,
    ) {
    }
    pub unsafe fn openarm_gripper_mit_control(
        h: OpenArmHandle,
        kp: f64,
        kd: f64,
        q: f64,
        dq: f64,
        tau: f64,
    ) {
    }
    pub unsafe fn openarm_gripper_pos_force_control(h: OpenArmHandle, q: f64, dq: f64, i: f64) {}
    pub unsafe fn openarm_gripper_get_state(
        h: OpenArmHandle,
        position: *mut f64,
        velocity: *mut f64,
        torque: *mut f64,
    ) {
    }
}

// SAFETY: Verified by inspection of enactic/openarm_can: CANSocket wraps a plain
// socket_fd_ (int) with no thread affinity; all other fields are heap-allocated
// (std::unique_ptr / std::vector / std::map); the library has no thread_local storage
// and no static mutable state. Transferring ownership across threads is safe.
// Sync is intentionally not impl'd: mutable motor state is read back without any
// synchronisation. Wrap in Arc<Mutex<_>> for cross-task sharing.
struct CanHandle {
    handle: inner::OpenArmHandle,
}

unsafe impl Send for CanHandle {}

impl CanHandle {
    fn new(can_interface: &str, enable_fd: bool) -> Result<Self> {
        let iface = CString::new(can_interface)
            .map_err(|_| CanError::InvalidInterface(can_interface.to_owned()))?;
        let handle = unsafe { inner::openarm_create(iface.as_ptr(), enable_fd) };
        if handle.is_null() {
            return Err(CanError::OpenFailed(can_interface.to_owned()));
        }
        Ok(Self { handle })
    }

    fn enable_all(&mut self) {
        unsafe { inner::openarm_enable_all(self.handle) }
    }

    fn disable_all(&mut self) {
        unsafe { inner::openarm_disable_all(self.handle) }
    }

    fn recv_all(&mut self, first_timeout_us: i32) {
        unsafe { inner::openarm_recv_all(self.handle, first_timeout_us) }
    }

    fn refresh_all(&mut self) {
        unsafe { inner::openarm_refresh_all(self.handle) }
    }

    fn set_callback_mode(&mut self, mode: CallbackMode) {
        unsafe { inner::openarm_set_callback_mode_all(self.handle, mode as i32) }
    }
}

impl Drop for CanHandle {
    fn drop(&mut self) {
        unsafe { inner::openarm_destroy(self.handle) }
    }
}

/// 7 DOF arm. Open with [`ArmCan::new`].
pub struct ArmCan(CanHandle);

impl ArmCan {
    pub fn new(can_interface: &str, enable_fd: bool) -> Result<Self> {
        Ok(Self(CanHandle::new(can_interface, enable_fd)?))
    }

    pub fn init_motors(
        &mut self,
        motor_types: &[MotorType; ARM_DOF],
        send_ids: &[u32; ARM_DOF],
        recv_ids: &[u32; ARM_DOF],
    ) {
        let types_u8: [u8; ARM_DOF] = std::array::from_fn(|i| motor_types[i] as u8);
        unsafe {
            inner::openarm_init_arm_motors(
                self.0.handle,
                types_u8.as_ptr(),
                send_ids.as_ptr(),
                recv_ids.as_ptr(),
                ARM_DOF as i32,
            );
        }
    }

    pub fn enable_all(&mut self) {
        self.0.enable_all()
    }
    pub fn disable_all(&mut self) {
        self.0.disable_all()
    }
    pub fn recv_all(&mut self, first_timeout_us: i32) {
        self.0.recv_all(first_timeout_us)
    }
    pub fn refresh_all(&mut self) {
        self.0.refresh_all()
    }
    pub fn set_callback_mode(&mut self, mode: CallbackMode) {
        self.0.set_callback_mode(mode)
    }

    pub fn mit_control(
        &mut self,
        kp: &JointVec,
        kd: &JointVec,
        q: &JointVec,
        dq: &JointVec,
        tau: &JointVec,
    ) {
        unsafe {
            inner::openarm_arm_mit_control(
                self.0.handle,
                kp.as_ptr(),
                kd.as_ptr(),
                q.as_ptr(),
                dq.as_ptr(),
                tau.as_ptr(),
                ARM_DOF as i32,
            );
        }
    }

    /// Snapshot of joint state from the most recent `recv_all`.
    /// Calls `std::abort` (via C++) if [`init_motors`](Self::init_motors) has not been called.
    pub fn get_state(&self) -> ArmState {
        let mut state = ArmState::default();
        unsafe {
            inner::openarm_arm_get_state(
                self.0.handle,
                state.positions.as_mut_ptr(),
                state.velocities.as_mut_ptr(),
                state.torques.as_mut_ptr(),
                ARM_DOF as i32,
            );
        }
        state
    }
}

/// 1 DOF gripper. Open with [`GripperCan::new`], then initialise the motor for the control
/// mode this generation uses: [`init_motor`](Self::init_motor) (MIT, v1.0) or
/// [`init_motor_pos_force`](Self::init_motor_pos_force) (POS_FORCE, v2.0). The handle
/// remembers the initialised mode and asserts each command matches it, so a frame can
/// never be sent in a protocol the motor is not configured for.
pub struct GripperCan {
    handle: CanHandle,
    mode: Option<ControlMode>,
}

impl GripperCan {
    pub fn new(can_interface: &str, enable_fd: bool) -> Result<Self> {
        Ok(Self {
            handle: CanHandle::new(can_interface, enable_fd)?,
            mode: None,
        })
    }

    /// Initialise the gripper motor in MIT mode (the v1.0 prismatic gripper).
    /// Command it with [`mit_control`](Self::mit_control).
    pub fn init_motor(&mut self, motor_type: MotorType, send_id: u32, recv_id: u32) {
        unsafe {
            inner::openarm_init_gripper_motor(
                self.handle.handle,
                motor_type as u8,
                send_id,
                recv_id,
            );
        }
        self.mode = Some(ControlMode::Mit);
    }

    /// Initialise the gripper motor in POS_FORCE mode (the v2.0 revolute pinch gripper).
    /// Command it with [`set_position`](Self::set_position).
    pub fn init_motor_pos_force(&mut self, motor_type: MotorType, send_id: u32, recv_id: u32) {
        unsafe {
            inner::openarm_init_gripper_motor_mode(
                self.handle.handle,
                motor_type as u8,
                send_id,
                recv_id,
                ControlMode::PosForce as u8,
            );
        }
        self.mode = Some(ControlMode::PosForce);
    }

    pub fn enable_all(&mut self) {
        self.handle.enable_all()
    }
    pub fn disable_all(&mut self) {
        self.handle.disable_all()
    }
    pub fn recv_all(&mut self, first_timeout_us: i32) {
        self.handle.recv_all(first_timeout_us)
    }
    pub fn refresh_all(&mut self) {
        self.handle.refresh_all()
    }
    pub fn set_callback_mode(&mut self, mode: CallbackMode) {
        self.handle.set_callback_mode(mode)
    }

    /// MIT-mode command (v1.0 prismatic gripper): PD to `q`/`dq` plus feedforward `tau`.
    /// Asserts the motor was initialised with [`init_motor`](Self::init_motor).
    pub fn mit_control(&mut self, kp: f64, kd: f64, q: f64, dq: f64, tau: f64) {
        assert_eq!(
            self.mode,
            Some(ControlMode::Mit),
            "mit_control requires init_motor (MIT mode) first"
        );
        unsafe { inner::openarm_gripper_mit_control(self.handle.handle, kp, kd, q, dq, tau) }
    }

    /// POS_FORCE-mode command (v2.0 pinch gripper): drive to motor angle `q_rad` with an
    /// absolute speed limit `speed_rad_s` and a torque-current limit `torque_pu`
    /// (per-unit, 0..=1; asserted). The commanded force is the grip force cap; measured
    /// torque comes back via [`get_state`](Self::get_state). Asserts the motor was
    /// initialised with [`init_motor_pos_force`](Self::init_motor_pos_force).
    pub fn set_position(&mut self, q_rad: f64, speed_rad_s: f64, torque_pu: f64) {
        assert_eq!(
            self.mode,
            Some(ControlMode::PosForce),
            "set_position requires init_motor_pos_force (POS_FORCE mode) first"
        );
        assert!(
            (0.0..=1.0).contains(&torque_pu),
            "torque_pu must be per-unit in 0..=1, got {torque_pu}"
        );
        unsafe {
            inner::openarm_gripper_pos_force_control(
                self.handle.handle,
                q_rad,
                speed_rad_s,
                torque_pu,
            )
        }
    }

    /// Snapshot of gripper state (position, velocity, torque) from the most recent
    /// `recv_all`. The `torque` field is the measured grip force feedback.
    /// Calls `std::abort` (via C++) if the motor has not been initialised.
    pub fn get_state(&self) -> GripperState {
        let mut state = GripperState::default();
        unsafe {
            inner::openarm_gripper_get_state(
                self.handle.handle,
                &mut state.position,
                &mut state.velocity,
                &mut state.torque,
            );
        }
        state
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

    #[test]
    fn control_mode_wire_ids_match_the_firmware() {
        assert_eq!(ControlMode::Mit as u8, 1);
        assert_eq!(ControlMode::PosForce as u8, 4);
    }
}
