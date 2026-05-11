//! Rust wrapper around the openarm_can C++ library.
//!
//! [`ArmCan`] and [`GripperCan`] must be wrapped in `Arc<Mutex<_>>` for
//! cross-task sharing — they are `Send` but not `Sync`.

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
}

/// Hardware constants for the OpenArm v10 platform.
pub mod v10 {
    use super::MotorType;

    pub const ARM_DOF: usize = 7;
    pub const ARM_MOTOR_TYPES: [MotorType; ARM_DOF] = [
        MotorType::DM8009,
        MotorType::DM8009,
        MotorType::DM4340,
        MotorType::DM4340,
        MotorType::DM4310,
        MotorType::DM4310,
        MotorType::DM4310,
    ];
    pub const ARM_SEND_IDS: [u32; ARM_DOF] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
    pub const ARM_RECV_IDS: [u32; ARM_DOF] = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17];

    /// A fixed-length array of one `f64` per arm joint.
    pub type JointVec = [f64; ARM_DOF];
}

/// Damiao motor callback mode. Controls which CAN frames the firmware emits.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CallbackMode {
    State = 0,
    Param = 1,
    Ignore = 2,
}

/// State of all arm joints from the most recent `recv_all`.
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmState {
    pub positions: v10::JointVec,
    pub velocities: v10::JointVec,
    pub torques: v10::JointVec,
}

#[derive(Debug, thiserror::Error)]
pub enum CanError {
    #[error("CAN interface name '{0}' contains an interior NUL byte")]
    InvalidInterface(String),
    #[error("failed to open CAN interface '{0}'")]
    OpenFailed(String),
}

pub type Result<T> = std::result::Result<T, CanError>;

mod inner {
    #![allow(
        non_upper_case_globals,
        non_camel_case_types,
        non_snake_case,
        dead_code
    )]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

// Send: handle is a pointer to a heap-allocated C++ object backed by a SocketCAN fd; no
// thread-local state, so ownership can be transferred across threads. Sync is intentionally
// not impl'd: most methods are &mut self and get_state(&self) reads internal C++ state whose
// concurrent-read safety is not documented. Wrap in Arc<Mutex<_>> for cross-task sharing.
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
        motor_types: &[MotorType; v10::ARM_DOF],
        send_ids: &[u32; v10::ARM_DOF],
        recv_ids: &[u32; v10::ARM_DOF],
    ) {
        let types_u8: [u8; v10::ARM_DOF] = std::array::from_fn(|i| motor_types[i] as u8);
        unsafe {
            inner::openarm_init_arm_motors(
                self.0.handle,
                types_u8.as_ptr(),
                send_ids.as_ptr(),
                recv_ids.as_ptr(),
                v10::ARM_DOF as i32,
            );
        }
    }

    pub fn enable_all(&mut self) { self.0.enable_all() }
    pub fn disable_all(&mut self) { self.0.disable_all() }
    pub fn recv_all(&mut self, first_timeout_us: i32) { self.0.recv_all(first_timeout_us) }
    pub fn refresh_all(&mut self) { self.0.refresh_all() }
    pub fn set_callback_mode(&mut self, mode: CallbackMode) { self.0.set_callback_mode(mode) }

    pub fn mit_control(
        &mut self,
        kp: &v10::JointVec,
        kd: &v10::JointVec,
        q: &v10::JointVec,
        dq: &v10::JointVec,
        tau: &v10::JointVec,
    ) {
        unsafe {
            inner::openarm_arm_mit_control(
                self.0.handle,
                kp.as_ptr(),
                kd.as_ptr(),
                q.as_ptr(),
                dq.as_ptr(),
                tau.as_ptr(),
                v10::ARM_DOF as i32,
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
                v10::ARM_DOF as i32,
            );
        }
        state
    }
}
