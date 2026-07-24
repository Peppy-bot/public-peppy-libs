//! Damiao DM motor wire protocol: pure frame encode/decode, no I/O.
//!
//! Byte layouts and scaling follow the Damiao firmware conventions (reference:
//! enactic/openarm_can 1.2.8). MIT command and state frames are big-endian
//! nibble-packed with truncating quantization over `2^bits - 1` steps; POS_FORCE
//! commands and parameter writes are little-endian.

use crate::{CanError, Result};

/// Fixed CAN id for parameter reads/writes; the target motor's send id is
/// embedded little-endian in the first two payload bytes.
const PARAM_CAN_ID: u32 = 0x7FF;

/// POS_FORCE commands go to the motor's send id plus this offset.
pub(crate) const POS_FORCE_ID_OFFSET: u32 = 0x300;

/// Register id of the control-mode parameter (Damiao `RID::CTRL_MODE`).
const RID_CTRL_MODE: u8 = 0x0A;

const CMD_ENABLE: u8 = 0xFC;
const CMD_DISABLE: u8 = 0xFD;

/// Damiao motor model, used to select the scaling limits for MIT command and
/// state frames. The model itself never goes on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorType {
    DM3507,
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
}

/// Per-model full-scale ranges: position +-`p_max` rad, velocity +-`v_max`
/// rad/s, torque +-`t_max` Nm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Limits {
    pub p_max: f64,
    pub v_max: f64,
    pub t_max: f64,
}

impl MotorType {
    pub(crate) const fn limits(self) -> Limits {
        let (p_max, v_max, t_max) = match self {
            Self::DM3507 => (12.5, 50.0, 5.0),
            Self::DM4310 => (12.5, 30.0, 10.0),
            Self::DM4310_48V => (12.5, 50.0, 10.0),
            Self::DM4340 => (12.5, 8.0, 28.0),
            Self::DM4340_48V => (12.5, 10.0, 28.0),
            Self::DM6006 => (12.5, 45.0, 20.0),
            Self::DM8006 => (12.5, 45.0, 40.0),
            Self::DM8009 => (12.5, 45.0, 54.0),
            Self::DM10010L => (12.5, 25.0, 200.0),
            Self::DM10010 => (12.5, 20.0, 200.0),
            Self::DMH3510 => (12.5, 280.0, 1.0),
            Self::DMH6215 => (12.5, 45.0, 10.0),
            Self::DMG6220 => (12.5, 45.0, 10.0),
        };
        Limits {
            p_max,
            v_max,
            t_max,
        }
    }
}

/// Damiao control mode; the values are the on-wire ids written to `CTRL_MODE`.
/// `pub` only to satisfy the sealed `Mode` trait's associated const; the
/// private `protocol` module keeps it out of the crate API.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Mit = 1,
    PosForce = 4,
}

/// Torque-current limit as a per-unit value in `0..=1` (actual current divided
/// by the motor's max current). Constructing it is the range check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct TorquePu(f64);

impl TorquePu {
    pub fn new(value: f64) -> Result<Self> {
        if !(0.0..=1.0).contains(&value) {
            return Err(CanError::TorqueOutOfRange(value));
        }
        Ok(Self(value))
    }
}

/// One decoded motor state frame.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct MotorState {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
}

/// A command frame ready to write: all Damiao commands carry exactly 8 bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct OutFrame {
    pub id: u32,
    pub data: [u8; 8],
}

pub(crate) fn enable_frame(send_id: u32) -> OutFrame {
    command_frame(send_id, CMD_ENABLE)
}

pub(crate) fn disable_frame(send_id: u32) -> OutFrame {
    command_frame(send_id, CMD_DISABLE)
}

fn command_frame(send_id: u32, cmd: u8) -> OutFrame {
    OutFrame {
        id: send_id,
        data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd],
    }
}

/// MIT-mode command: PD to `q`/`dq` with gains `kp`/`kd` plus feedforward `tau`.
/// Values are clamped to the motor's full-scale ranges (kp `0..=500`, kd `0..=5`).
pub(crate) fn mit_frame(
    motor_type: MotorType,
    send_id: u32,
    kp: f64,
    kd: f64,
    q: f64,
    dq: f64,
    tau: f64,
) -> OutFrame {
    let lim = motor_type.limits();
    let kp_u = quantize(kp, 0.0, 500.0, 12);
    let kd_u = quantize(kd, 0.0, 5.0, 12);
    let q_u = quantize(q, -lim.p_max, lim.p_max, 16);
    let dq_u = quantize(dq, -lim.v_max, lim.v_max, 12);
    let tau_u = quantize(tau, -lim.t_max, lim.t_max, 12);
    OutFrame {
        id: send_id,
        data: [
            (q_u >> 8) as u8,
            (q_u & 0xFF) as u8,
            (dq_u >> 4) as u8,
            (((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF)) as u8,
            (kp_u & 0xFF) as u8,
            (kd_u >> 4) as u8,
            (((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF)) as u8,
            (tau_u & 0xFF) as u8,
        ],
    }
}

/// POS_FORCE command: drive to `q_rad` under a speed limit (`0..=100` rad/s,
/// clamped) and the torque-current limit.
pub(crate) fn pos_force_frame(
    send_id: u32,
    q_rad: f64,
    speed_rad_s: f64,
    torque: TorquePu,
) -> OutFrame {
    let pos = (q_rad as f32).to_le_bytes();
    let speed_u = (speed_rad_s.clamp(0.0, 100.0) * 100.0) as u16;
    let torque_u = (torque.0 * 10000.0) as u16;
    let [speed_lo, speed_hi] = speed_u.to_le_bytes();
    let [torque_lo, torque_hi] = torque_u.to_le_bytes();
    OutFrame {
        id: send_id + POS_FORCE_ID_OFFSET,
        data: [
            pos[0], pos[1], pos[2], pos[3], speed_lo, speed_hi, torque_lo, torque_hi,
        ],
    }
}

/// Parameter write setting the motor's control mode.
pub(crate) fn ctrl_mode_frame(send_id: u32, mode: ControlMode) -> OutFrame {
    OutFrame {
        id: PARAM_CAN_ID,
        data: [
            (send_id & 0xFF) as u8,
            ((send_id >> 8) & 0xFF) as u8,
            0x55,
            RID_CTRL_MODE,
            mode as u8,
            0,
            0,
            0,
        ],
    }
}

/// Decode a motor state frame. Byte 0 (controller id / error nibble) is not
/// interpreted; bytes 6-7 (temperatures) are not exposed. Returns `None` for
/// payloads shorter than 8 bytes.
pub(crate) fn parse_state(motor_type: MotorType, data: &[u8]) -> Option<MotorState> {
    if data.len() < 8 {
        return None;
    }
    let lim = motor_type.limits();
    let q_u = (u16::from(data[1]) << 8) | u16::from(data[2]);
    let dq_u = (u16::from(data[3]) << 4) | (u16::from(data[4]) >> 4);
    let tau_u = (u16::from(data[4] & 0xF) << 8) | u16::from(data[5]);
    Some(MotorState {
        position: dequantize(q_u, -lim.p_max, lim.p_max, 16),
        velocity: dequantize(dq_u, -lim.v_max, lim.v_max, 12),
        torque: dequantize(tau_u, -lim.t_max, lim.t_max, 12),
    })
}

/// Clamp `x` to `[min, max]`, then map linearly onto `0..=2^bits - 1`,
/// truncating toward zero (firmware convention; not rounded).
fn quantize(x: f64, min: f64, max: f64, bits: u32) -> u16 {
    let norm = (x.clamp(min, max) - min) / (max - min);
    (norm * f64::from((1u32 << bits) - 1)) as u16
}

fn dequantize(x: u16, min: f64, max: f64, bits: u32) -> f64 {
    f64::from(x) / f64::from((1u32 << bits) - 1) * (max - min) + min
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantize_truncates_instead_of_rounding() {
        // DM4310 position span: (1.0 + 12.5) / 25.0 * 65535 = 35388.9; rounding
        // would give 35389.
        assert_eq!(quantize(1.0, -12.5, 12.5, 16), 35388);
    }

    #[test]
    fn quantize_clamps_to_full_scale() {
        assert_eq!(quantize(100.0, -12.5, 12.5, 16), 65535);
        assert_eq!(quantize(-100.0, -12.5, 12.5, 16), 0);
        assert_eq!(quantize(500.0, 0.0, 500.0, 12), 4095);
        assert_eq!(quantize(5.0, 0.0, 5.0, 12), 4095);
    }

    #[test]
    fn quantize_zero_lands_below_midpoint() {
        // trunc(0.5 * 65535) and trunc(0.5 * 4095): the firmware's zero point.
        assert_eq!(quantize(0.0, -12.5, 12.5, 16), 32767);
        assert_eq!(quantize(0.0, -45.0, 45.0, 12), 2047);
    }

    #[test]
    fn mit_frame_all_zero_command_dm8009() {
        // q=0x7FFF, dq=0x7FF, kp=0, kd=0, tau=0x7FF nibble-packed by hand.
        let f = mit_frame(MotorType::DM8009, 0x01, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert_eq!(f.id, 0x01);
        assert_eq!(f.data, [0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF]);
    }

    #[test]
    fn mit_frame_full_scale_command() {
        // Every field clamped high: all quantizers saturate.
        let f = mit_frame(MotorType::DM4310, 0x05, 501.0, 5.1, 13.0, 31.0, 11.0);
        assert_eq!(f.data, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn mit_frame_nibble_boundaries() {
        // DM4310: q=1.0 -> 35388 = 0x8A3C; kp=250.0 -> trunc(0.5*4095) = 2047 =
        // 0x7FF; kd/dq/tau at zero-point. Exercises every nibble splice.
        let f = mit_frame(MotorType::DM4310, 0x03, 250.0, 0.0, 1.0, 0.0, 0.0);
        assert_eq!(f.data[0], 0x8A);
        assert_eq!(f.data[1], 0x3C);
        assert_eq!(f.data[2], 0x7F); // dq 0x7FF high byte
        assert_eq!(f.data[3], 0xF7); // dq low nibble | kp high nibble
        assert_eq!(f.data[4], 0xFF); // kp low byte
        assert_eq!(f.data[5], 0x00); // kd 0 high byte
        assert_eq!(f.data[6], 0x07); // kd low nibble | tau high nibble
        assert_eq!(f.data[7], 0xFF); // tau low byte
    }

    #[test]
    fn enable_and_disable_magic_frames() {
        assert_eq!(
            enable_frame(0x07).data,
            [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        );
        assert_eq!(enable_frame(0x07).id, 0x07);
        assert_eq!(
            disable_frame(0x08).data,
            [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        );
    }

    #[test]
    fn ctrl_mode_frame_layout() {
        let f = ctrl_mode_frame(0x08, ControlMode::PosForce);
        assert_eq!(f.id, 0x7FF);
        assert_eq!(f.data, [0x08, 0x00, 0x55, 0x0A, 0x04, 0x00, 0x00, 0x00]);
        assert_eq!(ctrl_mode_frame(0x08, ControlMode::Mit).data[4], 0x01);
    }

    #[test]
    fn pos_force_frame_layout() {
        let f = pos_force_frame(0x08, 1.5, 5.0, TorquePu::new(0.5).unwrap());
        assert_eq!(f.id, 0x308);
        assert_eq!(f.data[..4], (1.5f32).to_le_bytes());
        assert_eq!(f.data[4..6], 500u16.to_le_bytes()); // 5.0 rad/s * 100
        assert_eq!(f.data[6..8], 5000u16.to_le_bytes()); // 0.5 pu * 10000
    }

    #[test]
    fn pos_force_frame_clamps_speed() {
        let f = pos_force_frame(0x08, 0.0, 150.0, TorquePu::new(1.0).unwrap());
        assert_eq!(f.data[4..6], 10000u16.to_le_bytes());
        assert_eq!(f.data[6..8], 10000u16.to_le_bytes());
        let f = pos_force_frame(0x08, 0.0, -3.0, TorquePu::new(0.0).unwrap());
        assert_eq!(f.data[4..6], 0u16.to_le_bytes());
        assert_eq!(f.data[6..8], 0u16.to_le_bytes());
    }

    #[test]
    fn torque_pu_rejects_out_of_range() {
        assert!(TorquePu::new(-0.01).is_err());
        assert!(TorquePu::new(1.01).is_err());
        assert!(TorquePu::new(f64::NAN).is_err());
        assert!(TorquePu::new(0.0).is_ok());
        assert!(TorquePu::new(1.0).is_ok());
    }

    #[test]
    fn parse_state_decodes_known_frame() {
        // DM4310, q_u=0x8A3C, dq_u=0x7FF, tau_u=0x7FF; byte 0 and temps ignored.
        let data = [0xAA, 0x8A, 0x3C, 0x7F, 0xF7, 0xFF, 0x30, 0x28];
        let s = parse_state(MotorType::DM4310, &data).unwrap();
        assert!((s.position - dequantize(0x8A3C, -12.5, 12.5, 16)).abs() < 1e-12);
        assert!((s.velocity - dequantize(0x7FF, -30.0, 30.0, 12)).abs() < 1e-12);
        assert!((s.torque - dequantize(0x7FF, -10.0, 10.0, 12)).abs() < 1e-12);
    }

    #[test]
    fn parse_state_rejects_short_frames() {
        assert_eq!(parse_state(MotorType::DM4310, &[0; 7]), None);
        assert_eq!(parse_state(MotorType::DM4310, &[]), None);
    }

    #[test]
    fn command_state_round_trip_within_one_quantum() {
        // Encode a command, feed the same quantized values back as a state
        // frame, and require the decode to land within one quantization step.
        let ty = MotorType::DM8009;
        let lim = ty.limits();
        let q_step = 2.0 * lim.p_max / 65535.0;
        let dq_step = 2.0 * lim.v_max / 4095.0;
        let tau_step = 2.0 * lim.t_max / 4095.0;
        for q in [-12.5, -3.7, -0.001, 0.0, 0.42, 7.9, 12.5] {
            for dq in [-45.0, -1.3, 0.0, 2.2, 45.0] {
                for tau in [-54.0, -8.05, 0.0, 0.5, 54.0] {
                    let f = mit_frame(ty, 0x01, 0.0, 0.0, q, dq, tau);
                    // Rebuild the state layout from the command layout.
                    let q_u = (u16::from(f.data[0]) << 8) | u16::from(f.data[1]);
                    let dq_u = (u16::from(f.data[2]) << 4) | (u16::from(f.data[3]) >> 4);
                    let state = [
                        0x00,
                        (q_u >> 8) as u8,
                        (q_u & 0xFF) as u8,
                        (dq_u >> 4) as u8,
                        (((dq_u & 0xF) << 4) as u8) | (f.data[6] & 0x0F),
                        f.data[7],
                        0x00,
                        0x00,
                    ];
                    let s = parse_state(ty, &state).unwrap();
                    assert!((s.position - q).abs() <= q_step, "q={q} got {}", s.position);
                    assert!(
                        (s.velocity - dq).abs() <= dq_step,
                        "dq={dq} got {}",
                        s.velocity
                    );
                    assert!(
                        (s.torque - tau).abs() <= tau_step,
                        "tau={tau} got {}",
                        s.torque
                    );
                }
            }
        }
    }

    #[test]
    fn limits_match_firmware_table() {
        // Transcribed from Damiao MOTOR_LIMIT_PARAMS (enactic 1.2.8).
        let expect = [
            (MotorType::DM3507, 50.0, 5.0),
            (MotorType::DM4310, 30.0, 10.0),
            (MotorType::DM4310_48V, 50.0, 10.0),
            (MotorType::DM4340, 8.0, 28.0),
            (MotorType::DM4340_48V, 10.0, 28.0),
            (MotorType::DM6006, 45.0, 20.0),
            (MotorType::DM8006, 45.0, 40.0),
            (MotorType::DM8009, 45.0, 54.0),
            (MotorType::DM10010L, 25.0, 200.0),
            (MotorType::DM10010, 20.0, 200.0),
            (MotorType::DMH3510, 280.0, 1.0),
            (MotorType::DMH6215, 45.0, 10.0),
            (MotorType::DMG6220, 45.0, 10.0),
        ];
        for (ty, v_max, t_max) in expect {
            let lim = ty.limits();
            assert_eq!(lim.p_max, 12.5, "{ty:?}");
            assert_eq!(lim.v_max, v_max, "{ty:?}");
            assert_eq!(lim.t_max, t_max, "{ty:?}");
        }
    }
}
