//! SocketCAN transport with a per-motor state cache.
//!
//! Mirrors the Damiao bus conventions: no kernel CAN filters (frames are
//! dispatched in software by recv id), CAN-FD frames carry the bit-rate-switch
//! flag, and a receive pass waits `first_timeout_us` for the first frame then
//! drains the rest without waiting.

use std::os::fd::AsFd;
use std::time::Duration;

use nix::errno::Errno;
use nix::poll::{PollFd, PollFlags, ppoll};
use nix::sys::time::TimeSpec;
use socketcan::id::FdFlags;
use socketcan::{
    CanAnyFrame, CanFdFrame, CanFdSocket, CanFrame, CanSocket, EmbeddedFrame, Frame, Socket,
    StandardId,
};

use crate::protocol::{self, ControlMode, MotorState, MotorType, OutFrame};
use crate::{CanError, Result};

/// Highest valid 11-bit standard CAN id.
const CAN_SFF_MAX: u32 = 0x7FF;

/// SO_RCVTIMEO on the bus socket: the reference implementation's 100us
/// stuck-read guard, kept so a blocking read can never outlive one kernel
/// timer tick even if the readiness poll misfires.
const RECV_BACKSTOP: Duration = Duration::from_micros(100);

/// One motor on the bus: its addressing plus the last decoded state.
#[derive(Debug, Clone, Copy)]
pub(crate) struct MotorSlot {
    motor_type: MotorType,
    send_id: u32,
    recv_id: u32,
    state: MotorState,
}

impl MotorSlot {
    /// Checks both ids fit the 11-bit standard range; `extra_send_offset` is
    /// added to the send id for modes that address an id above it (POS_FORCE).
    pub fn new(
        motor_type: MotorType,
        send_id: u32,
        recv_id: u32,
        extra_send_offset: u32,
    ) -> Result<Self> {
        for id in [send_id, recv_id] {
            if id > CAN_SFF_MAX {
                return Err(CanError::InvalidCanId(id));
            }
        }
        if send_id + extra_send_offset > CAN_SFF_MAX {
            return Err(CanError::InvalidCanId(send_id + extra_send_offset));
        }
        Ok(Self {
            motor_type,
            send_id,
            recv_id,
            state: MotorState::default(),
        })
    }

    pub fn state(&self) -> MotorState {
        self.state
    }

    pub fn motor_type(&self) -> MotorType {
        self.motor_type
    }

    pub fn send_id(&self) -> u32 {
        self.send_id
    }
}

enum CanSock {
    Classic(CanSocket),
    Fd(CanFdSocket),
}

impl CanSock {
    fn send(&self, frame: &OutFrame) -> Result<()> {
        let id = StandardId::new(frame.id as u16).expect("ids validated in MotorSlot::new");
        match self {
            Self::Classic(socket) => {
                let frame = CanFrame::new(id, &frame.data).expect("8 bytes fit a CAN frame");
                socket.write_frame(&frame)?;
            }
            Self::Fd(socket) => {
                let frame = CanFdFrame::with_flags(id, &frame.data, FdFlags::BRS)
                    .expect("8 bytes fit a CAN-FD frame");
                socket.write_frame(&frame)?;
            }
        }
        Ok(())
    }

    /// Reads one frame, waiting at most `timeout`. Returns `None` on timeout.
    fn recv(&self, timeout: Duration) -> Result<Option<CanAnyFrame>> {
        if !self.readable_within(timeout)? {
            return Ok(None);
        }
        let read = match self {
            Self::Classic(socket) => socket.read_frame().map(CanAnyFrame::from),
            Self::Fd(socket) => socket.read_frame(),
        };
        match read {
            Ok(frame) => Ok(Some(frame)),
            // The RECV_BACKSTOP timeout fired: poll readiness raced away.
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                Ok(None)
            }
            Err(e) => Err(e.into()),
        }
    }

    /// Waits for the socket to become readable via `ppoll`, which takes a
    /// nanosecond timeout; the crate-provided timed read only has millisecond
    /// resolution, which would flatten the firmware's microsecond receive
    /// windows. A signal during the wait reads as "nothing arrived", matching
    /// the reference implementation's select loop.
    fn readable_within(&self, timeout: Duration) -> Result<bool> {
        let socket = match self {
            Self::Classic(socket) => socket.as_raw_socket(),
            Self::Fd(socket) => socket.as_raw_socket(),
        };
        let mut fds = [PollFd::new(socket.as_fd(), PollFlags::POLLIN)];
        match ppoll(&mut fds, Some(TimeSpec::from(timeout)), None) {
            Ok(n) => Ok(n > 0),
            Err(Errno::EINTR) => Ok(false),
            Err(errno) => Err(std::io::Error::from(errno).into()),
        }
    }
}

/// A CAN interface with the motors that live on it.
pub(crate) struct MotorBus {
    socket: CanSock,
    slots: Vec<MotorSlot>,
}

impl MotorBus {
    pub fn open(interface: &str, enable_fd: bool, slots: Vec<MotorSlot>) -> Result<Self> {
        let open_err = |source| CanError::Open {
            interface: interface.to_owned(),
            source,
        };
        let socket = if enable_fd {
            CanSock::Fd(CanFdSocket::open(interface).map_err(open_err)?)
        } else {
            CanSock::Classic(CanSocket::open(interface).map_err(open_err)?)
        };
        // Backstop from the reference implementation: reads happen only after
        // a readiness poll, but if that invariant is ever wrong (kernel or
        // driver quirk), SO_RCVTIMEO bounds the blocking read instead of
        // letting it hang the caller's mutex forever. Kernel rounding makes
        // the effective bound one scheduler tick, not the literal value.
        let raw = match &socket {
            CanSock::Classic(s) => s.as_raw_socket(),
            CanSock::Fd(s) => s.as_raw_socket(),
        };
        raw.set_read_timeout(Some(RECV_BACKSTOP))
            .map_err(open_err)?;
        Ok(Self { socket, slots })
    }

    pub fn slots(&self) -> &[MotorSlot] {
        &self.slots
    }

    pub fn send(&mut self, frame: &OutFrame) -> Result<()> {
        self.socket.send(frame)
    }

    pub fn enable_all(&mut self) -> Result<()> {
        self.send_to_each(protocol::enable_frame)
    }

    pub fn disable_all(&mut self) -> Result<()> {
        self.send_to_each(protocol::disable_frame)
    }

    /// Writes the control mode to every motor (a parameter write on the
    /// shared param id).
    pub fn set_control_mode(&mut self, mode: ControlMode) -> Result<()> {
        self.send_to_each(|send_id| protocol::ctrl_mode_frame(send_id, mode))
    }

    /// Requests a state frame from every motor without commanding it.
    pub fn refresh_all(&mut self) -> Result<()> {
        self.send_to_each(protocol::refresh_frame)
    }

    /// Sends every frame even when one fails, reporting the first error.
    pub fn send_all(&mut self, frames: impl IntoIterator<Item = OutFrame>) -> Result<()> {
        first_error(frames.into_iter().map(|frame| self.socket.send(&frame)))
    }

    fn send_to_each(&mut self, frame_for: impl Fn(u32) -> OutFrame) -> Result<()> {
        first_error(
            self.slots
                .iter()
                .map(|slot| frame_for(slot.send_id))
                .map(|frame| self.socket.send(&frame)),
        )
    }

    /// Receives and decodes state frames into the cache: waits up to
    /// `first_timeout_us` for the first frame, then drains without waiting.
    /// Frames from unknown ids and undecodable payloads are ignored.
    /// Returns whether any motor state reached the cache this pass; `false`
    /// means `get_state` holds previous readings, not fresh ones.
    pub fn recv_all(&mut self, first_timeout_us: u32) -> Result<bool> {
        self.recv_loop(first_timeout_us, true)
    }

    /// Same receive pass as [`recv_all`](Self::recv_all) but discards every
    /// frame. Use to consume bus traffic that must not land in the state
    /// cache (bring-up replies, parameter echoes).
    pub fn drain(&mut self, first_timeout_us: u32) -> Result<()> {
        self.recv_loop(first_timeout_us, false).map(drop)
    }

    fn recv_loop(&mut self, first_timeout_us: u32, decode: bool) -> Result<bool> {
        let mut timeout = Duration::from_micros(first_timeout_us.into());
        let mut decoded = false;
        while let Some(frame) = self.socket.recv(timeout)? {
            timeout = Duration::ZERO;
            decoded |= decode && self.dispatch(&frame).is_some();
        }
        Ok(decoded)
    }

    /// The index of the slot the frame decoded into, if any.
    fn dispatch(&mut self, frame: &CanAnyFrame) -> Option<usize> {
        // Remote and error frames can never be motor state; the Damiao
        // protocol replies with plain data frames only.
        let (extended, id, data) = match frame {
            CanAnyFrame::Normal(f) => (f.is_extended(), f.raw_id(), f.data()),
            CanAnyFrame::Fd(f) => (f.is_extended(), f.raw_id(), f.data()),
            CanAnyFrame::Remote(_) | CanAnyFrame::Error(_) => return None,
        };
        decode_into_slots(&mut self.slots, extended, id, data)
    }
}

/// Drives every send in a group command and keeps the first error. Stopping at
/// the first failure would leave the group half applied: a partial disable has
/// some joints limp and some holding their last command (the arm folds
/// asymmetrically), and a partial refresh blinds the joints it skipped. The
/// reference implementation always addresses every motor.
fn first_error(results: impl Iterator<Item = Result<()>>) -> Result<()> {
    results.fold(Ok(()), Result::and)
}

/// Decodes a received data frame into the matching slot's state cache,
/// returning the slot's index when it lands.
/// Rejected without decoding: extended-id frames (motors address with 11-bit
/// ids only; `raw_id` strips the EFF flag, so without this gate a foreign
/// 29-bit frame whose low bits collide with a recv id would masquerade as
/// state) and parameter replies (the motor mirrors 0x7FF writes/queries back
/// on its state id; decoded as state, the echo reads as a violent full-scale
/// position).
fn decode_into_slots(
    slots: &mut [MotorSlot],
    extended: bool,
    id: u32,
    data: &[u8],
) -> Option<usize> {
    if extended {
        return None;
    }
    let (index, slot) = slots
        .iter_mut()
        .enumerate()
        .find(|(_, slot)| slot.recv_id == id)?;
    if is_param_reply(slot.send_id, data) {
        return None;
    }
    slot.state = protocol::parse_state(slot.motor_type, data)?;
    Some(index)
}

/// A parameter reply carries the motor's send id little-endian in bytes 0-1
/// and the write/query opcode in byte 2. A state frame matching all three
/// bytes would put the motor at a physically unreachable full-scale position,
/// so this cannot reject live states.
fn is_param_reply(send_id: u32, data: &[u8]) -> bool {
    data.len() >= 3
        && data[0] == (send_id & 0xFF) as u8
        && data[1] == ((send_id >> 8) & 0xFF) as u8
        && matches!(data[2], 0x33 | 0x55)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gripper_slot() -> MotorSlot {
        MotorSlot::new(MotorType::DM4310, 0x08, 0x18, 0x300).unwrap()
    }

    /// DM4310 state: q at +p_max, dq at -v_max, tau at +t_max.
    const STATE: [u8; 8] = [0x00, 0xFF, 0xFF, 0x00, 0x0F, 0xFF, 0x30, 0x28];

    /// Every group send routes through `first_error`; this pins its two
    /// guarantees: the iterator is driven to the end (every motor addressed
    /// even after a failure) and the first error is the one reported.
    #[test]
    fn first_error_drives_every_send_and_reports_the_first_failure() {
        let attempted = std::cell::Cell::new(0u32);
        let outcome = first_error(
            [
                Ok(()),
                Err(CanError::InvalidCanId(1)),
                Err(CanError::InvalidCanId(2)),
                Ok(()),
            ]
            .into_iter()
            .inspect(|_| attempted.set(attempted.get() + 1)),
        );
        assert_eq!(attempted.get(), 4, "a failed send must not skip the rest");
        assert!(matches!(outcome, Err(CanError::InvalidCanId(1))));
    }

    #[test]
    fn first_error_of_all_ok_is_ok() {
        assert!(first_error([Ok(()), Ok(())].into_iter()).is_ok());
    }

    #[test]
    fn state_frame_updates_the_matching_slot_and_reports_its_index() {
        let mut slots = [gripper_slot()];
        assert_eq!(decode_into_slots(&mut slots, false, 0x18, &STATE), Some(0));
        assert_eq!(slots[0].state().position, 12.5);
    }

    #[test]
    fn extended_id_frames_are_rejected() {
        // A 29-bit id whose low bits collide with the recv id must not
        // masquerade as motor state.
        let mut slots = [gripper_slot()];
        assert_eq!(decode_into_slots(&mut slots, true, 0x18, &STATE), None);
        assert_eq!(slots[0].state(), MotorState::default());
    }

    #[test]
    fn param_reply_echoes_are_rejected() {
        // The CTRL_MODE write (0x55) and query (0x33) echoes carry the send
        // id little-endian then the opcode; decoded as state they would read
        // as a full-scale position.
        let mut slots = [gripper_slot()];
        for opcode in [0x55, 0x33] {
            let decoded = decode_into_slots(
                &mut slots,
                false,
                0x18,
                &[0x08, 0x00, opcode, 0x0A, 4, 0, 0, 0],
            );
            assert_eq!(decoded, None);
            assert_eq!(slots[0].state(), MotorState::default());
        }
    }

    #[test]
    fn echo_shaped_frame_for_a_different_motor_still_decodes() {
        // The reply signature is keyed to this slot's send id; the same bytes
        // with a mismatched id prefix are treated as (implausible) state.
        let mut slots = [gripper_slot()];
        let decoded = decode_into_slots(
            &mut slots,
            false,
            0x18,
            &[0x07, 0x00, 0x55, 0x0A, 4, 0, 0, 0],
        );
        assert_eq!(decoded, Some(0));
        assert_ne!(slots[0].state(), MotorState::default());
    }

    #[test]
    fn unknown_ids_are_ignored() {
        let mut slots = [gripper_slot()];
        assert_eq!(decode_into_slots(&mut slots, false, 0x19, &STATE), None);
        assert_eq!(slots[0].state(), MotorState::default());
    }
}
