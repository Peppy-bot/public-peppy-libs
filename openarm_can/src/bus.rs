//! SocketCAN transport with a per-motor state cache.
//!
//! Mirrors the Damiao bus conventions: no kernel CAN filters (frames are
//! dispatched in software by recv id), CAN-FD frames carry the bit-rate-switch
//! flag, and a receive pass waits `first_timeout_us` for the first frame then
//! drains the rest without waiting.

use std::time::Duration;

use socketcan::id::FdFlags;
use socketcan::{
    CanAnyFrame, CanFdFrame, CanFdSocket, CanFrame, CanSocket, EmbeddedFrame, Frame, Socket,
    StandardId,
};

use crate::protocol::{self, ControlMode, MotorState, MotorType, OutFrame};
use crate::{CanError, Result};

/// Highest valid 11-bit standard CAN id.
const CAN_SFF_MAX: u32 = 0x7FF;

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
        let read = match self {
            Self::Classic(socket) => socket.read_frame_timeout(timeout).map(CanAnyFrame::from),
            Self::Fd(socket) => socket.read_frame_timeout(timeout),
        };
        match read {
            Ok(frame) => Ok(Some(frame)),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e.into()),
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

    fn send_to_each(&mut self, frame_for: impl Fn(u32) -> OutFrame) -> Result<()> {
        self.slots
            .iter()
            .map(|slot| frame_for(slot.send_id))
            .try_for_each(|frame| self.socket.send(&frame))
    }

    /// Receives and decodes state frames into the cache: waits up to
    /// `first_timeout_us` for the first frame, then drains without waiting.
    /// Frames from unknown ids and undecodable payloads are ignored.
    pub fn recv_all(&mut self, first_timeout_us: u32) -> Result<()> {
        self.recv_loop(first_timeout_us, true)
    }

    /// Same receive pass as [`recv_all`](Self::recv_all) but discards every
    /// frame. Use to consume bus traffic that must not land in the state
    /// cache (bring-up replies, parameter echoes).
    pub fn drain(&mut self, first_timeout_us: u32) -> Result<()> {
        self.recv_loop(first_timeout_us, false)
    }

    fn recv_loop(&mut self, first_timeout_us: u32, decode: bool) -> Result<()> {
        let mut timeout = Duration::from_micros(first_timeout_us.into());
        while let Some(frame) = self.socket.recv(timeout)? {
            timeout = Duration::ZERO;
            if decode {
                self.dispatch(&frame);
            }
        }
        Ok(())
    }

    fn dispatch(&mut self, frame: &CanAnyFrame) {
        // Remote and error frames can never be motor state; the Damiao
        // protocol replies with plain data frames only.
        let (id, data) = match frame {
            CanAnyFrame::Normal(f) => (f.raw_id(), f.data()),
            CanAnyFrame::Fd(f) => (f.raw_id(), f.data()),
            CanAnyFrame::Remote(_) | CanAnyFrame::Error(_) => return,
        };
        let Some(slot) = self.slots.iter_mut().find(|slot| slot.recv_id == id) else {
            return;
        };
        if let Some(state) = protocol::parse_state(slot.motor_type, data) {
            slot.state = state;
        }
    }
}
