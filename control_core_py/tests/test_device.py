"""DeviceThread lifecycle and the LatestSlot handoff."""

import threading
import time

import pytest

from control_core_py import device


class FakeHardware:
    def __init__(self):
        self.connected = False
        self.disconnected = False
        self.fail_connect: Exception | None = None

    def connect(self):
        if self.fail_connect is not None:
            raise self.fail_connect
        self.connected = True

    def disconnect(self):
        self.disconnected = True


class TickingLoop(device.DeviceThread):
    def __init__(self, hardware, first_read_error: Exception | None = None, period_s=0.001):
        super().__init__(hardware, period_s=period_s, thread_name="test-device")
        self.first_read_error = first_read_error
        self.ticked = threading.Event()

    def _verify_first_read(self):
        if self.first_read_error is not None:
            raise self.first_read_error

    def _tick(self):
        self.ticked.set()


def test_start_runs_ticks_and_stop_disconnects():
    hardware = FakeHardware()
    loop = TickingLoop(hardware)
    loop.start()
    assert loop.ready()
    assert loop.ticked.wait(1.0)
    loop.stop()
    assert hardware.disconnected
    assert not loop.ready()


def test_failed_connect_raises_and_never_reports_ready():
    hardware = FakeHardware()
    hardware.fail_connect = OSError("no port")
    loop = TickingLoop(hardware)
    with pytest.raises(IOError):
        loop.start()
    assert not loop.ready()


def test_failed_first_read_disconnects_before_raising():
    hardware = FakeHardware()
    loop = TickingLoop(hardware, first_read_error=ValueError("short read"))
    with pytest.raises(ValueError):
        loop.start()
    # The port (and any torque enabled during connect) is released.
    assert hardware.disconnected


@pytest.mark.parametrize("period_s", [0.0, -0.001, float("nan"), float("inf")])
def test_invalid_period_is_rejected_before_the_thread_exists(period_s):
    with pytest.raises(ValueError):
        TickingLoop(FakeHardware(), period_s=period_s)


def test_latest_slot_freshness():
    slot = device.LatestSlot()
    assert slot.fresh(10.0) is None
    slot.put(41)
    assert slot.fresh(10.0) == 41
    assert slot.get() == 41
    time.sleep(0.02)
    assert slot.fresh(0.01) is None
