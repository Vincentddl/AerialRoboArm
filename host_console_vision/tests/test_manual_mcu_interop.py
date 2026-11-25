"""Actual GUI and MCU C protocol, simulated transport/feedback (no hardware)."""
import os
from pathlib import Path
import subprocess
import threading
import time
import unittest
from unittest.mock import patch
from test_protocol_v1 import app
from PySide6.QtTest import QTest
from arm_console.window import MainWindow
from arm_console.v1_state import ExperimentController
from arm_console.video_capture import VideoCaptureService

PEER = os.environ.get('HC13_NATIVE_PEER_EXE', '')

class CPort:
    instances = []
    def __init__(self, *args, **kwargs):
        self.proc = subprocess.Popen([PEER, '--peer-unconfigured'], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.buffer = bytearray()
        self.writes = []
        self.lines = []
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()
        self.instances.append(self)
    def open(self):
        assert self.dtr is False and self.rts is False
    def reader(self):
        for line in self.proc.stdout:
            with self.lock:
                self.buffer.extend(line)
                self.lines.append(line)
    def __enter__(self): return self
    def __exit__(self, *args):
        self.proc.stdin.close()
        self.proc.wait(timeout=3)
        self.thread.join(timeout=1)
        self.proc.stdout.close()
    @property
    def in_waiting(self):
        with self.lock: return len(self.buffer)
    def read(self, n):
        with self.lock:
            data = bytes(self.buffer[:n])
            del self.buffer[:n]
        if not data: time.sleep(.002)
        return data
    def write(self, data):
        self.writes.append(data)
        self.control(data)
        return len(data)
    def control(self, data):
        self.proc.stdin.write(data)
        self.proc.stdin.flush()

@unittest.skipUnless(PEER and Path(PEER).is_file(), 'Set HC13_NATIVE_PEER_EXE')
class ManualMcuInterop(unittest.TestCase):
    def wait(self, predicate):
        end = time.monotonic() + 4
        while not predicate() and time.monotonic() < end: QTest.qWait(10)
        self.assertTrue(predicate())

    def test_manual_buttons_with_real_c_parser_and_na_waits(self):
        with patch('arm_console.serial_service.serial.Serial', CPort), patch(
                'arm_console.window.ExperimentController', lambda serial, _: ExperimentController(serial)):
            w = MainWindow(video_service=VideoCaptureService(enumerator=lambda: []))
            w.show()
            try:
                w.serial.connect_port('SIMULATED-C-MCU', 230400)
                self.wait(lambda: w.controller.link_online and w.controller.cal is not None)
                c, p = w.controller, CPort.instances[-1]
                self.assertIsNone(c.cal.open_wait_ms)
                self.assertIsNone(c.cal.close_wait_ms)
                self.assertFalse(c.request_visual_prepare())
                self.assertIsNone(c.cal.folded_mdeg)  # Global preset now uses MOVE; SHAPE remains uncalibrated.
                from arm_console.pages import ManualPage
                page = w.findChild(ManualPage)
                page.zero_button.click()
                self.wait(lambda: any(x.startswith(b'MOVE,') and x.endswith(b',90000,300\n') for x in p.writes))
                p.control(b'@ARRIVED,90000\n@DONE,90000\n')
                self.wait(lambda: not c._pending)
                page.speed_spin.setValue(20)
                page.angle_spin.setValue(30)
                page.start_button.click()
                self.wait(lambda: any(x.startswith(b'MOVE,') and x.endswith(b',60000,100\n') for x in p.writes))
                p.control(b'@ARRIVED,60100\n@DONE,60100\n')
                self.wait(lambda: not c._pending)
                self.wait(lambda: c.latest_state.pos_mdeg == 60100)
                self.assertIn('60.1', page.actual.text())
                # Each button sends one absolute PWM command, even when the
                # MCU has no previously reported jaw command angle.
                for button, angle in ((page.close_button, 90000),
                                      (page.open_button, 125000)):
                    before = len(p.writes)
                    button.click()
                    self.wait(lambda: not c._pending)
                    commands = [int(x.rstrip().split(b',')[-1]) for x in p.writes[before:]
                                if x.startswith(b'SERVO,')]
                    self.assertEqual(commands, [angle])
                    self.assertTrue(c.query('STATE'))
                    self.wait(lambda: c.latest_state.grip_cmd_mdeg == angle)
                before = len(p.writes)
                page.zero_button.click()
                self.wait(lambda: any(x.startswith(b'MOVE,') and x.endswith(b',90000,300\n') for x in p.writes[before:]))
                p.control(b'@ARRIVED,90000\n@DONE,90000\n')
                self.wait(lambda: not c._pending)
                self.assertFalse(any(x.startswith((b'PICK,', b'PREP,', b'HOME,')) for x in p.writes))
                self.assertFalse(any(x.startswith(b'ERR,') for x in p.lines))
                # Real C timeout -> fault -> STOP recovery, without moving automatically.
                page.start_button.click()
                self.wait(lambda: any(x.acked for x in c._pending.values()))
                # Keep transport alive so this tests motion timeout, not link loss.
                for _ in range(21):
                    p.control(b'PING,40000000\n@TICK,500\n')
                c.query('STATE')
                self.wait(lambda: c.latest_state.fault_code == 'MOTION_TIMEOUT')
                self.assertTrue(page.recover_button.isEnabled())
                before = len(p.writes)
                page.recover_button.click()
                self.wait(lambda: not c._pending and c.latest_state.fault_code == 'NONE')
                self.assertEqual(page.fault_label.text(), '故障：无（设备反馈）')
                self.assertTrue(all(x.startswith((b'STOP,', b'GET,', b'PING,')) for x in p.writes[before:]))
            finally:
                w.close()
                QTest.qWait(100)
