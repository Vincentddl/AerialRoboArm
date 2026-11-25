import unittest
from dataclasses import replace
from test_protocol_v1 import app, online_controller
from arm_console.pages import ManualPage
from arm_console.v1_protocol import AckFrame, ErrFrame


class ManualHealthTests(unittest.TestCase):
    def setUp(self):
        self.c, self.link = online_controller()
        self.page = ManualPage(self.c)
        self.fault = replace(self.c.latest_state, motion='FAULT', fault_code='MOTION_TIMEOUT')
        self.c.handle_frame(self.fault)

    def tearDown(self):
        self.page.close()
        self.c.shutdown()

    def test_voltage_fault_and_recovery_wait_for_device_state(self):
        self.assertIn('11.98 V', self.page.voltage_label.text())
        self.assertIn('MOTION_TIMEOUT', self.page.fault_label.text())
        self.assertTrue(self.page.recover_button.isEnabled())
        self.c._queued_mode = 'VISION_PICK'
        before = len(self.link.sent)
        self.page.recover_button.click()
        payload, priority = self.link.sent[-1]
        self.assertTrue(payload.startswith(b'STOP,'))
        self.assertTrue(priority)
        self.assertIsNone(self.c._queued_mode)
        self.assertFalse(self.page.recover_button.isEnabled())
        req = int(payload.split(b',')[1])
        self.c.handle_frame(AckFrame(req, 'STOP', 'APPLIED'))
        self.assertIn('MOTION_TIMEOUT', self.page.fault_label.text())
        self.assertTrue(self.link.sent[-1][0].endswith(b',STATE\n'))
        self.assertFalse(self.c.request_home())  # Old fault still blocks motion.
        self.c.handle_frame(replace(self.fault, t_ms=self.fault.t_ms+100,
                                   motion='IDLE', fault_code='NONE'))
        self.assertEqual(self.page.fault_label.text(), '故障：无（设备反馈）')
        self.assertFalse(self.page.recover_button.isEnabled())
        self.assertTrue(all(p.startswith((b'STOP,', b'GET,')) for p, _ in self.link.sent[before:]))

    def test_failed_recovery_and_disconnect_do_not_claim_success(self):
        self.assertTrue(self.c.request_recover())
        req = int(self.link.sent[-1][0].split(b',')[1])
        self.c.handle_frame(ErrFrame(req, 'HX8_OFFLINE'))
        self.assertIn('MOTION_TIMEOUT', self.page.fault_label.text())
        self.assertIn('HX8_OFFLINE', self.c.command_status)
        self.c.link_online = False
        self.c.state_changed.emit()
        self.assertFalse(self.page.recover_button.isEnabled())
        self.assertIn('未知', self.page.voltage_label.text())
        before = len(self.link.sent)
        self.assertFalse(self.c.request_recover())
        self.assertEqual(len(self.link.sent), before)
