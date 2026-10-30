import binascii
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from measure_hc13_latency import packet
from measure_hc13_rtt import parse_reply


class DiagnosticRttTests(unittest.TestCase):
    def test_requires_mcu_reply_prefix_and_crc(self):
        request=packet(0x1234,27)
        self.assertIsNone(parse_reply(request))
        body=b'R'+request[1:9]
        reply=body+f'{binascii.crc_hqx(body,0xffff):04X}\n'.encode()
        self.assertEqual(parse_reply(reply),(0x1234,27))
        self.assertIsNone(parse_reply(reply[:6]+b'F'+reply[7:]))
        self.assertIsNone(parse_reply(reply[:-1]))


if __name__=='__main__':unittest.main()
