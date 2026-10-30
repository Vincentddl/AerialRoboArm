import sys
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from measure_hc13_latency import Tracker,packet,parse_packet,latency_statistics


class LatencyTests(unittest.TestCase):
    def test_fixed_probe_is_not_control_and_rejects_corruption(self):
        raw=packet(0x1234,27)
        self.assertEqual(len(raw),14)
        self.assertFalse(raw.startswith((b'V,',b'VISION,',b'AT')))
        self.assertEqual(parse_packet(raw),(0x1234,27))
        self.assertIsNone(parse_packet(raw[:6]+b'F'+raw[7:]))

    def test_same_clock_delay_is_not_divided_by_two(self):
        t=Tracker(0x1234);t.register(1,1_000_000_000)
        t.observe(packet(0x1234,1),1_017_000_000)
        t.observe(packet(0x1234,1),1_090_000_000)
        stats=latency_statistics(list(t.rows.values()))
        self.assertEqual(stats['median_ms'],17)
        self.assertEqual(stats['count'],1)
        self.assertEqual(t.counts['duplicate'],1)

    def test_foreign_unknown_and_negative_timestamp_rejected(self):
        t=Tracker(1);t.register(1,1_000_000)
        t.observe(packet(2,1),2_000_000)
        t.observe(packet(1,2),2_000_000)
        t.observe(packet(1,1),999_999)
        self.assertIsNone(latency_statistics(list(t.rows.values())))

    def test_missing_and_write_failure_do_not_become_zero(self):
        t=Tracker(1);t.register(1,100)
        self.assertIsNone(latency_statistics(list(t.rows.values())))
        t.observe(packet(1,1),200);t.rows[1]['write_error']='short_write'
        self.assertIsNone(latency_statistics(list(t.rows.values())))


if __name__=='__main__':unittest.main()
