"""边沿定时接收（canbit.timing）的独立对照测试。

对照关系：理想位流由 encode_frame 真实编码；simulate_edges 用确定性
发送时钟偏差/毛刺生成边沿；receive_edges 恢复位流并解码。可修复与
超出修正范围两种情况分别断言最终帧状态。
"""

import unittest

from canbit import (
    BitTiming,
    Edge,
    Frame,
    Glitch,
    encode_frame,
    receive_edges,
    simulate_edges,
)
from canbit.errors import ValidationError

TIMING = BitTiming(prop_seg=3, phase_seg1=3, phase_seg2=3, sjw=2)
FRAME = Frame(0x321, b"hi")


def stream():
    return encode_frame(FRAME).stream


class BitTimingTest(unittest.TestCase):
    def test_derived_parameters(self):
        self.assertEqual(TIMING.tq_per_bit, 10)
        self.assertEqual(TIMING.sample_point, 7)

    def test_validation(self):
        for bad in (
            lambda: BitTiming(0, 3, 3, 2),      # 段长下界
            lambda: BitTiming(3, 3, 3, 0),      # sjw 下界
            lambda: BitTiming(3, 3, 3, 4),      # sjw > min(ps1, ps2)
            lambda: BitTiming(True, 3, 3, 2),   # bool 拒绝
            lambda: BitTiming(3.0, 3, 3, 2),    # float 拒绝
            lambda: BitTiming(3, 3, float("nan"), 1),  # NaN 拒绝
        ):
            with self.assertRaises(ValidationError):
                bad()


class IdealReceptionTest(unittest.TestCase):
    def test_ideal_clock_recovers_exact_stream(self):
        res = receive_edges(simulate_edges(stream(), TIMING), TIMING)
        self.assertEqual(res.status, "ok")
        self.assertEqual(res.frame, FRAME)
        self.assertTrue(res.crc_ok)
        self.assertEqual(list(res.bits), stream())
        self.assertIsNone(res.error_bit_index)

    def test_hard_sync_separated_from_resync(self):
        res = receive_edges(simulate_edges(stream(), TIMING), TIMING)
        hard = [c for c in res.corrections if c.kind == "hard_sync"]
        resync = [c for c in res.corrections if c.kind == "resync"]
        self.assertEqual(len(hard), 1)
        self.assertEqual(hard[0].edge_time, 0)
        self.assertTrue(resync)  # 帧内确有重同步记录
        for c in resync:
            self.assertLessEqual(abs(c.applied), TIMING.sjw)

    def test_sample_times_reported(self):
        res = receive_edges(simulate_edges(stream(), TIMING), TIMING)
        self.assertEqual(len(res.samples), len(res.bits))
        first = res.samples[0]
        self.assertEqual((first.bit_index, first.time, first.bit),
                         (0, TIMING.sample_point, 0))  # SOF 为显性

    def test_delayed_sof_hard_sync(self):
        # SOF 边沿不在 0 时刻：硬同步应对齐到实际边沿
        edges = [Edge(e.time + 55, e.level)
                 for e in simulate_edges(stream(), TIMING)]
        res = receive_edges(edges, TIMING)
        self.assertEqual(res.status, "ok")
        self.assertEqual(res.corrections[0].kind, "hard_sync")
        self.assertEqual(res.corrections[0].edge_time, 55)
        self.assertEqual(res.samples[0].time, 55 + TIMING.sample_point)


class DriftReceptionTest(unittest.TestCase):
    def test_recoverable_positive_drift(self):
        edges = simulate_edges(stream(), TIMING, drift_num=1, drift_den=8)
        res = receive_edges(edges, TIMING)
        self.assertEqual(res.status, "ok")
        self.assertEqual(res.frame, FRAME)
        self.assertEqual(list(res.bits), stream())
        self.assertTrue(any(c.applied > 0 for c in res.corrections
                            if c.kind == "resync"))

    def test_recoverable_negative_drift(self):
        edges = simulate_edges(stream(), TIMING, drift_num=-1, drift_den=8)
        res = receive_edges(edges, TIMING)
        self.assertEqual(res.status, "ok")
        self.assertEqual(res.frame, FRAME)
        self.assertTrue(any(c.applied < 0 for c in res.corrections
                            if c.kind == "resync"))

    def test_unrecoverable_fast_clock(self):
        # 发送时钟每位快 1 tq（10%），远超 SJW=2 的有界修正能力
        edges = simulate_edges(stream(), TIMING, drift_num=-1, drift_den=1)
        res = receive_edges(edges, TIMING)
        self.assertNotEqual(res.status, "ok")
        self.assertIsNone(res.frame)
        self.assertFalse(res.crc_ok)
        self.assertIsNotNone(res.error_bit_index)

    def test_unrecoverable_slow_clock(self):
        edges = simulate_edges(stream(), TIMING, drift_num=1, drift_den=1)
        res = receive_edges(edges, TIMING)
        self.assertNotEqual(res.status, "ok")
        self.assertIsNone(res.frame)

    def test_sjw_clamping_recorded(self):
        # 边沿比期望边界晚 5 tq，SJW=2：调整量被截断并标记
        edges = [Edge(0, 0), Edge(10, 1), Edge(25, 0)]
        res = receive_edges(edges, TIMING)
        resync = [c for c in res.corrections if c.kind == "resync"]
        self.assertEqual(len(resync), 1)
        self.assertEqual(resync[0].phase_error, 5)
        self.assertEqual(resync[0].applied, TIMING.sjw)
        self.assertTrue(resync[0].clamped)


class GlitchReceptionTest(unittest.TestCase):
    def test_short_recessive_glitch_on_dominant_bit_ignored(self):
        # 显性位中间的短隐性毛刺：远离采样点，不影响恢复
        edges = simulate_edges(stream(), TIMING,
                               glitches=[Glitch(12, 1, 2)])
        res = receive_edges(edges, TIMING)
        self.assertEqual(res.status, "ok")
        self.assertEqual(res.frame, FRAME)

    def test_short_dominant_glitch_on_recessive_bit_ignored(self):
        # 隐性位上的短显性毛刺会制造伪边沿，但有界重同步后仍恢复
        edges = simulate_edges(stream(), TIMING,
                               glitches=[(22, 0, 2)])
        res = receive_edges(edges, TIMING)
        self.assertEqual(res.status, "ok")
        self.assertEqual(res.frame, FRAME)

    def test_long_glitch_corrupts_frame(self):
        # 覆盖采样点的长毛刺：采样值被翻转，帧检查真实失败
        edges = simulate_edges(stream(), TIMING,
                               glitches=[(5, 1, 6)])  # 覆盖 t=7 的 SOF 采样点
        res = receive_edges(edges, TIMING)
        self.assertNotEqual(res.status, "ok")
        self.assertIsNone(res.frame)


class InputValidationTest(unittest.TestCase):
    def test_no_frame_when_bus_idle(self):
        res = receive_edges([], TIMING)
        self.assertEqual(res.status, "no_frame")
        res = receive_edges([Edge(3, 1), Edge(9, 1)], TIMING)
        self.assertEqual(res.status, "no_frame")

    def test_edge_validation(self):
        with self.assertRaises(ValidationError):
            receive_edges([(5, 0), (5, 1)], TIMING)   # 时间非严格递增
        with self.assertRaises(ValidationError):
            receive_edges([(True, 0)], TIMING)        # bool 拒绝
        with self.assertRaises(ValidationError):
            receive_edges([(0, 2)], TIMING)           # 电平越界
        with self.assertRaises(ValidationError):
            receive_edges(["x"], TIMING)              # 非法边沿类型
        with self.assertRaises(ValidationError):
            receive_edges([], "not-a-timing")         # 非法定时参数

    def test_simulate_validation(self):
        with self.assertRaises(ValidationError):
            simulate_edges([], TIMING)
        with self.assertRaises(ValidationError):
            simulate_edges([0, True], TIMING)
        with self.assertRaises(ValidationError):
            simulate_edges([0], TIMING, drift_den=0)
        with self.assertRaises(ValidationError):
            simulate_edges([0], TIMING, glitches=[(1, 2, 3, 4)])

    def test_simulate_edges_deterministic(self):
        a = simulate_edges(stream(), TIMING, drift_num=1, drift_den=8,
                           glitches=[(22, 0, 2)])
        b = simulate_edges(stream(), TIMING, drift_num=1, drift_den=8,
                           glitches=[(22, 0, 2)])
        self.assertEqual(a, b)


if __name__ == "__main__":
    unittest.main()
