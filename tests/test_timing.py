"""边沿时间输入入口（canbit.timing）的独立对照测试。

对照口径：理想位串由 encode_frame(...).stream 直接给出（发送侧），
接收端只能看到边沿时间表；恢复出的位流逐位与理想流前缀比对，
帧内容再与原始 Frame 比对。覆盖：
* 零偏差：逐位一致、零修正、硬同步/重同步标记分离
* 可修复偏差（正/负）：重同步有界（|修正| <= sjw），帧恢复成功
* 短毛刺：被滤除，不参与同步，帧仍恢复成功
* 超出修正范围的偏差：帧检查失败（非 ok 状态）
* 参数校验与旧直接位流入口的保留
"""

from __future__ import annotations

import unittest
from fractions import Fraction

from canbit import (
    BitTiming,
    Edge,
    Frame,
    Glitch,
    ValidationError,
    decode_frame,
    encode_frame,
    receive_edges,
    synthesize_edges,
)

TIMING = BitTiming(prop_seg=3, phase_seg1=4, phase_seg2=4, sjw=2)
FRAME = Frame(0x321, b"hi")


def ideal_stream() -> list[int]:
    return encode_frame(FRAME).stream


class TestBitTimingParams(unittest.TestCase):
    def test_derived_constants(self):
        self.assertEqual(TIMING.tq_per_bit, 12)
        self.assertEqual(TIMING.sample_point, 8)

    def test_rejects_bool_and_out_of_range(self):
        with self.assertRaises(ValidationError):
            BitTiming(prop_seg=True, phase_seg1=4, phase_seg2=4, sjw=2)
        with self.assertRaises(ValidationError):
            BitTiming(prop_seg=0, phase_seg1=4, phase_seg2=4, sjw=2)
        with self.assertRaises(ValidationError):
            BitTiming(prop_seg=3, phase_seg1=4, phase_seg2=4, sjw=5)  # > phase_seg2
        with self.assertRaises(ValidationError):
            BitTiming(prop_seg=3, phase_seg1=4, phase_seg2=4, sjw=2.5)

    def test_synthesize_validation(self):
        with self.assertRaises(ValidationError):
            synthesize_edges([0, 1, 2], TIMING)
        with self.assertRaises(ValidationError):
            synthesize_edges([0, True], TIMING)
        with self.assertRaises(ValidationError):
            synthesize_edges([0, 1], TIMING, skew_ppm=10**9)
        with self.assertRaises(ValidationError):  # 毛刺只能落在隐性位
            synthesize_edges([0, 1], TIMING, glitches=[Glitch(0, 1, 1)])

    def test_synthesize_deterministic(self):
        a = synthesize_edges(ideal_stream(), TIMING, skew_ppm=5000)
        b = synthesize_edges(ideal_stream(), TIMING, skew_ppm=5000)
        self.assertEqual(a, b)
        self.assertTrue(all(isinstance(e.time, Fraction) for e in a))

    def test_receive_rejects_unsorted_edges(self):
        edges = [Edge(Fraction(10), 0), Edge(Fraction(5), 1)]
        with self.assertRaises(ValidationError):
            receive_edges(edges, TIMING)


class TestIdealRecovery(unittest.TestCase):
    def test_zero_skew_bit_exact(self):
        stream = ideal_stream()
        rep = receive_edges(synthesize_edges(stream, TIMING), TIMING)
        self.assertEqual(rep.status, "ok")
        self.assertEqual(rep.error, "")
        # 恢复位流与理想流逐位一致（解码在 CRC 收齐后即完成，
        # 因此比对理想流前缀；SOF..CRC 全部覆盖）
        self.assertEqual(list(rep.recovered_bits), stream[: len(rep.recovered_bits)])
        self.assertGreaterEqual(len(rep.recovered_bits), 1 + len(encode_frame(FRAME).stuffed))
        # 零偏差：没有任何相位修正
        self.assertTrue(all(s.correction == 0 for s in rep.samples))
        self.assertEqual(rep.total_correction, 0)
        # 帧起始硬同步恰好一次，且在第一位
        self.assertEqual(rep.samples[0].sync, "hard")
        self.assertFalse(any(s.sync == "hard" for s in rep.samples[1:]))
        self.assertFalse(any(s.sync == "resync" for s in rep.samples))
        # 采样时刻严格按位定时网格
        for i, s in enumerate(rep.samples):
            self.assertEqual(s.sample_time, Fraction(TIMING.sample_point + 12 * i))
        # 帧内容与发送帧一致
        self.assertIsNotNone(rep.decoded)
        self.assertEqual(rep.decoded.frame, FRAME)
        self.assertTrue(rep.decoded.crc_ok)

    def test_no_edges_reports_no_frame(self):
        rep = receive_edges([], TIMING)
        self.assertEqual(rep.status, "no_frame")
        self.assertFalse(rep.ok)


class TestSkewRecovery(unittest.TestCase):
    def _check_fixable(self, skew_ppm: int):
        stream = ideal_stream()
        rep = receive_edges(synthesize_edges(stream, TIMING, skew_ppm=skew_ppm), TIMING)
        self.assertEqual(rep.status, "ok", rep.error)
        self.assertEqual(rep.decoded.frame, FRAME)
        self.assertEqual(list(rep.recovered_bits), stream[: len(rep.recovered_bits)])
        resyncs = [s for s in rep.samples if s.sync == "resync"]
        self.assertTrue(resyncs, "expected real resync corrections")
        # 每次修正都以 sjw 为界；硬同步与帧内重同步分开标记
        self.assertTrue(all(abs(s.correction) <= TIMING.sjw for s in rep.samples))
        self.assertTrue(all(s.correction != 0 for s in resyncs))
        self.assertEqual(rep.samples[0].sync, "hard")
        self.assertFalse(any(s.sync == "hard" for s in rep.samples[1:]))
        return rep

    def test_positive_skew_fixable(self):
        rep = self._check_fixable(20_000)  # 发送时钟快 2%
        self.assertGreater(rep.total_correction, 0)  # 边沿滞后 -> 延长

    def test_negative_skew_fixable(self):
        rep = self._check_fixable(-20_000)  # 发送时钟慢 2%
        self.assertLess(rep.total_correction, 0)  # 边沿提前 -> 缩短

    def test_skew_beyond_correction_range(self):
        # 12% 偏差：两位之间累积漂移就超过 sjw=2，采样点滑出本位
        rep = receive_edges(
            synthesize_edges(ideal_stream(), TIMING, skew_ppm=120_000), TIMING
        )
        self.assertNotEqual(rep.status, "ok")
        self.assertFalse(rep.ok)
        self.assertIsNone(rep.decoded)
        self.assertTrue(rep.error)

    def test_negative_skew_beyond_range(self):
        rep = receive_edges(
            synthesize_edges(ideal_stream(), TIMING, skew_ppm=-120_000), TIMING
        )
        self.assertNotEqual(rep.status, "ok")


class TestGlitches(unittest.TestCase):
    def test_glitch_filtered_and_frame_recovered(self):
        stream = ideal_stream()
        recessive = [i for i, b in enumerate(stream) if b == 1]
        glitches = [Glitch(bit_index=recessive[0], offset_tq=2, width_tq=1),
                    Glitch(bit_index=recessive[1], offset_tq=9, width_tq=1)]
        edges = synthesize_edges(stream, TIMING, skew_ppm=10_000, glitches=glitches)
        # 毛刺确实生成了额外边沿
        clean = synthesize_edges(stream, TIMING, skew_ppm=10_000)
        self.assertEqual(len(edges), len(clean) + 4)
        rep = receive_edges(edges, TIMING, glitch_filter_tq=1)
        self.assertEqual(rep.status, "ok", rep.error)
        self.assertEqual(rep.decoded.frame, FRAME)
        self.assertEqual(list(rep.recovered_bits), stream[: len(rep.recovered_bits)])

    def test_no_false_hard_sync_from_glitch(self):
        # 帧前毛刺之外再无显性脉冲时，硬同步仍落在真正的 SOF 边沿
        stream = ideal_stream()
        rep = receive_edges(synthesize_edges(stream, TIMING), TIMING)
        self.assertEqual(rep.samples[0].sample_time, Fraction(TIMING.sample_point))


class TestLegacyBitEntryKept(unittest.TestCase):
    def test_direct_bitstream_entry_unchanged(self):
        enc = encode_frame(FRAME)
        dec = decode_frame(enc.stuffed)  # 旧的直接位流入口
        self.assertEqual(dec.frame, FRAME)
        self.assertTrue(dec.crc_ok)


if __name__ == "__main__":
    unittest.main()

