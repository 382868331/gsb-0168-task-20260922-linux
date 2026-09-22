"""CRC、位填充与整帧编解码测试。

期望值来自 tests/reference.py 的独立整数长除法/独立状态机实现，
不调用被测核心产生期望值；往返一致只作为补充断言。
"""

import unittest

import reference as ref

from canbit import (
    CRC_MASK,
    Frame,
    bit_stuff,
    compute_crc15,
    decode_frame,
    destuff_bits,
    encode_frame,
    payload_bits,
)
from canbit.errors import FrameDecodeError, ValidationError

# 固定确定性样本：同 ID 不同负载必须产生不同 CRC/位流
SAME_ID = 0x123
PAYLOADS = [b"", b"\x00", b"\xff", b"CAN", bytes(range(8)), b"\xAA\x55\x0F\xF0"]

# 手工/构造的小向量（人可核对）
KNOWN_STUFF_CASES = [
    ([], []),
    ([0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1]),
    ([1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 0]),
    ([0, 0, 0, 0], [0, 0, 0, 0]),
    # 填充位作为新游程第 1 位：00000[stuff1] 后再来四个数据 1 即满
    # 五个 1，须插 stuff0，第五个数据 1 在其后
    ([0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
     [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1]),
    # 六连 0：00000 插 1，随后 0 重新计 1
    ([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0]),
]


class CrcAgainstReferenceTests(unittest.TestCase):
    def test_crc_matches_independent_long_division(self):
        rng = 0xBEEF  # 线性同余，确定性、无 random 依赖
        cases = []
        for dlc in range(9):
            for _ in range(12):
                rng = (rng * 1103515245 + 12345) & 0x7FFFFFFF
                ident = rng & 0x7FF
                data = bytes(((rng >> (k % 24)) & 0xFF) for k in range(dlc))
                cases.append((ident, dlc, data))
        for ident, dlc, data in cases:
            with self.subTest(ident=ident, dlc=dlc):
                self.assertEqual(
                    compute_crc15(ident, dlc, data),
                    ref.ref_crc15(ident, dlc, data),
                )

    def test_crc_in_range(self):
        for ident, dlc, data in [(0, 0, b""), (0x7FF, 8, bytes(8))]:
            self.assertIsInstance(compute_crc15(ident, dlc, data), int)
            self.assertLessEqual(compute_crc15(ident, dlc, data), CRC_MASK)

    def test_same_id_different_payload_differ(self):
        crcs = {compute_crc15(SAME_ID, len(p), p) for p in PAYLOADS}
        self.assertGreater(len(crcs), 1)
        # 与独立参考逐一相符
        for p in PAYLOADS:
            self.assertEqual(
                compute_crc15(SAME_ID, len(p), p),
                ref.ref_crc15(SAME_ID, len(p), p),
            )

    def test_frame_dlc_is_data_length(self):
        f = Frame(0x100, b"AB")
        self.assertEqual(f.dlc, 2)


class StuffTests(unittest.TestCase):
    def test_known_vectors(self):
        for src, expected in KNOWN_STUFF_CASES:
            with self.subTest(src=src):
                self.assertEqual(bit_stuff(src), expected)

    def test_no_six_equal_raw_in_output(self):
        # 正确填充流的任意窗口不含 6 连同值（原始位间必有填充位切断）
        for raw in [[0] * 40, [1] * 40, [0, 1] * 20, [1, 0, 0, 0, 0, 0, 1, 1] * 5]:
            stuffed = bit_stuff(raw)
            for i in range(len(stuffed) - 5):
                window = stuffed[i : i + 6]
                self.assertFalse(
                    all(b == window[0] for b in window),
                    f"six equal bits at {i} in {stuffed}",
                )

    def test_roundtrip_supplement(self):
        # 往返一致仅作补充
        raw = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0]
        self.assertEqual(destuff_bits(bit_stuff(raw)).bits, raw)

    def test_matches_reference_stuffer(self):
        for raw in [[0] * 20, [1] * 20, [0, 1] * 20, [0, 0, 0, 0, 0, 1] * 6]:
            self.assertEqual(bit_stuff(raw), ref.ref_stuff(raw))

    def test_corrupted_stuff_bit_detected(self):
        frame = Frame(0x001, bytes(8))  # 低 ID + 全 0 数据必然产生填充位
        enc = encode_frame(frame)
        stuff_positions = [
            i for i, m in enumerate(enc.stuffing_map) if m == 1
        ]
        self.assertTrue(stuff_positions, "test frame should contain stuffing bits")
        corrupted = list(enc.stuffed)
        pos = stuff_positions[0]
        # 填充位应为前一位的反位；把它改成同位即破坏
        corrupted[pos] = enc.stuffed[pos - 1]
        res = destuff_bits(corrupted)
        self.assertEqual(res.error_index, pos)
        with self.assertRaises(FrameDecodeError) as cm:
            decode_frame(corrupted)
        self.assertEqual(cm.exception.kind, "stuff")
        self.assertEqual(cm.exception.bit_index, pos)

    def test_truncated_missing_stuff_bit(self):
        # 末尾恰为 5 连同值、缺填充位：报流尾
        res = destuff_bits([0, 0, 0, 0, 0])
        self.assertEqual(res.error_index, 5)

    def test_stuff_input_validation(self):
        with self.assertRaises(ValidationError):
            bit_stuff([0, 1, True])
        with self.assertRaises(ValidationError):
            bit_stuff([2])


class EncodeDecodeTests(unittest.TestCase):
    def test_full_encode_matches_reference(self):
        for ident, data in [(0x000, b""), (0x7FF, bytes(range(8))),
                            (0x2B5, b"CAN-BIT"), (0x123, bytes([0xA5] * 8))]:
            with self.subTest(ident=hex(ident)):
                enc = encode_frame(Frame(ident, data))
                r = ref.ref_encode_stream(ident, len(data), data)
                self.assertEqual(enc.crc, r["crc"])
                self.assertEqual(enc.stuffed, r["stuffed"])
                self.assertEqual(enc.stream, r["stream"])
                self.assertEqual(enc.ack_index, r["ack_index"])
                # SOF 固定显性、ACK 释放隐性、EOF 隐性
                self.assertEqual(enc.stream[0], 0)
                self.assertEqual(enc.stream[enc.ack_index], 1)
                self.assertEqual(enc.stream[enc.eof_index], 1)

    def test_decode_good_frame(self):
        f = Frame(0x345, b"\xDE\xAD\xBE\xEF")
        enc = encode_frame(f)
        dec = decode_frame(enc.stuffed)
        self.assertEqual(dec.frame, f)
        self.assertTrue(dec.crc_ok)

    def test_roundtrip_all_dlc_supplement(self):
        # 往返一致：补充性断言（期望值不是它自己产生的——帧是固定输入）
        for dlc in range(9):
            f = Frame(dlc * 37, bytes((i * 17 + dlc) & 0xFF for i in range(dlc)))
            dec = decode_frame(encode_frame(f).stuffed)
            self.assertEqual(dec.frame, f)

    def test_crc_error_detected(self):
        f = Frame(0x100, b"abcd")
        enc = encode_frame(f)
        stuffed = list(enc.stuffed)
        # 翻转 CRC 区最后一位（在填充映射里找最后一个 CRC 原始位）
        raw_total = len(enc.raw_bits) + 15
        crc_raw_index = raw_total - 1
        raw_no = -1
        target = None
        for off, is_stuff in enumerate(enc.stuffing_map):
            if not is_stuff:
                raw_no += 1
                if raw_no == crc_raw_index:
                    target = off
                    break
        self.assertIsNotNone(target)
        stuffed[target] ^= 1
        # 若翻转后恰好引入填充违例，stuff 先报；否则必须是 crc
        with self.assertRaises(FrameDecodeError) as cm:
            decode_frame(stuffed)
        self.assertIn(cm.exception.kind, ("crc", "stuff"))

    def test_crc_error_constructed_via_reference_crc(self):
        # 构造一帧：正确填充结构，但 CRC 字段换成另一条消息的 CRC，
        # 从而确定性地只触发 crc 错误（不触碰填充）。
        ident, data = 0x201, b"payload!"
        good = encode_frame(Frame(ident, data))
        wrong_crc = ref.ref_crc15(ident, 8, b"payload?")  # 长度相同
        self.assertNotEqual(wrong_crc, good.crc)
        # 用独立参考重新拼出 msg + 错误CRC，再用本库填充器填充
        msg = payload_bits(ident, len(data), data)
        wrong_crc_bits = ref.ref_int_to_bits(wrong_crc, 15)
        # 保证错误 CRC 位串本身不制造填充问题：解码后只可能是 crc 错
        stuffed = bit_stuff(msg + wrong_crc_bits)
        with self.assertRaises(FrameDecodeError) as cm:
            decode_frame(stuffed)
        self.assertEqual(cm.exception.kind, "crc")
        dec = decode_frame(stuffed, raise_on_crc=False)
        self.assertFalse(dec.crc_ok)
        self.assertEqual(dec.frame, Frame(ident, data))

    def test_dlc_too_large_rejected(self):
        # 手工构造 DLC=9 的原始位串：11 ID + 1001 + 8*9 数据 + 15 CRC
        ident_bits = ref.ref_int_to_bits(0x010, 11)
        dlc_bits = ref.ref_int_to_bits(9, 4)
        data_bits = ref.ref_int_to_bits(0, 72)
        crc_bits = ref.ref_int_to_bits(0, 15)
        stuffed = bit_stuff(ident_bits + dlc_bits + data_bits + crc_bits)
        with self.assertRaises(FrameDecodeError) as cm:
            decode_frame(stuffed)
        self.assertEqual(cm.exception.kind, "protocol")

    def test_truncated_stream(self):
        with self.assertRaises(FrameDecodeError):
            decode_frame([0, 1, 0, 1])


if __name__ == "__main__":
    unittest.main()
