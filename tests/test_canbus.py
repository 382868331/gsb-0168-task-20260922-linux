"""canbus 模块测试。

ref_encode / ref_stuff 是独立编写的整帧编码参考（基于字符串操作），
不调用 canbus 核心实现，用于交叉验证 encode_frame / crc15。
"""

import unittest

import canbus
from canbus import (
    BusSimulation,
    CanBusError,
    CrcError,
    Frame,
    StuffError,
    crc15,
    decode_stream,
    destuff_bits,
    encode_frame,
    encode_unstuffed,
    simulate,
    stuff_bits,
)

REF_POLY = "1100010110011001"  # 0xC599，独立写出的生成多项式


def ref_crc(bitstr: str) -> str:
    """独立参考：字符串位串左移 15 位后对多项式做模 2 除法。"""
    work = list(bitstr + "0" * 15)
    for i in range(len(bitstr)):
        if work[i] == "1":
            for j in range(16):
                work[i + j] = "1" if work[i + j] != REF_POLY[j] else "0"
    return "".join(work[-15:])


def ref_stuff(bitstr: str) -> str:
    """独立参考：每 5 连同位插入反位。"""
    out = []
    run = 0
    prev = ""
    for b in bitstr:
        out.append(b)
        if b == prev:
            run += 1
        else:
            run = 1
            prev = b
        if run == 5:
            out.append("1" if b == "0" else "0")
            run = 0
            prev = ""
    return "".join(out)


def ref_encode(can_id: int, data: bytes) -> str:
    """独立整帧编码参考：填充后 ID..CRC + ACK 释放位。"""
    bits = format(can_id, "011b") + format(len(data), "04b")
    bits += "".join(format(byte, "08b") for byte in data)
    bits += ref_crc(bits)
    return ref_stuff(bits) + "1"  # ACK 槽发送端释放（隐性 1）


def to_bits(s: str):
    return [int(c) for c in s]


class TestCrc(unittest.TestCase):
    def test_known_vectors(self):
        # 空手推导：空串与全零串余数为 0；[1] << 15 = 0x8000，
        # 0x8000 ^ 0xC599 = 0x4599。
        self.assertEqual(crc15([]), 0)
        self.assertEqual(crc15([0] * 15), 0)
        self.assertEqual(crc15([1]), 0x4599)

    def test_matches_independent_reference(self):
        for can_id, data in [(0x000, b""), (0x123, b"\xDE\xAD"),
                             (0x7FF, bytes(range(8))), (0x2A0, b"\xFF" * 8)]:
            bitstr = format(can_id, "011b") + format(len(data), "04b")
            bitstr += "".join(format(b, "08b") for b in data)
            self.assertEqual(crc15(to_bits(bitstr)), int(ref_crc(bitstr), 2))


class TestEncoding(unittest.TestCase):
    FRAMES = [
        Frame(0x000, b""),
        Frame(0x7FF, b"\xFF" * 8),          # 长 1 串，填充密集
        Frame(0x000, b"\x00" * 8),          # 长 0 串
        Frame(0x2A0, b"\x11\x22\x33"),
        Frame(0x555, b"\xAA\x55\xAA\x55\xAA"),
    ]

    def test_encode_matches_independent_reference(self):
        for f in self.FRAMES:
            self.assertEqual(list(encode_frame(f)),
                             to_bits(ref_encode(f.id, f.data)))

    def test_unstuffed_layout(self):
        f = Frame(0x2A0, b"\x11\x22\x33")
        raw = encode_unstuffed(f)
        self.assertEqual(len(raw), 11 + 4 + 24 + 15)
        self.assertEqual(list(raw[:11]), to_bits(format(0x2A0, "011b")))
        self.assertEqual(list(raw[11:15]), to_bits("0011"))

    def test_stuff_roundtrip(self):
        bits = to_bits("0000011111101010")
        self.assertEqual(list(destuff_bits(stuff_bits(bits))), bits)

    def test_stuff_inserts_after_five(self):
        self.assertEqual(list(stuff_bits(to_bits("00000"))),
                         to_bits("000001"))
        self.assertEqual(list(stuff_bits(to_bits("1111100000"))),
                         to_bits("111110000001"))

    def test_destuff_rejects_six_run(self):
        with self.assertRaises(StuffError):
            destuff_bits(to_bits("000000"))
        with self.assertRaises(StuffError):
            destuff_bits(to_bits("1111110"))

    def test_destuff_rejects_truncated_run(self):
        with self.assertRaises(StuffError):
            destuff_bits(to_bits("00000"))  # 结尾应有填充位

    def test_roundtrip_decode(self):
        for f in self.FRAMES:
            wire = encode_frame(f)[:-1]  # 去掉 ACK 槽
            self.assertEqual(decode_stream(wire), f)


class TestArbitration(unittest.TestCase):
    def test_two_ids_lower_wins(self):
        a = Frame(0x2A0, b"\x01\x02")
        b = Frame(0x2A1, b"\x03\x04")
        result = simulate([a, b], ack_pull=True)
        self.assertTrue(result.ok)
        self.assertEqual(result.winners, (0,))
        self.assertEqual(result.frame, a)
        self.assertTrue(result.ack)
        # 失败者应在 ID 最低位（两 ID 首个不同位，槽 10）发送后退出。
        self.assertEqual(result.events[10].sent[1], 1)   # 仍参与该槽
        self.assertIsNone(result.events[11].sent[1])     # 下一槽起退出

    def test_identical_frames_both_win(self):
        f = Frame(0x100, b"\xAB")
        result = simulate([f, f, f], ack_pull=True)
        self.assertTrue(result.ok)
        self.assertEqual(result.winners, (0, 1, 2))
        self.assertEqual(result.frame, f)

    def test_stepwise_bus_matches_encoding(self):
        f = Frame(0x2A0, b"\x11\x22\x33")
        sim = BusSimulation([f], ack_pull=True)
        events = []
        while not sim.done:
            events.append(sim.step())
        wire = encode_frame(f)
        self.assertEqual(len(events), len(wire))
        self.assertEqual([ev.bus_value for ev in events[:-1]],
                         list(wire[:-1]))
        self.assertEqual(events[-1].field, "ack")
        self.assertEqual(events[-1].bus_value, 0)  # 接收者拉低
        self.assertTrue(sim.result.ok)

    def test_step_after_done_raises(self):
        sim = BusSimulation([Frame(1)], ack_pull=True)
        while not sim.done:
            sim.step()
        with self.assertRaises(RuntimeError):
            sim.step()


class TestProtocolErrors(unittest.TestCase):
    def test_same_id_different_payload_bit_error(self):
        a = Frame(0x2A0, b"\x00")
        b = Frame(0x2A0, b"\x80")  # 数据首位不同
        result = simulate([a, b], ack_pull=True)
        self.assertFalse(result.ok)
        err = result.error
        self.assertEqual(err.kind, "bit_error")
        self.assertEqual(err.field, "data")
        self.assertEqual(err.participants, (0, 1))
        # 出错槽 = 两个发送位流首个不同位置（独立参考计算）。
        wa, wb = ref_encode(a.id, a.data), ref_encode(b.id, b.data)
        first_diff = next(i for i, (x, y) in enumerate(zip(wa, wb))
                          if x != y)
        self.assertEqual(err.slot, first_diff)
        self.assertIsNone(result.frame)
        self.assertIsNone(result.ack)

    def test_ack_error_when_nobody_pulls(self):
        f = Frame(0x2A0, b"\x01")
        result = simulate([f], ack_pull=False)
        self.assertFalse(result.ok)
        self.assertEqual(result.error.kind, "ack_error")
        self.assertEqual(result.error.field, "ack")
        self.assertEqual(result.error.participants, (0,))
        self.assertEqual(result.events[-1].field, "ack")
        self.assertEqual(result.events[-1].bus_value, 1)

    def test_stuff_error_on_corrupted_stream(self):
        # 构造含 5 连同位的帧，删去填充位制造 6 连同位。
        f = Frame(0x000, b"\x00" * 2)
        wire = list(encode_frame(f)[:-1])
        # 独立参考：填充流与未填充流首个不同处即第一个填充位。
        raw = format(f.id, "011b") + format(len(f.data), "04b")
        raw += "".join(format(b, "08b") for b in f.data)
        raw += ref_crc(raw)
        ref = ref_stuff(raw)
        stuffed_at = next(i for i, (x, y) in enumerate(zip(ref, raw))
                          if x != y)
        del wire[stuffed_at]
        with self.assertRaises(StuffError):
            decode_stream(wire)

    def test_crc_error_on_corrupted_stream(self):
        # 独立构造：正确位串 + 错误 CRC，再填充，解码应报 CrcError。
        f = Frame(0x123, b"\xAB\xCD")
        raw = format(f.id, "011b") + format(len(f.data), "04b")
        raw += "".join(format(b, "08b") for b in f.data)
        bad_crc = format(int(ref_crc(raw), 2) ^ 0x0001, "015b")
        wire = to_bits(ref_stuff(raw + bad_crc))
        with self.assertRaises(CrcError):
            decode_stream(wire)

    def test_decode_rejects_bad_dlc_and_length(self):
        raw = format(1, "011b") + "1001" + "0" * 15  # DLC=9 越界
        with self.assertRaises(CanBusError):
            decode_stream(to_bits(raw))
        raw = format(1, "011b") + "0001" + "0" * 15  # 缺 8 位数据
        with self.assertRaises(CanBusError):
            decode_stream(to_bits(raw))


class TestValidation(unittest.TestCase):
    def test_frame_id_validation(self):
        with self.assertRaises(TypeError):
            Frame(True)                      # 整数参数拒绝 bool
        with self.assertRaises(TypeError):
            Frame(1.5)
        with self.assertRaises(TypeError):
            Frame(float("nan"))              # NaN 拒绝
        with self.assertRaises(TypeError):
            Frame(float("inf"))              # Infinity 拒绝
        with self.assertRaises(ValueError):
            Frame(-1)
        with self.assertRaises(ValueError):
            Frame(0x800)                     # 超过 11 位

    def test_frame_data_validation(self):
        with self.assertRaises(TypeError):
            Frame(1, "ab")
        with self.assertRaises(ValueError):
            Frame(1, b"\x00" * 9)
        f = Frame(1, bytearray(b"\x01"))     # bytearray 归一为 bytes
        self.assertEqual(f.data, b"\x01")

    def test_bits_validation(self):
        with self.assertRaises(TypeError):
            crc15([True, False])             # 位串同样拒绝 bool
        with self.assertRaises(ValueError):
            crc15([0, 2])
        with self.assertRaises(TypeError):
            stuff_bits("0101")
        with self.assertRaises(TypeError):
            destuff_bits(b"\x00\x01")

    def test_simulation_validation(self):
        with self.assertRaises(ValueError):
            BusSimulation([])
        with self.assertRaises(ValueError):
            BusSimulation([Frame(i) for i in range(9)])
        with self.assertRaises(TypeError):
            BusSimulation([Frame(1)], ack_pull=1)   # 必须是 bool
        with self.assertRaises(TypeError):
            BusSimulation([Frame(1), "not a frame"])
        with self.assertRaises(TypeError):
            BusSimulation("not frames")

    def test_error_result_is_terminal(self):
        # 出错后传输中止：结果已确定，事件不再增长，不能留下半截状态。
        result = simulate([Frame(0x10, b"\x00"), Frame(0x10, b"\xFF")],
                          ack_pull=True)
        self.assertFalse(result.ok)
        sim = BusSimulation([Frame(0x10, b"\x00"), Frame(0x10, b"\xFF")],
                            ack_pull=True)
        while not sim.done:
            sim.step()
        self.assertEqual(len(sim.result.events),
                         result.error.slot + 1)
        with self.assertRaises(RuntimeError):
            sim.step()


if __name__ == "__main__":
    unittest.main()
