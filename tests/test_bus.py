"""逐 bit 总线模型测试：仲裁、bit_error、ACK、错误帧、输入契约。

总线结果期望值来自 tests/reference.py 的独立一次性全量仿真；
逐 bit 步进语义、事件与错误帧由本文件直接断言。
"""

import unittest

import reference as ref

from canbit import BitBus, Frame, Sender, encode_frame
from canbit.errors import BusFinishedError, ValidationError


def frames_to_tuples(senders):
    return [(s.name, s.frame.identifier, s.frame.data) for s in senders]


class ArbitrationTests(unittest.TestCase):
    def test_two_ids_lower_id_wins(self):
        # 低 ID（数值小，更早出现显性 0）胜
        senders = [
            Sender("A", Frame(0x321, b"hi")),
            Sender("B", Frame(0x320, b"yo")),
        ]
        bus = BitBus(senders, ack_pullers={"RX"})
        result = bus.run()

        # 与独立参考一致
        ref_res = ref.ref_simulate_bus(
            frames_to_tuples(senders), ack_pullers={"RX"}
        )
        self.assertEqual(ref_res["outcome"], "complete")
        self.assertEqual(ref_res["winner"], ("B",))
        self.assertEqual(result.winners, ("B",))
        self.assertTrue(result.completed)
        self.assertTrue(result.acked)

        # 仲裁失败位置：两 ID 第一个不同位；独立参考给出同位置
        loss_events = [e for e in result.events if e.kind == "arbitration_loss"]
        self.assertEqual(len(loss_events), 1)
        self.assertEqual(loss_events[0].participants, ("A",))
        self.assertEqual(loss_events[0].bit_index, ref_res["losers"][0][1])
        # 手工核对：0x321=01100100001, 0x320=01100100000，首个差异为 ID 最后一位
        # （11 个 ID 位中的第 11 个），线上索引 = SOF + 10 = 11（其间无填充位）
        self.assertEqual(loss_events[0].bit_index, 11)

    def test_identical_ids_both_survive_then_data_diverges_biterror(self):
        # 同 ID：仲裁不分胜负；数据位一旦 1/0 失配，发 1 读 0 者报 bit_error
        senders = [
            Sender("A", Frame(0x100, b"\x00")),
            Sender("B", Frame(0x100, b"\x01")),
        ]
        bus = BitBus(senders)
        result = bus.run()
        ref_res = ref.ref_simulate_bus(frames_to_tuples(senders))
        self.assertEqual(ref_res["outcome"], "bit_error")
        self.assertIsNotNone(result.error_frame)
        self.assertEqual(result.error_frame.trigger, "bit_error")
        # A 数据全 0，B 末位发 1 读 0 -> B 报错（两实现一致）
        self.assertEqual(result.error_frame.participants, ref_res["participants"])
        self.assertEqual(result.error_frame.bit_index, ref_res["pos"])
        self.assertEqual(result.error_frame.participants, ("B",))
        self.assertIn("B", result.error_frame.aborted)
        self.assertIn("A", result.error_frame.aborted)

    def test_step_by_step_matches_batch_run(self):
        senders = [Sender("A", Frame(0x200, b"x")), Sender("B", Frame(0x1FF, b"y"))]
        bus = BitBus(senders, ack_pullers={"RX"})
        seen = []
        while not bus.finished:
            sr = bus.step()
            seen.append((sr.bit_index, sr.phase, sr.bus_bit))
        result = bus.result()
        self.assertEqual(result.winners, ("B",))
        # 第一位是 SOF=0
        self.assertEqual(seen[0], (0, "transmit", 0))
        phases = [p for _, p, _ in seen]
        self.assertIn("ack", phases)
        self.assertEqual(phases[-1], "done")

    def test_step_after_finish_raises(self):
        bus = BitBus([Sender("A", Frame(0, b""))], ack_pullers={"RX"})
        bus.run()
        with self.assertRaises(BusFinishedError):
            bus.step()

    def test_arbitration_with_stuffing_inside_id_field(self):
        # ID 0x000 vs 0x001：ID 前 5 个 0 后即出现填充位，仲裁仍须正确
        senders = [
            Sender("A", Frame(0x000, b"")),
            Sender("B", Frame(0x001, b"")),
        ]
        result = BitBus(senders, ack_pullers={"RX"}).run()
        ref_res = ref.ref_simulate_bus(
            frames_to_tuples(senders), ack_pullers={"RX"}
        )
        self.assertEqual(result.winners, ("A",))
        self.assertEqual(ref_res["winner"], ("A",))
        loss = [e for e in result.events if e.kind == "arbitration_loss"]
        self.assertEqual(loss[0].bit_index, ref_res["losers"][0][1])
        self.assertEqual(loss[0].participants, ("B",))

    def test_differential_against_reference_fixed_cases(self):
        # 固定 LCG 生成的对拍：两发送者 + 偶发外力注入 + 显式 ACK 集合
        seed = 0xC0FFEE
        cases = []
        for _ in range(24):
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            id1 = seed & 0x7FF
            d1 = seed % 4
            data1 = bytes((seed >> k) & 0xFF for k in range(0, 3 * d1, 3))[:d1]
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            id2 = seed & 0x7FF
            d2 = (seed >> 3) % 4
            data2 = bytes((seed >> k) & 0xFF for k in range(0, 3 * d2, 3))[:d2]
            cases.append((id1, data1, id2, data2))

        for n, (id1, data1, id2, data2) in enumerate(cases):
            senders = [
                Sender("A", Frame(id1, data1)),
                Sender("B", Frame(id2, data2)),
            ]
            forced = {}
            if n % 3 == 2:  # 确定性注入：在某数据区位置强制显性
                enc = encode_frame(senders[0].frame)
                # 取填充后流中段一个原始位
                mid = min(len(enc.stuffed) - 2, 20)
                if enc.stream[1 + mid] == 1:
                    forced = {1 + mid: 0}
            pullers = {"RX"} if n % 2 == 0 else set()

            result = BitBus(
                senders, ack_pullers=pullers, forced_bits=forced
            ).run()
            ref_res = ref.ref_simulate_bus(
                frames_to_tuples(senders),
                ack_pullers=pullers,
                forced_bits=forced,
            )

            if ref_res["outcome"] == "complete":
                self.assertTrue(
                    result.completed, f"case {n}: expected complete"
                )
                self.assertEqual(
                    result.winners, ref_res["winner"], f"case {n}"
                )
            else:
                self.assertFalse(
                    result.completed, f"case {n}: expected {ref_res['outcome']}"
                )
                self.assertEqual(
                    result.error_frame.trigger,
                    ref_res["outcome"],
                    f"case {n}",
                )
                self.assertEqual(
                    result.error_frame.bit_index, ref_res["pos"], f"case {n}"
                )

    def test_arbitration_loss_is_not_error_frame(self):
        senders = [Sender("A", Frame(0x400, b"")), Sender("B", Frame(0x3FF, b""))]
        result = BitBus(senders, ack_pullers={"RX"}).run()
        self.assertIsNone(result.error_frame)
        self.assertTrue(result.completed)
        self.assertEqual(result.winners, ("B",))


class ErrorAndAckTests(unittest.TestCase):
    def test_no_ack_puller_raises_ack_error(self):
        senders = [Sender("A", Frame(0x123, b"q"))]
        result = BitBus(senders).run()  # 无接收者拉低
        self.assertFalse(result.completed)
        self.assertIsNotNone(result.error_frame)
        self.assertEqual(result.error_frame.trigger, "ack_error")
        self.assertEqual(result.error_frame.participants, ("A",))
        enc = encode_frame(senders[0].frame)
        self.assertEqual(result.error_frame.bit_index, enc.ack_index)
        # 错误帧后有 6 个显性位
        flag_bits = [e for e in result.events if e.kind == "error_flag"]
        self.assertEqual(len(flag_bits), 6)
        self.assertTrue(all(e.bus_bit == 0 for e in flag_bits))

    def test_ack_error_agrees_with_reference(self):
        senders = [Sender("A", Frame(0x555, b"z"))]
        r = ref.ref_simulate_bus(frames_to_tuples(senders))
        self.assertEqual(r["outcome"], "ack_error")
        result = BitBus(senders).run()
        self.assertEqual(result.error_frame.bit_index, r["pos"])

    def test_stuff_bit_corruption_via_forced_bus(self):
        # 单发送者，外力把一个填充位强制成与前位相同 -> 发 1 读 0 / 发 0 读 1
        frame = Frame(0x001, bytes(8))
        enc = encode_frame(frame)
        stuff_positions = [i + 1 for i, m in enumerate(enc.stuffing_map) if m]
        self.assertTrue(stuff_positions)
        target = stuff_positions[0]
        sent = enc.stream[target]
        # 填充位是前位的反位；强制成前位值造成失配
        forced = {target: enc.stream[target - 1]}
        self.assertNotEqual(sent, forced[target])

        result = BitBus([Sender("A", frame)], ack_pullers={"RX"},
                        forced_bits=forced).run()
        self.assertFalse(result.completed)
        self.assertEqual(result.error_frame.trigger, "bit_error")
        self.assertEqual(result.error_frame.bit_index, target)
        self.assertEqual(result.error_frame.participants, ("A",))

        # 独立参考同注入同结论
        r = ref.ref_simulate_bus(
            frames_to_tuples([Sender("A", frame)]),
            ack_pullers={"RX"}, forced_bits=forced,
        )
        self.assertEqual(r["outcome"], "bit_error")
        self.assertEqual(r["pos"], target)

    def test_crc_corrupt_frame_on_bus_aborts(self):
        # 发送者发的帧在线上被干扰一位（非 ID、非 ACK）-> bit_error 优先中止
        frame = Frame(0x201, b"payload!")
        enc = encode_frame(frame)
        # 选一个发隐性 1 的数据区位，强制 0：发 1 读 0 的经典位错误
        target = None
        for off, bit in enumerate(enc.stream):
            if 15 < off < enc.ack_index and bit == 1 and off not in (
            ):
                target = off
                break
        self.assertIsNotNone(target)
        result = BitBus(
            [Sender("A", frame)], ack_pullers={"RX"}, forced_bits={target: 0}
        ).run()
        self.assertEqual(result.error_frame.trigger, "bit_error")
        self.assertEqual(result.error_frame.bit_index, target)

    def test_eof_mismatch_is_bit_error_not_silent(self):
        # EOF 固定隐性 1；外力在 EOF 槽强制显性 -> 发 1 读 0 报 bit_error
        frame = Frame(0x100, b"z")
        enc = encode_frame(frame)
        result = BitBus(
            [Sender("A", frame)],
            ack_pullers={"RX"},
            forced_bits={enc.eof_index: 0},
        ).run()
        self.assertFalse(result.completed)
        self.assertEqual(result.error_frame.trigger, "bit_error")
        self.assertEqual(result.error_frame.bit_index, enc.eof_index)
        self.assertEqual(result.error_frame.participants, ("A",))

    def test_decode_crc_error_offline_view(self):        # 对总线视角的补充：收到一帧 CRC 被篡改但填充完好的位流，
        # decode_frame 必须在去填充后报 crc（独立参考提供错误 CRC）
        from canbit import bit_stuff, decode_frame, payload_bits
        from canbit.errors import FrameDecodeError

        ident, data = 0x300, b"01234567"
        wrong = ref.ref_crc15(ident, len(b"76543210"), b"76543210")
        msg = payload_bits(ident, 8, data)
        stuffed = bit_stuff(msg + ref.ref_int_to_bits(wrong, 15))
        with self.assertRaises(FrameDecodeError) as cm:
            decode_frame(stuffed)
        self.assertEqual(cm.exception.kind, "crc")


class ContractTests(unittest.TestCase):
    def test_bool_rejected_everywhere(self):
        with self.assertRaises(ValidationError):
            Frame(True, b"")
        # DLC 由 data 长度决定不可直接传 bool，但 compute_crc15 可以
        from canbit import compute_crc15
        with self.assertRaises(ValidationError):
            compute_crc15(True, 0, b"")
        with self.assertRaises(ValidationError):
            compute_crc15(0, True, b"")

    def test_nan_infinity_rejected(self):
        from canbit import compute_crc15
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.assertRaises(ValidationError):
                compute_crc15(bad, 0, b"")

    def test_ranges(self):
        with self.assertRaises(ValidationError):
            Frame(0x800, b"")
        with self.assertRaises(ValidationError):
            Frame(-1, b"")
        with self.assertRaises(ValidationError):
            Frame(0, b"x" * 9)
        with self.assertRaises(ValidationError):
            Frame(0, "abc")  # type: ignore[arg-type]

    def test_sender_validation(self):
        with self.assertRaises(ValidationError):
            Sender("", Frame(0))
        with self.assertRaises(ValidationError):
            Sender("A", b"not-a-frame")  # type: ignore[arg-type]

    def test_bus_validation(self):
        with self.assertRaises(ValidationError):
            BitBus([])
        dup = [Sender("A", Frame(0)), Sender("A", Frame(1))]
        with self.assertRaises(ValidationError):
            BitBus(dup)
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", Frame(0))], ack_pullers={""})
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", Frame(0))], forced_bits={-1: 0})
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", Frame(0))], forced_bits={1: 2})
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", Frame(0))], forced_bits={True: 0})

    def test_result_before_finish_rejected(self):
        bus = BitBus([Sender("A", Frame(0))], ack_pullers={"RX"})
        with self.assertRaises(ValidationError):
            bus.result()


if __name__ == "__main__":
    unittest.main()
