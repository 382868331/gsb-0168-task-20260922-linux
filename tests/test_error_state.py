"""TEC/REC 错误计数测试：+8/+1/-1 规则、128 被动、256 离线、复位、契约。

期望值来源（不依赖被测实现生成）：
* 每次 ack_error 使唯一发送者 TEC += 8，故 k 次后 TEC = 8*k（纯算术）；
  8*15=120 < 128（active），8*16=128（被动边界），8*31=248（仍被动），
  8*32=256（离线边界）。
* 仲裁失败者观察错误帧 REC += 1；干净完成 REC -= 1（下限 0）。
* 注入位置用 encode_frame 定位（与既有测试同一做法），但计数与状态
  阈值断言全部来自上述手算关系。
"""

import unittest

from canbit import (
    BUS_OFF_THRESHOLD,
    ERROR_PASSIVE_THRESHOLD,
    RX_ERROR_PENALTY,
    TX_ERROR_PENALTY,
    BitBus,
    ErrorStateSnapshot,
    ErrorTracker,
    Frame,
    Sender,
    encode_frame,
)
from canbit.errors import ValidationError

#  sanity：测试所依赖的契约常量与题面一致
assert TX_ERROR_PENALTY == 8
assert RX_ERROR_PENALTY == 1
assert ERROR_PASSIVE_THRESHOLD == 128
assert BUS_OFF_THRESHOLD == 256


def run_ack_error(tracker, name="A", frame=None):
    """单发送者、无人 ACK -> 必然 ack_error 的一轮。"""
    frame = frame if frame is not None else Frame(0x123, b"q")
    bus = BitBus([Sender(name, frame)], error_tracker=tracker)
    result = bus.run()
    assert result.error_frame is not None
    assert result.error_frame.trigger == "ack_error"
    return result


def forced_data_bit_index(frame):
    """frame 编码后数据区中一个发送 1 的线上索引（用于确定性故障注入）。"""
    enc = encode_frame(frame)
    for i in range(1, enc.ack_index):
        if i > 12 and enc.stream[i] == 1:  # 跳过 SOF 与 ID 域
            return i
    raise AssertionError("no recessive data bit found")


class TecAccumulationTests(unittest.TestCase):
    def test_ack_error_adds_8_per_round(self):
        tracker = ErrorTracker()
        for k in range(1, 6):
            run_ack_error(tracker)
            self.assertEqual(tracker.tec("A"), 8 * k)
            self.assertEqual(tracker.rec("A"), 0)
            self.assertEqual(tracker.state("A"), "active")

    def test_passive_threshold_exactly_128(self):
        tracker = ErrorTracker()
        for _ in range(15):
            run_ack_error(tracker)
        self.assertEqual(tracker.tec("A"), 120)
        self.assertEqual(tracker.state("A"), "active")
        run_ack_error(tracker)  # 第 16 次：120 + 8 = 128
        self.assertEqual(tracker.tec("A"), 128)
        self.assertEqual(tracker.state("A"), "passive")

    def test_bus_off_threshold_exactly_256(self):
        tracker = ErrorTracker()
        for _ in range(31):
            run_ack_error(tracker)
        self.assertEqual(tracker.tec("A"), 248)
        self.assertEqual(tracker.state("A"), "passive")
        run_ack_error(tracker)  # 第 32 次：248 + 8 = 256
        self.assertEqual(tracker.tec("A"), 256)
        self.assertEqual(tracker.state("A"), "bus_off")

    def test_bus_off_sender_rejected_and_state_untouched(self):
        tracker = ErrorTracker()
        for _ in range(32):
            run_ack_error(tracker)
        self.assertEqual(tracker.state("A"), "bus_off")
        # 离线节点禁止发送：构造即拒绝
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", Frame(0x123, b"q"))], error_tracker=tracker)
        # 拒绝后计数不变（无半更新状态）
        self.assertEqual(tracker.tec("A"), 256)
        # 多发送者中混入离线节点同样整体拒绝，且不伤及无辜节点
        tracker.register("B")
        with self.assertRaises(ValidationError):
            BitBus(
                [Sender("A", Frame(0x123)), Sender("B", Frame(0x122))],
                error_tracker=tracker,
            )
        self.assertEqual(tracker.tec("B"), 0)
        self.assertEqual(tracker.state("B"), "active")
        # 其他健康节点仍可正常发送
        ok = BitBus(
            [Sender("B", Frame(0x122, b"z"))],
            ack_pullers={"RX"},
            error_tracker=tracker,
        ).run()
        self.assertTrue(ok.completed)

    def test_reset_clears_counters_and_bus_off(self):
        tracker = ErrorTracker()
        for _ in range(32):
            run_ack_error(tracker)
        self.assertEqual(tracker.state("A"), "bus_off")
        tracker.reset("A")
        self.assertEqual(tracker.tec("A"), 0)
        self.assertEqual(tracker.rec("A"), 0)
        self.assertEqual(tracker.state("A"), "active")
        # 复位后可重新发送；成功时 TEC 已在 0，保持 0（下限）
        result = BitBus(
            [Sender("A", Frame(0x123, b"q"))],
            ack_pullers={"RX"},
            error_tracker=tracker,
        ).run()
        self.assertTrue(result.completed)
        self.assertEqual(tracker.tec("A"), 0)

    def test_reset_all_nodes(self):
        tracker = ErrorTracker()
        run_ack_error(tracker, "A")
        run_ack_error(tracker, "B", Frame(0x321, b"x"))
        self.assertEqual(tracker.tec("A"), 8)
        self.assertEqual(tracker.tec("B"), 8)
        tracker.reset()
        self.assertEqual(tracker.tec("A"), 0)
        self.assertEqual(tracker.tec("B"), 0)


class SuccessDecrementTests(unittest.TestCase):
    def test_tx_success_decrements_tec_to_zero_floor(self):
        tracker = ErrorTracker()
        run_ack_error(tracker)
        run_ack_error(tracker)
        self.assertEqual(tracker.tec("A"), 16)
        for k in range(1, 18):
            BitBus(
                [Sender("A", Frame(0x123, b"q"))],
                ack_pullers={"RX"},
                error_tracker=tracker,
            ).run()
            # 16 次成功把 16 减到 0，之后保持 0（不为负）
            self.assertEqual(tracker.tec("A"), max(16 - k, 0))

    def test_passive_recovers_to_active_on_success(self):
        tracker = ErrorTracker()
        for _ in range(16):
            run_ack_error(tracker)
        self.assertEqual(tracker.tec("A"), 128)
        self.assertEqual(tracker.state("A"), "passive")
        BitBus(
            [Sender("A", Frame(0x123, b"q"))],
            ack_pullers={"RX"},
            error_tracker=tracker,
        ).run()
        self.assertEqual(tracker.tec("A"), 127)
        self.assertEqual(tracker.state("A"), "active")


class ArbitrationAndRecTests(unittest.TestCase):
    def test_arbitration_loss_is_not_an_error(self):
        # A(0x321) 输给 B(0x320)，干净完成：A 的 TEC/REC 都不变
        tracker = ErrorTracker()
        result = BitBus(
            [Sender("A", Frame(0x321, b"hi")), Sender("B", Frame(0x320, b"yo"))],
            ack_pullers={"RX"},
            error_tracker=tracker,
        ).run()
        self.assertTrue(result.completed)
        self.assertEqual(result.winners, ("B",))
        self.assertEqual(tracker.tec("A"), 0)
        self.assertEqual(tracker.rec("A"), 0)
        self.assertEqual(tracker.tec("B"), 0)  # 成功从 0 减仍是 0
        self.assertEqual(tracker.state("A"), "active")

    def test_loser_observing_error_frame_gets_rec_plus_1(self):
        # A=0x7FF 必在首个 ID 位输给 B=0x000；随后外力破坏 B 的数据位
        tracker = ErrorTracker()
        frame_b = Frame(0x000, b"\xFF")
        target = forced_data_bit_index(frame_b)
        result = BitBus(
            [Sender("A", Frame(0x7FF)), Sender("B", frame_b)],
            forced_bits={target: 0},
            error_tracker=tracker,
        ).run()
        self.assertFalse(result.completed)
        self.assertEqual(result.error_frame.trigger, "bit_error")
        self.assertEqual(tracker.rec("A"), 1)   # 接收错误 +1
        self.assertEqual(tracker.tec("A"), 0)   # 仲裁丢失本身不加错
        self.assertEqual(tracker.tec("B"), 8)   # 发送错误 +8
        self.assertEqual(tracker.rec("B"), 0)

    def test_rec_reaches_128_passive_but_never_bus_off(self):
        # A 每轮都仲裁失败并观察到错误帧：REC 每轮 +1；对手每轮换新名，
        # 避免对手 TEC 累积离线（隔离 REC 单变量）
        tracker = ErrorTracker()
        frame_b = Frame(0x000, b"\xFF")
        target = forced_data_bit_index(frame_b)
        for k in range(1, 131):
            BitBus(
                [Sender("A", Frame(0x7FF)), Sender(f"C{k}", frame_b)],
                forced_bits={target: 0},
                error_tracker=tracker,
            ).run()
            self.assertEqual(tracker.rec("A"), k)
            self.assertEqual(tracker.tec("A"), 0)
            if k < 128:
                self.assertEqual(tracker.state("A"), "active", f"k={k}")
            else:
                # REC 只能导致被动，不会离线（题面：TEC 到 256 才离线）
                self.assertEqual(tracker.state("A"), "passive", f"k={k}")
        self.assertEqual(tracker.rec("A"), 130)

    def test_rx_success_decrements_rec_to_zero_floor(self):
        tracker = ErrorTracker()
        frame_b = Frame(0x000, b"\xFF")
        target = forced_data_bit_index(frame_b)
        for _ in range(2):  # REC = 2
            BitBus(
                [Sender("A", Frame(0x7FF)), Sender("B", frame_b)],
                forced_bits={target: 0},
                error_tracker=tracker,
            ).run()
            tracker.reset("B")  # 隔离变量：只观察 A 的 REC
        self.assertEqual(tracker.rec("A"), 2)
        for k in range(1, 4):  # 干净完成：A 接收成功 REC-1，下限 0
            BitBus(
                [Sender("A", Frame(0x7FF)), Sender("B", frame_b)],
                ack_pullers={"RX"},
                error_tracker=tracker,
            ).run()
            self.assertEqual(tracker.rec("A"), max(2 - k, 0))

    def test_both_active_senders_aborted_get_tec_plus_8(self):
        # 同 ID 不同数据：数据位失配时双方仍都在活跃发送 -> 都 TEC+8
        tracker = ErrorTracker()
        result = BitBus(
            [Sender("A", Frame(0x100, b"\x00")), Sender("B", Frame(0x100, b"\x01"))],
            error_tracker=tracker,
        ).run()
        self.assertEqual(result.error_frame.trigger, "bit_error")
        self.assertEqual(tracker.tec("A"), 8)
        self.assertEqual(tracker.tec("B"), 8)
        self.assertEqual(tracker.rec("A"), 0)
        self.assertEqual(tracker.rec("B"), 0)


class TrackerContractTests(unittest.TestCase):
    def test_constructor_name_validation(self):
        ErrorTracker(["ok", "also_ok"])
        with self.assertRaises(ValidationError):
            ErrorTracker([""])
        with self.assertRaises(ValidationError):
            ErrorTracker([123])  # type: ignore[list-item]
        with self.assertRaises(ValidationError):
            ErrorTracker().register("")

    def test_unknown_node_rejected(self):
        tracker = ErrorTracker(["A"])
        for op in (
            tracker.tec,
            tracker.rec,
            tracker.state,
            tracker.snapshot,
            tracker.reset,
        ):
            with self.assertRaises(ValidationError):
                op("ghost")
        with self.assertRaises(ValidationError):
            tracker.tec("")

    def test_register_is_idempotent_and_keeps_counters(self):
        tracker = ErrorTracker()
        run_ack_error(tracker)
        tracker.register("A")  # 重复登记不清零
        self.assertEqual(tracker.tec("A"), 8)

    def test_bus_rejects_non_tracker(self):
        with self.assertRaises(ValidationError):
            BitBus(
                [Sender("A", Frame(0))],
                error_tracker="not-a-tracker",  # type: ignore[arg-type]
            )

    def test_bus_auto_registers_senders(self):
        tracker = ErrorTracker()
        self.assertFalse(tracker.is_registered("A"))
        BitBus([Sender("A", Frame(0))], ack_pullers={"RX"}, error_tracker=tracker)
        self.assertTrue(tracker.is_registered("A"))

    def test_snapshot_contents(self):
        tracker = ErrorTracker()
        run_ack_error(tracker)
        snap = tracker.snapshot("A")
        self.assertEqual(snap, ErrorStateSnapshot(tec=8, rec=0, state="active"))
        all_snaps = tracker.snapshots()
        self.assertEqual(set(all_snaps), {"A"})
        self.assertEqual(all_snaps["A"].tec, 8)

    def test_tracker_does_not_change_bus_outcome(self):
        # 同一故障场景：有无 tracker 的逐位结果必须一致
        def scenario(tracker):
            kwargs = {"error_tracker": tracker} if tracker else {}
            return BitBus(
                [Sender("A", Frame(0x321, b"hi")), Sender("B", Frame(0x320, b"yo"))],
                ack_pullers={"RX"},
                **kwargs,
            ).run()

        plain = scenario(None)
        tracked = scenario(ErrorTracker())
        self.assertEqual(plain.winners, tracked.winners)
        self.assertEqual(plain.bus_bits, tracked.bus_bits)
        self.assertEqual(plain.events, tracked.events)


if __name__ == "__main__":
    unittest.main()
