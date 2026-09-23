"""TEC/REC 错误计数测试：计分规则、阈值、离线禁发、显式复位。

期望值全部手算：发送错误 +8、接收错误 +1、相应成功 -1（到 0 止）、
任一计数 >=128 被动、TEC >=256 离线；不调用被测实现生成期望值。
"""

import unittest

from canbit import (
    BUS_OFF_THRESHOLD,
    ERROR_PASSIVE_THRESHOLD,
    BitBus,
    Frame,
    Node,
    Sender,
)
from canbit.errors import ValidationError

FRAME = Frame(0x123, b"x")


def run_round(*nodes, ack=(), senders=("A",), frame=FRAME):
    """跑一轮传输：senders 中的名字各发一帧，nodes 全部挂接。"""
    bus = BitBus(
        [Sender(name, frame) for name in senders],
        ack_pullers=set(ack),
        nodes=list(nodes),
    )
    return bus.run()


class CounterRuleTests(unittest.TestCase):
    def test_transmit_error_adds_8(self):
        # 无人 ACK -> ack_error，发送者 TEC 每轮 +8
        a = Node("A")
        run_round(a)  # 无 ack_pullers
        self.assertEqual(a.tec, 8)
        self.assertEqual(a.rec, 0)
        run_round(a)
        self.assertEqual(a.tec, 16)

    def test_receive_error_adds_1(self):
        # 监听节点不计 TEC；帧被错误帧中止 -> REC +1
        a, listener = Node("A"), Node("L")
        run_round(a, listener)
        self.assertEqual(a.tec, 8)
        self.assertEqual(listener.tec, 0)
        self.assertEqual(listener.rec, 1)

    def test_transmit_success_decrements_to_zero(self):
        a = Node("A", tec=3)
        for expected in (2, 1, 0, 0):  # 减 1 到 0 为止，不为负
            run_round(a, ack=("RX",))
            self.assertEqual(a.tec, expected)

    def test_receive_success_decrements_to_zero(self):
        a = Node("A")
        listener = Node("L", rec=2)
        for expected in (1, 0, 0):
            run_round(a, listener, ack=("RX",))
            self.assertEqual(listener.rec, expected)
        self.assertEqual(a.tec, 0)  # 发送成功时 TEC 从 0 保持 0

    def test_arbitration_loss_is_not_error(self):
        # A(0x400) 在 ID 首位发 1 读 0 输给 B(0x3FF)：仲裁丢失不加错
        a, b = Node("A", rec=4), Node("B", tec=10)
        bus = BitBus(
            [Sender("A", Frame(0x400, b"")), Sender("B", Frame(0x3FF, b""))],
            ack_pullers={"RX"},
            nodes=[a, b],
        )
        result = bus.run()
        self.assertEqual(result.winners, ("B",))
        self.assertEqual(a.tec, 0)          # 丢失仲裁：TEC 不加
        self.assertEqual(a.rec, 4 - 1)      # 败者转为接收者：接收成功 -1
        self.assertEqual(b.tec, 10 - 1)     # 胜者发送成功 -1
        self.assertEqual(b.rec, 0)
        self.assertEqual(a.state, "active")

    def test_bit_error_counts_all_aborted_senders(self):
        # 同 ID 不同数据：两个发送者都被中止，各记发送错误 +8
        a, b, listener = Node("A"), Node("B"), Node("L")
        bus = BitBus(
            [Sender("A", Frame(0x100, b"\x00")),
             Sender("B", Frame(0x100, b"\x01"))],
            nodes=[a, b, listener],
        )
        result = bus.run()
        self.assertEqual(result.error_frame.trigger, "bit_error")
        self.assertEqual((a.tec, b.tec), (8, 8))
        self.assertEqual(listener.rec, 1)

    def test_loser_then_error_counts_receive_error(self):
        # A 先输仲裁（不加错），随后 B 的帧无人 ACK 中止：
        # A 作为接收者 REC +1，B 作为被中止发送者 TEC +8
        a, b = Node("A"), Node("B")
        bus = BitBus(
            [Sender("A", Frame(0x400, b"")), Sender("B", Frame(0x3FF, b""))],
            nodes=[a, b],
        )
        result = bus.run()
        self.assertEqual(result.error_frame.trigger, "ack_error")
        self.assertEqual((a.tec, a.rec), (0, 1))
        self.assertEqual((b.tec, b.rec), (8, 0))

    def test_outcome_applied_exactly_once(self):
        a = Node("A")
        bus = BitBus([Sender("A", FRAME)], nodes=[a])
        bus.run()
        bus.result()
        bus.result()  # 重复取结果不重复计分
        self.assertEqual(a.tec, 8)


class ThresholdTests(unittest.TestCase):
    def test_passive_threshold_tec(self):
        self.assertEqual(Node("A", tec=127).state, "active")
        self.assertEqual(Node("A", tec=128).state, "passive")
        a = Node("A", tec=120)
        run_round(a)  # 120 + 8 = 128，恰好进入被动
        self.assertEqual(a.tec, 128)
        self.assertEqual(a.state, "passive")

    def test_passive_threshold_rec(self):
        self.assertEqual(Node("A", rec=127).state, "active")
        self.assertEqual(Node("A", rec=128).state, "passive")
        listener = Node("L", rec=127)
        run_round(Node("A"), listener)  # 127 + 1 = 128
        self.assertEqual(listener.rec, 128)
        self.assertEqual(listener.state, "passive")

    def test_bus_off_threshold_tec_only(self):
        self.assertEqual(Node("A", tec=255).state, "passive")
        self.assertEqual(Node("A", tec=256).state, "bus_off")
        # REC 再高也不离线（只进被动）
        self.assertEqual(Node("A", rec=1000).state, "passive")
        a = Node("A", tec=248)
        run_round(a)  # 248 + 8 = 256，恰好离线
        self.assertEqual(a.tec, 256)
        self.assertEqual(a.state, "bus_off")

    def test_passive_returns_to_active_on_success(self):
        a = Node("A", tec=128)
        self.assertEqual(a.state, "passive")
        run_round(a, ack=("RX",))  # 128 - 1 = 127
        self.assertEqual(a.tec, 127)
        self.assertEqual(a.state, "active")

    def test_full_lifecycle_active_to_bus_off(self):
        # 连续无人 ACK：TEC = 8*n；n=16 -> 128 被动，n=32 -> 256 离线
        a = Node("A")
        self.assertEqual(a.state, "active")
        for n in range(1, 33):
            run_round(a)
            self.assertEqual(a.tec, 8 * n)
            if n < 16:
                self.assertEqual(a.state, "active")
            elif n < 32:
                self.assertEqual(a.state, "passive")
            else:
                self.assertEqual(a.state, "bus_off")

    def test_constants_match_contract(self):
        self.assertEqual(ERROR_PASSIVE_THRESHOLD, 128)
        self.assertEqual(BUS_OFF_THRESHOLD, 256)


class BusOffAndResetTests(unittest.TestCase):
    def test_bus_off_sender_rejected_and_state_untouched(self):
        a = Node("A", tec=256)
        with self.assertRaises(ValidationError) as cm:
            BitBus([Sender("A", FRAME)], nodes=[a])
        self.assertEqual(cm.exception.field, "nodes")
        self.assertEqual((a.tec, a.rec), (256, 0))  # 失败不留半更新状态

    def test_bus_off_listener_not_counted(self):
        # 离线节点不参与总线：成功/失败轮都不计它
        off = Node("OFF", tec=256, rec=7)
        a = Node("A")
        run_round(a, off, ack=("RX",))
        self.assertEqual((off.tec, off.rec), (256, 7))
        run_round(a, off)  # 错误轮同样不计
        self.assertEqual((off.tec, off.rec), (256, 7))

    def test_reset_clears_and_allows_transmit(self):
        a = Node("A", tec=256, rec=130)
        self.assertEqual(a.state, "bus_off")
        a.reset()
        self.assertEqual((a.tec, a.rec), (0, 0))
        self.assertEqual(a.state, "active")
        run_round(a, ack=("RX",))  # 复位后可以正常发送
        self.assertEqual((a.tec, a.rec), (0, 0))

    def test_reset_anytime_clears(self):
        a = Node("A", tec=100, rec=50)
        a.reset()
        self.assertEqual((a.tec, a.rec, a.state), (0, 0, "active"))


class CounterContractTests(unittest.TestCase):
    def test_node_validation(self):
        for bad_name in ("", 1, None):
            with self.assertRaises(ValidationError):
                Node(bad_name)  # type: ignore[arg-type]
        for bad in (-1, True, 2.5, float("nan"), float("inf"), "8"):
            with self.assertRaises(ValidationError):
                Node("A", tec=bad)  # type: ignore[arg-type]
            with self.assertRaises(ValidationError):
                Node("A", rec=bad)  # type: ignore[arg-type]

    def test_bus_nodes_validation(self):
        a = Node("A")
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", FRAME)], nodes=[a, Node("A")])  # 重名
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", FRAME)], nodes=[Node("B")])  # 发送者无 Node
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", FRAME)], nodes=["not-a-node"])  # type: ignore[list-item]
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", FRAME)], nodes=[])  # 空集合等于未挂接发送者

    def test_mixed_sequence_hand_computed(self):
        # 手算序列：A 交替 错误/成功，L 全程监听
        # A.tec: 0 -E-> 8 -S-> 7 -E-> 15 -S-> 14
        # L.rec: 0 -E-> 1 -S-> 0 -E-> 1 -S-> 0
        a, listener = Node("A"), Node("L")
        expected = [(8, 1), (7, 0), (15, 1), (14, 0)]
        for k, (tec, rec) in enumerate(expected):
            run_round(a, listener, ack=("RX",) if k % 2 else ())
            self.assertEqual((a.tec, listener.rec), (tec, rec))


if __name__ == "__main__":
    unittest.main()
