"""接收过滤表与节点层测试：固定过滤表、ACK 语义、监听模式、重叠规则。

覆盖题面要求的五个场景：全过滤仍有 ACK、只有监听者时 ack_error、
匹配帧 CRC 损坏、两个重叠规则只交付一次、仲裁失败节点转接收。
所有场景都逐 bit 驱动底层原总线（NodeBus.step -> BitBus.step），
并核对每个节点的接收记录；过滤匹配另与 tests/reference.py 的
独立逐位参考对拍。
"""

import unittest

import reference as ref

from canbit import (
    BitBus,
    BusNode,
    FilterRule,
    Frame,
    NodeBus,
    Sender,
    compute_crc15,
    encode_frame,
    encode_frame_with_crc,
)
from canbit.errors import BusFinishedError, ValidationError


def run_bit_by_bit(node_bus):
    """逐 bit 驱动到底，返回 NodeBusResult。"""
    while not node_bus.finished:
        node_bus.step()
    return node_bus.result()


class FilteredAckTests(unittest.TestCase):
    def test_all_filtered_out_still_acks(self):
        # 全过滤（空表）：不上交任何帧，但正常接收节点仍贡献 ACK
        nodes = [
            BusNode("TX", tx_frame=Frame(0x2A5, b"data")),
            BusNode("RX", filters=()),  # 空表 = 全过滤
        ]
        bus = NodeBus(nodes)
        result = run_bit_by_bit(bus)

        self.assertTrue(result.bus.completed)
        self.assertTrue(result.bus.acked)  # 被过滤掉也照常 ACK
        self.assertIsNone(result.bus.error_frame)

        rx = result.records_of("RX")
        self.assertEqual(len(rx), 1)  # 每节点每帧恰好一条记录
        rec = rx[0]
        self.assertTrue(rec.crc_ok)  # 过滤不跳过去填充与 CRC 校验
        self.assertEqual(rec.matched_rules, ())
        self.assertFalse(rec.delivered)
        self.assertIsNone(rec.rule_index)
        self.assertEqual(rec.frame, Frame(0x2A5, b"data"))
        self.assertEqual(result.deliveries_of("RX"), ())

    def test_only_listeners_cause_ack_error(self):
        # 只有监听者：永不 ACK -> 仍在发送者报 ack_error；监听者也不发错误
        nodes = [
            BusNode("TX", tx_frame=Frame(0x123, b"q")),
            BusNode("L1", filters=(FilterRule(0x123, 0x7FF),), listen_only=True),
            BusNode("L2", filters=(), listen_only=True),
        ]
        bus = NodeBus(nodes)
        result = run_bit_by_bit(bus)

        self.assertFalse(result.bus.completed)
        self.assertFalse(result.bus.acked)
        self.assertIsNotNone(result.bus.error_frame)
        self.assertEqual(result.bus.error_frame.trigger, "ack_error")
        self.assertEqual(result.bus.error_frame.participants, ("TX",))
        # 错误标志由发送者发出；监听者不出现在任何错误事件里
        self.assertNotIn("L1", result.bus.error_frame.participants)
        self.assertNotIn("L2", result.bus.error_frame.participants)
        # 传输在 ACK 槽中止，没有完整帧：任何节点都无接收记录
        self.assertEqual(result.records, ())

    def test_crc_corrupt_matching_frame_recorded_not_delivered(self):
        # 匹配过滤表的帧 CRC 损坏：传输仍完成（ACK 是位级行为），
        # 节点记录 crc_ok=False，不上交
        frame = Frame(0x2A5, b"payload!")
        good_crc = compute_crc15(frame.identifier, frame.dlc, frame.data)
        bad = encode_frame_with_crc(frame, good_crc ^ 0x0001)
        self.assertNotEqual(bad.crc, encode_frame(frame).crc)

        nodes = [
            BusNode("TX", tx_frame=frame),
            BusNode("RX", filters=(FilterRule(0x2A5, 0x7FF),)),
            BusNode("LIS", filters=(FilterRule(0x2A5, 0x7FF),), listen_only=True),
        ]
        bus = NodeBus(nodes, encoded_overrides={"TX": bad})
        result = run_bit_by_bit(bus)

        self.assertTrue(result.bus.completed)
        self.assertTrue(result.bus.acked)  # 正常节点仍 ACK
        self.assertFalse(result.bus.decoded_crc_ok)

        for name in ("RX", "LIS"):
            (rec,) = result.records_of(name)
            self.assertFalse(rec.crc_ok)  # 监听者也记录看到的校验结果
            self.assertEqual(rec.matched_rules, (0,))  # ID 仍命中规则
            self.assertFalse(rec.delivered)  # CRC 错不上交
            self.assertIsNone(rec.rule_index)


class OverlappingRuleTests(unittest.TestCase):
    def test_overlapping_rules_deliver_once_earliest_rule(self):
        # 两条重叠规则同时命中：只交付一次，依据取最早规则序号
        rules = (
            FilterRule(0x100, 0x700),  # 命中 0x100..0x1FF 中高位模式 001
            FilterRule(0x180, 0x780),  # 与规则 0 重叠（如 0x1A0 同时命中）
        )
        frame = Frame(0x1A0, b"\xAA\x55")
        # 手工核对两条规则都命中 0x1A0
        self.assertTrue(all(r.matches(0x1A0) for r in rules))

        nodes = [
            BusNode("TX", tx_frame=frame),
            BusNode("RX", filters=rules),
        ]
        result = run_bit_by_bit(NodeBus(nodes))

        (rec,) = result.records_of("RX")
        self.assertTrue(rec.crc_ok)
        self.assertEqual(rec.matched_rules, (0, 1))  # 重叠命中全部记录
        self.assertTrue(rec.delivered)
        self.assertEqual(rec.rule_index, 0)  # 交付依据 = 最早规则
        # 只交付一次：每节点每帧恰好一条记录
        self.assertEqual(len(result.deliveries_of("RX")), 1)

    def test_earliest_rule_wins_regardless_of_table_order(self):
        # 命中规则的交付序号随表内顺序变化：先匹配到的最小序号为准
        frame = Frame(0x1A0, b"x")
        nodes = [
            BusNode("TX", tx_frame=frame),
            BusNode("RX", filters=(FilterRule(0x180, 0x780),
                                   FilterRule(0x100, 0x700))),
        ]
        result = run_bit_by_bit(NodeBus(nodes))
        (rec,) = result.records_of("RX")
        self.assertEqual(rec.matched_rules, (0, 1))
        self.assertEqual(rec.rule_index, 0)


class ArbitrationLoserReceiverTests(unittest.TestCase):
    def test_arbitration_loser_becomes_receiver_and_acks(self):
        # 仲裁失败节点转接收：B 带发送帧但输给 A，随后作为接收者
        # 参与 ACK 并按自己的过滤表接收 A 的帧
        nodes = [
            BusNode("A", tx_frame=Frame(0x320, b"from-A")),
            BusNode(
                "B",
                filters=(FilterRule(0x320, 0x7FF),),
                tx_frame=Frame(0x321, b"from-B"),
            ),
        ]
        bus = NodeBus(nodes)
        # 逐 bit 驱动并观察仲裁失败事件
        loss_events = []
        while not bus.finished:
            sr = bus.step()
            loss_events.extend(sr.arbitration_losers)
        result = bus.result()

        self.assertEqual(loss_events, ["B"])  # B 在 ID 域发1读0退出
        self.assertEqual(result.bus.winners, ("A",))
        self.assertTrue(result.bus.completed)
        # 总线上只有 A、B：A 是胜者不 ACK，acked=True 证明 B 失败后
        # 转为接收者并拉低了 ACK 槽
        self.assertTrue(result.bus.acked)

        # B 的接收记录：收到 A 的帧并命中自己的规则 0，交付
        (rec_b,) = result.records_of("B")
        self.assertEqual(rec_b.frame, Frame(0x320, b"from-A"))
        self.assertTrue(rec_b.crc_ok)
        self.assertEqual(rec_b.matched_rules, (0,))
        self.assertTrue(rec_b.delivered)
        self.assertEqual(rec_b.rule_index, 0)

        # A 的记录同样存在（每节点都记录线上帧），但 A 无过滤规则不上交
        (rec_a,) = result.records_of("A")
        self.assertFalse(rec_a.delivered)
        self.assertEqual(rec_a.matched_rules, ())

    def test_loser_ack_matches_plain_bitbus_semantics(self):
        # 同一仲裁场景在裸 BitBus 上手工指定 ack_pullers={"B"}，
        # 两者总线位流应逐位一致（NodeBus 的动态转接收 = 显式接收者）
        senders = [Sender("A", Frame(0x320, b"from-A")),
                   Sender("B", Frame(0x321, b"from-B"))]
        plain = BitBus(senders, ack_pullers={"B"}).run()

        nodes = [
            BusNode("A", tx_frame=Frame(0x320, b"from-A")),
            BusNode("B", tx_frame=Frame(0x321, b"from-B")),
        ]
        layered = run_bit_by_bit(NodeBus(nodes))
        self.assertEqual(plain.bus_bits, layered.bus.bus_bits)
        self.assertEqual(plain.acked, layered.bus.acked)


class FilterMatchDifferentialTests(unittest.TestCase):
    def test_matches_agree_with_independent_reference(self):
        # 固定 LCG 生成 (frame_id, identifier, mask)，与逐位参考对拍
        seed = 0xF17E9
        for _ in range(200):
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            frame_id = seed & 0x7FF
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            identifier = seed & 0x7FF
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            mask = seed & 0x7FF
            got = FilterRule(identifier, mask).matches(frame_id)
            want = ref.ref_filter_match(frame_id, identifier, mask)
            self.assertEqual(
                got, want, f"id=0x{frame_id:03X} rule=({identifier:#05x},{mask:#05x})"
            )

    def test_mask_edge_cases(self):
        self.assertTrue(FilterRule(0x000, 0x000).matches(0x7FF))  # mask=0 全命中
        exact = FilterRule(0x123, 0x7FF)
        self.assertTrue(exact.matches(0x123))
        self.assertFalse(exact.matches(0x124))


class NodeContractTests(unittest.TestCase):
    def test_filter_rule_validation(self):
        with self.assertRaises(ValidationError):
            FilterRule(0x800, 0x7FF)  # id 超 11 位
        with self.assertRaises(ValidationError):
            FilterRule(0x123, 0x800)  # mask 超 11 位
        with self.assertRaises(ValidationError):
            FilterRule(-1, 0x7FF)
        with self.assertRaises(ValidationError):
            FilterRule(True, 0x7FF)  # bool 拒绝
        with self.assertRaises(ValidationError):
            FilterRule(0x123, 1.5)  # float 拒绝
        with self.assertRaises(ValidationError):
            FilterRule(0x123, 0x7FF).matches(0x800)

    def test_bus_node_validation(self):
        with self.assertRaises(ValidationError):
            BusNode("")
        with self.assertRaises(ValidationError):
            BusNode("N", filters=(FilterRule(0, 0), "not-a-rule"))
        with self.assertRaises(ValidationError):
            BusNode("N", listen_only=1)  # 非 bool 拒绝
        with self.assertRaises(ValidationError):
            BusNode("N", tx_frame="not-a-frame")
        with self.assertRaises(ValidationError):
            BusNode("N", listen_only=True, tx_frame=Frame(0))  # 监听者不发送

    def test_node_bus_validation(self):
        with self.assertRaises(ValidationError):
            NodeBus([])
        with self.assertRaises(ValidationError):
            NodeBus([BusNode("L", listen_only=True)])  # 没有发送者
        dup = [BusNode("A", tx_frame=Frame(0)), BusNode("A")]
        with self.assertRaises(ValidationError):
            NodeBus(dup)
        with self.assertRaises(ValidationError):
            NodeBus([BusNode("A", tx_frame=Frame(0))],
                    encoded_overrides={"GHOST": encode_frame(Frame(0))})

    def test_filter_table_frozen_during_transmission(self):
        # 过滤表一次传输期间不可修改：frozen 配置 + 构造期快照
        node = BusNode("RX", filters=(FilterRule(0x100, 0x7FF),))
        with self.assertRaises(Exception):  # FrozenInstanceError
            node.filters = (FilterRule(0x200, 0x7FF),)
        with self.assertRaises(Exception):
            node.filters += (FilterRule(0x200, 0x7FF),)

        bus = NodeBus([BusNode("TX", tx_frame=Frame(0x100, b"")), node])
        table = bus.filter_table("RX")
        self.assertEqual(table, (FilterRule(0x100, 0x7FF),))
        with self.assertRaises(AttributeError):  # tuple 无修改入口
            table.append(FilterRule(0x200, 0x7FF))
        with self.assertRaises(ValidationError):
            bus.filter_table("GHOST")

    def test_add_ack_puller_contract(self):
        # 底层 BitBus 的动态 ACK 钩子：结束后/过 ACK 槽后调用非法
        bus = BitBus([Sender("A", Frame(0))], ack_pullers={"RX"})
        bus.run()
        with self.assertRaises(BusFinishedError):
            bus.add_ack_puller("LATE")

        bus2 = BitBus([Sender("A", Frame(0))], ack_pullers={"RX"})
        while bus2.phase == "transmit":
            bus2.step()
        with self.assertRaises(ValidationError):
            bus2.add_ack_puller("LATE")
        with self.assertRaises(ValidationError):
            BitBus([Sender("A", Frame(0))]).add_ack_puller("")

    def test_result_before_finish_rejected(self):
        bus = NodeBus([BusNode("TX", tx_frame=Frame(0))])
        with self.assertRaises(ValidationError):
            bus.result()
        with self.assertRaises(BusFinishedError):
            bus.run()
            bus.step()


if __name__ == "__main__":
    unittest.main()
