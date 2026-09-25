"""接收节点 / 固定过滤表测试。

覆盖题面要求：
  * 全过滤（无命中）的正确帧仍由正常接收节点贡献 ACK
  * 总线上只有监听模式节点时 -> ack_error；监听者不 ACK、不发错误，
    但记录自己看到的校验结果
  * 匹配帧在接收视角 CRC 损坏 -> 不上交、不 ACK（crc_ok=False）
  * 两个重叠规则只交付一次，命中序号取最早规则
  * 仲裁失败节点以同名 Node 身份转接收，收到胜者帧
所有总线用例一律**逐 bit 步进**驱动，并逐节点对照 tests/reference.py
的独立全量仿真（参考不 import canbit）。
"""

import unittest

import reference as ref

from canbit import BitBus, Filter, Frame, Node, Sender, encode_frame
from canbit.errors import ValidationError


def rules_to_ref(node):
    return [(f.id, f.mask) for f in node.filters]


def run_step_by_step(bus):
    """逐 bit 驱动原总线，返回 (finished 结果, 每步 phase 序列)。"""
    phases = []
    while not bus.finished:
        sr = bus.step()
        phases.append(sr.phase)
    return bus.result(), phases


def records_to_ref(result):
    """BusResult 接收记录 -> 与参考对齐的普通 dict。"""
    out = {}
    for name, rec in result.receptions.items():
        frame = None
        if rec.frame is not None:
            frame = (rec.frame.identifier, rec.frame.dlc, rec.frame.data)
        out[name] = {
            "frame": frame,
            "delivered": rec.delivered,
            "rule_index": rec.rule_index,
            "crc_ok": rec.crc_ok,
            "stuff_ok": rec.stuff_ok,
            "acked": rec.acked,
            "listen_only": rec.listen_only,
            "error": rec.error,
        }
    return out


def assert_records_match(testcase, result, ref_res):
    got = records_to_ref(result)
    want = ref_res["records"]
    testcase.assertEqual(set(got), set(want))
    for name in want:
        testcase.assertEqual(
            got[name], want[name], msg=f"node {name!r} record mismatch"
        )


class FilterTableTests(unittest.TestCase):
    def test_hit_condition(self):
        rule = Filter(0x123, 0x7FF)  # 全 11 位掩码 -> 精确
        self.assertTrue(rule.matches(0x123))
        self.assertFalse(rule.matches(0x122))
        broad = Filter(0x100, 0x700)  # 只看高 3 位
        self.assertTrue(broad.matches(0x123))
        self.assertTrue(broad.matches(0x1FF))
        self.assertFalse(broad.matches(0x200))
        any_rule = Filter(0x000, 0x000)  # 掩码 0 -> 命中所有 ID
        for fid in (0, 1, 0x7FF, 0x400):
            self.assertTrue(any_rule.matches(fid))

    def test_id_and_mask_limited_to_11_bits(self):
        Filter(0x7FF, 0x7FF)  # 边界合法
        Filter(0, 0)
        for bad_id, bad_mask in [(0x800, 0x7FF), (0x7FF, 0x800), (-1, 0), (0, -1)]:
            with self.assertRaises(ValidationError):
                Filter(bad_id, bad_mask)
        with self.assertRaises(ValidationError):
            Filter(True, 0x7FF)
        with self.assertRaises(ValidationError):
            Filter(0, True)

    def test_table_fixed_and_immutable_during_transfer(self):
        external = [Filter(0x100, 0x7FF)]
        node = Node("R", external)
        external.append(Filter(0x200, 0x7FF))  # 改外部列表不影响已建节点
        self.assertEqual(len(node.filters), 1)
        with self.assertRaises(Exception):
            node.filters = (Filter(0, 0),)  # frozen
        bus = BitBus([Sender("A", Frame(0x100))], nodes=[node])
        bus.run()
        # 传输期间/之后拿到的是同一张固定表
        self.assertEqual(bus.node_filters("R"), (Filter(0x100, 0x7FF),))
        with self.assertRaises(ValidationError):
            Node("R", [Filter(0, 0)] * 33)  # 超条目上限
        with self.assertRaises(ValidationError):
            Node("R", ["not-a-filter"])  # type: ignore[list-item]


class ReceptionBehaviorTests(unittest.TestCase):
    def test_all_filtered_out_still_acks(self):
        # 两个正常节点都不会收到该帧（一个只收 0x7FF，一个空表），
        # 但帧 CRC 正确 -> 它们仍须在 ACK 槽拉低，传输成功。
        frame = Frame(0x100, b"hi")
        nodes = [Node("R1", [Filter(0x7FF, 0x7FF)]), Node("R2", [])]
        result, phases = run_step_by_step(
            BitBus([Sender("A", frame)], nodes=nodes)
        )
        self.assertTrue(result.completed)
        self.assertTrue(result.acked)
        self.assertIn("ack", phases)
        self.assertEqual(phases[-1], "done")
        for name, rec in result.receptions.items():
            self.assertTrue(rec.crc_ok)          # CRC 照常校验
            self.assertFalse(rec.delivered)      # 被过滤，不上交
            self.assertIsNone(rec.rule_index)
            self.assertTrue(rec.acked)           # 但仍贡献 ACK
        self.assertEqual(result.delivered(), ())

        ref_res = ref.ref_simulate_bus_nodes(
            [("A", frame.identifier, frame.data)],
            [("R1", [(0x7FF, 0x7FF)], False), ("R2", [], False)],
        )
        assert_records_match(self, result, ref_res)

    def test_delivered_frame_carries_earliest_rule_index(self):
        frame = Frame(0x123, b"x")
        # 两条规则都命中 0x123：宽掩码(序号0) + 精确(序号1)
        nodes = [
            Node("broad_first", [Filter(0x100, 0x700), Filter(0x123, 0x7FF)]),
            Node("exact_first", [Filter(0x123, 0x7FF), Filter(0x100, 0x700)]),
        ]
        result, _ = run_step_by_step(BitBus([Sender("A", frame)], nodes=nodes))
        self.assertTrue(result.completed)
        # 每节点只交付一次，序号以本表最早规则为准
        self.assertEqual(len(result.delivered()), 2)
        self.assertEqual(result.receptions["broad_first"].rule_index, 0)
        self.assertEqual(result.receptions["exact_first"].rule_index, 0)
        rec = result.receptions["broad_first"]
        self.assertTrue(rec.delivered)
        self.assertEqual(rec.frame.identifier, 0x123)
        self.assertEqual(rec.frame.data, b"x")

        ref_res = ref.ref_simulate_bus_nodes(
            [("A", frame.identifier, frame.data)],
            [
                ("broad_first", [(0x100, 0x700), (0x123, 0x7FF)], False),
                ("exact_first", [(0x123, 0x7FF), (0x100, 0x700)], False),
            ],
        )
        assert_records_match(self, result, ref_res)

    def test_only_listener_present_raises_ack_error(self):
        # 监听节点能匹配帧；但监听者永不 ACK -> 发送者报 ack_error
        frame = Frame(0x100, b"hi")
        nodes = [Node("L", [Filter(0x100, 0x7FF)], listen_only=True)]
        bus = BitBus([Sender("A", frame)], nodes=nodes)
        result, phases = run_step_by_step(bus)
        self.assertFalse(result.completed)
        self.assertEqual(result.error_frame.trigger, "ack_error")
        self.assertEqual(result.error_frame.bit_index,
                         encode_frame(frame).ack_index)
        rec = result.receptions["L"]
        self.assertTrue(rec.crc_ok)       # 看到的帧是好的
        self.assertTrue(rec.delivered)    # 监听者照样按过滤表上交记录
        self.assertFalse(rec.acked)       # 但永不 ACK
        self.assertTrue(rec.listen_only)
        # 监听者不出现在错误帧参与者里（它不发错误标志）
        self.assertNotIn("L", result.error_frame.participants)
        self.assertNotIn("ack", phases)   # 未进入成功 ACK 相位

        ref_res = ref.ref_simulate_bus_nodes(
            [("A", frame.identifier, frame.data)],
            [("L", [(0x100, 0x7FF)], True)],
        )
        self.assertEqual(ref_res["outcome"], "ack_error")
        assert_records_match(self, result, ref_res)

    def test_listener_never_acks_or_errors_but_records_bad_crc(self):
        # 监听者 + 一个显式裸 ACK 者：传输成功；接收视角损坏一位，
        # 监听者记录 CRC 失败，且自己始终不 ACK、不触发任何错误帧。
        frame = Frame(0x201, b"payload!")
        enc = encode_frame(frame)
        target = next(
            off for off, bit in enumerate(enc.stream)
            if 15 < off < enc.ack_index and bit == 1
        )
        listener = Node("L", [Filter(0x201, 0x7FF)], listen_only=True)
        bus = BitBus(
            [Sender("A", frame)],
            nodes=[listener],
            ack_pullers={"X"},
            receive_glitches={target: 0},
        )
        result, _ = run_step_by_step(bus)
        self.assertTrue(result.completed)          # 总线层面完好
        self.assertIsNone(result.error_frame)
        rec = result.receptions["L"]
        self.assertFalse(rec.crc_ok)               # 记录看到的校验结果
        self.assertFalse(rec.delivered)            # CRC 坏 -> 不上交
        self.assertFalse(rec.acked)                # 监听者永不 ACK
        self.assertTrue(rec.listen_only)

        ref_res = ref.ref_simulate_bus_nodes(
            [("A", frame.identifier, frame.data)],
            [("L", [(0x201, 0x7FF)], True)],
            ack_pullers={"X"},
            receive_glitches={target: 0},
        )
        assert_records_match(self, result, ref_res)

    def test_matched_frame_crc_corrupt_not_delivered_or_acked(self):
        # 没有裸 ACK 者：唯一正常节点的匹配帧在接收视角 CRC 损坏，
        # 它不上交、也不 ACK -> 发送者走到 ack_error。
        frame = Frame(0x201, b"payload!")
        enc = encode_frame(frame)
        target = next(
            off for off, bit in enumerate(enc.stream)
            if 15 < off < enc.ack_index and bit == 1
        )
        node = Node("R", [Filter(0x201, 0x7FF)])
        result, _ = run_step_by_step(
            BitBus(
                [Sender("A", frame)],
                nodes=[node],
                receive_glitches={target: 0},
            )
        )
        self.assertFalse(result.completed)
        self.assertEqual(result.error_frame.trigger, "ack_error")
        rec = result.receptions["R"]
        self.assertFalse(rec.crc_ok)
        self.assertFalse(rec.delivered)
        self.assertIsNone(rec.rule_index)
        self.assertFalse(rec.acked)
        # 总线位本身未被破坏（发送者监控无失配，无 bit_error）
        self.assertEqual(
            result.bus_bits[target], enc.stream[target]
        )

    def test_arbitration_loser_converts_to_receiver(self):
        # A=0x321 vs B=0x320：B 胜；同名节点 A 仲裁失败后转接收，
        # 必须收到胜者帧并（CRC 正确时）以接收者身份贡献 ACK。
        frame_a, frame_b = Frame(0x321, b"hi"), Frame(0x320, b"yo")
        node_a = Node("A", [Filter(0x300, 0x700)])
        pure = Node("R", [Filter(0x320, 0x7FF)])
        result, _ = run_step_by_step(
            BitBus(
                [Sender("A", frame_a), Sender("B", frame_b)],
                nodes=[node_a, pure],
            )
        )
        self.assertEqual(result.winners, ("B",))
        self.assertEqual(result.arbitration_losers, (("A", 11),))
        rec_a = result.receptions["A"]
        rec_r = result.receptions["R"]
        self.assertEqual((rec_a.frame.identifier, rec_a.frame.data), (0x320, b"yo"))
        self.assertTrue(rec_a.crc_ok and rec_a.delivered and rec_a.acked)
        # 转接收者与纯接收者看到完全相同的帧与校验结果
        self.assertEqual(
            (rec_a.frame.identifier, rec_a.frame.data, rec_a.crc_ok,
             rec_a.rule_index),
            (rec_r.frame.identifier, rec_r.frame.data, rec_r.crc_ok,
             rec_r.rule_index),
        )

        ref_res = ref.ref_simulate_bus_nodes(
            [("A", frame_a.identifier, frame_a.data),
             ("B", frame_b.identifier, frame_b.data)],
            [("A", [(0x300, 0x700)], False),
             ("R", [(0x320, 0x7FF)], False)],
        )
        self.assertEqual(ref_res["winner"], ("B",))
        self.assertEqual(ref_res["losers"], (("A", 11),))
        assert_records_match(self, result, ref_res)

    def test_step_by_step_rx_buffer_aligns_with_wire(self):
        # 显式逐 bit：接收节点在 ACK 槽前的缓冲长度恰为胜者填充流长度，
        # 且与总线上的 ID..CRC 位逐位相同（无注入时）。
        frame = Frame(0x0AB, b"CAN")
        node = Node("R", [Filter(0x0AB, 0x7FF)])
        bus = BitBus([Sender("A", frame)], nodes=[node])
        saw_ack_at = None
        while not bus.finished:
            sr = bus.step()
            if sr.phase == "ack":
                saw_ack_at = sr.bit_index
        result = bus.result()
        enc = encode_frame(frame)
        self.assertEqual(saw_ack_at, enc.ack_index)
        wire_stuffed = result.bus_bits[1 : 1 + len(enc.stuffed)]
        self.assertEqual(tuple(wire_stuffed), tuple(enc.stuffed))
        self.assertTrue(result.receptions["R"].delivered)


class DifferentialNodeTests(unittest.TestCase):
    def test_differential_against_reference_mixed_cases(self):
        # 确定性 LCG 生成：双发送者（含仲裁失败转接收）、多节点多规则、
        # 监听者、接收视角注入；逐节点记录必须与独立参考一致。
        seed = 0xBEEF12
        node_specs = [
            ("N0", [(0x000, 0x000)], False),                 # 全收
            ("N1", [(0x200, 0x700), (0x201, 0x7FF)], False),  # 重叠规则
            ("N2", [(0x7FF, 0x7FF)], False),                 # 只收 0x7FF
            ("N3", [(0x000, 0x000)], True),                  # 监听者
            ("N4", [], False),                               # 空表
        ]
        nodes = [
            Node(name, [Filter(i, m) for i, m in rules], listen_only=listen)
            for name, rules, listen in node_specs
        ]
        for case in range(20):
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            id1 = seed & 0x7FF
            d1 = seed % 3
            data1 = bytes((seed >> k) & 0xFF for k in range(0, 3 * d1, 3))[:d1]
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            id2 = seed & 0x7FF
            d2 = (seed >> 4) % 3
            data2 = bytes((seed >> k) & 0xFF for k in range(0, 3 * d2, 3))[:d2]

            # 让某个发送者与节点同名以覆盖“仲裁失败转接收”
            name1 = "N1" if case % 2 == 0 else "S1"
            frames = [
                (name1, id1, data1),
                ("S2", id2, data2),
            ]
            senders = [
                Sender(name1, Frame(id1, data1)),
                Sender("S2", Frame(id2, data2)),
            ]
            active_nodes = list(nodes)
            if name1 == "S1":
                active_nodes.append(Node("S1", [Filter(id1, 0x7FF)]))
                specs = node_specs + [("S1", [(id1, 0x7FF)], False)]
            else:
                specs = node_specs

            glitch = {}
            if case % 4 == 1:
                enc = encode_frame(Frame(id1, data1))
                cand = next(
                    (off for off, b in enumerate(enc.stream)
                     if 15 < off < enc.ack_index and b == 1),
                    None,
                )
                if cand is not None:
                    glitch = {cand: 0}

            result, _ = run_step_by_step(
                BitBus(senders, nodes=active_nodes, receive_glitches=glitch)
            )
            ref_res = ref.ref_simulate_bus_nodes(
                frames, specs, receive_glitches=glitch
            )
            self.assertEqual(
                result.completed,
                ref_res["outcome"] == "complete",
                msg=f"case {case} outcome",
            )
            self.assertEqual(
                result.winners, ref_res["winner"], msg=f"case {case} winner"
            )
            self.assertEqual(
                result.arbitration_losers,
                ref_res["losers"],
                msg=f"case {case} losers",
            )
            assert_records_match(self, result, ref_res)


class NodeContractTests(unittest.TestCase):
    def test_node_validation(self):
        with self.assertRaises(ValidationError):
            Node("", [Filter(0, 0)])
        with self.assertRaises(ValidationError):
            Node("R", listen_only="yes")  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            Node("R", 123)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            Node("R", [(0x100, 0x7FF)])  # tuple, not Filter
        with self.assertRaises(ValidationError):
            Node("R", [Filter(0, 0)] * 33)

    def test_bus_node_validation(self):
        sender = [Sender("A", Frame(0x100))]
        with self.assertRaises(ValidationError):
            BitBus(sender, nodes=[Node("R"), Node("R")])  # 重名
        with self.assertRaises(ValidationError):
            BitBus(sender, nodes=["not-a-node"])  # type: ignore[list-item]
        # 监听节点不允许出现在 ack_pullers
        with self.assertRaises(ValidationError):
            BitBus(
                sender,
                nodes=[Node("L", [Filter(0x100, 0x7FF)], listen_only=True)],
                ack_pullers={"L"},
            )
        # 普通节点名也不允许重复出现在 ack_pullers（ACK 由其自身 CRC 决定）
        with self.assertRaises(ValidationError):
            BitBus(sender, nodes=[Node("R")], ack_pullers={"R"})
        with self.assertRaises(ValidationError):
            BitBus(sender, receive_glitches={-1: 0})
        with self.assertRaises(ValidationError):
            BitBus(sender, receive_glitches={1: 2})
        with self.assertRaises(ValidationError):
            bus = BitBus(sender)
            bus.node_filters("nobody")


if __name__ == "__main__":
    unittest.main()
