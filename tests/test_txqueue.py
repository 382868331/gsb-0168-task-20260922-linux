"""待发送队列测试：仲裁失败重试、错误分离、取消、最终记录分类。"""

import unittest

from canbit import (
    BitBus,
    ErrorFrameInfo,
    Frame,
    Sender,
    TxQueue,
    encode_frame,
)
from canbit.errors import ValidationError


class RetryTests(unittest.TestCase):
    def test_arbitration_loser_retries_and_succeeds(self):
        q = TxQueue(ack_pullers={"RX"})
        q.enqueue(Sender("A", Frame(0x321, b"hi")))
        q.enqueue(Sender("B", Frame(0x320, b"yo")))
        result = q.run()

        self.assertEqual([s.name for s in result.sent], ["B", "A"])
        self.assertEqual(result.sent[0].retries, 0)
        self.assertEqual(result.sent[1].retries, 1)  # A 实际重试了 1 次
        self.assertEqual(result.exhausted, ())
        self.assertEqual(result.pending, ())
        self.assertEqual(result.errors, ())
        # 两轮：第一轮 B 胜出 A 重新入队，第二轮 A 独占并胜出
        self.assertEqual(len(result.rounds), 2)
        self.assertEqual(result.rounds[0].sent, ("B",))
        self.assertEqual(result.rounds[0].requeued, ("A",))
        self.assertEqual(result.rounds[1].sent, ("A",))

    def test_retries_count_only_actual_occurrences(self):
        # max_retries=0：首轮仲裁失败即耗尽，没有发生过任何重试 -> retries=0
        q = TxQueue(ack_pullers={"RX"})
        q.enqueue(Sender("X", Frame(0x001, b"a")), max_retries=0)
        q.enqueue(Sender("Y", Frame(0x000, b"b")), max_retries=0)
        result = q.run()
        self.assertEqual([s.name for s in result.sent], ["Y"])
        self.assertEqual(len(result.exhausted), 1)
        self.assertEqual(result.exhausted[0].name, "X")
        self.assertEqual(result.exhausted[0].retries, 0)
        self.assertEqual(result.exhausted[0].max_retries, 0)

    def test_exhausted_after_last_budget_failure(self):
        # max_retries=1：允许 1 次真正的重试；第二次仲裁失败后不再重试
        # （该次失败不计入 retries，retries 仍为实际发生的 1 次）。
        q = TxQueue(ack_pullers={"RX"})
        q.enqueue(Sender("C", Frame(0x322, b"!!")), max_retries=1)
        q.enqueue(Sender("A", Frame(0x321, b"hi")), max_retries=5)
        q.enqueue(Sender("B", Frame(0x320, b"yo")), max_retries=5)
        result = q.run()

        self.assertEqual([s.name for s in result.sent], ["B", "A"])
        self.assertEqual(len(result.exhausted), 1)
        rec = result.exhausted[0]
        self.assertEqual(rec.name, "C")
        self.assertEqual(rec.retries, 1)
        self.assertEqual(rec.max_retries, 1)
        # C 在第 0、1 轮均仲裁失败；第 1 轮失败后才判定耗尽
        self.assertEqual(rec.retries, 1)

    def test_transmission_error_separate_from_arbitration(self):
        # ack_error：没有接收者拉低 ACK，活跃发送者报传输错误，
        # 不消耗重试次数、不再重试；与仲裁失败完全分开。
        q = TxQueue()  # 无 ack_pullers
        q.enqueue(Sender("A", Frame(0x100, b"x")), max_retries=5)
        result = q.run()
        self.assertEqual(result.sent, ())
        self.assertEqual(result.exhausted, ())
        self.assertEqual(len(result.errors), 1)
        err = result.errors[0]
        self.assertEqual(err.name, "A")
        self.assertEqual(err.error_frame.trigger, "ack_error")
        self.assertEqual(err.round_index, 0)

    def test_bit_error_after_arbitration_losers_keep_losers_separate(self):
        # 同一轮：仲裁失败者在错误帧之前已退出，应按仲裁失败重试；
        # 活跃发送者被错误帧中止记 errors。
        winner_frame = Frame(0x320, b"yo")
        enc = encode_frame(winner_frame)
        first_stuff = next(i + 1 for i, m in enumerate(enc.stuffing_map) if m == 1)
        # 选一个会被活跃胜者检测到失配的注入位（填充位本身）
        forced = {first_stuff: enc.stream[first_stuff - 1]}

        q = TxQueue(ack_pullers={"RX"}, forced_bits=forced)
        q.enqueue(Sender("LOSER", Frame(0x321, b"hi")), max_retries=3)
        q.enqueue(Sender("WIN", winner_frame), max_retries=3)
        r0 = q.step_round()
        self.assertEqual(r0.requeued, ("LOSER",))
        self.assertEqual(r0.errored, ("WIN",))
        # 败者不消耗额外预算：retries 恰为实际的 1 次
        self.assertEqual(q.pending_names, ("LOSER",))
        snap = q.result()
        self.assertEqual(snap.pending[0].retries, 1)

    def test_cancel_queued_does_not_affect_others(self):
        q = TxQueue(ack_pullers={"RX"})
        q.enqueue(Sender("A", Frame(0x321, b"hi")))
        q.enqueue(Sender("B", Frame(0x320, b"yo")))
        self.assertTrue(q.cancel("A"))
        self.assertFalse(q.cancel("A"))  # 已不在队列
        result = q.run()
        self.assertEqual([s.name for s in result.sent], ["B"])
        self.assertEqual(len(result.rounds), 1)

    def test_cancel_between_rounds_removes_only_waiting_frame(self):
        q = TxQueue(ack_pullers={"RX"})
        q.enqueue(Sender("A", Frame(0x321, b"hi")))
        q.enqueue(Sender("B", Frame(0x320, b"yo")))
        r0 = q.step_round()
        self.assertEqual(r0.sent, ("B",))
        # A 正在排队等待第二轮，取消它不影响已成功的 B
        self.assertTrue(q.cancel("A"))
        result = q.run()
        self.assertEqual([s.name for s in result.sent], ["B"])
        self.assertEqual(result.pending, ())
        self.assertEqual(result.exhausted, ())

    def test_pending_recorded_when_max_rounds_reached(self):
        q = TxQueue(ack_pullers={"RX"}, max_rounds=1)
        q.enqueue(Sender("A", Frame(0x321, b"hi")))
        q.enqueue(Sender("B", Frame(0x320, b"yo")))
        result = q.run()
        self.assertEqual([s.name for s in result.sent], ["B"])
        self.assertEqual([p.name for p in result.pending], ["A"])
        self.assertEqual(result.pending[0].retries, 1)
        self.assertEqual(result.exhausted, ())

    def test_final_record_categories_disjoint(self):
        q = TxQueue(ack_pullers={"RX"})
        q.enqueue(Sender("OK", Frame(0x100, b"ok")), max_retries=2)
        q.enqueue(Sender("LATE", Frame(0x200, b"z")), max_retries=0)
        q.enqueue(Sender("TOP", Frame(0x050, b"top")), max_retries=0)
        result = q.run()
        all_names = (
            {s.name for s in result.sent}
            | {p.name for p in result.pending}
            | {e.name for e in result.exhausted}
            | {e.name for e in result.errors}
        )
        self.assertEqual(len(all_names), 3)  # 三类记录互不重叠
        self.assertEqual({s.name for s in result.sent}, {"TOP", "OK"})
        self.assertEqual({e.name for e in result.exhausted}, {"LATE"})

    def test_contract_validation(self):
        q = TxQueue()
        with self.assertRaises(ValidationError):
            q.enqueue(Frame(0, b""))  # 不是 Sender
        q.enqueue(Sender("A", Frame(0, b"")))
        with self.assertRaises(ValidationError):
            q.enqueue(Sender("A", Frame(1, b"")))  # 同名重复排队
        with self.assertRaises(ValidationError):
            q.enqueue(Sender("B", Frame(0, b"")), max_retries=True)  # bool
        with self.assertRaises(ValidationError):
            q.enqueue(Sender("C", Frame(0, b"")), max_retries=-1)
        with self.assertRaises(ValidationError):
            TxQueue(max_rounds=0)
        with self.assertRaises(ValidationError):
            q.cancel("")

    def test_each_round_uses_original_arbitration_rules(self):
        # 三轮中败者重新参与时仍遵循 wired-AND / ID 域原规则：
        # 最低 ID 始终先赢；结果顺序按 ID 升序。
        q = TxQueue(ack_pullers={"RX"})
        for ident in (0x321, 0x320, 0x100, 0x200, 0x0FF):
            q.enqueue(Sender(f"N{ident:03X}", Frame(ident, b"d")),
                      max_retries=5)
        result = q.run()
        winners = [s.name for s in result.sent]
        self.assertEqual(winners, ["N0FF", "N100", "N200", "N320", "N321"])
        self.assertEqual(result.errors, ())
        self.assertEqual(result.exhausted, ())


if __name__ == "__main__":
    unittest.main()
