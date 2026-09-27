"""重试调度器测试：待发送队列、仲裁/错误分类计数、取消与终态记录。

期望值不来自被测核心之外的黑盒假设：仲裁规则本身由 tests/reference.py
与 test_bus.py 锁定；本文件断言调度层的队列/计数/状态语义。
"""

import unittest

from canbit import Frame, RetryScheduler, Sender
from canbit.errors import ValidationError


def sender(name, ident, data=b"x"):
    return Sender(name, Frame(ident, data))


class QueueRetryTests(unittest.TestCase):
    def test_loser_requeues_and_wins_next_round(self):
        # A(0x321) 输给 B(0x320) 后重新排队，第二轮独自发送成功
        sched = RetryScheduler(
            [sender("A", 0x321, b"hi"), sender("B", 0x320, b"yo")],
            max_retries=3,
            ack_pullers={"RX"},
        )
        report = sched.run()

        self.assertEqual(len(report.rounds), 2)
        self.assertEqual(report.rounds[0].result.winners, ("B",))
        self.assertEqual(report.rounds[0].requeued, ("A",))
        self.assertEqual(report.rounds[1].result.winners, ("A",))

        recs = {r.name: r for r in report.records}
        self.assertEqual(recs["B"].status, "sent")
        self.assertEqual(recs["B"].attempts, 1)
        self.assertEqual(recs["B"].arbitration_retries, 0)
        self.assertEqual(recs["A"].status, "sent")
        self.assertEqual(recs["A"].attempts, 2)
        # 仲裁失败只按实际发生计数：恰好 1 次，且无传输错误
        self.assertEqual(recs["A"].arbitration_retries, 1)
        self.assertEqual(recs["A"].error_retries, 0)
        self.assertEqual(set(report.sent), {"A", "B"})
        self.assertEqual(report.pending, ())
        self.assertEqual(report.exhausted, ())

    def test_retry_exhaustion_marks_exhausted(self):
        # 三个不同 ID，max_retries=1：C 两次仲裁失败后耗尽退出
        sched = RetryScheduler(
            [sender("A", 0x100), sender("B", 0x101), sender("C", 0x102)],
            max_retries=1,
            ack_pullers={"RX"},
        )
        report = sched.run()
        recs = {r.name: r for r in report.records}
        self.assertEqual(recs["A"].status, "sent")
        self.assertEqual(recs["B"].status, "sent")
        self.assertEqual(recs["C"].status, "exhausted")
        self.assertEqual(recs["C"].arbitration_retries, 2)
        self.assertEqual(recs["C"].attempts, 2)
        self.assertEqual(report.exhausted, ("C",))
        # 耗尽后不再参与：A、B 各胜一轮，C 在第二轮耗尽
        self.assertEqual(len(report.rounds), 2)

    def test_zero_retries_means_first_failure_exhausts(self):
        sched = RetryScheduler(
            [sender("A", 0x100), sender("B", 0x101)],
            max_retries=0,
            ack_pullers={"RX"},
        )
        report = sched.run()
        recs = {r.name: r for r in report.records}
        self.assertEqual(recs["A"].status, "sent")
        self.assertEqual(recs["B"].status, "exhausted")
        self.assertEqual(len(report.rounds), 1)

    def test_transmission_error_counted_separately(self):
        # 每轮在 DLC 末位（线上索引 17）强制显性 0：发送者发 1 读 0
        # 属 bit_error（非 ID 域），错误帧中止，计入 error_retries
        sched = RetryScheduler(
            [sender("A", 0x100, b"\x01")],
            max_retries=1,
            ack_pullers={"RX"},
            forced_bits={17: 0},
        )
        report = sched.run()
        (rec,) = report.records
        self.assertEqual(rec.status, "exhausted")
        self.assertEqual(rec.error_retries, 2)
        self.assertEqual(rec.arbitration_retries, 0)
        self.assertEqual(rec.attempts, 2)
        for rnd in report.rounds:
            self.assertIsNotNone(rnd.result.error_frame)
            self.assertEqual(rnd.result.error_frame.trigger, "bit_error")

    def test_ack_error_is_transmission_error_not_arbitration(self):
        # 无 ACK 接收者：每轮 ack_error，计入 error_retries
        sched = RetryScheduler([sender("A", 0x100)], max_retries=2)
        report = sched.run()
        (rec,) = report.records
        self.assertEqual(rec.status, "exhausted")
        self.assertEqual(rec.error_retries, 3)
        self.assertEqual(rec.arbitration_retries, 0)
        for rnd in report.rounds:
            self.assertEqual(rnd.result.error_frame.trigger, "ack_error")

    def test_error_round_loser_counts_arbitration_not_error(self):
        # 同轮中：B 先仲裁失败，随后 A 被 bit_error 中止——分类不得混淆
        sched = RetryScheduler(
            [sender("A", 0x100, b"\x01"), sender("B", 0x101, b"\x01")],
            max_retries=0,
            ack_pullers={"RX"},
            forced_bits={17: 0},
        )
        report = sched.run()
        recs = {r.name: r for r in report.records}
        self.assertEqual(recs["B"].arbitration_retries, 1)
        self.assertEqual(recs["B"].error_retries, 0)
        self.assertEqual(recs["A"].error_retries, 1)
        self.assertEqual(recs["A"].arbitration_retries, 0)


class CancelTests(unittest.TestCase):
    def test_cancel_queued_frame(self):
        sched = RetryScheduler(
            [sender("A", 0x100), sender("B", 0x101), sender("C", 0x102)],
            max_retries=5,
            ack_pullers={"RX"},
        )
        self.assertTrue(sched.cancel("C"))
        self.assertFalse(sched.cancel("C"))  # 已不在队列
        self.assertFalse(sched.cancel("NOPE"))  # 未知名字
        report = sched.run()
        recs = {r.name: r for r in report.records}
        self.assertEqual(recs["C"].status, "cancelled")
        self.assertEqual(recs["C"].attempts, 0)
        self.assertEqual(recs["A"].status, "sent")
        self.assertEqual(recs["B"].status, "sent")

    def test_cancel_does_not_touch_in_flight_round(self):
        sched = RetryScheduler(
            [sender("A", 0x100), sender("B", 0x101)],
            max_retries=5,
            ack_pullers={"RX"},
        )
        sched.step()  # 第一轮开始，A/B 都在发送中
        self.assertEqual(set(sched.in_flight), {"A", "B"})
        # 发送中的帧不可取消
        self.assertFalse(sched.cancel("A"))
        # 轮次进行中入队新帧再取消：只影响排队者
        sched.enqueue(sender("C", 0x102))
        self.assertEqual(sched.waiting, ("C",))
        self.assertTrue(sched.cancel("C"))
        report = sched.run()
        recs = {r.name: r for r in report.records}
        # 进行中的 A/B 均未受取消影响，正常完成
        self.assertEqual(recs["A"].status, "sent")
        self.assertEqual(recs["B"].status, "sent")
        self.assertEqual(recs["C"].status, "cancelled")
        self.assertEqual(recs["C"].attempts, 0)

    def test_pending_visible_mid_run(self):
        sched = RetryScheduler(
            [sender("A", 0x100), sender("B", 0x101)],
            max_retries=5,
            ack_pullers={"RX"},
        )
        sched.step()
        recs = {r.name: r for r in sched.records}
        self.assertEqual(recs["A"].status, "pending")
        self.assertEqual(recs["B"].status, "pending")
        sched.run()


class ContractTests(unittest.TestCase):
    def test_max_retries_validation(self):
        for bad in (True, 1.5, float("nan"), -1, "3"):
            with self.assertRaises(ValidationError):
                RetryScheduler([sender("A", 1)], max_retries=bad)

    def test_duplicate_enqueue_rejected(self):
        sched = RetryScheduler([sender("A", 1)], max_retries=1)
        with self.assertRaises(ValidationError):
            sched.enqueue(sender("A", 2))
        with self.assertRaises(ValidationError):
            sched.enqueue("not-a-sender")

    def test_step_after_finished_rejected(self):
        sched = RetryScheduler([sender("A", 1)], max_retries=1, ack_pullers={"RX"})
        sched.run()
        self.assertTrue(sched.finished)
        with self.assertRaises(ValidationError):
            sched.step()

    def test_empty_scheduler_finished_immediately(self):
        sched = RetryScheduler(max_retries=1)
        self.assertTrue(sched.finished)
        report = sched.run()
        self.assertEqual(report.records, ())
        self.assertEqual(report.rounds, ())

    def test_cancel_name_validation(self):
        sched = RetryScheduler(max_retries=1)
        for bad in ("", 1, None):
            with self.assertRaises(ValidationError):
                sched.cancel(bad)


if __name__ == "__main__":
    unittest.main()
