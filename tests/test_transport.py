"""传输层测试：本地事件序列驱动两个端点，独立按原字节串核对交付。

不经过真实总线；最终交付与测试内固定生成的原始字节串逐字节对照，
失败用例必须整次失败且不留部分消息。
"""

import unittest

from canbit import (
    CF_DATA_BYTES,
    FF_DATA_BYTES,
    FS_CTS,
    FS_WAIT,
    MAX_MESSAGE_BYTES,
    PCI_CF,
    PCI_FF,
    Receiver,
    TransportError,
    TransportSender,
    make_consecutive_frame,
    make_first_frame,
    make_flow_control_frame,
)
from canbit.errors import ValidationError

ID_TX = 0x700
ID_RX = 0x708


def make_payload(length, seed=0xBEEF01):
    """确定性 LCG 生成固定字节串（不依赖 random 模块）。"""
    out = bytearray()
    state = seed
    while len(out) < length:
        state = (state * 1103515245 + 12345) & 0x7FFFFFFF
        out.append((state >> 11) & 0xFF)
    return bytes(out)


def run_transfer(payload, *, block_size=0, st_min=0, drop_cf=(),
                 max_waits=3, timeout=100, max_message=MAX_MESSAGE_BYTES):
    """本地事件序列驱动发送/接收两端，返回 (delivered, log)。

    log 记录 (tick, 方向, frame)，方向为 "TX"（发到接收端）或 "FC"。
    drop_cf 给出要丢弃的连续帧序号（0 基发送次序），模拟丢帧。
    """
    sender = TransportSender(ID_TX, payload, max_waits=max_waits)
    receiver = Receiver(ID_RX, block_size=block_size, st_min=st_min,
                        timeout=timeout, max_message=max_message)
    tick = 0
    delivered = None
    log = []
    ff = sender.start(tick)
    log.append((tick, "TX", ff))
    res = receiver.on_frame(ff, tick)
    delivered = res.delivered if res.delivered is not None else None
    pending_fc = list(res.flow_frames)
    cf_no = 0
    while not sender.done:
        if pending_fc:
            fc = pending_fc.pop(0)
            log.append((tick, "FC", fc))
            sender.on_frame(fc, tick)
            continue
        frame = sender.next_frame(tick)
        if frame is None:
            tick += 1
            receiver.on_tick(tick)
            continue
        log.append((tick, "TX", frame))
        drop = cf_no in drop_cf
        cf_no += 1
        if drop:
            continue
        res = receiver.on_frame(frame, tick)
        if res.delivered is not None:
            delivered = res.delivered
        pending_fc.extend(res.flow_frames)
    return delivered, log


def independent_reassembly(log):
    """测试侧独立解析：不调用 canbit.transport，只按 README 帧格式手工拼。

    用纯字节操作从事件日志重建消息，与被测端点的交付对照。
    """
    total = None
    buf = bytearray()
    for _tick, direction, frame in log:
        if direction != "TX":
            continue
        pci = frame.data[0] & 0xF0
        if pci == 0x10:
            total = ((frame.data[0] & 0x0F) << 8) | frame.data[1]
            buf = bytearray(frame.data[2:8])
        elif pci == 0x20:
            buf += frame.data[1:8]
    assert total is not None
    return bytes(buf[:total])


class NormalTransferTests(unittest.TestCase):
    def test_delivery_matches_original_byte_string(self):
        payload = make_payload(100)
        delivered, log = run_transfer(payload)
        # 独立按原字节串核对最终交付
        self.assertEqual(delivered, payload)
        self.assertEqual(len(delivered), 100)
        # 测试侧独立重组同一事件序列，结论必须一致
        self.assertEqual(independent_reassembly(log), payload)

    def test_short_message_fits_first_frame(self):
        payload = make_payload(5)
        delivered, log = run_transfer(payload)
        self.assertEqual(delivered, payload)
        tx_frames = [f for _, d, f in log if d == "TX"]
        self.assertEqual(len(tx_frames), 1)
        self.assertEqual(tx_frames[0].data[0] & 0xF0, PCI_FF)
        # 短消息不需要流控帧
        self.assertFalse([e for e in log if e[1] == "FC"])

    def test_sequence_wraps_mod_16(self):
        # 200 字节 -> 首帧 6 + 28 个连续帧，序号必然跨 15->0 回绕
        payload = make_payload(200)
        delivered, log = run_transfer(payload)
        self.assertEqual(delivered, payload)
        seqs = [f.data[0] & 0x0F for _, d, f in log
                if d == "TX" and f.data[0] & 0xF0 == PCI_CF]
        self.assertEqual(len(seqs), 28)
        self.assertEqual(seqs[:16], list(range(1, 16)) + [0])
        self.assertEqual(seqs[16:], list(range(1, 13)))
        # 独立重组同样核对
        self.assertEqual(independent_reassembly(log), payload)

    def test_block_size_pauses_sender(self):
        payload = make_payload(120)
        delivered, log = run_transfer(payload, block_size=3)
        self.assertEqual(delivered, payload)
        # 两张连续 FC 之间发出的 CF 不得超过块大小 3
        run = 0
        runs = []
        for _, direction, frame in log:
            if direction == "TX" and frame.data[0] & 0xF0 == PCI_CF:
                run += 1
            elif direction == "FC":
                runs.append(run)
                run = 0
        runs.append(run)
        self.assertTrue(any(r == 3 for r in runs))
        self.assertTrue(all(r <= 3 for r in runs))

    def test_st_min_spacing_respected(self):
        payload = make_payload(60)
        delivered, log = run_transfer(payload, st_min=3)
        self.assertEqual(delivered, payload)
        ticks = [t for t, d, f in log
                 if d == "TX" and f.data[0] & 0xF0 == PCI_CF]
        for prev, cur in zip(ticks, ticks[1:]):
            self.assertGreaterEqual(cur - prev, 3)


class FailureTests(unittest.TestCase):
    def test_lost_frame_fails_whole_session(self):
        payload = make_payload(80)
        with self.assertRaises(TransportError) as cm:
            run_transfer(payload, drop_cf={2})
        self.assertEqual(cm.exception.kind, "sequence")

    def test_lost_frame_leaves_no_partial_message(self):
        payload = make_payload(80)
        receiver = Receiver(ID_RX)
        sender = TransportSender(ID_TX, payload)
        res = receiver.on_frame(sender.start(0), 0)
        sender.on_frame(res.flow_frames[0], 0)
        # 丢掉 seq=1，直接发 seq=2
        sender.next_frame(0)  # seq 1 被丢弃
        cf2 = sender.next_frame(0)
        with self.assertRaises(TransportError):
            receiver.on_frame(cf2, 0)
        self.assertEqual(receiver.buffered_bytes, 0)
        self.assertFalse(receiver.receiving)

    def test_oversize_first_frame_rejected(self):
        receiver = Receiver(ID_RX, max_message=32)
        ff = make_first_frame(ID_TX, 100, b"abcdef")
        with self.assertRaises(TransportError) as cm:
            receiver.on_frame(ff, 0)
        self.assertEqual(cm.exception.kind, "oversize")
        self.assertFalse(receiver.receiving)

    def test_new_first_frame_does_not_overwrite_in_progress(self):
        payload = make_payload(50)
        other = make_payload(60, seed=7)
        receiver = Receiver(ID_RX)
        sender = TransportSender(ID_TX, payload)
        res = receiver.on_frame(sender.start(0), 0)
        sender.on_frame(res.flow_frames[0], 0)
        receiver.on_frame(sender.next_frame(0), 0)
        buffered = receiver.buffered_bytes
        self.assertGreater(buffered, 0)
        # 会话中途来新首帧：整次失败，绝不无声覆盖
        with self.assertRaises(TransportError) as cm:
            receiver.on_frame(make_first_frame(ID_TX, len(other), other[:6]), 1)
        self.assertEqual(cm.exception.kind, "format")
        self.assertEqual(receiver.buffered_bytes, 0)
        self.assertFalse(receiver.receiving)

    def test_consecutive_frame_without_session_rejected(self):
        receiver = Receiver(ID_RX)
        with self.assertRaises(TransportError) as cm:
            receiver.on_frame(make_consecutive_frame(ID_TX, 1, b"x"), 0)
        self.assertEqual(cm.exception.kind, "format")

    def test_flow_control_timeout_cleans_buffer(self):
        payload = make_payload(50)
        receiver = Receiver(ID_RX, timeout=10)
        sender = TransportSender(ID_TX, payload)
        res = receiver.on_frame(sender.start(0), 0)
        sender.on_frame(res.flow_frames[0], 0)
        receiver.on_frame(sender.next_frame(0), 0)
        self.assertGreater(receiver.buffered_bytes, 0)
        receiver.on_tick(10)  # 未超时
        self.assertTrue(receiver.receiving)
        with self.assertRaises(TransportError) as cm:
            receiver.on_tick(11)
        self.assertEqual(cm.exception.kind, "timeout")
        self.assertEqual(receiver.buffered_bytes, 0)
        self.assertFalse(receiver.receiving)
        # 清理后可以正常开始新会话
        res2 = receiver.on_frame(sender.start.__self__ and
                                 make_first_frame(ID_TX, 3, b"abc"), 12)
        self.assertEqual(res2.delivered, b"abc")

    def test_wait_frames_limited(self):
        payload = make_payload(50)
        sender = TransportSender(ID_TX, payload, max_waits=2)
        sender.start(0)
        wait = make_flow_control_frame(ID_RX, FS_WAIT, 0, 0)
        sender.on_frame(wait, 0)
        sender.on_frame(wait, 1)
        with self.assertRaises(TransportError) as cm:
            sender.on_frame(wait, 2)
        self.assertEqual(cm.exception.kind, "wait_exceeded")

    def test_wait_then_cts_still_delivers(self):
        payload = make_payload(40)
        sender = TransportSender(ID_TX, payload, max_waits=5)
        receiver = Receiver(ID_RX)
        res = receiver.on_frame(sender.start(0), 0)
        # 先 WAIT 两次再 CTS（模拟接收方暂停续传）
        wait = make_flow_control_frame(ID_RX, FS_WAIT, 0, 0)
        sender.on_frame(wait, 0)
        sender.on_frame(wait, 1)
        self.assertIsNone(sender.next_frame(1))  # 暂停期间不发
        cts = res.flow_frames[0]
        self.assertEqual(cts.data[0] & 0x0F, FS_CTS)
        sender.on_frame(cts, 2)
        delivered = None
        tick = 2
        while not sender.done:
            frame = sender.next_frame(tick)
            if frame is None:
                tick += 1
                continue
            res = receiver.on_frame(frame, tick)
            if res.delivered is not None:
                delivered = res.delivered
        self.assertEqual(delivered, payload)


class ContractTests(unittest.TestCase):
    def test_validation_rejects_bad_inputs(self):
        with self.assertRaises(ValidationError):
            TransportSender(ID_TX, b"")
        with self.assertRaises(ValidationError):
            TransportSender(ID_TX, b"x" * (MAX_MESSAGE_BYTES + 1))
        with self.assertRaises(ValidationError):
            TransportSender(ID_TX, "not-bytes")
        with self.assertRaises(ValidationError):
            TransportSender(True, b"data")
        with self.assertRaises(ValidationError):
            TransportSender(ID_TX, b"data", max_waits=True)
        with self.assertRaises(ValidationError):
            Receiver(ID_RX, block_size=256)
        with self.assertRaises(ValidationError):
            Receiver(ID_RX, timeout=0)
        with self.assertRaises(ValidationError):
            make_first_frame(ID_TX, 0, b"")
        with self.assertRaises(ValidationError):
            make_consecutive_frame(ID_TX, 16, b"")

    def test_frame_layout_matches_readme(self):
        ff = make_first_frame(ID_TX, 0x123, b"abcdef")
        self.assertEqual(ff.data, bytes([0x11, 0x23]) + b"abcdef")
        cf = make_consecutive_frame(ID_TX, 15, b"1234567")
        self.assertEqual(cf.data, bytes([0x2F]) + b"1234567")
        fc = make_flow_control_frame(ID_RX, FS_CTS, 8, 2)
        self.assertEqual(fc.data, bytes([0x30, 8, 2]))

    def test_first_frame_carries_at_most_6_and_cf_7(self):
        self.assertEqual(FF_DATA_BYTES, 6)
        self.assertEqual(CF_DATA_BYTES, 7)
        with self.assertRaises(ValidationError):
            make_first_frame(ID_TX, 10, b"1234567")
        with self.assertRaises(ValidationError):
            make_consecutive_frame(ID_TX, 1, b"12345678")


if __name__ == "__main__":
    unittest.main()

