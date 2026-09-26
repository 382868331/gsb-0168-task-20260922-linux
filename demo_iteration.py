"""传输层演示：本地事件序列驱动两端，真实计算正常与边界结果。

固定输入、离线、无 sleep、无预录输出；实测远小于 90 秒预算。
"""

from canbit import (
    FS_WAIT,
    Receiver,
    TransportError,
    TransportSender,
    make_first_frame,
    make_flow_control_frame,
)

ID_TX = 0x700
ID_RX = 0x708


def make_payload(length, seed=0xC0DE):
    state = seed
    out = bytearray()
    while len(out) < length:
        state = (state * 1103515245 + 12345) & 0x7FFFFFFF
        out.append((state >> 13) & 0xFF)
    return bytes(out)


def drive(payload, *, block_size=0, st_min=0, drop_cf=(), timeout=100):
    """事件序列驱动：返回 (delivered, cf_count, last_tick)。"""
    sender = TransportSender(ID_TX, payload)
    receiver = Receiver(ID_RX, block_size=block_size, st_min=st_min,
                        timeout=timeout)
    tick = 0
    delivered = None
    res = receiver.on_frame(sender.start(tick), tick)
    delivered = res.delivered
    pending = list(res.flow_frames)
    cf_count = 0
    while not sender.done:
        if pending:
            sender.on_frame(pending.pop(0), tick)
            continue
        frame = sender.next_frame(tick)
        if frame is None:
            tick += 1
            receiver.on_tick(tick)
            continue
        drop = cf_count in drop_cf
        cf_count += 1
        if drop:
            continue
        res = receiver.on_frame(frame, tick)
        if res.delivered is not None:
            delivered = res.delivered
        pending.extend(res.flow_frames)
    return delivered, cf_count, tick


def main():
    print("=== 正常：200 字节消息，块大小 4，st_min 2 tick ===")
    payload = make_payload(200)
    delivered, cf_count, ticks = drive(payload, block_size=4, st_min=2)
    ok = delivered == payload
    print(f"连续帧 {cf_count} 个（序号模 16 回绕 {cf_count // 16} 次），"
          f"用时 {ticks} tick")
    print(f"交付 {len(delivered)} 字节，与原字节串逐字节一致: {ok}")
    assert ok

    print()
    print("=== 边界 1：丢帧 -> 序号跳跃，整次失败 ===")
    try:
        drive(make_payload(80), drop_cf={3})
        print("意外成功（不应发生）")
    except TransportError as exc:
        print(f"按预期失败: kind={exc.kind} ({exc})")

    print()
    print("=== 边界 2：接收方暂停续传，WAIT 超限中止 ===")
    sender = TransportSender(ID_TX, make_payload(50), max_waits=2)
    sender.start(0)
    wait = make_flow_control_frame(ID_RX, FS_WAIT, 0, 0)
    try:
        for tick in range(3):
            sender.on_frame(wait, tick)
        print("意外成功（不应发生）")
    except TransportError as exc:
        print(f"按预期失败: kind={exc.kind} ({exc})")

    print()
    print("=== 边界 3：对端沉默 -> 逻辑时钟超时清理缓存 ===")
    receiver = Receiver(ID_RX, timeout=10)
    receiver.on_frame(make_first_frame(ID_TX, 50, b"abcdef"), 0)
    print(f"超时前缓存 {receiver.buffered_bytes} 字节")
    try:
        receiver.on_tick(11)
        print("意外成功（不应发生）")
    except TransportError as exc:
        print(f"按预期失败: kind={exc.kind}，缓存已清理为 "
              f"{receiver.buffered_bytes} 字节")

    print()
    print("=== 边界 4：会话中途来新首帧，不无声覆盖 ===")
    receiver = Receiver(ID_RX)
    sender = TransportSender(ID_TX, make_payload(60))
    res = receiver.on_frame(sender.start(0), 0)
    sender.on_frame(res.flow_frames[0], 0)
    receiver.on_frame(sender.next_frame(0), 0)
    try:
        receiver.on_frame(make_first_frame(ID_TX, 10, b"012345"), 1)
        print("意外成功（不应发生）")
    except TransportError as exc:
        print(f"按预期失败: kind={exc.kind} ({exc})")


if __name__ == "__main__":
    main()

