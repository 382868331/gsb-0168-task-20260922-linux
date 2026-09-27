#!/usr/bin/env python3
"""canbit 固定输入演示：真实计算仲裁胜者与各错误位置（无预录结果）。

运行：python demo.py   （Python 3.14.7，仅标准库，离线）

四个场景全部由代码即时计算：
  1. 两个不同 ID 同时发送 —— 真实仲裁，打印胜者与败者失配位
  2. 同 ID 不同负载 —— 独立编码产物不同（CRC/位流对比）
  3. 填充位被外力破坏 —— 真实定位首个填充位并仿真错误帧中止
  4. CRC 被篡改 + 非法输入边界 —— 解码异常与输入拒绝真实触发
"""

from canbit import (
    BitBus,
    Frame,
    RetryScheduler,
    Sender,
    bit_stuff,
    compute_crc15,
    decode_frame,
    encode_frame,
    payload_bits,
)
from canbit.errors import FrameDecodeError, ValidationError


def hr(title: str) -> None:
    print("\n" + "=" * 68)
    print(title)
    print("=" * 68)


def main() -> None:
    # -----------------------------------------------------------------
    # 场景 1：正常结果 —— 两个 ID 的真实仲裁
    # -----------------------------------------------------------------
    hr("场景 1（正常）：两个发送者同时启动，逐 bit 仲裁")
    id_a, id_b = 0x321, 0x320
    senders = [
        Sender("ECU_A", Frame(id_a, b"hi")),
        Sender("ECU_B", Frame(id_b, b"yo")),
    ]
    print(f"ECU_A ID=0x{id_a:03X} 数据=b'hi' ; ECU_B ID=0x{id_b:03X} 数据=b'yo'")
    print("显性 0 胜隐性 1；ID 域发 1 读 0 仅退出仲裁。逐 bit 推进：")

    bus = BitBus(senders, ack_pullers={"RX_node"})
    loser_pos = None
    while not bus.finished:
        sr = bus.step()
        if sr.arbitration_losers:
            loser_pos = sr.bit_index
            print(
                f"  bit#{sr.bit_index:>3}: 总线={sr.bus_bit}  "
                f"-> {','.join(sr.arbitration_losers)} 在 ID 域发1读0，退出仲裁"
            )
    result = bus.result()
    print(f"真实仲裁胜者: {result.winners[0]} (ID=0x{id_b:03X}，数值更低)")
    print(f"败者失配位（线上绝对索引，SOF 为 0）: {loser_pos}")
    print(f"ACK 槽被 RX_node 拉低: {result.acked}；传输完整完成: {result.completed}")
    print(f"胜者帧解码: ID=0x{result.decoded_frame.identifier:03X}, "
          f"data={result.decoded_frame.data!r}, CRC 校验={result.decoded_crc_ok}")

    # -----------------------------------------------------------------
    # 场景 2：同 ID 不同负载 —— 编码参考必须区分
    # -----------------------------------------------------------------
    hr("场景 2（编码参考）：同 ID=0x123，不同负载")
    for tag, data in [("空负载", b""), ("负载1", b"CAN"), ("负载2", bytes(range(8)))]:
        crc = compute_crc15(0x123, len(data), data)
        enc = encode_frame(Frame(0x123, data))
        print(f"  {tag:6s} data={data!r:20s} CRC15=0x{crc:04X} "
              f"填充后 ID..CRC 长度={len(enc.stuffed)} 位")
    print("-> 同 ID 不同负载得到不同 CRC 与位流（运行时实际计算）")

    # -----------------------------------------------------------------
    # 场景 3：填充位破坏 —— 真实定位 + 总线错误帧
    # -----------------------------------------------------------------
    hr("场景 3（错误边界）：外力破坏首个填充位，错误帧中止传输")
    frame = Frame(0x001, bytes(8))
    enc = encode_frame(frame)
    first_stuff = next(i + 1 for i, m in enumerate(enc.stuffing_map) if m == 1)
    forced_value = enc.stream[first_stuff - 1]  # 强制成前一位的同值
    print(f"帧 ID=0x001 数据=8 字节全 0；首个填充位线上索引={first_stuff}，"
          f"原值={enc.stream[first_stuff]}")
    print(f"外力在该位强制为 {forced_value}（与前 5 连同值相同，违反填充规则）")

    bad_bus = BitBus(
        [Sender("ECU_A", frame)],
        ack_pullers={"RX_node"},
        forced_bits={first_stuff: forced_value},
    )
    bad = bad_bus.run()
    ef = bad.error_frame
    print(f"-> 触发 {ef.trigger}，检测位={ef.bit_index}，"
          f"参与者={ef.participants}，中止的活跃发送者={ef.aborted}")
    n_flags = sum(1 for e in bad.events if e.kind == "error_flag")
    print(f"-> 随后发出 {n_flags} 位显性错误标志，当前传输中止，completed={bad.completed}")

    # 纯解码视角也应在同一位置报填充错误（解码器按“填充后流”编号，
    # 不含 SOF，故索引比总线绝对索引小 1）
    corrupted = list(enc.stuffed)
    corrupted[first_stuff - 1] = enc.stuffed[first_stuff - 2]
    try:
        decode_frame(corrupted)
    except FrameDecodeError as exc:
        print(f"-> decode_frame 独立复核（不含 SOF 的填充后流索引）: {exc}")
        print(f"   即 {exc.bit_index} + 1(SOF) = 总线索引 {exc.bit_index + 1}，"
              f"与上面的检测位一致")

    # -----------------------------------------------------------------
    # 场景 4：CRC 篡改（去填充后 CRC 检查）与非法输入拒绝
    # -----------------------------------------------------------------
    hr("场景 4（错误边界）：CRC 字段被篡改 -> 去填充后 CRC 检查失败")
    ident, data = 0x201, b"payload!"
    good = encode_frame(Frame(ident, data))
    msg = payload_bits(ident, len(data), data)
    tampered = bit_stuff(msg + [0] * 15)  # 填充结构合法，CRC 字段被换成全 0
    print(f"原 CRC15=0x{good.crc:04X}（非 0），篡改后 CRC 字段=0x0000，"
          f"填充结构保持合法")
    try:
        decode_frame(tampered)
    except FrameDecodeError as exc:
        print(f"-> 实际触发异常: {type(exc).__name__}: {exc}")

    hr("场景 4b（输入拒绝边界）：非法构造真实抛出可定位异常")
    for label, make in [
        ("ID 超 11 位上限 0x800", lambda: Frame(0x800, b"")),
        ("bool 冒充 int", lambda: Frame(True, b"")),
        ("数据超 8 字节", lambda: Frame(0, b"x" * 9)),
        ("NaN", lambda: compute_crc15(float("nan"), 0, b"")),
    ]:
        try:
            make()
        except ValidationError as exc:
            print(f"  拒绝 [{label}] -> {type(exc).__name__}: {exc}")
        else:  # pragma: no cover - demo 不应走到这里
            raise AssertionError(f"illegal input was silently accepted: {label}")

    # -----------------------------------------------------------------
    # 场景 5：待发送队列 —— 仲裁失败者按原规则重试，终态分类记录
    # -----------------------------------------------------------------
    hr("场景 5（重试队列）：仲裁失败重新排队，错误与仲裁分开计数")
    sched = RetryScheduler(
        [
            Sender("ECU_A", Frame(0x102, b"A")),
            Sender("ECU_B", Frame(0x100, b"B")),
            Sender("ECU_C", Frame(0x101, b"C")),
        ],
        max_retries=2,
        ack_pullers={"RX_node"},
    )
    print("三个发送者同时待发送（ID 0x102/0x100/0x101），max_retries=2：")
    while not sched.finished:
        sched.step()
    for rnd in sched.rounds:
        print(f"  第{rnd.round_index}轮: 参与者={rnd.participants} "
              f"胜者={rnd.result.winners} 重排队={rnd.requeued} 耗尽={rnd.exhausted}")
    for rec in sched.records:
        print(f"  {rec.name}: status={rec.status} attempts={rec.attempts} "
              f"仲裁重试={rec.arbitration_retries} 错误重试={rec.error_retries}")

    print("取消只影响排队中的帧：")
    sched2 = RetryScheduler(
        [Sender("X", Frame(0x100, b"x")), Sender("Y", Frame(0x101, b"y"))],
        max_retries=1,
        ack_pullers={"RX_node"},
    )
    sched2.step()  # 第一轮开始，X/Y 均在发送中
    print(f"  取消发送中的 X -> {sched2.cancel('X')}（不受影响，本轮照常完成）")
    sched2.enqueue(Sender("Z", Frame(0x102, b"z")))
    print(f"  取消排队中的 Z -> {sched2.cancel('Z')}")
    sched2.run()
    for rec in sched2.records:
        print(f"  {rec.name}: status={rec.status} attempts={rec.attempts}")

    hr("演示结束")
    print("所有结果均为运行时真实计算；本模型仅实现本题契约子集，")
    print("不包含重传、错误界定/被动错误计数等完整 CAN 机制。")


if __name__ == "__main__":
    main()
