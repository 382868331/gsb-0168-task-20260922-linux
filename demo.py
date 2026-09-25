#!/usr/bin/env python3
"""canbit 固定输入演示：真实计算仲裁胜者与各错误位置（无预录结果）。

运行：./.venv/bin/python demo.py   （Python 3.14.7，仅标准库，离线）

五个场景全部由代码即时计算：
  1. 两个不同 ID 同时发送 —— 真实仲裁，打印胜者与败者失配位
  2. 同 ID 不同负载 —— 独立编码产物不同（CRC/位流对比）
  3. 填充位被外力破坏 —— 真实定位首个填充位并仿真错误帧中止
  4. CRC 被篡改 + 非法输入边界 —— 解码异常与输入拒绝真实触发
  5. 接收过滤表 —— 全过滤仍 ACK、重叠规则、CRC 损坏记录、
     仲裁失败者转接收、只有监听者时 ack_error
"""

from canbit import (
    BitBus,
    BusNode,
    FilterRule,
    Frame,
    NodeBus,
    Sender,
    bit_stuff,
    compute_crc15,
    decode_frame,
    encode_frame,
    encode_frame_with_crc,
    payload_bits,
)
from canbit.errors import FrameDecodeError, ValidationError


def hr(title: str) -> None:
    print("\n" + "=" * 68)
    print(title)
    print("=" * 68)


def demo_filters() -> None:
    """场景 5：接收过滤表（NodeBus），全部逐 bit 驱动原总线真实计算。"""
    hr("场景 5（接收过滤表）：过滤只决定上交，不影响 ACK 与 CRC 校验")

    # 5a：全过滤（空表）仍有 ACK ------------------------------------------
    print("\n[5a] 全过滤（空过滤表）：不上交，但正常接收节点仍 ACK")
    bus = NodeBus([
        BusNode("TX", tx_frame=Frame(0x2A5, b"data")),
        BusNode("RX", filters=()),
    ])
    while not bus.finished:
        bus.step()
    res = bus.result()
    (rec,) = res.records_of("RX")
    print(f"  传输完成={res.bus.completed}，RX 拉低 ACK={res.bus.acked}")
    print(f"  RX 记录：crc_ok={rec.crc_ok}（过滤不跳过校验），"
          f"命中规则={rec.matched_rules}，delivered={rec.delivered}")

    # 5b：重叠规则只交付一次，取最早规则序号 ------------------------------
    print("\n[5b] 两条重叠规则命中同一帧：只交付一次，依据最早规则")
    rules = (FilterRule(0x100, 0x700), FilterRule(0x180, 0x780))
    bus = NodeBus([
        BusNode("TX", tx_frame=Frame(0x1A0, b"\xAA\x55")),
        BusNode("RX", filters=rules),
    ])
    res = bus.run()
    (rec,) = res.records_of("RX")
    print(f"  规则表: (id=0x100,mask=0x700), (id=0x180,mask=0x780)；"
          f"帧 ID=0x1A0 同时命中两条")
    print(f"  RX 记录：命中规则={rec.matched_rules}，delivered={rec.delivered}，"
          f"交付依据 rule_index={rec.rule_index}（最早规则）")
    print(f"  交付次数={len(res.deliveries_of('RX'))}（重叠只交付一次）")

    # 5c：匹配帧 CRC 损坏：记录校验结果但不上交 ---------------------------
    print("\n[5c] 命中过滤的帧 CRC 损坏：照常 ACK 与记录，不上交")
    frame = Frame(0x2A5, b"payload!")
    good_crc = compute_crc15(frame.identifier, frame.dlc, frame.data)
    bad = encode_frame_with_crc(frame, good_crc ^ 0x0001)
    print(f"  故障注入：CRC 字段 0x{good_crc:04X} -> 0x{bad.crc:04X}"
          f"（填充结构保持合法）")
    bus = NodeBus(
        [
            BusNode("TX", tx_frame=frame),
            BusNode("RX", filters=(FilterRule(0x2A5, 0x7FF),)),
            BusNode("LIS", filters=(FilterRule(0x2A5, 0x7FF),), listen_only=True),
        ],
        encoded_overrides={"TX": bad},
    )
    res = bus.run()
    print(f"  传输完成={res.bus.completed}，ACK={res.bus.acked}")
    for name in ("RX", "LIS"):
        (rec,) = res.records_of(name)
        print(f"  {name} 记录：crc_ok={rec.crc_ok}，命中={rec.matched_rules}，"
              f"delivered={rec.delivered}")

    # 5d：仲裁失败节点转接收 ----------------------------------------------
    print("\n[5d] 仲裁失败节点转接收：失败者参与 ACK 并收下胜者帧")
    bus = NodeBus([
        BusNode("A", tx_frame=Frame(0x320, b"from-A")),
        BusNode("B", filters=(FilterRule(0x320, 0x7FF),),
                tx_frame=Frame(0x321, b"from-B")),
    ])
    losers = []
    while not bus.finished:
        sr = bus.step()
        losers.extend(sr.arbitration_losers)
    res = bus.result()
    (rec_b,) = res.records_of("B")
    print(f"  仲裁失败者={losers}，胜者={res.bus.winners}，"
          f"ACK={res.bus.acked}（总线仅 A/B，证明 B 失败后转为接收者拉低 ACK）")
    print(f"  B 记录：ID=0x{rec_b.frame.identifier:03X} "
          f"data={rec_b.frame.data!r}，delivered={rec_b.delivered}，"
          f"rule_index={rec_b.rule_index}")

    # 5e：只有监听者 -> ack_error ------------------------------------------
    print("\n[5e] 只有监听者：永不 ACK -> 发送者报 ack_error")
    bus = NodeBus([
        BusNode("TX", tx_frame=Frame(0x123, b"q")),
        BusNode("L1", filters=(FilterRule(0x123, 0x7FF),), listen_only=True),
    ])
    res = bus.run()
    ef = res.bus.error_frame
    print(f"  触发 {ef.trigger}，检测位={ef.bit_index}，参与者={ef.participants}")
    print(f"  传输完成={res.bus.completed}，接收记录数={len(res.records)}"
          f"（帧未完成，无记录）")


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
        ("过滤 mask 超 11 位", lambda: FilterRule(0x123, 0x800)),
        ("监听节点携带发送帧", lambda: BusNode("N", listen_only=True,
                                               tx_frame=Frame(0))),
    ]:
        try:
            make()
        except ValidationError as exc:
            print(f"  拒绝 [{label}] -> {type(exc).__name__}: {exc}")
        else:  # pragma: no cover - demo 不应走到这里
            raise AssertionError(f"illegal input was silently accepted: {label}")

    demo_filters()

    hr("演示结束")
    print("所有结果均为运行时真实计算；本模型仅实现本题契约子集，")
    print("不包含重传、错误界定/被动错误计数等完整 CAN 机制。")


if __name__ == "__main__":
    main()
