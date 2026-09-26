#!/usr/bin/env python3
"""canbit 边沿定时接收入口演示：真实计算正常与边界结果（无预录输出）。

运行：python demo_iteration.py   （Python 3.14.7，仅标准库，离线）

场景全部由代码即时计算（实测 << 90s，无 sleep）：
  1. 理想发送时钟 —— 边沿输入恢复位流，与理想位流逐位对照，帧校验通过
  2. 可修复的发送时钟偏差 —— 有界重同步真实修正，打印每次相位调整
  3. 超出修正范围的偏差 —— 帧检查真实失败，报告错误位置
  4. 短毛刺 —— 伪边沿被有界修正吸收，帧仍恢复
"""

from canbit import (
    BitTiming,
    Frame,
    encode_frame,
    receive_edges,
    simulate_edges,
)

TIMING = BitTiming(prop_seg=3, phase_seg1=3, phase_seg2=3, sjw=2)
FRAME = Frame(0x321, b"hi")


def hr(title):
    print("\n" + "=" * 68)
    print(title)
    print("=" * 68)


def show_result(res, limit=6):
    print(f"  状态: {res.status}  CRC: {res.crc_ok}  帧: {res.frame}")
    print(f"  采样数: {len(res.samples)}  首采样: "
          f"t={res.samples[0].time} bit={res.samples[0].bit}  末采样: "
          f"t={res.samples[-1].time} bit={res.samples[-1].bit}")
    corr = res.corrections
    print(f"  相位修正 {len(corr)} 次（硬同步 "
          f"{sum(1 for c in corr if c.kind == 'hard_sync')} + 重同步 "
          f"{sum(1 for c in corr if c.kind == 'resync')}）:")
    for c in corr[:limit]:
        print(f"    bit#{c.bit_index:3d} {c.kind:9s} 边沿t={c.edge_time:4d} "
              f"误差={c.phase_error:+d} 调整={c.applied:+d}"
              f"{'(被SJW截断)' if c.clamped else ''}")
    if len(corr) > limit:
        print(f"    ... 其余 {len(corr) - limit} 次略")
    if res.status != "ok":
        print(f"  错误位置(采样位): {res.error_bit_index}  详情: {res.detail}")


def main():
    enc = encode_frame(FRAME)
    print(f"位定时: prop_seg={TIMING.prop_seg} ps1={TIMING.phase_seg1} "
          f"ps2={TIMING.phase_seg2} sjw={TIMING.sjw} "
          f"=> {TIMING.tq_per_bit} tq/位, 采样点 @{TIMING.sample_point}")
    print(f"帧: ID=0x{FRAME.identifier:03X} data={FRAME.data!r} "
          f"线上 {len(enc.stream)} 位")

    hr("场景 1：理想发送时钟（正常结果）")
    res = receive_edges(simulate_edges(enc.stream, TIMING), TIMING)
    show_result(res)
    assert res.status == "ok" and list(res.bits) == enc.stream
    print("  对照: 恢复位流与理想位流逐位一致")

    hr("场景 2：可修复的发送时钟偏差（每 8 位慢 1 tq，SJW=2 有界修正）")
    edges = simulate_edges(enc.stream, TIMING, drift_num=1, drift_den=8)
    res = receive_edges(edges, TIMING)
    show_result(res)
    assert res.status == "ok" and res.frame == FRAME
    print("  对照: 偏差被逐次有界重同步吸收，恢复位流与理想位流一致")

    hr("场景 3：超出修正范围的偏差（每位快 1 tq = 10%，远超 SJW=2）")
    edges = simulate_edges(enc.stream, TIMING, drift_num=-1, drift_den=1)
    res = receive_edges(edges, TIMING)
    show_result(res)
    assert res.status != "ok" and res.frame is None
    print("  对照: 采样点滑出位窗口，去填充/帧检查真实报错")

    hr("场景 4：短毛刺（隐性位上 2 tq 显性毛刺，产生伪边沿）")
    edges = simulate_edges(enc.stream, TIMING, glitches=[(22, 0, 2)])
    res = receive_edges(edges, TIMING)
    show_result(res)
    assert res.status == "ok" and res.frame == FRAME
    print("  对照: 伪边沿只触发有界修正，采样点未越界，帧正常恢复")

    print("\n全部场景真实计算完成。")


if __name__ == "__main__":
    main()
