"""边沿时间输入入口演示：正常（可修复偏差+毛刺）与边界（偏差超限）。

真实计算、无 sleep、无预录输出；实测运行远低于 90 秒预算。
"""

from __future__ import annotations

from canbit import (
    BitTiming,
    Frame,
    Glitch,
    encode_frame,
    receive_edges,
    synthesize_edges,
)

TIMING = BitTiming(prop_seg=3, phase_seg1=4, phase_seg2=4, sjw=2)
FRAME = Frame(0x321, b"hi")


def show_report(title: str, rep) -> None:
    print(f"[{title}] status={rep.status} samples={len(rep.samples)}")
    resyncs = [s for s in rep.samples if s.sync == "resync"]
    print(f"  hard sync at bit 0, resync count={len(resyncs)}, "
          f"total correction={rep.total_correction} tq")
    for s in resyncs[:4]:
        print(f"  bit {s.bit_index}: sampled at t={s.sample_time} tq, "
              f"correction={s.correction:+} tq")
    if rep.decoded is not None:
        f = rep.decoded.frame
        print(f"  decoded: ID=0x{f.identifier:03X} data={f.data!r} "
              f"crc_ok={rep.decoded.crc_ok}")
    if rep.error:
        print(f"  error: {rep.error}")


def main() -> None:
    stream = encode_frame(FRAME).stream
    print(f"timing: {TIMING.tq_per_bit} tq/bit, sample at tq {TIMING.sample_point}, "
          f"sjw={TIMING.sjw}; ideal stream {len(stream)} bits")

    # 正常场景：发送时钟快 2%（可修复）+ 两个短毛刺
    recessive = [i for i, b in enumerate(stream) if b == 1]
    glitches = [Glitch(bit_index=recessive[0], offset_tq=2, width_tq=1),
                Glitch(bit_index=recessive[1], offset_tq=9, width_tq=1)]
    edges_ok = synthesize_edges(stream, TIMING, skew_ppm=20_000, glitches=glitches)
    rep_ok = receive_edges(edges_ok, TIMING, glitch_filter_tq=1)
    show_report("normal: +2% clock skew, 2 glitches filtered", rep_ok)
    assert rep_ok.status == "ok" and rep_ok.decoded.frame == FRAME

    # 边界场景：发送时钟快 12%，位间漂移超过 sjw，修正无能为力
    edges_bad = synthesize_edges(stream, TIMING, skew_ppm=120_000)
    rep_bad = receive_edges(edges_bad, TIMING)
    show_report("boundary: +12% clock skew beyond sjw", rep_bad)
    assert rep_bad.status != "ok"

    print("demo_iteration: normal case recovered, boundary case reported "
          f"({rep_bad.status})")


if __name__ == "__main__":
    main()

