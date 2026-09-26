"""边沿时间输入入口：位定时参数、确定性发送时钟偏差/毛刺合成与接收恢复。

本轮新增契约（**仍是明确定义的合成模型，不宣称完整控制器兼容**）：

* 每个位由整数个时间量子（tq）组成：tq_per_bit = 1(同步段) + prop_seg
  + phase_seg1 + phase_seg2；采样点固定位内偏移
  sample_point = 1 + prop_seg + phase_seg1（调用方只给传播段、相位段
  与允许的重同步幅度 sjw，采样点由这些参数算出，不预先知道原始位串）。
* 输入是**边沿时间表**（电平跳变时刻），不是 0/1 位串；位串由接收端按
  采样点恢复。旧的直接位流入口（decode_frame 等）继续保留。
* 同步分两类，分开记录：
    - 帧起始硬同步（hard）：第一个隐性->显性边沿定义为 SOF 位起点；
    - 帧内重同步（resync）：只认隐性->显性边沿，调整幅度以 sjw 为界。
      边沿早于期望位结束 -> 缩短当前位；晚于期望位结束（且不晚于下一个
      采样点）-> 延长当前位；落在更后面的边沿属于更晚的位边界，不消费。
* 发送侧用确定性时钟偏差（skew_ppm，整数百万分比）与短毛刺（隐性位内
  宽度 <= glitch_filter_tq 的短显性脉冲）合成边沿；接收端把宽度
  <= glitch_filter_tq 的显性脉冲判为毛刺，不参与同步（但仍影响电平采样，
  与真实采样行为一致）。
* 恢复出的位流先做增量去填充，凑齐 ID+DLC+数据+CRC 后交给已有的
  decode_frame 做帧检查；每个采样时刻、每次相位修正与最终帧状态都记录
  在 ReceiveReport 中。

所有时间量为 fractions.Fraction（精确有理数），本模块无任何浮点运算。
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from fractions import Fraction

from .errors import ValidationError
from .framing import (
    CRC_BITS,
    DLC_BITS,
    ID_BITS,
    MAX_DATA_LEN,
    DecodedFrame,
    _bits_to_int,
    decode_frame,
    destuff_bits,
)
from .framing import check_int as _check_int

__all__ = [
    "BitTiming",
    "Edge",
    "Glitch",
    "SampleRecord",
    "ReceiveReport",
    "synthesize_edges",
    "receive_edges",
]

SKEW_PPM_MAX = 500_000  # 发送时钟偏差上限：±50%（合成模型的接口边界）
DEFAULT_MAX_BITS = 200  # 接收端采样位数上限（最大帧约 125 位，留有余量）


# ---- 位定时参数 ------------------------------------------------------------


@dataclass(frozen=True)
class BitTiming:
    """位定时参数（单位都是时间量子 tq，整数）。

    prop_seg:   传播段 tq 数，1..16
    phase_seg1: 相位缓冲段 1，1..16
    phase_seg2: 相位缓冲段 2，1..16
    sjw:        重同步跳转宽度（单次相位调整上限），1..phase_seg2
    """

    prop_seg: int
    phase_seg1: int
    phase_seg2: int
    sjw: int

    def __post_init__(self) -> None:
        _check_int(self.prop_seg, "prop_seg", 1, 16)
        _check_int(self.phase_seg1, "phase_seg1", 1, 16)
        _check_int(self.phase_seg2, "phase_seg2", 1, 16)
        _check_int(self.sjw, "sjw", 1, self.phase_seg2)

    @property
    def tq_per_bit(self) -> int:
        """每位的 tq 总数：1（同步段）+ 传播段 + 相位段 1 + 相位段 2。"""
        return 1 + self.prop_seg + self.phase_seg1 + self.phase_seg2

    @property
    def sample_point(self) -> int:
        """采样点的位内 tq 偏移：1（同步段）+ 传播段 + 相位段 1。"""
        return 1 + self.prop_seg + self.phase_seg1


# ---- 边沿与毛刺 ------------------------------------------------------------


@dataclass(frozen=True)
class Edge:
    """一次电平跳变：time 时刻之后总线电平变为 level（0 显性 / 1 隐性）。"""

    time: Fraction
    level: int

    def __post_init__(self) -> None:
        if not isinstance(self.time, Fraction):
            raise ValidationError(
                f"edge time must be Fraction, got {type(self.time).__name__}",
                "time",
            )
        _check_int(self.level, "level", 0, 1)


@dataclass(frozen=True)
class Glitch:
    """确定性短毛刺：在 bit_index 位（必须为隐性位）内注入短显性脉冲。

    offset_tq: 脉冲起点相对该位起点的 tq 偏移（1..tq_per_bit-2）
    width_tq:  脉冲宽度（>=1；接收端按 glitch_filter_tq 判定并滤除）
    """

    bit_index: int
    offset_tq: int
    width_tq: int

    def __post_init__(self) -> None:
        _check_int(self.bit_index, "bit_index", 0, 1_000_000)
        _check_int(self.offset_tq, "offset_tq", 1, 1_000_000)
        _check_int(self.width_tq, "width_tq", 1, 1_000_000)


def synthesize_edges(
    bits: list[int] | tuple[int, ...],
    timing: BitTiming,
    *,
    skew_ppm: int = 0,
    glitches: tuple[Glitch, ...] | list[Glitch] = (),
) -> list[Edge]:
    """用确定性发送时钟合成边沿时间表。

    bits:     理想线上位串（0/1 整数序列，例如 EncodedFrame.stream）
    skew_ppm: 发送时钟偏差，整数百万分比；发送位宽 =
              tq_per_bit * (1 + skew_ppm/1e6)，用 Fraction 精确表示
    glitches: 短显性脉冲注入（只允许落在隐性位内）

    返回按时间排序的 Edge 列表；初始电平为隐性 1。相同输入必然得到
    相同输出（无随机源、无浮点）。
    """
    if not isinstance(timing, BitTiming):
        raise ValidationError(
            f"timing must be BitTiming, got {type(timing).__name__}", "timing"
        )
    if not isinstance(bits, (list, tuple)):
        raise ValidationError(
            f"bits must be list/tuple, got {type(bits).__name__}", "bits"
        )
    for i, bit in enumerate(bits):
        _check_int(bit, f"bits[{i}]", 0, 1)
    _check_int(skew_ppm, "skew_ppm", -SKEW_PPM_MAX, SKEW_PPM_MAX)
    glitches = list(glitches)
    for g in glitches:
        if not isinstance(g, Glitch):
            raise ValidationError(
                f"glitches entries must be Glitch, got {type(g).__name__}",
                "glitches",
            )
        if g.bit_index >= len(bits):
            raise ValidationError(
                f"glitch bit_index {g.bit_index} out of range", "glitches"
            )
        if bits[g.bit_index] != 1:
            raise ValidationError(
                "glitch requires a recessive (1) bit slot", "glitches"
            )
        if g.offset_tq + g.width_tq >= timing.tq_per_bit:
            raise ValidationError(
                "glitch must end before its bit slot ends", "glitches"
            )

    bit_len = Fraction(timing.tq_per_bit) * Fraction(1_000_000 + skew_ppm, 1_000_000)
    edges: list[Edge] = []
    t = Fraction(0)
    prev = 1  # 总线空闲为隐性
    for bit in bits:
        if bit != prev:
            edges.append(Edge(t, bit))
            prev = bit
        t += bit_len
    for g in glitches:
        start = bit_len * g.bit_index + g.offset_tq
        edges.append(Edge(start, 0))
        edges.append(Edge(start + g.width_tq, 1))
    edges.sort(key=lambda e: e.time)
    return edges


# ---- 接收端：采样、有界相位调整、帧检查 -------------------------------------


@dataclass(frozen=True)
class SampleRecord:
    """一次采样的可审计记录。

    sync:       "hard"（本位由帧起始硬同步定位）|"resync"（帧内重同步）|""
    correction: 本位起点相对标称起点的相位修正（tq，Fraction，带符号；
                正=延长上一位，负=缩短上一位，0=无修正），绝对值 <= sjw
    """

    bit_index: int
    sample_time: Fraction
    bit: int
    sync: str
    correction: Fraction


@dataclass(frozen=True)
class ReceiveReport:
    """边沿接收的完整报告。

    status: "ok" | "no_frame" | "stuff_error" | "crc_error"
            | "protocol_error" | "length_error" | "incomplete"
    """

    samples: tuple[SampleRecord, ...]
    recovered_bits: tuple[int, ...]
    status: str
    decoded: DecodedFrame | None
    error: str

    @property
    def ok(self) -> bool:
        return self.status == "ok"

    @property
    def total_correction(self) -> Fraction:
        """全部相位修正的代数和（tq）。"""
        total = Fraction(0)
        for s in self.samples:
            total += s.correction
        return total


def _level_at(times: list[Fraction], edges: list[Edge], t: Fraction) -> int:
    i = bisect_right(times, t)
    return edges[i - 1].level if i else 1


def _try_decode(
    recovered: list[int],
) -> tuple[str, DecodedFrame | None, str] | None:
    """增量帧检查。返回 None 表示“位还不够，继续采样”。"""
    stuffed = recovered[1:]  # 去掉 SOF
    if len(stuffed) < ID_BITS + DLC_BITS:
        return None
    res = destuff_bits(stuffed)
    if res.error_index is not None and res.error_index < len(stuffed):
        return ("stuff_error", None,
                f"stuff rule violated at stuffed index {res.error_index}")
    raw = res.bits
    if len(raw) < ID_BITS + DLC_BITS:
        return None
    dlc = _bits_to_int(raw[ID_BITS : ID_BITS + DLC_BITS])
    if dlc > MAX_DATA_LEN:
        return ("protocol_error", None, f"decoded DLC={dlc} exceeds max 8")
    needed = ID_BITS + DLC_BITS + 8 * dlc + CRC_BITS
    if len(raw) < needed:
        return None
    # 第 needed 个原始位在填充后流中的位置；若其后紧跟填充位则一并取入
    raw_no = -1
    cut = -1
    for i, is_stuff in enumerate(res.stuffing_map):
        if not is_stuff:
            raw_no += 1
            if raw_no == needed - 1:
                cut = i
                break
    end = cut + 1
    if end < len(stuffed) and res.stuffing_map[end]:
        end += 1
    elif end == len(stuffed) and res.error_index == len(stuffed):
        return None  # 尾部填充位还没采到
    prefix = stuffed[:end]
    try:
        dec = decode_frame(prefix, raise_on_crc=False)
    except Exception as exc:  # FrameDecodeError：kind 即状态
        kind = getattr(exc, "kind", "length")
        return (f"{kind}_error", None, str(exc))
    if dec.crc_ok:
        return ("ok", dec, "")
    return ("crc_error", None,
            f"CRC mismatch: got 0x{dec.crc:04X}")


def receive_edges(
    edges: list[Edge] | tuple[Edge, ...],
    timing: BitTiming,
    *,
    glitch_filter_tq: int = 1,
    max_bits: int = DEFAULT_MAX_BITS,
) -> ReceiveReport:
    """从边沿时间表恢复位流并做帧检查。

    只在隐性->显性边沿做相位调整：帧起始硬同步一次，帧内重同步每次
    幅度以 timing.sjw 为界；宽度 <= glitch_filter_tq 的显性脉冲判为
    毛刺，不参与同步。采样点由 BitTiming 计算，不访问原始位串。
    """
    if not isinstance(timing, BitTiming):
        raise ValidationError(
            f"timing must be BitTiming, got {type(timing).__name__}", "timing"
        )
    edges = list(edges)
    for e in edges:
        if not isinstance(e, Edge):
            raise ValidationError(
                f"edges entries must be Edge, got {type(e).__name__}", "edges"
            )
    if any(edges[i].time > edges[i + 1].time for i in range(len(edges) - 1)):
        raise ValidationError("edges must be sorted by time", "edges")
    _check_int(glitch_filter_tq, "glitch_filter_tq", 0, timing.tq_per_bit)
    _check_int(max_bits, "max_bits", 1, 1_000_000)

    times = [e.time for e in edges]
    # 毛刺过滤：1->0 边沿后 glitch_filter_tq 之内就回到隐性的，不是同步边沿
    sync_edges: list[Edge] = []
    for i, e in enumerate(edges):
        if e.level != 0:
            continue
        nxt = edges[i + 1] if i + 1 < len(edges) else None
        if nxt is not None and nxt.level == 1 and nxt.time - e.time <= glitch_filter_tq:
            continue  # 短显性脉冲 = 毛刺
        sync_edges.append(e)

    if not sync_edges:
        return ReceiveReport((), (), "no_frame", None,
                             "no recessive-to-dominant edge found")

    records: list[SampleRecord] = []
    recovered: list[int] = []
    status, decoded, error = "incomplete", None, (
        f"frame not completed within max_bits={max_bits}"
    )

    bit_start = sync_edges[0].time  # 帧起始硬同步
    edge_ptr = 1
    pending_sync = "hard"
    pending_corr = Fraction(0)
    tq = timing.tq_per_bit

    for bit_index in range(max_bits):
        sample_time = bit_start + timing.sample_point
        bit = _level_at(times, edges, sample_time)
        records.append(SampleRecord(bit_index, sample_time, bit,
                                    pending_sync, pending_corr))
        recovered.append(bit)

        outcome = _try_decode(recovered)
        if outcome is not None:
            status, decoded, error = outcome
            break

        expected_end = bit_start + tq
        edge = None
        while edge_ptr < len(sync_edges):
            cand = sync_edges[edge_ptr]
            if cand.time <= sample_time:
                edge_ptr += 1  # 属于更早位边界的边沿，跳过
                continue
            edge = cand
            break

        pending_sync = ""
        pending_corr = Fraction(0)
        if edge is not None and edge.time <= expected_end + timing.sample_point:
            edge_ptr += 1
            if edge.time < expected_end:
                # 边沿提前：缩短当前位，最多 sjw
                shift = min(Fraction(timing.sjw), expected_end - edge.time)
                bit_start = expected_end - shift
                pending_corr = -shift
            elif edge.time > expected_end:
                # 边沿滞后：延长当前位，最多 sjw
                shift = min(Fraction(timing.sjw), edge.time - expected_end)
                bit_start = expected_end + shift
                pending_corr = shift
            else:
                bit_start = expected_end
            if pending_corr:
                pending_sync = "resync"
        else:
            bit_start = expected_end  # 本边界无可用边沿：标称推进

    return ReceiveReport(
        samples=tuple(records),
        recovered_bits=tuple(recovered),
        status=status,
        decoded=decoded,
        error=error,
    )

