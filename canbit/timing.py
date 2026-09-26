"""边沿时间输入的位定时接收模型（合成模型，非完整控制器）。

在既有 0/1 位流入口之外，提供以**边沿时间**为输入的接收入口：

* 调用方给出位定时参数（传播段、相位段 1/2、重同步跳转宽度 SJW），
  每位由整数个时间量子（tq）构成；采样点由参数计算，接收方**不预先
  知道原始位串**。
* 帧起始（空闲->SOF）的隐性到显性边沿做**硬同步**；帧内只在隐性到
  显性边沿做**有界相位调整**（幅度 <= SJW）。两种同步分开记录。
* 恢复出的位流交给既有的去填充与帧检查（decode_frame），报告每个
  采样时刻、每次相位修正与最终帧状态。
* simulate_edges 用确定性发送时钟偏差（有理数 tq/位）与短毛刺
  从理想位流生成边沿序列，用于对照测试：偏差可修复与超出修正范围
  两种情况都能真实算出。

本模块是纯整数运算（无浮点），是明确定义的合成模型，不宣称完整
CAN 控制器兼容（无振荡器容差模型、无三次采样、无错误界定等）。
"""

from __future__ import annotations

from dataclasses import dataclass

from .errors import FrameDecodeError, ValidationError
from .framing import check_int, decode_frame

# 段长上限（合成模型的接口边界，防止无意义的大参数）
MAX_SEG = 64
# 接收终止保护：连续隐性采样数上限（无后续边沿时判帧已结束/损坏）
_IDLE_RUN_STOP = 8
# 采样总数上限（接口边界；正常帧远小于此值）
MAX_SAMPLES = 512

RECESSIVE = 1
DOMINANT = 0


@dataclass(frozen=True)
class BitTiming:
    """位定时参数（单位：时间量子 tq）。

    每位 = 同步段(固定 1) + prop_seg + phase_seg1 + phase_seg2 个 tq；
    采样点位于同步段+传播段+相位段 1 之后。SJW 为重同步的最大有界
    调整幅度，不得超过 min(phase_seg1, phase_seg2)。
    """

    prop_seg: int
    phase_seg1: int
    phase_seg2: int
    sjw: int

    def __post_init__(self) -> None:
        check_int(self.prop_seg, "prop_seg", 1, MAX_SEG)
        check_int(self.phase_seg1, "phase_seg1", 1, MAX_SEG)
        check_int(self.phase_seg2, "phase_seg2", 1, MAX_SEG)
        check_int(self.sjw, "sjw", 1, MAX_SEG)
        if self.sjw > min(self.phase_seg1, self.phase_seg2):
            raise ValidationError(
                f"sjw={self.sjw} must be <= min(phase_seg1, phase_seg2)="
                f"{min(self.phase_seg1, self.phase_seg2)}",
                "sjw",
            )

    @property
    def tq_per_bit(self) -> int:
        """每位的时间量子总数（含 1 tq 同步段）。"""
        return 1 + self.prop_seg + self.phase_seg1 + self.phase_seg2

    @property
    def sample_point(self) -> int:
        """采样点相对位起始的 tq 偏移。"""
        return 1 + self.prop_seg + self.phase_seg1


@dataclass(frozen=True)
class Edge:
    """一次电平跳变：time（tq，>=0）时刻变为 level（0/1）。"""

    time: int
    level: int

    def __post_init__(self) -> None:
        check_int(self.time, "time", 0, 1 << 62)
        check_int(self.level, "level", 0, 1)


def _normalize_edges(edges):
    out = []
    for i, e in enumerate(edges):
        if isinstance(e, Edge):
            edge = e
        elif isinstance(e, tuple) and len(e) == 2:
            edge = Edge(e[0], e[1])
        else:
            raise ValidationError(
                f"edge #{i} must be Edge or (time, level) tuple", "edges"
            )
        if out and edge.time <= out[-1].time:
            raise ValidationError(
                f"edge times must be strictly increasing "
                f"(edge #{i} time={edge.time} <= {out[-1].time})",
                "edges",
            )
        out.append(edge)
    return out


def _level_at(edges, t):
    """时刻 t 的总线电平（空闲默认为隐性 1；边沿时刻取跳变后的电平）。"""
    level = RECESSIVE
    for e in edges:
        if e.time > t:
            break
        level = e.level
    return level


@dataclass(frozen=True)
class SampleRecord:
    """一次采样：第 bit_index 个采样位、采样时刻（tq）与采样值。"""

    bit_index: int
    time: int
    bit: int


@dataclass(frozen=True)
class PhaseCorrection:
    """一次相位调整记录。

    kind:        "hard_sync"（帧起始）| "resync"（帧内有界重同步）
    edge_time:   触发调整的隐性到显性边沿时刻
    phase_error: 边沿相对期望位边界的有符号偏差（tq）
    applied:     实际应用的调整量（|applied| <= sjw；hard_sync 为 0，
                 因为硬同步直接重对齐边界而非有界调整）
    clamped:     相位误差超出 SJW、被截断时为 True
    """

    bit_index: int
    kind: str
    edge_time: int
    phase_error: int
    applied: int
    clamped: bool


@dataclass(frozen=True)
class ReceptionResult:
    """边沿接收的完整报告。

    samples:      每次采样的时刻与值
    corrections:  每次相位调整（硬同步与重同步分开记录）
    bits:         恢复出的位流（SOF 起，含 ACK/EOF）
    status:       "ok" | "crc_error" | "stuff_error" | "length_error"
                  | "protocol_error" | "no_frame" | "incomplete"
    frame:        status=="ok" 时解码出的 Frame，否则 None
    crc_ok:       是否通过 CRC 检查（未解码成功时为 False）
    error_bit_index: 出错位置（采样位索引），无错为 None
    detail:       人可读的状态说明
    """

    samples: tuple
    corrections: tuple
    bits: tuple
    status: str
    frame: object
    crc_ok: bool
    error_bit_index: object
    detail: str


def receive_edges(edges, timing):
    """从边沿时间序列恢复位流并解码整帧。

    流程：空闲时等待首个隐性到显性边沿做硬同步（SOF 起点）；随后每位
    在采样点采样；帧内每个隐性到显性边沿触发一次有界（<=sjw）相位
    调整。恢复出的位流按 SOF + 填充流 + ACK + EOF 切分，填充流交给
    decode_frame 做去填充与 CRC 检查。
    """
    if not isinstance(timing, BitTiming):
        raise ValidationError(
            f"timing must be BitTiming, got {type(timing).__name__}", "timing"
        )
    edge_list = _normalize_edges(edges)

    samples = []
    corrections = []
    bits = []

    def finish(status, frame, crc_ok, error_bit_index, detail):
        return ReceptionResult(
            samples=tuple(samples),
            corrections=tuple(corrections),
            bits=tuple(bits),
            status=status,
            frame=frame,
            crc_ok=crc_ok,
            error_bit_index=error_bit_index,
            detail=detail,
        )

    # 找首个隐性到显性边沿（帧起始硬同步）
    start_edge = None
    prev_level = RECESSIVE
    for e in edge_list:
        if prev_level == RECESSIVE and e.level == DOMINANT:
            start_edge = e
            break
        prev_level = e.level
    if start_edge is None:
        return finish("no_frame", None, False, None,
                      "no recessive-to-dominant edge found; bus idle")

    corrections.append(
        PhaseCorrection(0, "hard_sync", start_edge.time, 0, 0, False)
    )

    tq = timing.tq_per_bit
    sp = timing.sample_point
    sjw = timing.sjw
    bit_start = start_edge.time
    last_edge_time = edge_list[-1].time
    edge_idx = 0
    recessive_run = 0
    dominant_run = 0
    last_decode_error = None

    for bit_index in range(MAX_SAMPLES):
        sample_time = bit_start + sp
        bit = _level_at(edge_list, sample_time)
        samples.append(SampleRecord(bit_index, sample_time, bit))
        bits.append(bit)
        if bit_index == 0 and bit != DOMINANT:
            return finish("protocol_error", None, False, bit_index,
                          "SOF sample is not dominant")
        recessive_run = recessive_run + 1 if bit == RECESSIVE else 0
        dominant_run = dominant_run + 1 if bit == DOMINANT else 0

        boundary = bit_start + tq

        # 终止判定：帧尾 ACK+EOF 之后尝试整帧解码
        if len(bits) >= 4 and bits[-1] == RECESSIVE and bits[-2] == RECESSIVE:
            prefix = bits[1:-2]  # 去掉 SOF 与末尾 ACK/EOF 候选位
            try:
                dec = decode_frame(prefix, raise_on_crc=False)
            except FrameDecodeError as exc:
                last_decode_error = exc
                if exc.kind == "stuff" and exc.bit_index < len(prefix):
                    # 违例发生在已收前缀内部：真实的填充错误
                    return finish("stuff_error", None, False,
                                  bit_index, str(exc))
                # stuff 违例在当前流尾（等填充位）或 length：帧未完整，继续
                if exc.kind == "protocol":
                    return finish("protocol_error", None, False,
                                  bit_index, str(exc))
                # length：帧尚未完整，继续采样
            else:
                if dec.crc_ok:
                    return finish("ok", dec.frame, True, None,
                                  "frame decoded and CRC verified")
                return finish("crc_error", None, False, bit_index,
                              f"CRC mismatch: got 0x{dec.crc:04X}")

        # 6 个连续显性采样：填充规则必然被破坏（错误帧场景）
        if dominant_run >= 6:
            return finish("stuff_error", None, False, bit_index,
                          "six consecutive dominant samples "
                          "(stuff rule violated)")

        # 无后续边沿且长时间隐性：帧已结束但未能解码（损坏帧）
        if recessive_run >= _IDLE_RUN_STOP and boundary > last_edge_time:
            detail = "frame ended without a decodable structure"
            status = "incomplete"
            if last_decode_error is not None:
                detail = f"last decode attempt failed: {last_decode_error}"
                status = f"{last_decode_error.kind}_error"
            return finish(status, None, False, bit_index, detail)

        # 帧内重同步：本位窗口内的首个隐性到显性边沿
        while edge_idx < len(edge_list) and edge_list[edge_idx].time <= bit_start:
            edge_idx += 1
        prev = _level_at(edge_list, bit_start)
        next_start = boundary
        scan = edge_idx
        while scan < len(edge_list) and edge_list[scan].time <= boundary:
            e = edge_list[scan]
            if prev == RECESSIVE and e.level == DOMINANT:
                # 采样点之前出现的边沿：上一位边界来晚（e>0，本位加长）；
                # 采样点之后出现的边沿：下一位边界来早（e<0，本位缩短）
                if e.time - bit_start <= sp:
                    err = e.time - bit_start
                else:
                    err = e.time - boundary
                applied = max(-sjw, min(sjw, err))
                corrections.append(
                    PhaseCorrection(bit_index, "resync", e.time, err,
                                    applied, applied != err)
                )
                next_start = boundary + applied
                break
            prev = e.level
            scan += 1

        bit_start = next_start

    return finish("incomplete", None, False, MAX_SAMPLES,
                  f"sample cap {MAX_SAMPLES} reached without frame end")


@dataclass(frozen=True)
class Glitch:
    """毛刺：time 时刻电平被强制为 level，持续 duration tq 后恢复。"""

    time: int
    level: int
    duration: int

    def __post_init__(self) -> None:
        check_int(self.time, "time", 0, 1 << 62)
        check_int(self.level, "level", 0, 1)
        check_int(self.duration, "duration", 1, 1 << 30)


def simulate_edges(bits, timing, *, drift_num=0, drift_den=1, glitches=()):
    """由理想位流确定性地生成边沿时间序列。

    发送时钟偏差以有理数给出：每 drift_den 位累计漂移 drift_num 个
    tq（可正可负），用整数累加器逐位摊派，无浮点。drift_num=0 即
    理想发送时钟。glitches 在边沿序列上叠加短脉冲（time 变为 level，
    duration tq 后恢复）。
    """
    if not isinstance(timing, BitTiming):
        raise ValidationError(
            f"timing must be BitTiming, got {type(timing).__name__}", "timing"
        )
    bit_list = list(bits)
    for i, b in enumerate(bit_list):
        if isinstance(b, bool) or not isinstance(b, int) or b not in (0, 1):
            raise ValidationError(
                f"bit at index {i} must be int 0/1 (bool rejected)", "bits"
            )
    if not bit_list:
        raise ValidationError("bits must be non-empty", "bits")
    drift_num = check_int(drift_num, "drift_num", -(1 << 30), 1 << 30)
    drift_den = check_int(drift_den, "drift_den", 1, 1 << 30)
    glitch_list = []
    for i, g in enumerate(glitches):
        if isinstance(g, Glitch):
            glitch_list.append(g)
        elif isinstance(g, tuple) and len(g) == 3:
            glitch_list.append(Glitch(g[0], g[1], g[2]))
        else:
            raise ValidationError(
                f"glitch #{i} must be Glitch or (time, level, duration)",
                "glitches",
            )

    tq = timing.tq_per_bit
    edges = []
    t = 0
    acc = 0
    prev = RECESSIVE  # 总线空闲为隐性
    for bit in bit_list:
        acc += drift_num
        extra = 0
        sign = 1 if acc >= 0 else -1
        while acc * sign >= drift_den:
            extra += sign
            acc -= sign * drift_den
        if bit != prev:
            edges.append(Edge(t, bit))
            prev = bit
        t += tq + extra

    # 叠加毛刺：强制电平 + 恢复沿
    for g in glitch_list:
        level_before = _level_at(edges, g.time)
        if level_before == g.level:
            continue  # 毛刺电平与当前相同，无边沿效果
        edges.append(Edge(g.time, g.level))
        edges.append(Edge(g.time + g.duration, level_before))

    edges.sort(key=lambda e: e.time)
    # 同刻边沿去重（保留最后一个电平），保证时间严格递增
    merged = []
    for e in edges:
        if merged and e.time == merged[-1].time:
            merged[-1] = e
        else:
            merged.append(e)
    # 去掉不改变电平的冗余边沿
    out = []
    level = RECESSIVE
    for e in merged:
        if e.level != level:
            out.append(e)
            level = e.level
    return out
