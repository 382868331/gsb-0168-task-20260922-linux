"""CAN 合成位流的仲裁与错误约束 —— 有限总线模型（非完整 CAN 标准）。

帧结构（本模型自定义的有限子集）：
    ID(11 位, 高位先发) + DLC(4 位, 0..8) + 数据(0..8 字节) + CRC(15 位)
  - CRC：上述 ID+DLC+数据位串左移 15 位后，对生成多项式 0x4599
    （隐含最高位，即 0xC599，15 次）取余，初值 0，余数高位先发。
  - 位填充：从 ID 到 CRC，每连续 5 个相同位插入 1 个反位。
  - ACK：固定 1 个独立槽，不参与填充；发送者释放（隐性 1），
    接收者由显式参数 ack_pull 决定是否拉低（显性 0）。

总线规则：
  - 显性 0 胜隐性 1（线与）。
  - ID 域内：发送 1 读到 0 的发送者只退出仲裁，不算错误。
  - 其余位置：发送 1 读到 0 的发送者报 bit_error，中止当前传输。
  - 接收侧去填充后发现 6 连同位 -> stuff_error；CRC 校验失败 -> crc_error。
  - ACK 槽无人拉低 -> ack_error。
  - 任何错误都会中止当前传输，并在结果中记录参与者。

本模块不宣称兼容完整 CAN 标准（无 SOF/RTR/IDE/EOF/错误帧/重传等）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

__all__ = [
    "CRC_POLY",
    "MAX_ID",
    "MAX_DATA_LEN",
    "MAX_TRANSMITTERS",
    "MAX_SLOTS",
    "Frame",
    "BitEvent",
    "BusError",
    "SimResult",
    "CanBusError",
    "StuffError",
    "CrcError",
    "crc15",
    "encode_unstuffed",
    "stuff_bits",
    "destuff_bits",
    "encode_frame",
    "decode_stream",
    "BusSimulation",
    "simulate",
]

# 生成多项式 0x4599 隐含 x^15 项，补全为 0xC599（15 次多项式）。
CRC_POLY = 0xC599
MAX_ID = 0x7FF          # 11 位标识符上限
MAX_DATA_LEN = 8        # 数据长度上限（字节）
MAX_TRANSMITTERS = 8    # 单次仿真的发送者上限
# 未填充最长 11+4+64+15 = 94 位；填充最多 floor(94/5)=18 位；ACK 1 槽。
MAX_SLOTS = 94 + 18 + 1

_DOMINANT = 0
_RECESSIVE = 1


# ---------------------------------------------------------------------------
# 输入校验：非法输入给出可定位错误，不静默纠正。
# ---------------------------------------------------------------------------

def _require_int(name: str, value, lo: int, hi: int) -> int:
    """整数参数校验：拒绝 bool、拒绝非 int（含 NaN/Infinity 等浮点）。"""
    if isinstance(value, bool):
        raise TypeError(f"{name} 必须是 int，拒绝 bool：{value!r}")
    if not isinstance(value, int):
        raise TypeError(f"{name} 必须是 int，得到 {type(value).__name__}：{value!r}")
    if not (lo <= value <= hi):
        raise ValueError(f"{name} 超出范围 [{lo}, {hi}]：{value}")
    return value


def _require_bits(name: str, bits) -> Tuple[int, ...]:
    """位串校验：元素必须为 int 0/1，拒绝 bool 与容器字符串。"""
    if isinstance(bits, (str, bytes, bytearray)):
        raise TypeError(f"{name} 必须是 0/1 整数序列，拒绝 {type(bits).__name__}")
    try:
        items = tuple(bits)
    except TypeError:
        raise TypeError(f"{name} 必须是可迭代的 0/1 整数序列") from None
    for i, b in enumerate(items):
        if isinstance(b, bool):
            raise TypeError(f"{name}[{i}] 必须是 int，拒绝 bool：{b!r}")
        if not isinstance(b, int):
            raise TypeError(
                f"{name}[{i}] 必须是 int，得到 {type(b).__name__}：{b!r}")
        if b not in (0, 1):
            raise ValueError(f"{name}[{i}] 只能是 0 或 1：{b}")
    return items


def _int_bits(value: int, width: int) -> Tuple[int, ...]:
    """无符号整数转固定位宽，高位先发。"""
    return tuple((value >> (width - 1 - i)) & 1 for i in range(width))


def _bits_to_int(bits: Sequence[int]) -> int:
    value = 0
    for b in bits:
        value = (value << 1) | b
    return value


# ---------------------------------------------------------------------------
# 帧与错误的数据结构
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Frame:
    """一帧：11 位 ID + 0..8 字节数据。DLC 由数据长度推出。"""
    id: int
    data: bytes = b""

    def __post_init__(self):
        _require_int("Frame.id", self.id, 0, MAX_ID)
        if isinstance(self.data, (bytes, bytearray)):
            object.__setattr__(self, "data", bytes(self.data))
        else:
            raise TypeError(
                f"Frame.data 必须是 bytes，得到 {type(self.data).__name__}")
        if len(self.data) > MAX_DATA_LEN:
            raise ValueError(
                f"Frame.data 长度上限 {MAX_DATA_LEN} 字节，得到 {len(self.data)}")

    @property
    def dlc(self) -> int:
        return len(self.data)


class CanBusError(Exception):
    """位流解码/协议错误基类。"""


class StuffError(CanBusError):
    """位填充违规：ID..CRC 区域出现 6 个连续同位，或结尾缺少填充位。"""


class CrcError(CanBusError):
    """去填充后 CRC 校验失败。"""


@dataclass(frozen=True)
class BitEvent:
    """逐 bit step 的一个槽位记录。

    slot:       槽位序号（0 起，按发送的填充后位流计）。
    field:      "id" / "dlc" / "data" / "crc" / "stuff" / "ack"。
    bus_value:  该槽总线电平（线与结果）。
    sent:       每个发送者该槽发送的位；已退出仲裁者为 None。
    active:     该槽开始时仍在发送的发送者下标。
    """
    slot: int
    field: str
    bus_value: int
    sent: Tuple[Optional[int], ...]
    active: Tuple[int, ...]


@dataclass(frozen=True)
class BusError:
    """仿真中记录到的协议错误（中止当前传输）。

    kind:         "bit_error" / "stuff_error" / "crc_error" / "ack_error"。
    slot:         检测到的槽位（解码类错误为 CRC 域结束后的位置）。
    field:        出错所在域。
    participants: 错误发生时仍参与传输的发送者下标。
    detail:       人类可读的补充说明。
    """
    kind: str
    slot: int
    field: str
    participants: Tuple[int, ...]
    detail: str


@dataclass(frozen=True)
class SimResult:
    """一次传输仿真的最终结果。

    ok:       无协议错误且 ACK 被拉低。
    winners:  完成发送的发送者下标（相同帧可多个并列）。
    frame:    接收侧解码出的帧；失败时为 None。
    error:    协议错误记录；成功时为 None。
    events:   全部槽位记录（含出错的最后一槽）。
    ack:      ACK 槽总线电平是否为显性；未走到 ACK 槽为 None。
    """
    ok: bool
    winners: Tuple[int, ...]
    frame: Optional[Frame]
    error: Optional[BusError]
    events: Tuple[BitEvent, ...]
    ack: Optional[bool]


# ---------------------------------------------------------------------------
# CRC-15：位串左移 15 位后对 0xC599 取余，初值 0。
# ---------------------------------------------------------------------------

def crc15(bits) -> int:
    """计算 15 位 CRC。bits 为 0/1 整数序列（高位先发的位串）。"""
    data = _require_bits("bits", bits)
    value = _bits_to_int(data) << 15
    top = value.bit_length() - 1
    for i in range(top, 14, -1):
        if (value >> i) & 1:
            value ^= CRC_POLY << (i - 15)
    return value & 0x7FFF


# ---------------------------------------------------------------------------
# 编码：未填充位流 -> 位填充 -> （发送端）附 ACK 释放位
# ---------------------------------------------------------------------------

def encode_unstuffed(frame: Frame) -> Tuple[int, ...]:
    """ID + DLC + 数据 + CRC 的未填充位串（高位先发）。"""
    if not isinstance(frame, Frame):
        raise TypeError(f"frame 必须是 Frame，得到 {type(frame).__name__}")
    bits = list(_int_bits(frame.id, 11))
    bits += _int_bits(frame.dlc, 4)
    for byte in frame.data:
        bits += _int_bits(byte, 8)
    bits += _int_bits(crc15(bits), 15)
    return tuple(bits)


def stuff_bits(bits) -> Tuple[int, ...]:
    """位填充：每连续 5 个相同位插入 1 个反位。"""
    data = _require_bits("bits", bits)
    out = []
    run = 0
    prev = None
    for b in data:
        out.append(b)
        if prev is not None and b == prev:
            run += 1
        else:
            run = 1
            prev = b
        if run == 5:
            out.append(1 - b)
            run = 0
            prev = None
    return tuple(out)


def destuff_bits(bits) -> Tuple[int, ...]:
    """去填充：移除每 5 连同位后的反位；发现 6 连同位报 StuffError。"""
    data = _require_bits("bits", bits)
    out = []
    run = 0
    prev = None
    for i, b in enumerate(data):
        if run == 5:
            if b == prev:
                raise StuffError(
                    f"位 {i} 处出现 6 个连续 {prev}，违反填充规则")
            run = 0
            prev = None
            continue  # 跳过填充位
        out.append(b)
        if prev is not None and b == prev:
            run += 1
        else:
            run = 1
            prev = b
    if run == 5:
        raise StuffError("位流在应有填充位处被截断（结尾 5 连同位）")
    return tuple(out)


def _annotated_stream(frame: Frame):
    """填充后的发送位流，每位标注 (bit, field, in_arbitration)。

    in_arbitration：该槽是否仍处 ID 仲裁域（含 ID 域内插入的填充位）。
    """
    raw = [(b, "id") for b in _int_bits(frame.id, 11)]
    raw += [(b, "dlc") for b in _int_bits(frame.dlc, 4)]
    for byte in frame.data:
        raw += [(b, "data") for b in _int_bits(byte, 8)]
    body = [b for b, _ in raw]
    raw += [(b, "crc") for b in _int_bits(crc15(body), 15)]

    stuffed = []
    run = 0
    prev = None
    for b, field in raw:
        stuffed.append((b, field))
        if prev is not None and b == prev:
            run += 1
        else:
            run = 1
            prev = b
        if run == 5:
            stuffed.append((1 - b, "stuff"))
            run = 0
            prev = None

    out = []
    id_count = 0
    for b, field in stuffed:
        out.append((b, field, id_count < 11))
        if field == "id":
            id_count += 1
    return out


def encode_frame(frame: Frame) -> Tuple[int, ...]:
    """发送端完整驱动位流：填充后的 ID..CRC + 1 位 ACK 释放（隐性 1）。"""
    return tuple(b for b, _, _ in _annotated_stream(frame)) + (_RECESSIVE,)


def decode_stream(bits) -> Frame:
    """接收侧解码：去填充 -> 解析 ID/DLC/数据/CRC -> CRC 校验。

    输入为总线上 ID..CRC 区域的填充后位串（不含 ACK 槽）。
    填充违规抛 StuffError，CRC 不符抛 CrcError，
    长度/DLC 不合法抛 CanBusError。
    """
    data = _require_bits("bits", bits)
    raw = destuff_bits(data)
    if len(raw) < 11 + 4 + 15:
        raise CanBusError(
            f"去填充后仅 {len(raw)} 位，不足最小帧长 {11 + 4 + 15} 位")
    can_id = _bits_to_int(raw[0:11])
    dlc = _bits_to_int(raw[11:15])
    if dlc > MAX_DATA_LEN:
        raise CanBusError(f"DLC={dlc} 超出上限 {MAX_DATA_LEN}")
    need = 11 + 4 + 8 * dlc + 15
    if len(raw) != need:
        raise CanBusError(
            f"去填充后 {len(raw)} 位，与 DLC={dlc} 要求的 {need} 位不符")
    payload = bytes(
        _bits_to_int(raw[15 + 8 * k: 15 + 8 * k + 8]) for k in range(dlc))
    crc_rx = _bits_to_int(raw[15 + 8 * dlc: need])
    crc_calc = crc15(raw[: 15 + 8 * dlc])
    if crc_rx != crc_calc:
        raise CrcError(
            f"CRC 校验失败：收到 0x{crc_rx:04X}，应为 0x{crc_calc:04X}")
    return Frame(id=can_id, data=payload)


# ---------------------------------------------------------------------------
# 总线仿真：逐 bit step
# ---------------------------------------------------------------------------

class BusSimulation:
    """多发送者同时发一帧的有限总线仿真。

    构造时完成全部输入校验；校验失败不创建任何状态。
    step() 每次推进一个槽位；出错或正常结束后 done 为 True，
    之后再调用 step() 抛 RuntimeError。出错时传输即中止，
    结果（含参与者）写入 result，不留未说明的中间状态。
    """

    def __init__(self, frames, ack_pull: bool = True):
        if not isinstance(ack_pull, bool):
            raise TypeError(
                f"ack_pull 必须是 bool，得到 {type(ack_pull).__name__}")
        if isinstance(frames, (str, bytes, bytearray)):
            raise TypeError("frames 必须是 Frame 序列")
        try:
            items = tuple(frames)
        except TypeError:
            raise TypeError("frames 必须是可迭代的 Frame 序列") from None
        if not (1 <= len(items) <= MAX_TRANSMITTERS):
            raise ValueError(
                f"发送者数量须在 1..{MAX_TRANSMITTERS}，得到 {len(items)}")
        for i, f in enumerate(items):
            if not isinstance(f, Frame):
                raise TypeError(
                    f"frames[{i}] 必须是 Frame，得到 {type(f).__name__}")

        self._n = len(items)
        self._streams = [_annotated_stream(f) for f in items]
        self._ack_pull = ack_pull
        self._active = list(range(self._n))
        self._slot = 0
        self._events = []
        self._received = []
        self._decoded: Optional[Frame] = None
        self._done = False
        self._result: Optional[SimResult] = None

    @property
    def done(self) -> bool:
        return self._done

    @property
    def result(self) -> Optional[SimResult]:
        """结束后为 SimResult，否则为 None。"""
        return self._result

    def step(self) -> BitEvent:
        """推进一个槽位并返回该槽记录。"""
        if self._done:
            raise RuntimeError("仿真已结束，不能继续 step")
        if self._slot > MAX_SLOTS:
            raise RuntimeError("内部错误：槽位超出模型上限")
        stream = self._streams[self._active[0]]
        if self._slot == len(stream):
            return self._ack_slot()
        return self._data_slot(stream)

    # -- 内部 ------------------------------------------------------------

    def _data_slot(self, stream) -> BitEvent:
        slot = self._slot
        sent = []
        for i in range(self._n):
            if i in self._active:
                s = self._streams[i]
                if slot >= len(s):  # 理论上不会发生：分叉即退出/报错
                    raise RuntimeError("内部错误：发送者位流失去同步")
                sent.append(s[slot][0])
            else:
                sent.append(None)
        bus = _DOMINANT if any(b == _DOMINANT for b in sent
                               if b is not None) else _RECESSIVE

        still, detectors = [], []
        for i in self._active:
            b, _field, in_arb = self._streams[i][slot]
            if b == _RECESSIVE and bus == _DOMINANT:
                if in_arb:
                    continue  # ID 域：只退出仲裁
                detectors.append(i)
            still.append(i)

        field = stream[slot][1]
        event = BitEvent(slot=slot, field=field, bus_value=bus,
                         sent=tuple(sent), active=tuple(self._active))
        self._events.append(event)
        self._received.append(bus)
        self._slot += 1

        if detectors:
            self._finish_error(BusError(
                kind="bit_error", slot=slot, field=field,
                participants=tuple(self._active),
                detail=(f"发送者 {tuple(detectors)} 在 {field} 域发送隐性 1 "
                        f"却读到显性 0")))
            return event

        self._active = still
        if self._slot == len(stream):
            # CRC 域结束：接收侧对 ID..CRC 位流去填充并校验 CRC。
            self._check_received()
        return event

    def _check_received(self) -> None:
        try:
            self._decoded = decode_stream(self._received)
        except StuffError as exc:
            self._finish_error(BusError(
                kind="stuff_error", slot=self._slot, field="frame",
                participants=tuple(self._active), detail=str(exc)))
        except CrcError as exc:
            self._finish_error(BusError(
                kind="crc_error", slot=self._slot, field="crc",
                participants=tuple(self._active), detail=str(exc)))
        except CanBusError as exc:
            self._finish_error(BusError(
                kind="decode_error", slot=self._slot, field="frame",
                participants=tuple(self._active), detail=str(exc)))

    def _ack_slot(self) -> BitEvent:
        slot = self._slot
        sent = tuple(_RECESSIVE if i in self._active else None
                     for i in range(self._n))
        bus = _DOMINANT if self._ack_pull else _RECESSIVE
        event = BitEvent(slot=slot, field="ack", bus_value=bus,
                         sent=sent, active=tuple(self._active))
        self._events.append(event)
        self._slot += 1

        if bus == _RECESSIVE:
            self._finish_error(BusError(
                kind="ack_error", slot=slot, field="ack",
                participants=tuple(self._active),
                detail="ACK 槽无任何接收者拉低"))
        else:
            self._result = SimResult(
                ok=True, winners=tuple(self._active), frame=self._decoded,
                error=None, events=tuple(self._events), ack=True)
            self._done = True
        return event

    def _finish_error(self, error: BusError) -> None:
        self._result = SimResult(
            ok=False, winners=(), frame=None, error=error,
            events=tuple(self._events), ack=None)
        self._done = True


def simulate(frames, ack_pull: bool = True) -> SimResult:
    """一次性运行完整仿真，返回 SimResult。"""
    sim = BusSimulation(frames, ack_pull=ack_pull)
    while not sim.done:
        sim.step()
    return sim.result
