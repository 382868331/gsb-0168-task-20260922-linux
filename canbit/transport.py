"""基于现有 CAN 帧的小型显式传输层（本题契约子集，非完整车规标准）。

帧格式（承载于 Frame.data，DLC ≤ 8，PCI 为第 1 字节）：

* 首帧 FF  0x1：byte0 = 0x10 | (total >> 8)，byte1 = total & 0xFF，
  byte2..7 = 前 6 个数据字节；total 为消息总字节数，范围 1..4095（12 位）。
* 连续帧 CF 0x2：byte0 = 0x20 | (seq & 0x0F)，byte1..7 = 至多 7 个
  数据字节；seq 从 1 起按模 16 递增（…, 14, 15, 0, 1, …）。
* 流控帧 FC 0x3：byte0 = 0x30 | fs，byte1 = block_size（0 = 不限），
  byte2 = st_min（相邻 CF 的最小发送间隔，单位：逻辑时钟 tick）。
  fs：0=CTS 继续，1=WAIT 暂停，2=OVFLW 中止。

语义（本层自定义，不宣称兼容 ISO-TP 等完整标准）：

* 接收端只在全部字节到齐时一次性交付；任何失败整次作废，绝不交付部分消息。
* 序号跳跃、超长首帧（total > max_message）、会话内格式变化
  （接收中途又来首帧 / 空闲时收到 CF 或 FC）都使整次接收失败并清空缓存。
* 流控 WAIT 最多连续接受 max_waits 次，超过即发送方中止。
* 逻辑时钟超时（timeout tick 内没有下一帧）后接收端清理缓存。
* 时间完全由调用方以整数 tick 驱动，本模块不读真实时钟、不 sleep。
"""

from __future__ import annotations

from dataclasses import dataclass

from .errors import CanError, ValidationError
from .framing import Frame, check_int

# ---- 协议常量 ------------------------------------------------------------

PCI_FF = 0x10
PCI_CF = 0x20
PCI_FC = 0x30
PCI_MASK = 0xF0

FS_CTS = 0
FS_WAIT = 1
FS_OVFLW = 2

FF_DATA_BYTES = 6
CF_DATA_BYTES = 7
SEQ_MOD = 16
MAX_MESSAGE_BYTES = 0xFFF  # total 为 12 位字段

DEFAULT_TIMEOUT = 100
DEFAULT_MAX_WAITS = 3


class TransportError(CanError):
    """传输层失败。kind 为机器可读类别：

    "sequence"      序号跳跃（含丢帧导致的序号不连续）
    "oversize"      首帧声明的总字节数超过接收端上限
    "format"        会话内格式变化（中途新首帧 / 空闲时收到 CF、FC / PCI 非法）
    "timeout"       逻辑时钟超时，接收缓存已清理
    "wait_exceeded" 流控 WAIT 次数超过 max_waits，发送方中止
    "abort"         收到 OVFLW，对端要求中止
    """

    def __init__(self, kind: str, detail: str = "") -> None:
        self.kind = kind
        text = f"transport {kind} error"
        if detail:
            text += f": {detail}"
        super().__init__(text)


def _check_bytes(data: object, field_name: str = "data") -> bytes:
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    raise ValidationError(
        f"expected bytes, got {type(data).__name__}", field_name
    )


def make_first_frame(identifier: int, total: int, chunk: bytes) -> Frame:
    """构造首帧（纯函数，便于测试/对照）。"""
    check_int(total, "total", 1, MAX_MESSAGE_BYTES)
    chunk = _check_bytes(chunk, "chunk")
    if len(chunk) > FF_DATA_BYTES:
        raise ValidationError("first frame carries at most 6 data bytes", "chunk")
    data = bytes([PCI_FF | ((total >> 8) & 0x0F), total & 0xFF]) + chunk
    return Frame(identifier, data)


def make_consecutive_frame(identifier: int, seq: int, chunk: bytes) -> Frame:
    """构造连续帧（纯函数）。seq 取模 16。"""
    check_int(seq, "seq", 0, SEQ_MOD - 1)
    chunk = _check_bytes(chunk, "chunk")
    if len(chunk) > CF_DATA_BYTES:
        raise ValidationError(
            "consecutive frame carries at most 7 data bytes", "chunk"
        )
    return Frame(identifier, bytes([PCI_CF | (seq & 0x0F)]) + chunk)


def make_flow_control_frame(
    identifier: int, flow_status: int, block_size: int, st_min: int
) -> Frame:
    """构造流控帧（纯函数）。"""
    check_int(flow_status, "flow_status", 0, 2)
    check_int(block_size, "block_size", 0, 0xFF)
    check_int(st_min, "st_min", 0, 0xFF)
    return Frame(identifier, bytes([PCI_FC | flow_status, block_size, st_min]))


def parse_pci(frame: Frame) -> tuple[int, int]:
    """返回 (帧类型 PCI 高 4 位, 低 4 位参数)。非法输入抛 ValidationError。"""
    if not isinstance(frame, Frame):
        raise ValidationError(
            f"frame must be Frame, got {type(frame).__name__}", "frame"
        )
    if not frame.data:
        raise ValidationError("transport frame needs at least a PCI byte", "frame")
    return frame.data[0] & PCI_MASK, frame.data[0] & 0x0F


# ---- 发送端 ---------------------------------------------------------------


class Sender:
    """分段发送端：由调用方以逻辑 tick 与 FC 帧驱动。

    用法：ff = sender.start(tick) 发出首帧；收到流控帧时
    sender.on_frame(frame, tick)；随后反复 sender.next_frame(tick)
    取下一帧（受块大小与 st_min 约束，未就绪返回 None）。
    """

    def __init__(
        self,
        identifier: int,
        data: bytes,
        *,
        max_waits: int = DEFAULT_MAX_WAITS,
    ) -> None:
        check_int(identifier, "identifier", 0, 0x7FF)
        check_int(max_waits, "max_waits", 0, 0xFF)
        data = _check_bytes(data)
        if not (1 <= len(data) <= MAX_MESSAGE_BYTES):
            raise ValidationError(
                f"data length must be in [1, {MAX_MESSAGE_BYTES}]", "data"
            )
        self.identifier = identifier
        self.data = data
        self.max_waits = max_waits

        self._offset = 0
        self._seq = 0  # 下一个 CF 的序号（首帧后为 1）
        self._state = "idle"  # idle|wait_fc|sending|done
        self._block_remaining = 0
        self._st_min = 0
        self._waits = 0
        self._last_send_tick: int | None = None

    @property
    def done(self) -> bool:
        return self._state == "done"

    @property
    def state(self) -> str:
        return self._state

    def start(self, tick: int) -> Frame:
        """发出首帧，进入等待流控状态。"""
        check_int(tick, "tick", 0, 1 << 62)
        if self._state != "idle":
            raise ValidationError("start() only valid once from idle", "state")
        chunk = self.data[:FF_DATA_BYTES]
        self._offset = len(chunk)
        self._seq = 1
        self._last_send_tick = tick
        if self._offset >= len(self.data):
            self._state = "done"  # 6 字节以内一帧装完
        else:
            self._state = "wait_fc"
        return make_first_frame(self.identifier, len(self.data), chunk)

    def on_frame(self, frame: Frame, tick: int) -> None:
        """处理对端流控帧。WAIT 超限或对端 OVFLW 时抛 TransportError。"""
        check_int(tick, "tick", 0, 1 << 62)
        pci, param = parse_pci(frame)
        if pci != PCI_FC:
            raise TransportError("format", "sender expects flow control frames")
        if self._state == "done":
            raise TransportError("format", "flow control after completion")
        if len(frame.data) < 3:
            raise TransportError("format", "flow control frame too short")
        block_size = frame.data[1]
        st_min = frame.data[2]
        if param == FS_CTS:
            self._waits = 0
            self._st_min = st_min
            self._block_remaining = block_size if block_size else 1 << 30
            self._state = "sending"
        elif param == FS_WAIT:
            self._waits += 1
            if self._waits > self.max_waits:
                self._state = "idle"
                raise TransportError(
                    "wait_exceeded",
                    f"more than {self.max_waits} consecutive WAIT frames",
                )
            self._state = "wait_fc"
        elif param == FS_OVFLW:
            self._state = "idle"
            raise TransportError("abort", "receiver overflow")
        else:  # 低 4 位只定义了 0..2
            raise TransportError("format", f"unknown flow status {param}")

    def next_frame(self, tick: int) -> Frame | None:
        """若允许发送（块未用完且满足 st_min），返回下一连续帧，否则 None。"""
        check_int(tick, "tick", 0, 1 << 62)
        if self._state != "sending":
            return None
        if self._block_remaining <= 0:
            self._state = "wait_fc"
            return None
        if (
            self._last_send_tick is not None
            and tick - self._last_send_tick < self._st_min
        ):
            return None
        chunk = self.data[self._offset : self._offset + CF_DATA_BYTES]
        frame = make_consecutive_frame(self.identifier, self._seq, chunk)
        self._seq = (self._seq + 1) % SEQ_MOD
        self._offset += len(chunk)
        self._block_remaining -= 1
        self._last_send_tick = tick
        if self._offset >= len(self.data):
            self._state = "done"
        return frame


# ---- 接收端 ---------------------------------------------------------------


@dataclass(frozen=True)
class ReceiveResult:
    """on_frame 的返回：要回发的流控帧 + 本帧后完成交付的字节（若有）。"""

    flow_frames: tuple[Frame, ...]
    delivered: bytes | None  # 仅当整次消息到齐时非 None


class Receiver:
    """分段接收端：整次到齐才交付，任何失败整次作废并清缓存。"""

    def __init__(
        self,
        identifier: int,
        *,
        block_size: int = 0,
        st_min: int = 0,
        max_message: int = MAX_MESSAGE_BYTES,
        timeout: int = DEFAULT_TIMEOUT,
    ) -> None:
        check_int(identifier, "identifier", 0, 0x7FF)
        check_int(block_size, "block_size", 0, 0xFF)
        check_int(st_min, "st_min", 0, 0xFF)
        check_int(max_message, "max_message", 1, MAX_MESSAGE_BYTES)
        check_int(timeout, "timeout", 1, 1 << 62)
        self.identifier = identifier
        self.block_size = block_size
        self.st_min = st_min
        self.max_message = max_message
        self.timeout = timeout
        self._reset()

    def _reset(self) -> None:
        self._buf = bytearray()
        self._total = 0
        self._expected_seq = 0
        self._since_fc = 0
        self._receiving = False
        self._last_tick: int | None = None

    @property
    def receiving(self) -> bool:
        return self._receiving

    @property
    def buffered_bytes(self) -> int:
        """当前缓存的已收字节数（超时/失败后应为 0）。"""
        return len(self._buf)

    def _fail(self, kind: str, detail: str) -> TransportError:
        self._reset()
        return TransportError(kind, detail)

    def on_frame(self, frame: Frame, tick: int) -> ReceiveResult:
        """处理一个到达帧；失败时抛 TransportError 且缓存已清理。"""
        check_int(tick, "tick", 0, 1 << 62)
        pci, param = parse_pci(frame)
        if pci == PCI_FF:
            return self._on_first(frame, param, tick)
        if pci == PCI_CF:
            return self._on_consecutive(frame, param, tick)
        # FC 是发给发送端的；接收端收到属格式错误
        raise self._fail("format", "receiver got unexpected flow control frame")

    def _on_first(self, frame: Frame, total_lo: int, tick: int) -> ReceiveResult:
        if self._receiving:
            raise self._fail(
                "format", "new first frame while a message is being received"
            )
        if len(frame.data) < 2:
            raise self._fail("format", "first frame too short for total length")
        total = (total_lo << 8) | frame.data[1]
        if total < 1 or total > self.max_message:
            raise self._fail(
                "oversize",
                f"declared total {total} exceeds limit {self.max_message}",
            )
        chunk = bytes(frame.data[2 : 2 + min(FF_DATA_BYTES, total)])
        self._buf = bytearray(chunk)
        self._total = total
        self._expected_seq = 1
        self._since_fc = 0
        self._last_tick = tick
        if len(self._buf) >= total:
            delivered = bytes(self._buf)
            self._reset()
            return ReceiveResult((), delivered)
        self._receiving = True
        fc = make_flow_control_frame(
            self.identifier, FS_CTS, self.block_size, self.st_min
        )
        return ReceiveResult((fc,), None)

    def _on_consecutive(self, frame: Frame, seq: int, tick: int) -> ReceiveResult:
        if not self._receiving:
            raise self._fail("format", "consecutive frame with no active session")
        if seq != self._expected_seq:
            raise self._fail(
                "sequence", f"expected seq {self._expected_seq}, got {seq}"
            )
        need = self._total - len(self._buf)
        self._buf += bytes(frame.data[1 : 1 + min(CF_DATA_BYTES, need)])
        self._expected_seq = (self._expected_seq + 1) % SEQ_MOD
        self._since_fc += 1
        self._last_tick = tick
        if len(self._buf) >= self._total:
            delivered = bytes(self._buf)
            self._reset()
            return ReceiveResult((), delivered)
        flow: tuple[Frame, ...] = ()
        if self.block_size and self._since_fc >= self.block_size:
            self._since_fc = 0
            flow = (
                make_flow_control_frame(
                    self.identifier, FS_CTS, self.block_size, self.st_min
                ),
            )
        return ReceiveResult(flow, None)

    def on_tick(self, tick: int) -> None:
        """推进逻辑时钟；接收中超时则清理缓存并抛 TransportError。"""
        check_int(tick, "tick", 0, 1 << 62)
        if (
            self._receiving
            and self._last_tick is not None
            and tick - self._last_tick > self.timeout
        ):
            raise self._fail(
                "timeout",
                f"no frame for {tick - self._last_tick} ticks"
                f" (limit {self.timeout})",
            )

