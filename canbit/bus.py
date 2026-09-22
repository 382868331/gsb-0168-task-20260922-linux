"""逐 bit 的合成总线模型：wired-AND 仲裁、位错误、ACK、错误帧。

规则（本题契约，不是完整 CAN 标准）：

* 显性 0 胜隐性 1：总线位 = 所有当前驱动者贡献的按位与。
* 多个发送者：在 **ID 域的原始位**上“发 1 读 0”仅退出仲裁
  （自该位起不再驱动）；在其余任何位置（SOF/DLC/数据/CRC/填充位/EOF）
  出现发送位与读回位失配即记该参与者的 bit_error。
* 位错误被检测的那个位时间即中止当前传输，检测者随后发送 6 位显性
  错误标志；ErrorFrameInfo 记录触发位置与参与者。
* ACK 是独立槽位：所有发送者释放（发 1）；总线是否被拉低完全由
  显式参数 ``ack_pullers``（接收者参与者名集合）决定；无人拉低则
  仍在发送的参与者报 ack_error 并同样发错误帧。
* 位监控在两个方向上都成立（发 0 读 1 只能由 forced_bits 外力注入，
  物理 wired-AND 上不会自然发生；不静默忽略）。
* 存活发送者的已发位历史必然相同（否则已在该位分出胜负/报错），
  因此它们的填充位置天然对齐，逐位按各自 stream[i] 比较即可。

本模型不实现重传、错误界定/被动错误计数、错误界定符等完整 CAN 机制。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .errors import BusFinishedError, ValidationError
from .framing import (
    EOF,
    EncodedFrame,
    Frame,
    decode_frame,
    encode_frame,
)

ERROR_FLAG_BITS = 6


@dataclass(frozen=True)
class Sender:
    """发送参与者：名字 + 待发帧。"""

    name: str
    frame: Frame

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name:
            raise ValidationError("sender name must be a non-empty str", "name")
        if not isinstance(self.frame, Frame):
            raise ValidationError(
                f"frame must be Frame, got {type(self.frame).__name__}", "frame"
            )


@dataclass(frozen=True)
class Event:
    """总线上每一位时间的可审计记录。

    kind: bit | arbitration_loss | bit_error | ack | ack_error | eof
          | complete | error_flag
    """

    bit_index: int
    kind: str
    bus_bit: int
    participants: tuple[str, ...]
    detail: str = ""


@dataclass(frozen=True)
class StepResult:
    """单个位时间的结果。

    phase: transmit | ack | eof | error | done
    """

    bit_index: int
    phase: str
    bus_bit: int
    transmitted: tuple[str, ...]
    arbitration_losers: tuple[str, ...] = ()
    bit_errors: tuple[str, ...] = ()


@dataclass(frozen=True)
class ErrorFrameInfo:
    """一次错误帧中止事件。

    trigger:      "bit_error" | "ack_error"
    bit_index:    检测到错误、开始发错误标志的位索引（0 基）
    participants: 检测到错误并发出错误标志的参与者
    aborted:      因此中止当前发送的全部仍活跃发送者
    """

    trigger: str
    bit_index: int
    participants: tuple[str, ...]
    aborted: tuple[str, ...]


@dataclass(frozen=True)
class BusResult:
    """一次完整传输的结果快照。"""

    winners: tuple[str, ...]
    completed: bool
    bus_bits: tuple[int, ...]
    events: tuple[Event, ...]
    error_frame: ErrorFrameInfo | None
    decoded_frame: Frame | None
    decoded_crc_ok: bool | None
    acked: bool


class BitBus:
    """逐 bit 推进的有线与总线。

    参数：
        senders:      本轮同时开始发送的参与者（至少 1 个，名字唯一）
        ack_pullers:  显式接收者：在 ACK 槽拉低总线的参与者名；可为
                      任意名字（含仲裁失败者或纯接收者），空集合表示无人 ACK
        forced_bits:  位索引 -> 线上被外力强制的电平（0/1），与所有驱动者
                      线与。用于确定性地注入故障（如破坏填充位）；
                      例如在某发送 1 的位置强制 0 -> 该发送者发 1 读 0
    """

    def __init__(
        self,
        senders: Iterable[Sender],
        *,
        ack_pullers: Iterable[str] = (),
        forced_bits: dict[int, int] | None = None,
    ) -> None:
        senders = list(senders)
        if not senders:
            raise ValidationError("at least one sender required", "senders")
        names = [s.name for s in senders]
        if len(set(names)) != len(names):
            raise ValidationError(f"sender names must be unique: {names}", "senders")

        pullers: set[str] = set()
        for n in ack_pullers:
            if not isinstance(n, str) or not n:
                raise ValidationError(
                    "ack_pullers must be non-empty participant name strings",
                    "ack_pullers",
                )
            pullers.add(n)

        forced: dict[int, int] = {}
        if forced_bits is not None:
            if not isinstance(forced_bits, dict):
                raise ValidationError(
                    "forced_bits must be dict[int, int]", "forced_bits"
                )
            for idx, val in forced_bits.items():
                if isinstance(idx, bool) or not isinstance(idx, int):
                    raise ValidationError(
                        "forced_bits keys must be int (bool rejected)", "forced_bits"
                    )
                if idx < 0:
                    raise ValidationError(
                        f"forced_bits index {idx} must be >= 0", "forced_bits"
                    )
                if isinstance(val, bool) or not isinstance(val, int) or val not in (0, 1):
                    raise ValidationError(
                        f"forced_bits[{idx}] must be int 0/1 (bool rejected)",
                        "forced_bits",
                    )
                forced[idx] = val

        self._senders = senders
        self._ack_pullers = pullers
        self._forced = forced
        self._enc: dict[str, EncodedFrame] = {
            s.name: encode_frame(s.frame) for s in senders
        }

        self._active: set[str] = set(names)
        self._pos = 0
        self._phase: str = "transmit"  # transmit|ack|eof|error|done
        self._error_to_go = 0
        self._error_info: ErrorFrameInfo | None = None
        self._bus_bits: list[int] = []
        self._events: list[Event] = []
        self._acked = False
        self._finished = False

    # ---- 状态访问 -------------------------------------------------------

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def active_senders(self) -> tuple[str, ...]:
        return tuple(self._active_ordered())

    @property
    def bit_index(self) -> int:
        return self._pos

    @property
    def events(self) -> tuple[Event, ...]:
        return tuple(self._events)

    @property
    def bus_bits(self) -> tuple[int, ...]:
        return tuple(self._bus_bits)

    @property
    def error_frame(self) -> ErrorFrameInfo | None:
        return self._error_info

    def encoded_stream(self, sender_name: str) -> EncodedFrame:
        """查看某参与者的编码产物（含填充位映射），便于定位演示。"""
        if sender_name not in self._enc:
            raise ValidationError(f"unknown sender {sender_name!r}", "sender_name")
        return self._enc[sender_name]

    # ---- 逐 bit 推进 -----------------------------------------------------

    def step(self) -> StepResult:
        """推进一个位时间。传输结束后再调用抛 BusFinishedError。"""
        if self._finished:
            raise BusFinishedError("transmission already finished")
        if self._phase == "error":
            return self._step_error_flag()
        return self._step_normal()

    def _step_normal(self) -> StepResult:
        i = self._pos
        losers: list[str] = []
        bit_error_nodes: list[str] = []
        contributions: dict[str, int] = {}
        slot = "data"  # data | ack | eof

        for name in self._active_ordered():
            enc = self._enc[name]
            if i == enc.ack_index:
                slot = "ack"
                contributions[name] = 1  # 发送者 ACK 槽释放
            elif i == enc.eof_index:
                slot = "eof"
                contributions[name] = EOF
            else:
                contributions[name] = enc.stream[i]

        bus = 0 if any(b == 0 for b in contributions.values()) else 1
        if slot == "ack" and self._ack_pullers:
            bus = 0  # 显式接收者拉低
        if i in self._forced:
            bus &= self._forced[i]  # 外力注入与总线线与

        # 位监控：ACK 槽发送者释放，不监控；其余位置（含 EOF）双向失配检查。
        if slot in ("data", "eof"):
            for name, bit in contributions.items():
                if bit == bus:
                    continue
                if (
                    slot == "data"
                    and bit == 1
                    and bus == 0
                    and i in self._enc[name].id_field_indices
                ):
                    losers.append(name)
                else:
                    # ID 域发0读1（仅外力可造），或 ID 之外任何位置失配
                    bit_error_nodes.append(name)

        self._bus_bits.append(bus)

        if bit_error_nodes:
            self._events.append(Event(i, "bit", bus, tuple(contributions), ""))
            self._begin_error_flag("bit_error", i, bit_error_nodes)
            return StepResult(i, "error", bus, tuple(contributions), (),
                              tuple(bit_error_nodes))

        if losers:
            for name in losers:
                self._active.discard(name)
                self._events.append(
                    Event(
                        i,
                        "arbitration_loss",
                        bus,
                        (name,),
                        f"{name} sent recessive 1 but read dominant 0 in ID field",
                    )
                )

        if slot == "ack":
            if bus == 0:
                self._acked = True
                self._events.append(
                    Event(i, "ack", bus, tuple(contributions),
                          f"ACK slot pulled low by {sorted(self._ack_pullers)}")
                )
                self._phase = "eof"
                out_phase = "ack"
            else:
                detectors = self._active_ordered()
                self._events.append(
                    Event(i, "ack_error", bus, tuple(detectors),
                          "no receiver pulled ACK low")
                )
                self._begin_error_flag("ack_error", i, detectors)
                return StepResult(i, "error", bus, tuple(contributions))
        elif slot == "eof":
            self._events.append(Event(i, "eof", bus, tuple(contributions), "EOF"))
            self._phase = "done"
            self._finished = True
            self._events.append(
                Event(i, "complete", bus, tuple(self._active_ordered()),
                      "frame completed successfully")
            )
            out_phase = "done"
        else:
            if not losers:
                self._events.append(
                    Event(i, "bit", bus, tuple(contributions), "")
                )
            out_phase = "transmit"

        self._pos += 1
        return StepResult(
            i,
            out_phase,
            bus,
            tuple(contributions),
            tuple(losers),
        )

    def _begin_error_flag(
        self, trigger: str, index: int, participants: list[str]
    ) -> None:
        aborted = self._active_ordered()
        self._error_info = ErrorFrameInfo(
            trigger=trigger,
            bit_index=index,
            participants=tuple(participants),
            aborted=tuple(aborted),
        )
        self._events.append(
            Event(
                index,
                trigger,
                0,
                tuple(participants),
                f"{trigger} detected: error frame begins, current transfer aborted",
            )
        )
        self._active = set()
        self._phase = "error"
        self._error_to_go = ERROR_FLAG_BITS
        self._pos += 1

    def _step_error_flag(self) -> StepResult:
        i = self._pos
        flag_no = ERROR_FLAG_BITS - self._error_to_go + 1
        info = self._error_info
        self._bus_bits.append(0)
        self._events.append(
            Event(
                i,
                "error_flag",
                0,
                info.participants if info else (),
                f"dominant error flag bit {flag_no}/{ERROR_FLAG_BITS}",
            )
        )
        self._error_to_go -= 1
        self._pos += 1
        if self._error_to_go == 0:
            self._phase = "done"
            self._finished = True
        return StepResult(i, "error", 0, ())

    # ---- 批量运行与结果 --------------------------------------------------

    def run(self) -> BusResult:
        """推进到本轮传输结束（正常完成或错误帧发完），返回结果快照。"""
        while not self._finished:
            self.step()
        return self.result()

    def result(self) -> BusResult:
        """传输结束后的不可变结果；未结束时调用为契约错误。"""
        if not self._finished:
            raise ValidationError(
                "result() requires a finished transmission; call run() first"
            )
        decoded = None
        crc_ok: bool | None = None
        winners: tuple[str, ...] = ()
        if self._error_info is None:
            winners = tuple(self._active_ordered())
            winner_enc = self._enc[winners[0]]
            stuffed = list(self._bus_bits[1 : 1 + len(winner_enc.stuffed)])
            dec = decode_frame(stuffed)
            decoded = dec.frame
            crc_ok = dec.crc_ok
        return BusResult(
            winners=winners,
            completed=self._error_info is None,
            bus_bits=tuple(self._bus_bits),
            events=tuple(self._events),
            error_frame=self._error_info,
            decoded_frame=decoded,
            decoded_crc_ok=crc_ok,
            acked=self._acked,
        )

    # ---- 内部工具 --------------------------------------------------------

    def _active_ordered(self) -> list[str]:
        return [s.name for s in self._senders if s.name in self._active]
