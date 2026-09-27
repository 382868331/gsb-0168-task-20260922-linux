"""待发送队列：仲裁失败者按原有规则重新排队参与后续仲裁。

语义（本题契约）：

* 每轮把当前队列中的全部帧交给一个 BitBus 同时仲裁；
  胜者记录为“成功发送”，退出队列。
* 仲裁失败（ID 域发 1 读 0）不是传输错误：失败者 retries 加 1，
  未超过 max_retries 则重新入队，下一轮按同一套仲裁规则再参与。
* 真正的传输错误（bit_error / ack_error 触发错误帧）与仲裁失败分开
  处理：被错误帧中止的发送者记入 errors，不消耗重试次数，
  也不再重试；同一轮里已经仲裁失败的帧仍按仲裁失败处理。
* cancel(name) 只从待发送队列中移除帧；已经开始的一轮传输
  （其他正在发送的帧）不受任何影响。
* 最终 QueueResult 明确区分四类归宿：
  sent（成功发送）、pending（仍在等待，仅在达到 max_rounds
  上限时出现）、exhausted（重试耗尽）、errors（传输错误）。
"""

from __future__ import annotations

from dataclasses import dataclass

from .bus import BitBus, BusResult, ErrorFrameInfo, Sender
from .errors import ValidationError
from .framing import Frame


@dataclass(frozen=True)
class SentRecord:
    """成功发送记录：retries 为实际发生过的仲裁失败次数。"""

    name: str
    frame: Frame
    retries: int
    round_index: int


@dataclass(frozen=True)
class PendingRecord:
    """仍在队列中等待的记录（retries 为已用重试次数）。"""

    name: str
    frame: Frame
    retries: int


@dataclass(frozen=True)
class ExhaustedRecord:
    """重试耗尽记录：仲裁失败次数超过 max_retries，移出队列。"""

    name: str
    frame: Frame
    retries: int
    max_retries: int


@dataclass(frozen=True)
class ErrorRecord:
    """传输错误记录：被错误帧中止，不消耗重试次数、不再重试。"""

    name: str
    frame: Frame
    round_index: int
    error_frame: ErrorFrameInfo


@dataclass(frozen=True)
class RoundRecord:
    """单轮仲裁的结果汇总。"""

    round_index: int
    bus_result: BusResult
    sent: tuple[str, ...]
    requeued: tuple[str, ...]
    exhausted: tuple[str, ...]
    errored: tuple[str, ...]


@dataclass(frozen=True)
class QueueResult:
    """队列排空（或达到轮次上限）后的最终记录。"""

    sent: tuple[SentRecord, ...]
    pending: tuple[PendingRecord, ...]
    exhausted: tuple[ExhaustedRecord, ...]
    errors: tuple[ErrorRecord, ...]
    rounds: tuple[RoundRecord, ...]


@dataclass
class _Entry:
    sender: Sender
    retries: int
    max_retries: int


def _check_count(value: int, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValidationError(
            f"{field_name} must be int (bool rejected)", field_name
        )
    if value < 0:
        raise ValidationError(f"{field_name} must be >= 0", field_name)
    return value


class TxQueue:
    """待发送队列：逐轮仲裁，仲裁失败者按原规则重试。

    参数：
        ack_pullers: 每轮传给 BitBus 的显式 ACK 接收者集合
        forced_bits: 每轮都生效的确定性故障注入（测试/演示用）
        max_rounds:  轮次上限；None 表示不限。达到上限时队列中剩余
                     的帧在结果里记为 pending（仍在等待）
    """

    def __init__(
        self,
        *,
        ack_pullers=(),
        forced_bits: dict[int, int] | None = None,
        max_rounds: int | None = None,
    ) -> None:
        if max_rounds is not None:
            _check_count(max_rounds, "max_rounds")
            if max_rounds == 0:
                raise ValidationError("max_rounds must be >= 1", "max_rounds")
        self._ack_pullers = tuple(ack_pullers)
        self._forced_bits = forced_bits
        self._max_rounds = max_rounds
        self._entries: list[_Entry] = []
        self._sent: list[SentRecord] = []
        self._exhausted: list[ExhaustedRecord] = []
        self._errors: list[ErrorRecord] = []
        self._rounds: list[RoundRecord] = []
        self._round_index = 0

    # ---- 队列操作 -------------------------------------------------------

    def enqueue(self, sender: Sender, *, max_retries: int = 3) -> None:
        """把一帧放入待发送队列；同名参与者不允许重复排队。"""
        if not isinstance(sender, Sender):
            raise ValidationError(
                f"sender must be Sender, got {type(sender).__name__}", "sender"
            )
        _check_count(max_retries, "max_retries")
        if any(e.sender.name == sender.name for e in self._entries):
            raise ValidationError(
                f"sender {sender.name!r} already queued", "sender"
            )
        self._entries.append(_Entry(sender, 0, max_retries))

    def cancel(self, name: str) -> bool:
        """取消仍在排队的一帧；返回是否真的移除了。

        只影响待发送队列：已经开始的一轮传输中其他发送者不受任何
        影响。名字不在队列中返回 False（不静默成功也不报错）。
        """
        if not isinstance(name, str) or not name:
            raise ValidationError("name must be a non-empty str", "name")
        for i, entry in enumerate(self._entries):
            if entry.sender.name == name:
                del self._entries[i]
                return True
        return False

    @property
    def pending_names(self) -> tuple[str, ...]:
        return tuple(e.sender.name for e in self._entries)

    # ---- 运行 -----------------------------------------------------------

    def step_round(self) -> RoundRecord | None:
        """执行一轮仲裁；队列为空或已达轮次上限时返回 None。"""
        if not self._entries:
            return None
        if self._max_rounds is not None and self._round_index >= self._max_rounds:
            return None

        round_index = self._round_index
        self._round_index += 1
        entries = self._entries
        self._entries = []

        bus = BitBus(
            [e.sender for e in entries],
            ack_pullers=self._ack_pullers,
            forced_bits=self._forced_bits,
        )
        result = bus.run()

        order = [e.sender.name for e in entries]
        by_name = {e.sender.name: e for e in entries}
        losers = {
            ev.participants[0]
            for ev in result.events
            if ev.kind == "arbitration_loss"
        }

        sent: list[str] = []
        requeued: list[str] = []
        exhausted: list[str] = []
        errored: list[str] = []

        if result.error_frame is not None:
            # 真正的传输错误：被中止者记 errors，不消耗重试、不重试；
            # 本轮已仲裁失败者仍按仲裁失败重新排队。
            for name in result.error_frame.aborted:
                entry = by_name[name]
                self._errors.append(
                    ErrorRecord(name, entry.sender.frame, round_index,
                                result.error_frame)
                )
                errored.append(name)
        else:
            for name in result.winners:
                entry = by_name[name]
                self._sent.append(
                    SentRecord(name, entry.sender.frame, entry.retries,
                               round_index)
                )
                sent.append(name)

        for name in sorted(losers, key=order.index):
            entry = by_name[name]
            # 只统计真正发生的重试：预算已用完时本次仲裁失败后不再重试。
            if entry.retries >= entry.max_retries:
                self._exhausted.append(
                    ExhaustedRecord(name, entry.sender.frame, entry.retries,
                                    entry.max_retries)
                )
                exhausted.append(name)
            else:
                entry.retries += 1
                self._entries.append(entry)
                requeued.append(name)

        record = RoundRecord(
            round_index=round_index,
            bus_result=result,
            sent=tuple(sent),
            requeued=tuple(requeued),
            exhausted=tuple(exhausted),
            errored=tuple(errored),
        )
        self._rounds.append(record)
        return record

    def run(self) -> QueueResult:
        """逐轮运行直到队列排空或达到 max_rounds，返回最终记录。"""
        while self.step_round() is not None:
            pass
        return self.result()

    def result(self) -> QueueResult:
        """当前记录快照（sent/pending/exhausted/errors 四类）。"""
        return QueueResult(
            sent=tuple(self._sent),
            pending=tuple(
                PendingRecord(e.sender.name, e.sender.frame, e.retries)
                for e in self._entries
            ),
            exhausted=tuple(self._exhausted),
            errors=tuple(self._errors),
            rounds=tuple(self._rounds),
        )
