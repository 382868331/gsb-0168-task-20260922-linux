"""待发送队列与仲裁失败重试调度（本题契约子集的增量扩展）。

在 BitBus 单轮传输之上增加一个多轮调度器：

* 每轮把当前等待队列中的全部发送者交给 BitBus，按原有仲裁规则
  （ID 域发 1 读 0 退出仲裁，其余位置失配报 bit_error）真实逐 bit 仿真。
* **仲裁失败**（arbitration_loss）与**真正的传输错误**（bit_error /
  ack_error 引发的错误帧中止）分开计数：前者计入 arbitration_retries，
  后者计入 error_retries；重试次数只按实际发生的情况累加，不预扣、
  不估算。
* 每次失败只要重试预算（max_retries，两类合计）未耗尽就重新入队，
  下一轮再次参与仲裁；预算耗尽标记为 exhausted，不再参与。
* cancel(name) 只从等待队列中移除帧；正在进行的本轮传输不受影响，
  该轮中的其他帧照常完成。
* 最终记录（FrameRecord.status）区分：
  sent（成功发送）、pending（仍在等待）、exhausted（重试耗尽）、
  cancelled（排队期间被取消）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

from .bus import BitBus, BusResult, Sender
from .errors import ValidationError
from .framing import check_int

STATUS_SENT = "sent"
STATUS_PENDING = "pending"
STATUS_EXHAUSTED = "exhausted"
STATUS_CANCELLED = "cancelled"


@dataclass(frozen=True)
class FrameRecord:
    """一个待发送帧的全生命周期记录。

    status:              sent | pending | exhausted | cancelled
    arbitration_retries: 因仲裁失败（ID 域发 1 读 0）实际重试的次数
    error_retries:       因传输错误（bit_error/ack_error 错误帧中止）
                         实际重试的次数
    attempts:            已真实参与的总线轮数（含成功的那一轮）
    """

    name: str
    frame: object
    status: str
    arbitration_retries: int
    error_retries: int
    attempts: int

    @property
    def retries(self) -> int:
        """实际发生的重试总次数（仲裁失败 + 传输错误）。"""
        return self.arbitration_retries + self.error_retries


@dataclass(frozen=True)
class RoundRecord:
    """一轮总线传输的结算快照。"""

    round_index: int
    participants: tuple[str, ...]
    result: BusResult
    sent: tuple[str, ...]
    requeued: tuple[str, ...]
    exhausted: tuple[str, ...]


@dataclass(frozen=True)
class ScheduleReport:
    """调度结束后的完整记录。

    records 按入队顺序给出每个帧的最终状态与分类重试计数；
    所有帧均为终态（sent/exhausted/cancelled）时调度才算结束。
    """

    records: tuple[FrameRecord, ...]
    rounds: tuple[RoundRecord, ...]

    @property
    def sent(self) -> tuple[str, ...]:
        return tuple(r.name for r in self.records if r.status == STATUS_SENT)

    @property
    def pending(self) -> tuple[str, ...]:
        return tuple(r.name for r in self.records if r.status == STATUS_PENDING)

    @property
    def exhausted(self) -> tuple[str, ...]:
        return tuple(r.name for r in self.records if r.status == STATUS_EXHAUSTED)


@dataclass
class _Entry:
    sender: Sender
    status: str = STATUS_PENDING
    arbitration_retries: int = 0
    error_retries: int = 0
    attempts: int = 0

    def record(self) -> FrameRecord:
        return FrameRecord(
            name=self.sender.name,
            frame=self.sender.frame,
            status=self.status,
            arbitration_retries=self.arbitration_retries,
            error_retries=self.error_retries,
            attempts=self.attempts,
        )


class RetryScheduler:
    """带待发送队列的多轮仲裁调度器。

    参数：
        senders:      初始入队的发送者（可为空，之后用 enqueue 追加）
        max_retries:  每个帧允许的重试次数上限（仲裁失败与传输错误合计，
                      只按实际发生计数）；0 表示失败一次即耗尽
        ack_pullers:  每轮 BitBus 的显式 ACK 接收者（同 BitBus 契约）
        forced_bits:  None、dict（每轮相同的故障注入）或可调用对象
                      f(round_index) -> dict[int, int] | None（按轮注入）
    """

    def __init__(
        self,
        senders: Iterable[Sender] = (),
        *,
        max_retries: int,
        ack_pullers: Iterable[str] = (),
        forced_bits=None,
    ) -> None:
        self._max_retries = check_int(max_retries, "max_retries", 0, 1 << 30)
        self._ack_pullers = tuple(ack_pullers)
        if forced_bits is not None and not (
            callable(forced_bits) or isinstance(forced_bits, dict)
        ):
            raise ValidationError(
                "forced_bits must be None, a dict, or callable(round_index)",
                "forced_bits",
            )
        self._forced_bits = forced_bits

        self._entries: dict[str, _Entry] = {}
        self._order: list[str] = []  # 入队顺序（含已终态），保证记录稳定
        self._queue: list[str] = []  # 等待下一轮的名字（FIFO）
        self._rounds: list[RoundRecord] = []
        self._bus: BitBus | None = None
        self._inflight: list[str] = []

        for s in senders:
            self.enqueue(s)

    # ---- 队列操作 -------------------------------------------------------

    def enqueue(self, sender: Sender) -> None:
        """把一个新帧加入等待队列；名字必须唯一（含历史记录）。"""
        if not isinstance(sender, Sender):
            raise ValidationError(
                f"sender must be Sender, got {type(sender).__name__}", "sender"
            )
        if sender.name in self._entries:
            raise ValidationError(
                f"duplicate sender name {sender.name!r}", "sender"
            )
        self._entries[sender.name] = _Entry(sender)
        self._order.append(sender.name)
        self._queue.append(sender.name)

    def cancel(self, name: str) -> bool:
        """取消仍在等待队列中的帧；正在发送的帧不受影响。

        返回 True 表示该帧本在队列中、已标记 cancelled 并移除；
        返回 False 表示该名字当前不在等待队列（未知、在发送中或已终态）。
        """
        if not isinstance(name, str) or not name:
            raise ValidationError("name must be a non-empty str", "name")
        if name not in self._queue:
            return False
        self._queue.remove(name)
        self._entries[name].status = STATUS_CANCELLED
        return True

    # ---- 状态访问 -------------------------------------------------------

    @property
    def finished(self) -> bool:
        """所有帧均为终态（无人在等待、无轮次在进行）。"""
        return self._bus is None and not self._queue

    @property
    def waiting(self) -> tuple[str, ...]:
        return tuple(self._queue)

    @property
    def in_flight(self) -> tuple[str, ...]:
        return tuple(self._inflight)

    @property
    def records(self) -> tuple[FrameRecord, ...]:
        return tuple(self._entries[n].record() for n in self._order)

    @property
    def rounds(self) -> tuple[RoundRecord, ...]:
        return tuple(self._rounds)

    # ---- 推进 -----------------------------------------------------------

    def step(self):
        """推进一个位时间；空闲且队列非空时先开启新一轮。

        调度结束后再调用抛 ValidationError（无待办却要求推进属契约错误）。
        返回当前 BitBus 的 StepResult；开启新一轮的那一拍即传输该轮第 0 位。
        """
        if self._bus is None:
            if not self._queue:
                raise ValidationError(
                    "scheduler is finished: no queued frames and no active round"
                )
            self._start_round()
        sr = self._bus.step()
        if self._bus.finished:
            self._settle_round()
        return sr

    def run(self) -> ScheduleReport:
        """推进到所有帧到达终态，返回完整调度报告。"""
        while not self.finished:
            self.step()
        return ScheduleReport(records=self.records, rounds=self.rounds)

    # ---- 内部 ------------------------------------------------------------

    def _forced_for(self, round_index: int):
        fb = self._forced_bits
        if callable(fb):
            return fb(round_index)
        return fb

    def _start_round(self) -> None:
        names = list(self._queue)
        self._queue.clear()
        self._inflight = names
        senders = [self._entries[n].sender for n in names]
        self._bus = BitBus(
            senders,
            ack_pullers=self._ack_pullers,
            forced_bits=self._forced_for(len(self._rounds)),
        )

    def _settle_round(self) -> None:
        assert self._bus is not None
        result = self._bus.result()
        index = len(self._rounds)
        participants = tuple(self._inflight)
        self._inflight = []
        self._bus = None

        sent: list[str] = []
        requeued: list[str] = []
        exhausted: list[str] = []

        lost_names = {
            e.participants[0] for e in result.events if e.kind == "arbitration_loss"
        }
        aborted = set(result.error_frame.aborted) if result.error_frame else set()

        for name in participants:
            entry = self._entries[name]
            entry.attempts += 1
            if name in aborted:
                # 真正的传输错误（bit_error / ack_error）中止
                entry.error_retries += 1
            elif name in lost_names or name not in result.winners:
                # 仲裁失败（含错误发生前已退出仲裁者）
                entry.arbitration_retries += 1
            else:
                entry.status = STATUS_SENT
                sent.append(name)
                continue
            if entry.arbitration_retries + entry.error_retries > self._max_retries:
                entry.status = STATUS_EXHAUSTED
                exhausted.append(name)
            else:
                self._queue.append(name)
                requeued.append(name)

        self._rounds.append(
            RoundRecord(
                round_index=index,
                participants=participants,
                result=result,
                sent=tuple(sent),
                requeued=tuple(requeued),
                exhausted=tuple(exhausted),
            )
        )

