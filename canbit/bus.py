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

from dataclasses import dataclass, field, replace
from typing import Iterable, Mapping

from .errors import BusFinishedError, ValidationError
from .framing import (
    EOF,
    EncodedFrame,
    Frame,
    decode_frame,
    encode_frame,
)
from .node import Node, ReceiveRecord, evaluate_reception

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
    """一次完整传输的结果快照。

    receptions: 节点名 -> ReceiveRecord；包含本轮全程在收的所有节点
                （纯接收节点与仲裁失败转接收的发送者同名节点）。
    arbitration_losers: 本轮仲裁失败者（名字, 失配位索引），按发生顺序。
    """

    winners: tuple[str, ...]
    completed: bool
    bus_bits: tuple[int, ...]
    events: tuple[Event, ...]
    error_frame: ErrorFrameInfo | None
    decoded_frame: Frame | None
    decoded_crc_ok: bool | None
    acked: bool
    receptions: Mapping[str, ReceiveRecord] = field(default_factory=dict)
    arbitration_losers: tuple[tuple[str, int], ...] = ()

    def delivered(self) -> tuple[tuple[str, ReceiveRecord], ...]:
        """实际上交 payload 的 (节点名, 记录)，按节点加入顺序。"""
        return tuple(
            (name, rec)
            for name, rec in self.receptions.items()
            if rec.delivered
        )


class BitBus:
    """逐 bit 推进的有线与总线。

    参数：
        senders:      本轮同时开始发送的参与者（至少 1 个，名字唯一）
        ack_pullers:  显式 ACK 参与者名：在 ACK 槽拉低总线的**额外**
                      参与者；可为任意名字（含纯接收者）。提供 nodes
                      时，普通接收节点对“完整且 CRC 正确”的帧会**自动**
                      ACK（即使该帧被其过滤掉），监听节点永不 ACK；
                      本参数用于没有 Node 对象的旧用法/直接指定
        nodes:        本轮在收的接收节点（Node，含固定过滤表）。名字
                      与发送者同名不冲突：该参与者先发送，仲裁失败后
                      自动以该 Node 身份转接收；节点名之间须唯一。
                      过滤表在本次传输期间不可修改（Node 为 frozen）
        forced_bits:  位索引 -> 线上被外力强制的电平（0/1），与所有驱动者
                      线与。用于确定性地注入故障（如破坏填充位）；
                      例如在某发送 1 的位置强制 0 -> 该发送者发 1 读 0
        receive_glitches:
                      位索引 -> 仅作用于**接收视角**位缓冲的确定性注入，
                      按线与语义与读到位相与（与 forced_bits 一致），
                      因此只可能把某接收位 1->0。它不改变总线电平与
                      发送者监控：发送者照常发完且不报位错误，接收者
                      却各自看到被破坏的位，用于确定性模拟“匹配帧
                      CRC 损坏”（接收侧 CRC 校验失败、不上交、不 ACK，
                      而总线本身无错误帧）
    """

    def __init__(
        self,
        senders: Iterable[Sender],
        *,
        ack_pullers: Iterable[str] = (),
        nodes: Iterable[Node] = (),
        forced_bits: dict[int, int] | None = None,
        receive_glitches: dict[int, int] | None = None,
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

        if isinstance(nodes, (str, bytes)) or not hasattr(nodes, "__iter__"):
            raise ValidationError(
                f"nodes must be an iterable of Node, got {type(nodes).__name__}",
                "nodes",
            )
        node_list = list(nodes)
        for i, nd in enumerate(node_list):
            if not isinstance(nd, Node):
                raise ValidationError(
                    f"nodes[{i}] must be Node, got {type(nd).__name__}", "nodes"
                )
        node_names = [nd.name for nd in node_list]
        if len(set(node_names)) != len(node_names):
            raise ValidationError(
                f"node names must be unique: {node_names}", "nodes"
            )
        node_name_set = set(node_names)
        listener_conflict = pullers & {
            nd.name for nd in node_list if nd.listen_only
        }
        if listener_conflict:
            raise ValidationError(
                f"listen-only node(s) {sorted(listener_conflict)} must never pull "
                "ACK, but are listed in ack_pullers",
                "ack_pullers",
            )
        # 普通节点的 ACK 完全由其自身 CRC 判定决定，不允许再用 ack_pullers
        # 对同一名字重复指定（避免“坏帧也被迫 ACK”的歧义）。
        overlap = pullers & node_name_set
        if overlap:
            raise ValidationError(
                f"node name(s) {sorted(overlap)} control their own ACK by CRC; "
                "do not also list them in ack_pullers",
                "ack_pullers",
            )

        forced = self._validate_indexed_bits(forced_bits, "forced_bits")
        glitched = self._validate_indexed_bits(
            receive_glitches, "receive_glitches"
        )

        self._senders = senders
        self._ack_pullers = pullers
        self._nodes = node_list
        self._node_by_name = {nd.name: nd for nd in node_list}
        self._forced = forced
        self._glitched = glitched
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

        # 接收视角状态（逐 bit 驱动；与总线推进严格同步）
        # 每个在收节点维护 ID..CRC 填充位缓冲。仲裁失败者在失配的下一
        # 位时间才转为接收（失配位它仍在发送），缓冲按已发总线位回填。
        self._rx_active: set[str] = {
            nd.name for nd in node_list if nd.name not in self._active
        }
        self._rx_bits: dict[str, list[int]] = {n: [] for n in self._rx_active}
        self._loser_positions: list[tuple[str, int]] = []
        self._receptions: dict[str, ReceiveRecord] = {}
        self._rx_records: dict[str, ReceiveRecord] = {}

    @staticmethod
    def _validate_indexed_bits(values: object, field_name: str) -> dict[int, int]:
        """校验 {位索引: 0/1} 形态的确定性注入映射。"""
        out: dict[int, int] = {}
        if values is None:
            return out
        if not isinstance(values, dict):
            raise ValidationError(f"{field_name} must be dict[int, int]", field_name)
        for idx, val in values.items():
            if isinstance(idx, bool) or not isinstance(idx, int):
                raise ValidationError(
                    f"{field_name} keys must be int (bool rejected)", field_name
                )
            if idx < 0:
                raise ValidationError(
                    f"{field_name} index {idx} must be >= 0", field_name
                )
            if isinstance(val, bool) or not isinstance(val, int) or val not in (0, 1):
                raise ValidationError(
                    f"{field_name}[{idx}] must be int 0/1 (bool rejected)",
                    field_name,
                )
            out[idx] = val
        return out

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

    @property
    def receiver_names(self) -> tuple[str, ...]:
        """本轮参与接收的节点名（构造顺序）：纯接收节点全集。"""
        return tuple(nd.name for nd in self._nodes)

    def node_filters(self, node_name: str) -> tuple:
        """只读返回某节点的固定过滤表（元组视图；传输期间不可修改）。"""
        if node_name not in self._node_by_name:
            raise ValidationError(f"unknown node {node_name!r}", "node_name")
        return self._node_by_name[node_name].filters

    # ---- 逐 bit 推进 -----------------------------------------------------

    def step(self) -> StepResult:
        """推进一个位时间。传输结束后再调用抛 BusFinishedError。"""
        if self._finished:
            raise BusFinishedError("transmission already finished")
        if self._phase == "error":
            return self._step_error_flag()
        return self._step_normal()

    def _rx_sample(self, index: int, wire_bit: int) -> int:
        """接收节点在某位置实际采样到的位。

        receive_glitches 与 forced_bits 一样按线与语义注入：该位被
        外力强制成 glitched[index]，故只有 1->0 的破坏能自然发生。
        """
        if index in self._glitched:
            return wire_bit & self._glitched[index]
        return wire_bit

    def _promote_loser(self, name: str, through_index: int) -> None:
        """仲裁失败者转接收：以同名 Node 身份从下一位起接收。

        失配位之前它一直在发送/监听总线，且存活发送者的已发位历史
        必然相同，因此用总线位（含接收视角注入）回填 1..through_index-1
        的接收缓冲，当前位由本轮的数据位采样统一追加。
        """
        node = self._node_by_name.get(name)
        if node is None or name in self._rx_active:
            return
        seeded = [
            self._rx_sample(k, self._bus_bits[k]) for k in range(1, through_index)
        ]
        self._rx_bits[name] = seeded
        self._rx_active.add(name)

    def _evaluate_ack(self) -> tuple[set[str], dict[str, ReceiveRecord]]:
        """ACK 槽：对每个在收节点用其逐位缓冲做去填充/CRC/过滤判定。

        返回 (实际拉低 ACK 的节点集合, 各节点接收记录)。普通节点仅当
        帧完整且 CRC/填充正确时才拉低（**与是否被过滤无关**）；监听
        节点永不拉低。
        """
        pullers: set[str] = set()
        records: dict[str, ReceiveRecord] = {}
        for nd in self._nodes:
            if nd.name not in self._rx_active:
                continue
            rec = evaluate_reception(
                nd, self._rx_bits[nd.name], acked=False
            )
            if not nd.listen_only and rec.valid:
                pullers.add(nd.name)
            records[nd.name] = rec
        return pullers, records

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

        # ACK 槽：先让每个在收节点基于截至上一位的完整缓冲做 CRC 判定，
        # 再决定总线电平。过滤不参与此判定（被过滤的正确帧仍可 ACK）。
        # 构造期已保证 ack_pullers 与任何节点名不重叠，故直接并集。
        rx_pullers: set[str] = set()
        if slot == "ack":
            rx_pullers, self._rx_records = self._evaluate_ack()
            effective_pullers = self._ack_pullers | rx_pullers
        else:
            effective_pullers = self._ack_pullers

        bus = 0 if any(b == 0 for b in contributions.values()) else 1
        if slot == "ack" and effective_pullers:
            bus = 0  # 健康普通接收节点 / 显式接收者拉低
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
            # 接收节点同样采样到这个（可能被干扰的）ID..CRC 位
            if slot == "data":
                self._sample_data_slot(i, bus, skip=frozenset())
            self._events.append(Event(i, "bit", bus, tuple(contributions), ""))
            self._begin_error_flag("bit_error", i, bit_error_nodes)
            return StepResult(i, "error", bus, tuple(contributions), (),
                              tuple(bit_error_nodes))

        if losers:
            for name in losers:
                self._active.discard(name)
                self._loser_positions.append((name, i))
                # 仲裁失败的同名节点下一位起转接收（缓冲回填到 i-1）
                self._promote_loser(name, i)
                self._events.append(
                    Event(
                        i,
                        "arbitration_loss",
                        bus,
                        (name,),
                        f"{name} sent recessive 1 but read dominant 0 in ID field",
                    )
                )

        # 接收视角逐位采样
        if slot == "data":
            self._sample_data_slot(i, bus, skip=frozenset(losers))
        elif slot == "ack":
            for name, rec in self._rx_records.items():
                # 记录“该节点自己”是否拉低（裸 ack_pullers 不计入节点行为）
                did_pull = bus == 0 and name in rx_pullers
                self._rx_records[name] = replace(rec, acked=did_pull)

        if slot == "ack":
            if bus == 0:
                self._acked = True
                self._events.append(
                    Event(i, "ack", bus, tuple(contributions),
                          f"ACK slot pulled low by {sorted(effective_pullers)}")
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

    def _sample_data_slot(
        self, index: int, bus_bit: int, *, skip: frozenset[str]
    ) -> None:
        """所有在收节点采样本位（ID..CRC 区，不含 SOF/ACK/EOF）。

        skip 为本位刚失配、其缓冲此前尚未包含本位的仲裁失败者；
        普通节点的缓冲已在循环中追加。
        """
        if index == 0:
            return  # SOF 不属于 ID..CRC 填充流，不进接收缓冲
        for name in tuple(self._rx_active):
            if name in skip:
                continue
            self._rx_bits[name].append(self._rx_sample(index, bus_bit))
        # 本位失配者的缓冲已回填到 index-1，这里补上失配位自身
        for name in skip:
            if name in self._rx_active:
                self._rx_bits[name].append(self._rx_sample(index, bus_bit))

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
            receptions = dict(self._rx_records)
        else:
            receptions = self._aborted_receptions()
        return BusResult(
            winners=winners,
            completed=self._error_info is None,
            bus_bits=tuple(self._bus_bits),
            events=tuple(self._events),
            error_frame=self._error_info,
            decoded_frame=decoded,
            decoded_crc_ok=crc_ok,
            acked=self._acked,
            receptions=receptions,
            arbitration_losers=tuple(self._loser_positions),
        )

    def _aborted_receptions(self) -> dict[str, ReceiveRecord]:
        """错误帧中止后各接收节点的记录。

        ack_error：ACK 槽前帧体完整，沿用槽上已做的去填充/CRC/过滤
        判定（acked 恒 False）。bit_error：帧体在错误位被截断，接收者
        按各自逐位缓冲尽解码（监听节点同样记录看到的校验结果）。
        """
        trigger = self._error_info.trigger if self._error_info else ""
        if trigger == "ack_error":
            return dict(self._rx_records)
        out: dict[str, ReceiveRecord] = {}
        for nd in self._nodes:
            if nd.name not in self._rx_active:
                continue
            out[nd.name] = evaluate_reception(
                nd, self._rx_bits[nd.name], acked=False
            )
        return out

    # ---- 内部工具 --------------------------------------------------------

    def _active_ordered(self) -> list[str]:
        return [s.name for s in self._senders if s.name in self._active]
