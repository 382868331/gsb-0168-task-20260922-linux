"""接收过滤表与节点层：在 BitBus 之上为每个节点配置固定的接收过滤表。

规则（本题契约的新增部分，仍不是完整 CAN 标准）：

* 每个节点配置若干条过滤规则 ``(identifier, mask)``，两者均限 11 位；
  命中条件为 ``(frame_id & mask) == (identifier & mask)``。空表 = 全过滤
  （任何帧都不上交，但正常节点仍参与 ACK 与校验记录）。
* 过滤表在构造 NodeBus 时快照固化，**一次传输期间不允许修改**（frozen
  dataclass + tuple 快照，没有任何运行期修改入口）。
* 过滤只决定是否**上交 payload**：节点对线上每一帧都照常做去填充与
  CRC 校验并记录结果，不因过滤而跳过。
* 正确完整的帧即使被过滤掉，正常（非监听）接收节点仍贡献 ACK；
  仲裁失败的节点自失败位起转为接收者，同样参与 ACK。
* 监听模式节点永不 ACK、永不发错误标志，但记录看到的每帧校验结果。
* 上交时附命中的规则序号；多条规则重叠命中时只交付一次，
  依据序号取表中最早的规则。
* CRC 校验失败的帧只记录、不上交（无论过滤是否命中）。

本层不改变 BitBus 的逐 bit 仲裁/错误帧语义，也不实现错误界定计数、
重传等完整 CAN 机制。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .bus import BitBus, BusResult, Sender, StepResult
from .errors import ValidationError
from .framing import ID_MAX, EncodedFrame, Frame, check_int


# ---- 过滤规则与节点配置 ----------------------------------------------------


@dataclass(frozen=True)
class FilterRule:
    """一条接收过滤规则：命中条件 (frame_id & mask) == (identifier & mask)。

    identifier 与 mask 均限 11 位（0..0x7FF）。mask 中为零的位不参与
    比较，因此 mask=0 的规则命中所有帧。
    """

    identifier: int
    mask: int

    def __post_init__(self) -> None:
        check_int(self.identifier, "identifier", 0, ID_MAX)
        check_int(self.mask, "mask", 0, ID_MAX)

    def matches(self, frame_id: int) -> bool:
        """判断该规则是否命中给定 11 位帧 ID。"""
        frame_id = check_int(frame_id, "frame_id", 0, ID_MAX)
        return (frame_id & self.mask) == (self.identifier & self.mask)


@dataclass(frozen=True)
class BusNode:
    """一个总线节点：固定的接收过滤表 + 可选的本轮发送帧。

    name:        节点名（非空字符串，NodeBus 内唯一）
    filters:     过滤规则表（构造后冻结，一次传输期间不可修改）；
                 空表 = 全过滤：任何帧都不上交，但正常节点仍 ACK、仍记录校验
    listen_only: 监听模式：永不 ACK、永不发错误标志，只记录看到的校验结果；
                 监听节点不允许携带 tx_frame
    tx_frame:    非 None 时本轮作为发送者参与仲裁；仲裁失败后自动转为
                 接收者（参与 ACK 并按过滤表接收胜者帧）
    """

    name: str
    filters: tuple[FilterRule, ...] = ()
    listen_only: bool = False
    tx_frame: Frame | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name:
            raise ValidationError("node name must be a non-empty str", "name")
        if not isinstance(self.listen_only, bool):
            raise ValidationError(
                f"listen_only must be bool, got {type(self.listen_only).__name__}",
                "listen_only",
            )
        try:
            filters = tuple(self.filters)
        except TypeError:
            raise ValidationError(
                "filters must be an iterable of FilterRule", "filters"
            ) from None
        for f in filters:
            if not isinstance(f, FilterRule):
                raise ValidationError(
                    f"filters entries must be FilterRule, got {type(f).__name__}",
                    "filters",
                )
        # 快照为 tuple：过滤表随节点配置一起冻结
        object.__setattr__(self, "filters", filters)
        if self.tx_frame is not None and not isinstance(self.tx_frame, Frame):
            raise ValidationError(
                f"tx_frame must be Frame or None, got {type(self.tx_frame).__name__}",
                "tx_frame",
            )
        if self.listen_only and self.tx_frame is not None:
            raise ValidationError(
                "listen-only node must not carry tx_frame", "tx_frame"
            )


# ---- 接收记录 --------------------------------------------------------------


@dataclass(frozen=True)
class ReceiveRecord:
    """一个节点对一帧已完成传输的接收记录。

    crc_ok:        节点对线上位流去填充 + CRC 校验的结果（被过滤的帧也
                   照常校验并记录）
    matched_rules: 命中过滤表的全部规则序号，按表内顺序（早的在前）
    delivered:     是否上交 payload（要求 crc_ok 且至少命中一条规则；
                   重叠命中只交付一次）
    rule_index:    交付依据的规则序号 = 最早命中规则；未交付为 None
    """

    node: str
    frame: Frame
    crc_ok: bool
    matched_rules: tuple[int, ...]
    delivered: bool
    rule_index: int | None


@dataclass(frozen=True)
class NodeBusResult:
    """一轮 NodeBus 传输的结果：底层总线结果 + 每个节点的接收记录。"""

    bus: BusResult
    records: tuple[ReceiveRecord, ...]

    def records_of(self, node: str) -> tuple[ReceiveRecord, ...]:
        """某节点的全部接收记录（本轮 0 或 1 条）。"""
        if not isinstance(node, str) or not node:
            raise ValidationError("node must be a non-empty str", "node")
        return tuple(r for r in self.records if r.node == node)

    def deliveries_of(self, node: str) -> tuple[ReceiveRecord, ...]:
        """某节点实际交付（delivered=True）的记录。"""
        return tuple(r for r in self.records_of(node) if r.delivered)


# ---- 节点总线 --------------------------------------------------------------


class NodeBus:
    """带接收过滤表的节点总线：内部逐 bit 驱动一个 BitBus。

    参数：
        nodes:     本轮参与的全部节点（至少 1 个，名字唯一；至少 1 个
                   节点携带 tx_frame 作为发送者）
        forced_bits / encoded_overrides: 原样透传给 BitBus 的故障注入钩子

    与 BitBus 同为一次性对象：构造时快照全部过滤表，传输期间没有任何
    修改入口；再次传输需新建 NodeBus。
    """

    def __init__(
        self,
        nodes: Iterable[BusNode],
        *,
        forced_bits: dict[int, int] | None = None,
        encoded_overrides: dict[str, EncodedFrame] | None = None,
    ) -> None:
        nodes = list(nodes)
        if not nodes:
            raise ValidationError("at least one node required", "nodes")
        for n in nodes:
            if not isinstance(n, BusNode):
                raise ValidationError(
                    f"nodes entries must be BusNode, got {type(n).__name__}",
                    "nodes",
                )
        names = [n.name for n in nodes]
        if len(set(names)) != len(names):
            raise ValidationError(f"node names must be unique: {names}", "nodes")
        tx_nodes = [n for n in nodes if n.tx_frame is not None]
        if not tx_nodes:
            raise ValidationError(
                "at least one node must carry tx_frame (a sender is required)",
                "nodes",
            )

        self._nodes = tuple(nodes)
        self._by_name = {n.name: n for n in nodes}
        # 过滤表快照：本轮传输期间以此为准，外部无法再改动（BusNode frozen）
        self._filters: dict[str, tuple[FilterRule, ...]] = {
            n.name: tuple(n.filters) for n in nodes
        }

        senders = [Sender(n.name, n.tx_frame) for n in tx_nodes]
        # 纯接收的正常节点从一开始就参与 ACK；携带 tx_frame 的节点若仲裁
        # 失败，在失败位由 step() 动态加入（转为接收者）
        pullers = {
            n.name for n in nodes if not n.listen_only and n.tx_frame is None
        }
        self._bus = BitBus(
            senders,
            ack_pullers=pullers,
            forced_bits=forced_bits,
            encoded_overrides=encoded_overrides,
        )
        self._records: tuple[ReceiveRecord, ...] | None = None

    # ---- 状态访问（委托底层 BitBus） --------------------------------------

    @property
    def finished(self) -> bool:
        return self._bus.finished

    @property
    def phase(self) -> str:
        return self._bus.phase

    @property
    def bit_index(self) -> int:
        return self._bus.bit_index

    @property
    def events(self):
        return self._bus.events

    @property
    def bus_bits(self):
        return self._bus.bus_bits

    @property
    def bus(self) -> BitBus:
        """底层 BitBus（只读审计用：events/encoded_stream 等）。"""
        return self._bus

    def filter_table(self, node: str) -> tuple[FilterRule, ...]:
        """本轮冻结的过滤表快照（构造时固定，传输期间不可修改）。"""
        if node not in self._filters:
            raise ValidationError(f"unknown node {node!r}", "node")
        return self._filters[node]

    # ---- 逐 bit 推进 -------------------------------------------------------

    def step(self) -> StepResult:
        """推进底层总线一个位时间；仲裁失败者在此转为接收者（加入 ACK）。"""
        sr = self._bus.step()
        for loser in sr.arbitration_losers:
            node = self._by_name[loser]
            if not node.listen_only:
                self._bus.add_ack_puller(loser)
        return sr

    def run(self) -> NodeBusResult:
        """逐 bit 推进到本轮结束，返回总线结果与全部节点的接收记录。"""
        while not self._bus.finished:
            self.step()
        return self.result()

    def result(self) -> NodeBusResult:
        """传输结束后的结果；未结束时调用为契约错误（ValidationError）。"""
        bus_res = self._bus.result()
        if self._records is None:
            self._records = self._build_records(bus_res)
        return NodeBusResult(bus=bus_res, records=self._records)

    # ---- 内部工具 ----------------------------------------------------------

    def _build_records(self, bus_res: BusResult) -> tuple[ReceiveRecord, ...]:
        # 传输被错误帧中止：没有完整帧，任何节点都不产生接收记录
        if not bus_res.completed:
            return ()
        frame = bus_res.decoded_frame
        crc_ok = bool(bus_res.decoded_crc_ok)
        records: list[ReceiveRecord] = []
        for node in self._nodes:
            filters = self._filters[node.name]
            matched = tuple(
                i for i, rule in enumerate(filters) if rule.matches(frame.identifier)
            )
            delivered = crc_ok and bool(matched)
            records.append(
                ReceiveRecord(
                    node=node.name,
                    frame=frame,
                    crc_ok=crc_ok,
                    matched_rules=matched,
                    delivered=delivered,
                    rule_index=matched[0] if delivered else None,
                )
            )
        return tuple(records)
