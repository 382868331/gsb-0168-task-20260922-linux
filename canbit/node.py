"""接收节点与固定接收过滤表（本题契约，不是完整 CAN 标准）。

* 每个 :class:`Node` 拥有一张在构造时固定的过滤表：若干条
  :class:`Filter`（11 位 id + 11 位 mask）。
* 命中条件：``(frame_id & mask) == (id & mask)``。规则按表内顺序
  从 0 编号；一帧同时命中多条规则时**只交付一次**，命中序号取
  **最早**（索引最小）的规则；空表不匹配任何帧。
* 过滤**只决定是否上交 payload**：节点对每一帧都照常逐位接收、
  去填充并做 CRC 校验，过滤结果不影响 ACK 行为。
* ``listen_only=True`` 的监听模式节点：永不拉低 ACK、永不发送
  错误标志，但逐位记录自己看到的校验结果（CRC/填充正确与否）。
* 过滤表在一次传输期间不允许修改：Filter 与 Node 均为 frozen
  数据结构，且 :class:`~canbit.bus.BitBus` 只暴露只读视图。
"""

from __future__ import annotations

from dataclasses import dataclass

from .errors import FrameDecodeError, ValidationError
from .framing import ID_MAX, decode_frame

FILTER_ID_BITS = 11
FILTER_MASK_MAX = ID_MAX  # 掩码限 11 位：0..0x7FF
FILTER_MAX_RULES = 32  # 每节点固定表的条目上限（契约边界）


@dataclass(frozen=True)
class Filter:
    """一条接收过滤规则：``(frame_id & mask) == (id & mask)`` 即命中。

    id/mask 均限 11 位（0..0x7FF）；bool 不当整数；mask=0 命中全部
    11 位 ID，mask=0x7FF 要求 ID 精确相等。
    """

    id: int
    mask: int = FILTER_MASK_MAX

    def __post_init__(self) -> None:
        if isinstance(self.id, bool) or not isinstance(self.id, int):
            raise ValidationError(
                f"filter id must be int (bool rejected), got "
                f"{type(self.id).__name__}",
                "id",
            )
        if isinstance(self.mask, bool) or not isinstance(self.mask, int):
            raise ValidationError(
                f"filter mask must be int (bool rejected), got "
                f"{type(self.mask).__name__}",
                "mask",
            )
        if not 0 <= self.id <= FILTER_MASK_MAX:
            raise ValidationError(
                f"filter id {self.id} out of 11-bit range "
                f"[0, 0x{FILTER_MASK_MAX:03X}]",
                "id",
            )
        if not 0 <= self.mask <= FILTER_MASK_MAX:
            raise ValidationError(
                f"filter mask {self.mask} out of 11-bit range "
                f"[0, 0x{FILTER_MASK_MAX:03X}]",
                "mask",
            )

    def matches(self, frame_id: int) -> bool:
        """frame_id 是否命中本规则（frame_id 假定为合法 11 位 ID）。"""
        return (frame_id & self.mask) == (self.id & self.mask)


def first_match(filters: tuple[Filter, ...], frame_id: int) -> int | None:
    """返回最早命中规则的序号；无命中返回 None。重叠规则只取最早者。"""
    for index, rule in enumerate(filters):
        if rule.matches(frame_id):
            return index
    return None


@dataclass(frozen=True)
class ReceiveRecord:
    """一个节点对**一帧**的接收记录（无论上交与否都会生成）。

    frame:        按线上位流解码出的帧；填充/长度违例导致字段无法
                  切分时为 None
    delivered:    是否上交 payload（CRC 正确 **且** 命中过滤表）
    rule_index:   上交时命中的规则序号（最早规则）；未上交为 None
    crc_ok:       该节点看到的校验结果（去填充后 CRC 是否通过）
    stuff_ok:     填充规则是否完好
    acked:        该节点是否在 ACK 槽拉低了总线（监听节点恒为 False）
    listen_only:  该节点是否为监听模式
    error:        未能形成完整帧时的机器可读原因：
                  None | "stuff" | "length" | "protocol"
    """

    frame: object | None
    delivered: bool
    rule_index: int | None
    crc_ok: bool
    stuff_ok: bool
    acked: bool
    listen_only: bool
    error: str | None = None

    @property
    def valid(self) -> bool:
        """完整且 CRC/填充正确的帧。"""
        return self.stuff_ok and self.crc_ok and self.error is None


@dataclass(frozen=True)
class Node:
    """接收节点：固定过滤表 + 模式（普通/监听）。

    name:         节点名（节点间唯一）；与某发送者同名时，该参与者
                  先以发送者身份参战，**仲裁失败后自动转为接收节点**
                  （沿用本节点的过滤表与监听模式）
    filters:      固定过滤规则序列（0..FILTER_MAX_RULES 条）；frozen
                  结构只保存元组，构造后不可修改
    listen_only:  True=监听模式：永不 ACK、永不发错误标志，只记录
    """

    name: str
    filters: tuple[Filter, ...] = ()
    listen_only: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name:
            raise ValidationError("node name must be a non-empty str", "name")
        if isinstance(self.filters, (str, bytes)) or not hasattr(
            self.filters, "__iter__"
        ):
            raise ValidationError(
                f"filters must be an iterable of Filter, got "
                f"{type(self.filters).__name__}",
                "filters",
            )
        rules = tuple(self.filters)
        for i, rule in enumerate(rules):
            if not isinstance(rule, Filter):
                raise ValidationError(
                    f"filters[{i}] must be Filter, got {type(rule).__name__}",
                    "filters",
                )
        if len(rules) > FILTER_MAX_RULES:
            raise ValidationError(
                f"filter table has {len(rules)} rules, exceeds "
                f"FILTER_MAX_RULES={FILTER_MAX_RULES}",
                "filters",
            )
        if not isinstance(self.listen_only, bool):
            raise ValidationError("listen_only must be bool", "listen_only")
        object.__setattr__(self, "filters", rules)

    def first_match(self, frame_id: int) -> int | None:
        """frame_id 最早命中的规则序号；无命中 None。"""
        return first_match(self.filters, frame_id)


def evaluate_reception(
    node: Node,
    stuffed_bits: list[int] | tuple[int, ...],
    *,
    acked: bool,
) -> ReceiveRecord:
    """纯函数：节点对一段 ID..CRC 填充流做去填充/CRC/过滤的完整判定。

    CRC 损坏（填充结构完好）不抛异常，记录 crc_ok=False；填充规则
    违例等使字段无法切分的情况记录 error，frame=None。过滤只决定
    delivered 与 rule_index，不改变 crc_ok/stuff_ok。
    """
    try:
        dec = decode_frame(stuffed_bits, raise_on_crc=False)
    except FrameDecodeError as exc:
        kind = exc.kind if exc.kind in ("stuff", "length", "protocol") else "stuff"
        return ReceiveRecord(
            frame=None,
            delivered=False,
            rule_index=None,
            crc_ok=False,
            stuff_ok=kind != "stuff",
            acked=acked,
            listen_only=node.listen_only,
            error=kind,
        )

    frame = dec.frame
    rule = node.first_match(frame.identifier)
    delivered = dec.crc_ok and rule is not None
    return ReceiveRecord(
        frame=frame,
        delivered=delivered,
        rule_index=rule if delivered else None,
        crc_ok=dec.crc_ok,
        stuff_ok=True,
        acked=acked,
        listen_only=node.listen_only,
        error=None,
    )
