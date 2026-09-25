"""帧结构、CRC15、位填充与整帧编解码（纯函数数据结构）。

本题契约（**不是**完整 CAN 标准，见 README“设计取舍/已知限制”）：

* 11 位标识符 ID（0..0x7FF）
* 4 位 DLC，取值 0..8，语义为数据字节数
* DLC 个数据字节（每字节 8 位，高位先发）
* 15 位 CRC：对「ID(11) + DLC(4) + 数据(8*DLC)」位串左移 15 位后
  除以生成多项式（0x4599，即 x^15+x^14+x^10+x^8+x^7+x^4+x^3+1）
  的余数，初值 0，无反射、无异或；CRC 自身高位先发
* 对 ID..CRC 的连续位流做 CAN 式位填充：每连续 5 个同位插入一个反位；
  若 CRC 末位恰好构成第 5 个同位，尾部填充位照常插入，ACK 槽在其后
* 帧的线上顺序：SOF(固定 0)，[ID..CRC 填充流]，ACK 槽，EOF(固定 1)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field as _dc_field

from .errors import FrameDecodeError, ValidationError

# ---- 公开契约常量 -------------------------------------------------------

ID_BITS = 11
DLC_BITS = 4
CRC_BITS = 15
MAX_DATA_LEN = 8
ID_MAX = (1 << ID_BITS) - 1
CRC_MASK = (1 << CRC_BITS) - 1
# 15 次生成多项式（位 15 隐式存在）：x15+x14+x10+x8+x7+x4+x3+1
POLYNOMIAL = 0x4599

SOF = 0
EOF = 1
ACK_RELEASED = 1  # 发送方在 ACK 槽释放总线（发隐性 1）


# ---- 输入校验（数值拒绝 NaN/Infinity；整数参数拒绝 bool） ---------------


def check_int(value: object, field_name: str, minimum: int, maximum: int) -> int:
    """契约化整数校验：bool 拒绝；float（含 NaN/Inf）拒绝；越界拒绝。"""
    if isinstance(value, bool):
        raise ValidationError("integer parameter must not be bool", field_name)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValidationError(
                "numeric value must be finite (NaN/Infinity rejected)", field_name
            )
        raise ValidationError("integer parameter must be int, got float", field_name)
    if not isinstance(value, int):
        raise ValidationError(
            f"expected int in [{minimum}, {maximum}], got {type(value).__name__}",
            field_name,
        )
    if not (minimum <= value <= maximum):
        raise ValidationError(
            f"value {value} out of range [{minimum}, {maximum}]", field_name
        )
    return value


def _validate_data(data: object, dlc: int | None = None) -> bytes:
    if not isinstance(data, (bytes, bytearray)):
        raise ValidationError(
            f"data must be bytes/bytearray, got {type(data).__name__}", "data"
        )
    if len(data) > MAX_DATA_LEN:
        raise ValidationError(
            f"len(data)={len(data)} exceeds MAX_DATA_LEN={MAX_DATA_LEN}", "data"
        )
    if dlc is not None and len(data) != dlc:
        raise ValidationError(f"len(data)={len(data)} must equal dlc={dlc}", "data")
    return bytes(data)


def _bits_of(value: int, width: int) -> list[int]:
    return [(value >> (width - 1 - i)) & 1 for i in range(width)]


def _bits_to_int(bits: list[int]) -> int:
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


# ---- 帧数据结构 ----------------------------------------------------------


@dataclass(frozen=True)
class Frame:
    """一帧的发送意图：11 位 ID + 0..8 字节数据。DLC 即 len(data)。"""

    identifier: int
    data: bytes = b""

    def __post_init__(self) -> None:
        check_int(self.identifier, "identifier", 0, ID_MAX)
        # frozen dataclass 里校验/规范化 bytes
        if not isinstance(self.data, (bytes, bytearray)):
            raise ValidationError(
                f"data must be bytes/bytearray, got {type(self.data).__name__}",
                "data",
            )
        if len(self.data) > MAX_DATA_LEN:
            raise ValidationError(
                f"len(data)={len(self.data)} exceeds MAX_DATA_LEN={MAX_DATA_LEN}",
                "data",
            )
        object.__setattr__(self, "data", bytes(self.data))

    @property
    def dlc(self) -> int:
        return len(self.data)


# ---- CRC ----------------------------------------------------------------


def payload_bits(identifier: int, dlc: int, data: bytes | bytearray) -> list[int]:
    """未填充的受保护位串：ID(11) + DLC(4) + 数据（字节高位先发）。"""
    identifier = check_int(identifier, "identifier", 0, ID_MAX)
    dlc = check_int(dlc, "dlc", 0, MAX_DATA_LEN)
    _validate_data(data, dlc)
    bits = _bits_of(identifier, ID_BITS)
    bits += _bits_of(dlc, DLC_BITS)
    for byte in data:
        bits += _bits_of(byte, 8)
    return bits


def compute_crc15(identifier: int, dlc: int, data: bytes | bytearray) -> int:
    """题面 CRC：(ID+DLC+数据 位串) 左移 15 位后除 POLYNOMIAL 的余数。

    实现为 MSB-first 的 GF(2) 多项式长除法：先逐位喂入消息位，再喂入
    15 个 0（等价于位串左移 15 位）。初值 0，无输入/输出反射与异或。
    """
    msg = payload_bits(identifier, dlc, data)
    # MSB-first 多项式除法 LFSR：输入位从寄存器低端移入；移出的最高位
    # 为 1 时减去（异或）生成多项式低 15 位。喂完消息后再喂 15 个 0，
    # 等价于题面的“位串左移 15 位后取余”。初值 0，无反射/异或。
    remainder = 0
    for bit in [*msg, *([0] * CRC_BITS)]:
        top = (remainder >> (CRC_BITS - 1)) & 1
        remainder = ((remainder << 1) | bit) & CRC_MASK
        if top:
            remainder ^= POLYNOMIAL & CRC_MASK
    return remainder


# ---- 位填充 --------------------------------------------------------------


def bit_stuff_mapped(
    bits: list[int] | tuple[int, ...],
) -> tuple[list[int], list[int]]:
    """位填充并同时返回填充位标记（1=该输出位为插入的填充位）。

    填充位开启新的同值计数段（镜像标准 CAN：插入后计数重置为 1）。
    """
    out: list[int] = []
    smap: list[int] = []
    run = 0
    last = -1
    for i, bit in enumerate(bits):
        if isinstance(bit, bool) or not isinstance(bit, int) or bit not in (0, 1):
            raise ValidationError(
                f"bit at index {i} must be int 0/1 (bool rejected)", "bits"
            )
        out.append(bit)
        smap.append(0)
        run = run + 1 if bit == last else 1
        last = bit
        if run == 5:
            out.append(1 - bit)
            smap.append(1)
            last = 1 - bit
            run = 1
    return out, smap


def bit_stuff(bits: list[int] | tuple[int, ...]) -> list[int]:
    """CAN 式位填充：每连续 5 个同位后插入一个反位。"""
    return bit_stuff_mapped(bits)[0]


@dataclass(frozen=True)
class DestuffResult:
    """去填充结果。

    bits:        去掉填充位后的原始位串
    stuffing_map: 与填充后流等长；1 表示该位是填充位
    error_index: 首个填充规则违例的填充后流索引；流尾缺填充位时取流长度；
                 无错误为 None
    """

    bits: list[int]
    stuffing_map: list[int]
    error_index: int | None


def destuff_bits(stuffed: list[int] | tuple[int, ...]) -> DestuffResult:
    """逆位填充。每 5 个同位后的下一位必须存在且为反位，否则记录违例。

    违例后仍按“该位是填充位”将其删去并以其线上实际电平重新起算，尽量
    多解码（首个违例位置已记录，严格调用方据此报错即可）。
    """
    out: list[int] = []
    smap: list[int] = []
    run = 0
    last = -1
    expect_stuff = False
    error_index: int | None = None

    for i, bit in enumerate(stuffed):
        if isinstance(bit, bool) or not isinstance(bit, int) or bit not in (0, 1):
            raise ValidationError(
                f"bit at index {i} must be int 0/1 (bool rejected)", "stuffed"
            )
        if expect_stuff:
            smap.append(1)
            if bit != 1 - last and error_index is None:
                error_index = i
            last = bit  # 以线上实际电平起算新段
            run = 1
            expect_stuff = False
            continue
        out.append(bit)
        smap.append(0)
        run = run + 1 if bit == last else 1
        last = bit
        if run == 5:
            expect_stuff = True

    if expect_stuff and error_index is None:
        error_index = len(stuffed)
    return DestuffResult(bits=out, stuffing_map=smap, error_index=error_index)


# ---- 整帧编码 ------------------------------------------------------------


@dataclass(frozen=True)
class EncodedFrame:
    """帧的编码产物（发送方视角）。

    raw_bits:    ID+DLC+data（未填充受保护位串）
    crc:         CRC15 余数
    crc_bits:    CRC 的 15 个位（高位先发）
    stuffed:     ID..CRC 填充流（bit_stuff(raw_bits+crc_bits)）
    stuffing_map:与 stuffed 等长，1 标记填充位
    stream:      线上完整发送序列 SOF + stuffed + ACK(释放=1) + EOF(1)
    ack_index/eof_index: stream 中的槽位索引
    id_field_indices: stream 中属于 11 个 ID 原始位的线上索引集合
    """

    frame: Frame
    crc: int
    raw_bits: list[int]
    crc_bits: list[int]
    stuffed: list[int]
    stuffing_map: list[int]
    stream: list[int]
    ack_index: int
    eof_index: int
    id_field_indices: frozenset[int] = _dc_field(default_factory=frozenset)

    @property
    def stuffed_start(self) -> int:
        return 1  # SOF 之后


def encode_frame(frame: Frame) -> EncodedFrame:
    """把 Frame 编码为填充后的线上位流（ACK 槽按发送方释放处理）。"""
    if not isinstance(frame, Frame):
        raise ValidationError(
            f"frame must be Frame, got {type(frame).__name__}", "frame"
        )
    crc = compute_crc15(frame.identifier, frame.dlc, frame.data)
    return _encode_with_crc(frame, crc)


def encode_frame_with_crc(frame: Frame, crc: int) -> EncodedFrame:
    """故障注入用编码：以显式给定的 CRC 值替代真实计算结果。

    用于在总线上构造「填充结构合法但 CRC 校验失败」的帧（接收端
    decode_frame 报 crc 错误 / crc_ok=False）。除 CRC 字段外与
    encode_frame 完全一致；不属于正常发送路径，正常发送请用 encode_frame。
    """
    if not isinstance(frame, Frame):
        raise ValidationError(
            f"frame must be Frame, got {type(frame).__name__}", "frame"
        )
    crc = check_int(crc, "crc", 0, CRC_MASK)
    return _encode_with_crc(frame, crc)


def _encode_with_crc(frame: Frame, crc: int) -> EncodedFrame:
    raw = payload_bits(frame.identifier, frame.dlc, frame.data)
    crc_bits = _bits_of(crc, CRC_BITS)
    stuffed, smap = bit_stuff_mapped([*raw, *crc_bits])

    ack_index = 1 + len(stuffed)
    eof_index = ack_index + 1
    stream = [SOF, *stuffed, ACK_RELEASED, EOF]

    id_indices: set[int] = set()
    raw_no = 0
    for off, is_stuff in enumerate(smap):
        if not is_stuff and raw_no < ID_BITS:
            id_indices.add(1 + off)
        if not is_stuff:
            raw_no += 1

    return EncodedFrame(
        frame=frame,
        crc=crc,
        raw_bits=raw,
        crc_bits=crc_bits,
        stuffed=stuffed,
        stuffing_map=smap,
        stream=stream,
        ack_index=ack_index,
        eof_index=eof_index,
        id_field_indices=frozenset(id_indices),
    )


# ---- 整帧解码 ------------------------------------------------------------


@dataclass(frozen=True)
class DecodedFrame:
    frame: Frame
    crc: int
    crc_ok: bool
    raw_bits: list[int]


def decode_frame(
    stuffed: list[int] | tuple[int, ...], *, raise_on_crc: bool = True
) -> DecodedFrame:
    """对 ID..CRC 填充流整帧解码：去填充 -> 切字段 -> CRC 校验。

    填充规则违例、DLC>8（本契约不支持）、长度不自洽抛 FrameDecodeError；
    raise_on_crc=True 时 CRC 不匹配同样抛出，False 时返回 crc_ok=False。
    bit_index 一律以“填充后流”的 0 基索引标定。
    """
    if not isinstance(stuffed, (list, tuple)):
        raise ValidationError(
            f"stuffed must be list/tuple of bits, got {type(stuffed).__name__}",
            "stuffed",
        )
    result = destuff_bits(stuffed)
    if result.error_index is not None:
        raise FrameDecodeError(
            "stuff",
            result.error_index,
            "expected opposite bit after five equal bits",
        )
    raw = result.bits
    if len(raw) < ID_BITS + DLC_BITS:
        raise FrameDecodeError(
            "length",
            max(len(stuffed) - 1, 0),
            f"stream truncated before DLC: {len(raw)} raw bits",
        )
    identifier = _bits_to_int(raw[:ID_BITS])
    dlc = _bits_to_int(raw[ID_BITS : ID_BITS + DLC_BITS])
    if dlc > MAX_DATA_LEN:
        # 定位到 DLC 字段最后一个原始位对应的填充后索引
        dlc_last_raw = ID_BITS + DLC_BITS - 1
        bit_index = _raw_index_to_stuffed(result.stuffing_map, dlc_last_raw)
        raise FrameDecodeError(
            "protocol", bit_index, f"decoded DLC={dlc} exceeds contract max 8"
        )
    expected_len = ID_BITS + DLC_BITS + 8 * dlc + CRC_BITS
    if len(raw) != expected_len:
        raise FrameDecodeError(
            "length",
            len(stuffed) - 1,
            f"raw length {len(raw)} != expected {expected_len} for DLC={dlc}",
        )
    data_bits = raw[ID_BITS + DLC_BITS : ID_BITS + DLC_BITS + 8 * dlc]
    data = bytes(
        _bits_to_int(data_bits[i : i + 8]) for i in range(0, len(data_bits), 8)
    )
    crc = _bits_to_int(raw[-CRC_BITS:])
    crc_ok = crc == compute_crc15(identifier, dlc, data)
    if not crc_ok and raise_on_crc:
        raise FrameDecodeError(
            "crc",
            len(stuffed) - 1,
            f"CRC mismatch: got 0x{crc:04X}, expected "
            f"0x{compute_crc15(identifier, dlc, data):04X}",
        )
    return DecodedFrame(
        frame=Frame(identifier=identifier, data=data),
        crc=crc,
        crc_ok=crc_ok,
        raw_bits=raw,
    )


def _raw_index_to_stuffed(stuffing_map: list[int], raw_index: int) -> int:
    """第 raw_index 个原始位（0 基）在填充后流中的索引。"""
    raw_no = -1
    for i, is_stuff in enumerate(stuffing_map):
        if not is_stuff:
            raw_no += 1
            if raw_no == raw_index:
                return i
    return len(stuffing_map) - 1
