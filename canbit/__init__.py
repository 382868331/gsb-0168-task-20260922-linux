"""canbit: 有限 CAN 合成位流模型的仲裁与错误约束（本题契约子集）。

本库只实现题目定义的有限协议子集：11 位 ID、4 位 DLC(0..8)、
0..8 字节数据、15 位 CRC（多项式 0x4599，初值 0）、ID..CRC 区段的
位填充、独立 ACK 槽、逐 bit 的总线仲裁/错误帧模型，以及节点级
固定接收过滤表（(frame_id & mask) == (id & mask)，过滤只决定上交，
不影响 ACK 与 CRC 校验）。

不宣称兼容完整 CAN 标准。
"""

from .errors import (
    BusFinishedError,
    CanError,
    FrameDecodeError,
    ValidationError,
)
from .framing import (
    CRC_BITS,
    CRC_MASK,
    DLC_BITS,
    ID_BITS,
    MAX_DATA_LEN,
    POLYNOMIAL,
    DecodedFrame,
    DestuffResult,
    EncodedFrame,
    Frame,
    bit_stuff,
    compute_crc15,
    decode_frame,
    destuff_bits,
    encode_frame,
    encode_frame_with_crc,
    payload_bits,
)
from .bus import (
    BitBus,
    BusResult,
    ErrorFrameInfo,
    Event,
    Sender,
    StepResult,
)
from .nodes import (
    BusNode,
    FilterRule,
    NodeBus,
    NodeBusResult,
    ReceiveRecord,
)

__all__ = [
    # 常量
    "ID_BITS",
    "DLC_BITS",
    "CRC_BITS",
    "CRC_MASK",
    "MAX_DATA_LEN",
    "POLYNOMIAL",
    # 数据结构
    "Frame",
    "EncodedFrame",
    "DestuffResult",
    "Sender",
    "Event",
    "StepResult",
    "ErrorFrameInfo",
    "BusResult",
    "FilterRule",
    "BusNode",
    "ReceiveRecord",
    "NodeBusResult",
    # 编解码
    "compute_crc15",
    "payload_bits",
    "bit_stuff",
    "destuff_bits",
    "encode_frame",
    "encode_frame_with_crc",
    "decode_frame",
    # 总线
    "BitBus",
    "NodeBus",
    # 异常
    "CanError",
    "ValidationError",
    "FrameDecodeError",
    "BusFinishedError",
]
