"""异常类型与错误语义。

所有由本库主动抛出的异常都继承 CanError，便于调用方统一捕获。
非法输入一律拒绝（不静默纠正、不做隐式截断或类型转换）。
"""

from __future__ import annotations


class CanError(Exception):
    """本库所有自定义异常的基类。"""


class ValidationError(CanError):
    """构造/调用参数违反契约（范围、类型、NaN/Infinity、bool 伪装 int 等）。

    message 中以 ``field`` 名称定位出错参数。
    """

    def __init__(self, message: str, field: str | None = None) -> None:
        self.field = field
        if field is not None and "[" not in message:
            message = f"{message} (field={field!r})"
        super().__init__(message)


class FrameDecodeError(CanError):
    """位流解码失败。

    bit_index 为出错时的**填充后**流位置；kind 为机器可读错误类别：
    "stuff"（违反 5 同位置必有反位的填充规则）、"crc"（CRC 不匹配）。
    """

    def __init__(self, kind: str, bit_index: int, detail: str = "") -> None:
        self.kind = kind
        self.bit_index = bit_index
        text = f"{kind} error at stuffed-stream bit index {bit_index}"
        if detail:
            text += f": {detail}"
        super().__init__(text)


class BusFinishedError(CanError):
    """总线传输已结束（正常完成或已发出错误帧），再调用 step() 即非法。"""
