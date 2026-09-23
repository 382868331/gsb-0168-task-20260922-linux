"""TEC/REC 错误计数与 主动/被动/离线 状态（本题契约子集）。

题面规则：

* 发送错误：TEC += 8；接收错误：REC += 1
* 发送成功：TEC -= 1（最小到 0）；接收成功：REC -= 1（最小到 0）
* TEC 或 REC 任一达到 128 -> 错误被动（passive）
* TEC 达到 256 -> 离线（bus-off），离线节点禁止发送
* 仲裁丢失不算错误；显式 ``reset()`` 把两个计数清零

一轮 BitBus 传输的结果如何折算到计数（BitBus 在结果确定的那一刻
恰好应用一次，见 ``BitBus._apply_outcome``）：

* 正常完成（无错误帧）：仍在发送的胜者记一次发送成功；其余参与者
  （仲裁失败者、纯监听节点）记一次接收成功。
* 错误帧中止（bit_error / ack_error）：被中止时仍在发送的参与者
  （``ErrorFrameInfo.aborted``）各记一次发送错误；其余参与者各记
  一次接收错误。
* 离线节点不参与总线：作为发送者构造 BitBus 直接被拒绝；列为
  监听节点时不计任何数。

与完整 CAN 的差异（本迭代不扩展）：被动节点不改变错误标志形态
（不区分主动/被动错误标志、不叠加错误位流）；无自动恢复与重传
队列（离线只能显式 reset）；接收错误按“帧被错误帧中止”计一次，
不做逐位错误界定。
"""

from __future__ import annotations

from .errors import ValidationError
from .framing import check_int

# ---- 公开契约常量 -------------------------------------------------------

ERROR_PASSIVE_THRESHOLD = 128  # TEC 或 REC 达到该值 -> 错误被动
BUS_OFF_THRESHOLD = 256        # TEC 达到该值 -> 离线（仅 TEC，REC 不离线）
TRANSMIT_ERROR_PENALTY = 8     # 发送错误加分
RECEIVE_ERROR_PENALTY = 1      # 接收错误加分
SUCCESS_CREDIT = 1             # 相应成功减分（到 0 为止）

STATE_ACTIVE = "active"
STATE_PASSIVE = "passive"
STATE_BUS_OFF = "bus_off"

# 计数构造上限：仅为防御性边界，正常模型内 TEC 不会超过 255+8
_COUNTER_MAX = (1 << 63) - 1


class Node:
    """总线上的持久参与者：名字 + TEC/REC 计数，状态由计数派生。

    Node 跨多轮 ``BitBus`` 传输复用（经 ``BitBus(..., nodes=[...])``
    挂接），计数随每轮结果累积；一轮传输的结算恰好应用一次。
    可用 ``tec=``/``rec=`` 指定初始计数（便于从任意状态开始建模），
    运行中计数只由 BitBus 结算或 ``reset()`` 改变。
    """

    __slots__ = ("_name", "_tec", "_rec")

    def __init__(self, name: str, *, tec: int = 0, rec: int = 0) -> None:
        if not isinstance(name, str) or not name:
            raise ValidationError("node name must be a non-empty str", "name")
        self._name = name
        self._tec = check_int(tec, "tec", 0, _COUNTER_MAX)
        self._rec = check_int(rec, "rec", 0, _COUNTER_MAX)

    @property
    def name(self) -> str:
        return self._name

    @property
    def tec(self) -> int:
        """发送错误计数。"""
        return self._tec

    @property
    def rec(self) -> int:
        """接收错误计数。"""
        return self._rec

    @property
    def state(self) -> str:
        """由计数派生的状态：active | passive | bus_off。

        TEC >= 256 -> bus_off；否则 TEC 或 REC >= 128 -> passive；
        否则 active。被动节点在计数回落到 128 以下时自动回到主动；
        离线只能经 ``reset()`` 恢复（离线期间 TEC 不会变化）。
        """
        if self._tec >= BUS_OFF_THRESHOLD:
            return STATE_BUS_OFF
        if self._tec >= ERROR_PASSIVE_THRESHOLD or self._rec >= ERROR_PASSIVE_THRESHOLD:
            return STATE_PASSIVE
        return STATE_ACTIVE

    def reset(self) -> None:
        """显式复位：TEC/REC 清零，状态回到主动。"""
        self._tec = 0
        self._rec = 0

    # ---- 由 BitBus 在传输结果确定时调用（包内契约，非公开入口） --------

    def _note_transmit_error(self) -> None:
        self._tec += TRANSMIT_ERROR_PENALTY

    def _note_receive_error(self) -> None:
        self._rec += RECEIVE_ERROR_PENALTY

    def _note_transmit_success(self) -> None:
        self._tec = max(0, self._tec - SUCCESS_CREDIT)

    def _note_receive_success(self) -> None:
        self._rec = max(0, self._rec - SUCCESS_CREDIT)

    def __repr__(self) -> str:
        return (
            f"Node({self._name!r}, tec={self._tec}, rec={self._rec}, "
            f"state={self.state!r})"
        )
