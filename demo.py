"""demo.py —— 固定输入的真实计算演示（离线、无 sleep、无预录结果）。

演示内容：
  1. 正常结果：两个不同 ID 的发送者仲裁，真实逐 bit 算出胜者与解码帧。
  2. 协议错误：同 ID 不同负载 -> bit_error，打印出错槽位与参与者。
  3. 协议错误：ACK 槽无人拉低 -> ack_error。
  4. 拒绝边界：非法输入（bool 充当整数 ID）被可定位地拒绝。
"""

import canbus
from canbus import BusSimulation, Frame, simulate


def show_normal_arbitration():
    print("== 1. 正常仲裁：ID 0x2A0 对 0x2A1 ==")
    frames = [Frame(0x2A0, b"\x11\x22\x33"), Frame(0x2A1, b"\x44\x55")]
    sim = BusSimulation(frames, ack_pull=True)
    while not sim.done:
        sim.step()
    result = sim.result
    print(f"  总槽位数: {len(result.events)}，胜者: 发送者 {result.winners}")
    print(f"  解码帧: id=0x{result.frame.id:03X} data={result.frame.data.hex()}")
    withdraw = next(ev.slot for ev in result.events if ev.sent[1] is None)
    print(f"  发送者 1 在槽 {withdraw - 1}（ID 域）仲裁失败退出，无错误记录")
    print(f"  ok={result.ok} ack={result.ack}")


def show_bit_error():
    print("== 2. 协议错误：同 ID 不同负载 ==")
    result = simulate([Frame(0x2A0, b"\x00"), Frame(0x2A0, b"\x80")],
                      ack_pull=True)
    err = result.error
    print(f"  ok={result.ok} 错误: {err.kind} @ 槽 {err.slot}（{err.field} 域）")
    print(f"  参与者: {err.participants}；{err.detail}")


def show_ack_error():
    print("== 3. 协议错误：ACK 槽无人拉低 ==")
    result = simulate([Frame(0x123, b"\xAB")], ack_pull=False)
    err = result.error
    print(f"  ok={result.ok} 错误: {err.kind} @ 槽 {err.slot}；{err.detail}")


def show_rejection():
    print("== 4. 拒绝边界：bool 充当整数 ID ==")
    try:
        Frame(True, b"\x00")
    except TypeError as exc:
        print(f"  触发 TypeError: {exc}")


if __name__ == "__main__":
    print(f"canbus 模块演示（Python 标准库，多项式 0x{canbus.CRC_POLY:04X}）")
    show_normal_arbitration()
    show_bit_error()
    show_ack_error()
    show_rejection()
