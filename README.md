# canbit — CAN 合成位流的仲裁、错误约束与 TEC/REC 计数（本题契约子集）

一个零依赖的离线 Python 题库，对**有限定义**的 CAN 风格总线做逐 bit 建模：
帧编码、CRC15、位填充、wired-AND 仲裁、位错误、显式 ACK 槽、错误帧中止，
以及跨轮累积的 TEC/REC 错误计数（主动/被动/离线状态）。

> **本库只实现题面契约定义的有限协议子集，不宣称兼容完整 CAN 标准**
> （无 RTR/IDE/控制场其余位、无重传、无错误位流叠加、无被动错误标志
> 区分、无位时序与同步、无错误界定符/帧间隙等）。

## 运行环境

* Python **3.14.7**（仅使用标准库；在 3.10+ 上语法也兼容，但以 3.14.7 为准）
* Linux 原生运行，**无需安装任何第三方依赖**，无需网络/账号/密钥/硬件/
  数据库/中间件/云 API/Docker
* 核心、测试、demo 全部离线、确定性

## 准确命令

```bash
# 必要测试（58 个用例，实测 < 0.1s）
python -m unittest discover -s tests -v

# 固定输入演示（实测约 0.08s，远小于 90s 预算；无 sleep、无预录结果）
python demo.py
```

在仓库根目录执行；`python` 指向 3.14.7 解释器即可。

## 协议契约（本题）

| 项 | 定义 |
|---|---|
| ID | 11 位无符号整数，0..0x7FF，高位先发 |
| DLC | 4 位，取值 **0..8**，语义即数据字节数（>8 拒绝） |
| 数据 | DLC 字节，每字节高位先发 |
| CRC | 15 位。把 `ID(11)+DLC(4)+数据` 位串**左移 15 位**后对生成多项式取余；多项式 `0x4599`（x¹⁵+x¹⁴+x¹⁰+x⁸+x⁷+x⁴+x³+1），初值 0，无输入/输出反射与异或；CRC 高位先发 |
| 位填充 | 从 ID 到 CRC 的连续位流中，每连续 5 个同位插入 1 个反位；**填充位本身计入新游程**（与线上解码一致，保证填充后流不出现 6 连同值） |
| 帧线序 | `SOF(固定0)` + 填充后的 `ID..CRC` + `ACK 独立槽` + `EOF(固定1)` |
| 总线 | 显性 0 胜隐性 1，总线位 = 所有驱动者按位与 |
| 仲裁 | 多个发送者在 **ID 域原始位**（非填充位）“发 1 读 0”仅退出仲裁、自该位起停止驱动；其他位置任何方向失配记该参与者 `bit_error` |
| ACK | 所有发送者在 ACK 槽释放（发 1）；总线是否被拉低**完全由显式参数 `ack_pullers` 决定**；无任何接收者拉低则仍在发送者报 `ack_error` |
| 错误帧 | 检测到错误的那个位时间即中止当前传输，检测者随后发 **6 位显性错误标志**；`ErrorFrameInfo` 记录触发类型、位索引、检测者与被中止的全部参与者 |
| CRC 检查时机 | 接收/解码视角先去填充，再切字段，再校验 CRC；填充错误先于 CRC 报告 |
| 错误计数 | 发送错误 TEC+8、接收错误 REC+1；相应成功各 −1（到 0 止）。任一计数 ≥128 进入被动，TEC ≥256 离线；仲裁丢失不加错；离线禁止发送；`Node.reset()` 显式清零 |

## 公开接口

```python
from canbit import (
    # 数据结构
    Frame,                # Frame(identifier:int, data:bytes)；frozen，DLC=len(data)
    EncodedFrame,         # encode_frame 的产物（raw_bits/crc/stuffed/stream/映射）
    DecodedFrame,         # decode_frame 的产物（frame/crc/crc_ok）
    Sender,               # Sender(name:str, frame:Frame)
    Node,                 # Node(name:str, *, tec=0, rec=0)：持久参与者的 TEC/REC
    BitBus, Event, StepResult, ErrorFrameInfo, BusResult,
    # 纯函数
    compute_crc15,        # (identifier, dlc, data) -> int 0..0x7FFF
    payload_bits,         # (identifier, dlc, data) -> list[int]
    bit_stuff, destuff_bits,   # 位填充 / 逆填充（DestuffResult 带违例索引）
    encode_frame, decode_frame,
    # 常量
    ID_BITS, DLC_BITS, CRC_BITS, MAX_DATA_LEN, POLYNOMIAL,
    ERROR_PASSIVE_THRESHOLD,  # 128：TEC 或 REC 达到即被动
    BUS_OFF_THRESHOLD,        # 256：TEC 达到即离线（仅 TEC）
    TRANSMIT_ERROR_PENALTY,   # 8     RECEIVE_ERROR_PENALTY,  # 1
    SUCCESS_CREDIT,           # 1（相应成功减分，到 0 止）
)
from canbit.errors import (
    CanError, ValidationError, FrameDecodeError, BusFinishedError,
)
```

逐 bit 用法：

```python
bus = BitBus(
    [Sender("A", Frame(0x321, b"hi")), Sender("B", Frame(0x320, b"yo"))],
    ack_pullers={"RX_node"},          # 显式接收者；省略=无人 ACK
    forced_bits={6: 0},               # 可选：位索引->强制电平（确定性故障注入）
    nodes=[Node("A"), Node("B")],     # 可选：挂接 TEC/REC 计数（见下节）
)
while not bus.finished:
    sr = bus.step()        # StepResult(bit_index, phase, bus_bit,
                           #           transmitted, arbitration_losers, bit_errors)
result = bus.result()      # 或直接 bus.run()
result.winners             # ('B',)
result.error_frame         # None 或 ErrorFrameInfo(trigger, bit_index, participants, aborted)
bus.events                 # 每位时间的可审计 Event 序列
```

解码：

```python
dec = decode_frame(encoded.stuffed)                 # 非法即抛 FrameDecodeError
dec = decode_frame(stuffed, raise_on_crc=False)     # CRC 错不抛，返回 crc_ok=False
# FrameDecodeError.kind: "stuff" | "crc" | "length" | "protocol"
# FrameDecodeError.bit_index：填充后流的 0 基索引（可定位）
```

## 错误计数（TEC/REC）

`Node` 是跨多轮 `BitBus` 传输复用的持久参与者；把同一批 `Node` 经
`nodes=` 挂接到每一轮总线上，计数随每轮结果累积（**结果确定时恰好结算
一次**，重复取 `result()` 不会重复计分）：

```python
tx, rx = Node("ECU_TX"), Node("ECU_RX")     # 可用 tec=/rec= 指定初始计数
BitBus([Sender("ECU_TX", Frame(0x123, b"x"))], nodes=[tx, rx]).run()
# 无人 ACK -> ack_error：发送者 TEC+8，监听者 REC+1
tx.tec, rx.rec      # (8, 1)
tx.state            # "active" | "passive" | "bus_off"（由计数派生）
tx.reset()          # 显式复位：TEC/REC 清零，回到 active
```

一轮传输的折算规则（`nodes=` 省略时不跟踪计数，行为与旧版一致）：

| 一轮结果 | 仍在发送的参与者 | 其余节点（仲裁失败者、纯监听者） |
|---|---|---|
| 正常完成（无错误帧） | 发送成功：TEC −1（到 0 止） | 接收成功：REC −1（到 0 止） |
| 错误帧中止（bit_error / ack_error） | 发送错误：TEC +8 | 接收错误：REC +1 |

* 仲裁丢失本身**不加任何错**；输仲裁的节点随后按接收者参与本轮。
* `state` 由计数派生：TEC ≥ 256 → `bus_off`；否则 TEC 或 REC ≥ 128 →
  `passive`；否则 `active`。被动节点计数回落到 128 以下自动回到主动。
* **离线节点禁止发送**：以其为发送者构造 `BitBus` 立即抛
  `ValidationError`（构造期失败，计数不被触碰）；离线节点列为监听者时
  不参与总线、不计数；只能经 `reset()` 恢复。
* `nodes` 校验：必须是 `Node` 实例、名字唯一、覆盖全部发送者，否则
  构造期抛 `ValidationError`（`field='nodes'`）。

## 输入上限与错误语义

* ID ∈ [0, 2047]；DLC ∈ [0, 8]；数据长度必须等于 DLC（直接调用 CRC/payload
  函数时）或 ≤ 8（`Frame` 自动以长度为 DLC）。
* 整数参数拒绝 `bool`（`True/False` 不隐式当 1/0）；拒绝 `float`，其中
  **NaN/+∞/−∞ 单独给明确报错文案**；非 int/非 bytes 类型拒绝。
* 非法输入立即抛 `ValidationError`（消息带 `field=...` 可定位），**不静默
  纠正、不截断、不隐式转换**。`Node` 的名字/初始计数同样校验（bool、
  NaN/Infinity、负数一律拒绝）。
* `BitBus` 为一次性本轮传输对象：结束后再 `step()` 抛 `BusFinishedError`；
  未结束调 `result()` 抛 `ValidationError`。构造期完成全部校验（含
  `nodes` 的重名/缺发送者/离线发送者检查），运行期不会因输入问题产生
  半完成状态；`run()` 要么得到完整结果，要么得到带 `ErrorFrameInfo` 的
  完整错误帧记录，不存在未说明的部分变更。TEC/REC 结算在结果确定时
  恰好应用一次；构造失败不触碰任何计数。
* 本库无“搜索预算/无解”类算法，故不存在预算耗尽与无解混淆的情形。

## 浮点 / 误差口径

本库**没有任何浮点计算与浮点输出**：CRC、位流、索引、仲裁结果全部是整数
或整数序列，结果为**精确定义值，无误差、无容差**。输入侧的浮点（含
NaN/Infinity）一律拒绝，不参与任何数值运算。测试中不存在“宽容差”断言。

## 设计取舍

1. **CRC 双实现交叉验证**：核心用 15 位 MSB-first LFSR 移位（等价“左移 15
   位取余”）；`tests/reference.py` 的独立参考用“大整数左移 + GF(2) 长除法”，
   两者结构不同且参考模块**不 import 被测核心**，期望值不来自被测代码。
2. **填充位计入游程**：`00000` 后插入的 `1` 作为新游程第 1 位，因此
   `00000 11111` 编码为 `00000 1 1111 0 1`。这保证线上流任意窗口不含 6
   连同值，与解码端计数一致。
3. **位监控双向检查**：wired-AND 上“发 0 读 1”不会自然发生，只能由
   `forced_bits` 注入；模型不静默忽略它，同样记 `bit_error`。
4. **ACK 显式化**：不模拟自动应答；接收者是否拉低完全由 `ack_pullers`
   给定，名字可以是任意参与者（仲裁失败者、纯接收者均可）。
5. **错误帧固定 6 位显性标志**：只表达“中止 + 谁报的 + 在哪一位”，不实现
   错误界定符与重传。
6. **TEC/REC 挂在持久 Node 上**：`BitBus` 仍是一次性本轮传输对象，计数
   状态放在跨轮复用的 `Node` 里，经 `nodes=` 注入；结算在传输结果确定的
   那一刻恰好应用一次（错误帧在检测时、成功在 EOF 完成时），不存在
   “计了一半”的中间态。
7. **接收错误按帧粒度计**：本模型接收侧不逐位界定错误，帧被错误帧中止
   即对所有非发送参与者记一次接收错误（REC+1）；被动状态不改变错误
   标志形态（不区分主动/被动错误标志，见已知限制）。
8. **可审计**：每个位时间产生一条 `Event`；错误位置同时给出填充后流
   0 基索引，便于和 `EncodedFrame.stuffing_map` 对照定位。

## 已知限制

* 不是完整 CAN/CAN FD：无 RTR、IDE、保留位、控制场其余编码、远程帧、
  过载帧、帧间隙、位时序/同步、重同步跳转宽度。
* 无重传、无错误位流叠加、无错误界定符；被动节点不区分主动/被动错误
  标志（`state` 只影响计数与发送许可，不改变线上行为）。
* 离线（bus-off）只能经 `Node.reset()` 显式恢复，不实现真实 CAN 的
  128×11 隐性位自动恢复；离线节点不参与总线（不计数、不可发送）。
* 接收错误按“帧被错误帧中止”计一次 REC+1，不做逐位错误界定。
* ACK 槽为 1 位模型，不区分 ACK 槽/界定符；EOF 为 1 位而非 7 位。
* `forced_bits` 是测试/演示用的确定性故障注入钩子，不代表真实收发器行为。
* DLC 仅 0..8；解码出 DLC>8 直接报 `protocol` 错误（不做经典 CAN 的
  DLC≥8 映射）。

## 目录

```
canbit/__init__.py     # 公开导出
canbit/errors.py       # CanError/ValidationError/FrameDecodeError/BusFinishedError
canbit/framing.py      # Frame/CRC15/位填充/整帧编码与解码
canbit/bus.py          # BitBus：逐 bit 仲裁/位错误/ACK/错误帧
canbit/counters.py     # Node：TEC/REC 计数与 主动/被动/离线 状态
tests/test_framing.py  # CRC、填充、编解码测试（对照独立参考）
tests/test_bus.py      # 仲裁/错误/ACK/契约/差分对拍测试
tests/test_counters.py # TEC/REC 计分、阈值、离线禁发、复位测试
tests/reference.py     # 独立参考实现（不 import canbit）
demo.py                # 固定输入演示
```
