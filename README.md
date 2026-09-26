# canbit — CAN 合成位流的仲裁与错误约束（本题契约子集）

一个零依赖的离线 Python 题库，对**有限定义**的 CAN 风格总线做逐 bit 建模：
帧编码、CRC15、位填充、wired-AND 仲裁、位错误、显式 ACK 槽与错误帧中止。

> **本库只实现题面契约定义的有限协议子集，不宣称兼容完整 CAN 标准**
> （无 RTR/IDE/控制场其余位、无重传、无错误界定/被动错误计数、无位时序
> 与同步、无错误界定符/帧间隙等）。

## 运行环境

* Python **3.14.7**（仅使用标准库；在 3.10+ 上语法也兼容，但以 3.14.7 为准）
* Linux 原生运行，**无需安装任何第三方依赖**，无需网络/账号/密钥/硬件/
  数据库/中间件/云 API/Docker
* 核心、测试、demo 全部离线、确定性

## 准确命令

```bash
# 必要测试（53 个用例，实测 < 0.1s）
python -m unittest discover -s tests -v

# 位流层演示（实测约 0.04s；无 sleep、无预录结果）
python demo.py

# 传输层演示：正常 + 4 个边界，真实计算（实测约 0.05s，远小于 90s 预算）
python demo_iteration.py
```

在仓库根目录执行；`python` 指向 3.14.7 解释器即可。

venv 准备（可选，库只用标准库，venv 内无需安装任何依赖）：

```bash
python3.14 -m venv .venv
.venv/bin/python -m unittest discover -s tests -v
```

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

## 公开接口

```python
from canbit import (
    # 数据结构
    Frame,                # Frame(identifier:int, data:bytes)；frozen，DLC=len(data)
    EncodedFrame,         # encode_frame 的产物（raw_bits/crc/stuffed/stream/映射）
    DecodedFrame,         # decode_frame 的产物（frame/crc/crc_ok）
    Sender,               # Sender(name:str, frame:Frame)
    BitBus, Event, StepResult, ErrorFrameInfo, BusResult,
    # 纯函数
    compute_crc15,        # (identifier, dlc, data) -> int 0..0x7FFF
    payload_bits,         # (identifier, dlc, data) -> list[int]
    bit_stuff, destuff_bits,   # 位填充 / 逆填充（DestuffResult 带违例索引）
    encode_frame, decode_frame,
    # 常量
    ID_BITS, DLC_BITS, CRC_BITS, MAX_DATA_LEN, POLYNOMIAL,
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

## 传输层（本轮新增，本题自定义小子集）

在 `Frame` 之上提供显式分段传输：首帧声明总字节数，连续帧带模 16 序号，
接收端用流控帧给出块大小与最小发送间隔。**不宣称覆盖 ISO-TP 等车规标准。**

### 帧格式（承载于 Frame.data，DLC ≤ 8，第 1 字节为 PCI）

| 帧 | PCI | 格式 |
|---|---|---|
| 首帧 FF | 0x1 | `0x10|(total>>8), total&0xFF, 前 6 数据字节`；total ∈ 1..4095（12 位） |
| 连续帧 CF | 0x2 | `0x20|(seq&0x0F), 至多 7 数据字节`；seq 从 1 起模 16 递增 |
| 流控帧 FC | 0x3 | `0x30|fs, block_size, st_min`；fs：0=CTS / 1=WAIT / 2=OVFLW |

`block_size`：收到该数量 CF 后接收端回发新 FC（0 = 不限）；
`st_min`：相邻 CF 的最小发送间隔，单位为逻辑时钟 tick（本层不读真实
时钟，时间完全由调用方以整数 tick 驱动，无 sleep）。

### 语义

* 接收端只在全部字节到齐时一次性交付；任何失败整次作废，绝不交付部分消息。
* 序号跳跃（含丢帧）、超长首帧（total > max_message）、会话内格式变化
  （接收中途来新首帧 / 空闲时收到 CF、FC）均使整次接收失败并清空缓存；
  新首帧不会无声覆盖正在接收的数据。
* 流控 WAIT 最多连续接受 `max_waits` 次（默认 3），超过发送方中止。
* 逻辑时钟超过 `timeout` tick 没有下一帧，接收端清理缓存并报超时。

### 接口

```python
from canbit import (
    TransportSender, Receiver, ReceiveResult, TransportError,
    make_first_frame, make_consecutive_frame, make_flow_control_frame,
    parse_pci, PCI_FF, PCI_CF, PCI_FC, FS_CTS, FS_WAIT, FS_OVFLW,
    FF_DATA_BYTES, CF_DATA_BYTES, SEQ_MOD, MAX_MESSAGE_BYTES,
)

sender = TransportSender(0x700, payload)          # 1..4095 字节
receiver = Receiver(0x708, block_size=4, st_min=2, timeout=100)
ff = sender.start(tick=0)                          # 首帧
res = receiver.on_frame(ff, 0)                     # ReceiveResult(flow_frames, delivered)
sender.on_frame(res.flow_frames[0], 0)             # 处理流控
cf = sender.next_frame(tick)                       # 受块大小/st_min 约束，未就绪为 None
receiver.on_tick(tick)                             # 推进逻辑时钟（超时检查）
```

失败抛 `TransportError`，`kind` ∈ `sequence / oversize / format /
timeout / wait_exceeded / abort`；非法参数仍抛 `ValidationError`。
测试与 `demo_iteration.py` 均用本地事件序列驱动两端，不经过真实总线，
最终交付与固定生成的原始字节串逐字节独立核对。

## 输入上限与错误语义

* ID ∈ [0, 2047]；DLC ∈ [0, 8]；数据长度必须等于 DLC（直接调用 CRC/payload
  函数时）或 ≤ 8（`Frame` 自动以长度为 DLC）。
* 整数参数拒绝 `bool`（`True/False` 不隐式当 1/0）；拒绝 `float`，其中
  **NaN/+∞/−∞ 单独给明确报错文案**；非 int/非 bytes 类型拒绝。
* 非法输入立即抛 `ValidationError`（消息带 `field=...` 可定位），**不静默
  纠正、不截断、不隐式转换**。
* `BitBus` 为一次性本轮传输对象：结束后再 `step()` 抛 `BusFinishedError`；
  未结束调 `result()` 抛 `ValidationError`。构造期完成全部校验，运行期不会
  因输入问题产生半完成状态；`run()` 要么得到完整结果，要么得到带
  `ErrorFrameInfo` 的完整错误帧记录，不存在未说明的部分变更。
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
   错误界定符、错误计数与重传。
6. **可审计**：每个位时间产生一条 `Event`；错误位置同时给出填充后流
   0 基索引，便于和 `EncodedFrame.stuffing_map` 对照定位。

## 已知限制

* 不是完整 CAN/CAN FD：无 RTR、IDE、保留位、控制场其余编码、远程帧、
  过载帧、帧间隙、位时序/同步、重同步跳转宽度。
* 无重传、无主动/被动错误/总线关闭状态机、无错误界定符与错误计数。
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
canbit/transport.py    # 传输层：FF/CF/FC、发送端、接收端
tests/test_framing.py  # CRC、填充、编解码测试（对照独立参考）
tests/test_bus.py      # 仲裁/错误/ACK/契约/差分对拍测试
tests/test_transport.py # 传输层：序号回绕/分块暂停/丢帧/超时等
tests/reference.py     # 独立参考实现（不 import canbit）
demo.py                # 位流层固定输入演示
demo_iteration.py      # 传输层演示（正常 + 边界，真实计算）
```
