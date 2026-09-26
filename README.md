# canbit — CAN 合成位流的仲裁与错误约束（本题契约子集）

一个零依赖的离线 Python 题库，对**有限定义**的 CAN 风格总线做逐 bit 建模：
帧编码、CRC15、位填充、wired-AND 仲裁、位错误、显式 ACK 槽与错误帧中止；
另提供**边沿时间输入入口**：按位定时参数（传播段/相位段/SJW）计算采样点，
在隐性到显性边沿做帧起始硬同步与帧内有界重同步，恢复位流后交给同一套
去填充与帧检查。

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
# 必要测试（55 个用例，实测 < 0.1s）
python -m unittest discover -s tests -v

# 固定输入演示（实测约 0.04s；无 sleep、无预录结果）
python demo.py

# 边沿定时接收入口演示（实测约 0.04s，远小于 90s 预算；真实计算正常
# 与边界结果，无 sleep、无预录输出）
python demo_iteration.py
```

在仓库根目录执行；`python` 指向 3.14.7 解释器即可。如需隔离环境：
`python -m venv .venv && . .venv/bin/activate`（无第三方依赖，venv 可选）。

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

边沿时间输入入口（本轮新增，与逐 bit 入口并存）：

```python
from canbit import (
    BitTiming,            # BitTiming(prop_seg, phase_seg1, phase_seg2, sjw)
    Edge, Glitch,         # 边沿 / 毛刺（均为整数 tq 参数）
    SampleRecord, PhaseCorrection, ReceptionResult,
    receive_edges,        # (edges, timing) -> ReceptionResult
    simulate_edges,       # (bits, timing, drift_num=, drift_den=, glitches=) -> [Edge]
    MAX_SAMPLES,
)

timing = BitTiming(prop_seg=3, phase_seg1=3, phase_seg2=3, sjw=2)
timing.tq_per_bit      # 10（同步段 1 + 3 + 3 + 3）
timing.sample_point    # 7（采样点相对位起始的 tq 偏移）

# 发送侧（测试/演示用）：理想位流 + 确定性时钟偏差 + 短毛刺 -> 边沿序列
edges = simulate_edges(enc.stream, timing, drift_num=1, drift_den=8,
                       glitches=[(22, 0, 2)])

# 接收侧：只给边沿时间与位定时参数，不预先知道原始位串
res = receive_edges(edges, timing)
res.status          # "ok" | "crc_error" | "stuff_error" | "length_error"
                    # | "protocol_error" | "no_frame" | "incomplete"
res.samples         # 每次采样 SampleRecord(bit_index, time, bit)
res.corrections     # 每次相位调整 PhaseCorrection(kind/edge_time/phase_error/
                    # applied/clamped)；kind="hard_sync"（帧起始）与
                    # "resync"（帧内，|applied|<=sjw）分开记录
res.bits            # 恢复出的位流（SOF 起，含 ACK/EOF）
res.frame, res.crc_ok, res.error_bit_index
```

边沿接收契约：

* 每位 = 1（同步段）+ prop_seg + phase_seg1 + phase_seg2 个整数 tq；
  采样点由参数计算，接收方不预先知道原始位串。
* 空闲时首个隐性到显性边沿做**硬同步**（SOF 起点，SOF 采样必须为显性）；
  帧内只在隐性到显性边沿做**有界**相位调整：采样点之前出现的边沿视为
  上一位边界来晚（本位加长），之后出现的视为下一位边界来早（本位缩短），
  调整幅度截断到 ±sjw 并记录 clamped。
* 恢复位流按 SOF + 填充流 + ACK + EOF 切分，填充流交给既有
  `decode_frame` 去填充并校验 CRC；终止于帧尾成功解码、填充规则违例、
  6 连续显性采样、或无后续边沿后的持续隐性（损坏帧）。
* `simulate_edges` 的时钟偏差是有理数：每 drift_den 位累计 drift_num
  个 tq（整数累加器摊派，可正可负）；毛刺为 (time, level, duration)。
* 全模块纯整数运算，无浮点。

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
  过载帧、帧间隙。
* 边沿定时接收是明确定义的合成模型，不宣称完整控制器兼容：无振荡器
  容差/抖动模型、无多次采样多数判决、无错误界定符与错误计数、无
  发送延迟补偿；硬同步只发生在空闲后的首个隐性到显性边沿。
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
canbit/timing.py       # 边沿时间输入：位定时/硬同步/有界重同步/发送方仿真
tests/test_framing.py  # CRC、填充、编解码测试（对照独立参考）
tests/test_bus.py      # 仲裁/错误/ACK/契约/差分对拍测试
tests/test_timing.py   # 边沿接收：理想/可修复偏差/超界偏差/毛刺/校验
tests/reference.py     # 独立参考实现（不 import canbit）
demo.py                # 固定输入演示（逐 bit 总线模型）
demo_iteration.py      # 边沿定时接收演示（正常 + 边界结果）
```
