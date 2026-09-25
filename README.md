# canbit — CAN 合成位流的仲裁与错误约束（本题契约子集）

一个零依赖的离线 Python 题库，对**有限定义**的 CAN 风格总线做逐 bit 建模：
帧编码、CRC15、位填充、wired-AND 仲裁、位错误、显式 ACK 槽、错误帧中止，
以及节点级**固定接收过滤表**（过滤只决定 payload 是否上交，不影响 ACK
与 CRC 校验）。

> **本库只实现题面契约定义的有限协议子集，不宣称兼容完整 CAN 标准**
> （无 RTR/IDE/控制场其余位、无重传、无错误界定/被动错误计数、无位时序
> 与同步、无错误界定符/帧间隙等）。

## 运行环境

* Python **3.14.7**（仅使用标准库；在 3.10+ 上语法也兼容，但以 3.14.7 为准）
* Linux 原生运行，**无需安装任何第三方依赖**，无需网络/账号/密钥/硬件/
  数据库/中间件/云 API/Docker
* 核心、测试、demo 全部离线、确定性
* 仓库内 `.venv/` 为已建好的 3.14.7 隔离环境（若失效可重建：
  `python3.14 -m venv .venv`，无需联网安装任何包）

## 准确命令

```bash
# 必要测试（52 个用例，实测 < 0.1s）
./.venv/bin/python -m unittest discover -s tests -v

# 固定输入演示（实测约 0.06s，远小于 90s 预算；无 sleep、无预录结果）
./.venv/bin/python demo.py
```

在仓库根目录执行。

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
| 接收过滤表 | 每个节点配置若干条规则 `(identifier, mask)`，**两者均限 11 位**；命中条件 `(frame_id & mask) == (identifier & mask)`。空表 = 全过滤。表在 `NodeBus` 构造时快照，**一次传输期间不允许修改**（frozen 配置 + tuple 快照，无运行期修改入口） |
| 过滤语义 | 过滤只决定是否**上交 payload**：节点对线上每帧照常去填充 + CRC 校验并记录，不因过滤跳过；CRC 失败的帧只记录不上交 |
| 过滤与 ACK | 正确完整的帧即使被过滤掉，正常（非监听）接收节点仍贡献 ACK；**监听模式节点永不 ACK、永不发错误标志**，但记录看到的每帧校验结果 |
| 上交记录 | 交付时附命中的规则序号；多条规则重叠命中**只交付一次**，依据取表中**最早**的规则序号 |
| 仲裁失败者 | 携带发送帧的节点仲裁失败后自动转为接收者：参与 ACK 并按自己的过滤表接收胜者帧 |

## 公开接口

```python
from canbit import (
    # 数据结构
    Frame,                # Frame(identifier:int, data:bytes)；frozen，DLC=len(data)
    EncodedFrame,         # encode_frame 的产物（raw_bits/crc/stuffed/stream/映射）
    DecodedFrame,         # decode_frame 的产物（frame/crc/crc_ok）
    Sender,               # Sender(name:str, frame:Frame)
    BitBus, Event, StepResult, ErrorFrameInfo, BusResult,
    # 接收过滤表 / 节点层
    FilterRule,           # FilterRule(identifier:int, mask:int)，均限 11 位
    BusNode,              # BusNode(name, filters=(), listen_only=False, tx_frame=None)
    ReceiveRecord,        # 一节点对一帧的接收记录（含命中规则序号）
    NodeBus, NodeBusResult,
    # 纯函数
    compute_crc15,        # (identifier, dlc, data) -> int 0..0x7FFF
    payload_bits,         # (identifier, dlc, data) -> list[int]
    bit_stuff, destuff_bits,   # 位填充 / 逆填充（DestuffResult 带违例索引）
    encode_frame, decode_frame,
    encode_frame_with_crc,     # 故障注入：以指定 CRC 值编码（构造线上 CRC 错误帧）
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
    encoded_overrides={"A": enc},     # 可选：发送者名->预编码帧（故障注入钩子）
)
while not bus.finished:
    sr = bus.step()        # StepResult(bit_index, phase, bus_bit,
                           #           transmitted, arbitration_losers, bit_errors)
    # bus.add_ack_puller("B")         # 可选：transmit 阶段动态加入 ACK 拉低者
result = bus.result()      # 或直接 bus.run()
result.winners             # ('B',)
result.error_frame         # None 或 ErrorFrameInfo(trigger, bit_index, participants, aborted)
bus.events                 # 每位时间的可审计 Event 序列
```

带接收过滤表的节点层（内部逐 bit 驱动上面的 BitBus）：

```python
bus = NodeBus(
    [
        BusNode("TX", tx_frame=Frame(0x1A0, b"\xAA\x55")),
        BusNode("RX", filters=(FilterRule(0x100, 0x700),   # 规则 0
                               FilterRule(0x180, 0x780))),  # 规则 1（与 0 重叠）
        BusNode("LIS", filters=(FilterRule(0x1A0, 0x7FF),), listen_only=True),
    ],
    encoded_overrides={"TX": encode_frame_with_crc(Frame(0x1A0, b"\xAA\x55"), 0)},
)
while not bus.finished:
    bus.step()                       # 逐 bit 驱动原总线
res = bus.result()                   # NodeBusResult(bus=BusResult, records=...)
rec = res.records_of("RX")[0]        # ReceiveRecord：每节点每帧恰好一条
rec.crc_ok                           # 该校验结果（被过滤的帧也照常校验记录）
rec.matched_rules                    # 命中的全部规则序号，如 (0, 1)
rec.delivered, rec.rule_index        # 是否上交 / 交付依据的最早规则序号
res.bus.acked                        # 被过滤的正确帧仍由正常节点 ACK
```

节点层参数与错误状态：

* `BusNode(name, filters=(), listen_only=False, tx_frame=None)`：`filters`
  为空表 = 全过滤；`listen_only=True` 的节点不允许携带 `tx_frame`
  （违反抛 `ValidationError`）；`listen_only` 必须是 `bool`。
* `NodeBus(nodes, ...)`：至少 1 个节点、名字唯一、至少 1 个节点携带
  `tx_frame`，否则 `ValidationError`；`encoded_overrides` 的键必须是本轮
  发送者名。
* 过滤表在 `NodeBus` 构造时快照固化，一次传输期间不可修改（`BusNode` 为
  frozen dataclass，赋值即 `FrozenInstanceError`；快照为 tuple，无修改入口）。
* 传输被错误帧中止（bit_error/ack_error）时没有完整帧，任何节点都不产生
  接收记录（`records == ()`）；CRC 损坏的完整帧产生 `crc_ok=False`、
  `delivered=False` 的记录。
* `BitBus.add_ack_puller(name)`：传输已结束抛 `BusFinishedError`；已过
  ACK 槽（非 transmit 阶段）或名字非法抛 `ValidationError`。
* `BitBus.result()`：胜者帧 CRC 不匹配（只能来自 `encoded_overrides`
  故障注入）时不抛异常，以 `decoded_crc_ok=False` 返回；填充/长度类
  结构性错误仍抛 `FrameDecodeError`。

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
* 过滤规则的 `identifier` 与 `mask` 均 ∈ [0, 2047]（11 位），越界拒绝。
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
7. **过滤与 ACK 解耦**：ACK 槽建模位级接收成功，正常接收节点对每帧都
   拉低（包括被过滤掉的帧与 CRC 损坏的帧）；过滤只作用于“是否上交
   payload”。这让“全过滤仍有 ACK”“只有监听者时 ack_error”成为同一
   套显式 ACK 规则的自然推论，不引入真实 CAN 的错误界定计数。
8. **仲裁失败者动态转接收**：`NodeBus.step` 在看到 `arbitration_losers`
   的同一个位时间调用 `BitBus.add_ack_puller`，与裸 BitBus 上手工把
   失败者写进 `ack_pullers` 的总线位流逐位一致（有对拍测试）。
9. **过滤匹配双实现对拍**：核心用整数掩码运算
   `(frame_id & mask) == (id & mask)`；`tests/reference.py` 的独立参考
   把三者展开成 11 位列表逐位比较，结构不同且不 import 被测核心。
10. **CRC 故障注入走显式钩子**：`encode_frame_with_crc` +
    `encoded_overrides` 让发送者发出“填充合法但 CRC 错误”的帧（wired-AND
    外力无法把 0 抬成 1，不能用它造这种帧）；该钩子只替换编码产物，
    不改变逐 bit 仲裁/监控语义。

## 已知限制

* 不是完整 CAN/CAN FD：无 RTR、IDE、保留位、控制场其余编码、远程帧、
  过载帧、帧间隙、位时序/同步、重同步跳转宽度。
* 无重传、无主动/被动错误/总线关闭状态机、无错误界定符与错误计数。
* ACK 槽为 1 位模型，不区分 ACK 槽/界定符；EOF 为 1 位而非 7 位。
* `forced_bits` 是测试/演示用的确定性故障注入钩子，不代表真实收发器行为。
* `encoded_overrides` / `encode_frame_with_crc` 同为故障注入钩子，只用于
  构造线上 CRC 错误帧，不属于正常发送路径。
* DLC 仅 0..8；解码出 DLC>8 直接报 `protocol` 错误（不做经典 CAN 的
  DLC≥8 映射）。
* 接收过滤表只决定 payload 是否上交；不模拟真实控制器的验收滤波硬件
  队列、FIFO 溢出或远程帧过滤，也没有错误主动/被动计数与总线关闭状态。

## 目录

```
canbit/__init__.py     # 公开导出
canbit/errors.py       # CanError/ValidationError/FrameDecodeError/BusFinishedError
canbit/framing.py      # Frame/CRC15/位填充/整帧编码与解码（含故障注入编码）
canbit/bus.py          # BitBus：逐 bit 仲裁/位错误/ACK/错误帧
canbit/nodes.py        # FilterRule/BusNode/NodeBus：固定接收过滤表与接收记录
tests/test_framing.py  # CRC、填充、编解码测试（对照独立参考）
tests/test_bus.py      # 仲裁/错误/ACK/契约/差分对拍测试
tests/test_nodes.py    # 过滤表/监听模式/重叠规则/仲裁失败者转接收测试
tests/reference.py     # 独立参考实现（不 import canbit）
demo.py                # 固定输入演示
```
