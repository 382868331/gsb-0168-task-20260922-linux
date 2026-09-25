# canbit — CAN 合成位流的仲裁与错误约束（本题契约子集）

一个零依赖的离线 Python 题库，对**有限定义**的 CAN 风格总线做逐 bit 建模：
帧编码、CRC15、位填充、wired-AND 仲裁、位错误、显式 ACK 槽与错误帧中止，
以及**接收节点的固定接收过滤表**（id/掩码命中、监听模式、仲裁失败转接收）。

> **本库只实现题面契约定义的有限协议子集，不宣称兼容完整 CAN 标准**
> （无 RTR/IDE/控制场其余位、无重传、无错误界定/被动错误计数、无位时序
> 与同步、无错误界定符/帧间隙等）。

## 运行环境

* Python **3.14.7**（仅使用标准库；在 3.10+ 上语法也兼容，但以 3.14.7 为准）
* Linux 原生运行，**无需安装任何第三方依赖**，无需网络/账号/密钥/硬件/
  数据库/中间件/云 API/Docker
* 核心、测试、demo 全部离线、确定性

## 准确命令

仓库内置 `./.venv`（Python 3.14.7，仅标准库）。在仓库根目录：

```bash
# 必要测试（48 个用例，实测 < 0.1s）
./.venv/bin/python -m unittest discover -s tests -v

# 固定输入演示（实测约 0.05s，远小于 90s 预算；无 sleep、无预录结果）
./.venv/bin/python demo.py
```

也可使用任意指向 3.14.7 的解释器：`python -m unittest discover -s tests -v`、
`python demo.py`。

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
| ACK | 所有发送者在 ACK 槽释放（发 1）；总线是否被拉低由 `ack_pullers` 与 `nodes` 中的普通接收节点共同决定；无任何接收者拉低则仍在发送者报 `ack_error` |
| 接收过滤表 | 每个 `Node` 固定若干条 `Filter(id, mask)`，id/mask 均限 **11 位**（0..0x7FF）；命中条件 `(frame_id & mask) == (id & mask)`。规则按顺序从 0 编号，一帧命中多条时**只上交一次**且序号取**最早**规则；空表不匹配任何帧。过滤**只决定是否上交 payload**，不影响去填充/CRC 校验与 ACK |
| 接收与 ACK 判定 | 接收节点对**每一帧**逐位采样、去填充并复核 CRC。普通节点仅当帧完整且 CRC/填充正确时在 ACK 槽拉低——**即使该帧被它过滤掉仍贡献 ACK**；CRC 损坏的帧不上交也不 ACK |
| 监听模式 | `Node(..., listen_only=True)`：**永不**拉低 ACK、**永不**发错误标志；仍逐位接收、复核 CRC，并在 `ReceiveRecord` 中记录看到的校验结果（含损坏帧的 crc_ok=False） |
| 仲裁失败转接收 | 若某发送者与一个 `Node` 同名，该参与者在 ID 域仲裁失败后，自下一位置起**自动以该节点身份转接收**（沿用其过滤表/监听模式），接收缓冲按总线位回填，能收到胜者帧 |
| 表的不可变性 | `Filter`/`Node` 均为 frozen；构造时复制为元组，传输期间无法修改；`BitBus.node_filters()` 只给只读元组视图 |
| 错误帧 | 检测到错误的那个位时间即中止当前传输，检测者随后发 **6 位显性错误标志**；`ErrorFrameInfo` 记录触发类型、位索引、检测者与被中止的全部参与者（监听节点从不出现在检测者中） |
| CRC 检查时机 | 接收/解码视角先去填充，再切字段，再校验 CRC；填充错误先于 CRC 报告 |

## 公开接口

```python
from canbit import (
    # 数据结构
    Frame,                # Frame(identifier:int, data:bytes)；frozen，DLC=len(data)
    EncodedFrame,         # encode_frame 的产物（raw_bits/crc/stuffed/stream/映射）
    DecodedFrame,         # decode_frame 的产物（frame/crc/crc_ok）
    Sender,               # Sender(name:str, frame:Frame)
    Filter,               # Filter(id:int, mask:int=0x7FF)，均限 11 位，frozen
    Node,                 # Node(name, filters=(), *, listen_only=False)，frozen
    ReceiveRecord,        # 节点对一帧的接收记录（见下）
    BitBus, Event, StepResult, ErrorFrameInfo, BusResult,
    # 过滤/接收纯函数
    first_match,          # (filters, frame_id) -> 最早命中序号 | None
    evaluate_reception,   # (node, stuffed_bits, *, acked) -> ReceiveRecord
    # 纯函数
    compute_crc15,        # (identifier, dlc, data) -> int 0..0x7FFF
    payload_bits,         # (identifier, dlc, data) -> list[int]
    bit_stuff, destuff_bits,   # 位填充 / 逆填充（DestuffResult 带违例索引）
    encode_frame, decode_frame,
    # 常量
    ID_BITS, DLC_BITS, CRC_BITS, MAX_DATA_LEN, POLYNOMIAL,
    FILTER_ID_BITS, FILTER_MASK_MAX, FILTER_MAX_RULES,   # 11 / 0x7FF / 32
)
from canbit.errors import (
    CanError, ValidationError, FrameDecodeError, BusFinishedError,
)
```

接收节点 / 固定过滤表用法：

```python
bus = BitBus(
    [Sender("A", Frame(0x321, b"hi")), Sender("B", Frame(0x320, b"yo"))],
    nodes=[
        Node("GW", [Filter(0x300, 0x700), Filter(0x320, 0x7FF)]),  # 重叠规则
        Node("SNIFF", [Filter(0, 0)], listen_only=True),            # 监听者
        Node("BLIND", []),                                          # 空表
        Node("A", [Filter(0x300, 0x700)]),     # 与发送者同名：败则转接收
    ],
    # ack_pullers={"X"}          # 可选：没有 Node 对象的额外裸 ACK 者
    # receive_glitches={18: 0},  # 可选：仅接收视角的 1->0 确定性注入
)
result = bus.run()                 # 同样支持 while not bus.finished: bus.step()
result.winners                     # ('B',)
result.arbitration_losers          # (('A', 11),)
for name, rec in result.receptions.items():
    rec.crc_ok        # 该节点看到的 CRC 校验结果（被过滤也照算）
    rec.delivered     # 仅当 crc_ok 且命中过滤表
    rec.rule_index    # 上交时命中的最早规则序号；否则 None
    rec.acked         # 该节点是否在 ACK 槽拉低（监听节点恒 False）
    rec.frame         # 解码出的 Frame（结构无法切分时为 None）
result.delivered()                 # [(name, ReceiveRecord), ...] 仅实际上交者
bus.node_filters("GW")             # 只读元组视图；传输期间不可改
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
* 过滤表参数：`Filter.id/mask ∈ [0, 0x7FF]`（11 位），拒绝 bool/越界；
  每节点最多 `FILTER_MAX_RULES=32` 条，元素必须是 `Filter`；节点名必须
  非空且彼此唯一；`listen_only` 必须是 bool；监听节点若同时出现在
  `ack_pullers` 中（语义矛盾：监听者不能 ACK）构造期即拒绝。
* 错误状态分类：发送侧 `bit_error`（非仲裁位失配）与 `ack_error`（ACK 槽
  无人拉低）都会中止当前传输并发 6 位显性错误标志，见 `BusResult.error_frame`；
  接收侧不产生错误帧，而是在每个节点的 `ReceiveRecord` 中给出
  `crc_ok`/`stuff_ok` 与 `error ∈ {stuff, length, protocol}`。监听节点永不
  作为错误帧 `participants`。
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
4. **ACK 显式化**：不模拟自动应答；`ack_pullers` 可直接指定任意参与者拉低
   （仲裁失败者、纯接收者均可）。提供 `nodes` 时，普通接收节点对“结构完好
   且 CRC 正确”的帧**自动**拉低——**与该帧是否命中其过滤表无关**；监听
   节点永不拉低。即“过滤只管上交，不管 ACK”。
5. **错误帧固定 6 位显性标志**：只表达“中止 + 谁报的 + 在哪一位”，不实现
   错误界定符、错误计数与重传。
6. **可审计**：每个位时间产生一条 `Event`；错误位置同时给出填充后流
   0 基索引，便于和 `EncodedFrame.stuffing_map` 对照定位。
7. **接收与发送解耦**：节点各自维护逐位接收缓冲，独立做去填充/CRC/过滤；
   `receive_glitches` 仅作用于接收缓冲（线与 1->0），因此能确定性复现
   “发送完好、接收 CRC 坏”的情形而不在总线上制造 bit_error。仲裁失败者
   转接收时缓冲按已发生的总线位回填（存活发送者此前位流必然相同）。

## 已知限制

* 不是完整 CAN/CAN FD：无 RTR、IDE、保留位、控制场其余编码、远程帧、
  过载帧、帧间隙、位时序/同步、重同步跳转宽度。
* 无重传、无主动/被动错误/总线关闭状态机、无错误界定符与错误计数。
* ACK 槽为 1 位模型，不区分 ACK 槽/界定符；EOF 为 1 位而非 7 位。
* `forced_bits` 与 `receive_glitches` 是测试/演示用的确定性故障注入钩子，
  不代表真实收发器行为；后者只改接收缓冲、不改总线电平。
* 接收过滤仅支持本题的 11 位 id+mask 形式（无扩展帧 29 位过滤、无过滤模式
  切换/运行期改表）；表在一次传输期间固定不可改。
* DLC 仅 0..8；解码出 DLC>8 直接报 `protocol` 错误（不做经典 CAN 的
  DLC≥8 映射）。

## 目录

```
canbit/__init__.py     # 公开导出
canbit/errors.py       # CanError/ValidationError/FrameDecodeError/BusFinishedError
canbit/framing.py      # Frame/CRC15/位填充/整帧编码与解码
canbit/bus.py          # BitBus：逐 bit 仲裁/位错误/ACK/错误帧/节点接收
canbit/node.py         # Filter/Node/ReceiveRecord 与固定过滤表接收判定
tests/test_framing.py  # CRC、填充、编解码测试（对照独立参考）
tests/test_bus.py      # 仲裁/错误/ACK/契约/差分对拍测试
tests/test_nodes.py    # 过滤表/监听/CRC 损坏/重叠规则/仲裁失败转接收
tests/reference.py     # 独立参考实现（不 import canbit）
demo.py                # 固定输入演示（含接收过滤新场景 5..7）
```
