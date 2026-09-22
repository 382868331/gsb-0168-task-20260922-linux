# canbus —— CAN 合成位流的仲裁与错误约束（有限总线模型）

离线车载通信工程用可复用模块：对多发送者同时发送的 CAN 风格帧做
**逐 bit 仲裁仿真与错误约束检查**。纯 Python 标准库实现，不调用任何
第三方同类实现。

- **Python 版本**：3.14.7（Linux 原生运行）
- **依赖**：仅标准库，安装 Python 后无需安装任何其他依赖
- **离线**：核心、测试、演示全部离线，不需要网络、账号、硬件或 Docker

> 本模块实现的是题目契约定义的**有限协议子集**，不宣称兼容完整
> CAN 标准（无 SOF/RTR/IDE/EOF/帧间隔/错误帧/自动重传等）。

## 帧模型与规则

- 帧 = 11 位 ID + 4 位 DLC（0..8）+ 0..8 字节数据 + 15 位 CRC，均高位先发。
- CRC：ID+DLC+数据位串左移 15 位后对生成多项式 **0x4599**（隐含 x^15，
  即 0xC599）取余，初值 0。
- 位填充：从 ID 到 CRC，每连续 5 个相同位插入 1 个反位。
- ACK：固定 1 个独立槽，不参与填充；发送者释放（隐性 1），是否由接收者
  拉低由显式参数 `ack_pull` 决定；无人拉低报 `ack_error`。
- 显性 0 胜隐性 1（线与）。ID 域内发送 1 读到 0 只退出仲裁；其余位置
  失配报 `bit_error`。接收侧去填充后发现 6 连同位报 `stuff_error`，
  CRC 不符报 `crc_error`。任何错误中止当前传输并记录参与者。

## 接口（`canbus.py`）

| 名称 | 说明 |
|---|---|
| `Frame(id, data=b"")` | 不可变帧；`id` 0..0x7FF，`data` 0..8 字节 |
| `crc15(bits) -> int` | 15 位 CRC（位串为 0/1 int 序列） |
| `encode_unstuffed(frame)` | ID+DLC+数据+CRC 未填充位串 |
| `stuff_bits(bits)` / `destuff_bits(bits)` | 位填充 / 去填充（后者抛 `StuffError`） |
| `encode_frame(frame)` | 发送端完整位流：填充后 ID..CRC + ACK 释放位 |
| `decode_stream(bits) -> Frame` | 接收侧解码（不含 ACK 槽），抛 `StuffError`/`CrcError`/`CanBusError` |
| `BusSimulation(frames, ack_pull=True)` | 逐 bit 仿真；`.step() -> BitEvent`，`.done`，`.result` |
| `simulate(frames, ack_pull=True) -> SimResult` | 一次性跑完 |
| `BitEvent` / `BusError` / `SimResult` | 槽位记录 / 错误记录 / 最终结果 |
| `StuffError` / `CrcError` / `CanBusError` | 解码异常层级 |

`SimResult`：`ok, winners, frame, error, events, ack`。
`BusError`：`kind ∈ {bit_error, stuff_error, crc_error, ack_error,
decode_error}, slot, field, participants, detail`。

## 输入上限与错误语义

- ID：0..0x7FF；数据：0..8 字节；发送者：1..8 个；单帧最长 113 槽。
- 整数参数**拒绝 bool**；浮点（含 NaN/Infinity）一律 `TypeError`；
  越界 `ValueError`；位串元素必须是 int 0/1。非法输入报可定位错误
  （含参数名/下标），不静默纠正。
- 构造 `BusSimulation` 时完成全部校验，失败不留下任何状态；仿真中
  出错即中止，`result` 一次性写定，之后 `step()` 抛 `RuntimeError`，
  不存在未说明的部分变更。
- **浮点误差口径**：本模块全部输入输出为整数/布尔/位串，无任何浮点
  计算，不涉及误差容差。

## 设计取舍与已知限制

- 协议错误（bit/stuff/crc/ack error）作为**仿真结果记录**返回而非异常，
  因为它们是预期的仿真输出；非法输入才抛异常。
- 无故障注入的仿真中，发送位流必然合法，故 stuff/crc 错误只能经
  `decode_stream` 对受损位流触发（测试即如此覆盖）。
- 相同帧的多个发送者可并列获胜；仲裁退出者不计入 `winners`。
- 单次传输，不重传；错误帧不在总线上显式建模，仅记录并中止。

## 命令

```bash
python -m unittest discover -s tests -v   # 单元测试（含独立整帧编码参考交叉验证）
python demo.py                            # 约 0.1 秒：正常仲裁 + bit/ack 错误 + 拒绝边界
```

测试中 `ref_encode`/`ref_crc`/`ref_stuff` 为独立编写的整帧编码参考
（字符串模 2 除法），不调用被测核心；往返一致性仅作补充验证。
