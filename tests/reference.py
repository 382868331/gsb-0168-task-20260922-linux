"""独立整帧编码/总线参考实现（仅测试与 demo 校验使用）。

与被测核心 canbit 的算法刻意不同：

* CRC：把整条消息当作一个大整数，左移 15 位后做 MSB 对齐的 GF(2)
  长除法（核心用的是 15 位 LFSR 逐位移位）。
* 位填充：用「待写计数 + 独立扫描」的状态机（核心用 run/last 计数）。
* 总线：一次性全量仿真的独立实现（核心是逐 bit 步进对象）。

本模块不 import canbit，期望值不来自被测代码。
"""

from __future__ import annotations

REF_POLY = 0x4599
REF_CRC_WIDTH = 15


def ref_int_to_bits(value: int, width: int) -> list[int]:
    return [(value >> k) & 1 for k in range(width - 1, -1, -1)]


def ref_bits_to_int(bits: list[int]) -> int:
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


def ref_message(identifier: int, dlc: int, data: bytes) -> list[int]:
    bits = ref_int_to_bits(identifier, 11)
    bits += ref_int_to_bits(dlc, 4)
    for byte in data:
        bits += ref_int_to_bits(byte, 8)
    return bits


def ref_crc15(identifier: int, dlc: int, data: bytes) -> int:
    """整数多项式长除法版本，独立于核心的 LFSR 实现。"""
    nbits = 11 + 4 + 8 * dlc
    msg = identifier
    msg = (msg << 4) | dlc
    for byte in data:
        msg = (msg << 8) | byte
    msg <<= REF_CRC_WIDTH  # 题面：位串左移 15 位

    # 从最高位（位置 nbits+14）向下除到余数区（位置 15）
    for pos in range(nbits + REF_CRC_WIDTH - 1, REF_CRC_WIDTH - 1, -1):
        if (msg >> pos) & 1:
            msg ^= REF_POLY << (pos - REF_CRC_WIDTH)
    return msg & ((1 << REF_CRC_WIDTH) - 1)


def ref_stuff(bits: list[int]) -> list[int]:
    """填充：扫描计数，见到第 5 个连续同位立刻追加反位。

    填充位本身作为新游程的第 1 位计数（与线上解码端一致：
    00000 后插 1，再来连续四个 1 即满 5 个 1，须再插 0）。
    """
    out: list[int] = []
    same = 0
    prev = None
    for b in bits:
        out.append(b)
        if prev == b:
            same += 1
        else:
            same = 1
            prev = b
        if same == 5:
            out.append(1 - b)
            prev = 1 - b
            same = 1
    return out


def ref_encode_stream(identifier: int, dlc: int, data: bytes) -> dict:
    """独立整帧编码：返回 SOF+填充(ID..CRC)+ACK释放+EOF 的完整线上位流。"""
    msg = ref_message(identifier, dlc, data)
    crc = ref_crc15(identifier, dlc, data)
    crc_bits = ref_int_to_bits(crc, 15)
    stuffed = ref_stuff(msg + crc_bits)
    stream = [0] + stuffed + [1, 1]  # SOF, ..., ACK 释放(1), EOF(1)
    return {
        "raw": msg,
        "crc": crc,
        "crc_bits": crc_bits,
        "stuffed": stuffed,
        "stream": stream,
        "ack_index": 1 + len(stuffed),
        "eof_index": 2 + len(stuffed),
    }


def ref_destuff(stuffed: list[int]) -> tuple[list[int], int | None]:
    """参考去填充：返回 (原始位, 首个违例索引或 None)。"""
    out: list[int] = []
    same = 0
    prev = None
    waiting = False
    err = None
    for idx, b in enumerate(stuffed):
        if waiting:
            if b == prev and err is None:
                err = idx
            waiting = False
            same = 1
            prev = b
            continue
        out.append(b)
        if prev == b:
            same += 1
        else:
            same = 1
            prev = b
        if same == 5:
            waiting = True
    if waiting and err is None:
        err = len(stuffed)
    return out, err


def ref_id_indices(stuffed_len: int, stuffing_positions: set[int]) -> set[int]:
    """填充后流中 11 个 ID 原始位的线上绝对索引（含 SOF 偏移 1）。"""
    ids: set[int] = set()
    raw_no = 0
    for off in range(stuffed_len):
        if off in stuffing_positions:
            continue
        if raw_no < 11:
            ids.add(1 + off)
        raw_no += 1
    return ids


def ref_stuff_positions(bits: list[int]) -> set[int]:
    """填充后流里填充位的偏移集合。"""
    out: list[tuple[int, bool]] = []
    same = 0
    prev = None
    for b in bits:
        out.append((b, False))
        if prev == b:
            same += 1
        else:
            same = 1
            prev = b
        if same == 5:
            out.append((1 - b, True))
            prev = 1 - b
            same = 1
    return {i for i, (_, is_stuff) in enumerate(out) if is_stuff}


def ref_filter_match(frame_id: int, identifier: int, mask: int) -> bool:
    """独立过滤匹配参考：把三者展开成 11 位列表，逐位比较 mask 覆盖位。

    与被测核心 FilterRule.matches 的整数掩码运算结构不同，用于对拍。
    """
    fid = ref_int_to_bits(frame_id, 11)
    iid = ref_int_to_bits(identifier, 11)
    msk = ref_int_to_bits(mask, 11)
    return all(f == i for f, i, m in zip(fid, iid, msk) if m == 1)


def ref_simulate_bus(
    frames: list[tuple[str, int, bytes]],
    *,
    ack_pullers: set[str] | None = None,
    forced_bits: dict[int, int] | None = None,
) -> dict:
    """独立的一次性总线全量仿真。

    frames: [(name, identifier, data), ...]，同时开始发送。
    返回 winner（正常完成时）、losers（仲裁失败者及位置）、
    error（bit_error/ack_error 的位置与参与者）或 completed 标记。
    """
    ack_pullers = ack_pullers or set()
    forced_bits = forced_bits or {}
    enc = {
        name: ref_encode_stream(ident, len(data), data)
        for name, ident, data in frames
    }
    active = [name for name, _, _ in frames]
    losers: list[tuple[str, int]] = []
    order = [name for name, _, _ in frames]

    pos = 0
    while True:
        slot = "data"
        contrib: dict[str, int] = {}
        for name in active:
            e = enc[name]
            if pos == e["ack_index"]:
                slot = "ack"
                contrib[name] = 1
            elif pos == e["eof_index"]:
                slot = "eof"
                contrib[name] = 1
            else:
                contrib[name] = e["stream"][pos]

        bus = 0 if any(v == 0 for v in contrib.values()) else 1
        if slot == "ack" and ack_pullers:
            bus = 0
        if pos in forced_bits:
            bus &= forced_bits[pos]

        if slot == "data":
            id_sets = {
                name: ref_id_indices(
                    len(enc[name]["stuffed"]),
                    ref_stuff_positions(
                        enc[name]["raw"] + enc[name]["crc_bits"]
                    ),
                )
                for name in active
            }
            new_losers = []
            err_nodes = []
            for name, bit in contrib.items():
                if bit == bus:
                    continue
                if bit == 1 and bus == 0 and pos in id_sets[name]:
                    new_losers.append(name)
                else:
                    err_nodes.append(name)
            if err_nodes:
                return {
                    "outcome": "bit_error",
                    "pos": pos,
                    "participants": tuple(err_nodes),
                    "losers": tuple(losers),
                    "bus": bus,
                }
            for name in new_losers:
                active.remove(name)
                losers.append((name, pos))

        if slot == "eof":
            # EOF 固定隐性，任何失配（外力注入）也是位错误
            err_nodes = [name for name, bit in contrib.items() if bit != bus]
            if err_nodes:
                return {
                    "outcome": "bit_error",
                    "pos": pos,
                    "participants": tuple(err_nodes),
                    "losers": tuple(losers),
                    "bus": bus,
                }

        if slot == "ack":
            if bus == 1:
                return {
                    "outcome": "ack_error",
                    "pos": pos,
                    "participants": tuple(active),
                    "losers": tuple(losers),
                    "bus": bus,
                }
        if slot == "eof":
            return {
                "outcome": "complete",
                "pos": pos,
                "winner": tuple(active),
                "losers": tuple(losers),
                "bus": bus,
            }
        pos += 1
