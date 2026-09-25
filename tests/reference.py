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


# ---- 接收节点 / 固定过滤表的独立参考 ------------------------------------
#
# 不 import canbit：节点逐位缓冲、去填充、CRC 复核、过滤命中与 ACK
# 决策全部在本模块独立复算，期望值不来自被测代码。


def ref_filter_match(rules, frame_id):
    """rules: [(id, mask), ...]；返回最早命中序号，否则 None。"""
    for index, (rid, mask) in enumerate(rules):
        if (frame_id & mask) == (rid & mask):
            return index
    return None


def ref_decode_reception(stuffed):
    """独立“接收视角”解码：去填充 -> 切字段 -> 复核 CRC。

    CRC 损坏（结构完好）不抛，给 crc_ok=False；结构无法切分给 error。
    """
    raw, stuff_err = ref_destuff(stuffed)
    if stuff_err is not None:
        return {"ok": False, "stuff_ok": False, "error": "stuff",
                "frame": None, "crc_ok": False}
    if len(raw) < 15:
        return {"ok": False, "stuff_ok": True, "error": "length",
                "frame": None, "crc_ok": False}
    ident = ref_bits_to_int(raw[:11])
    dlc = ref_bits_to_int(raw[11:15])
    if dlc > 8:
        return {"ok": False, "stuff_ok": True, "error": "protocol",
                "frame": None, "crc_ok": False}
    expected = 11 + 4 + 8 * dlc + 15
    if len(raw) != expected:
        return {"ok": False, "stuff_ok": True, "error": "length",
                "frame": None, "crc_ok": False}
    data_bits = raw[15 : 15 + 8 * dlc]
    data = bytes(
        ref_bits_to_int(data_bits[i : i + 8]) for i in range(0, len(data_bits), 8)
    )
    crc_ok = ref_bits_to_int(raw[-15:]) == ref_crc15(ident, dlc, data)
    return {"ok": crc_ok, "stuff_ok": True, "error": None,
            "frame": (ident, dlc, data), "crc_ok": crc_ok}


def ref_node_records(specs, rx_active, rxbits, rxack):
    """根据当前接收缓冲独立生成每个在收节点的接收记录。"""
    records = {}
    for n in rx_active:
        rules, listen = specs[n]
        dec = ref_decode_reception(rxbits[n])
        rule = (
            ref_filter_match(rules, dec["frame"][0])
            if dec["frame"] is not None
            else None
        )
        delivered = dec["ok"] and rule is not None
        records[n] = {
            "frame": dec["frame"],
            "delivered": delivered,
            "rule_index": rule if delivered else None,
            "crc_ok": dec["crc_ok"],
            "stuff_ok": dec["stuff_ok"],
            "acked": rxack.get(n, False),
            "listen_only": listen,
            "error": dec["error"],
        }
    return records


def ref_simulate_bus_nodes(
    frames,
    nodes,
    *,
    ack_pullers=None,
    forced_bits=None,
    receive_glitches=None,
):
    """带接收节点/固定过滤表的一次性独立全量仿真。

    frames: [(name, identifier, data), ...]
    nodes:  [(name, [(filter_id, mask), ...], listen_only), ...]
    仲裁失败者若有同名节点，则自失配位起转接收（回填此前总线位）。
    """
    ack_pullers = set(ack_pullers or ())
    forced_bits = forced_bits or {}
    receive_glitches = receive_glitches or {}
    enc = {
        name: ref_encode_stream(ident, len(data), data)
        for name, ident, data in frames
    }
    specs = {name: (list(rules), bool(listen)) for name, rules, listen in nodes}
    listener_names = {n for n, (_, listen) in specs.items() if listen}

    active = [name for name, _, _ in frames]
    rx_active = {n for n in specs if n not in active}
    rxbits = {n: [] for n in rx_active}
    rxack = {n: False for n in rx_active}
    losers = []
    wire = {}  # 位置 -> 实际总线电平（仲裁失败前各发送者位流必然相同）

    def sample(name, pos, bus_bit):
        bit = bus_bit & receive_glitches[pos] if pos in receive_glitches else bus_bit
        rxbits[name].append(bit)

    pos = 0
    outcome = "complete"
    records = {}
    while True:
        slot = "data"
        contrib = {}
        for name in active:
            e = enc[name]
            if pos == e["ack_index"]:
                slot, contrib[name] = "ack", 1
            elif pos == e["eof_index"]:
                slot, contrib[name] = "eof", 1
            else:
                contrib[name] = e["stream"][pos]

        bus = 0 if any(v == 0 for v in contrib.values()) else 1

        node_pullers = set()
        if slot == "ack":
            for name in rx_active:
                _, listen = specs[name]
                if not listen and ref_decode_reception(rxbits[name])["ok"]:
                    node_pullers.add(name)
            effective = (ack_pullers - listener_names) | node_pullers
        else:
            effective = ack_pullers
        if slot == "ack" and effective:
            bus = 0
        if pos in forced_bits:
            bus &= forced_bits[pos]
        if slot == "data":
            wire[pos] = bus

        if slot == "data":
            id_sets = {
                name: ref_id_indices(
                    len(enc[name]["stuffed"]),
                    ref_stuff_positions(enc[name]["raw"] + enc[name]["crc_bits"]),
                )
                for name in active
            }
            new_losers, err_nodes = [], []
            for name, bit in contrib.items():
                if bit == bus:
                    continue
                if bit == 1 and bus == 0 and pos in id_sets[name]:
                    new_losers.append(name)
                else:
                    err_nodes.append(name)
            if err_nodes:
                for n in rx_active:
                    sample(n, pos, bus)
                outcome = "bit_error"
                break
            for name in new_losers:
                active.remove(name)
                losers.append((name, pos))
                if name in specs and name not in rx_active:
                    rx_active.add(name)
                    rxbits[name] = [
                        (wire[k] & receive_glitches[k])
                        if k in receive_glitches else wire[k]
                        for k in range(1, pos)
                    ]
                    rxack[name] = False

        if slot == "eof" and any(bit != bus for bit in contrib.values()):
            outcome = "bit_error"
            break

        if slot == "data" and pos != 0:
            for n in list(rx_active):
                sample(n, pos, bus)

        if slot == "ack":
            for n in rx_active:
                rxack[n] = bus == 0 and n in effective
            records = ref_node_records(specs, rx_active, rxbits, rxack)
            if bus == 1:
                outcome = "ack_error"
                break

        if slot == "eof":
            break
        pos += 1

    if outcome == "bit_error":
        records = ref_node_records(specs, rx_active, rxbits, rxack)
    # complete / ack_error：记录在 ACK 槽已生成

    return {
        "outcome": outcome,
        "winner": tuple(active) if outcome == "complete" else (),
        "losers": tuple(losers),
        "records": records,
        "acked": outcome == "complete",
    }
