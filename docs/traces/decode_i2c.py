#!/usr/bin/env python3
"""Decode I2C transactions from a 2-channel VCD (D0=SDA, D1=SCL)."""
import sys

def parse_vcd(path):
    ids = {}
    t = 0
    changes = []  # (time, name, value)
    with open(path) as f:
        in_defs = True
        for line in f:
            line = line.strip()
            if in_defs:
                if line.startswith('$var'):
                    parts = line.split()
                    ids[parts[3]] = parts[4]
                elif line.startswith('$enddefinitions'):
                    in_defs = False
                continue
            if line.startswith('#'):
                t = int(line[1:])
            elif line and line[0] in '01':
                val = int(line[0])
                sig = line[1:]
                if sig in ids:
                    changes.append((t, ids[sig], val))
    return changes

def decode(changes):
    sda = scl = 1
    events = []
    bits = []
    byte_start = None
    in_frame = False
    first_byte = True
    out = []
    cur = None
    for t, name, val in changes:
        if name in ('D0', 'SDA'):
            if scl == 1 and in_frame is not None:
                if val == 0 and sda == 1 and scl == 1:
                    # START (or repeated START)
                    if cur:
                        out.append(cur)
                    cur = {'t': t, 'repeated': in_frame, 'bytes': []}
                    in_frame = True
                    bits = []
                elif val == 1 and sda == 0 and scl == 1 and in_frame:
                    # STOP
                    if cur:
                        cur['stop'] = t
                        out.append(cur)
                        cur = None
                    in_frame = False
                    bits = []
            sda = val
        elif name in ('D1', 'SCL'):
            if val == 1 and scl == 0 and in_frame:
                bits.append((t, sda))
                if len(bits) == 9:
                    byte = 0
                    for _, b in bits[:8]:
                        byte = (byte << 1) | b
                    ack = bits[8][1] == 0
                    cur['bytes'].append((bits[0][0], byte, ack))
                    bits = []
            scl = val
    if cur:
        out.append(cur)
    return out

def fmt(trans):
    for tr in trans:
        parts = []
        for i, (t, byte, ack) in enumerate(tr['bytes']):
            if i == 0:
                addr = byte >> 1
                rw = 'R' if byte & 1 else 'W'
                parts.append(f"addr 0x{addr:02X} {rw} {'ACK' if ack else 'NACK'}")
            else:
                parts.append(f"0x{byte:02X} {'ACK' if ack else 'NACK'}")
        s = 'Sr' if tr.get('repeated') else 'S'
        stop = ' P' if 'stop' in tr else ''
        print(f"t={tr['t']/1e9:10.6f}s {s} {' | '.join(parts)}{stop}")

if __name__ == '__main__':
    changes = parse_vcd(sys.argv[1])
    trans = decode(changes)
    print(f"{len(trans)} transactions")
    fmt(trans)
