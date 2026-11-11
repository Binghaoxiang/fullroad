#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""logger.py — 连续采集 K EDGES 原始数据到 CSV

用法: python logger.py COM5 秒数 输出文件
每行: 时间戳, x_mm, y_mm, 然后四条边的 raw + proc 摘要
"""

import sys
import time
import json
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
outf = sys.argv[3] if len(sys.argv) > 3 else "data/log.csv"

ser = serial.Serial(port, 115200, timeout=0.1)
time.sleep(1.0)
ser.reset_input_buffer()

rows = []
t0 = time.time()
n = 0

while time.time() - t0 < dur:
    ser.reset_input_buffer()
    ser.write(b"K EDGES\r\n")
    buf = ""
    t1 = time.time()
    while time.time() - t1 < 6.0:
        c = ser.read(256)
        if c:
            buf += c.decode("ascii", "ignore")
            if buf.count("\r\n") >= 4:
                break
        else:
            if buf.count("\r\n") >= 4:
                break
            time.sleep(0.005)

    rec = {"t": round(time.time() - t0, 3)}
    got = 0
    for line in buf.split("\r\n"):
        line = line.strip()
        if not line or line[0] != "E" or len(line) < 3 or line[2] != ":":
            continue
        try:
            tag = int(line[1])
            parts = line[3:].split()
            cnt = int(parts[0])
            vals = [int(x) for x in parts[1:1 + cnt]]
            if len(vals) == cnt:
                rec["E%d" % tag] = vals
                got += 1
        except Exception:
            pass
    if got == 4:
        rows.append(rec)
        n += 1

ser.close()

with open(outf, "w", encoding="utf-8") as f:
    f.write(json.dumps(rows, ensure_ascii=False))

print("saved %d frames -> %s" % (n, outf))
