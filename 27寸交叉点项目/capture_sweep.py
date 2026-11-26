#!/usr/bin/env python3
# 扫掠核对：激光沿一条边缓慢扫过，统计每个传感器索引是否"亮过"
# 用法: python capture_sweep.py COM7 30 right [阈值]
# 检查点: 索引48/49(第7个mux的lower 0/1)是否亮; 哪些索引从未亮过

import sys
import time

import numpy as np
import serial

from serial_view import parse_full_frame, MAX_SEARCH, EDGES

port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
edge = sys.argv[3] if len(sys.argv) > 3 else 'right'
thr = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05

length = dict(EDGES)[edge]
offset = sum(n for _, n in EDGES[:[n for n, _ in EDGES].index(edge)])

ser = serial.Serial(port, 460800, timeout=0.05)
print(f"[capture] {port} {seconds}s, 目标边={edge}({length}颗), 阈值={thr}")
buf = bytearray()
mx = np.zeros(length)
cnt = np.zeros(length, dtype=int)
frames = 0
t0 = time.time()
while time.time() - t0 < seconds:
    data = ser.read(4096)
    if data:
        buf += data
    while True:
        if len(buf) < 5:
            break
        if len(buf) > MAX_SEARCH * 2:
            del buf[0]
            continue
        if buf[0] != 0xAA or buf[1] != 0x55:
            del buf[0]
            continue
        tail = -1
        for i in range(2, min(len(buf), MAX_SEARCH)):
            if buf[i] == 0xFE and i + 1 < len(buf) and buf[i + 1] == 0xFD:
                tail = i
                break
        if tail < 0:
            break
        frame = bytes(buf[:tail + 2])
        del buf[:tail + 2]
        f = parse_full_frame(frame)
        if f is None:
            continue
        prof = f['raw'][offset:offset + length].astype(float) / 4095.0
        mx = np.maximum(mx, prof)
        cnt += (prof > thr)
        frames += 1
ser.close()

out = open(r"C:\Users\light\AppData\Local\Temp\opencode\sweep_result.txt", 'w', encoding='utf-8')
out.write(f"edge={edge} frames={frames} thr={thr}\n\n")
out.write("[每个索引: 最大归一化幅值 / 超过阈值的帧数]\n")
for i in range(length):
    out.write(f"  idx {i:3d}: max={mx[i]:.3f} cnt={cnt[i]:4d}\n")
lit = [i for i in range(length) if cnt[i] > 2]
dark = [i for i in range(length) if cnt[i] <= 2]
out.write(f"\n亮过的索引({len(lit)}): {lit}\n")
out.write(f"从未亮/极少亮({len(dark)}): {dark}\n")
for a, b in ((48, 49), (98, 99)):
    if length > b:
        out.write(f"重点: idx{a} max={mx[a]:.3f} cnt={cnt[a]}; idx{b} max={mx[b]:.3f} cnt={cnt[b]}\n")
    elif length > a:
        out.write(f"重点: idx{a} max={mx[a]:.3f} cnt={cnt[a]}\n")
out.close()
print("done")
