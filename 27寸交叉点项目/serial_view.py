#!/usr/bin/env python3
# 独立串口查看工具（不动工程代码）：解析固件全帧（帧长按 board_layout.py 自动计算）
# 用法:
#   python serial_view.py COM7                 # 每0.2s打一行摘要
#   python serial_view.py COM7 --all           # 每帧都打
#   python serial_view.py COM7 --csv data.csv  # 同时存CSV(质心索引/峰/坐标/交点)
#   python serial_view.py COM7 --wave in_right # 每秒打印该边原始波形(归一化0~1)
# 注意: 运行前先关掉综合UI（串口独占）；只有主控会发数据

import argparse
import struct
import sys
import time

import numpy as np
import serial

# 每边点数按板型表计算（与主控固件逐段板型表同步，混合 30/35cm 板改 board_layout.py）
from board_layout import EDGES, RAW_BYTES
RESULT_BYTES = 8 * 9 + 9 + 18
FULL_LEN = 2 + RAW_BYTES + RESULT_BYTES + 1 + 2
MAX_SEARCH = FULL_LEN + 16


def parse_full_frame(frame):
    if len(frame) != FULL_LEN:
        return None
    cs = 0
    for b in frame[2:-3]:
        cs ^= b
    if cs != frame[-3]:
        return None
    payload = frame[2:-3]
    raw = np.frombuffer(payload[:RAW_BYTES], dtype='<u2')

    off = RAW_BYTES
    cents = {}
    for name, n in EDGES:
        pos, peak = struct.unpack('<ff', payload[off:off + 8])
        valid = payload[off + 8]
        cents[name] = (pos, peak, valid)
        off += 9
    x, y = struct.unpack('<ff', payload[off:off + 8])
    pvalid = payload[off + 8]
    off += 9
    xl, yl, xf, yf = struct.unpack('<ffff', payload[off:off + 16])
    hx, hy = payload[off + 16], payload[off + 17]

    out = {'raw': raw, 'cents': cents, 'x': x, 'y': y, 'valid': pvalid,
           'x_line': xl, 'y_line': yl, 'x_fused': xf, 'y_fused': yf,
           'have_x': hx, 'have_y': hy, 'off': {name: i for i, (name, _) in enumerate(EDGES)}}
    return out


def fmt_edges(c, names):
    parts = []
    for name in names:
        pos, pk, v = c[name]
        if v:
            parts.append(f"{name:>8} i={pos:7.3f} pk={pk:.2f}")
        else:
            parts.append(f"{name:>8} -")
    return " | ".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port')
    ap.add_argument('baud', nargs='?', type=int, default=460800)
    ap.add_argument('--all', action='store_true', help='每帧都打印')
    ap.add_argument('--csv', default=None, help='保存CSV路径')
    ap.add_argument('--wave', default=None, help='每秒打印该边原始波形, 如 in_right/up/right')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    print(f"[已连接] {args.port} @ {args.baud}  FULL_LEN={FULL_LEN}")

    csvf = open(args.csv, 'w', encoding='utf-8') if args.csv else None
    if csvf:
        cols = ['ts']
        for name, _ in EDGES:
            cols += [f'{name}_i', f'{name}_pk', f'{name}_v']
        cols += ['x', 'y', 'valid', 'x_line', 'y_line', 'x_fused', 'y_fused', 'have_x', 'have_y']
        csvf.write(','.join(cols) + '\n')

    buf = bytearray()
    frames = 0
    fps = 0
    t_fps = time.time()
    t_print = 0.0
    t_wave = 0.0
    try:
        while True:
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
                frames += 1
                now = time.time()
                if csvf:
                    row = [f"{now:.3f}"]
                    for name, _ in EDGES:
                        pos, pk, v = f['cents'][name]
                        row += [f"{pos:.4f}", f"{pk:.4f}", str(v)]
                    row += [f"{f['x']:.2f}", f"{f['y']:.2f}", str(f['valid']),
                            f"{f['x_line']:.2f}", f"{f['y_line']:.2f}",
                            f"{f['x_fused']:.2f}", f"{f['y_fused']:.2f}",
                            str(f['have_x']), str(f['have_y'])]
                    csvf.write(','.join(row) + '\n')
                    csvf.flush()

                if args.wave and (now - t_wave >= 1.0):
                    t_wave = now
                    idx = f['off'].get(args.wave)
                    if idx is not None:
                        start = sum(n for _, n in EDGES[:idx])
                        n = EDGES[idx][1]
                        vals = f['raw'][start:start + n].astype(float) / 4095.0
                        print(f"[wave] {args.wave}: " + ' '.join(f"{v:.3f}" for v in vals))

                if args.all or (now - t_print >= 0.2):
                    t_print = now
                    if now - t_fps >= 1.0:
                        fps = frames / (now - t_fps)
                        frames = 0
                        t_fps = now
                    print(f"FPS={fps:4.1f} " + fmt_edges(f['cents'], ['up', 'in_up', 'right', 'in_right']))
                    print(f"        out=({f['x']:7.1f},{f['y']:7.1f}) v={f['valid']} | "
                          f"cross=({f['x_line']:7.1f},{f['y_line']:7.1f}) "
                          f"hx={f['have_x']} hy={f['have_y']} | fused=({f['x_fused']:.1f},{f['y_fused']:.1f})")
    except KeyboardInterrupt:
        print("\n[结束]")
    finally:
        if csvf:
            csvf.close()
            print(f"[已保存] {args.csv}")
        ser.close()


if __name__ == '__main__':
    main()
