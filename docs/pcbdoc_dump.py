import struct, sys, re, collections, os

PATH = r"C:\Users\a\Desktop\Pro\Laser_communication\HardWare\Serveo\ADC64 for mcu g\ADC64 for mcu g.PcbDoc"
OUT = r"C:\Users\a\Desktop\Pro\Laser_communication\Software\Severo\docs\adc64_pcbdoc_report.txt"

data = open(PATH, "rb").read()
assert data[:8] == bytes.fromhex("D0CF11E0A1B11AE1"), "不是 OLE 复合文件"

sect_shift = struct.unpack_from("<H", data, 30)[0]
mini_shift = struct.unpack_from("<H", data, 32)[0]
SECT = 1 << sect_shift
MINI = 1 << mini_shift
num_fat = struct.unpack_from("<I", data, 44)[0]
dir_start = struct.unpack_from("<I", data, 48)[0]
mini_start = struct.unpack_from("<I", data, 60)[0]
num_mini = struct.unpack_from("<I", data, 64)[0]
difat_start = struct.unpack_from("<I", data, 68)[0]
num_difat = struct.unpack_from("<I", data, 72)[0]

def sector_off(s):
    return (s + 1) * SECT

# DIFAT -> FAT 扇区列表
difat = list(struct.unpack_from("<109I", data, 76))
s = difat_start
for _ in range(num_difat):
    if s >= 0xFFFFFFFA: break
    off = sector_off(s)
    vals = struct.unpack_from("<%dI" % (SECT // 4), data, off)
    difat.extend(vals[:-1])
    s = vals[-1]

fat = []
for fs in difat[:num_fat]:
    if fs >= 0xFFFFFFFA: continue
    fat.extend(struct.unpack_from("<%dI" % (SECT // 4), data, sector_off(fs)))

def chain(start):
    out = []
    s = start
    while s < 0xFFFFFFFA and len(out) < 100000:
        out.append(s)
        if s >= len(fat): break
        s = fat[s]
    return out

def read_stream(start, size):
    if size == 0: return b""
    if size < 4096:
        # mini stream
        mini_fat = []
        for ms in chain(mini_start)[:num_mini]:
            mini_fat.extend(struct.unpack_from("<%dI" % (SECT // 4), data, sector_off(ms)))
        root_chain = chain(dirent_stat[0])
        rootdata = b"".join(data[sector_off(x):sector_off(x) + SECT] for x in root_chain)
        chunks = []
        s = start
        while s < 0xFFFFFFFA:
            chunks.append(rootdata[s * MINI:(s + 1) * MINI])
            if s >= len(mini_fat): break
            s = mini_fat[s]
        return b"".join(chunks)[:size]
    return b"".join(data[sector_off(x):sector_off(x) + SECT] for x in chain(start))[:size]

# 目录项（带层级树）
dirents = []
dirent_stat = (0, 0)
ENT = []
for ds in chain(dir_start):
    off = sector_off(ds)
    for i in range(SECT // 128):
        e = off + i * 128
        nlen = struct.unpack_from("<H", data, e + 64)[0]
        name = data[e:e + nlen].decode("utf-16le", "ignore").rstrip("\x00") if nlen else ""
        typ = data[e + 66]
        left  = struct.unpack_from("<I", data, e + 68)[0]
        right = struct.unpack_from("<I", data, e + 72)[0]
        child = struct.unpack_from("<I", data, e + 76)[0]
        start = struct.unpack_from("<I", data, e + 116)[0]
        size = struct.unpack_from("<Q", data, e + 120)[0]
        ENT.append((name, typ, left, right, child, start, size))
        if typ == 5:
            dirent_stat = (start, size)

def walk(idx, prefix, depth=0):
    if idx >= len(ENT) or idx == 0xFFFFFFFF or depth > 8: return
    name, typ, left, right, child, start, size = ENT[idx]
    walk(left, prefix, depth + 1)
    if name:
        dirents.append((prefix + name, typ, start, size))
        if (typ == 1) or (typ == 5):
            walk(child, prefix + name + "/", depth + 1)
    walk(right, prefix, depth + 1)

walk(0, "")

out = open(OUT, "w", encoding="utf-8")
def w(*a): print(*a, file=out)

w("OLE 流清单：")
for name, typ, start, size in dirents:
    if typ == 2:
        w(f"  {name:24s} size={size}")

streams = {}
for n, t, st, sz in dirents:
    if t == 2:
        streams[n] = read_stream(st, sz)
        parts = n.split("/")
        if len(parts) >= 3:
            streams.setdefault(parts[-2], streams[n])

def strings(buf, minlen=4):
    return [m.group().decode("latin-1") for m in re.finditer(rb"[\x20-\x7E]{%d,}" % minlen, buf)]

# 1) 网络名：Nets6 里的 NAME=xxx
w("\n=== 网络名 ===")
netnames = collections.Counter()
for sname in ("Nets6", "Nets"):
    if sname in streams:
        for s in strings(streams[sname], 2):
            for m in re.finditer(r"NAME=([^|]+)", s):
                netnames[m.group(1)] += 1
if not netnames:
    for s in strings(streams.get("Nets6", b""), 2):
        netnames[s] += 1
for k, v in sorted(netnames.items()):
    w(f"  {k}")

# 2) 位号与型号
w("\n=== 元件（位号 / 型号 / 封装）===")
comp = {}
for s in strings(streams.get("Components6", b""), 4):
    if "|" in s:
        d = dict(kv.split("=", 1) for kv in s.split("|") if "=" in kv)
        des = d.get("SOURCEDESIGNATOR") or d.get("DESIGNATOR") or ""
        if des:
            comp[des] = (d.get("SOURCECOMPONENTLIBRARY", ""), d.get("SOURCEFOOTPRINTLIBRARY", ""), d.get("COMMENT", ""))
for k in sorted(comp):
    lib, fp, cm = comp[k]
    w(f"  {k:8s} {cm:20s} {fp:30s} {lib}")
w(f"  （共 {len(comp)} 个）")

# 3) 焊盘上的网络 -> 按网络统计引脚数
w("\n=== 网络引脚数（Pads6）===")
netpins = collections.Counter()
padinfo = []
for s in strings(streams.get("Pads6", b""), 4):
    if "|" in s:
        d = dict(kv.split("=", 1) for kv in s.split("|") if "=" in kv)
        net = d.get("NET")
        if net:
            netpins[net] += 1
            padinfo.append((d.get("COMPONENT", ""), d.get("NAME", ""), net,
                            d.get("X", ""), d.get("Y", "")))
for k, v in netpins.most_common(200):
    w(f"  {k:24s} {v}")

out.close()
print("done ->", OUT)
