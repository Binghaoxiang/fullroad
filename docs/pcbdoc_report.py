import struct, re, sys, collections, io

PATH = sys.argv[1]
OUT = sys.argv[2]

data = open(PATH, "rb").read()
if data[:8] != bytes.fromhex("D0CF11E0A1B11AE1"):
    print("不是 OLE 复合文件"); sys.exit(1)

SECT = 1 << struct.unpack_from("<H", data, 30)[0]
MINI = 1 << struct.unpack_from("<H", data, 32)[0]
num_fat = struct.unpack_from("<I", data, 44)[0]
dir_start = struct.unpack_from("<I", data, 48)[0]
mini_start = struct.unpack_from("<I", data, 60)[0]
num_mini = struct.unpack_from("<I", data, 64)[0]
difat_start = struct.unpack_from("<I", data, 68)[0]
num_difat = struct.unpack_from("<I", data, 72)[0]

def off(s): return (s + 1) * SECT

difat = list(struct.unpack_from("<109I", data, 76))
s = difat_start
for _ in range(num_difat):
    if s >= 0xFFFFFFFA: break
    v = struct.unpack_from("<%dI" % (SECT // 4), data, off(s)); difat.extend(v[:-1]); s = v[-1]

fat = []
for fs in difat[:num_fat]:
    if fs >= 0xFFFFFFFA: continue
    fat.extend(struct.unpack_from("<%dI" % (SECT // 4), data, off(fs)))

def chain(st):
    o = []; s = st
    while s < 0xFFFFFFFA and len(o) < 200000:
        o.append(s)
        if s >= len(fat): break
        s = fat[s]
    return o

ENT = []
for ds in chain(dir_start):
    for i in range(SECT // 128):
        e = off(ds) + i * 128
        nlen = struct.unpack_from("<H", data, e + 64)[0]
        name = data[e:e + nlen].decode("utf-16le", "ignore").rstrip("\x00") if 0 < nlen <= 64 else ""
        typ = data[e + 66]
        left, right, child = struct.unpack_from("<III", data, e + 68)
        start = struct.unpack_from("<I", data, e + 116)[0]
        size = struct.unpack_from("<Q", data, e + 120)[0]
        ENT.append((name, typ, left, right, child, start, size))

root_data = b"".join(data[off(x):off(x) + SECT] for x in chain(ENT[0][5]))[:ENT[0][6]]
mini_fat = []
if num_mini:
    for ms in chain(mini_start)[:num_mini]:
        mini_fat.extend(struct.unpack_from("<%dI" % (SECT // 4), data, off(ms)))

def read_stream(start, size):
    if size == 0: return b""
    if size < 4096 and mini_fat:
        ch = []; s = start
        while s < 0xFFFFFFFA:
            ch.append(root_data[s * MINI:(s + 1) * MINI])
            if s >= len(mini_fat): break
            s = mini_fat[s]
        return b"".join(ch)[:size]
    return b"".join(data[off(x):off(x) + SECT] for x in chain(start))[:size]

streams = {}
def walk(idx, prefix, depth=0):
    if idx >= len(ENT) or idx == 0xFFFFFFFF or depth > 8: return
    name, typ, left, right, child, start, size = ENT[idx]
    walk(left, prefix, depth + 1)
    if name:
        full = prefix + name
        if typ == 2: streams[full] = read_stream(start, size)
        if typ in (1, 5): walk(child, full + "/", depth + 1)
    walk(right, prefix, depth + 1)
walk(0, "")

def get(short):
    for k, v in streams.items():
        p = k.split("/")
        if len(p) >= 2 and p[-2] == short: return v
    return b""

def recs(buf):
    """Altium 记录是 '|KEY=VALUE|KEY=VALUE' 文本，二进制长度前缀被剥掉后按 | 分"""
    txt = buf.decode("latin-1", "ignore")
    out = []
    for chunk in re.split(r"[^\x20-\x7E]+", txt):
        if "|" in chunk and "=" in chunk:
            d = {}
            for kv in chunk.split("|"):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    d[k] = v
            if d: out.append(d)
    return out

out = io.open(OUT, "w", encoding="utf-8")
def w(*a): print(*a, file=out)

w("文件:", PATH, " 流数:", len(streams))

# 网络
nets = []
for d in recs(get("Nets6")):
    n = d.get("NAME")
    if n: nets.append(n)
w("\n=== 网络名 (%d) ===" % len(set(nets)))
w("  " + "  ".join(sorted(set(nets))))

# 元件
comps = []
for d in recs(get("Components6")):
    des = d.get("SOURCEDESIGNATOR") or d.get("DESIGNATOR")
    if des:
        comps.append((des, d.get("PATTERN", ""), d.get("SOURCELIBREFERENCE", ""), d.get("X", ""), d.get("Y", ""), d.get("ROTATION", "")))
w("\n=== 元件 (%d) ===" % len(comps))
for c in sorted(comps, key=lambda t: (t[0][:1], len(t[0]), t[0])):
    w("  %-6s %-16s %-24s x=%-10s y=%-10s rot=%s" % c)

# 丝印文字（去掉字体噪声）
texts = []
for m in re.finditer(rb"(?:[\x20-\x7E]\x00){2,60}", get("Texts6")):
    t = m.group().decode("utf-16le", "ignore").strip()
    if t and len(t) > 1: texts.append(t)
w("\n=== 丝印文字 (%d) ===" % len(texts))
w("  " + " | ".join(texts[:200]))

out.close()
print("ok ->", OUT)
