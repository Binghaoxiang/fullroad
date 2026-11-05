import struct, re, collections

PATH = r"C:\Users\a\Desktop\Pro\Laser_communication\HardWare\Serveo\ADC64 for mcu g\ADC64 for mcu g.PcbDoc"
data = open(PATH, "rb").read()

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

def off(s):
    return (s + 1) * SECT

difat = list(struct.unpack_from("<109I", data, 76))
s = difat_start
for _ in range(num_difat):
    if s >= 0xFFFFFFFA:
        break
    vals = struct.unpack_from("<%dI" % (SECT // 4), data, off(s))
    difat.extend(vals[:-1])
    s = vals[-1]

fat = []
for fs in difat[:num_fat]:
    if fs >= 0xFFFFFFFA:
        continue
    fat.extend(struct.unpack_from("<%dI" % (SECT // 4), data, off(fs)))

def chain(st):
    o = []
    s = st
    while s < 0xFFFFFFFA and len(o) < 200000:
        o.append(s)
        if s >= len(fat):
            break
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

root_start, root_size = ENT[0][5], ENT[0][6]
root_data = b"".join(data[off(x):off(x) + SECT] for x in chain(root_start))[:root_size]

mini_fat = []
if num_mini:
    for ms in chain(mini_start)[:num_mini]:
        mini_fat.extend(struct.unpack_from("<%dI" % (SECT // 4), data, off(ms)))

def read_stream(start, size):
    if size == 0:
        return b""
    if size < 4096 and mini_fat:
        chunks = []
        s = start
        while s < 0xFFFFFFFA:
            chunks.append(root_data[s * MINI:(s + 1) * MINI])
            if s >= len(mini_fat):
                break
            s = mini_fat[s]
        return b"".join(chunks)[:size]
    return b"".join(data[off(x):off(x) + SECT] for x in chain(start))[:size]

streams = {}
def walk(idx, prefix, depth=0):
    if idx >= len(ENT) or idx == 0xFFFFFFFF or depth > 8:
        return
    name, typ, left, right, child, start, size = ENT[idx]
    walk(left, prefix, depth + 1)
    if name:
        full = prefix + name
        if typ == 2:
            streams[full] = read_stream(start, size)
        if typ in (1, 5):
            walk(child, full + "/", depth + 1)
    walk(right, prefix, depth + 1)

walk(0, "")

def get(short):
    for k, v in streams.items():
        parts = k.split("/")
        if len(parts) >= 2 and parts[-2] == short:
            return v
    return b""

def readable(b):
    t = b.decode("latin-1", "ignore")
    t = re.sub(r"[^\x20-\x7E]+", " ", t)
    return re.sub(r"\s{2,}", " ", t)

for name in ("Pads6", "Nets6", "Components6", "Texts6", "Board6"):
    b = get(name)
    print("=" * 25, name, "size", len(b))
    print(readable(b)[:1500])
    print()
