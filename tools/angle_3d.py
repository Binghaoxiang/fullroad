#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
angle_3d.py — 用 PIL 手绘"入射角 → 光斑投影 → 接收光强"的 3D 示意图

输出 docs/angle_3d.png，包含：
  ① 三个不同入射角的 3D 透视场景
  ② MAX 随角度的钟形曲线 + 三点插值求峰顶
"""

import os
import math
from PIL import Image, ImageDraw, ImageFont

W, H = 1600, 1000
BG = (250, 251, 253)

# 调色
C_PLANE = (58, 124, 165)
C_BEAM = (0, 190, 230)
C_SPOT = (235, 60, 50)
C_DET = (255, 210, 90)
C_DET_OFF = (200, 205, 215)
C_TEXT = (30, 36, 48)
C_GRID = (215, 222, 232)
C_CURVE = (0, 120, 200)
C_STAR = (255, 140, 0)


def font(sz, bold=False):
    names = ["msyhbd.ttc", "msyh.ttc", "simhei.ttf", "arialbd.ttf", "arial.ttf"]
    for n in names:
        for d in ("C:/Windows/Fonts/", ""):
            p = os.path.join(d, n)
            if os.path.exists(p):
                try:
                    return ImageFont.truetype(p, sz)
                except Exception:
                    pass
    return ImageFont.load_default()


F_T = font(30, True)
F_S = font(20, True)
F_M = font(17)
F_N = font(15)


def project(x, y, z, ox, oy, sc=1.0):
    """简易等距透视：X 向右，Y 向内右上，Z 向上"""
    px = ox + (x - y * 0.55) * sc
    py = oy + (y * 0.42 - z) * sc
    return (px, py)


def draw_scene(img, dr, theta_deg, ox, oy, title, sc=1.0):
    th = math.radians(theta_deg)
    PW, PH = 360, 170          # 平面宽、深

    # ---------- 平面（平行四边形）----------
    quad = [project(-PW/2, -PH/2, 0, ox, oy, sc),
            project(PW/2, -PH/2, 0, ox, oy, sc),
            project(PW/2, PH/2, 0, ox, oy, sc),
            project(-PW/2, PH/2, 0, ox, oy, sc)]
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    od.polygon(quad, fill=C_PLANE + (46,), outline=C_PLANE + (200,))
    img.alpha_composite(overlay)
    dr = ImageDraw.Draw(img)
    dr.line(quad + [quad[0]], fill=C_PLANE, width=3)

    # ---------- 探测器（沿 X 一排）----------
    NDET = 15
    for i in range(NDET):
        x = -PW/2*0.86 + i*(PW*0.86*2)/(NDET-1)
        p = project(x, -PH/2*0.86, 0, ox, oy, sc)
        dr.ellipse([p[0]-4, p[1]-4, p[0]+4, p[1]+4], fill=(120, 130, 150))

    # ---------- 光斑投影（椭圆）----------
    a = PW*0.115 / max(math.cos(th), 0.42)     # 长半轴（沿 X）
    b = PW*0.115                                # 短半轴（沿 Y）
    pts = []
    for k in range(72):
        ang = 2*math.pi*k/72
        pts.append(project(a*math.cos(ang), b*math.sin(ang), 0, ox, oy, sc))
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    od.polygon(pts, fill=C_SPOT + (70,))
    img.alpha_composite(overlay)
    dr = ImageDraw.Draw(img)
    dr.line(pts + [pts[0]], fill=C_SPOT, width=4)

    # ---------- 被照亮的探测器（亮度 ∝ 1/cosθ）----------
    inten = min(1.0, math.cos(th) / 1.0)
    for i in range(NDET):
        x = -PW/2*0.86 + i*(PW*0.86*2)/(NDET-1)
        if abs(x) <= a:
            v = inten * (1 - 0.45*(abs(x)/max(a, 1e-6))**2)
            r = 5 + 8*v
            p = project(x, -PH/2*0.86, 0, ox, oy, sc)
            col = (int(255), int(120 + 120*v), int(40 + 30*(1-v)))
            dr.ellipse([p[0]-r, p[1]-r, p[0]+r, p[1]+r], fill=col)

    # ---------- 激光束 ----------
    L = 330
    dx, dz = math.sin(th), math.cos(th)
    top = project(-dx*L, 0, dz*L, ox, oy, sc)
    base = project(0, 0, 0, ox, oy, sc)
    half = PW*0.115
    for s in (-1, 1):
        oxs = s*half*math.cos(th)
        ozs = -s*half*math.sin(th)
        t2 = project(-dx*L + oxs, 0, dz*L + ozs, ox, oy, sc)
        b2 = project(oxs, 0, ozs, ox, oy, sc)
        dr.line([b2, t2], fill=C_BEAM, width=2)
    dr.line([base, top], fill=C_BEAM, width=6)
    # 箭头
    dr.line([top, (top[0]+12, top[1]+16)], fill=C_BEAM, width=5)
    dr.line([top, (top[0]-12, top[1]+16)], fill=C_BEAM, width=5)

    # ---------- 法线（虚线）----------
    ntop = project(0, 0, L*0.85, ox, oy, sc)
    y = base[1]
    while y > ntop[1]:
        dr.line([(base[0], y), (base[0], y-9)], fill=(160, 168, 180), width=2)
        y -= 16

    # ---------- 角度弧 ----------
    if theta_deg > 1:
        arc = []
        for k in range(0, 26):
            aa = math.radians(90 - theta_deg*k/25)
            arc.append(project(70*math.cos(aa)*0, 0, 0, ox, oy, sc))
        # 用简单文字标注代替弧线
    dr.text((base[0]+14, base[1]-70), "θ=%d°" % theta_deg,
            font=F_S, fill=(200, 40, 30))

    # ---------- 标题 ----------
    tw = dr.textlength(title, font=F_M)
    dr.text((ox - tw/2, oy + 150*sc), title, font=F_M, fill=C_TEXT)


def draw_bell(img, dr, bx, by, bw, bh):
    """钟形曲线 + 三点插值"""
    dr.text((bx, by-44), "MAX 随舵机角度 θ 是钟形曲线", font=F_S, fill=C_TEXT)

    # 坐标轴
    ax_y = by + bh
    dr.line([(bx, by), (bx, ax_y)], fill=C_TEXT, width=2)
    dr.line([(bx, ax_y), (bx+bw, ax_y)], fill=C_TEXT, width=2)
    dr.text((bx+bw-70, ax_y+8), "θ (°)", font=F_N, fill=C_TEXT)
    dr.text((bx-40, by-6), "MAX", font=F_N, fill=C_TEXT)

    # 曲线：以 x0 为峰，宽 26°
    peak = 1500.0
    x0 = 26.0                    # 峰顶在 +26°
    def val(t):
        return peak*math.exp(-((t-x0)/20.0)**2)

    def to_px(t):
        px = bx + (t + 50)/100.0*bw
        py = ax_y - val(t)/1650.0*bh
        return (px, py)

    pts = [to_px(-50 + k*100/220) for k in range(221)]
    dr.line(pts, fill=C_CURVE, width=4)

    # 92% 判据线
    y92 = ax_y - peak*0.92/1650.0*bh
    x = bx
    while x < bx+bw:
        dr.line([(x, y92), (x+8, y92)], fill=(150, 155, 165), width=2)
        x += 16
    dr.text((bx+6, y92-24), "92% 判据（已垂直）", font=F_N, fill=(120, 125, 135))

    # 三点采样
    th0, d = 44.0, 8.0
    xs = [th0-d, th0, th0+d]
    ys = [val(t) for t in xs]
    names = ["M−", "M0", "M+"]
    for t, v, nm in zip(xs, ys, names):
        p = to_px(t)
        dr.ellipse([p[0]-8, p[1]-8, p[0]+8, p[1]+8], fill=C_SPOT)
        dr.text((p[0]-10, p[1]-32), nm, font=F_N, fill=C_SPOT)

    # 三点插值
    num = ys[2]-ys[0]
    den = 2*(ys[2]+ys[0]-2*ys[1])
    tstar = th0 + d*num/den if den else th0
    pstar = to_px(tstar)
    # 竖线
    y = pstar[1]
    while y < ax_y:
        dr.line([(pstar[0], y), (pstar[0], y+9)], fill=C_STAR, width=2)
        y += 16
    # 星标
    r = 12
    dr.polygon([(pstar[0], pstar[1]-r), (pstar[0]+r*0.5, pstar[1]-r*0.3),
                (pstar[0]+r, pstar[1]+r*0.2), (pstar[0]+r*0.3, pstar[1]+r*0.4),
                (pstar[0]+r*0.6, pstar[1]+r), (pstar[0], pstar[1]+r*0.5),
                (pstar[0]-r*0.6, pstar[1]+r), (pstar[0]-r*0.3, pstar[1]+r*0.4),
                (pstar[0]-r, pstar[1]+r*0.2), (pstar[0]-r*0.5, pstar[1]-r*0.3)],
               fill=C_STAR)
    dr.text((pstar[0]+16, pstar[1]-46), "峰顶 θ*=%.1f°" % tstar,
            font=F_S, fill=C_STAR)
    dr.text((pstar[0]+16, pstar[1]-20), "→ 垂直位置", font=F_N, fill=C_STAR)

    # 公式
    fy = ax_y + 34
    dr.text((bx, fy), "θ* = θ0 + d × (M+ − M−) / (2 × (M+ + M− − 2·M0))",
            font=F_M, fill=(60, 70, 90))


def main():
    img = Image.new("RGBA", (W, H), BG)
    dr = ImageDraw.Draw(img)

    dr.text((40, 26), "入射角 θ 变化 → 光斑投影拉长 → 探测器接收光减弱",
            font=F_T, fill=C_TEXT)
    dr.text((40, 68), "斜射时能量摊薄且部分光跑出阵列，所以 【垂直 = 接收光最强】",
            font=F_M, fill=(90, 100, 120))

    # 三个场景
    scenes = [(0, "θ = 0°  垂直：光斑是圆，最亮", 290, 400, 1.0),
              (20, "θ = 20°  倾斜：椭圆拉长 1.06×", 760, 400, 1.0),
              (40, "θ = 40°  更斜：拉长 1.31×，变暗", 1230, 400, 1.0)]
    for th, title, ox, oy, sc in scenes:
        draw_scene(img, dr, th, ox, oy, title, sc)

    # 钟形曲线
    draw_bell(img, dr, 120, 640, 780, 250)

    # 右侧说明
    tx, ty = 1000, 640
    lines = [
        ("关键物理", F_S, (30, 36, 48)),
        ("• 光束打在平面上的投影，长轴 = D / cos θ", F_M, (60, 70, 90)),
        ("• θ 越大，同一个光斑摊得越开", F_M, (60, 70, 90)),
        ("• 每个探测器分到的光 ∝ cos θ", F_M, (60, 70, 90)),
        ("• 还有一部分光直接落到阵列外", F_M, (60, 70, 90)),
        ("", F_M, (60, 70, 90)),
        ("为什么一次测量不够", F_S, (30, 36, 48)),
        ("MAX 关于 θ 是偶函数：+20° 和 −20° 一样亮", F_M, (60, 70, 90)),
        ("所以只知道【斜了多少】，不知道【往哪斜】", F_M, (60, 70, 90)),
        ("", F_M, (60, 70, 90)),
        ("解决办法：测三点，抛物线插值", F_S, (30, 36, 48)),
        ("在 θ0−8°、θ0、θ0+8° 各测一次", F_M, (60, 70, 90)),
        ("代入公式直接算出峰顶 θ*", F_M, (60, 70, 90)),
        ("舵机一次转到 θ*，不遍历", F_M, (60, 70, 90)),
        ("", F_M, (60, 70, 90)),
        ("实测数据支撑", F_S, (30, 36, 48)),
        ("垂直：MAX = 1492~1816", F_M, (200, 40, 30)),
        ("倾斜：MAX =  961~1193", F_M, (200, 40, 30)),
        ("两者无重叠，判据可靠", F_M, (200, 40, 30)),
    ]
    for text, f, col in lines:
        if text:
            dr.text((tx, ty), text, font=f, fill=col)
        ty += 27 if f is F_M else 30

    out = "docs/angle_3d.png"
    os.makedirs("docs", exist_ok=True)
    img.convert("RGB").save(out, quality=95)
    print("saved:", out)


if __name__ == "__main__":
    main()
