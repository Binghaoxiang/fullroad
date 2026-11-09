#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slide5_3d.py — 为 PPT 第 5 页生成的 3D 入射角几何示意

画面构成：
  左：一个倾斜的 3D 场景 —— 膜平面、激光束、法线、入射角 θ、光斑投影椭圆
  右：三个 θ 值下的投影形状对比 + 投影长轴公式
"""

import os
import math
from PIL import Image, ImageDraw, ImageFont

W, H = 1600, 660
BG = (255, 255, 255)
C_PLANE_F = (222, 236, 246)
C_PLANE_E = (58, 124, 165)
C_BEAM = (0, 176, 214)
C_NORM = (150, 160, 175)
C_SPOT = (226, 60, 52)
C_TXT = (28, 34, 46)
C_DIM = (110, 120, 134)
C_GRID = (218, 226, 236)
C_ACC = (224, 108, 30)


def font(sz, bold=False):
    names = (["msyhbd.ttc", "msyh.ttc", "simhei.ttf"] if bold
             else ["msyh.ttc", "simhei.ttf", "arial.ttf"])
    for n in names:
        p = os.path.join("C:/Windows/Fonts/", n)
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, sz)
            except Exception:
                pass
    return ImageFont.load_default()


F_T = font(30, True)
F_S = font(21, True)
F_M = font(17)
F_N = font(15)


def proj(x, y, z, ox, oy, k=1.0):
    """等轴测投影：X 右，Y 向内右上，Z 上"""
    return (ox + (x - y * 0.52) * k, oy + (y * 0.40 - z) * k)


def draw_scene(img, dr, ox, oy):
    """主 3D 场景：斜入射"""
    PW, PH = 430, 200
    th = math.radians(34)          # 入射角
    k = 1.0

    # ---------- 膜平面 ----------
    q = [proj(-PW/2, -PH/2, 0, ox, oy, k),
         proj(PW/2, -PH/2, 0, ox, oy, k),
         proj(PW/2, PH/2, 0, ox, oy, k),
         proj(-PW/2, PH/2, 0, ox, oy, k)]
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(ov)
    od.polygon(q, fill=C_PLANE_F + (255,), outline=C_PLANE_E + (255,))
    img.alpha_composite(ov)
    dr = ImageDraw.Draw(img)
    dr.line(q + [q[0]], fill=C_PLANE_E, width=3)

    # 平面上的网格（增强立体感）
    for i in range(1, 6):
        gx = -PW/2 + PW * i / 6
        dr.line([proj(gx, -PH/2, 0, ox, oy, k),
                 proj(gx, PH/2, 0, ox, oy, k)], fill=C_GRID, width=1)
    for j in range(1, 4):
        gy = -PH/2 + PH * j / 4
        dr.line([proj(-PW/2, gy, 0, ox, oy, k),
                 proj(PW/2, gy, 0, ox, oy, k)], fill=C_GRID, width=1)

    # ---------- 探测器（沿 X 一排）----------
    ND = 15
    for i in range(ND):
        x = -PW/2 * 0.72 + i * (PW * 0.72 * 2) / (ND - 1)
        p = proj(x, -PH/2 * 0.80, 0, ox, oy, k)
        dr.ellipse([p[0]-5, p[1]-5, p[0]+5, p[1]+5], fill=(120, 132, 150))

    # ---------- 光斑投影（椭圆，长轴沿 X）----------
    a = PW * 0.105 / math.cos(th)
    b = PW * 0.085
    pts = []
    for i in range(80):
        ang = 2 * math.pi * i / 80
        pts.append(proj(a * math.cos(ang), b * math.sin(ang), 0, ox, oy, k))
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(ov)
    od.polygon(pts, fill=C_SPOT + (90,))
    img.alpha_composite(ov)
    dr = ImageDraw.Draw(img)
    dr.line(pts + [pts[0]], fill=C_SPOT, width=4)

    # ---------- 法线（虚线）----------
    n0 = proj(0, 0, 0, ox, oy, k)
    n1 = proj(0, 0, 340, ox, oy, k)
    y = n0[1]
    while y > n1[1]:
        dr.line([(n0[0], y), (n0[0], max(y - 11, n1[1]))], fill=C_NORM, width=2)
        y -= 19

    # ---------- 激光束 ----------
    L = 250
    dx, dz = math.sin(th), math.cos(th)
    top = proj(-dx * L, 0, dz * L, ox, oy, k)
    half = PW * 0.075
    for s in (-1, 1):
        oxs = s * half * math.cos(th)
        ozs = -s * half * math.sin(th)
        b2 = proj(oxs, 0, ozs, ox, oy, k)
        t2 = proj(-dx * L + oxs, 0, dz * L + ozs, ox, oy, k)
        dr.line([b2, t2], fill=C_BEAM, width=2)
    dr.line([n0, top], fill=C_BEAM, width=7)
    # 箭头
    dr.line([top, (top[0] + 15, top[1] + 20)], fill=C_BEAM, width=6)
    dr.line([top, (top[0] - 12, top[1] + 22)], fill=C_BEAM, width=6)

    # ---------- 角度弧 + 标注 ----------
    r = 118
    arc = []
    for i in range(41):
        t = th * i / 40
        arc.append((n0[0] + r * math.sin(t), n0[1] - r * math.cos(t)))
    dr.line(arc, fill=C_ACC, width=3)
    mid = th / 2
    lx = n0[0] + (r + 30) * math.sin(mid)
    ly = n0[1] - (r + 30) * math.cos(mid)
    dr.text((lx, ly - 12), "θ", font=F_T, fill=C_ACC)

    # ---------- 文字标注 ----------
    dr.text((ox - PW/2 - 30, oy + PH * 0.40 + 40), "膜平面（探测器阵列）",
            font=F_M, fill=C_PLANE_E)
    dr.text((n1[0] + 8, n1[1] - 6), "法线", font=F_N, fill=C_NORM)
    dr.text((top[0] + 18, top[1] + 8), "激光束", font=F_M, fill=C_BEAM)
    dr.text((n0[0] + a * 0.55, n0[1] + 22), "光斑投影", font=F_N, fill=C_SPOT)

    # 平面高度标注
    dr.text((ox - PW/2 - 4, oy + 8), "D", font=F_N, fill=C_PLANE_E)


def draw_compare(img, dr, bx, by):
    """右侧：三个角度的投影形状对比"""
    dr.text((bx, by), "投影形状随 θ 变化", font=F_S, fill=C_TXT)

    cases = [(0, "θ = 0°", "圆  D", (46, 139, 87)),
             (20, "θ = 20°", "椭圆  1.06·D", C_ACC),
             (40, "θ = 40°", "椭圆  1.31·D", C_SPOT)]

    cy = by + 88
    for th, label, shape, col in cases:
        # 俯视图：椭圆
        ew = 120 + th * 2.0
        eh = 66
        cx = bx + 96
        dr.ellipse([cx - ew/2, cy - eh/2, cx + ew/2, cy + eh/2],
                   fill=tuple(list(col) + []) if isinstance(col, tuple) else col,
                   outline=col)
        # 用半透明填充效果：画实心浅色 + 边框
        dr.ellipse([cx - ew/2, cy - eh/2, cx + ew/2, cy + eh/2],
                   fill=None, outline=col, width=3)
        # 直径标注线
        dr.line([cx - ew/2, cy, cx + ew/2, cy], fill=col, width=1)
        dr.text((bx + 210, cy - 22), label, font=F_M, fill=C_TXT)
        dr.text((bx + 210, cy + 2), shape, font=F_N, fill=col)
        cy += 122

    # 公式
    fy = cy + 6
    dr.rectangle([bx, fy, bx + 430, fy + 54], fill=(253, 243, 231), outline=C_ACC)
    dr.text((bx + 14, fy + 8), "投影长轴  L = D / cos θ", font=F_M, fill=C_ACC)
    dr.text((bx + 14, fy + 30), "每个探测器接收能量 ∝ cos θ", font=F_N, fill=C_DIM)


def main():
    img = Image.new("RGBA", (W, H), BG + (255,))
    dr = ImageDraw.Draw(img)

    # 主场景
    draw_scene(img, dr, 470, 400)

    # 右侧对比
    draw_compare(img, dr, 1060, 70)

    # 标题
    dr.text((40, 26), "斜入射的几何关系", font=F_T, fill=C_TXT)
    dr.text((40, 66), "激光与法线的夹角 θ 决定光斑投影的长短",
            font=F_M, fill=C_DIM)

    out = "docs/slide5_3d.png"
    os.makedirs("docs", exist_ok=True)
    img.convert("RGB").save(out, quality=95)
    print("saved:", out)


if __name__ == "__main__":
    main()
