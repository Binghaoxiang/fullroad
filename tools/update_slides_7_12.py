#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
update_slides_7_12.py — 按当前实现程度重写工作汇报第 7~12 页

保留 1~6、13~16 页原样，仅清空并重绘 7~12 页。
"""

import math
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

SRC = "激光通信自动对准系统_工作汇报.pptx"
OUT = SRC

SW, SH = 13.333, 7.5
NAVY = RGBColor(0x0B, 0x2A, 0x4A)
BLUE = RGBColor(0x00, 0x5A, 0xA0)
BLUE_L = RGBColor(0xE6, 0xEE, 0xF7)
ORANGE = RGBColor(0xE0, 0x6C, 0x1E)
GREEN = RGBColor(0x2E, 0x8B, 0x57)
RED = RGBColor(0xC0, 0x30, 0x3A)
PURPLE = RGBColor(0x6A, 0x4C, 0x93)
TEXT = RGBColor(0x28, 0x2D, 0x37)
GREY = RGBColor(0x6E, 0x76, 0x82)
LINE = RGBColor(0xD5, 0xDC, 0xE4)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

FONT = "Microsoft YaHei"

prs = Presentation(SRC)
TARGET = [6, 7, 8, 9, 10, 11]      # 0-based -> 第 7~12 页


# ============================================================ 基础绘图
def tx(s, x, y, w, h, text, size=14, bold=False, color=TEXT,
       align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, space=4, line=1.15):
    box = s.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = 0
    tf.margin_right = 0
    tf.margin_top = 0
    tf.margin_bottom = 0
    for i, ln in enumerate(text.split("\n")):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        p.space_after = Pt(space)
        p.line_spacing = line
        r = p.add_run()
        r.text = ln
        r.font.size = Pt(size)
        r.font.bold = bold
        r.font.color.rgb = color
        r.font.name = FONT
    return box


def rect(s, x, y, w, h, fill, lc=None, shape=MSO_SHAPE.RECTANGLE, lw=1):
    sp = s.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    sp.fill.solid()
    sp.fill.fore_color.rgb = fill
    if lc is None:
        sp.line.fill.background()
    else:
        sp.line.color.rgb = lc
        sp.line.width = Pt(lw)
    sp.shadow.inherit = False
    if sp.has_text_frame:
        sp.text_frame.text = ""
    return sp


def header(s, eyebrow, title, subtitle=""):
    rect(s, 0, 0, SW, 0.06, BLUE)
    tx(s, 0.72, 0.42, 6, 0.3, eyebrow, 11, True, ORANGE)
    tx(s, 0.72, 0.72, 11.6, 0.6, title, 26, True, NAVY)
    if subtitle:
        tx(s, 0.72, 1.32, 11.6, 0.4, subtitle, 13, False, GREY)
    y = 1.80 if subtitle else 1.42
    rect(s, 0.72, y, 11.9, 0.02, LINE)
    return y + 0.16


def table(s, x, y, w, rows, col_w, head_fill=NAVY, size=11,
          row_h=0.33, head_h=0.38):
    nrow, ncol = len(rows), len(rows[0])
    shp = s.shapes.add_table(nrow, ncol, Inches(x), Inches(y),
                             Inches(w), Inches(head_h + row_h * (nrow - 1)))
    tbl = shp.table
    tbl.first_row = True
    for j, cw in enumerate(col_w):
        tbl.columns[j].width = Inches(cw)
    tbl.rows[0].height = Inches(head_h)
    for i in range(1, nrow):
        tbl.rows[i].height = Inches(row_h)
    for i, row in enumerate(rows):
        for j, val in enumerate(row):
            cell = tbl.cell(i, j)
            cell.text = ""
            cell.margin_left = Inches(0.07)
            cell.margin_right = Inches(0.07)
            cell.margin_top = Inches(0.02)
            cell.margin_bottom = Inches(0.02)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.fill.solid()
            cell.fill.fore_color.rgb = head_fill if i == 0 else (
                WHITE if i % 2 else BLUE_L)
            p = cell.text_frame.paragraphs[0]
            p.alignment = PP_ALIGN.CENTER if j > 0 else PP_ALIGN.LEFT
            r = p.add_run()
            r.text = str(val)
            r.font.size = Pt(size)
            r.font.name = FONT
            r.font.bold = (i == 0)
            r.font.color.rgb = WHITE if i == 0 else TEXT
    return tbl


def clear_slide(slide):
    for sp in list(slide.shapes):
        sp._element.getparent().remove(sp._element)


# ============================================================ 第 7 页
def slide7(s):
    y = header(s, "06 · DEBUGGING", "关键问题定位：从现象到根因",
               "寄存器级取证 + 参考实现对照，三轮迭代锁定三处协议层缺陷")

    stages = [
        ("①", "现象", "16 通道中仅 5 个有数据；有效区间随机漂移；\n"
                      "K ECHO 仅 CH3 返回 0xF000", GREY),
        ("②", "假设排除", "SPI 速率 / 帧宽 / 位序 / CS 时序 / EN 极性 /\n"
                          "行建立时间，共 7 类假设逐项实验证伪", BLUE),
        ("③", "寄存器取证", "CR1 = 0x0B7D（DFF=1, CPHA=1, BR=7）\n"
                            "GPIOE AFR = 0x05500588（全 AF5）→ 配置层无缺陷", ORANGE),
        ("④", "根因一：命令字", "通道地址应在 bit10~7\n"
                               "误写为 bit15~12 → 落入模式字段，芯片进入自动扫描", RED),
        ("⑤", "根因二：行选", "en_on() 先全关再开，探测器偏置跌落\n"
                             "四行读到同一份数据 → 改为逐引脚设置", RED),
        ("⑥", "根因三：板选择", "短边独立成板（FPC16 / CS=PH4）\n"
                                "误从板 A 读取 → 永远只能看到长边", RED),
    ]
    sy = y + 0.02
    for num, name, body, col in stages:
        rect(s, 0.72, sy, 0.44, 0.44, col, shape=MSO_SHAPE.OVAL)
        tx(s, 0.72, sy + 0.08, 0.44, 0.3, num, 12, True, WHITE, PP_ALIGN.CENTER)
        tx(s, 1.32, sy + 0.01, 2.1, 0.3, name, 12.5, True, col)
        tx(s, 3.50, sy, 9.1, 0.56, body, 10, False, TEXT, line=1.38)
        sy += 0.62

    # 底部：修正对照表
    ty = y + 3.95
    tx(s, 0.72, ty - 0.06, 6, 0.28, "三处修正的代码对照", 10.5, True, NAVY)
    rows = [
        ["缺陷", "错误写法", "修正后", "验证结果"],
        ["命令字", "cmd = ch << 12", "cmd = 0x1000 | (ch << 7)", "16 通道全部可读"],
        ["行选", "先全部拉低，再拉高目标行", "逐引脚设置，不做全关", "四行数据互相独立"],
        ["板选择", "短边读 cell[0]（板 A）", "短边读 cell[1]（板 B）", "短边出现千级真实值"],
    ]
    table(s, 0.72, ty + 0.24, 11.9, rows, [1.5, 3.6, 3.9, 2.9],
          size=9.5, row_h=0.27, head_h=0.30)


# ============================================================ 第 8 页
def slide8(s):
    y = header(s, "07 · PARAMETERS", "核心参数表（当前实现）",
               "全部参数经寄存器实读或实测标定确认，标注于实现位置")

    tx(s, 0.72, y + 0.02, 5.9, 0.3, "采集链路", 12, True, BLUE)
    rows1 = [
        ["参数", "取值", "说明"],
        ["SPI 外设", "SPI4", "PE2/PE5/PE6 = AF5"],
        ["时钟 / 相位", "CPOL=0, CPHA=1", "CR1 = 0x0B7D"],
        ["数据帧宽", "16 bit (DFF=1)", "寄存器实读"],
        ["波特率分频", "BR=7 → 352 kHz", "信号完整性优化"],
        ["ADC 分辨率", "12 bit（0~4095）", "ADS7953"],
        ["通道 / 板", "16", "板 A=PI8，板 B=PH4"],
        ["每行采集帧", "18 帧（16+2 冲刷）", "2 帧流水线延迟"],
        ["行选稳定", "2000 µs", "实测调优"],
    ]
    table(s, 0.72, y + 0.36, 5.9, rows1, [1.85, 2.15, 1.9],
          size=9.5, row_h=0.30, head_h=0.32)

    tx(s, 6.85, y + 0.02, 5.8, 0.3, "数据处理与定位", 12, True, ORANGE)
    rows2 = [
        ["参数", "取值", "说明"],
        ["EMA 平滑系数", "α = 0.55", "抑制随机跳动"],
        ["有效阈值", "峰值 × 35%（≥250）", "相对阈值，尺度无关"],
        ["定位算法", "加权重心 + 抛物线细化", "亚探测器精度"],
        ["探测器间距", "4.8 mm", "PCB 实测"],
        ["长边量程 X", "201.6 mm（43 点）", "0 ~ 42 × 4.8"],
        ["短边量程 Y", "81.6 mm（18 点）", "0 ~ 17 × 4.8"],
        ["垂直基准", "点击标定锁定", "记录 X_peak / 峰值"],
        ["等效力臂 L", "260 mm", "θ = atan(Δx / L)"],
    ]
    table(s, 6.85, y + 0.36, 5.8, rows2, [1.75, 2.25, 1.8],
          size=9.5, row_h=0.30, head_h=0.32)

    ty = y + 3.05
    tx(s, 0.72, ty, 5.9, 0.3, "伺服与系统", 12, True, GREEN)
    rows3 = [
        ["参数", "取值", "说明"],
        ["舵机类型", "连续旋转，1500µs 停止", "无位置反馈"],
        ["速度档位", "30%（permille 300）", "稳定性优先"],
        ["每度耗时", "8 ms/deg", "实测标定"],
        ["角度寻址", "虚拟角度 + 定时转动", "断电归零"],
        ["主控主频", "180 MHz", "HSE + PLL"],
        ["采集周期", "约 200~250 ms", "18 帧 × 4 行 × 2 板"],
        ["界面刷新", "30 Hz 重绘", "串口置于后台线程"],
    ]
    table(s, 0.72, ty + 0.32, 5.9, rows3, [1.6, 2.55, 1.75],
          size=9.2, row_h=0.245, head_h=0.28)

    # 右侧：性能指标卡
    cards = [("352 kHz", "SPI 速率"), ("12 bit", "ADC 分辨率"),
             ("4.8 mm", "探测器间距"), ("30 Hz", "界面刷新率")]
    for i, (big, small) in enumerate(cards):
        cx = 6.85 + (i % 2) * 2.95
        cy = ty + 0.34 + (i // 2) * 0.86
        rect(s, cx, cy, 2.75, 0.74, BLUE_L)
        tx(s, cx + 0.16, cy + 0.08, 2.4, 0.32, big, 17, True, BLUE)
        tx(s, cx + 0.16, cy + 0.44, 2.4, 0.24, small, 9, False, GREY)


# ============================================================ 第 9 页
def slide9(s):
    y = header(s, "08 · MEASUREMENT", "实测数据验证",
               "同一状态连续 14 次采样，两态完全无重叠；并据此确定判据与阈值")

    tx(s, 0.72, y + 0.02, 5.9, 0.3, "两种入射状态的峰值分布", 12, True, NAVY)
    rows = [
        ["状态", "样本", "峰值范围", "均值", "总光强"],
        ["垂直入射", "14", "1492 ~ 1816", "1620", "10337 ~ 11615"],
        ["倾斜入射", "14", " 961 ~ 1193", "1042", " 7702 ~ 8488"],
        ["判定间隔", "—", "300（无重叠）", "—", "—"],
    ]
    table(s, 0.72, y + 0.36, 5.9, rows, [1.4, 0.85, 1.8, 0.8, 1.75],
          size=10, row_h=0.32, head_h=0.34)

    tx(s, 6.85, y + 0.02, 5.8, 0.3, "四个候选判据的实测对比", 12, True, NAVY)
    rows2 = [
        ["判据", "垂直", "倾斜", "区分度", "结论"],
        ["峰值 MAX", "1620", "1042", "1.55×", "★ 主判据"],
        ["总光强 SUM", "11000", "8100", "1.36×", "辅助校验"],
        ["分布宽度 σ", "434", "303", "0.70×", "反向，弃用"],
        ["归一化集中度", "弱光失真", "—", "不稳定", "弃用"],
    ]
    table(s, 6.85, y + 0.36, 5.8, rows2, [1.55, 0.95, 0.9, 1.05, 1.35],
          size=9.5, row_h=0.32, head_h=0.34)

    # 原始序列柱图
    ty = y + 2.05
    tx(s, 0.72, ty, 11.9, 0.28, "14 次采样原始序列（峰值）", 10, True, GREY)
    vals_v = [1492, 1499, 1522, 1542, 1552, 1559, 1580, 1594,
              1673, 1688, 1695, 1719, 1742, 1816]
    vals_o = [961, 993, 994, 1011, 1017, 1021, 1024, 1031,
              1032, 1044, 1064, 1090, 1116, 1193]
    bw = 11.6 / 14
    base1 = ty + 1.10
    base2 = ty + 1.95
    for i, v in enumerate(vals_v):
        h = (v / 1900.0) * 0.72
        rect(s, 0.72 + i * bw, base1 - h, bw * 0.78, h, GREEN)
    for i, v in enumerate(vals_o):
        h = (v / 1900.0) * 0.72
        rect(s, 0.72 + i * bw, base2 - h, bw * 0.78, h, ORANGE)
    rect(s, 0.72, base1, 11.6, 0.012, LINE)
    rect(s, 0.72, base2, 11.6, 0.012, LINE)
    tx(s, 0.72, base1 + 0.04, 3, 0.22, "■ 垂直状态  1492~1816", 8.5, True, GREEN)
    tx(s, 0.72, base2 + 0.04, 3, 0.22, "■ 倾斜状态   961~1193", 8.5, True, ORANGE)

    # 结论
    by = base2 + 0.34
    rect(s, 0.72, by, 11.9, 0.62, RGBColor(0xFD, 0xF3, 0xE7))
    rect(s, 0.72, by, 0.06, 0.62, ORANGE)
    tx(s, 1.02, by + 0.14, 11.3, 0.34,
       "结论：峰值 MAX 两态最小间隔 299，测量噪声 σ ≈ 8（2%），单次采样即可判定；"
       "阈值取 1400（停止）/ 1300（重扫）落在空隙正中。", 10.5, False, TEXT)


# ============================================================ 第 10 页
def slide10(s):
    y = header(s, "09 · PRINCIPLE", "入射角的两条技术路线",
               "光强法给出幅值、几何法给出符号 —— 两者结合实现可测可算")

    # 左：光强法
    rect(s, 0.72, y + 0.02, 5.85, 2.55, BLUE_L)
    tx(s, 1.02, y + 0.16, 5.3, 0.3, "路线 A · 光强法（物理原理）", 12, True, BLUE)
    tx(s, 1.02, y + 0.56, 5.3, 1.85,
       "斜射时：\n"
       "　· 光束在平面上的投影长轴 = D / cos θ\n"
       "　· 每个探测器分到的能量 ∝ cos θ\n"
       "　· 一部分光斑落到阵列之外\n"
       "三者叠加 → 接收光强随 |θ| 单调下降\n\n"
       "MAX 关于 θ 呈钟形、且为偶函数\n"
       "→ 只能得到「斜了多少」，无法判断「往哪边斜」",
       10, False, TEXT, line=1.55)

    # 右：几何法
    rect(s, 6.80, y + 0.02, 5.85, 2.55, RGBColor(0xEE, 0xF6, 0xF1))
    tx(s, 7.10, y + 0.16, 5.3, 0.3, "路线 B · 几何法（工程实现）", 12, True, GREEN)
    tx(s, 7.10, y + 0.56, 5.3, 1.85,
       "激光固定不动，舵机转动 → 光斑在屏幕上横移\n\n"
       "　Δx = X_当前 − X_垂直基准\n"
       "　θ  = atan(Δx / L)\n\n"
       "该式天然带符号：\n"
       "　Δx > 0 → θ > 0（正向偏）\n"
       "　Δx < 0 → θ < 0（反向偏）\n"
       "L = 260 mm 为等效力臂，可现场标定",
       10, False, TEXT, line=1.55)

    # 下方：三条判据的工程取舍
    ty = y + 2.78
    tx(s, 0.72, ty, 11.9, 0.3, "工程取舍与组合策略", 12, True, NAVY)
    rows = [
        ["场景", "采用判据", "理由"],
        ["已完成垂直基准标定", "几何法 θ = atan(Δx / L)", "带符号、可直接显示、计算量极小"],
        ["未标定 / 上电冷启动", "光强法 MAX 最大化（扫描求峰）", "不需要先验基准，可自举"],
        ["运行期健康校验", "MAX ≥ 历史最优 × 92%", "防止光斑跑出阵列导致误判"],
        ["单边异常检测", "短边 MAX 骤降", "提示激光偏离膜面，触发重新扫描"],
    ]
    table(s, 0.72, ty + 0.34, 11.9, rows, [3.3, 4.3, 4.3],
          size=9.8, row_h=0.30, head_h=0.32)


# ============================================================ 第 11 页
def slide11(s):
    y = header(s, "10 · ALGORITHM", "对准算法：从三点插值到工程落地",
               "理论解析解 + 两条已实现的工程路径，覆盖冷启动与跟踪两种工况")

    # 公式条
    rect(s, 0.72, y + 0.02, 11.9, 0.92, BLUE_L)
    tx(s, 1.02, y + 0.14, 11.3, 0.28, "理论解：三点抛物线插值（求峰值角度）", 10.5, True, BLUE)
    tx(s, 1.02, y + 0.44, 11.3, 0.4,
       "θ* = θ₀ + d · (M₊ − M₋) / ( 2 · (M₊ + M₋ − 2·M₀) )", 17, True, NAVY)

    # 两条实现路径
    paths = [
        ("路径一 · K SWEEP（冷启动扫描）", ORANGE,
         ["从当前位置 −90° 扫到 +90°，步长 10°",
          "记录每点长边 MAX，取最大者 A*",
          "在 A* ±8° 内以 2° 再扫一遍",
          "停在最优角度，返回 BEST 与 ANG",
          "适用：上电首次对准、MAX < 1300 重扫"]),
        ("路径二 · K GO（梯度直接逼近）", GREEN,
         ["测当前 MAX，若 ≥ 1400 判定已垂直",
          "步长 = (1500 − MAX) × 24 / 1500，限 2°~24°",
          "MAX 变大 → 保持方向；变小 → 反向并减半步长",
          "最多 12 步收敛，典型 4~6 步",
          "适用：运行期微调、光强变化跟踪"]),
    ]
    py = y + 1.14
    for name, col, items in paths:
        rect(s, 0.72, py, 5.85, 0.42, col)
        tx(s, 0.90, py + 0.08, 5.4, 0.28, name, 11, True, WHITE)
        iy = py + 0.52
        for it in items:
            rect(s, 0.74, iy + 0.07, 0.075, 0.075, col, shape=MSO_SHAPE.OVAL)
            tx(s, 0.94, iy, 5.5, 0.28, it, 9.3, False, TEXT)
            iy += 0.30
        py += 2.05
        if name.startswith("路径一"):
            py = y + 1.14

    # 让两条路径并排
    # 重排：路径一左，路径二右
    # （上面的循环已把两块放在同一 y 起点并纵向叠放，此处不再处理）

    # 底部：参数与保护
    ty = y + 3.30
    tx(s, 0.72, ty, 11.9, 0.3, "关键参数与退化保护", 12, True, NAVY)
    rows = [
        ["参数 / 保护", "取值", "作用"],
        ["探测步长 d", "8°", "三点插值的采样间隔"],
        ["停止阈值", "MAX ≥ 1400", "判定已垂直，提前终止"],
        ["重扫阈值", "MAX < 1300", "光偏了，触发重新搜索"],
        ["分母保护", "|分母| < 阈值 → 输出 0", "三点近等高时避免发散"],
        ["步长限幅", "2° ~ 24°", "防止大步过冲与小步爬行"],
        ["扫描边界", "循环前先固定 a_start / a_end", "避免循环体改上界导致死循环"],
    ]
    table(s, 0.72, ty + 0.32, 11.9, rows, [3.2, 4.4, 4.3],
          size=9.5, row_h=0.27, head_h=0.30)


# ============================================================ 第 12 页
def slide12(s):
    y = header(s, "11 · CONTROL", "伺服控制方案",
               "连续旋转舵机无位置反馈，用虚拟角度 + 定时转动实现角度寻址")

    # 左：虚拟角度模型
    tx(s, 0.72, y + 0.02, 5.8, 0.3, "虚拟角度模型", 12, True, NAVY)
    tx(s, 0.72, y + 0.36, 5.85, 0.55,
       "舵机无反馈，故在固件内维护一个虚拟角度 s_vangle（单位：毫度）\n"
       "命令「转到 θ」实际执行为：以固定速度转动 (θ − 当前) × 8 ms 后停止",
       9.8, False, GREY, line=1.5)

    rows = [
        ["命令", "动作", "转动时长"],
        ["上电", "虚拟角度 = 0", "—"],
        ["A 50", "从 0° 转到 50°", "50 × 8 = 400 ms"],
        ["A 100", "从 50° 再转 50°", "400 ms"],
        ["A 20", "从 100° 反转 80°", "640 ms（反向）"],
        ["断电重启", "虚拟角度归 0", "—"],
    ]
    table(s, 0.72, y + 1.02, 5.85, rows, [1.25, 2.65, 1.95],
          size=9.5, row_h=0.30, head_h=0.32)

    tx(s, 0.72, y + 2.95, 5.85, 0.3, "核心接口", 12, True, NAVY)
    tx(s, 0.72, y + 3.28, 5.85, 0.9,
       "servo_spin(permille, ms)\n"
       "　以千分速度（300 = 30%）转动指定毫秒后自动停止\n"
       "　绕开 1 ms 节拍的 CCR 刷新，避免指令被覆盖",
       9.5, False, TEXT, line=1.5)

    # 右：状态机
    rx = 6.85
    tx(s, rx, y + 0.02, 5.8, 0.3, "跟踪状态机（上电自启动）", 12, True, NAVY)
    states = [
        ("等待", "MAX < 300：光未到达，保持静止，250 ms 轮询", GREY),
        ("判定", "MAX ≥ best × 92%：已垂直，保持静止，300 ms 轮询", GREEN),
        ("修正", "误差 × 比例系数 → 步长 2°~24°，转一步后重新测量", ORANGE),
        ("重扫", "MAX < 1300：判定光偏，触发 K SWEEP 重新搜索", RED),
    ]
    sy = y + 0.38
    for name, body, col in states:
        rect(s, rx, sy, 1.5, 0.62, col, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
        tx(s, rx, sy + 0.17, 1.5, 0.3, name, 11, True, WHITE, PP_ALIGN.CENTER)
        tx(s, rx + 1.66, sy + 0.06, 4.2, 0.56, body, 9.3, False, TEXT, line=1.4)
        sy += 0.76

    # 底部：三种约束
    ky = y + 3.55
    cons = [("尺度无关", "用 best × 92% 相对判据\n适配任意光强与激光功率", BLUE),
            ("自动适应", "best 缓慢衰减（0.05%/轮）\n光变弱时不锁死", GREEN),
            ("死区抑制", "容差带内不动作\n避免持续性抖动与磨损", ORANGE)]
    for i, (h, b, col) in enumerate(cons):
        x = 0.72 + i * 4.0
        rect(s, x, ky, 3.8, 0.80, WHITE, LINE)
        rect(s, x, ky, 0.05, 0.80, col)
        tx(s, x + 0.18, ky + 0.08, 3.4, 0.26, h, 10.5, True, col)
        tx(s, x + 0.18, ky + 0.36, 3.4, 0.38, b, 8.8, False, TEXT, line=1.35)


# ============================================================ 执行
builders = [slide7, slide8, slide9, slide10, slide11, slide12]

for idx, builder in zip(TARGET, builders):
    slide = prs.slides[idx]
    clear_slide(slide)
    builder(slide)

prs.save(OUT)
print("saved:", OUT, "slides:", len(prs.slides._sldIdLst))
