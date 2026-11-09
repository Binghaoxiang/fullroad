#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_report.py — 激光通信自动对准系统 工作汇报 PPT

工程结构：
  01 项目背景与目标      02 系统总体架构
  03 硬件拓扑与信号链    04 探测器阵列布局
  05 采集协议设计        06 关键问题定位过程
  07 核心参数表          08 数据验证
  09 入射角判据          10 三点插值算法
  11 舵机控制方案        12 上位机工具
  13 调试方法论          14 后续计划
"""

from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

# ============================================================ 设计系统
SW, SH = 13.333, 7.5
NAVY   = RGBColor(0x0B, 0x2A, 0x4A)
BLUE   = RGBColor(0x00, 0x5A, 0xA0)
BLUE_L = RGBColor(0xE6, 0xEE, 0xF7)
ORANGE = RGBColor(0xE0, 0x6C, 0x1E)
GREEN  = RGBColor(0x2E, 0x8B, 0x57)
RED    = RGBColor(0xC0, 0x30, 0x3A)
TEXT   = RGBColor(0x28, 0x2D, 0x37)
GREY   = RGBColor(0x6E, 0x76, 0x82)
LINE   = RGBColor(0xD5, 0xDC, 0xE4)
WHITE  = RGBColor(0xFF, 0xFF, 0xFF)

FONT = "Microsoft YaHei"

prs = Presentation()
prs.slide_width = Inches(SW)
prs.slide_height = Inches(SH)

_page = [0]


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
    lines = text.split("\n")
    for i, ln in enumerate(lines):
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


def rect(s, x, y, w, h, fill, line_color=None, shape=MSO_SHAPE.RECTANGLE, lw=1):
    sp = s.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    sp.fill.solid()
    sp.fill.fore_color.rgb = fill
    if line_color is None:
        sp.line.fill.background()
    else:
        sp.line.color.rgb = line_color
        sp.line.width = Pt(lw)
    sp.shadow.inherit = False
    if sp.has_text_frame:
        sp.text_frame.text = ""
    return sp


def new_slide(title, eyebrow="", subtitle=""):
    """标准内容页：顶部眉标 + 标题 + 分隔线 + 页脚"""
    s = prs.slides.add_slide(prs.slide_layouts[6])
    rect(s, 0, 0, SW, 0.06, BLUE)

    if eyebrow:
        tx(s, 0.72, 0.42, 6, 0.3, eyebrow, 11, True, ORANGE)

    tx(s, 0.72, 0.72, 11.6, 0.6, title, 26, True, NAVY)

    if subtitle:
        tx(s, 0.72, 1.32, 11.6, 0.4, subtitle, 13, False, GREY)

    y = 1.42 if not subtitle else 1.80
    rect(s, 0.72, y, 11.9, 0.02, LINE)

    # 页脚
    _page[0] += 1
    tx(s, 0.72, SH - 0.52, 6, 0.3,
       "激光通信自动对准系统 · 工作汇报", 9, False, GREY)
    tx(s, SW - 1.4, SH - 0.52, 0.7, 0.3,
       "%02d" % _page[0], 10, True, BLUE, PP_ALIGN.RIGHT)
    return s, y + 0.16


def bullet_block(s, x, y, w, items, size=13, gap=0.42, color=TEXT, dot=BLUE):
    """带色块的要点列表"""
    cy = y
    for it in items:
        if isinstance(it, tuple):
            head, body = it
        else:
            head, body = it, ""
        rect(s, x, cy + 0.07, 0.10, 0.10, dot, shape=MSO_SHAPE.OVAL)
        tx(s, x + 0.24, cy, w - 0.24, 0.36, head, size, True, color)
        if body:
            tx(s, x + 0.24, cy + 0.27, w - 0.24, 0.5, body, size - 1.5, False, GREY)
            cy += gap + 0.24
        else:
            cy += gap
    return cy


def table(s, x, y, w, rows_data, col_w, head_fill=NAVY,
          size=11, row_h=0.33, head_h=0.38):
    nrow = len(rows_data)
    ncol = len(rows_data[0])
    shp = s.shapes.add_table(nrow, ncol, Inches(x), Inches(y),
                             Inches(w), Inches(head_h + row_h * (nrow - 1)))
    tbl = shp.table
    tbl.first_row = True
    for j, cw in enumerate(col_w):
        tbl.columns[j].width = Inches(cw)
    tbl.rows[0].height = Inches(head_h)
    for i in range(1, nrow):
        tbl.rows[i].height = Inches(row_h)

    for i, row in enumerate(rows_data):
        for j, val in enumerate(row):
            cell = tbl.cell(i, j)
            cell.text = ""
            cell.margin_left = Inches(0.07)
            cell.margin_right = Inches(0.07)
            cell.margin_top = Inches(0.03)
            cell.margin_bottom = Inches(0.03)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.fill.solid()
            if i == 0:
                cell.fill.fore_color.rgb = head_fill
            else:
                cell.fill.fore_color.rgb = WHITE if i % 2 else BLUE_L
            p = cell.text_frame.paragraphs[0]
            p.alignment = PP_ALIGN.CENTER if j > 0 else PP_ALIGN.LEFT
            r = p.add_run()
            r.text = str(val)
            r.font.size = Pt(size)
            r.font.name = FONT
            r.font.bold = (i == 0)
            r.font.color.rgb = WHITE if i == 0 else TEXT
    return tbl


# ============================================================ 01 封面
s = prs.slides.add_slide(prs.slide_layouts[6])
rect(s, 0, 0, SW, SH, NAVY)
rect(s, 0, 0, 0.14, SH, BLUE)
rect(s, 7.9, 0, 5.44, SH, RGBColor(0x0E, 0x33, 0x58))

tx(s, 1.0, 1.65, 6.4, 0.4, "WORK REPORT · 2026", 12, True, ORANGE)
tx(s, 1.0, 2.10, 6.6, 1.5,
   "激光通信\n自动对准系统", 40, True, WHITE, line=1.18)
rect(s, 1.0, 3.85, 1.6, 0.05, ORANGE)
tx(s, 1.0, 4.15, 6.4, 0.9,
   "探测器阵列采集 · 入射角判据 · 伺服闭环对准\n"
   "STM32F429IGT6 + 双 ADS7953 + 64 路光电探测器",
   14, False, RGBColor(0xB8, 0xCB, 0xDD), line=1.5)

tx(s, 1.0, 5.85, 5, 0.35, "工作汇报 · 阶段总结", 13, True, WHITE)
tx(s, 1.0, 6.25, 5, 0.35, "软件开发 · 硬件联调 · 算法验证", 11, False, RGBColor(0x8F, 0xA8, 0xBF))

# 右侧参数卡
cards = [("64", "路光电探测器"), ("43 / 18", "长边 / 短边"),
         ("12 bit", "ADC 分辨率"), ("1500+", "垂直态峰值")]
cy = 1.9
for big, small in cards:
    tx(s, 8.5, cy, 4.2, 0.62, big, 30, True, WHITE)
    tx(s, 8.5, cy + 0.62, 4.2, 0.3, small, 11, False, RGBColor(0x8F, 0xA8, 0xBF))
    cy += 1.28

# ============================================================ 02 项目概述
s, y = new_slide("项目概述与工程目标", "01 · OVERVIEW",
                 "从「看得见」到「算得准」再到「调得动」—— 三级递进的工程目标")

col_w = 3.75
goals = [
    ("目标一 · 看得见", "打通 64 路探测器采集链路\n"
     "· 双采集板独立寻址\n· 16 通道 × 4 行扫描\n· 12 位量化、实时光斑成像"),
    ("目标二 · 算得准", "建立入射角度的可测判据\n"
     "· 峰值 / 展宽 / 集中度三指标\n· 实测标定垂直与倾斜分界\n· 三点插值解析求角度"),
    ("目标三 · 调得动", "伺服闭环自动对准\n"
     "· 相对角度定时转动\n· 全行程粗扫 + 局部细扫\n· 上电自启动、无需人工干预"),
]
for i, (h, b) in enumerate(goals):
    x = 0.72 + i * (col_w + 0.32)
    rect(s, x, y + 0.05, col_w, 0.05, BLUE if i == 0 else (ORANGE if i == 1 else GREEN))
    tx(s, x, y + 0.22, col_w, 0.4, h, 15, True, NAVY)
    tx(s, x, y + 0.72, col_w, 2.0, b, 11.5, False, TEXT, line=1.55)

# 底部技术栈
ty = y + 2.95
rect(s, 0.72, ty, 11.9, 1.55, BLUE_L)
tx(s, 1.02, ty + 0.16, 3, 0.3, "技术栈", 12, True, BLUE)
stack = [
    ("主控", "STM32F429IGT6 · 180 MHz · Cortex-M4"),
    ("采集", "ADS7953 × 2（16 通道 / 12 位 / SPI）"),
    ("固件", "Keil MDK AC5 · 寄存器级 CMSIS · 无 HAL"),
    ("上位机", "Python 3.12 + Tkinter → PyInstaller 打包 EXE"),
]
for i, (k, v) in enumerate(stack):
    cx = 1.02 + (i % 2) * 5.85
    cy2 = ty + 0.52 + (i // 2) * 0.46
    tx(s, cx, cy2, 1.1, 0.3, k, 10.5, True, ORANGE)
    tx(s, cx + 1.05, cy2, 4.6, 0.3, v, 10.5, False, TEXT)

# ============================================================ 03 总体架构
s, y = new_slide("系统总体架构", "02 · ARCHITECTURE",
                 "三层解耦：物理感知层 → 数据采集层 → 决策执行层")

layers = [
    ("决策执行层", "对上位机提供数据，对舵机输出动作",
     ["入射角判据计算", "三点插值求最优角", "伺服相对角度控制"], ORANGE, 0.72),
    ("数据采集层", "双板分时复用同一 MISO",
     ["SPI4 主机 / 352 kHz", "CS 独立：PI8 / PH4", "18 帧流水线读 16 通道"], BLUE, 4.85),
    ("物理感知层", "64 路光电探测器构成 L 形阵列",
     ["长边 43 颗：测 X 向分布", "短边 18 颗：测 Y 向分布", "4 条 EN 行选线"], GREEN, 8.98),
]

for name, sub, items, col, x in layers:
    rect(s, x, y + 0.05, 3.62, 0.62, col)
    tx(s, x + 0.16, y + 0.14, 3.3, 0.3, name, 14, True, WHITE)
    tx(s, x + 0.16, y + 0.72, 3.3, 0.5, sub, 10, False, GREY, line=1.35)
    iy = y + 1.22
    for it in items:
        rect(s, x, iy, 3.62, 0.42, WHITE, LINE)
        rect(s, x, iy, 0.045, 0.42, col)
        tx(s, x + 0.18, iy + 0.09, 3.2, 0.3, it, 10.5, False, TEXT)
        iy += 0.52

# 层间箭头
for x in (4.46, 8.59):
    a = s.shapes.add_shape(MSO_SHAPE.ISOSCELES_TRIANGLE,
                           Inches(x), Inches(y + 1.75), Inches(0.28), Inches(0.28))
    a.rotation = 90
    a.fill.solid()
    a.fill.fore_color.rgb = RGBColor(0xB0, 0xBC, 0xC8)
    a.line.fill.background()
    a.shadow.inherit = False

# 底部闭环说明
by = y + 3.25
rect(s, 0.72, by, 11.9, 0.92, BLUE_L)
tx(s, 1.02, by + 0.13, 11.3, 0.3, "闭环数据流", 11, True, BLUE)
tx(s, 1.02, by + 0.45, 11.3, 0.35,
   "光照射 → 64 路采集 → 峰值/展宽判据 → 计算目标角 → 舵机转动 → 重新采集 → 收敛",
   11, False, TEXT)

# ============================================================ 04 硬件拓扑
s, y = new_slide("硬件拓扑与信号链", "03 · HARDWARE",
                 "两块独立采集板、共用一根 MISO，靠片选分时读取")

# 拓扑图（方框 + 连线）
def node(x, y, w, h, title, sub, fill, tcol=WHITE):
    rect(s, x, y, w, h, fill, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
    tx(s, x + 0.12, y + 0.14, w - 0.24, 0.3, title, 11.5, True, tcol)
    if sub:
        tx(s, x + 0.12, y + 0.44, w - 0.24, 0.5, sub, 9, False,
           tcol if tcol == WHITE else GREY, line=1.3)

ny = y + 0.05
node(0.72, ny, 2.35, 0.95, "STM32F429IGT6", "SPI4 主机 · 180 MHz", NAVY)
node(5.0, ny, 3.3, 0.95, "采集板 A（长边）", "ADS7953 · CS=PI8 · FPC12", BLUE)
node(9.0, ny, 3.3, 0.95, "采集板 B（短边）", "ADS7953 · CS=PH4 · FPC16", GREEN)

# 主控 → 两板 的连线（SCLK/SDI/EN 共线）
for tx0 in (5.6, 9.6):
    ln = s.shapes.add_connector(1, Inches(3.10), Inches(ny + 0.30),
                                Inches(tx0 - 0.02), Inches(ny + 0.30))
    ln.line.color.rgb = RGBColor(0x9A, 0xA6, 0xB2)
    ln.line.width = Pt(1.5)

tx(s, 2.55, ny - 0.02, 3.0, 0.26, "SCLK · SDI · EN1-4（并联）", 8.5, False, GREY)

# MISO 共用
rect(s, 3.10, ny + 1.35, 9.4, 0.44, RGBColor(0xF2, 0xE6, 0xD8), LINE)
tx(s, 3.22, ny + 1.44, 9.1, 0.3,
   "SDO / MISO 并联于 H11 第 2 脚 → PE5　（同一时刻只有一块板被选中，避免总线冲突）",
   9.5, False, RGBColor(0x8A, 0x5A, 0x22))

# 下方：信号定义表
th = y + 2.15
tx(s, 0.72, th, 4, 0.3, "关键信号与引脚分配", 13, True, NAVY)
rows = [
    ["信号", "主控引脚", "采集板", "方向", "备注"],
    ["SPI4_SCK", "PE2 (AF5)", "P4.2", "MCU→ADC", "352 kHz"],
    ["SPI4_MOSI/SDI", "PE6 (AF5)", "P4.5", "MCU→ADC", "命令字 16 位"],
    ["SPI4_MISO/SDO", "PE5 (AF5)", "H11.2", "ADC→MCU", "两板并联"],
    ["CS_A", "PI8", "P4.3", "MCU→ADC", "板 A 片选"],
    ["CS_B", "PH4", "P4.3", "MCU→ADC", "板 B 片选"],
    ["EN1-EN4", "PD13/PD12/PH11/PH10", "P4.6-9", "MCU→板", "行选，高有效"],
]
table(s, 0.72, th + 0.38, 11.9, rows,
      [1.55, 2.45, 2.0, 1.25, 1.35], size=10.5, row_h=0.30, head_h=0.34)

# ============================================================ 05 阵列布局
s, y = new_slide("探测器阵列布局与索引映射", "04 · GEOMETRY",
                 "64 颗单排排布，按 PCB X 坐标升序编号，再按裁剪方案切片")

# 左侧：索引公式
rect(s, 0.72, y + 0.05, 5.2, 2.55, BLUE_L)
tx(s, 0.98, y + 0.20, 4.7, 0.3, "索引解码规则（由 PCB 坐标反推）", 12, True, BLUE)
tx(s, 0.98, y + 0.62, 4.7, 0.9,
   "索引 i ∈ [0, 63]（按 X 坐标升序）\n"
   "　　列号 = i / 4　（十六进制）\n"
   "　　行号 = i % 4 + 1",
   12, False, TEXT, line=1.75)
tx(s, 0.98, y + 1.62, 4.7, 0.8,
   "示例：i=21 → 列5 行2 → D25\ni=39 → 列9 行4 → D49",
   11, False, GREY, line=1.6)

# 右侧：两段切片
tx(s, 6.25, y + 0.12, 6.2, 0.3, "裁剪后保留范围", 12, True, NAVY)

seg = [
    ("长边 · 43 颗", "索引 21 ~ 63", "D25 → D4F（左 → 右）", ORANGE,
     "D25 D35 D45 D16 … D1F D2F D3F D4F"),
    ("短边 · 18 颗", "索引 22 ~ 39", "D35 → D49（下 → 上）", BLUE,
     "D35 D45 D16 … D19 D29 D39 D49"),
]
sy = y + 0.55
for name, idx, order, col, lst in seg:
    rect(s, 6.25, sy, 0.05, 0.95, col)
    tx(s, 6.45, sy, 3.0, 0.3, name, 13, True, col)
    tx(s, 9.3, sy, 3.4, 0.3, idx, 11, True, GREY)
    tx(s, 6.45, sy + 0.32, 5.9, 0.28, order, 11, False, TEXT)
    tx(s, 6.45, sy + 0.60, 5.9, 0.28, lst, 9.5, False, GREY)
    sy += 1.12

# 底部：实测验证结论
by = y + 2.80
rect(s, 0.72, by, 11.9, 0.85, RGBColor(0xEE, 0xF6, 0xF1))
tx(s, 1.02, by + 0.12, 11.3, 0.3, "验证结论", 11, True, GREEN)
tx(s, 1.02, by + 0.44, 11.3, 0.35,
   "长边 43 颗：光左右移动时峰值单调扫过位置 4 → 41，顺序正确；"
   "短边 18 颗：峰值随上下移动单调变化，顺序与实物丝印一致。",
   10.5, False, TEXT)

# ============================================================ 06 采集协议
s, y = new_slide("采集协议设计：命令字与流水线", "05 · PROTOCOL",
                 "ADS7953 的通道地址位于 bit10~7，而非 bit15~12 —— 这是全项目的核心修正点")

# 命令字位域图
tx(s, 0.72, y + 0.05, 5, 0.3, "16 位命令字格式", 12, True, NAVY)
bits = [("D15-D12", "0001", "模式", 1.55, NAVY),
        ("D11", "1", "写配置", 0.95, BLUE),
        ("D10-D7", "CH3..CH0", "通道号", 1.95, ORANGE),
        ("D6", "0", "量程", 0.95, GREY),
        ("D5-D0", "—", "保留", 1.0, RGBColor(0xB8, 0xC0, 0xCC))]
bx = 0.72
for name, val, note, w, col in bits:
    rect(s, bx, y + 0.42, w, 0.55, col)
    tx(s, bx, y + 0.47, w, 0.22, val, 10, True, WHITE, PP_ALIGN.CENTER)
    tx(s, bx, y + 0.72, w, 0.22, name, 7.5, False, WHITE, PP_ALIGN.CENTER)
    bx += w

tx(s, 0.72, y + 1.10, 6.4, 0.6,
   "cmd = 0x1000 | ((ch & 0x0F) << 7)", 13, True, ORANGE)
tx(s, 0.72, y + 1.44, 6.4, 0.5,
   "对比：错误写法 ch<<12 会把通道号写入「模式字段」，\n"
   "导致芯片进入无效/自动扫描模式。", 9.5, False, GREY, line=1.5)

# 右侧流水线时序
rx = 6.9
tx(s, rx, y + 0.05, 5.7, 0.3, "18 帧流水线（2 帧延迟）", 12, True, NAVY)
frames = ["F0", "F1", "F2", "F3", "...", "F15", "F16", "F17"]
cmdch = ["CH0", "CH1", "CH2", "CH3", "...", "CH15", "CH0", "CH0"]
dstch = ["—", "—", "CH0", "CH1", "...", "CH13", "CH14", "CH15"]
fy = y + 0.42
for k, (f, c, d) in enumerate(zip(frames, cmdch, dstch)):
    cx = rx + k * 0.72
    rect(s, cx, fy, 0.66, 0.36, BLUE if d != "—" else RGBColor(0xD8, 0xDE, 0xE6))
    tx(s, cx, fy + 0.08, 0.66, 0.22, f, 8, True, WHITE if d != "—" else GREY, PP_ALIGN.CENTER)
    tx(s, cx, fy + 0.42, 0.66, 0.20, c, 7.5, False, TEXT, PP_ALIGN.CENTER)
    tx(s, cx, fy + 0.62, 0.66, 0.20, d, 7.5, True, ORANGE if d != "—" else GREY, PP_ALIGN.CENTER)

tx(s, rx, fy + 0.86, 5.7, 0.24, "↑发往 ADC 的通道", 7.5, False, GREY)
tx(s, rx, fy + 1.06, 5.7, 0.24, "↑本帧 SDO 实际对应的通道", 7.5, False, ORANGE)

# 底部三个关键点
ky = y + 2.55
keys = [("2 帧延迟", "第 n 帧命令的转换结果在第 n+2 帧返回，必须错位取值"),
        ("2 帧冲刷", "末尾补 CH0 两帧，把流水线里剩余结果推出来"),
        ("连续 CS", "每帧独立 CS 低脉冲，单帧 16 个 SCLK，中间不留空")]
for i, (h, b) in enumerate(keys):
    x = 0.72 + i * 4.0
    rect(s, x, ky, 3.8, 0.88, WHITE, LINE)
    rect(s, x, ky, 0.045, 0.88, ORANGE)
    tx(s, x + 0.18, ky + 0.10, 3.45, 0.28, h, 11, True, ORANGE)
    tx(s, x + 0.18, ky + 0.40, 3.45, 0.42, b, 9, False, TEXT, line=1.35)

# ============================================================ 07 问题定位
s, y = new_slide("关键问题定位：从现象到根因", "06 · DEBUGGING",
                 "寄存器级取证 + 参考实现对照，逐层排除，最终锁定协议层错误")

stages = [
    ("现象", "16 个通道中仅 5 个有数据\n有效区间随机漂移\nK ECHO 仅 CH3 返回 0xF000",
     GREY, "①"),
    ("假设与排除", "SPI 速度 / 帧宽度 / 位序\nCS 时序 / EN 极性 / 行建立时间\n共 7 类假设，逐项实验证伪",
     BLUE, "②"),
    ("寄存器取证", "CR1 = 0x0B7D → DFF=1, CPHA=1, BR=7\nGPIOE AFR = 0x05500588 → 全 AF5\n证明配置层无缺陷",
     ORANGE, "③"),
    ("根因锁定", "对照参考工程 ads7953.h\n命令字应为 0x1000|(ch<<7)\n实际写成 ch<<12 → 落入模式字段",
     RED, "④"),
    ("验证", "修正后 16 通道全部可读\n四行数据互相独立\n峰值随光单调移动",
     GREEN, "⑤"),
]
sy = y + 0.08
for name, body, col, num in stages:
    rect(s, 0.72, sy, 0.5, 0.5, col, shape=MSO_SHAPE.OVAL)
    tx(s, 0.72, sy + 0.10, 0.5, 0.3, num, 13, True, WHITE, PP_ALIGN.CENTER)
    tx(s, 1.42, sy + 0.02, 2.0, 0.3, name, 13, True, col)
    tx(s, 3.35, sy, 9.2, 0.62, body, 10, False, TEXT, line=1.4)
    sy += 0.72

# 右下方：错误命令的映射表
ty = y + 3.72
tx(s, 0.72, ty - 0.06, 6, 0.3, "错误命令的实际解码", 10.5, True, NAVY)
rows = [
    ["误发命令", "对应通道", "芯片解析为", "结果"],
    ["0x0000", "CH0", "模式 0000 保持配置", "通道不变"],
    ["0x1000", "CH1", "模式 0001 + 通道0", "永远读 CH0"],
    ["0x2000~0x7000", "CH2~CH7", "模式 0010~0111 保留", "无响应，读 0"],
    ["0x8000~0xF000", "CH8~CH15", "模式 1000~1111 自动扫描", "芯片自走，忽略地址"],
]
table(s, 0.72, ty + 0.26, 11.9, rows, [2.3, 1.8, 4.3, 3.5],
      size=9.5, row_h=0.27, head_h=0.30)

# ============================================================ 08 核心参数
s, y = new_slide("核心参数表", "07 · PARAMETERS",
                 "全部参数经寄存器实读或实测标定确认")

tx(s, 0.72, y + 0.02, 5.9, 0.3, "SPI 与 ADC", 12, True, BLUE)
rows1 = [
    ["参数", "取值", "依据"],
    ["SPI 外设", "SPI4", "引脚复用"],
    ["SCK / MOSI / MISO", "PE2 / PE6 / PE5", "GPIOE AFR = AF5"],
    ["时钟极性相位", "CPOL=0, CPHA=1", "CR1 = 0x0B7D"],
    ["数据帧宽", "16 bit (DFF=1)", "寄存器实读"],
    ["波特率分频", "BR=7 → 352 kHz", "信号完整性优化"],
    ["ADC 分辨率", "12 bit (0~4095)", "ADS7953"],
    ["通道数", "16 / 板", "ADS7953"],
    ["每板采集帧数", "18 帧 / 行", "2 帧流水线"],
    ["行选稳定时间", "2000 µs", "实测调优"],
]
table(s, 0.72, y + 0.36, 5.9, rows1, [2.1, 2.0, 1.8], size=9.5, row_h=0.30, head_h=0.32)

tx(s, 6.85, y + 0.02, 5.8, 0.3, "伺服与系统", 12, True, ORANGE)
rows2 = [
    ["参数", "取值", "依据"],
    ["舵机类型", "连续旋转", "1500µs 停"],
    ["速度档位", "30 % (permille 300)", "稳定性要求"],
    ["每度耗时", "8 ms/deg", "实测标定"],
    ["虚拟角度量程", "±90° 工作区", "机械限位"],
    ["角度分辨率", "1°", "定时精度"],
    ["主控主频", "180 MHz", "HSE + PLL"],
    ["系统节拍", "1 ms (SysTick)", "伺服斜坡"],
    ["帧率", "约 4~8 Hz", "18 帧 × 4 行"],
    ["上位机刷新", "80 ms 轮询", "K BOTH 合并查询"],
]
table(s, 6.85, y + 0.36, 5.8, rows2, [1.9, 2.1, 1.8], size=9.5, row_h=0.30, head_h=0.32)

# ============================================================ 09 数据验证
s, y = new_slide("实测数据验证", "08 · MEASUREMENT",
                 "同一状态连续 14 次采样，两态完全无重叠，判据具备工程可用性")

tx(s, 0.72, y + 0.02, 5.9, 0.3, "两种入射状态的峰值分布", 12, True, NAVY)
rows = [
    ["状态", "样本数", "峰值范围", "均值", "总光强"],
    ["垂直入射", "14", "1492 ~ 1816", "1620", "10337 ~ 11615"],
    ["倾斜入射", "14", "961 ~ 1193", "1042", "7702 ~ 8488"],
    ["判定间隔", "—", "300（无重叠）", "—", "—"],
]
table(s, 0.72, y + 0.36, 5.9, rows, [1.45, 0.9, 1.75, 0.85, 1.75],
      size=10, row_h=0.32, head_h=0.34)

tx(s, 6.85, y + 0.02, 5.8, 0.3, "三个候选判据的对比", 12, True, NAVY)
rows2 = [
    ["判据", "垂直", "倾斜", "区分度", "可用性"],
    ["峰值 MAX", "1620", "1042", "1.55×", "★ 首选"],
    ["总光强 SUM", "11000", "8100", "1.36×", "辅助"],
    ["分布宽度 σ", "434", "303", "0.70×", "反向，弃用"],
    ["归一化集中度", "—", "—", "弱光失真", "弃用"],
]
table(s, 6.85, y + 0.36, 5.8, rows2, [1.5, 0.9, 0.9, 1.2, 1.4],
      size=9.5, row_h=0.32, head_h=0.34)

# 结论条
by = y + 2.05
rect(s, 0.72, by, 11.9, 1.05, RGBColor(0xFD, 0xF3, 0xE7))
rect(s, 0.72, by, 0.06, 1.05, ORANGE)
tx(s, 1.02, by + 0.12, 11.3, 0.3, "结论与设计取舍", 11, True, ORANGE)
tx(s, 1.02, by + 0.44, 11.3, 0.5,
   "① 峰值 MAX 的两种状态最小间隔为 299，测量噪声 σ≈8（2%），单次采样即可判定；\n"
   "② 分布宽度与入射角呈反向关系（斜射时部分光斑落到阵列外，分布反而收窄），不可作为判据。",
   10, False, TEXT, line=1.5)

# 实拍数据条带
ty2 = by + 1.30
tx(s, 0.72, ty2, 11.9, 0.28, "14 次采样原始序列（峰值）", 10, True, GREY)
vals_v = [1492, 1499, 1522, 1542, 1552, 1559, 1580, 1594, 1673, 1688, 1695, 1719, 1742, 1816]
vals_o = [961, 993, 994, 1011, 1017, 1021, 1024, 1031, 1032, 1044, 1064, 1090, 1116, 1193]
bw = 11.6 / 14
for i, v in enumerate(vals_v):
    h = (v / 1900.0) * 0.60
    rect(s, 0.72 + i * bw, ty2 + 0.95 - h, bw * 0.8, h, GREEN)
for i, v in enumerate(vals_o):
    h = (v / 1900.0) * 0.60
    rect(s, 0.72 + i * bw, ty2 + 1.55 - h, bw * 0.8, h, ORANGE)
tx(s, 0.72, ty2 + 0.98, 3, 0.22, "■ 垂直状态", 8.5, True, GREEN)
tx(s, 3.6, ty2 + 0.98, 3, 0.22, "■ 倾斜状态", 8.5, True, ORANGE)

# ============================================================ 10 入射角判据
s, y = new_slide("入射角判据的物理依据", "09 · PHYSICS",
                 "斜射使光斑投影拉长、部分能量溢出阵列 —— 接收光强随角度呈钟形分布")

# 左侧：三态示意
scenes = [("θ = 0°", "投影为圆\n单个探测器最亮", GREEN),
          ("θ = 20°", "投影 1.06×\n亮度降至 94%", ORANGE),
          ("θ = 40°", "投影 1.31×\n亮度降至 77%", RED)]
for i, (th, desc, col) in enumerate(scenes):
    cx = 0.72 + i * 2.05
    rect(s, cx, y + 0.10, 1.85, 2.25, WHITE, LINE)
    # 平面
    rect(s, cx + 0.12, y + 1.62, 1.6, 0.10, RGBColor(0xC8, 0xD4, 0xE0))
    # 光斑（椭圆用圆角矩形近似）
    ew = 0.55 + i * 0.30
    rect(s, cx + 0.925 - ew / 2, y + 1.30, ew, 0.30, col,
         shape=MSO_SHAPE.OVAL)
    # 光柱
    lx = cx + 0.925
    rect(s, lx - 0.02, y + 0.18, 0.04, 1.12, RGBColor(0x6F, 0xB8, 0xC8))
    tx(s, cx + 0.10, y + 0.14, 1.6, 0.26, th, 11, True, col, PP_ALIGN.CENTER)
    tx(s, cx + 0.10, y + 1.88, 1.6, 0.45, desc, 8.5, False, GREY, PP_ALIGN.CENTER, line=1.35)

# 右侧：钟形曲线（用形状近似）
cx0, cy0 = 7.4, y + 0.20
cw, chh = 5.0, 1.85
rect(s, cx0, cy0, cw, chh, WHITE, LINE)
# 曲线用一组细柱拼出钟形
import math
for k in range(60):
    t = -1.0 + 2.0 * k / 59.0            # -1 ~ 1
    g = math.exp(-((t - 0.12) ** 2) / 0.16)
    bh = g * (chh - 0.34)
    rect(s, cx0 + 0.2 + k * (cw - 0.4) / 60, cy0 + chh - 0.14 - bh,
         (cw - 0.4) / 60 * 0.9, bh, RGBColor(0x7F, 0xB8, 0xD8))
# 92% 判据线
rect(s, cx0 + 0.2, cy0 + chh - 0.14 - 0.92 * (chh - 0.34), cw - 0.4, 0.018, ORANGE)
tx(s, cx0 + 0.28, cy0 + chh - 0.14 - 0.92 * (chh - 0.34) - 0.24, 3.4, 0.24,
   "92% 判据线（工程容差）", 8, True, ORANGE)
tx(s, cx0 + cw - 0.75, cy0 + chh + 0.02, 0.7, 0.24, "θ", 10, True, GREY)
tx(s, cx0 + 0.02, cy0 + 0.06, 0.5, 0.24, "MAX", 8, True, GREY)

tx(s, 0.72, y + 2.55, 5.9, 0.3, "为什么单次测量不够", 12, True, NAVY)
tx(s, 0.72, y + 2.88, 5.9, 0.8,
   "MAX(θ) 是偶函数：MAX(+20°) ≡ MAX(−20°)\n"
   "单点只能得到「斜了多少」，无法判断「往哪边斜」。",
   10.5, False, TEXT, line=1.55)

tx(s, 6.85, y + 2.55, 5.8, 0.3, "解决思路", 12, True, ORANGE)
tx(s, 6.85, y + 2.88, 5.8, 0.8,
   "在三个已知角度各取一次，用抛物线插值\n解析求出峰顶位置 → 一次转动到目标角。",
   10.5, False, TEXT, line=1.55)

# ============================================================ 11 三点插值
s, y = new_slide("对准算法：三点抛物线插值", "10 · ALGORITHM",
                 "不做全行程遍历，3 次测量解析求出垂直角，约 1 秒完成一次修正")

# 公式区
rect(s, 0.72, y + 0.05, 11.9, 1.02, BLUE_L)
tx(s, 1.02, y + 0.18, 11.3, 0.3, "峰值角度解析解", 11, True, BLUE)
tx(s, 1.02, y + 0.50, 11.3, 0.4,
   "θ* = θ₀ + d · (M₊ − M₋) / ( 2 · (M₊ + M₋ − 2·M₀) )",
   18, True, NAVY)

# 参数解释
params = [("θ₀", "当前舵机角度"), ("d", "探测步长 8°"),
          ("M₋", "θ₀−d 处峰值"), ("M₀", "θ₀ 处峰值"), ("M₊", "θ₀+d 处峰值")]
py = y + 1.30
for i, (sym, desc) in enumerate(params):
    cx = 0.72 + i * 2.4
    rect(s, cx, py, 2.2, 0.62, WHITE, LINE)
    tx(s, cx + 0.12, py + 0.06, 0.6, 0.28, sym, 13, True, ORANGE)
    tx(s, cx + 0.62, py + 0.14, 1.5, 0.3, desc, 9.5, False, GREY)

# 流程
fy = y + 2.15
tx(s, 0.72, fy, 5, 0.3, "单次修正流程", 12, True, NAVY)
steps = [
    ("S1", "测 M₀", "当前位置"),
    ("S2", "右转 d，测 M₊", "+8°"),
    ("S3", "左转 2d，测 M₋", "−8°"),
    ("S4", "代入公式求 θ*", "解析求解"),
    ("S5", "转到 θ*", "一次到位"),
]
sx = 0.72
for code, name, note in steps:
    rect(s, sx, fy + 0.36, 2.22, 0.86, WHITE, LINE)
    rect(s, sx, fy + 0.36, 0.05, 0.86, BLUE)
    tx(s, sx + 0.14, fy + 0.42, 0.5, 0.26, code, 9, True, BLUE)
    tx(s, sx + 0.14, fy + 0.66, 2.0, 0.3, name, 10, True, TEXT)
    tx(s, sx + 0.14, fy + 0.94, 2.0, 0.22, note, 8.5, False, GREY)
    sx += 2.36

# 实例与鲁棒性
iy = fy + 1.42
rect(s, 0.72, iy, 5.85, 0.95, RGBColor(0xEE, 0xF6, 0xF1))
tx(s, 0.95, iy + 0.10, 5.4, 0.26, "算例", 10, True, GREEN)
tx(s, 0.95, iy + 0.38, 5.4, 0.5,
   "M₋=1420  M₀=1250  M₊=1080\nd=8° → θ* = 11.7°（向右偏 11.7°）",
   9.5, False, TEXT, line=1.45)

rect(s, 6.77, iy, 5.85, 0.95, RGBColor(0xFD, 0xF3, 0xE7))
tx(s, 7.00, iy + 0.10, 5.4, 0.26, "退化保护", 10, True, ORANGE)
tx(s, 7.00, iy + 0.38, 5.4, 0.5,
   "|分母| < 阈值 → 三点近等高，说明已在峰顶\n或信号无效；此时输出 0，不动作。",
   9.5, False, TEXT, line=1.45)

# ============================================================ 12 舵机控制
s, y = new_slide("伺服控制方案", "11 · CONTROL",
                 "连续旋转舵机无位置反馈，用「虚拟角度 + 定时转动」实现角度寻址")

# 左：原理
tx(s, 0.72, y + 0.02, 5.8, 0.3, "虚拟角度模型", 12, True, NAVY)
rows = [
    ["命令", "动作", "脉宽"],
    ["A 50", "从 0° 转到 50°", "1500+δ，持续 50×8ms"],
    ["A 100", "从 50° 再转 50°", "同上"],
    ["A 0", "从 100° 转回 0°", "反向，100×8ms"],
    ["断电重启", "虚拟角度归 0", "—"],
]
table(s, 0.72, y + 0.36, 5.8, rows, [1.2, 2.6, 2.0], size=9.5, row_h=0.32, head_h=0.32)

tx(s, 0.72, y + 2.10, 5.8, 0.3, "核心接口", 12, True, NAVY)
tx(s, 0.72, y + 2.44, 5.8, 0.9,
   "servo_spin(permille, ms)\n"
   "  以千分速度转动指定毫秒后自动停止\n"
   "  避免与 1 ms 节拍的 CCR 刷新冲突",
   10, False, TEXT, line=1.5)

# 右：闭环状态机
rx = 6.85
tx(s, rx, y + 0.02, 5.8, 0.3, "跟踪状态机", 12, True, NAVY)
states = [
    ("等待", "MAX < 300\n光未到达，静止", GREY, 0.10),
    ("判定", "MAX ≥ best×92%\n已垂直，保持静止", GREEN, 1.05),
    ("修正", "差值 × 比例系数\n步长 2°~24°", ORANGE, 2.00),
]
for name, body, col, dy in states:
    rect(s, rx, y + 0.40 + dy, 1.35, 0.72, col, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
    tx(s, rx, y + 0.52 + dy, 1.35, 0.28, name, 11, True, WHITE, PP_ALIGN.CENTER)
    tx(s, rx + 1.50, y + 0.46 + dy, 4.2, 0.62, body, 9.5, False, TEXT, line=1.4)

# 三个约束
ky = y + 3.30
cons = [("尺度无关", "用 best×92% 相对判据，\n适配任意光强"),
        ("自动适应", "best 缓慢衰减，\n光变弱时不锁死"),
        ("死区抑制", "容差带内不动作，\n避免持续抖动")]
for i, (h, b) in enumerate(cons):
    x = 0.72 + i * 4.0
    rect(s, x, ky, 3.8, 0.92, WHITE, LINE)
    rect(s, x, ky, 0.05, 0.92, GREEN)
    tx(s, x + 0.18, ky + 0.10, 3.4, 0.28, h, 11, True, GREEN)
    tx(s, x + 0.18, ky + 0.42, 3.4, 0.45, b, 9, False, TEXT, line=1.35)

# ============================================================ 13 上位机
s, y = new_slide("上位机工具：可视化与调试", "12 · TOOLING",
                 "按实物 L 形布局还原，独立 EXE 分发，现场无需安装环境")

# 左：功能列表
tx(s, 0.72, y + 0.02, 5.6, 0.3, "功能构成", 12, True, NAVY)
feats = [
    ("布局还原", "长边 43 竖条 + 短边 18 横条，与实物一致"),
    ("实时曲线", "双通道折线，直观看出峰值与展宽"),
    ("数据规整", "EMA 平滑 α=0.5 + 阈值过滤"),
    ("四项指标", "峰值 / 重心 / 展宽 / 集中度"),
    ("联动控制", "角度输入、全行程对准、串口参数"),
]
fy = y + 0.40
for h, b in feats:
    rect(s, 0.72, fy, 0.045, 0.52, BLUE)
    tx(s, 0.94, fy + 0.02, 5.2, 0.26, h, 11, True, BLUE)
    tx(s, 0.94, fy + 0.28, 5.2, 0.26, b, 9, False, GREY)
    fy += 0.58

# 右：技术栈与打包
rx = 6.85
tx(s, rx, y + 0.02, 5.8, 0.3, "技术实现", 12, True, ORANGE)
rows = [
    ["项目", "选型", "说明"],
    ["界面框架", "Tkinter Canvas", "无外部 GUI 依赖"],
    ["串口", "pyserial 3.5", "115200 8N1"],
    ["绘图", "Canvas 图元", "柱 + 折线"],
    ["打包", "PyInstaller 6.22", "onedir 模式"],
    ["产物", "DetectorUI.exe", "26.9 MB / 独立运行"],
]
table(s, rx, y + 0.36, 5.8, rows, [1.5, 2.0, 2.3], size=9.5, row_h=0.30, head_h=0.32)

# 通信协议
ty = y + 2.28
rect(s, rx, ty, 5.8, 1.35, BLUE_L)
tx(s, rx + 0.20, ty + 0.10, 5.4, 0.26, "通信协议", 10.5, True, BLUE)
tx(s, rx + 0.20, ty + 0.40, 5.4, 0.85,
   "K BOTH  → 一次扫描返回两边\n"
   "　LONG43 v1..v43 / SHORT18 v1..v18\n"
   "K SWEEP → 全行程对准　A <deg> → 手动转动",
   9, False, TEXT, line=1.5)

# 左下方：延迟优化
ly = y + 3.25
rect(s, 0.72, ly, 5.6, 0.95, RGBColor(0xEE, 0xF6, 0xF1))
tx(s, 0.95, ly + 0.10, 5.2, 0.26, "延迟优化", 10, True, GREEN)
tx(s, 0.95, ly + 0.38, 5.2, 0.5,
   "两次独立查询 → 合并为单次 K BOTH\n端到端延迟由约 600 ms 降至约 150 ms",
   9, False, TEXT, line=1.45)

# ============================================================ 14 方法论
s, y = new_slide("调试方法论沉淀", "13 · METHODOLOGY",
                 "本次问题排查形成的可复用工程方法")

methods = [
    ("寄存器级取证", "配置类问题先读寄存器，\n不靠推测。CR1 / AFR / MODER\n一次读取即可证伪或证实。", BLUE),
    ("对照参考实现", "同一硬件的历史可用工程\n是最强参照；逐宏比对\n比盲改参数高效一个数量级。", ORANGE),
    ("每次只改一个变量", "同批次改动多项会导致\n归因失败；本次曾因\n多变量同改多走弯路。", GREEN),
    ("建立可观测性", "K ECHO / K SPREAD / 遥测打印\n让内部状态可见，\n否则调试是盲猜。", NAVY),
]
for i, (h, b, col) in enumerate(methods):
    x = 0.72 + (i % 2) * 6.05
    yy = y + 0.10 + (i // 2) * 2.30
    rect(s, x, yy, 5.85, 2.05, WHITE, LINE)
    rect(s, x, yy, 5.85, 0.06, col)
    tx(s, x + 0.24, yy + 0.22, 5.3, 0.32, h, 14, True, col)
    tx(s, x + 0.24, yy + 0.68, 5.3, 1.2, b, 10.5, False, TEXT, line=1.65)

# 底部教训
by = y + 4.55
rect(s, 0.72, by, 11.9, 0.62, RGBColor(0xFD, 0xEC, 0xEC))
rect(s, 0.72, by, 0.06, 0.62, RED)
tx(s, 1.02, by + 0.16, 11.3, 0.3,
   "教训：协议层错误会伪装成硬件故障 —— 一旦发现「调整任何时序参数都无改善」，应立即转向协议格式核查。",
   10.5, False, RGBColor(0x8A, 0x28, 0x30))

# ============================================================ 15 后续计划
s, y = new_slide("后续工作计划", "14 · ROADMAP",
                 "按优先级推进：先固化算法，再做产品化收敛")

phases = [
    ("P0 · 立即", "三点插值算法落地",
     ["实现 find_peak() 解析求解", "加入退化保护与限幅", "实测校准 d 与容差带"], RED),
    ("P1 · 本周", "闭环稳定性验证",
     ["连续 30 分钟跟踪测试", "统计角度残差与恢复时间", "调优步长系数与死区"], ORANGE),
    ("P2 · 本月", "产品化收敛",
     ["清理调试命令与冗余代码", "上电自启动流程固化", "异常自恢复（丢光/限位）"], BLUE),
    ("P3 · 后续", "双自由度扩展",
     ["启用短边闭环控制 Y 向", "双轴联动解耦", "引入滤波与预测控制"], GREEN),
]
for i, (ph, title, items, col) in enumerate(phases):
    x = 0.72 + i * 3.05
    rect(s, x, y + 0.10, 2.85, 0.55, col)
    tx(s, x + 0.14, y + 0.20, 2.6, 0.3, ph, 11, True, WHITE)
    tx(s, x + 0.14, y + 0.76, 2.6, 0.5, title, 12, True, col, line=1.3)
    iy = y + 1.32
    for it in items:
        rect(s, x + 0.02, iy + 0.07, 0.08, 0.08, col, shape=MSO_SHAPE.OVAL)
        tx(s, x + 0.20, iy, 2.6, 0.5, it, 9.5, False, TEXT, line=1.35)
        iy += 0.56

# 风险
ry = y + 3.60
tx(s, 0.72, ry, 6, 0.3, "风险与对策", 12, True, NAVY)
rows = [
    ["风险", "影响", "对策"],
    ["峰值判据在小角度范围趋于平坦", "定位精度下降", "缩小 d，改用二次拟合"],
    ["连续旋转舵机无位置反馈", "累计角度漂移", "定期回归基准 + 视觉复核"],
    ["单侧光强受环境光干扰", "误判垂直", "加入短边健康度校验"],
]
table(s, 0.72, ry + 0.34, 11.9, rows, [4.2, 3.2, 4.5], size=9.5, row_h=0.30, head_h=0.32)

# ============================================================ 16 结束页
s = prs.slides.add_slide(prs.slide_layouts[6])
rect(s, 0, 0, SW, SH, NAVY)
rect(s, 0, 0, 0.14, SH, BLUE)
tx(s, 1.2, 2.45, 10, 0.8, "汇报完毕 · 请指正", 38, True, WHITE)
rect(s, 1.2, 3.45, 1.8, 0.05, ORANGE)
tx(s, 1.2, 3.80, 10, 1.0,
   "采集链路已打通 · 判据已完成实测标定 · 算法方案已确定\n"
   "下一阶段聚焦闭环精度与产品化收敛",
   15, False, RGBColor(0xB8, 0xCB, 0xDD), line=1.7)

out = "激光通信自动对准系统_工作汇报.pptx"
prs.save(out)
print("saved:", out, "slides:", len(prs.slides.__iter__.__self__._sldIdLst))
