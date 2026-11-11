#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
screen_pos.py — 四边探测器阵列的屏幕坐标定位

物理模型
========
四条探测器边围绕一块 200 × 100 mm 的膜，膜把入射光导到四边。
光斑落在 (x, y) 时：

    下长边 E0（横向，D25→D4F，左→右）    →  给出 X
    上长边 E2（横向，D4F→D35，左→右）    →  给出 X
    右短边 E1（纵向，D35→D49，下→上）    →  给出 Y
    左短边 E3（纵向，D35→D3A，上→下）    →  给出 Y

每条轴有两个独立测量，按信号强度加权融合：

    X = (X₀·w₀ + X₂·w₂) / (w₀ + w₂)
    Y = (Y₁·w₁ + Y₃·w₃) / (w₁ + w₃)

    w = 该边的总光强（越亮越可信）

方向约定
========
    屏幕坐标系：(0, 0) 在左下角，X 向右，Y 向上
    E0 索引 0 → X = 0        索引 N-1 → X = 200
    E2 索引 0 → X = 0        索引 N-1 → X = 200
    E1 索引 0 → Y = 0        索引 N-1 → Y = 100
    E3 索引 0 → Y = 100      索引 N-1 → Y = 0      ← 上→下，需翻转

使用
====
    from screen_pos import ScreenLocator
    loc = ScreenLocator()
    loc.configure(width_mm=200.0, height_mm=100.0)
    result = loc.localize(edge_results)
    print(result.x_mm, result.y_mm, result.confidence)
"""

import math


# ============================================================ 边定义
# (边号, 中文名, 颗数, 测的轴, 索引0 对应的值是否在最大端)
# (idx, 名称, 探测器数, 轴, 是否翻转, 校准偏移mm, 校准缩放)
# 校准偏移为 0 时表示不做修正（原始映射）
EDGES = [
    (0, "下长边",  43,   "x",   False,   0.0,   1.0),   # D25→D4F 左→右
    (1, "右短边",  18,   "y",   False,   0.0,   1.0),   # D35→D49 下→上
    (2, "上长边",  42,   "x",   False,   0.0,   1.0),   # D4F→D35 左→右
    (3, "左短边",  21,   "y",   True,    0.0,   1.0),   # D35→D3A 上→下（翻转）
]


class PosResult:
    """定位结果"""

    def __init__(self):
        self.valid = False
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.x0 = None        # E0 单独给出的 X
        self.x2 = None        # E2 单独给出的 X
        self.y1 = None        # E1 单独给出的 Y
        self.y3 = None        # E3 单独给出的 Y
        self.wx0 = 0.0
        self.wx2 = 0.0
        self.wy1 = 0.0
        self.wy3 = 0.0
        self.consistency = 0.0    # 0~1，两条边吻合程度
        self.quality = 0.0        # 0~1，综合可信度

    def __repr__(self):
        if not self.valid:
            return "<PosResult invalid>"
        return ("<PosResult x=%.1f y=%.1f mm  consistency=%.0f%%  quality=%.0f%%>"
                % (self.x_mm, self.y_mm, self.consistency * 100,
                   self.quality * 100))


# 运行时可修改的校准表：{边号: (offset_mm, scale)}
CAL = {}


class ScreenLocator:
    """四边融合定位器"""

    def __init__(self, width_mm=200.0, height_mm=100.0,
                 min_total=200.0, consistency_scale=200.0):
        """
        width_mm / height_mm   屏幕物理尺寸
        min_total              单边有效的最小总光强
        consistency_scale      一致性归一化尺度（mm）
        """
        self.W = float(width_mm)
        self.H = float(height_mm)
        self.min_total = float(min_total)
        self.cscale = float(consistency_scale)
        self.min_peak = 120.0      # 绝对峰值门限（已放宽，弱信号靠 SNR 判据）
        self.rel_peak = 0.001      # 相对门限已基本禁用，改用 SNR 判据

    # ---------------- 单边 → 物理坐标 ----------------
    def _edge_to_mm(self, e, pos_index, count, total):
        """把某条边的重心（索引坐标）换算成沿轴物理位置（mm）

        pos_index  重心位置，单位是探测器索引（0 .. count-1）
        count      该边的探测器数量
        total      该边的总光强（用作融合权重）

        返回 (mm, weight)；无效时返回 (None, 0)
        """
        if pos_index is None or count < 2:
            return None, 0.0
        if total < self.min_total:
            return None, 0.0

        # 归一化到 0..1
        u = pos_index / float(count - 1)
        if u < -0.5 or u > 1.5:
            return None, 0.0
        u = max(0.0, min(1.0, u))

        _, _, _, axis, invert, d_off, d_scale = EDGES[e]
        cal_off, cal_scale = CAL.get(e, (d_off, d_scale))
        if invert:
            u = 1.0 - u

        span = self.W if axis == "x" else self.H
        mm = u * span * cal_scale + cal_off
        return mm, float(total)

    # ---------------- 主入口 ----------------
    def localize(self, edges):
        """edges: 长度为 4 的列表，每项是 dict 或 None

        每项需包含（来自 edge_dsp.process_edge 的返回）：
            pos    重心位置（探测器索引，可为 float）
            total  总光强
            peak   峰值
            valid  是否有效

        返回 PosResult
        """
        res = PosResult()

        mm = [None] * 4
        wt = [0.0] * 4

        for e in range(4):
            d = edges[e] if e < len(edges) else None
            if not d or not d.get("valid"):
                continue
            count = EDGES[e][2]
            # 优先用高斯拟合的中心，退化时用重心
            pos = d.get("mu")
            if pos is None:
                pos = d.get("pos")
            mm[e], wt[e] = self._edge_to_mm(e, pos, count, d.get("total", 0.0))
            # 权重改进：用「峰值 × 集中度²」而非总光强
            # 集中度 = 峰值/总和，单峰干净波形接近 1，杂乱波形接近 0
            pk = float(d.get("peak", 0.0))
            tt = float(d.get("total", 0.0))
            if pk > 0.0 and mm[e] is not None and tt > 0.0:
                conc = max(0.0, min(1.0, pk / tt))
                wt[e] = pk * conc * conc

        # ---- 相对门控：峰值低于最佳边一定比例的直接剔除 ----
        pks = []
        for e in range(4):
            d = edges[e] if e < len(edges) else None
            if d and d.get("valid"):
                pks.append(float(d.get("peak", 0.0)))
            else:
                pks.append(0.0)
        best_pk = max(pks) if pks else 0.0
        if best_pk > 0.0:
            for e in range(4):
                if mm[e] is not None and pks[e] < best_pk * self.rel_peak:
                    mm[e] = None
                    wt[e] = 0.0

        # ---- 权重重归一化：峰值低于门限的边权重再降一档 ----
        # ---- X 轴：E0 + E2 ----
        num = 0.0
        den = 0.0
        if mm[0] is not None:
            num += mm[0] * wt[0]
            den += wt[0]
            res.x0 = mm[0]
            res.wx0 = wt[0]
        if mm[2] is not None:
            num += mm[2] * wt[2]
            den += wt[2]
            res.x2 = mm[2]
            res.wx2 = wt[2]
        if den > 0.0:
            res.x_mm = num / den

        # ---- Y 轴：E1 + E3 ----
        num = 0.0
        den = 0.0
        if mm[1] is not None:
            num += mm[1] * wt[1]
            den += wt[1]
            res.y1 = mm[1]
            res.wy1 = wt[1]
        if mm[3] is not None:
            num += mm[3] * wt[3]
            den += wt[3]
            res.y3 = mm[3]
            res.wy3 = wt[3]
        if den > 0.0:
            res.y_mm = num / den

        has_x = (res.x0 is not None) or (res.x2 is not None)
        has_y = (res.y1 is not None) or (res.y3 is not None)
        res.valid = has_x and has_y

        # ---- 一致性：两条边的差值归一化 ----
        diffs = []
        if res.x0 is not None and res.x2 is not None:
            diffs.append(abs(res.x0 - res.x2))
        if res.y1 is not None and res.y3 is not None:
            diffs.append(abs(res.y1 - res.y3))
        if diffs:
            dmax = max(diffs)
            res.consistency = max(0.0, 1.0 - dmax / self.cscale)

        # ---- 综合质量：一致性与信号强度 ----
        n_edges = sum(1 for e in range(4) if mm[e] is not None)
        strength = min(1.0, (sum(wt) / 4.0) / 3000.0)
        res.quality = res.consistency * (0.5 + 0.5 * n_edges / 4.0) * (0.4 + 0.6 * strength)

        return res

    # ---------------- 便利方法 ----------------
    def localize_from_raw(self, raw_edges, dsp):
        """直接从四条边的原始数组定位

        raw_edges  长度 4 的列表，每项是原始 ADC 数组
        dsp        edge_dsp 模块

        返回 (PosResult, [processed_edge_result, ...])
        """
        proc = []
        for e in range(4):
            raw = raw_edges[e] if e < len(raw_edges) else None
            if raw:
                proc.append(dsp.process_edge(raw, None))
            else:
                proc.append(None)
        return self.localize(proc), proc


# ============================================================ 自检
if __name__ == "__main__":
    # 构造：光斑在 (60mm, 30mm)
    # E0 43 颗 → 索引 = 60/200*42 = 12.6
    # E2 42 颗 → 索引 = 60/200*41 = 12.3
    # E1 18 颗 → 索引 = 30/100*17 = 5.1
    # E3 21 颗 → 上→下，索引 = (1-30/100)*20 = 14.0
    def mk(pos, total=4000, mu=None):
        return {"valid": True, "pos": pos, "mu": mu if mu is not None else pos,
                "total": total, "peak": 2000}

    loc = ScreenLocator()
    edges = [mk(12.6), mk(5.1), mk(12.3), mk(14.0)]
    r = loc.localize(edges)
    print("真值: x=60.0  y=30.0")
    print("结果:", r)
    print("  E0 -> X=%.1f (w=%.0f)" % (r.x0, r.wx0))
    print("  E2 -> X=%.1f (w=%.0f)" % (r.x2, r.wx2))
    print("  E1 -> Y=%.1f (w=%.0f)" % (r.y1, r.wy1))
    print("  E3 -> Y=%.1f (w=%.0f)" % (r.y3, r.wy3))

    print()
    print("--- 边界测试 ---")
    for tx, ty in [(0, 0), (200, 0), (0, 100), (200, 100), (100, 50)]:
        i0 = tx / 200.0 * 42
        i2 = tx / 200.0 * 41
        i1 = ty / 100.0 * 17
        i3 = (1 - ty / 100.0) * 20
        rr = loc.localize([mk(i0), mk(i1), mk(i2), mk(i3)])
        print("  真值(%6.1f,%6.1f) -> (%6.1f,%6.1f)  质量 %.0f%%"
              % (tx, ty, rr.x_mm, rr.y_mm, rr.quality * 100))
