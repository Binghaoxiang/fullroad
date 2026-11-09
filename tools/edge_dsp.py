#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
edge_dsp.py — 探测器阵列信号处理模块

行业标准链路（光电二极管阵列 / PSD / 光束轮廓仪通用）：

    原始 ADC
      ↓ ① 基线校正（暗电流扣除）
      ↓ ② 中值滤波（3 点，剔除单点尖刺/坏点）
      ↓ ③ Savitzky-Golay 平滑（保形，不压峰）
      ↓ ④ 相对阈值分割（峰值 25%）
      ↓ ⑤ 加权重心（亚探测器定位）
      ↓ ⑥ 高斯拟合（可选，更高精度）
      ↓ ⑦ 时域 EMA（跟踪平滑）

对外提供：
    Baseline      — 自适应基线跟踪
    median3       — 3 点中值
    savgol5       — 5 点二阶平滑
    process_edge  — 完整处理一维数组，返回 dict
    GaussianFit   — 高斯拟合结果
"""

import math


# ============================================================ 基础滤波
def median3(a):
    """3 点中值滤波：有效剔除单点尖刺和坏点，且不模糊边缘"""
    n = len(a)
    if n < 3:
        return list(a)
    out = list(a)
    for i in range(1, n - 1):
        x, y, z = a[i - 1], a[i], a[i + 1]
        out[i] = x + y + z - min(x, y, z) - max(x, y, z)
    return out


def savgol5(a):
    """5 点二阶 Savitzky-Golay 平滑

    系数 [-3, 12, 17, 12, -3] / 35
    相比移动平均，它保留峰高和峰形（不压低峰值）
    """
    n = len(a)
    if n < 5:
        return list(a)
    k = [-3.0, 12.0, 17.0, 12.0, -3.0]
    out = list(a)
    for i in range(2, n - 2):
        s = 0.0
        for j in range(5):
            s += k[j] * a[i - 2 + j]
        out[i] = s / 35.0
    return out


def moving_avg(a, w=3):
    """对称移动平均（w 为窗口宽度，奇数）"""
    n = len(a)
    if n < w or w < 3:
        return list(a)
    h = w // 2
    out = list(a)
    for i in range(h, n - h):
        s = 0.0
        for j in range(w):
            s += a[i - h + j]
        out[i] = s / w
    return out




# ============================================================ 自适应噪声估计
def estimate_noise(v, low_ratio=0.40):
    """从数据本身估计噪声底噪和噪声标准差

    做法：取最低 low_ratio 比例的点作为"无光区"，
         用它们的中位数作底噪，用 MAD 作噪声尺度。

    返回 (noise_floor, sigma)
        sigma = 1.4826 * MAD  （MAD 到标准差的换算系数）
    """
    n = len(v)
    if n < 4:
        return 0.0, 1.0
    srt = sorted(float(x) for x in v)
    k = max(2, int(n * low_ratio))
    low = srt[:k]
    med = low[len(low) // 2]
    devs = sorted(abs(x - med) for x in low)
    mad = devs[len(devs) // 2]
    sigma = 1.4826 * mad
    if sigma < 1.0:
        sigma = 1.0
    return med, sigma


def adaptive_threshold(peak, noise, sigma, snr):
    """按 SNR 自适应的阈值

    强信号（SNR 高）：用较高的相对阈值，只保留主瓣，重心更准
    弱信号（SNR 低）：用较低的阈值，保住主瓣，避免重心被噪声拖走
    """
    if peak <= noise:
        return peak
    if snr >= 20.0:
        k = 0.30
    elif snr >= 10.0:
        k = 0.22
    elif snr >= 5.0:
        k = 0.15
    else:
        k = 0.10
    thr_a = peak * k                      # 相对阈值
    thr_b = noise + 3.0 * sigma           # 绝对阈值（噪声的 3σ）
    return max(thr_a, thr_b)


# ============================================================ 基线跟踪
class Baseline:
    """自适应暗电流基线

    用指数分位数跟踪"无光时的电平"：
      base ← base + α·(v - base)   仅在 v 接近或低于 base 时更新
    这样光斑出现时基线不会被拉高
    """

    def __init__(self, n, alpha=0.02):
        self.base = [0.0] * n
        self.alpha = alpha
        self.ready = False

    def update(self, raw):
        n = min(len(raw), len(self.base))
        if not self.ready:
            # 首次：用 20 分位数作为暗电平（不能直接用当前帧，否则有光时把自己当基线）
            srt = sorted(float(x) for x in raw[:n]) if n else [0.0]
            p20 = srt[int(len(srt) * 0.20)] if srt else 0.0
            for i in range(n):
                self.base[i] = p20
        a = self.alpha
        for i in range(n):
            v = float(raw[i])
            # 只向下跟踪，或缓慢向上，避免把光斑当基线
            if v < self.base[i]:
                self.base[i] += a * 4.0 * (v - self.base[i])
            else:
                self.base[i] += a * (v - self.base[i])

    def subtract(self, raw):
        n = min(len(raw), len(self.base))
        out = [0.0] * len(raw)
        for i in range(len(raw)):
            if i < n:
                d = float(raw[i]) - self.base[i]
                out[i] = d if d > 0.0 else 0.0
            else:
                out[i] = float(raw[i])
        return out


# ============================================================ 重心
def centroid(v, thr_ratio=0.25, floor=0.0):
    """加权重心：返回 (位置, 峰值, 有效总和, 有效点数)

    阈值 = max(峰值 × thr_ratio, floor)
    只统计阈值以上的点，避免噪声拖偏重心
    """
    n = len(v)
    if n == 0:
        return None, 0.0, 0.0, 0
    mx = max(v)
    if mx <= floor:
        return None, mx, 0.0, 0

    thr = max(mx * thr_ratio, floor)
    s0 = 0.0
    s1 = 0.0
    cnt = 0
    for i in range(n):
        if v[i] >= thr:
            s0 += v[i]
            s1 += v[i] * i
            cnt += 1
    if s0 <= 0.0:
        return None, mx, 0.0, 0
    return s1 / s0, mx, s0, cnt


def gaussian_fit(v, thr_ratio=0.15):
    """高斯拟合：v(i) = A·exp(-(i-μ)²/(2σ²)) + B

    做法：先取阈值以上的点，用加权最小二乘拟合 log(v-B) 的二次式
    返回 (μ, σ, A, B) 或 None

    比单纯重心更抗噪，且能给出光斑宽度 σ（用于判断离焦程度）
    """
    n = len(v)
    if n < 3:
        return None
    mx = max(v)
    if mx <= 1.0:
        return None

    B = min(v)
    thr = max(mx * thr_ratio, B + (mx - B) * thr_ratio)

    pts = []
    for i in range(n):
        y = v[i] - B
        if v[i] >= thr and y > 0.5:
            pts.append((float(i), y))

    if len(pts) < 3:
        return None

    # 拟合 ln(y) = a·i² + b·i + c
    S0 = S1 = S2 = S3 = S4 = 0.0
    T0 = T1 = T2 = 0.0
    for x, y in pts:
        ly = math.log(y)
        x2 = x * x
        S0 += 1.0
        S1 += x
        S2 += x2
        S3 += x2 * x
        S4 += x2 * x2
        T0 += ly
        T1 += x * ly
        T2 += x2 * ly

    # 解 3x3 正规方程
    M = [[S4, S3, S2, T2],
         [S3, S2, S1, T1],
         [S2, S1, S0, T0]]

    for c in range(3):
        piv = max(range(c, 3), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-12:
            return None
        M[c], M[piv] = M[piv], M[c]
        pv = M[c][c]
        for j in range(c, 4):
            M[c][j] /= pv
        for r in range(3):
            if r != c and abs(M[r][c]) > 1e-15:
                f = M[r][c]
                for j in range(c, 4):
                    M[r][j] -= f * M[c][j]

    a, b, _ = M[0][3], M[1][3], M[2][3]
    if a >= 0.0:            # 抛物线开口向上，不是高斯峰
        return None

    mu = -b / (2.0 * a)
    if not (-1.0 <= mu <= n):
        return None
    sigma = math.sqrt(-1.0 / (2.0 * a))
    if sigma <= 0.05 or sigma > n:
        return None
    A = math.exp(M[2][3] - b * b / (4.0 * a))
    return mu, sigma, A, B


# ============================================================ 时域跟踪
class Tracker:
    """α-β 滤波：对位置做时域平滑，抑制跳动，同时保留响应速度

    α 控制跟随时对新测量的权重，β 控制对速度的权重
    """

    def __init__(self, alpha=0.45, beta=0.08):
        self.a = alpha
        self.b = beta
        self.x = None
        self.v = 0.0

    def update(self, z):
        if z is None:
            return self.x
        if self.x is None:
            self.x = z
            self.v = 0.0
            return self.x
        pred = self.x + self.v
        err = z - pred
        self.x = pred + self.a * err
        self.v = self.v + self.b * err
        return self.x


# ============================================================ 完整处理
def process_edge(raw, baseline=None, thr_ratio=0.25, noise_floor=60.0,
                 use_median=True, use_savgol=True):
    """自适应处理一条边的原始数组

    与旧版的区别：
      · 噪声底噪和标准差从数据本身估计，不用固定常数
      · 阈值按 SNR 自适应
      · 平滑强度按 SNR 自适应（弱信号多平滑一遍）
      · 输出 snr，供上层做融合加权
    """
    n = len(raw)
    a = [float(x) for x in raw]
    if n == 0:
        return {"raw": raw, "corr": [], "peak": 0.0, "floor": 0.0,
                "valid": False, "pos": None, "total": 0.0, "count": 0,
                "fit": None, "mu": None, "sigma": None, "snr": 0.0}

    # ---- ① 从原始数据估噪声 ----
    nf, sigma = estimate_noise(a)

    # ---- ② 基线校正 ----
    if baseline is not None:
        baseline.update(raw)
        corr = baseline.subtract(raw)
    else:
        corr = [max(0.0, x - nf) for x in a]

    # ---- ③ 中值滤波 ----
    if use_median:
        corr = median3(corr)

    # ---- ④ 自适应平滑 ----
    peak0 = max(corr) if corr else 0.0
    snr0 = (peak0 - nf) / sigma if sigma > 0 else 0.0
    if use_savgol:
        corr = savgol5(corr)
        if snr0 < 6.0:            # 弱信号再平滑一遍
            corr = savgol5(corr)

    # ---- ⑤ 峰值与自适应阈值 ----
    mx = max(corr) if corr else 0.0
    snr = (mx - nf) / sigma if sigma > 0 else 0.0
    if snr < 0.0:
        snr = 0.0
    thr = adaptive_threshold(mx, nf, sigma, snr)

    # 有效判据：峰值必须明显高于噪声（3σ 以上）且绝对幅值够
    valid = (mx > nf + 3.0 * sigma) and (mx > noise_floor * 0.5)

    # ---- ⑥ 重心（用自适应阈值）----
    s0 = 0.0
    s1 = 0.0
    cnt = 0
    for i in range(n):
        if corr[i] >= thr:
            s0 += corr[i]
            s1 += corr[i] * i
            cnt += 1
    if s0 > 0.0:
        pos = s1 / s0
        total = s0
    else:
        pos = None
        total = 0.0

    # ---- ⑦ 高斯拟合 ----
    fit = None
    if valid and n >= 5:
        fit = gaussian_fit(corr, max(0.10, thr_ratio * 0.6))

    return {
        "raw": raw,
        "corr": corr,
        "peak": mx,
        "floor": nf,
        "sigma_noise": sigma,
        "snr": snr,
        "thr": thr,
        "valid": valid,
        "pos": pos,
        "total": total,
        "count": cnt,
        "fit": fit,
        "mu": (fit[0] if fit else pos),
        "sigma": (fit[1] if fit else None),
    }
