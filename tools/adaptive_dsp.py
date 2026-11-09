#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""adaptive_dsp.py — 给 edge_dsp.py 加入自适应处理

自适应策略：
  1. 噪声底噪从数据本身估计（最低 40% 的中位数）
  2. 噪声标准差 = 这些低值点的 MAD（中位绝对偏差）
  3. SNR = (峰值 - 噪声) / (1.4826 * MAD)
  4. 阈值 = 噪声 + k*σ，k 随 SNR 变化（弱信号用更小的 k 以保住主瓣）
  5. 平滑强度随 SNR 变化：弱信号做 2 遍 SG，强信号 1 遍
  6. 结果里输出 snr，供融合权重使用
"""

import io
import re

F = "tools/edge_dsp.py"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 1) 新增自适应辅助函数 ----------
if "def estimate_noise" not in s:
    helpers = '''

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
'''
    s = s.replace("# ============================================================ 基线跟踪",
                  helpers + "\n\n# ============================================================ 基线跟踪")

# ---------- 2) 用自适应版本替换 process_edge ----------
start = s.find("def process_edge(")
if start > 0:
    end = len(s)
    new_fn = '''def process_edge(raw, baseline=None, thr_ratio=0.25, noise_floor=60.0,
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
'''
    s = s[:start] + new_fn

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
