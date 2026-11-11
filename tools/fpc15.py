#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fpc15.py — 下长边改用 FPC15

FPC15: CS = SPI4_CS1 = PH3
       EN = 网络 05,06,07,08 = PD3, PD2, PD1, PD0
"""

import io
import re

F = "Core/Src/adc64.c"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 1) 新增 k_en_e ----------
if "k_en_e" not in s:
    s = s.replace(
        "static const en_t k_en_d[ADC64_ROWS] = {",
        "/* 板 E 的行选（FPC15 pin6-9 -> 网络05..08 = PD3/PD2/PD1/PD0） */\n"
        "static const en_t k_en_e[ADC64_ROWS] = {\n"
        "    { GPIOD,  3U }, { GPIOD,  2U }, { GPIOD,  1U }, { GPIOD,  0U }\n"
        "};\n\n"
        "static const en_t k_en_d[ADC64_ROWS] = {", 1)

# ---------- 2) en_off_all 加入 k_en_e ----------
s = s.replace(
    "        en_low(&k_en_d[r]);\n    }\n}",
    "        en_low(&k_en_d[r]);\n        en_low(&k_en_e[r]);\n    }\n}", 1)

# ---------- 3) en_on 加入 group 4 ----------
if "g == 4U" not in s:
    s = s.replace(
        """        if ((g == 0xFFU) || (g == 3U)) {
            if (hi != 0U) { en_high(&k_en_d[r]); } else { en_low(&k_en_d[r]); }
        } else {
            en_low(&k_en_d[r]);
        }""",
        """        if ((g == 0xFFU) || (g == 3U)) {
            if (hi != 0U) { en_high(&k_en_d[r]); } else { en_low(&k_en_d[r]); }
        } else {
            en_low(&k_en_d[r]);
        }

        if ((g == 4U)) {
            if (hi != 0U) { en_high(&k_en_e[r]); } else { en_low(&k_en_e[r]); }
        } else {
            en_low(&k_en_e[r]);
        }""", 1)

# ---------- 4) set_en_group 支持 0..4 ----------
s = s.replace(
    "void adc64_set_en_group(uint8_t g) { s_en_group = (g > 3U) ? 0xFFU : g; }",
    "void adc64_set_en_group(uint8_t g) { s_en_group = (g > 4U) ? 0xFFU : g; }")

# ---------- 5) adc64_init 里配置 k_en_e 的 GPIO ----------
s = s.replace(
    """    for (i = 0U; i < ADC64_ROWS; i++) {
        en_gpio_cfg(&k_en_a[i]);
        en_gpio_cfg(&k_en_c[i]);
        en_gpio_cfg(&k_en_d[i]);
        en_gpio_cfg(&k_en_b[i]);
    }""",
    """    for (i = 0U; i < ADC64_ROWS; i++) {
        en_gpio_cfg(&k_en_a[i]);
        en_gpio_cfg(&k_en_c[i]);
        en_gpio_cfg(&k_en_d[i]);
        en_gpio_cfg(&k_en_b[i]);
        en_gpio_cfg(&k_en_e[i]);      /* FPC15: PD3/PD2/PD1/PD0 */
    }""", 1)

# ---------- 6) 片选表：pass0 改用 PH3 ----------
s = s.replace(
    "    static const GPIO_TypeDef *tport[4] = { GPIOI,  GPIOC,  GPIOI,  GPIOH  };\n"
    "    static const uint8_t       tpin[4]  = { 8U,     13U,    9U,     4U     };",
    "    static const GPIO_TypeDef *tport[4] = { GPIOH,  GPIOC,  GPIOI,  GPIOH  };\n"
    "    static const uint8_t       tpin[4]  = { 3U,     13U,    9U,     4U     };")

# ---------- 7) EN 组表：pass0 改用 group 4 (k_en_e) ----------
s = s.replace(
    "static const uint8_t grp[4] = { 0U, 2U, 3U, 1U };  /* A, C, D, B */",
    "static const uint8_t grp[4] = { 4U, 2U, 3U, 1U };  /* E(FPC15), C, D, B */")

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
