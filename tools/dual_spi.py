#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dual_spi.py — 增加 SPI5 支持，上/左边迁移到 FPC11/FPC10

SPI4: PI8(FPC12下) + PH4(FPC16右)   MISO -> H11
SPI5: PE12(FPC11上) + PE13(FPC10左)  MISO -> H9  (PF7/PF8/PF9, AF5)
"""

import io
import re

# ============================================================ board.c
FB = "Core/Src/board.c"
sb = io.open(FB, "r", encoding="utf-8").read()

# 1) 全局活动 SPI 指针 + 选择函数
if "s_active_spi" not in sb:
    sb = sb.replace("void board_spi_init(void)",
        "static volatile SPI_TypeDef *s_active_spi = SPI4;\n"
        "\n"
        "/* 切换当前使用的 SPI 外设：0=SPI4(下/右) 1=SPI5(上/左) */\n"
        "void board_spi_select(uint8_t which)\n"
        "{\n"
        "    s_active_spi = (which != 0U) ? SPI5 : SPI4;\n"
        "}\n"
        "\n"
        "void board_spi_init(void)", 1)

# 2) transfer16 用活动指针
sb = sb.replace("while (((ADC64_SPI->SR & SPI_SR_TXE) == 0U) && (guard != 0U)) { guard--; }",
                "while (((s_active_spi->SR & SPI_SR_TXE) == 0U) && (guard != 0U)) { guard--; }")
sb = sb.replace("*(volatile uint16_t *)&ADC64_SPI->DR = tx;",
                "*(volatile uint16_t *)&s_active_spi->DR = tx;")
sb = sb.replace("while (((ADC64_SPI->SR & SPI_SR_RXNE) == 0U) && (guard != 0U)) { guard--; }",
                "while (((s_active_spi->SR & SPI_SR_RXNE) == 0U) && (guard != 0U)) { guard--; }")
sb = sb.replace("rx = *(volatile uint16_t *)&ADC64_SPI->DR;",
                "rx = *(volatile uint16_t *)&s_active_spi->DR;")
sb = sb.replace("if (guard == 0U) { (void)*(volatile uint16_t *)&ADC64_SPI->DR; return 0xFFFFU; }",
                "if (guard == 0U) { (void)*(volatile uint16_t *)&s_active_spi->DR; return 0xFFFFU; }")

# 3) SPI5 初始化
if "board_spi5_init" not in sb:
    sb = sb.rstrip() + r'''

/*----------------------------------------------------------------------------
 * SPI5 初始化：PF7=SCK PF8=MISO PF9=MOSI (AF5)
 * 供上长边(FPC11)与左短边(FPC10)使用，MISO 走 H9
 *--------------------------------------------------------------------------*/
void board_spi5_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    board_gpio_af_config(GPIOF, 7U, 5U, 0U);    /* SCK  AF5 */
    board_gpio_af_config(GPIOF, 8U, 5U, 0U);    /* MISO AF5 */
    board_gpio_af_config(GPIOF, 9U, 5U, 0U);    /* MOSI AF5 */

    SPI5->CR1 = 0U;
    SPI5->CR2 = 0U;
    SPI5->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_DFF
              | ((uint32_t)ADC64_SPI_BR << 3) | SPI_CR1_SPE;
}
'''

io.open(FB, "w", encoding="utf-8").write(sb)

# ============================================================ board.h
FH = "Core/Inc/board.h"
sh = io.open(FH, "r", encoding="utf-8").read()
if "board_spi_select" not in sh:
    sh = re.sub(r"(void\s+board_spi_init\(void\s*\);)",
                r"\1\nvoid     board_spi5_init(void);\nvoid     board_spi_select(uint8_t which);", sh, count=1)
    io.open(FH, "w", encoding="utf-8").write(sh)

# ============================================================ adc64.c
FA = "Core/Src/adc64.c"
sa = io.open(FA, "r", encoding="utf-8").read()

# 4) 初始化：配置 PE12 / PE13 为输出，并调 board_spi5_init
if "PE12" not in sa:
    sa = sa.replace("    en_off_all();\n    adc64_default_masks();",
        "    /* 新增：SPI5 侧的两条片选 PE12(FPC11 上长边) / PE13(FPC10 左短边) */\n"
        "    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;\n"
        "    (void)RCC->AHB1ENR;\n"
        "    GPIOE->MODER = (GPIOE->MODER & ~((3UL << 24) | (3UL << 26)))\n"
        "                 | ((1UL << 24) | (1UL << 26));      /* PE12/PE13 输出 */\n"
        "    GPIOE->OTYPER &= ~((1UL << 12) | (1UL << 13));\n"
        "    GPIOE->OSPEEDR = (GPIOE->OSPEEDR & ~((3UL << 24) | (3UL << 26)))\n"
        "                  | ((2UL << 24) | (2UL << 26));     /* 高速 */\n"
        "    GPIOE->PUPDR = (GPIOE->PUPDR & ~((3UL << 24) | (3UL << 26)));\n"
        "    GPIOE->BSRR = (1UL << 12) | (1UL << 13);         /* 初始高电平 */\n"
        "\n"
        "    board_spi5_init();\n"
        "\n"
        "    en_off_all();\n"
        "    adc64_default_masks();", 1)

# 5) cs_all_high 覆盖六条片选
sa = sa.replace(
    "    GPIOI->BSRR = (1UL << 8U);      /* FPC12 下边  PI8  */",
    "    GPIOI->BSRR = (1UL << 8U);      /* FPC12 下边  PI8  */\n"
    "    GPIOE->BSRR = (1UL << 12U);     /* FPC11 上边  PE12 */\n"
    "    GPIOE->BSRR = (1UL << 13U);     /* FPC10 左边  PE13 */")

# 6) 扫描主流程：按板切换 SPI 外设
old = sa[sa.index("void adc64_read_edges_all"):]
old_end = old.index("\n}\n") + 3
old_fn = old[:old_end]

new_fn = '''void adc64_read_edges_all(uint16_t *b0, uint16_t *b1, uint16_t *b2, uint16_t *b3,
                          uint8_t *n_out)
{
    static adc64_frame_t fr;
    /* 每遍：片选端口 / 片选引脚 / EN 组 / SPI 外设 */
    static GPIO_TypeDef * const tport[4] = { GPIOI,  GPIOH,  GPIOE,  GPIOE  };
    static const uint8_t        tpin[4]  = { 8U,     4U,     12U,    13U    };
    static const uint8_t        grp[4]   = { 0U,     1U,     1U,     4U     };
    static const uint8_t        spisel[4] = { 0U,    0U,     1U,     1U     };
    cs_t    save0;
    cs_t    save1;
    uint8_t pass;

    /*   pass 0: SPI4 PI8  (FPC12) -> E0 下长边 43 颗  EN=网络17-20
     *   pass 1: SPI4 PH4  (FPC16) -> E1 右短边 18 颗  EN=网络01-04
     *   pass 2: SPI5 PE12 (FPC11) -> E2 上长边 42 颗  EN=网络01-04
     *   pass 3: SPI5 PE13 (FPC10) -> E3 左短边 21 颗  EN=网络05-08
     */
    for (pass = 0U; pass < 4U; pass++) {
        adc64_set_en_group(grp[pass]);
        adc64_set_en_hold(0U);
        board_spi_select(spisel[pass]);

        save0 = k_cs[0];
        save1 = k_cs[1];

        k_cs[0].port = tport[pass];
        k_cs[0].pin  = tpin[pass];
        k_cs[0].port->BSRR = (1UL << k_cs[0].pin);

        adc64_scan(&fr);

        k_cs[0] = save0;
        k_cs[1] = save1;

        switch (pass) {
        case 0U: n_out[0] = edge_extract(&fr, 0U, 0U, b0); break;  /* 下长边 43 */
        case 1U: n_out[1] = edge_extract(&fr, 0U, 1U, b1); break;  /* 右短边 18 */
        case 2U: n_out[2] = edge_extract(&fr, 0U, 2U, b2); break;  /* 上长边 42 */
        case 3U: n_out[3] = edge_extract(&fr, 0U, 3U, b3); break;  /* 左短边 21 */
        default: break;
        }
    }

    board_spi_select(0U);
}
'''

sa = sa.replace(old_fn, new_fn, 1)
io.open(FA, "w", encoding="utf-8").write(sa)
print("patched board.c / board.h / adc64.c")
