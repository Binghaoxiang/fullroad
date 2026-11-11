/**
  ******************************************************************************
  * @file    config.h
  * @brief   工程配置：所有需要按实际硬件修改的内容都集中在本文件
  * @note    目标板：50 寸主控板（ProPrj_50寸主�?26-09-21�?
  *          主控 MCU：STM32F429IGT6（LQFP176�?MB Flash / 192KB SRAM + 64KB CCM�?
  *
  *          本工程不依赖标准外设�?/ HAL，只�?CMSIS 头文件做寄存器级编程�?
  *          因此 Keil 里只要有 STM32F4xx 器件包就能编译�?
  *
  *          !! 重要 !!
  *          �?BOARD_HSE_HZ 时，必须同时�?Keil 工程
  *          Options for Target -> C/C++ -> Define 里的 HSE_VALUE 改成同一个值，
  *          否则 SystemCoreClock 与实际主频不符，串口波特率与 PWM 频率会一起偏�?
  ******************************************************************************
  */
#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 0. 器件选择
 *    50 寸主控板 = STM32F429IGT6。若回到旧的 F407ZGT6 板，
 *    把下面改�?STM32F407xx，并同步�?Keil �?Device 与启动文件（�?README）�?
 *    工程 Define 里已给出 STM32F429xx，这里只是兜底�?
 *============================================================================*/
#if !defined(STM32F429xx) && !defined(STM32F407xx)
  #define STM32F429xx
#endif

/*==============================================================================
 * 1. 系统时钟
 *    50 寸主控板晶振 X2 = X32258MOB4SI = 8MHz，负载电�?C29/C30 = 12pF�?
 *    8MHz HSE -> PLL(M=8, N=360, P=2) -> SYSCLK 180MHz
 *        AHB 180MHz / APB1 45MHz(定时�?90MHz) / APB2 90MHz
 *    PLLQ=8 -> 45MHz，仅 USB/SDIO 使用；本板命令口�?CH340C，不需�?48MHz�?
 *    若以后要�?MCU 原生 USB：BOARD_SYSCLK_HZ �?168000000�?
 *    并把 board.c �?PLL_N_VALUE �?336、PLL_Q_VALUE �?7�?
 *
 *    HSE 起振失败�?PLL 不锁定时自动退�?HSI(16MHz) 方案，UART/PWM 依然正确�?
 *============================================================================*/
#if !defined(HSE_VALUE)
  #define HSE_VALUE                 8000000UL
#endif
#define BOARD_HSE_HZ                8000000UL
#define BOARD_SYSCLK_HZ             180000000UL
#define BOARD_SYSTICK_HZ            1000UL        /* 1kHz 时基：舵机速度规划的最小时间片 */

/*==============================================================================
 * 2. 命令串口：UART8 -> CH340C -> Type-C（插�?Type-C 就是虚拟串口，免 USB-TTL�?
 *    原理图：CH340C.TXD -> UART8_RX(PE0)，CH340C.RXD -> UART8_TX(PE1)
 *    板上另有 UART2(PA2/PA3)、UART4(PA0/PA1)，原理图上留�?WiFi / 其它模块�?
 *============================================================================*/
#define BOARD_UART                  UART8
#define BOARD_UART_IRQn             UART8_IRQn
#define BOARD_UART_IRQ_HANDLER      UART8_IRQHandler
#define BOARD_UART_GPIO             GPIOE
#define BOARD_UART_GPIO_AHB1EN      RCC_AHB1ENR_GPIOEEN
#define BOARD_UART_APB1EN           RCC_APB1ENR_UART8EN
#define BOARD_UART_TX_PIN           1U            /* PE1 = UART8_TX -> CH340C */
#define BOARD_UART_RX_PIN           0U            /* PE0 = UART8_RX <- CH340C */
#define BOARD_UART_AF               8U            /* AF8 = UART4/5/7/8 */
#define BOARD_UART_BAUD             115200UL

/*==============================================================================
 * 3. 舵机信号 PWM：TIM4_CH1 -> PD12
 *    50 寸主控板原理图上没有舵机接口（板上只�?6 �?SPI 采集口�?6 �?FPC�?
 *    WiFi、双 Type-C），PD12 经查为空闲脚（LQFP176 �?100 脚），飞线即可�?
 *
 *    TIM4 �?APB1�?80MHz 下定时器时钟 90MHz，PSC 分频后计数时�?= 2MHz�?
 *    �?0.5us 分辨率（�?1us 精度提高一倍），周期仍精确 20.000ms�?
 *
 *    其它已确认空闲的定时器输出脚�?
 *      PB6 = TIM4_CH1    PA8 = TIM1_CH1    PF9 = TIM14_CH1
 *    换引脚只要改下面�?71~77 行；换到 TIM1/TIM8 �?APB2 定时器时�?
 *    SERVO_TIM_APB1EN 换成 SERVO_TIM_APB2EN（如 RCC_APB2ENR_TIM1EN）�?
 *============================================================================*/
#define SERVO_TIM                   TIM4
#define SERVO_TIM_APB1EN            RCC_APB1ENR_TIM4EN
#define SERVO_GPIO                  GPIOD
#define SERVO_GPIO_AHB1EN           RCC_AHB1ENR_GPIODEN
#define SERVO_PIN                   0U           /* PD0 = H5 pin 8 */
#define SERVO_PIN_AF                2U           /* AF2 = TIM4_CH3 */
#define SERVO_CCR                   CCR3
#define SERVO_PWM_PERIOD_US         20000UL       /* 50Hz，舵机标准刷新率 */
#define SERVO_PWM_TICKS_PER_US      2UL           /* 2 = 0.5us 分辨率；1 = 1us */

/*==============================================================================
 * 4. 舵机模型默认参数（可用串口命�?W / L / T / S 在线修改，掉电不保存�?
 *
 *    国华(GH) 系列标准舵机，常见规�?500~2500us 对应 0~180°�?
 *    不同批次/型号的实际行程端点会有几�?us 的差异，标定方法�?README�?
 *    若为 270° 型号：把 SERVO_ANGLE_MAX_MDEG 改成 270000L 即可�?
 *============================================================================*/
#define SERVO_PULSE_MIN_US_DEF      500U          /* 对应 0°    */
#define SERVO_PULSE_MAX_US_DEF      2500U         /* 对应 180°  */
#define SERVO_ANGLE_MIN_MDEG_DEF    0L
#define SERVO_ANGLE_MAX_MDEG_DEF    180000L       /* 单位 0.001°，内部全整数运算 */
#define SERVO_SPEED_MDEG_S_DEF      180000UL      /* 180°/s�? 表示不限速（全速） */
#define SERVO_TRIM_MDEG_DEF         0L            /* 安装零点修正 */

#define SERVO_START_ANGLE_MDEG      90000L        /* 上电起始角度 90.000° */
#define SERVO_START_DELAY_MS        500UL         /* 上电延时后再输出脉冲，等电源稳定 */

/* 绝对安全边界：任何命令都不可能把脉宽打到这个范围之外，保护舵机不堵转烧毁 */
#define SERVO_PULSE_ABS_MIN_US      200U
#define SERVO_PULSE_ABS_MAX_US      3000U

/*==============================================================================
 * 5. 分档位点动（�?多级的指令转�?�?
 *    �?+ / - 时按当前档位的步长走一步，N1~N4 切档�?
 *    最低档�?PWM 分辨率限制：0.5us �?0.045°，所�?0.1° 档是实际可用下限�?
 *    再调小（0.01°）在脉宽上体现不出来�?
 *============================================================================*/
#define SERVO_JOG_LEVELS            4U
#define SERVO_JOG_STEP_TABLE        { 100L, 1000L, 5000L, 45000L }  /* 0.1° | 1° | 5° | 45° */
#define SERVO_JOG_LEVEL_DEF         2U            /* 上电默认�?2 �?= 1° */

/*==============================================================================
 * 6. 自动对准（单边探测器模拟�?
 *
 *    板上 01~20 那些模拟通道是走 FPC 到采集板的，并没有引�?MCU �?ADC 脚，
 *    所以台架模拟需要在空闲�?ADC 引脚上飞线。下面默认用 PC0~PC3�?
 *        PC0 = ADC123_IN10   PC1 = ADC123_IN11
 *        PC2 = ADC123_IN12   PC3 = ADC123_IN13     （这 4 个脚在原理图上都是空的）
 *    实际只有 1 个探测器时，�?TRK_CH_COUNT 改成 1，只�?PC0 即可�?
 *
 *    通道顺序必须与物理位置一致（从左到右），S 的符号才正确�?
 *============================================================================*/
#define TRK_CH_COUNT            4U                                    /* 探测器路�?1..8 */
#define TRK_ADC_GPIO            GPIOC
#define TRK_ADC_GPIO_AHB1EN     RCC_AHB1ENR_GPIOCEN
#define TRK_ADC_PINS            { 0U, 1U, 2U, 3U }                    /* PC0..PC3 */
#define TRK_ADC_CHANNELS        { 10U, 11U, 12U, 13U }                /* ADC1_IN10..13 */

#define TRK_AVG_SAMPLES         16U       /* 每组平均次数�? 组取中值，�?3×16 次采样） */
#define TRK_DARK_SAMPLES        200U      /* 暗电流标定采样次�?*/
#define TRK_MIN_TOTAL           300U      /* 总强度低于此值认�?无光" */

#define TRK_SETTLE_MS           300U      /* 每次移动后等结构稳定的时�?*/
#define TRK_SEARCH_STEP_MDEG    2000L     /* 搜索状态步�?2° */
#define TRK_STEP_COARSE_MDEG    500L      /* 粗调步长 0.5° */
#define TRK_STEP_FINE_MDEG      100L      /* 精调步长 0.1° */
#define TRK_STEP_MIN_MDEG       50L       /* 最小步�?0.05° */
#define TRK_STEP_MAX_MDEG       1000L     /* 单步上限 1.0°（大惯量结构防止过冲�?*/
#define TRK_KP_MDEG_PER_PERMILLE 5L       /* 步长 = Kp × |S(�?|，S=100�?时走 0.5° */

#define TRK_S_DEADBAND          20L       /* |S| < 20�?判为锁住 */
#define TRK_S_OUTBAND           60L       /* |S| > 60�?判为失锁，重新追 */
#define TRK_P_DROP_PERMILLE     250L      /* P �?25% 判为失锁 */
#define TRK_DITHER_MDEG         300L      /* 方案 B 的扰动步�?0.3° */

/*==============================================================================
 * 6.1 ADC64 采集板（ADS7953 16 通道 + 4:1 行选）—�?两条边各一块板
 *
 *    两块板的 **SDO 已经并联到同一�?MISO**，这�?SPI 多从机的标准做法�?
 *      ADS7953 �?SDO �?CS 为高时是高阻态，只有被选中的那块才驱动数据线，
 *      所�?SCK / SDI / SDO 三根共享，只需 **CS 分开**�?
 *    !! 任何时刻只允许一块板�?CS 为低，否则两块同时驱�?�?总线冲突（会烧片子）�?
 *
 *    共用：SCK=PE2  SDO(->MISO)=PE5  SDI(<-MOSI)=PE6  EN1~EN4=PC0~PC3
 *    独立：板 A(长边) CS=PE3        �?B(短边) CS=PE4
 *
 *    接线�?docs/接线与联调步�?md
 *============================================================================*/
#define ADC64_BOARD_COUNT        2U

/* One shared SPI4; the two boards are selected by their own CS. */
/* SCK=PE2  SDO->MISO=PE5  SDI<-MOSI=PE6                          */
#define ADC64_SPI               SPI4
#define ADC64_SPI_APB2EN        RCC_APB2ENR_SPI4EN
#define ADC64_GPIO              GPIOE
#define ADC64_GPIO_AHB1EN       RCC_AHB1ENR_GPIOEEN
#define ADC64_SCK_PIN           2U            /* PE2 */
#define ADC64_MISO_PIN          5U            /* PE5 <- SDO (yellow wires) */
#define ADC64_MOSI_PIN          6U            /* PE6 -> SDI (fly wire) */
#define ADC64_AF                5U
#define ADC64_SPI_BR            7U            /* fPCLK2/32 = 2.8MHz */
#define ADC64_A_CS_PORT         GPIOI
#define ADC64_A_CS_PIN          8U            /* PI8 = SPI4_CS4 = FPC12 = 下长边 E0 */
#define ADC64_B_CS_PORT         GPIOH
#define ADC64_B_CS_PIN          4U            /* PH4 = SPI4_CS0 = FPC16 = 右短边 E1 */
#define ADC64_C_CS_PORT      GPIOC
#define ADC64_C_CS_PIN       13U
#define ADC64_D_CS_PORT      GPIOI
#define ADC64_D_CS_PIN       9U
#define ADC64_SPI_BR            7U            /* fPCLK2/32 = 2.8MHz */

/* ---- 两块板共用行�?---- */
#define ADC64_EN_PORT           GPIOD
#define ADC64_EN_GPIO_AHB1EN    RCC_AHB1ENR_GPIODEN
#define ADC64_EN_A_PORT_1       GPIOD
#define ADC64_EN_A_PIN_1        13U
#define ADC64_EN_A_PORT_2       GPIOD
#define ADC64_EN_A_PIN_2        12U
#define ADC64_EN_A_PORT_3       GPIOH
#define ADC64_EN_A_PIN_3        11U
#define ADC64_EN_A_PORT_4       GPIOH
#define ADC64_EN_A_PIN_4        10U  /* nets 17-20 = PD13/PD12/PH11/PH10 -> FPC12 下长边 E0 */
#define ADC64_EN_B_PORT_1       GPIOD
#define ADC64_EN_B_PIN_1        7U   /* nets 01-04 = PD7/PD6/PD5/PD4 -> FPC16 右短边 E1 */
#define ADC64_EN_B_PORT_2       GPIOD
#define ADC64_EN_B_PIN_2        6U
#define ADC64_EN_B_PORT_3       GPIOD
#define ADC64_EN_B_PIN_3        5U
#define ADC64_EN_B_PORT_4       GPIOD
#define ADC64_EN_B_PIN_4        4U
#define ADC64_EN_SETTLE_US      2000U          /* 切行后等模拟通路稳定（EN 组已隔离，负载降低） */
#define ADC64_MIN_TOTAL         2000U         /* �?有光"的最小总码�?*/
/*==============================================================================
 * 7. 版本信息
 *============================================================================*/
#define FW_VERSION                  "2.2"

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */
