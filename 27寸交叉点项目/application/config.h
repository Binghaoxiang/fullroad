// config.h
#ifndef CONFIG_H
#define CONFIG_H

#define EDGE_UP      0
#define EDGE_DOWN    1
#define EDGE_LEFT    2
#define EDGE_RIGHT   3
#define EDGE_IN_UP   4
#define EDGE_IN_DOWN 5
#define EDGE_IN_LEFT 6
#define EDGE_IN_RIGHT 7 
#define EDGE_COUNT   8

// ==================== 编译时只改这几个 ====================
#ifndef MY_EDGE
#define MY_EDGE       EDGE_UP      // 当前烧录目标：外上左侧主控
#endif
#ifndef MY_SEGMENT
#define MY_SEGMENT    0             // 外上左段
#endif
#ifndef MCU_IS_MASTER
#define MCU_IS_MASTER 1             // 主控
#endif

// ==================== 板型选择（本 MCU 按实物只改这一行） ====================
// 30cm 板: 7 个 mux, 前 6 个各接 8 颗, 第 7 个接 2 颗 => 6*8+2 = 50 颗/段
// 35cm 板: 8 个 mux, 前 7 个各接 8 颗, 第 8 个接 2 颗 => 7*8+2 = 58 颗/段
#define BOARD_30CM   0
#define BOARD_35CM   1
#ifndef BOARD_TYPE
#define BOARD_TYPE    BOARD_30CM    // 外上左侧30cm板
#endif

// ==================== 采集路径选择（v2 采集核心，按原理图接线）====================
// 原理图: ADC1(PA0) = 带通+包络检波输出    ADC2(PB2) = TIA 裸信号
//   SENSOR_PATH_TIA : TIA 裸信号 + 固件18kHz锁相  ← 推荐
//                     切换尖峰 TIA 1.4µs 就衰减完，不会被带通"振铃"污染；
//                     18kHz 选择性由固件锁相(数字滤波)提供，同样抗环境光
//   SENSOR_PATH_ENV : 带通+包络（慢路）
//                     切换尖峰会激励带通(17.5~22kHz)，振铃约 330µs ✗
//                     必须把 MUX_STABLE_US 加到 >=400µs，否则读到上一通道的振铃
#define SENSOR_PATH_TIA   0
#define SENSOR_PATH_ENV   1
#define SENSOR_PATH       SENSOR_PATH_TIA

// SENSOR_MODE: 旧接口(0/1 单路)保留兼容；新采集核心用哪路由 SENSOR_PATH 决定
// 2(双路)在新核心下不支持（锁相/突发解调每次只处理一路，v2 已去掉双路模式）
#define SENSOR_MODE   1

// ==================== 临时调试：固定单通道（验证 mux/ADC/滤波） ====================
// 1=只固定拨通一颗传感器（mux 不动，主循环打印 raw/filt），0=正常扫描
#define TEST_FIXED_SENSOR  0
#define TEST_UPPER  0  // 固定的 bank（0-7）
#define TEST_LOWER  0  // 固定的 channel（0-7）



#define SEG_TAIL_COUNT   2  // 最后一个 mux 上实际接的传感器数
#define MUX_LOWER_MAX    8  // 每个 mux 的通道数
#define MUX_FULL_30CM    6  // 30cm 板满 8 通道的 mux 个数
#define MUX_FULL_35CM    7  // 35cm 板满 8 通道的 mux 个数
#define SEG_SENSORS_30CM (MUX_FULL_30CM * MUX_LOWER_MAX + SEG_TAIL_COUNT)  // 50 颗/段
#define SEG_SENSORS_35CM (MUX_FULL_35CM * MUX_LOWER_MAX + SEG_TAIL_COUNT)  // 58 颗/段

#if   BOARD_TYPE == BOARD_30CM
  #define MUX_FULL_COUNT   MUX_FULL_30CM
#elif BOARD_TYPE == BOARD_35CM
  #define MUX_FULL_COUNT   MUX_FULL_35CM
#else
  #error "BOARD_TYPE must be BOARD_30CM or BOARD_35CM"
#endif

#define MUX_UPPER_MAX    (MUX_FULL_COUNT + 1)
#define SEG_SENSOR_COUNT (MUX_FULL_COUNT * MUX_LOWER_MAX + SEG_TAIL_COUNT)

// ==================== 时钟源 ====================
// 混晶振: 全部板型都先试外部晶振(HSE)，起振失败自动回退内部 HSI(两路都锁 170MHz)。
// 30cm 板焊了晶振、35cm 板没有 —— 统一 try 才能"内部/外部晶振混用"，
// 没焊晶振的板只是多花几 ms 启动时间，功能与频率完全一致。
#ifndef BOARD_TRY_HSE
  #define BOARD_TRY_HSE  1
#endif

// ==================== 各边段数（按实物布局） ====================
#define UP_SEGMENTS       2
#define DOWN_SEGMENTS     2
#define LEFT_SEGMENTS     1
#define RIGHT_SEGMENTS    1
#define IN_UP_SEGMENTS    2
#define IN_DOWN_SEGMENTS  2
#define IN_LEFT_SEGMENTS  1
#define IN_RIGHT_SEGMENTS 1
#define TOTAL_MCU_COUNT   (UP_SEGMENTS + DOWN_SEGMENTS + LEFT_SEGMENTS + RIGHT_SEGMENTS + IN_UP_SEGMENTS + IN_DOWN_SEGMENTS + IN_LEFT_SEGMENTS + IN_RIGHT_SEGMENTS)

// 两路模式时缓冲/上传点数翻倍
#if SENSOR_MODE == 2
  #define SENSOR_UNIT 2
#else
  #define SENSOR_UNIT 1
#endif

// 本 MCU 每段采集点数
#define MY_SENSOR_COUNT  (SEG_SENSOR_COUNT * SENSOR_UNIT)

// 单边数据缓冲上限（3 段 * 每段 116 = 348，留余量）
#define MAX_SENSORS_PER_EDGE 512

// 各边总点数（缓冲/上传/原始帧用，已含 SENSOR_UNIT）
#define UP_SENSORS    (UP_SEGMENTS   * SEG_SENSOR_COUNT * SENSOR_UNIT)
#define DOWN_SENSORS  (DOWN_SEGMENTS * SEG_SENSOR_COUNT * SENSOR_UNIT)
#define LEFT_SENSORS  (LEFT_SEGMENTS * SEG_SENSOR_COUNT * SENSOR_UNIT)
#define RIGHT_SENSORS (RIGHT_SEGMENTS* SEG_SENSOR_COUNT * SENSOR_UNIT)
#define IN_UP_SENSORS    (IN_UP_SEGMENTS   * SEG_SENSOR_COUNT * SENSOR_UNIT)
#define IN_DOWN_SENSORS  (IN_DOWN_SEGMENTS * SEG_SENSOR_COUNT * SENSOR_UNIT)
#define IN_LEFT_SENSORS  (IN_LEFT_SEGMENTS * SEG_SENSOR_COUNT * SENSOR_UNIT)
#define IN_RIGHT_SENSORS (IN_RIGHT_SEGMENTS* SEG_SENSOR_COUNT * SENSOR_UNIT)

// 各边物理传感器数（坐标归一化用，不含 SENSOR_UNIT）
#define UP_SENSORS_PHYS    (UP_SEGMENTS   * SEG_SENSOR_COUNT)
#define DOWN_SENSORS_PHYS  (DOWN_SEGMENTS * SEG_SENSOR_COUNT)
#define LEFT_SENSORS_PHYS  (LEFT_SEGMENTS * SEG_SENSOR_COUNT)
#define RIGHT_SENSORS_PHYS (RIGHT_SEGMENTS* SEG_SENSOR_COUNT)
#define IN_UP_SENSORS_PHYS    (IN_UP_SEGMENTS   * SEG_SENSOR_COUNT)
#define IN_DOWN_SENSORS_PHYS  (IN_DOWN_SEGMENTS * SEG_SENSOR_COUNT)
#define IN_LEFT_SENSORS_PHYS  (IN_LEFT_SEGMENTS * SEG_SENSOR_COUNT)
#define IN_RIGHT_SENSORS_PHYS (IN_RIGHT_SEGMENTS* SEG_SENSOR_COUNT)

// ==================== 镜像翻转（按边开关，采集后翻转传感器顺序） ====================
// 某些边硬件安装方向导致传感器顺序与坐标方向相反，采集后需镜像翻转。
// 1=翻转, 0=不翻转。按本板 MY_EDGE 自动选择对应宏。
#define MIRROR_UP        0
#define MIRROR_DOWN      1
#define MIRROR_LEFT      0
#define MIRROR_RIGHT     0
#define MIRROR_IN_UP     0
#define MIRROR_IN_DOWN   1
#define MIRROR_IN_LEFT   0
#define MIRROR_IN_RIGHT  0

// 物理段序与默认段序相反的边：整条边反向，不能只做段内镜像
#define REVERSE_EDGE_DOWN     0
#define REVERSE_EDGE_IN_DOWN  0
#define REVERSE_EDGE_LEFT     0
#define REVERSE_EDGE_IN_LEFT  0

#define MY_MIRROR 0
#if   MY_EDGE == EDGE_UP       && MIRROR_UP
#undef  MY_MIRROR
#define MY_MIRROR 1
#elif MY_EDGE == EDGE_DOWN     && MIRROR_DOWN
#undef  MY_MIRROR
#define MY_MIRROR 1
#elif MY_EDGE == EDGE_LEFT     && MIRROR_LEFT
#undef  MY_MIRROR
#define MY_MIRROR 1
#elif MY_EDGE == EDGE_RIGHT    && MIRROR_RIGHT
#undef  MY_MIRROR
#define MY_MIRROR 1
#elif MY_EDGE == EDGE_IN_UP    && MIRROR_IN_UP
#undef  MY_MIRROR
#define MY_MIRROR 1
#elif MY_EDGE == EDGE_IN_DOWN  && MIRROR_IN_DOWN
#undef  MY_MIRROR
#define MY_MIRROR 1
#elif MY_EDGE == EDGE_IN_LEFT  && MIRROR_IN_LEFT
#undef  MY_MIRROR
#define MY_MIRROR 1
#elif MY_EDGE == EDGE_IN_RIGHT && MIRROR_IN_RIGHT
#undef  MY_MIRROR
#define MY_MIRROR 1
#endif

// ==================== 主控: 各边各段板型表（同一条边内可逐段混用） ====================
// 从机不用填此表(从机只按自身 BOARD_TYPE 采集/发送); 主控必须按实物逐段填写,
// 个数必须与各边段数一致, 否则 CAN 解析错位。主控自己那一段必须与本机 BOARD_TYPE 相同。
// 27 寸膜实物布局:
//   up       = 30,35
//   down     = 35,30
//   left/right = 35
//   in_up   = 30,35  (内上探测器槽延伸到外框，左端不从内屏原点开始)
//   in_down = 35,30  (内下探测器槽延伸到外框，左端不从内屏原点开始)
//   in_left/in_right = 35
#define UP_SEG_BOARD        BOARD_30CM, BOARD_35CM
#define DOWN_SEG_BOARD      BOARD_35CM, BOARD_30CM
#define LEFT_SEG_BOARD      BOARD_35CM
#define RIGHT_SEG_BOARD     BOARD_35CM
#define IN_UP_SEG_BOARD     BOARD_30CM, BOARD_35CM
#define IN_DOWN_SEG_BOARD   BOARD_35CM, BOARD_30CM
#define IN_LEFT_SEG_BOARD   BOARD_35CM
#define IN_RIGHT_SEG_BOARD  BOARD_35CM

// ==================== 公共常量 ====================
// 全系统总段数（组帧缓冲按最坏情况 35cm 板分配）
#define TOTAL_SEGMENTS   (UP_SEGMENTS + DOWN_SEGMENTS + LEFT_SEGMENTS + RIGHT_SEGMENTS + \
                          IN_UP_SEGMENTS + IN_DOWN_SEGMENTS + IN_LEFT_SEGMENTS + IN_RIGHT_SEGMENTS)
#define FRAME_RAW_BYTES_MAX  (TOTAL_SEGMENTS * SEG_SENSORS_35CM * SENSOR_UNIT * 2)

#define ADC_COUNT       1
#define ADC_SAMPLE_COUNT  32
#define ADC_DISCARD_COUNT 4

// ADC 采样窗口(ADC时钟周期数)档位: 0=2.5 / 1=24.5 / 2=47.5 / 3=247.5
// 仅作为 CubeMX 初始值；实际采样时间由 bsp_adc_init 按路径分路覆盖
// (ENV 路 6.5 周期短孔径采尖峰, TIA 路 24.5 周期长孔径充分建立)
#define ADC_SAMPLETIME_SEL   1
#if   ADC_SAMPLETIME_SEL == 0
  #define ADC_USER_SAMPLETIME  ADC_SAMPLETIME_2CYCLES_5
#elif ADC_SAMPLETIME_SEL == 1
  #define ADC_USER_SAMPLETIME  ADC_SAMPLETIME_24CYCLES_5
#elif ADC_SAMPLETIME_SEL == 2
  #define ADC_USER_SAMPLETIME  ADC_SAMPLETIME_47CYCLES_5
#elif ADC_SAMPLETIME_SEL == 3
  #define ADC_USER_SAMPLETIME  ADC_SAMPLETIME_247CYCLES_5
#else
  #error "ADC_SAMPLETIME_SEL must be 0/1/2/3"
#endif
#define HW_SENSOR_MAX   (MUX_UPPER_MAX * MUX_LOWER_MAX * SENSOR_UNIT)

// ==================== v2 采集核心参数 ====================
// MUX 建立时间（切换后到开始采样的等待，背景校准与采集必须一致！）
//   TIA 路径: 切换尖峰 ~1.4µs 衰减完, 20µs 充足
//   ENV 路径: 带通振铃 ~330µs, 必须 >=400µs（否则读到上一通道振铃）
#define MUX_STABLE_TIA_US 6
#define MUX_STABLE_US     400
#define MUX_STABLE_CUR    ((SENSOR_PATH == SENSOR_PATH_ENV) ? MUX_STABLE_US : MUX_STABLE_TIA_US)

// 换 bank 时切换台阶更大（电荷注入更多），额外多等这段时间
#define MUX_BANK_EXTRA_US 20

// 带通包络路突发采样点数（取峰用）: ~0.45µs/点 × 512 ≈ 4 个 18kHz 周期,
// 更多"圈数"让采样梳齿相位覆盖更全, 抵抗与光源频率的相位锁定伪影
#define ENV_BURST_SAMPLES 512

// 18kHz 锁相解调（TIA 路用调制激光时开=1；常亮光源=0）
// 1=锁相解调(适合 18kHz 调制光源, 抗环境光/直流); 0=32 点去极值平均(常亮光源)
// 注: 锁相结果与开机时是否有光无关（bg 只用于防溢出, 均值扣除对直流免疫）
#define DEMOD_18K_ENABLE  1
// 18kHz 基频锁相（适配 50% 及以上占空比；36kHz 二次谐波在 50% 占空比时为 0）
#define DEMOD_FREQ_HZ     17974
#define DEMOD_SAMPLES     28  // 采样时刻表 demod_at[] 在 bsp_sensor.c:
                                 // 2µs 均匀密采, 窗口 54µs(≈1 个 18kHz 周期)
// 四周期匹配锁相：约222us窗口，噪声带宽约为单周期的1/4。
// 同一批样本同时计算相邻正交频点，宽带开关尖刺在两个频点都有能量，
// 真实18kHz调制光只在目标频点占优，据此做频域恒虚警门控。
#define DEMOD_ADAPT_CYCLES   4
#define DEMOD_ADAPT_SAMPLES  (DEMOD_SAMPLES * DEMOD_ADAPT_CYCLES)
#define DEMOD_SPECTRAL_GUARD_ENABLE 1
#define DEMOD_SPECTRAL_GUARD_RATIO  1.20f

#define DYN_BG_ENABLE   0  // 动态背景(旧): 由新的"平滑+基线减除"替代, 保持关闭

// ==================== 采集后处理 ====================
// 空间平滑 + 基线减除(替代强噪声门):
//   锁相 |I|+|Q| 对噪声有正偏置(暗态均值~76/尾巴~250) -> 波形像"噪声很强";
//   5 点平滑[v2 同款抽头]压掉逐通道独立噪声 ~50%, 光斑翼部(空间相关)保留;
//   再减本帧中位数 -> 暗态归 0, 弱光斑翼部完整保留 -> 响应点数变多
#define ACQ_SMOOTH_ENABLE       1

// 残留散点绝对门: 平滑+基线后低于此值的点归零（0=关闭）
// 暗态平滑残余 max≈84 -> 取 100 可让暗态完全归零; 光斑翼部 100+ 基本完整保留
#define NOISE_GATE_ABS          12

// 主控整边鲁棒单峰滤波。CAN拼接完成后统一处理，UI看到的也是此结果。
// 先用中位数/MAD估计暗噪声，再保留最强连续峰；同侧内外必须相互印证。
#define WAVE_FILTER_ENABLE            1
#define WAVE_NOISE_ABS                8u
#define WAVE_NOISE_MAD_K              5u
#define WAVE_WING_ABS                 3u
#define WAVE_WING_MAD_K               2u
#define WAVE_MIN_CORE_WIDTH           2u
// 6 mm间距下窄光斑可能只让一颗二极管越过core门限。若该点明显高于
// MAD噪声，仍允许进入后续3帧稳定性确认；普通暗噪声不能靠单帧通过。
#define WAVE_SINGLE_CORE_ABS          8u
#define WAVE_SINGLE_CORE_MAD_K        5u
#define WAVE_EDGE_GUARD_SENSORS       1u
#define WAVE_MAX_SUPPORT_WIDTH        28u
#define WAVE_CONFIRM_FRAMES           3u
#define WAVE_TRACK_JUMP_SENSORS       10.0f
// 单峰确认后只平滑同一空间支撑内的幅值；支撑外立即清零，因此不产生拖尾/双峰。
// Q8=64即alpha=0.25，静止波形明显稳定，真实移动的新通道仍立即建立。
#define WAVE_TEMPORAL_ALPHA_Q8        64u
// 产品光路是十字：至少一条水平探测边和一条垂直探测边同时有单峰，
// 且总支持边数不少于3。允许任意单边缺失，但拒绝单轴/少数板的偶发串扰。
#define WAVE_REQUIRE_BOTH_AXES        1
#define WAVE_MIN_EDGE_SUPPORT         3u

// ==================== 自适应噪声门(旧版强门, 已由平滑+基线替代) ====================
#define NOISE_GATE_K            0
#define NOISE_GATE_MAX_MEDIAN   300

#if (SENSOR_PATH == SENSOR_PATH_ENV) && (MUX_STABLE_US < 400)
#warning "SENSOR_PATH_ENV: 带通振铃约330us, MUX_STABLE_US 应 >= 400"
#endif

#define SAMPLE_PERIOD_MS  5

// 综合调试 UI 数据流：只上传板端处理后的每传感器幅值，
// 以及固件已经算好的质心、坐标和交点调试量。UI 只显示，不重算。
// 不采集、不经 CAN/串口发送处理前副本。
#define DEBUG_UI_STREAM        1

#define SYNC_TIMEOUT_MS   300  // 从机等主机的同步帧（> 主机一轮周期）
#define DATA_TIMEOUT_MS   100 // 全部板正常到齐会立即返回；100ms只作异常保护
#define CAN_DISCOVERY_GRACE_MS  10 // 每轮至少留出时间接收/发现全部从机首帧
                                       // 就把超时降到 20ms：全 24 段原始帧 1M CAN 约 29ms，
                                       // 20ms 会在最后几段到达前超时并清标志，导致下一帧错位。
                                       // 正常到齐时 wait 在最后一段就返回，超时值不影响高帧率。
#define SLAVE_TX_TIMEOUT_MS 100 // 低优先级边在总线突发仲裁时也必须等到成功入队

// ==================== CAN 速率模式 ====================
// 0 = 1Mbps 整帧（原状，最稳）
// 1 = CAN-FD BRS：仲裁段 1Mbps + 数据段 2Mbps（标准 FD 提速，推荐先试；需 FD 收发器）
// 2 = 整帧 2Mbps（仲裁/数据都 2M；对收发器/走线要求最高）
// ⚠ 改速率后：总线上所有板必须重烧同版本；通信异常先回 0
// 帧时间预期(64B)：1M 0.60ms → BRS2M ≈0.32ms → 整帧2M 0.30ms
#define CAN_SPEED_MODE  0

// 等待从机策略:
//  0 = 必须集齐全部 EDGE_COUNT 条边(最严; 缺板白等满超时)
//  1 = 只等"见过面"的边(整条边所有段到齐; 全系统接齐时与 0 等价)
//  2 = 只等"见过面"的段(台架逐块调试: 没接的段不白等超时;
//      注意缺段时该段数据为上轮缓存)
#define CAN_WAIT_MODE  0

// 27寸膜: 外圈 649mm x 402mm，内圈 599mm x 352mm，内外圈间距 25mm
#define SCREEN_W  599.0f
#define SCREEN_H  352.0f

/* 探测器沿边方向的默认几何模型。
 *
 * 30/35 cm 是PCB名义长度，不是二极管有效长度；机械外框尺寸也不是
 * 探测器阵列长度。当前PCB均按6 mm节距、阵列在PCB上居中建模：
 *   30 cm: 50颗，中心跨度49*6=294 mm，两端各留3 mm；
 *   35 cm: 58颗，中心跨度57*6=342 mm，两端各留4 mm。
 * 多块PCB按实际板型顺序分段映射，整组PCB在对应膜边上居中安装。
 * 这些值只提供未标定时的物理初值；实际焊盘位置、板间隙和装配偏移
 * 最终必须由固件多点标定修正。拿到PCB CAD后应以CAD值替换这里。 */
#define DETECTOR_PITCH_MM              6.0f
#define PCB_30_NOMINAL_LENGTH_MM     300.0f
#define PCB_35_NOMINAL_LENGTH_MM     350.0f
#define PCB_30_FIRST_DETECTOR_MM       3.0f
#define PCB_35_FIRST_DETECTOR_MM       4.0f
#define UP_X_MIN     (-25.0f)
#define UP_X_MAX     624.0f
#define DOWN_X_MIN   (-25.0f)
#define DOWN_X_MAX   624.0f
#define LEFT_Y_MIN   (-25.0f)
#define LEFT_Y_MAX   377.0f
#define RIGHT_Y_MIN  377.0f
#define RIGHT_Y_MAX  (-25.0f)

// 新结构中四条长边探测器总成都延伸到外框宽度：
//   上外/上内均为左30+右35；下外/下内均为左35+右30。
// 内屏坐标原点仍为有效区左下角，但内长边PCB左端位于原点左侧约25mm，
// 不能把内上/内下探测器强行映射为0..599。实际默认映射由board_map.c按
// 分段PCB居中到599mm坐标系，机械覆盖约-25..624；标定后以实测节点为准。
#define IN_UP_X_CAL_SHIFT 0.0f
#define IN_UP_X_MIN       (-25.0f)
#define IN_UP_X_MAX       624.0f
// 右侧: 内圈起点相对外圈内缩5mm(外圈起点-25) -> [-20.0, 676.7-3.1=673.6]
#define IN_RIGHT_Y_MIN    352.0f
#define IN_RIGHT_Y_MAX    0.0f
// 底/左暂未实测, 维持标定值, 测好后同步
#define IN_DOWN_X_MIN     (-25.0f)
#define IN_DOWN_X_MAX     624.0f
#define IN_LEFT_Y_MIN     0.0f
#define IN_LEFT_Y_MAX     352.0f

#define LEFT_Y0_MIN      0.0f
#define LEFT_Y0_MAX      176.0f
#define LEFT_Y1_MIN      176.0f
#define LEFT_Y1_MAX      352.0f
#define IN_LEFT_Y0_MIN   0.0f
#define IN_LEFT_Y0_MAX   176.0f
#define IN_LEFT_Y1_MIN   176.0f
#define IN_LEFT_Y1_MAX   352.0f

/* IN_LEFT 索引折算到 LEFT 外圈的等效索引：垂直扫描拟合 */
#define IN_LEFT_OUTER_A  1.0f
#define IN_LEFT_OUTER_B  0.0f

// ==================== 内外边质心连线交点定位 ====================
// 同一侧内外两条传感器条的垂直/水平间距(mm):
//   上下两条水平条相距 25（外圈条整体比内圈每端多 25mm 垂直）；
//   左右两条竖直条相距 25（水平方向每端多 25mm）。
// 27寸膜内外圈间距: (649-599)/2 = (402-352)/2 = 25mm
#define IN_UP_OFFSET_Y     25.0f
#define IN_DOWN_OFFSET_Y   25.0f
#define IN_LEFT_OFFSET_X   25.0f
#define IN_RIGHT_OFFSET_X  25.0f

// 连线方向(内-外质心差)的时域平滑系数：0=不平滑；越接近1越稳但更滞后。
// 方向本质是安装/视差常数，不是每帧随机量；高帧率下 0.75 等效时间常数只有 4 帧，
// 所以这里用更慢的0.98（100fps≈0.5s，17fps≈3s）。平滑后的方向直接用于交点连线。
#define LINE_DIR_EMA  0.98f
// 连线方向启动保护：
//   上电第一帧 in_right 只有 45.7（正常 70.9），peak 仅 0.011；旧代码把它当有效方向，
//   初始化出 -157mm/20mm 的斜率，EMA 0.98 要几秒才收敛，绿菱形因此很久才出现。
//   峰值低于门限的边不参与方向初始化；单帧方向超过 MAX_MM 或相对当前值跳变超过
//   RESET_MM 时按异常处理，重新初始化。
#define LINE_DIR_MIN_PEAK   0.02f
#define LINE_DIR_MAX_MM     30.0f
#define LINE_DIR_RESET_MM   15.0f

#define CENTROID_THRESHOLD  0.0025f
#define MIN_REGION_WIDTH    3

// 质心算法：1=峰值锚定新算法(相对峰值窗口+连续性跟踪+抛物线)，0=旧绝对阈值区域法(可回退)
#define CENTROID_ALGO_PEAK    1
// 窗口边界 = max(峰值*该比例, 绝对阈值)：边界落在陡坡上，ADC噪声不会逐帧翻转窗口
#define CENTROID_PEAK_RATIO   0.5f
// 连续性跟踪半径(采样点)：上帧质心附近找局部峰，防止鬼影/野峰把质心抢走
#define CENTROID_TRACK_RADIUS 12
// 跟踪半径内的局部峰 >= 全局峰*该比例 时才优先用局部峰(否则用全局峰，保证快速移动不丢失)
#define CENTROID_TRACK_RATIO  0.4f
// 平顶判据：峰两侧采样必须至少低于峰值该比例，才启用抛物线亚像素；
// 平顶(饱和)光斑两侧几乎等高，抛物线会时灵时不灵导致跳峰，此时退回加权平均
#define CENTROID_FLAT_MARGIN  0.05f

// v3 跟踪窗高斯质心：窗口中心=上帧质心，高斯核只取峰值附近，避免峰值 26/27 跳变
// 把窗口一起带着跳。W=4(±4 点), sigma=1.2(≈±2 点有效权重)。
// 实测静止 sd: up/in_up 0.32/0.47 -> 0.05/0.05 量级，right/in_right 0.12/0.35 -> 0.03/0.03 量级。
#define CENTROID_GAUSS_W      4
#define CENTROID_GAUSS_SIGMA  1.2f

// 宽平台/双峰剖面（如 in_up）：峰附近 ±10 内 >=0.8*peak 的点数 >=5 时，
// 自动用宽窗，避免窄高斯锁到平台左边或右边。
#define CENTROID_WIDE_W        7
#define CENTROID_WIDE_SIGMA    2.5f
#define CENTROID_FLAT_RATIO    0.8f
#define CENTROID_FLAT_SCAN     10
#define CENTROID_FLAT_MIN_CNT  5
// 静止时保持连续跟踪；主峰移动超过约1个索引时，centroid.c立即重定中心。
// TRACK_EMA只负责主峰附近的细小连续变化，不再承担大范围追赶。
#define CENTROID_TRACK_EMA     0.65f
#define CENTROID_TRACK_JUMP    4.0f
#define CENTROID_RECENTER_EXTRA 2.0f
#define CENTROID_MOVE_RECENTER  1.0f

// ==================== 质心时间稳定（每边 One Euro 自适应低通） ====================
// 旧实现的问题：
//   1) 速度用 (pos-pred)/dt 直接差分，dt 变大时速度估计被噪声放大；
//   2) p_vv 只增不减、p_pp/p_vv 没有互协方差，帧率一变 K 就变，参数不可迁移；
//   3) 静止检测用原始逐帧差 d>CF_MOVE，1fps 时要连续 3 帧才进静止，滞后 3s；
//      up/in_up 原始 sd 0.4~0.9，d 很难长期小于 0.6，旧固件几乎一直处于运动模式。
// One Euro 用速度自适应截止频率：静止时 alpha 小 -> 抖动小；运动时截止频率升高 -> 延迟低。
// 内部 alpha 由 dt 计算，17fps 和 100fps 用同一组参数。
#define CENTROID_FILTER_ENABLE  1
#define CF_PI                   3.14159265f
// mincutoff: 静止截止频率(Hz)；beta: 速度每增加 1 索引/s 提高的截止频率(Hz)
#define CF_MIN_CUTOFF_WEAK      0.65f
#define CF_MIN_CUTOFF_STRONG    0.85f
#define CF_BETA_WEAK            0.080f
#define CF_BETA_STRONG          0.100f
#define CF_D_CUTOFF             2.0f  // 速度估计自身的低通截止(Hz)
// 大跳变重置(索引)：原始值和滤波值同时超过才重置，避免把真实快速移动当野值
#define CF_JUMP_WEAK            3.0f
#define CF_JUMP_STRONG          1.5f
#define CF_DEADBAND_WEAK        0.08f   // 约0.48mm（6mm间距）静止死区
#define CF_DEADBAND_STRONG      0.06f   // 约0.36mm静止死区
#define CF_LOST_MAX             3
#define CF_MEDIAN_BYPASS        1.0f    // >约6mm视为真实移动，绕过3帧中值

// ==================== 交叉点定位 ====================
// 同侧内外探测点构造视线，多条有效视线做二维加权最小二乘交点。
// 产品坐标只来自交叉点；交点无效时输出无效，不使用轴向融合点兜底。
#define POSITION_OUTLIER_MM      25.0f  // 偏离加权中位数超过此值的观测丢弃
#define POSITION_INNER_BIAS_TOP  0.0f  // 预留: 上内/外圈残余常值偏置(mm)
#define POSITION_INNER_BIAS_BOT  0.0f
#define POSITION_INNER_BIAS_LFT  0.0f
#define POSITION_INNER_BIAS_RGT  0.0f

// ==================== 内外边装配偏置在线自学习 ====================
// 同一光点被同侧内外两条边同时看到，映射后的差值就是这套装配的固定偏置。
// 开机后自动学，约 1~2 秒收敛；不需要每套装配手改 -20/-25/-19.6 之类常量。
// 只学“慢分量”：差值突然跳变 >RESET 视为换装配/野值，重新学；
// 差值在 GATE 内才慢更新，避免把运动视差学进去。
// ⚠ 交叉点要用真实内外差里的视差：自动 PairBias 会把倾斜入射的视差也一起学掉，
// 交叉点会退化成 WLS。所以做“交叉点 + 慢视差校正”时这里设 0。
// 装配偏置用 IN_UP_X_CAL_SHIFT 等一次性标定常量；不同装配做一次已知点标定。
#define POSITION_BIAS_ENABLE       0
#define POSITION_BIAS_ALPHA        0.05f
#define POSITION_BIAS_GATE_MM      8.0f
#define POSITION_BIAS_RESET_MM     20.0f
#define POSITION_BIAS_MAX_MM       100.0f
#define POSITION_BIAS_INIT_N       5
#define POSITION_BIAS_MIN_PEAK     0.02f

// ==================== 多线约束 WLS 交点 + 慢视差校正 ====================
// 相交线少于 2 条、近平行（det 比小）、方向不可信或残差大 -> 不输出二维交点。
#define POSITION_CROSS_MIN_LINES         2   // 2 条线即可求解
#define POSITION_CROSS_USE_MIN_LINES     2   // 2 条线有条件使用，3 条以上优先
#define POSITION_CROSS_2LINE_MIN_CONF    0.25f
#define POSITION_CROSS_2LINE_MAX_DIFF_MM 80.0f
#define POSITION_CROSS_JUMP_GATE_MM      60.0f
#define POSITION_CROSS_PENDING_GATE_MM   40.0f
#define POSITION_CROSS_CONFIRM_FRAMES    2
#define POSITION_CROSS_HOLD_FRAMES       15
#define POSITION_CROSS_MIN_DET_RATIO     0.02f
#define POSITION_CROSS_RESIDUAL_MAX_MM   25.0f
#define POSITION_CROSS_MIN_CONF          0.15f
#define POSITION_CROSS_WEIGHT_DIR_REF    4.0f
#define DIR_VAR_ALPHA                    0.05f
// 位置 One Euro（仅对固件交叉点做轻量自适应平滑；速度单位 mm/s）
#define KALMAN_MIN_CUTOFF        1.5f
#define KALMAN_BETA              0.04f
#define KALMAN_D_CUTOFF          2.0f
#define KALMAN_JUMP_MM           80.0f
#define KALMAN_JUMP_MMPS         3000.0f
#define KALMAN_GATE_MIN_MM       12.0f   // 仅较大异常做确认，避免拦截正常笔迹
#define KALMAN_PENDING_MM        6.0f    // 连续两帧新位置相符才接受大跳变
#define KALMAN_STILL_MM          0.35f   // 输出静止死区（不是定位精度承诺）
#define KALMAN_HOLD_SECONDS      0.20f   // 短暂丢帧保持，超时撤销有效状态
#define KALMAN_MAX_SPEED_MMPS    1200.0f // 确认后位移仍限制每帧最大移动量

// ==================== 分步耗时实测(仅主控) ====================
// 1=启用 DWT 周期计时, 每 TIMING_REPORT_EVERY 帧串口打印一次; 测完改回 0, 代码被预处理剔除
// 产品模式关掉；需要看 acq/can/period 时改成 1 重新编译。
#define TIMING_DEBUG           0
// 每多少帧打印一次(>=1)。高帧率下 30 帧才 0.3s，打印太频繁；用 100。
#define TIMING_REPORT_EVERY    100
// 1=额外实测串口在线发送时间(会等 DMA 发完, 会拖慢循环); 0=不测(sendOn 显示0, 用公式 bytes*10/baud 算)
#define TIMING_MEASURE_ONLINE  0

// ==================== 上报模式 ====================
// 1 = 全帧上报（原始波形+8边质心+坐标+交点调试, 约2632B）—— 调试/看波形用
//     2632B @460800 ≈ 57ms，会把帧率锁在 ~17FPS；产品模式必须关掉。
// 0 = 只上报坐标帧（14B: x/y/valid）—— 产品模式:
//     串口在线 57.1ms→0.30ms。当前 8 段实测：
//       acq 5.9ms + 等 CAN 7.0ms + 处理 0.6ms -> period 13.5ms (~74FPS)。
//     全 24 段时原始帧 CAN 约 29ms -> ~28FPS；要 60~100Hz 需紧凑帧
//     （每段只发峰值附近窗口/或从机算质心发 8B）或 CAN BRS 2Mbps。
//     上位机按 9 字节数据（14B 帧）自动识别。
#define SEND_FULL_FRAME        0
// 调试用：每 N 帧插一帧全帧（0=从不）。全帧会临时把该帧周期拉到 57ms，
// 建议 N>=100；需要连续波形时再用 SEND_FULL_FRAME=1 + 单独固件。
#define FULL_FRAME_EVERY_N     0

// Iteration 01 验证：发送“原始波形 + C 候选”调试帧。
// 1 = 优先发送候选调试帧（综合UI调试视图使用）
// 0 = 正常全帧/产品帧（候选层验证完成后改回 0）
#define SEND_CAND_DEBUG_FRAME  0
#define SEND_SELECT_POS        0        // 已不再控制产品输出；产品坐标始终来自固件交叉选择器

// 1 = 纯包络调试模式：只发所有传感器包络原始 uint16，不算质心/坐标/交点
//     上位机用综合UI看真实波形；关闭时改回 0
#define RAW_ONLY               0

// ==================== 多候选算法迭代（iteration 00/01）====================
// 1 = 启用多候选层，并作为产品交叉点输出的唯一几何来源
#define MULTI_CAND_ENABLE      1

// 产品默认候选数；K=4 仅调试/最坏情况（3 个端面鬼影 + 1 个真峰）
#define PROD_K                 3
#define DEBUG_K                4
#define MAX_CAND               4
#define MAX_LINES              16
#define MAX_HYPS               4

// 候选提取参数
#define CAND_ABS_THRESHOLD     0.0015f // 试调：弱边峰值约 0.004，不能被固定 0.005 门限提前滤掉
#define CAND_PROMINENCE        0.0008f // 试调：弱边保留候选，后续仍由 SNR/峰宽/连续帧门禁
#define CAND_MERGE_DIST        6.0f  // 近峰合并距离（传感器）
#define CAND_MERGE_RATIO       0.35f    // 谷底高于此比例判为同一光斑
#define CAND_RATIO_MERGE_SPAN  12       // 按谷底比例合并的最大峰间距（传感器）
#define CAND_SEG_MERGE_TOL     6        // 分段边界附近裂峰合并容差
#define CAND_SEG_MERGE_SPAN    20       // 只有两峰跨度 <= 此值才按接缝合并
#define CAND_GROUP_PAD         15  // 首/末候选组的边界余量
#define CAND_CENTROID_HALF     6        // 候选质心只统计峰值附近 ±N 点
#define CAND_WIDE_ON_SPAN      12
#define CAND_WIDE_OFF_SPAN     9
#define CAND_WIDE_CONFIRM      3
#define CAND_TRACK_RADIUS      15
#define CAND_TRACK_WINDOW      8
#define CAND_TRACK_ALPHA       0.5f
#define CAND_IN_LEFT_ALPHA     0.20f
#define CAND_LEFT_ALPHA        0.20f
#define CAND_LEFT_MED_RESET    6.0f
#define CAND_LEFT_DEADBAND     0.15f
#define CAND_IN_LEFT_DEADBAND  0.35f
#define CAND_REACQ_FRAMES      2
#define CAND_REACQ_MIN_Q       0.10f
#define CAND_PENDING_RADIUS    12.0f
#define CAND_HOLD_FRAMES       2
#define CAND_MIN_WIDTH         1.5f  // 过滤极窄尖峰
#define CAND_MAX_WIDTH         25.0f  // 过滤过宽平台
#define CAND_PEAK_REF          0.05f  // 候选质量归一化峰值参考
#define CAND_SNR_REF           20.0f  // SNR 归一化参考
#define CAND_WIDTH_TYP         7.0f  // 典型光斑宽度
#define CAND_WIDTH_SIGMA       4.0f  // 宽度惩罚尺度
// 暗态实测偶发峰可到 254 ADC；单边过门限仍不算“有光”，
// 必须再通过后续正交方向的内外成线与连续帧确认。
#define CAND_PRODUCT_MIN_PEAK    0.0020f // 试调：约 8 ADC；弱边仍需通过 SNR、峰宽和几何连续性
#define CAND_PRODUCT_MIN_SNR     2.0f    // 当前噪声估计有约 8 ADC 下限，允许弱边约 2 倍信噪比
#define CAND_PRODUCT_MIN_QUALITY 0.010f
#define CAND_PRODUCT_WIDTH_MIN   2.0f
#define CAND_PRODUCT_WIDTH_MAX   24.0f

// 内外配对/多边投票（iteration 02）
#define SELECT_D_SOFT          60.0f  // d 软惩罚宽度（mm）
#define SELECT_D_HARD          80.0f    // d 极端硬限（mm），超过直接丢弃该候选
#define SELECT_RES_GATE        25.0f  // 线到交点的内点距离（mm）
#define SELECT_DET_MIN         0.02f  // 两线近平行拒绝
#define SELECT_HQ_W            0.25f  // 高质量线权重门限
#define SELECT_MIN_LINE_W      0.0005f  // 弱边内外质量相乘约0.003；仍需峰值/SNR/宽度/正交/连续帧门禁
#define SELECT_AXIS_SPREAD_W   0.15f    // 轴向离散惩罚权重（/mm）
#define SELECT_PAIR_W          80.0f
#define SELECT_WLS_ITERATIONS  3       // 多边视线鲁棒加权最小二乘迭代次数
#define SELECT_WLS_HUBER_MM    8.0f    // 线残差超过该值后按 Huber 权重降权
#define SELECT_WLS_MIN_DET_RATIO 0.002f // 法方程条件数保护；拒绝同向/近平行线束
#define SELECT_LINE_SLOPE_ALPHA 0.12f   // 静止/慢动时综合线方向低通
#define SELECT_LINE_SLOPE_FAST_ALPHA 0.55f // 真正大角度变化仍分两三帧跟随，拒绝单帧尖跳
#define SELECT_LINE_SLOPE_FAST_MM 12.0f // 全跨度方向变化超过此值才进入快速跟随
#define SELECT_LINE_DEADBAND_MM 0.35f   // 静止输出死区，低于0.5mm精度目标
#define CENTROID_LINE_HUBER_MM  3.0f    // 四点拟合中单点偏差超过3mm自动降权
#define SELECT_COMBO_POS_W     0.05f    // 时域只解并列候选，不得压过当前帧几何证据
#define SELECT_COMBO_LINE_W    2.0f    // 组合候选线延续奖励权重
#define SELECT_SEAM_GUARD      7.0f     // 分段接缝保护区宽度（mm）
#define SELECT_SEAM_W          0.15f    // 接缝不一致时的权重系数
#define SELECT_AXIS_TEMP_SCALE 40.0f    // 轴线时间连续性惩罚尺度（mm）
#define SELECT_AXIS_SMOOTH_ALPHA 0.28f  // 选中边的共同位置低通，降低远端交点抖动
#define SELECT_AXIS_FAST_GATE_MM 4.0f   // 大于此变化视为真实移动，快速跟随
#define SELECT_CAND_TRACK      6.0f     // 候选逐帧跟踪半径（传感器索引）
#define SELECT_CAND_UPGRADE    1.3f     // 新候选超过旧峰值的替换倍率
#define SELECT_PEAK_DOMINANCE  1.3f     // 峰值主导判定倍率
// 同侧内外联合状态：共同位置可快动，视差必须连续确认后才允许形成斜率。
#define SELECT_PAIR_VERTICAL_GATE_MM  2.0f  // 小于此值按垂直模型处理
#define SELECT_PAIR_SLANT_ENTER_MM    3.5f  // 连续超过此值才进入斜入射模型
#define SELECT_PAIR_SLANT_EXIT_MM     1.5f  // 连续低于此值退出斜入射模型
#define SELECT_PAIR_D_CONFIRM_FRAMES  4     // 视差/角度变化连续确认帧数
#define SELECT_PAIR_D_EXIT_FRAMES     4
#define SELECT_PAIR_D_CONFIRM_GATE_MM 2.5f  // 连续视差的一致性门限
#define SELECT_PAIR_D_STEP_MM         5.0f  // 单帧允许的视差变化
#define SELECT_PAIR_D_INNOV_SCALE_MM  4.0f  // 单边跳动降权尺度
#define SELECT_PAIR_ACQUIRE_SCALE_MM  20.0f // 未确认斜率时只轻度降权
#define SELECT_PAIR_COMMON_SCALE_MM   60.0f // 共同运动预测权重尺度
#define SELECT_PAIR_Q_ALPHA           0.65f
#define SELECT_PAIR_Q_VEL_ALPHA       0.35f
#define SELECT_PAIR_D_MEAS_ALPHA      0.35f
#define SELECT_PAIR_D_STATE_ALPHA     0.25f
#define SELECT_PAIR_D_VEL_ALPHA       0.20f
#define SELECT_PAIR_D_ZERO_ALPHA      0.30f
#define SELECT_PAIR_PENDING_ALPHA     0.35f
#define SELECT_CONF_MIN        0.15f    // 输出最低置信度
#define SELECT_CONFIRM_FRAMES  2        // 有光/新交点需连续两帧一致
#define SELECT_HOLD_FRAMES     2        // 弱边偶发丢1帧时短保持；持续无光约0.14s后失效
#define SELECT_TEMPORAL_GATE_MM 30.0f   // 时域门限基值（mm）
#define SELECT_VEL_GATE_GAIN   2.0f     // 速度门限增益
#define SELECT_VEL_ALPHA       0.5f     // 速度平滑权重
#define SELECT_LINE_DIR_ALPHA      1.0f
#define SELECT_LINE_DIR_ALPHA_FAST 1.0f
#define SELECT_LINE_DIR_SPEED      25.0f
#define SELECT_OUTPUT_ALPHA_MIN 0.10f   // 输出最小平滑系数
#define SELECT_OUTPUT_SPEED_SCALE 80.0f // 输出速度尺度（mm/帧）
#define SELECT_INTERSECTION_MARGIN_MM 150.0f // UI允许显示屏外交点，同时拒绝无穷远伪交点
#define PRODUCT_SCREEN_MARGIN_MM      5.0f   // 产品坐标仅在内屏附近有效

// ==================== 垂直入射专用定位 ====================
// 当前产品阶段不估计斜率：上下四边映射到共同X，左右四边映射到共同Y。
// 4/3边优先，缺边时允许同侧内外2边输出；始终强制横竖方向，不用两点估计斜率。
#define VERTICAL_ONLY_MODE             1
#define VERTICAL_FUSE_MIN_POINTS       2u
#define VERTICAL_ALIGNMENT_TARGET_MM   1.00f
#define VERTICAL_FUSE_OUTLIER_MM       4.00f
#define VERTICAL_FUSE_HUBER_MM         0.50f
/* Four strips observe one physical motion in vertical-incidence mode.
   Follow the median per-frame motion immediately, while the independently
   calibrated absolute fusion only corrects slow drift.  This prevents one
   noisy strip from moving the common coordinate. */
#define VERTICAL_FUSE_TETHER_ALPHA      0.010f
#define VERTICAL_FUSE_TETHER_MAX_MM     0.50f
// 同步位移跟踪只用于压制亚毫米抖动，不能让直线长期停留在旧位置。
// 若预测线与当前有效质心鲁棒中心持续偏离，认为发生了掉线重获/真实跳变，
// 连续若干帧后重新锚定当前中心。
#define VERTICAL_FUSE_REANCHOR_MM        2.00f
#define VERTICAL_FUSE_REANCHOR_FRAMES    3u
#define VERTICAL_FUSE_REANCHOR_STABLE_MM 1.00f

// ==================== 多点几何标定 ====================
// UI只下发已知真实坐标；候选选择、中位数、单调检查、拟合与应用都在固件。
#define CALIB_CAPTURE_MIN_VALID_FRAMES 5
#define CALIB_SETTLE_FRAMES            8  // 移动到标定点后先等待滤波器稳定，约0.5s@14fps
// 21点标准子集：长边7个X节点覆盖双PCB拼接区；短边3个Y节点覆盖单PCB。
#define CALIB_MIN_POINTS_LONG         7
#define CALIB_MIN_POINTS_SHORT        3
#define CALIB_MIN_PEAK                0.020f
#define CALIB_TARGET_REPLACE_MM       0.5f
#define CALIB_MIN_RAW_SPACING         0.20f
// 临时调试：1=节点数完整时允许当前八边标定整体应用，便于先观察效果。
// RMS和单调性仍照常计算并显示，但不再阻止应用；正式使用前必须改回0。
#define CALIB_ACCEPT_CURRENT_FOR_TEST  0
#endif
