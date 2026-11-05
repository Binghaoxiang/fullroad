/**
  ******************************************************************************
  * @file    cmd.c
  * @brief   串口命令行协议
  *
  *  命令格式：<字母><参数>，以 CR / LF 结尾，大小写不敏感，前导空格忽略。
  *  例如：A90.5<CR><LF>      把舵机转到 90.5°
  *        L0,180<CR><LF>     设置软件行程 0°~180°
  *
  *  A<deg>       设定角度（可带 3 位小数，可为负）
  *  P<us>        直接给脉宽（1us 精度，用于示波器/标定验证，无斜率限制）
  *  C            回中位
  *  L<min>,<max> 设定角度限位（软件行程）
  *  W<min>,<max> 设定脉宽标定端点（角度 <-> 脉宽映射）
  *  T<deg>       零点修正（机械安装偏差补偿），只改输出脉宽
  *  S<deg/s>     最大角速度，0 = 不限速
  *  X<0|1>       关闭/打开脉冲输出
  *  Q 或 ?       查询状态
  *  R            恢复默认参数
  *  Z            往返扫掠自检
  *  V            版本
  *  H            帮助
  ******************************************************************************
  */
#include "cmd.h"
#include "uart.h"
#include "servo.h"
#include "board.h"
#include "sensor.h"
#include "track.h"
#include <string.h>

/* 相对角度移动：虚拟角度与每度转动时间 */
static int32_t  s_vang_mdeg  = 0;
static uint16_t s_ms_per_deg = 8U;   /* 每度转动毫秒数（30% 速度下标定） */
static void aa_step_deg(int32_t deg);   /* 前向声明 */

#define CMD_LINE_MAX      48U
#define SWEEP_SPEED_MDEG  180000UL      /* 自检扫掠速度 180°/s */
#define SWEEP_TIMEOUT_MS  6000UL
#define SWEEP_HOME_MDEG   90000L

static char    s_line[CMD_LINE_MAX];
static uint8_t s_len;
static uint8_t s_telem;                 /* 遥测开关 */
static uint32_t s_telem_ms;

/* 360 度舵机开环角度模式：按标定角速度换算转动时间 */
static uint8_t s_360mode;
static int32_t s_vangle;          /* 虚拟角度（毫度 0..359999） */
static int32_t s_degps360 = 360;  /* 满速角速度（度/秒） */

static void print_hex64(uint64_t v);    /* 前向声明 */

/*----------------------------------------------------------------------------
 * 输出辅助
 *--------------------------------------------------------------------------*/
static void reply_err(const char *why)
{
    uart_puts("ERR ");
    uart_puts(why);
    uart_puts("\r\n");
}

static void print_angle_pulse(int32_t mdeg, uint16_t pulse_us)
{
    uart_puts("A=");
    uart_put_mdeg(mdeg);
    uart_puts(" P=");
    uart_put_u32(pulse_us);
    uart_puts("\r\n");
}

static void print_state(void)
{
    servo_state_t st = servo_get_state();
    servo_cfg_t   cf = servo_get_cfg();

    uart_puts("A=");
    uart_put_mdeg(st.pos_mdeg);
    uart_puts(" P=");
    uart_put_u32(st.pulse_us);
    uart_puts(" TGT=");
    uart_put_mdeg(st.target_mdeg);
    uart_puts(" MOV=");
    uart_put_u32(st.moving);
    uart_puts(" EN=");
    uart_put_u32(st.enabled);
    uart_puts(" VMAX=");
    uart_put_deg1((int32_t)cf.speed_mdeg_s);
    uart_puts(" LIM=");
    uart_put_mdeg(cf.angle_min_mdeg);
    uart_puts(",");
    uart_put_mdeg(cf.angle_max_mdeg);
    uart_puts(" W=");
    uart_put_u32(cf.pulse_min_us);
    uart_puts(",");
    uart_put_u32(cf.pulse_max_us);
    uart_puts(" TRIM=");
    uart_put_mdeg(cf.trim_mdeg);
    uart_puts(" LVL=");
    uart_put_u32(servo_get_jog_level());
    uart_puts(" STEP=");
    uart_put_mdeg(servo_get_jog_step());
    uart_puts(" SRC=");
    uart_puts((board_is_hse_clock() != 0U) ? "PLL" : "HSI");
    uart_puts(" UP=");
    uart_put_u32(board_millis() / 1000UL);
    uart_puts("s\r\n");
}

static void print_help(void)
{
    uart_puts("--- COMMANDS (end with CR or LF, case insensitive) ---\r\n");
    uart_puts(" A<deg>        set angle          e.g. A90.5 / A-5\r\n");
    uart_puts(" +[n] / -[n]   jog n steps at current level (n default 1)\r\n");
    uart_puts(" N[1..4]       jog level 0.1 / 1 / 5 / 45 deg (N alone = show)\r\n");
    uart_puts(" K             tracking status; K ON|OFF|A|B|DARK|RAW|TEL 1|RESET\r\n");
    uart_puts(" K SIGN <1|-1> flip seek direction\r\n");
    uart_puts(" K MASKA/MASKB <hex64>  edge masks, bit = row*16+col\r\n");
    uart_puts(" K EN <0..15>    force EN1..EN4 levels (bring-up)\r\n");
    uart_puts(" K CURVE <s> <e> <step>  sweep and print ANG,S,P csv\r\n");
    uart_puts(" P<us>         set pulse directly e.g. P1500 (no ramp)\r\n");
    uart_puts(" C             center\r\n");
    uart_puts(" L<min>,<max>  angle limits       e.g. L0,180\r\n");
    uart_puts(" W<min>,<max>  pulse calibration  e.g. W600,2400\r\n");
    uart_puts(" T<deg>        trim offset        e.g. T-1.5\r\n");
    uart_puts(" S<deg/s>      max speed          e.g. S180 (0 = unlimited)\r\n");
    uart_puts(" X<0|1>        output off/on\r\n");
    uart_puts(" Q or ?        query state\r\n");
    uart_puts(" R             restore defaults\r\n");
    uart_puts(" Z             sweep self test\r\n");
    uart_puts(" V             version\r\n");
    uart_puts(" H             this help\r\n");
}

/*----------------------------------------------------------------------------
 * 数字解析：成功返回剩余的字符串指针，失败返回 NULL
 *--------------------------------------------------------------------------*/
static const char *parse_mdeg(const char *s, int32_t *out)
{
    int32_t sign   = 1;
    int32_t whole  = 0;
    int32_t frac   = 0;
    uint8_t digits = 0U;

    while (*s == ' ') {
        s++;
    }
    if (*s == '+') {
        s++;
    } else if (*s == '-') {
        sign = -1;
        s++;
    }

    while ((*s >= '0') && (*s <= '9')) {
        if (whole > 1000000) {
            return 0;                       /* 明显溢出，直接拒绝 */
        }
        whole = (whole * 10) + (int32_t)(*s - '0');
        digits++;
        s++;
    }

    if (*s == '.') {
        int32_t scale = 100;                /* 小数最多取 3 位 = 0.001° */
        s++;
        while ((*s >= '0') && (*s <= '9')) {
            if (scale > 0) {
                frac += (int32_t)(*s - '0') * scale;
                scale /= 10;
            }
            digits++;
            s++;
        }
    }

    if (digits == 0U) {
        return 0;
    }
    if (whole > 1000000) {
        return 0;
    }
    *out = sign * ((whole * 1000) + frac);
    return s;
}

static const char *parse_u32(const char *s, uint32_t *out)
{
    uint32_t value  = 0U;
    uint8_t  digits = 0U;

    while (*s == ' ') {
        s++;
    }
    while ((*s >= '0') && (*s <= '9')) {
        if (value > 100000000U) {
            return 0;
        }
        value = (value * 10U) + (uint32_t)(*s - '0');
        digits++;
        s++;
    }
    if (digits == 0U) {
        return 0;
    }
    *out = value;
    return s;
}

static uint8_t at_end(const char *s)
{
    while ((*s == ' ') || (*s == '\t')) {
        s++;
    }
    return (uint8_t)((*s == '\0') ? 1U : 0U);
}

/* 把 "1,2" 从逗号处劈成两段，返回第二段指针，没有逗号返回 NULL */
static char *split_comma(char *s)
{
    while (*s != '\0') {
        if (*s == ',') {
            *s = '\0';
            return s + 1;
        }
        s++;
    }
    return 0;
}

/*----------------------------------------------------------------------------
 * 各命令实现
 *--------------------------------------------------------------------------*/
static void do_angle(const char *arg)
{
    const char *rest;
    int32_t     deg;

    rest = parse_mdeg(arg, &deg);
    if ((rest == 0) || (at_end(rest) == 0U)) {
        reply_err("SYNTAX");
        return;
    }
    if ((deg < servo_angle_min()) || (deg > servo_angle_max())) {
        uart_puts("ERR RANGE ");
        uart_put_mdeg(servo_angle_min());
        uart_puts("..");
        uart_put_mdeg(servo_angle_max());
        uart_puts("\r\n");
        return;
    }
    /* 360 舵机：开环角度模式（按时间换算），支持 0~360 循环 */
    if (s_360mode != 0U) {
        int32_t target = deg % 360000;
        int32_t delta;
        int32_t mag;
        int32_t ms;

        if (target < 0) { target += 360000; }
        delta = target - s_vangle;
        if (delta >  180000) { delta -= 360000; }
        if (delta < -180000) { delta += 360000; }
        mag = (delta < 0) ? -delta : delta;
        if (mag > 0) {
            ms = (mag / 10) * 10 / ((s_degps360 > 0) ? s_degps360 : 1);
            if (ms < 30) { ms = 30; }
            servo_spin((delta > 0) ? 1000 : -1000, (uint32_t)ms);
        }
        s_vangle = target;
        uart_puts("A360=");
        uart_put_i32(target / 1000);
        uart_puts("d=");
        uart_put_i32(mag / 1000);
        uart_puts("deg ms=");
        uart_put_u32((uint32_t)((mag > 0) ? ((mag / 10) * 10 / ((s_degps360 > 0) ? s_degps360 : 1)) : 0));
        uart_puts("\r\n");
        return;
    }
    {
        /* 连续旋转舵机无位置反馈：绝对角度 -> 从当前虚拟位置相对转动 + 定时停止 */
        int32_t delta = deg - s_vangle;                 /* mdeg */
        int32_t mag   = (delta < 0) ? -delta : delta;   /* mdeg */
        int32_t ms    = 0;

        if (mag > 0) {
            ms = (mag / 1000) * (int32_t)s_ms_per_deg;
            if (ms < 10) { ms = 10; }
            if (ms > 8000) { ms = 8000; }
            servo_set_enabled(1U);
            servo_spin((delta > 0) ? 600 : -600, (uint32_t)ms);
            s_vangle = deg;
        }

        uart_puts("A=");
        uart_put_i32(deg / 1000);
        uart_puts(" deg  spin=");
        uart_put_u32((uint32_t)ms);
        uart_puts("ms\r\n");
        return;
    }
}

static void do_pulse(const char *arg)
{
    const char    *rest;
    uint32_t       us;
    servo_state_t  st;

    rest = parse_u32(arg, &us);
    if ((rest == 0) || (at_end(rest) == 0U)) {
        reply_err("SYNTAX");
        return;
    }
    if ((us < SERVO_PULSE_ABS_MIN_US) || (us > SERVO_PULSE_ABS_MAX_US)) {
        uart_puts("ERR RANGE ");
        uart_put_u32(SERVO_PULSE_ABS_MIN_US);
        uart_puts("..");
        uart_put_u32(SERVO_PULSE_ABS_MAX_US);
        uart_puts("\r\n");
        return;
    }
    servo_set_pulse((uint16_t)us);
    st = servo_get_state();
    print_angle_pulse(st.pos_mdeg, st.pulse_us);
}

static void do_limits(char *arg)
{
    char       *second;
    const char *rest;
    int32_t     lo, hi;

    second = split_comma(arg);
    if (second == 0) {
        reply_err("SYNTAX");
        return;
    }
    rest = parse_mdeg(arg, &lo);
    if ((rest == 0) || (at_end(rest) == 0U)) {
        reply_err("SYNTAX");
        return;
    }
    rest = parse_mdeg(second, &hi);
    if ((rest == 0) || (at_end(rest) == 0U)) {
        reply_err("SYNTAX");
        return;
    }
    if (servo_set_angle_range(lo, hi) == 0U) {
        reply_err("RANGE min<max, span>=1deg, |angle|<=360deg");
        return;
    }
    uart_puts("LIM=");
    uart_put_mdeg(servo_angle_min());
    uart_puts(",");
    uart_put_mdeg(servo_angle_max());
    uart_puts("\r\n");
}

static void do_calib(char *arg)
{
    char       *second;
    const char *rest;
    uint32_t    min_us, max_us;

    second = split_comma(arg);
    if (second == 0) {
        reply_err("SYNTAX");
        return;
    }
    rest = parse_u32(arg, &min_us);
    if ((rest == 0) || (at_end(rest) == 0U)) {
        reply_err("SYNTAX");
        return;
    }
    rest = parse_u32(second, &max_us);
    if ((rest == 0) || (at_end(rest) == 0U)) {
        reply_err("SYNTAX");
        return;
    }
    if (servo_set_pulse_range((uint16_t)min_us, (uint16_t)max_us) == 0U) {
        reply_err("RANGE 200..3000us, span>=100us");
        return;
    }
    uart_puts("W=");
    uart_put_u32(min_us);
    uart_puts(",");
    uart_put_u32(max_us);
    uart_puts("\r\n");
}

static void do_trim(const char *arg)
{
    const char *rest;
    int32_t     deg;
    servo_state_t st;

    rest = parse_mdeg(arg, &deg);
    if ((rest == 0) || (at_end(rest) == 0U)) {
        reply_err("SYNTAX");
        return;
    }
    servo_set_trim(deg);
    st = servo_get_state();
    uart_puts("TRIM=");
    uart_put_mdeg(servo_get_trim());
    uart_puts(" P=");
    uart_put_u32(st.pulse_us);
    uart_puts("\r\n");
}

static void do_speed(const char *arg)
{
    const char *rest;
    int32_t     deg_s;

    rest = parse_mdeg(arg, &deg_s);          /* 参数以 度/秒 为单位，可带小数 */
    if ((rest == 0) || (at_end(rest) == 0U)) {
        reply_err("SYNTAX");
        return;
    }
    if (deg_s < 0) {
        reply_err("RANGE");
        return;
    }
    servo_set_speed((uint32_t)deg_s);        /* 0.001°/s * 1000 = mdeg/s */
    uart_puts("VMAX=");
    uart_put_deg1((int32_t)servo_get_speed());
    uart_puts("\r\n");
}

static void do_enable(const char *arg)
{
    const char *rest;
    uint32_t    en;

    rest = parse_u32(arg, &en);
    if ((rest == 0) || (at_end(rest) == 0U) || (en > 1U)) {
        reply_err("SYNTAX");
        return;
    }
    servo_set_enabled((uint8_t)en);
    uart_puts("EN=");
    uart_put_u32(en);
    uart_puts("\r\n");
}

static void do_center(void)
{
    int32_t mid = (servo_angle_min() + servo_angle_max()) / 2;

    servo_set_angle(mid);
    print_angle_pulse(mid, servo_angle_to_pulse(mid));
}

static void do_version(void)
{
    uart_puts("SEVERO SERVO CONTROLLER v" FW_VERSION "\r\n");
    uart_puts("BUILD " __DATE__ " " __TIME__ "\r\n");
    uart_puts("SYSCLK=");
    uart_put_u32(SystemCoreClock / 1000000UL);
    uart_puts("MHz SRC=");
    uart_puts((board_is_hse_clock() != 0U) ? "HSE+PLL" : "HSI+PLL");
    uart_puts(" PWM=50Hz 0.5us, UART=");
    uart_put_u32(BOARD_UART_BAUD);
    uart_puts(" 8N1\r\n");
}

static void do_sweep(void)
{
    int32_t  lo = servo_angle_min();
    int32_t  hi = servo_angle_max();
    uint32_t old_speed = servo_get_speed();

    uart_puts("SWEEP START\r\n");
    servo_set_speed(SWEEP_SPEED_MDEG);

    servo_set_angle(lo);
    servo_wait_done(SWEEP_TIMEOUT_MS);
    uart_puts(" MIN REACHED P=");
    uart_put_u32(servo_get_state().pulse_us);
    uart_puts("\r\n");

    servo_set_angle(hi);
    servo_wait_done(SWEEP_TIMEOUT_MS);
    uart_puts(" MAX REACHED P=");
    uart_put_u32(servo_get_state().pulse_us);
    uart_puts("\r\n");

    servo_set_angle((lo + hi) / 2);
    servo_wait_done(SWEEP_TIMEOUT_MS);

    servo_set_speed(old_speed);
    uart_puts("OK SWEEP\r\n");
}

/*----------------------------------------------------------------------------
 * 分档位点动：+ / - 按当前档位走一步，可带步数
 *--------------------------------------------------------------------------*/
static void print_jog(void)
{
    servo_state_t st = servo_get_state();

    uart_puts("A=");
    uart_put_mdeg(st.target_mdeg);
    uart_puts(" LVL=");
    uart_put_u32(servo_get_jog_level());
    uart_puts(" STEP=");
    uart_put_mdeg(servo_get_jog_step());
    uart_puts(" P=");
    uart_put_u32(servo_angle_to_pulse(st.target_mdeg));
    uart_puts("\r\n");
}

static void do_jog(const char *arg, int32_t sign)
{
    const char *rest;
    uint32_t    n = 1U;
    int32_t     before;
    int32_t     after;

    if (*arg != '\0') {
        rest = parse_u32(arg, &n);
        if ((rest == 0) || (at_end(rest) == 0U) || (n == 0U) || (n > 1000U)) {
            reply_err("SYNTAX [+n], n=1..1000");
            return;
        }
    }
    before = servo_get_state().target_mdeg;
    after  = servo_jog(sign * (int32_t)n);
    print_jog();
    if (after == before) {
        uart_puts("AT LIMIT\r\n");      /* 已经在行程端点上，走不动了 */
    }
}

static void do_level(const char *arg)
{
    const char *rest;
    uint32_t    lv;

    if (*arg == '\0') {
        print_jog();
        return;
    }
    rest = parse_u32(arg, &lv);
    if (rest == 0) {
        reply_err("SYNTAX");
        return;
    }
    if (at_end(rest) == 0U) {
        reply_err("SYNTAX");
        return;
    }
    if (servo_set_jog_level((uint8_t)lv) == 0U) {
        uart_puts("ERR RANGE 1..");
        uart_put_u32(SERVO_JOG_LEVELS);
        uart_puts("\r\n");
        return;
    }
    print_jog();
}

/*----------------------------------------------------------------------------
 * 自动对准相关命令（统一用 K 字母，子命令区分）
 *   K              查询跟踪状态
 *   K ON / K OFF   开/关自动跟踪
 *   K A / K B      模式 A(差分锁零) / B(强度爬坡)
 *   K SIGN n       方向符号 ±1（发现越追越远就翻一下）
 *   K DARK         遮光状态下标定暗电流
 *   K RAW          打印各通道原始值
 *   K TEL 0/1      遥测输出开关
 *   K CURVE s e st 自动扫掠出 θ-S 曲线，例如 K CURVE -5 5 0.5
 *   K RESET        复位状态机
 *--------------------------------------------------------------------------*/
static char *next_word(char *s, char *tok, uint8_t toksz)
{
    uint8_t n = 0U;

    while (*s == ' ') {
        s++;
    }
    while ((*s != '\0') && (*s != ' ') && (n < (uint8_t)(toksz - 1U))) {
        tok[n] = *s;
        n++;
        s++;
    }
    tok[n] = '\0';
    while (*s == ' ') {
        s++;
    }
    return s;
}

static uint8_t eqi(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0')) {
        char ca = ((*a >= 'a') && (*a <= 'z')) ? (char)(*a - 32) : *a;
        char cb = ((*b >= 'a') && (*b <= 'z')) ? (char)(*b - 32) : *b;
        if (ca != cb) {
            return 0U;
        }
        a++;
        b++;
    }
    return (uint8_t)(((*a == '\0') && (*b == '\0')) ? 1U : 0U);
}

static void print_trk_status(void)
{
    const sensor_meas_t *m = track_last();

    uart_puts("TRK=");
    uart_puts((track_enabled() != 0U) ? "ON" : "OFF");
    uart_puts(" MODE=");
    uart_puts((track_mode() == 1U) ? "A" : "B");
    uart_puts(" SIGN=");
    uart_put_i32((int32_t)track_sign());
    uart_puts(" ST=");
    uart_puts(track_state_name());
    uart_puts(" S=");
    uart_put_i32(m->s_permille);
    uart_puts(" P=");
    uart_put_u32(m->total);
    uart_puts(" ANG=");
    uart_put_mdeg(servo_get_state().target_mdeg);
    uart_puts(" LOCK=");
    uart_put_u32(track_lock_ms());
    uart_puts("ms MA=");
    print_hex64(adc64_mask(0U));
    uart_puts(" MB=");
    print_hex64(adc64_mask(1U));
    uart_puts("\r\n");
}

static void print_raw(void)
{
    sensor_meas_t m;
    uint8_t b;
    uint8_t r;
    uint8_t c;

    sensor_measure(&m);

    for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
        uart_puts((b == 0U) ? "# A long  RC" : "# B short RC");
        for (c = 0U; c < ADC64_COLS; c++) {
            uart_putc(' ');
            uart_put_u32((uint32_t)c);
        }
        uart_puts("\r\n");
        for (r = 0U; r < ADC64_ROWS; r++) {
            uart_putc('R');
            uart_put_u32((uint32_t)r + 1U);
            for (c = 0U; c < ADC64_COLS; c++) {
                uart_putc(' ');
                uart_put_u32((uint32_t)m.cell[b][r][c]);
            }
            uart_puts("\r\n");
        }
    }
    uart_puts("A=");
    uart_put_u32(m.sum_a);
    uart_puts(" B=");
    uart_put_u32(m.sum_b);
    uart_puts(" S=");
    uart_put_i32(m.s_permille);
    uart_puts(" P=");
    uart_put_u32(m.total);
    uart_puts("\r\n");
}

/* 纯十进制整数解析（不乘 1000） */
static const char *parse_int_plain(const char *s, int32_t *out)
{
    int32_t v = 0;
    uint8_t neg = 0U;

    while (*s == ' ') { s++; }
    if (*s == '-') { neg = 1U; s++; }
    else if (*s == '+') { s++; }
    if ((*s < '0') || (*s > '9')) { return 0; }
    while ((*s >= '0') && (*s <= '9')) {
        v = v * 10 + (int32_t)(*s - '0');
        s++;
    }
    *out = (neg != 0U) ? -v : v;
    return s;
}

/* 十六进制 64 位解析，允许 0x 前缀 */
static uint8_t parse_hex64(const char *s, uint64_t *out)
{
    uint64_t v = 0U;
    uint8_t  n = 0U;

    while (*s == ' ') {
        s++;
    }
    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) {
        s += 2;
    }
    while (*s != '\0') {
        char    c = *s;
        uint8_t d;

        if ((c >= '0') && (c <= '9')) {
            d = (uint8_t)(c - '0');
        } else if ((c >= 'a') && (c <= 'f')) {
            d = (uint8_t)(c - 'a' + 10);
        } else if ((c >= 'A') && (c <= 'F')) {
            d = (uint8_t)(c - 'A' + 10);
        } else {
            break;
        }
        v = (v << 4) | (uint64_t)d;
        n++;
        s++;
        if (n > 16U) {
            return 0U;
        }
    }
    if (n == 0U) {
        return 0U;
    }
    *out = v;
    return 1U;
}

static void print_hex64(uint64_t v)
{
    char    buf[17];
    uint8_t i = 16U;
    uint8_t d;

    do {
        d = (uint8_t)(v & 0x0FU);
        i--;
        buf[i] = (char)((d < 10U) ? ('0' + d) : ('A' + d - 10U));
        v >>= 4;
    } while ((v != 0U) && (i > 0U));
    while (i > 0U) {
        i--;
        buf[i] = '0';
    }
    for (i = 0U; i < 16U; i++) {
        uart_putc(buf[i]);
    }
}

/* K CURVE <起始角度> <结束角度> <步长>，自动扫掠并输出 CSV */
static void do_curve(char *arg)
{
    const char *r;
    int32_t a0 = 0;
    int32_t a1 = 0;
    int32_t st = 0;
    int32_t a;
    uint32_t n = 0U;
    sensor_meas_t m;

    r = parse_mdeg(arg, &a0);
    if (r == 0) {
        reply_err("SYNTAX");
        return;
    }
    while ((*r == ',') || (*r == ' ')) {
        r++;
    }
    r = parse_mdeg(r, &a1);
    if (r == 0) {
        reply_err("SYNTAX");
        return;
    }
    while ((*r == ',') || (*r == ' ')) {
        r++;
    }
    r = parse_mdeg(r, &st);
    if (r == 0) {
        reply_err("SYNTAX");
        return;
    }
    if ((st <= 0) || (st > 10000) || (a0 >= a1)) {
        reply_err("RANGE need start<end, 0<step<=10deg");
        return;
    }
    if ((a0 < servo_angle_min()) || (a1 > servo_angle_max())) {
        reply_err("RANGE out of LIM");
        return;
    }

    uart_puts("# CURVE ");
    uart_put_mdeg(a0);
    uart_putc(' ');
    uart_put_mdeg(a1);
    uart_putc(' ');
    uart_put_mdeg(st);
    uart_puts("\r\n# ANG,S,P\r\n");

    for (a = a0; (a <= a1) && (n < 200U); a += st) {
        servo_set_angle(a);
        servo_wait_done(4000U);
        board_delay_ms(TRK_SETTLE_MS);
        sensor_measure(&m);
        uart_put_mdeg(a);
        uart_putc(',');
        uart_put_i32(m.s_permille);
        uart_putc(',');
        uart_put_u32(m.total);
        uart_puts("\r\n");
        n++;
    }
    servo_set_angle(a0);
    uart_puts("OK CURVE\r\n");
}

static void do_track(char *arg)
{
    char  tok[10];
    char *rest;

    rest = next_word(arg, tok, (uint8_t)sizeof(tok));

    if ((tok[0] == '\0') || (tok[0] == '?')) {
        print_trk_status();
        return;
    }
    if (eqi(tok, "ON") != 0U) {
        track_enable(1U);
        uart_puts("TRK ON\r\n");
        return;
    }
    if (eqi(tok, "OFF") != 0U) {
        track_enable(0U);
        uart_puts("TRK OFF\r\n");
        return;
    }
    if (eqi(tok, "A") != 0U) {
        track_set_mode(1U);
        uart_puts("MODE A (diff)\r\n");
        return;
    }
    if (eqi(tok, "B") != 0U) {
        track_set_mode(2U);
        uart_puts("MODE B (peak)\r\n");
        return;
    }
    if (eqi(tok, "SIGN") != 0U) {
        const char *r2;
        int32_t v = 0;

        r2 = parse_mdeg(rest, &v);
        if (r2 == 0) {
            reply_err("SYNTAX");
            return;
        }
        track_set_sign((int8_t)((v < 0) ? -1 : 1));
        uart_puts("SIGN=");
        uart_put_i32((int32_t)track_sign());
        uart_puts("\r\n");
        return;
    }
    if (eqi(tok, "DARK") != 0U) {
        sensor_cal_dark();
        uart_puts("OK DARK\r\n");
        return;
    }
    if (eqi(tok, "RAW") != 0U) {
        print_raw();
        return;
    }
    if (eqi(tok, "TEL") != 0U) {
        const char *r2;
        int32_t v = 0;

        r2 = parse_mdeg(rest, &v);
        if (r2 == 0) {
            reply_err("SYNTAX");
            return;
        }
        s_telem = (v != 0) ? 1U : 0U;
        uart_puts("TEL=");
        uart_put_u32(s_telem);
        uart_puts("\r\n");
        return;
    }
    if (eqi(tok, "CURVE") != 0U) {
        do_curve(rest);
        return;
    }
    if (eqi(tok, "EN") != 0U) {
        const char *r2;
        int32_t v = 0;

        r2 = parse_int_plain(rest, &v);
        if ((r2 == 0) || (v < 0) || (v > 15)) {
            reply_err("SYNTAX K EN <0..15>");
            return;
        }
        adc64_set_en_mask((uint8_t)v);
        uart_puts("EN=");
        uart_put_u32((uint32_t)v);
        uart_puts("\r\n");
        return;
    }
    if ((eqi(tok, "MASKA") != 0U) || (eqi(tok, "MASKB") != 0U)) {
        uint64_t v = 0U;

        if (parse_hex64(rest, &v) == 0U) {
            reply_err("SYNTAX MASKA <hex64>");
            return;
        }
        if (eqi(tok, "MASKA") != 0U) {
            adc64_set_mask(0U, v);
            uart_puts("MASK_A(long)=");
        } else {
            adc64_set_mask(1U, v);
            uart_puts("MASK_B(short)=");
        }
        print_hex64(v);
        uart_puts("\r\n");
        return;
    }
    if (eqi(tok, "SC") != 0U) {
        const char *r2;
        int32_t sp = 0;
        int32_t ms = 0;

        r2 = parse_int_plain(rest, &sp);
        if (r2 == 0) { reply_err("SYNTAX K SC <permille> [ms]"); return; }
        (void)parse_int_plain(r2, &ms);
        servo_spin(sp, (ms > 0) ? (uint32_t)ms : 0U);
        uart_puts("SPIN=");
        uart_put_i32(sp);
        uart_puts("ms=");
        uart_put_u32((uint32_t)((ms > 0) ? ms : 0));
        uart_puts("\r\n");
        return;
    }
    if (eqi(tok, "SCN") != 0U) {
        const char *r2;
        int32_t us = 0;
        r2 = parse_int_plain(rest, &us);
        if ((r2 == 0) || (us < 500) || (us > 2500)) { reply_err("SYNTAX K SCN <us>"); return; }
        servo_spin_neutral((uint16_t)us);
        uart_puts("NEUTRAL=");
        uart_put_u32((uint32_t)us);
        uart_puts("\r\n");
        return;
    }    if ((eqi(tok, "KP") != 0U) || (eqi(tok, "KI") != 0U) || (eqi(tok, "KD") != 0U)) {
        const char *r2;
        int32_t v = 0;
        r2 = parse_int_plain(rest, &v);
        if (r2 == 0) { reply_err("SYNTAX K KP|KI|KD <n>"); return; }
        if (eqi(tok, "KP") != 0U)      { track_set_pid(v, track_get_ki(), track_get_kd()); }
        else if (eqi(tok, "KI") != 0U) { track_set_pid(track_get_kp(), v, track_get_kd()); }
        else                           { track_set_pid(track_get_kp(), track_get_ki(), v); }
        uart_puts("PID=");
        uart_put_i32(track_get_kp()); uart_putc(',');
        uart_put_i32(track_get_ki()); uart_putc(',');
        uart_put_i32(track_get_kd());
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "360") != 0U) {
        const char *r2;
        int32_t v = 1;
        r2 = parse_int_plain(rest, &v);
        s_360mode = (v != 0) ? 1U : 0U;
        uart_puts("MODE360=");
        uart_put_u32((uint32_t)s_360mode);
        uart_puts(" DEGPS=");
        uart_put_i32(s_degps360);
        uart_puts("\r\n");
        return;
    }
    if (eqi(tok, "DEGPS") != 0U) {
        const char *r2;
        int32_t v = 0;
        r2 = parse_int_plain(rest, &v);
        if ((r2 == 0) || (v < 10) || (v > 3000)) { reply_err("SYNTAX K DEGPS <deg/s>"); return; }
        s_degps360 = v;
        uart_puts("DEGPS=");
        uart_put_i32(v);
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "ECHO") != 0U) {
        uint16_t raw[ADC64_COLS];
        uint8_t c;
        adc64_echo_test(0U, raw);
        uart_puts("ECHO r2: col:word/echo/data\r\n");
        for (c = 0U; c < ADC64_COLS; c++) {
            uart_puts("  ");
            uart_put_u32((uint32_t)c); uart_putc(':');
            uart_put_u32((uint32_t)raw[c]); uart_putc('/');
            uart_put_u32((uint32_t)(raw[c] >> 12)); uart_putc('/');
            uart_put_u32((uint32_t)(raw[c] & 0x0FFFU));
            uart_puts("\r\n");
        }
        return;
    }    if (eqi(tok, "SPI") != 0U) {
        const char *r2;
        int32_t v = 0;
        r2 = parse_int_plain(rest, &v);
        if ((r2 == 0) || (v < 0) || (v > 3)) { reply_err("SYNTAX K SPI <0..3>"); return; }
        board_spi_set_mode((uint8_t)v);
        uart_puts("SPI MODE=");
        uart_put_u32((uint32_t)v);
        uart_puts("\r\n");
        return;
    }
    if (eqi(tok, "DELAY") != 0U) {
        const char *r2;
        int32_t v = 0;
        r2 = parse_int_plain(rest, &v);
        if ((r2 == 0) || (v < 0) || (v > 200)) { reply_err("SYNTAX K DELAY <0..200us>"); return; }
        adc64_set_acc_us((uint16_t)v);
        uart_puts("ACC=");
        uart_put_u32((uint32_t)adc64_get_acc_us());
        uart_puts("us\r\n");
        return;
    }    if (eqi(tok, "RAW1") != 0U) {
        adc64_frame_t fr;
        uint8_t i;
        uint8_t c;
        uint8_t r;

        adc64_scan(&fr);
        /* 完全按 PCB 丝印物理顺序：列 0→F，每列行 1,2,3,4
           → D10 D20 D30 D40 D11 D21 D31 D41 … D1F D2F D3F D4F */
        uart_puts("LONG64");
        for (i = 0U; i < 64U; i++) {
            c = (uint8_t)(15U - (i / 4U));      /* 列 F→0 */
            r = (uint8_t)(3U - (i % 4U));       /* 行 4,3,2,1 */
            uart_putc(' ');
            uart_put_u32((uint32_t)fr.cell[0][r][c]);
        }
        uart_puts("\r\nSILK  ");
        for (i = 0U; i < 64U; i++) {
            uint8_t hv;
            c = (uint8_t)(15U - (i / 4U));
            hv = (uint8_t)(c);
            uart_putc(' ');
            uart_putc('D');
            uart_putc((char)('1' + (3U - (i % 4U))));
            uart_putc((hv < 10U) ? (char)('0' + hv) : (char)('A' + hv - 10U));
        }
        uart_puts("\r\nSUM=");
        uart_put_u32(fr.sum[0]);
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "SHIFT") != 0U) {
        const char *r2;
        int32_t v = 0;
        r2 = parse_int_plain(rest, &v);
        if ((r2 == 0) || (v < 0) || (v > 15)) { reply_err("SYNTAX K SHIFT <0..15>"); return; }
        adc64_set_shift((uint8_t)v);
        uart_puts("SHIFT=");
        uart_put_u32((uint32_t)adc64_get_shift());
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "ENON") != 0U) {
        const char *r2;
        int32_t v = 0;
        r2 = parse_int_plain(rest, &v);
        if ((r2 == 0) || (v < 0) || (v > 15)) { reply_err("SYNTAX K ENON <0..15>"); return; }
        adc64_set_en_hold((uint8_t)v);
        uart_puts("ENHOLD=");
        uart_put_u32((uint32_t)adc64_get_en_hold());
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "ONE") != 0U) {
        const char *r2;
        int32_t v = 0;
        uint16_t vals[ADC64_ROWS];
        uint8_t k;
        r2 = parse_int_plain(rest, &v);
        if ((r2 == 0) || (v < 0) || (v > 15)) { reply_err("SYNTAX K ONE <0..15>"); return; }
        adc64_read_column((uint8_t)v, vals);
        uart_puts("ONE CH");
        uart_put_u32((uint32_t)v);
        uart_puts(" :");
        for (k = 0U; k < ADC64_ROWS; k++) {
            uart_putc(' ');
            uart_put_u32((uint32_t)vals[k]);
        }
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "ALL") != 0U) {
        uint8_t c;
        uint8_t k;
        uint16_t vals[ADC64_ROWS];

        uart_puts("CH : r4 r3 r2 r1\r\n");
        for (c = 0U; c < ADC64_COLS; c++) {
            adc64_read_column(c, vals);
            uart_puts("CH");
            if (c < 10U) { uart_putc((char)('0' + c)); }
            else { uart_putc('1'); uart_putc((char)('0' + c - 10U)); }
            uart_puts(" :");
            for (k = ADC64_ROWS; k > 0U; k--) {
                uart_putc(' ');
                uart_put_u32((uint32_t)vals[k - 1U]);
            }
            uart_puts("\r\n");
        }
        return;
    }    if (eqi(tok, "FOUR") != 0U) {
        uint8_t c;
        uint8_t k;
        uint16_t vals[ADC64_ROWS];
        uint16_t best[ADC64_ROWS];
        uint32_t bestsum = 0U;
        uint8_t  bestch = 0U;

        adc64_set_en_hold(0U);          /* 强制逐行扫描 */

        for (c = 0U; c < ADC64_COLS; c++) {
            uint32_t s = 0U;
            adc64_read_column(c, vals);
            for (k = 0U; k < ADC64_ROWS; k++) { s += (uint32_t)vals[k]; }
            if (s > bestsum) {
                bestsum = s;
                bestch = c;
                for (k = 0U; k < ADC64_ROWS; k++) { best[k] = vals[k]; }
            }
        }
        uart_puts("BEST CH");
        uart_put_u32((uint32_t)bestch);
        uart_puts("  SUM=");
        uart_put_u32(bestsum);
        uart_puts("  r4 r3 r2 r1 =");
        for (k = ADC64_ROWS; k > 0U; k--) {
            uart_putc(' ');
            uart_put_u32((uint32_t)best[k - 1U]);
        }
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "LEFT") != 0U) {
        uint16_t v0[ADC64_ROWS];
        uint16_t v15[ADC64_ROWS];
        uint8_t k;

        adc64_set_en_hold(0U);

        /* 只扫两列：CH0 和 CH15（列 F 的两种可能映射），各 4 行 */
        adc64_read_column(0U, v0);
        adc64_read_column(15U, v15);

        uart_puts("CH0  :");
        for (k = ADC64_ROWS; k > 0U; k--) {
            uart_putc(' '); uart_put_u32((uint32_t)v0[k - 1U]);
        }
        uart_puts("\r\nCH15 :");
        for (k = ADC64_ROWS; k > 0U; k--) {
            uart_putc(' '); uart_put_u32((uint32_t)v15[k - 1U]);
        }
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "CUT") != 0U) {
        adc64_frame_t fr;
        uint8_t col;
        uint8_t rr;
        uint8_t n = 0U;

        adc64_set_en_hold(0U);
        adc64_scan(&fr);

        uart_puts("CUT43");
        for (col = 15U; col >= 5U; col--) {
            for (rr = 3U; rr < 4U; rr--) {          /* 行 4,3,2,1 */
                if ((col == 5U) && (rr == 0U)) { break; }   /* 列5只到 D25(行2) */
                uart_putc(' ');
                uart_put_u32((uint32_t)fr.cell[0][rr][col]);
                n++;
            }
            if (col == 0U) { break; }
        }
        uart_puts("\r\nSILK ");
        for (col = 15U; col >= 5U; col--) {
            uint8_t hv = col;
            for (rr = 3U; rr < 4U; rr--) {
                if ((col == 5U) && (rr == 0U)) { break; }
                uart_putc(' ');
                uart_putc('D');
                uart_putc((char)('1' + rr));
                uart_putc((hv < 10U) ? (char)('0' + hv) : (char)('A' + hv - 10U));
            }
            if (col == 0U) { break; }
        }
        uart_puts("\r\nN=");
        uart_put_u32((uint32_t)n);
        uart_puts("\r\n");
        return;
    }    if ((eqi(tok, "LONG") != 0U) || (eqi(tok, "SHORT") != 0U)) {
        adc64_frame_t fr;
        uint16_t i;
        uint16_t i0;
        uint16_t i1;

        adc64_set_en_hold(0U);
        adc64_scan(&fr);

        /* 按 PCB X 坐标升序的索引：位号 = D{(i%4)+1}{hex(i/4)} */
        if (eqi(tok, "LONG") != 0U) {
            i0 = 21U;  i1 = 63U;              /* D25 -> D4F，43 颗 */
            uart_puts("LONG43");
        } else {
            i0 = 22U;  i1 = 39U;              /* 短边：板 B，D35 -> D49，18 颗 */
            uart_puts("SHORT18");
        }

        {
            uint8_t bd = (i0 == 22U) ? 1U : 0U;   /* SHORT 读板 B(FPC16)，LONG 读板 A(FPC12) */
            for (i = i0; i <= i1; i++) {
                uint8_t col = (uint8_t)(i / 4U);
                uint8_t rr  = (uint8_t)(i % 4U);
                uart_putc(' ');
                uart_put_u32((uint32_t)fr.cell[bd][rr][col]);
            }
        }
        uart_puts("\r\nSILK  ");
        {
            /* 短边标签按丝印 D35..D49；长边按 PCB 索引 */
            /* 从下到上：D35 -> D49 */
            static const char *k_short_lbl[18] = {
                "D35","D45","D16","D26","D36","D46","D17","D27","D37",
                "D47","D18","D28","D38","D48","D19","D29","D39","D49"
            };
            uint16_t n = 0U;
            for (i = i0; i <= i1; i++) {
                uint8_t col = (uint8_t)(i / 4U);
                uint8_t rr  = (uint8_t)(i % 4U);
                uart_putc(' ');
                if (i0 == 46U) {
                    uart_puts(k_short_lbl[n]);
                } else {
                    uart_putc('D');
                    uart_putc((char)('1' + rr));
                    uart_putc((col < 10U) ? (char)('0' + col) : (char)('A' + col - 10U));
                }
                n++;
            }
        }
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "SPREAD") != 0U) {
        adc64_frame_t fr;
        uint16_t i;
        uint32_t mx = 0U;
        uint32_t sum = 0U;
        uint32_t wsum = 0U;
        uint32_t sw = 0U;
        uint32_t mean;
        uint32_t var;
        uint16_t peak_i = 0U;
        uint32_t v;

        adc64_set_en_hold(0U);
        adc64_scan(&fr);

        /* 只统计长边 43 颗（索引 21~63） */
        for (i = 21U; i < 64U; i++) {
            uint8_t col = (uint8_t)(i / 4U);
            uint8_t rr  = (uint8_t)(i % 4U);
            uint16_t pos = (uint16_t)(i - 21U + 1U);      /* 1~43 */
            v = (uint32_t)fr.cell[0][rr][col];
            sum += v;
            sw  += v * (uint32_t)pos;
            if (v > mx) { mx = v; peak_i = pos; }
        }
        if (sum == 0U) {
            uart_puts("LONG MAX=0 SUM=0\r\n");
            return;
        }
        mean = sw / sum;

        /* 二阶矩（宽度） */
        for (i = 21U; i < 64U; i++) {
            uint8_t col = (uint8_t)(i / 4U);
            uint8_t rr  = (uint8_t)(i % 4U);
            uint16_t pos = (uint16_t)(i - 21U + 1U);
            int32_t d;
            v = (uint32_t)fr.cell[0][rr][col];
            d = (int32_t)pos - (int32_t)mean;
            wsum += v * (uint32_t)(d * d);
        }
        var = wsum / sum;

        uart_puts("LONG MAX=");
        uart_put_u32(mx);
        uart_puts(" SUM=");
        uart_put_u32(sum);
        uart_puts(" PEAK@");
        uart_put_u32((uint32_t)peak_i);
        uart_puts(" MEAN=");
        uart_put_u32(mean);
        uart_puts(" VAR=");
        uart_put_u32(var);
        uart_puts(" CONC=");
        uart_put_u32((mx * 100U) / sum);
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "LOCK") != 0U) {
        uint8_t  it;
        int16_t  dir = 1;
        int32_t  delta = 5000;           /* 5 度起步 */
        int32_t  ang = 90000;            /* 90 度 */
        uint32_t best;
        uint32_t cur;

        servo_set_enabled(1U);
        servo_set_speed(60000U);         /* 60 度/秒，避免太慢 */
        servo_set_angle(ang);
        servo_wait_done(3000U);
        board_delay_us(300000U);

        best = adc64_long_max_fast();
        uart_puts("LOCK start MAX=");
        uart_put_u32(best);
        uart_puts(" ang=90\r\n");

        for (it = 0U; it < 40U; it++) {
            ang += (int32_t)dir * delta;
            if (ang < servo_angle_min()) { ang = servo_angle_min(); dir = 1; }
            if (ang > servo_angle_max()) { ang = servo_angle_max(); dir = -1; }

            servo_set_angle(ang);
            servo_wait_done(3000U);
            board_delay_us(250000U);

            cur = adc64_long_max_fast();

            if (cur > best) {
                best = cur;
            } else {
                dir = (int16_t)(-dir);
                delta = (delta > 1000) ? (delta * 2 / 3) : 1000;
            }

            if ((it % 4U) == 0U) {
                uart_puts("  it=");
                uart_put_u32((uint32_t)it);
                uart_puts(" ang=");
                uart_put_u32((uint32_t)(ang / 1000));
                uart_puts(" MAX=");
                uart_put_u32(cur);
                uart_puts(" BEST=");
                uart_put_u32(best);
                uart_puts("\r\n");
            }
        }
        uart_puts("LOCK done BEST=");
        uart_put_u32(best);
        uart_puts(" ang=");
        uart_put_u32((uint32_t)(ang / 1000));
        uart_puts("\r\n");
        return;
    }    /* ---- 相对角度移动（连续旋转舵机 / 无位置反馈） ---- */
    if (eqi(tok, "M") != 0U) {
        const char *r2;
        int32_t deg = 0;
        int32_t ms;
        int32_t pm;

        r2 = parse_int_plain(rest, &deg);
        if ((r2 == 0) || (deg == 0) || (deg > 360) || (deg < -360)) {
            reply_err("SYNTAX M <deg| -360..360>");
            return;
        }
        ms = ((deg < 0) ? -deg : deg) * (int32_t)s_ms_per_deg;
        if (ms > 8000) { ms = 8000; }
        pm = (deg > 0) ? 300 : -300;

        servo_set_enabled(1U);
        servo_spin(pm, (uint32_t)ms);
        s_vang_mdeg += deg * 1000;

        uart_puts("M ");
        uart_put_i32(s_vang_mdeg / 1000);
        uart_puts(" deg  spin=");
        uart_put_u32((uint32_t)ms);
        uart_puts("ms\r\n");
        return;
    }
    if (eqi(tok, "RATE") != 0U) {
        const char *r2;
        int32_t v = 0;
        r2 = parse_int_plain(rest, &v);
        if ((r2 == 0) || (v < 1) || (v > 200)) {
            reply_err("SYNTAX K RATE <1..200 ms/deg>");
            return;
        }
        s_ms_per_deg = (uint16_t)v;
        uart_puts("RATE=");
        uart_put_u32((uint32_t)s_ms_per_deg);
        uart_puts(" ms/deg\r\n");
        return;
    }
    if (eqi(tok, "Z") != 0U) {
        s_vang_mdeg = 0;
        uart_puts("VANG=0 deg\r\n");
        return;
    }    if (eqi(tok, "AUTO") != 0U) {
        uint8_t  it;
        int16_t  dir  = 1;
        int16_t  step = 10;
        uint32_t best;
        uint32_t cur;
        uint32_t prev;

        servo_set_enabled(1U);
        board_delay_us(200000U);

        best = adc64_long_max_fast();
        prev = best;
        uart_puts("AUTO start MAX=");
        uart_put_u32(best);
        uart_puts("\r\n");

        for (it = 0U; it < 80U; it++) {
            if (best >= 2800U) { break; }            /* 够亮：认为已垂直 */

            aa_step_deg((int32_t)dir * (int32_t)step);
            board_delay_us(80000U);

            cur = adc64_long_max_fast();

            if (cur > prev) {
                prev = cur;
                if (cur > best) { best = cur; }
                if (step < 30) { step = (int16_t)(step + 3); }   /* 同向加速 */
            } else {
                dir = (int16_t)(-dir);                        /* 反向 */
                if (step > 2) { step = (int16_t)(step / 2); }
                if (step < 2) { break; }
            }

            if ((it % 8U) == 0U) {
                uart_puts("AUTO it=");
                uart_put_u32((uint32_t)it);
                uart_puts(" MAX=");
                uart_put_u32(cur);
                uart_puts(" BEST=");
                uart_put_u32(best);
                uart_puts(" step=");
                uart_put_u32((uint32_t)step);
                uart_puts("\r\n");
            }
        }
        uart_puts("AUTO done BEST=");
        uart_put_u32(best);
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "SWEEP") != 0U) {
        int32_t  cur_ang = s_vangle / 1000;
        int32_t  a;
        int32_t  d;
        int32_t  best_ang = cur_ang;
        uint32_t best = 0U;
        uint32_t cur;

        servo_set_enabled(1U);

        /* ① 先记录扫描边界（必须在循环前算好，循环体会改 cur_ang） */
        {
        int32_t a_start = cur_ang - 90;
        int32_t a_end   = cur_ang + 90;
        for (a = a_start; a <= a_end; a += 10) {
            aa_step_deg(a - cur_ang);
            s_vangle = a * 1000;
            cur_ang  = a;
            board_delay_us(90000U);
            cur = adc64_long_max_fast();
            if (cur > best) { best = cur; best_ang = a; }
        }
        }

        /* ② 回到最优点 */
        aa_step_deg(best_ang - cur_ang);
        s_vangle = best_ang * 1000;
        cur_ang  = best_ang;
        board_delay_us(150000U);

        /* ③ 细扫 ±10°，步长 2° */
        for (d = -10; d <= 10; d += 2) {
            aa_step_deg((best_ang + d) - cur_ang);
            s_vangle = (best_ang + d) * 1000;
            cur_ang  = best_ang + d;
            board_delay_us(90000U);
            cur = adc64_long_max_fast();
            if (cur > best) { best = cur; }
        }

        /* ④ 停在最优角度 */
        aa_step_deg(best_ang - cur_ang);
        s_vangle = best_ang * 1000;

        uart_puts("SWEEP BEST=");
        uart_put_u32(best);
        uart_puts(" ANG=");
        uart_put_i32(best_ang);
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "BOTH") != 0U) {
        adc64_frame_t fr;
        uint16_t i;

        adc64_set_en_hold(0U);
        adc64_scan(&fr);

        uart_puts("LONG43");
        for (i = 21U; i < 64U; i++) {
            uart_putc(' ');
            uart_put_u32((uint32_t)fr.cell[0][(uint8_t)(i % 4U)][(uint8_t)(i / 4U)]);
        }
        uart_puts("\r\nSHORT18");
        for (i = 22U; i < 40U; i++) {
            uart_putc(' ');
            uart_put_u32((uint32_t)fr.cell[1][(uint8_t)(i % 4U)][(uint8_t)(i / 4U)]);
        }
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "GO") != 0U) {
        uint32_t m0;
        uint32_t m1;
        int16_t  dir  = 1;
        int16_t  step;
        int32_t  err;
        uint8_t  it;

        servo_set_enabled(1U);

        m0 = adc64_long_max_fast();
        uart_puts("GO start MAX=");
        uart_put_u32(m0);
        uart_puts("\r\n");

        for (it = 0U; it < 12U; it++) {
            if (m0 >= 1500U) { break; }            /* 已垂直 */

            /* 步长与误差成正比：离目标远就大步，近了就小步 */
            err = (int32_t)(1500U - m0);
            step = (int16_t)((err * 30) / 1500);
            if (step > 30) { step = 30; }
            if (step < 2)  { step = 2; }

            aa_step_deg((int32_t)dir * (int32_t)step);
            board_delay_us(120000U);

            m1 = adc64_long_max_fast();

            if (m1 > m0) {
                m0 = m1;                            /* 变好：保持方向 */
            } else {
                dir = (int16_t)(-dir);              /* 变差：反向 */
                if (step > 4) { step = (int16_t)(step / 2); }
            }

            uart_puts("GO it=");
            uart_put_u32((uint32_t)it);
            uart_puts(" MAX=");
            uart_put_u32(m1);
            uart_puts(" step=");
            uart_put_u32((uint32_t)step);
            uart_puts(" dir=");
            uart_put_i32((int32_t)dir);
            uart_puts("\r\n");
        }

        uart_puts("GO done MAX=");
        uart_put_u32(m0);
        uart_puts("\r\n");
        return;
    }    if (eqi(tok, "POS") != 0U) {
        adc64_frame_t fr;
        uint8_t r;
        uint8_t c;
        uint32_t wsum = 0U;
        uint32_t wpos = 0U;
        uint32_t peak = 0U;
        uint8_t peakc = 0U;
        uint32_t tot = 0U;

        adc64_scan(&fr);
        for (r = 0U; r < ADC64_ROWS; r++) {
            for (c = 0U; c < ADC64_COLS; c++) {
                uint32_t v = fr.cell[0][r][c];
                if (c == 3U) { continue; }              /* 跳过 CH3 坏列 */
                if (v < 15U) { continue; }              /* 噪声门限 */
                wsum += v;
                wpos += v * (uint32_t)c;
                tot  += v;
                if (v > peak) { peak = v; peakc = c; }
            }
        }
        uart_puts("POS=");
        if (wsum == 0U) {
            uart_puts("--  SUM=0\r\n");
        } else {
            uint32_t x10 = (wpos * 10U) / wsum;
            uart_put_u32(x10 / 10U);
            uart_putc('.');
            uart_put_u32(x10 % 10U);
            uart_puts("  PEAK=CH");
            if (peakc < 10U) { uart_putc((char)('0' + peakc)); }
            else { uart_putc((char)('A' + peakc - 10U)); }
            uart_puts("  VAL=");
            uart_put_u32(peak);
            uart_puts("  SUM=");
            uart_put_u32(tot);
            uart_puts("\r\n");
        }
        return;
    }    if (eqi(tok, "RESET") != 0U) {
        track_reset();
        uart_puts("OK RESET\r\n");
        return;
    }
    reply_err("SYNTAX K[ON|OFF|A|B|SIGN|DARK|RAW|TEL|CURVE|MASKA|MASKB|RESET]");
}

/* 遥测：跟踪时输出 S/P/角度/状态，未跟踪时输出原始通道值（用于接线检查） */
void cmd_telem_poll(void)
{
    if (s_telem == 0U) {
        return;
    }
    if ((uint32_t)(board_millis() - s_telem_ms) < 100U) {
        return;
    }
    s_telem_ms = board_millis();

    if (track_enabled() != 0U) {
        const sensor_meas_t *m = track_last();

        uart_puts("$T,");
        uart_put_i32(m->s_permille);
        uart_putc(',');
        uart_put_u32(m->total);
        uart_putc(',');
        uart_put_mdeg(servo_get_state().target_mdeg);
        uart_putc(',');
        uart_put_u32((uint32_t)track_state());
        uart_putc(',');
        uart_put_u32((uint32_t)m->valid);
        uart_puts("\r\n");
    } else {
        sensor_meas_t m;

        sensor_measure(&m);
        uart_puts("$R,");
        uart_put_u32(m.total);
        uart_putc(',');
        uart_put_u32(m.sum_a);
        uart_putc(',');
        uart_put_u32(m.sum_b);
        uart_putc(',');
        uart_put_i32(m.s_permille);
        uart_puts("\r\n");
    }
}

/*----------------------------------------------------------------------------
 * 命令分发
 *--------------------------------------------------------------------------*/
void cmd_exec(char *line)
{
    char  cmd;
    char *arg;

    while ((*line == ' ') || (*line == '\t')) {
        line++;
    }
    if (*line == '\0') {
        return;
    }

    cmd = *line;
    if ((cmd >= 'a') && (cmd <= 'z')) {
        cmd = (char)(cmd - 'a' + 'A');
    }
    arg = line + 1;
    while ((*arg == ' ') || (*arg == '\t')) {
        arg++;
    }

    switch (cmd) {
    case 'A': do_angle(arg);            break;
    case '+': do_jog(arg, 1);           break;
    case '-': do_jog(arg, -1);          break;
    case 'N': do_level(arg);            break;
    case 'K': do_track(arg);            break;
    case 'P': do_pulse(arg);            break;
    case 'C': do_center();              break;
    case 'L': do_limits(arg);           break;
    case 'W': do_calib(arg);            break;
    case 'T': do_trim(arg);             break;
    case 'S': do_speed(arg);            break;
    case 'X': do_enable(arg);           break;
    case 'Q':
    case '?': print_state();            break;
    case 'R':
        servo_defaults();
        uart_puts("OK RESET\r\n");
        print_state();
        break;
    case 'Z': do_sweep();               break;
    case 'V': do_version();             break;
    case 'H': print_help();             break;
    default:  reply_err("CMD");         break;
    }
}

/*----------------------------------------------------------------------------
 * 行组装（非阻塞）
 *--------------------------------------------------------------------------*/
void cmd_init(void)
{
    s_len = 0U;
}

void cmd_poll(void)
{
    uint8_t ch;

    while (uart_read(&ch) != 0U) {
        if ((ch == '\r') || (ch == '\n')) {
            if (s_len > 0U) {
                s_line[s_len] = '\0';
                cmd_exec(s_line);
                s_len = 0U;
            }
        } else if ((ch == 0x08U) || (ch == 0x7FU)) {
            if (s_len > 0U) {
                s_len--;                        /* 退格，方便串口助手手动输入 */
            }
        } else if ((ch >= 0x20U) && (ch < 0x7FU)) {
            if (s_len < (CMD_LINE_MAX - 1U)) {
                s_line[s_len] = (char)ch;
                s_len++;
            } else {
                s_len = 0U;
                reply_err("LINE_TOO_LONG");
            }
        } else {
            /* 其它控制字符直接忽略 */
        }
    }
}

/*----------------------------------------------------------------------------
 * 上电自动对准：无需串口命令
 *   ① 等光出现
 *   ② 爬山法转舵机，让长边峰值 MAX 最大
 *   ③ 持续监控，光偏了自动重找
 *--------------------------------------------------------------------------*/
static void aa_step_deg(int32_t deg)
{
    int32_t ms;

    if (deg == 0) { return; }
    ms = (deg < 0 ? -deg : deg) * (int32_t)s_ms_per_deg;
    if (ms < 10)   { ms = 10; }
    if (ms > 4000) { ms = 4000; }
    servo_set_enabled(1U);
    servo_spin((deg > 0) ? 300 : -300, (uint32_t)ms);
}

void autoalign_run(void)
{
    uint32_t best = 0U;
    uint32_t cur;
    int16_t  dir  = 1;
    int16_t  step = 6;
    uint8_t  warm = 0U;
    uint16_t cnt  = 0U;
    uint32_t last = 0U;
    int32_t  err  = 0;

    servo_set_enabled(1U);
    uart_puts("TRK start\r\n");

    for (;;) {
        cur = adc64_long_max_fast();
        cnt++;

        if (cnt >= 4U) {
            cnt = 0U;
            uart_puts("TRK MAX=");
            uart_put_u32(cur);
            uart_puts(" step=");
            uart_put_u32((uint32_t)step);
            uart_puts(" dir=");
            uart_put_i32((int32_t)dir);
            uart_puts("\r\n");
        }

        /* 没光：不动作，等着 */
        if (cur < 300U) {
            best = 0U;
            warm = 0U;
            board_delay_us(250000U);
            continue;
        }

        /* 刷新历史最优（缓慢衰减，适应光强变化） */
        if (warm == 0U) {
            best = cur;
            warm = 1U;
        } else if (cur > best) {
            best = cur;
        } else {
            best -= (best / 2000U);          /* 每轮缓慢衰减 0.05% */
            if (best < 100U) { best = 100U; } /* 防止下溢，不钳到 cur */
        }

        /* 相对判据：接近历史最优（92%）→ 认为已垂直，静止 */
        if (cur * 100U >= best * 92U) {
            board_delay_us(300000U);
            continue;
        }

        /* 方向：本轮相对上一次是否变好 */
        if (cur > last) {
            /* 保持方向 */
        } else {
            dir = (int16_t)(-dir);
        }
        last = cur;

        /* 步长与"距离最优的差距"成正比 */
        err = (int32_t)(best - cur);
        step = (int16_t)(((int32_t)best > 0) ? ((err * 24) / (int32_t)best) : 6);
        if (step > 24) { step = 24; }
        if (step < 2)  { step = 2; }

        aa_step_deg((int32_t)dir * (int32_t)step);
        board_delay_us(120000U);
    }
}
/* 上电全自动：粗扫 360° -> 细扫 ±10° -> 停最优 -> 持续监控 */
void autoalign_sweep(void)
{
    for (;;) {
        int32_t  cur_ang;
        int32_t  a;
        int32_t  d;
        int32_t  best_ang;
        uint32_t best;
        uint32_t cur;

        /* 等光 */
        for (;;) {
            if (adc64_long_max_fast() >= 1400U) { break; }
            board_delay_us(300000U);
        }

        servo_set_enabled(1U);
        cur_ang  = s_vangle / 1000;
        best_ang = cur_ang;
        best     = 0U;

        /* 先记录边界（循环体会改 cur_ang，不能在条件里用） */
        {
        int32_t a_start = cur_ang - 90;
        int32_t a_end   = cur_ang + 90;
        for (a = a_start; a <= a_end; a += 10) {
            aa_step_deg(a - cur_ang);
            s_vangle = a * 1000;
            cur_ang  = a;
            board_delay_us(90000U);
            cur = adc64_long_max_fast();
            if (cur > best) { best = cur; best_ang = a; }
        }
        }

        /* 回最优点 */
        aa_step_deg(best_ang - cur_ang);
        s_vangle = best_ang * 1000;
        cur_ang  = best_ang;
        board_delay_us(150000U);

        /* 细扫 ±10°，2° 一步 */
        for (d = -10; d <= 10; d += 2) {
            aa_step_deg((best_ang + d) - cur_ang);
            s_vangle = (best_ang + d) * 1000;
            cur_ang  = best_ang + d;
            board_delay_us(90000U);
            cur = adc64_long_max_fast();
            if (cur > best) { best = cur; }
        }

        aa_step_deg(best_ang - cur_ang);
        s_vangle = best_ang * 1000;

        /* 监控：MAX 掉到 70% 以下就重扫 */
        for (;;) {
            board_delay_us(300000U);
            cur = adc64_long_max_fast();
            if (cur < 1300U) { break; }
        }
    }
}