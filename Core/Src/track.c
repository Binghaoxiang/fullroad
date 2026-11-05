/**
  ******************************************************************************
  * @file    track.c
  * @brief   自动对准状态机实现
  *
  *  节拍：track_tick_20ms() 每 20ms 一次；每次动作后等 TRK_SETTLE_MS
  *        让 50 寸膜结构稳定，再做下一次测量，避免大惯量结构振荡。
  ******************************************************************************
  */
#include "track.h"
#include "servo.h"
#include "board.h"

#define TICK_MS         20U
#define SETTLE_TICKS    ((uint16_t)(TRK_SETTLE_MS / TICK_MS))

static uint8_t       s_enable;
static uint8_t       s_mode   = 1U;      /* 1 = A 差分, 2 = B 强度 */
static int8_t        s_sign   = 1;
static trk_state_t   s_state  = TRK_ST_OFF;
static sensor_meas_t s_last;
static uint16_t      s_wait;
static int8_t        s_dir    = 1;
static uint32_t      s_best_p;
static uint32_t      s_prev_p;
static int32_t       s_angle;
static uint32_t      s_lock_start;

static int32_t abs32(int32_t v)
{
    return (v >= 0) ? v : -v;
}

static void wait_settle(void)
{
    s_wait = SETTLE_TICKS;
}

/* 把角度限制在舵机行程内；返回 1 表示被夹住 */
static uint8_t move_to(int32_t mdeg)
{
    if (mdeg < servo_angle_min()) {
        servo_set_angle(servo_angle_min());
        return 1U;
    }
    if (mdeg > servo_angle_max()) {
        servo_set_angle(servo_angle_max());
        return 1U;
    }
    servo_set_angle(mdeg);
    return 0U;
}

void track_init(void)
{
    s_enable  = 0U;
    s_state   = TRK_ST_OFF;
    s_last.total     = 0U;
    s_last.s_permille = 0;
    s_last.valid     = 0U;
    s_wait    = 0U;
    s_dir     = 1;
    s_best_p  = 0U;
    s_prev_p  = 0U;
    s_angle   = servo_get_state().target_mdeg;
    s_lock_start = 0U;
#if (TRK_CH_COUNT < 2U)
    s_mode = 2U;                        /* 单通道只能用强度爬坡 */
#endif
}

void track_reset(void)
{
    s_state   = s_enable ? TRK_ST_SEARCH : TRK_ST_OFF;
    s_wait    = 0U;
    s_best_p  = 0U;
    s_prev_p  = 0U;
    s_lock_start = 0U;
    s_dir     = 1;
}

void track_enable(uint8_t on)
{
    s_enable = (on != 0U) ? 1U : 0U;
    track_reset();
}

uint8_t track_enabled(void)
{
    return s_enable;
}

void track_set_mode(uint8_t mode)
{
    if ((mode == 1U) || (mode == 2U)) {
        s_mode = mode;
    }
#if (TRK_CH_COUNT < 2U)
    s_mode = 2U;
#endif
}

uint8_t track_mode(void)
{
    return s_mode;
}

void track_set_sign(int8_t sign)
{
    s_sign = (sign < 0) ? -1 : 1;
}

int8_t track_sign(void)
{
    return s_sign;
}

trk_state_t track_state(void)
{
    return s_state;
}

uint32_t track_lock_ms(void)
{
    return (s_state == TRK_ST_LOCK) ? (board_millis() - s_lock_start) : 0U;
}

const sensor_meas_t *track_last(void)
{
    return &s_last;
}

const char *track_state_name(void)
{
    switch (s_state) {
    case TRK_ST_SEARCH: return "SEARCH";
    case TRK_ST_SEEK:   return "SEEK";
    case TRK_ST_LOCK:   return "LOCK";
    case TRK_ST_LOST:   return "LOST";
    default:            return "OFF";
    }
}

/* 搜索：单向慢扫，撞到行程端点就反向 */
static void do_search(void)
{
    int32_t next = s_angle + (int32_t)s_dir * TRK_SEARCH_STEP_MDEG;

    if (next > servo_angle_max()) {
        s_dir = -1;
        next  = s_angle + (int32_t)s_dir * TRK_SEARCH_STEP_MDEG;
    } else if (next < servo_angle_min()) {
        s_dir = 1;
        next  = s_angle + (int32_t)s_dir * TRK_SEARCH_STEP_MDEG;
    }
    (void)move_to(next);
    wait_settle();
}

/* ---- PID 参数（内部 x1000 定点）：输出 = (Kp*|e| + Ki*|∫e| + Kd*|Δe|)/1000 毫度 ---- */
static int32_t s_kp1000 = (int32_t)TRK_KP_MDEG_PER_PERMILLE * 1000;
static int32_t s_ki1000;
static int32_t s_kd1000;
static int32_t s_iacc;
static int32_t s_eprev;

void track_set_pid(int32_t kp, int32_t ki, int32_t kd)
{
    s_kp1000 = kp * 1000;
    s_ki1000 = ki * 1000;
    s_kd1000 = kd * 1000;
    s_iacc   = 0;
    s_eprev  = 0;
}
int32_t track_get_kp(void) { return s_kp1000 / 1000; }
int32_t track_get_ki(void) { return s_ki1000 / 1000; }
int32_t track_get_kd(void) { return s_kd1000 / 1000; }

/* 方案 A：按归一化差分 S 比例步进，S 过零即垂直入射 */
static void do_seek_diff(void)
{
    int32_t mag = abs32(s_last.s_permille);
    int32_t step;
    int32_t dir;
    int32_t next;

    if (mag <= TRK_S_DEADBAND) {
        s_state   = TRK_ST_LOCK;
        s_lock_start = board_millis();
        s_best_p  = s_last.total;
        wait_settle();
        return;
    }

    /* PID：误差取 |S|，输出角度增量（毫度） */
    {
        int32_t d = s_last.s_permille - s_eprev;
        s_eprev = s_last.s_permille;
        s_iacc += s_last.s_permille;
        if (s_iacc >  400000) { s_iacc =  400000; }
        if (s_iacc < -400000) { s_iacc = -400000; }
        step = (s_kp1000 * mag) / 1000
             + (s_ki1000 * abs32(s_iacc)) / 1000
             + (s_kd1000 * abs32(d)) / 1000;
    }
    if (step < TRK_STEP_MIN_MDEG) {
        step = TRK_STEP_MIN_MDEG;
    }
    if (step > TRK_STEP_MAX_MDEG) {
        step = TRK_STEP_MAX_MDEG;
    }
    /* S>0 表示光斑偏左半，需要往右转；符号不对时用 TRK SIGN 翻转 */
    dir  = (s_last.s_permille > 0) ? -1 : 1;
    next = s_angle + (dir * (int32_t)s_sign * step);

    if (move_to(next) != 0U) {
        s_state = TRK_ST_LOCK;          /* 顶到行程端点，先锁住并上报 */
        s_lock_start = board_millis();
    }
    wait_settle();
}

/* 方案 B：扰动观察法爬 P 峰值 */
static void do_seek_p(void)
{
    uint32_t p = s_last.total;
    int32_t  noise;
    int32_t  diff;

    if (s_prev_p == 0U) {
        s_prev_p = p;
        s_best_p = p;
    }
    noise = (int32_t)(s_prev_p / 100U) + 1;      /* 1% 死区 */
    diff  = (int32_t)p - (int32_t)s_prev_p;

    if (diff > noise) {
        s_best_p = p;                            /* 还在爬升，方向不变 */
    } else if (diff < -noise) {
        s_dir = (int8_t)(-s_dir);                /* 翻过山顶，掉头 */
    } else {
        s_state   = TRK_ST_LOCK;                 /* 两次变化都在噪声内：到顶 */
        s_lock_start = board_millis();
        s_best_p  = p;
        s_prev_p  = p;
        wait_settle();
        return;
    }
    s_prev_p = p;
    (void)move_to(s_angle + (int32_t)s_dir * TRK_DITHER_MDEG);
    wait_settle();
}

void track_tick_20ms(void)
{
    if (s_enable == 0U) {
        s_state = TRK_ST_OFF;
        return;
    }
    if (s_wait > 0U) {
        s_wait--;
        return;
    }

    sensor_measure(&s_last);
    s_angle = servo_get_state().target_mdeg;

    switch (s_state) {
    case TRK_ST_OFF:
        s_state = TRK_ST_SEARCH;
        break;

    case TRK_ST_SEARCH:
        if (s_last.valid != 0U) {
            s_state  = TRK_ST_SEEK;
            s_best_p = s_last.total;
            s_prev_p = s_last.total;
        } else {
            do_search();
        }
        break;

    case TRK_ST_SEEK:
        if (s_last.valid == 0U) {
            s_state = TRK_ST_LOST;
            wait_settle();
        } else if (s_mode == 1U) {
            do_seek_diff();
        } else {
            do_seek_p();
        }
        break;

    case TRK_ST_LOCK:
        if (s_last.valid == 0U) {
            s_state = TRK_ST_LOST;
        } else if (s_mode == 1U) {
            if (abs32(s_last.s_permille) > TRK_S_OUTBAND) {
                s_state = TRK_ST_SEEK;                     /* 跑了，重新追 */
                s_wait  = 0U;
                break;
            }
        } else {
            if ((s_last.total * 1000U) < (s_best_p * (1000U - TRK_P_DROP_PERMILLE))) {
                s_state = TRK_ST_SEEK;
                s_prev_p = 0U;
                s_wait  = 0U;
                break;
            }
        }
        wait_settle();                                     /* 锁定时只监视不动作 */
        break;

    case TRK_ST_LOST:
        if (s_last.valid != 0U) {
            s_state  = TRK_ST_SEEK;
            s_best_p = s_last.total;
            s_prev_p = s_last.total;
            s_wait   = 0U;
        } else {
            wait_settle();
        }
        break;

    default:
        s_state = TRK_ST_SEARCH;
        break;
    }
}
