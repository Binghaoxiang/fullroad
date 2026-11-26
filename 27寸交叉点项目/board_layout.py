# board_layout.py — 每边每段板型表（必须与主控固件 config.h 的 *_SEG_BOARD 一一对应）
# 50 = 30cm 板（7 mux, 6*8+2 颗/段）; 58 = 35cm 板（8 mux, 7*8+2 颗/段）
# 同一条边内可逐段混用; 顺序 = 段0, 段1, ...
# 27 寸膜当前实物布局:
#   up               = 30,35
#   down             = 35,30
#   left/right       = 35
#   in_up            = 30,35（槽延伸到外框，左端在内屏原点左侧）
#   in_down          = 35,30（槽延伸到外框，左端在内屏原点左侧）
#   in_left/in_right = 35

SEG_BOARDS = {
    'up':       [50, 58],
    'down':     [58, 50],
    'left':     [58],
    'right':    [58],
    'in_up':    [50, 58],
    'in_down':  [58, 50],
    'in_left':  [58],
    'in_right': [58],
}

# 全帧内的边顺序（与固件 frame.c 一致）
EDGE_ORDER = ['up', 'down', 'left', 'right', 'in_up', 'in_down', 'in_left', 'in_right']

EDGE_SENSORS = {name: sum(SEG_BOARDS[name]) for name in EDGE_ORDER}
EDGES = [(name, EDGE_SENSORS[name]) for name in EDGE_ORDER]
RAW_BYTES = sum(EDGE_SENSORS.values()) * 2
