# 27寸交叉点项目

基于 STM32G431CB 和八边光电探测器的 27 寸光学膜激光定位项目，包含板端固件、综合调试 UI、板型映射及建模参考图。

本目录是独立工程，仅存放在 `fullroad` 仓库中；与仓库根目录原有的四边探测器/STM32F429 工程分别维护。**在 VS Code 中请直接打开本目录 `27寸交叉点项目`**，使编译、烧录和调试任务使用正确的工作目录。

## 当前功能

- 内外上、下、左、右八条边的采集和 CAN FD 数据汇总。
- 固件完成 18 kHz 同步解调、背景/噪声抑制、单峰保留、运动自适应幅值平滑、质心提取、坐标标定映射、连线和交点稳定输出。
- X 方向使用上下有效质心形成竖直线：优先信号相近的 3/4 点，按条件回退到同侧内外点对或可用点。
- Y 方向优先使用近侧内外点对，结合左右 3/4 点一致性约束、离群点处理和短基线方向限制。
- 产品交点进行范围保护、异常跳点确认和自适应平滑；短暂无效最多保持约 200 ms，超过后撤销有效状态，不持续外推位置。
- UI 同屏显示处理后波形、八边质心、固件实际选线、原始交点数值和稳定输出；支持轨迹显示/隐藏、清空轨迹、连线开关、缩放和平移。
- UI 按接收帧缓存固件坐标，不重新计算质心、选线、交点或滤波。稳定输出可能偏离当帧原始求交线，两个结果分别显示。
- 固件提供 CAN bus-off 恢复，恢复检查不依赖收到同步或发送队列存在空位。

## 硬件与坐标

| 项目 | 配置 |
| --- | --- |
| MCU | STM32G431CB，Cortex-M4，170 MHz |
| 有效显示区 | 599 × 352 mm |
| 外框 | 649 × 402 mm，四侧各扩展 25 mm |
| 坐标原点 | 内屏有效显示区左下角，X 向右、Y 向上 |
| 30 cm PCB | 每块 50 颗探测器 |
| 35 cm PCB | 每块 58 颗探测器 |
| 探测器 | SFH203PFA，相邻中心间距约 6 mm |
| 激光 | 当前测试为 940 nm，约 18 kHz 调制 |
| 通信 | 板间 CAN FD，默认 1 Mbps、不启用 BRS；主控串口 460800 |

共 12 块板。长边分段编号按左至右为 0、1；短边只有第 0 段。

| 边 | 第 0 段 | 第 1 段 | 探测器总数 |
| --- | --- | --- | ---: |
| 外上 `EDGE_UP` | 30 cm，主控 | 35 cm，从机 | 108 |
| 外下 `EDGE_DOWN` | 35 cm | 30 cm | 108 |
| 外左 `EDGE_LEFT` | 35 cm | — | 58 |
| 外右 `EDGE_RIGHT` | 35 cm | — | 58 |
| 内上 `EDGE_IN_UP` | 30 cm | 35 cm | 108 |
| 内下 `EDGE_IN_DOWN` | 35 cm | 30 cm | 108 |
| 内左 `EDGE_IN_LEFT` | 35 cm | — | 58 |
| 内右 `EDGE_IN_RIGHT` | 35 cm | — | 58 |

当前结构的内长边槽延伸到外框，内长边 PCB 左端位于有效显示区原点左侧，不能把每条探测器数组首元素都当作内屏 X=0。板上二极管首末位置、方向、拼接和实际坐标由固件映射及标定共同处理；PCB 标称 30/35 cm 不等于二极管有效跨度。

## 目录

```text
27寸交叉点项目/
├─ application/
│  ├─ config.h              # 本板身份、板型表、几何和算法参数
│  ├─ app_core.c            # 主控采集、处理和输出链路
│  ├─ board_map.c           # 各边各段传感器数量及拼接
│  ├─ runtime_params.c      # 调参、标定、Flash 持久化
│  ├─ algorithm/            # 质心、滤波、选线及交点
│  ├─ bsp/                  # ADC、传感器、CAN、串口等驱动封装
│  ├─ collect/              # 从板等待同步、采集并上传
│  └─ protocol/             # CAN 分帧及串口协议
├─ Src/、Inc/、Drivers/     # STM32 初始化、CMSIS 和 HAL
├─ UI/unified_ui.py         # 综合产品调试台
├─ board_layout.py          # Python 八边布局，与固件段表一致
├─ .vscode/                 # 编译、J-Link 烧录和调试任务
├─ 建模/                    # 尺寸 PDF 和参考图
├─ Makefile
├─ STM32G431CBTx_FLASH.ld
└─ screen_ctrl_ultra.ioc
```

根目录的 `capture_*.py`、`lab_*.py`、`serial_view.py`、`plot_trajectory.py` 为历史采集/分析辅助脚本，其中部分针对旧串口帧格式；当前综合调试以 `UI/unified_ui.py` 为入口。构建目录、Python 缓存及历史测试数据不提交。

## 编译与烧录

### 环境

- Windows、GNU Make、Arm GNU Toolchain（当前验证使用 GCC 13.2.1）。
- SEGGER J-Link 软件及 SWD 调试器，目标设备 `STM32G431CB`。
- VS Code 可安装 Cortex-Debug 扩展用于调试。

`.vscode/tasks.json` 和 `launch.json` 保留当前测试电脑的工具绝对路径。其他电脑需将 Make、GCC、J-Link 路径以及调试器序列号调整为本机配置。`flash.jlink` 使用相对路径 `build/screen_ctrl_ultra.elf`，运行目录必须为本工程根目录。

### 逐板配置

烧录前只在 `application/config.h` 修改以下四项，再重新编译：

```c
// 默认目标：外上左侧30cm主控
#define MY_EDGE       EDGE_UP
#define MY_SEGMENT    0
#define MCU_IS_MASTER 1
#define BOARD_TYPE    BOARD_30CM
```

例如内右从板使用：

```c
#define MY_EDGE       EDGE_IN_RIGHT
#define MY_SEGMENT    0
#define MCU_IS_MASTER 0
#define BOARD_TYPE    BOARD_35CM
```

其余板按照硬件表选择边、段号、板型，只有外上第 0 段设置主控。所有板共用源码和 `build` 输出目录，因此每次换板身份后必须重新编译，不能把上一块板的 ELF 直接烧给下一块。段表 `*_SEG_BOARD` 与 `board_layout.py` 应保持一致。

### 命令行

在本目录的 PowerShell 终端中执行（路径按本机安装位置修改）：

```powershell
& "C:\Users\a\AppData\Local\Programs\make\bin\make.exe" -j4 BUILD_DIR=build GCC_PATH=C:/Users/a/AppData/Local/Programs/gcc-arm-none-eabi/bin
```

输出 `build/screen_ctrl_ultra.elf`、`.hex`、`.bin`。

```powershell
& "C:\Program Files\SEGGER\JLink_V986\JLink.exe" -NoGui 1 -ExitOnError 1 -device STM32G431CB -if SWD -speed 2000 -autoconnect 1 -CommandFile .vscode/flash.jlink
```

VS Code 也可运行任务“固件：一键编译并烧录”，它会先按 `config.h` 编译，再烧录当前连接的板。更换固件后检查日志中的下载/校验结果与板端运行状态。

## 运行 UI

Python 3.12 已验证。首次安装依赖：

```powershell
python -m pip install numpy pyserial PyQt5 pyqtgraph
```

在本工程根目录运行：

```powershell
python UI/unified_ui.py
```

选择主控实际串口和 `460800` 波特率，点击“连接”。也可传入端口：

```powershell
python UI/unified_ui.py COM5
```

`pyautogui` 是鼠标联动的可选依赖，普通波形和轨迹显示不需要安装。每个串口只能由一个程序占用，直接采集或其他串口工具使用前先断开 UI。

界面中的“通信未收到”表示该边本轮 CAN 分段不完整；“无光/质心无效”表示已收到该边，但波形未满足定位条件，两者应分别排查。长边由两块板组成，任一段未到齐都会使整边显示未收到。

轨迹使用固件最终输出；真实无效或断光会断段，不补造位置。可通过“显示移动轨迹”“清空轨迹”“隐藏/显示连线”调整视图。

## 物理标定

1. 固定 PCB、光学膜和激光测试结构，确认板型、段号、方向及八边通信正常。
2. 以有效区左下角为原点，使用垂直入射的十字光，进入“固件几何标定”。
3. 使用 UI 固定的 7×3、共 21 点蛇形序列：X 为 `30、95、225、290、355、485、550 mm`；Y 为 `30、160、290 mm`。
4. 按当前目标坐标放置光斑并保持稳定，点击“记录当前点”，等待固件返回记录状态，再切到下一点。默认每点采集 20 帧。
5. 完成 21 点并确认八边唯一节点完整后，点击“固件检查、拟合并应用”，检查状态为“拟合已应用并保存”。横向边目标为 7 个 X 节点，纵向边为 3 个 Y 节点，因此节点图数量不等于记录次数。
6. 在标定点之间另外选点验证，不以标定点重合代替全屏精度验证。重新装配或改变二极管映射、质心估计方法后，应复核并按需要重新标定。

标定用于建立每条边的传感器质心索引与物理坐标关系，修正可重复的装配/映射偏差；不能保证消除膜内部随位置变化的非规则光路误差或运动误差。拟合、应用和保存均由固件执行。主控 Flash 最后 2 KB 用于标定存储，链接脚本仅分配 126 KB 给程序；UI 还通过本机 QSettings 保存安全备份。**本仓库不包含实机 Flash 或电脑注册表中的标定数据，新硬件应自行标定。**

## 当前验证与限制

2026-11-26 的一次实机连续移动采样：268 帧均收到八边完整数据，最终交点 268 帧有效，无屏外输出，串口 CRC 错误为 0；其中 19 帧进入短暂保持。当前完整诊断流帧长约 3323 字节、周期约 74 ms（约 13.5 FPS），采样与串口带宽仍限制快速笔迹的时间分辨率。

一次静止点 6 秒观测中，输出变化范围约 X=0.15 mm、Y=0.03 mm。该结果仅表示当次点位的重复稳定性，不能作为全屏绝对定位精度或所有速度下无跳动的承诺。后续需使用独立已知坐标、不同区域和移动速度验证。

CAN bus-off 会使旧从板停止上传。当前源码在等同步、等发送空位等路径均执行恢复服务；只更新主控不能修复旧从板自身的恢复逻辑。若频繁触发，仍需检查终端电阻、接地、供电、接线及位时序，自动恢复不能替代总线故障排查。

## 第三方代码

`Drivers/` 等目录包含 ST CMSIS/HAL 代码，使用时遵守各源文件及随附目录中的许可声明。
