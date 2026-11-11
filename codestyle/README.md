# codestyle —— 代码规范工具包

**一个文件夹装齐"代码管理"需要的东西**：检查工具、提交钩子、流水线入口、制度文档、模板。
按小公司（1~3 人）设计，全部条款都能在 5 分钟内做完。

---

## 里面有什么

| 文件 | 干什么 | 什么时候用 |
|---|---|---|
| `代码管理制度.md` | 制度正文：分支怎么开、提交怎么写、怎么发版、怎么强制检查 | 新人第一天读一遍；有争议时看这里 |
| `check_code.py` | 规范检查工具（纯 Python 标准库，不用装包） | 手动体检 / 钩子自动调 / 流水线自动调 |
| `install_hooks.ps1` | 一键启用本地提交钩子 | **每台电脑装一次** |
| `ci_check.sh` | 云效流水线入口（Linux 构建机） | 配在流水线的"执行命令"里 |
| `hooks/pre-commit` | 提交前检查（被 `install_hooks.ps1` 启用，不用手动跑） | 自动 |
| `hooks/commit-msg` | 检查提交信息格式（同上） | 自动 |
| `templates/gitignore.txt` | `.gitignore` 模板 | **换新项目时**复制到仓库根目录 |
| `templates/gitattributes.txt` | `.gitattributes` 模板 | 同上 |

> `.gitignore` 和 `.gitattributes` **必须放在仓库根目录**才生效，所以这里只放模板。
> 本仓库根目录已经有一份了，不用动。

---

## 3 分钟上手

### 拉完代码，第一次（每台电脑一次）

```powershell
powershell -ExecutionPolicy Bypass -File codestyle\install_hooks.ps1
py -3 codestyle\check_code.py            # 确认能跑起来
```

装完 `git config core.hooksPath` 应该是 `codestyle/hooks`。

### 每天干活

```powershell
git switch dev && git pull                    # 1. 拿最新代码
git switch -c feature/我要加的功能             # 2. 开分支
#  ... 改代码、编译、上板测 ...
git add -A                                    # 3. 暂存
git commit -m "feat(模块): 做了什么"           # 4. 提交（钩子自动检查，红了就改）
git push -u origin feature/我要加的功能        # 5. 推到云效，然后建合并请求
```

### 发版

```powershell
git switch -c release/v1.2
#  改 Core\Inc\config.h 里的 FW_VERSION → "1.2"
py -3 codestyle\check_code.py --release v1.2   # 版本号三处一致检查
#  编译出 hex、合到 main、打 tag（详见 代码管理制度.md 第 2.3 节）
```

---

## 检查工具能手动跑哪几种

```powershell
py -3 codestyle\check_code.py                   # 整仓体检（最常用）
py -3 codestyle\check_code.py --staged          # 只查暂存区（钩子用）
py -3 codestyle\check_code.py --message "feat(x): 试一下"
py -3 codestyle\check_code.py --branch feature/xxx
py -3 codestyle\check_code.py --release v1.2
py -3 codestyle\check_code.py --ci origin/main  # 流水线用：查本次改动
py -3 codestyle\check_code.py --all             # 列出全部问题（默认每类只显示 3 个）
```

退出码 **0 = 通过，1 = 有错误**，所以能直接当门禁。

---

## 云效上怎么强制（让开发人员绕不过去）

只装本地钩子不够（`--no-verify` 能跳）。真正的门禁在服务端，三步：

1. `main` 设成**保护分支**，禁止直接推送
2. 建流水线 `code-check`，执行命令填 `sh codestyle/ci_check.sh`
3. 合并请求设置里勾选「合并前必须通过流水线」

配完之后：**检查不通过 = 合并按钮点不动**。详细步骤见 `代码管理制度.md` 第 6 章。

---

## 复制到别的项目

整个 `codestyle/` 文件夹拷过去，然后改两处：

| 改什么 | 在哪 |
|---|---|
| 仓库根目录需要 `.gitignore` / `.gitattributes` | 从 `templates/` 复制过去，改一下里面的路径规则 |
| 钩子路径 | 跑一次 `powershell -ExecutionPolicy Bypass -File codestyle\install_hooks.ps1` 就自动指对了 |

工具本身是**零依赖**的，只管"哪些文件不该进仓库 / 提交信息格式 / 大文件 / 密码 / 中断里延时"，
和具体项目无关；只有"源文件有没有加进 Keil 工程"这一项会找 `*.uvprojx`，没有就自动跳过。

---

## 想改规则

| 你想改 | 改哪里 |
|---|---|
| 哪些文件不许提交（正则） | `check_code.py` 里的 `BAD_PATH_RULES` |
| 允许的提交类型 | `check_code.py` 里的 `ALLOWED_TYPES` |
| 大文件阈值（现在 20MB） | `check_code.py` 里的 `MAX_FILE_MB` |
| 分支名规则 | `check_code.py` 里的 `BRANCH_RE` |
| 制度条文 | `代码管理制度.md` |

改完在本地跑一遍 `py -3 codestyle\check_code.py` 确认没把正常文件误判，再提交。

---

## 常见情况

| 现象 | 处理 |
|---|---|
| 提交被拦 | 看屏幕上的"怎么改"，`代码管理制度.md` 第 5.3 节有对照表 |
| 钩子没反应 | `git config core.hooksPath` 应为 `codestyle/hooks`；不对就重跑 `install_hooks.ps1` |
| 合并按钮点不动 | 流水线没过，点进流水线看日志（第 6 章） |
| 想临时跳过一次 | `git commit --no-verify` |
| 想彻底关掉 | `git config --unset core.hooksPath` |
| `python` 报 9009 | 这台机器要用 `py -3` |
