#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
代码规范检查 —— 提交前自动跑一遍，不合规直接拦住，不用靠记性。

用法（在仓库根目录执行）：
    py -3 codestyle/check_code.py                 体检整个仓库
    py -3 codestyle/check_code.py --staged        只查暂存区（装了钩子后每次提交自动跑）
    py -3 codestyle/check_code.py --message "feat(x): 加了什么"
    py -3 codestyle/check_code.py --branch feature/adc-pcb-order
    py -3 codestyle/check_code.py --release v2.3.0  发版前查版本号是否三处一致
    py -3 codestyle/check_code.py --ci origin/main  云效流水线用：查本次改动（不合规流水线失败）
    加 --all 可以列出全部问题（默认每类只显示 3 个例子）

退出码：0 = 通过；1 = 有错误，先改。
只依赖 Python 3 标准库 + 本机 git，不需要装第三方包。
"""
import io
import os
import re
import subprocess
import sys

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

ROOT = ""
SHOW_ALL = "--all" in sys.argv

# ---------------------------------------------------------------------------
# 规则表
# ---------------------------------------------------------------------------
# (正则, 归类名, 一句怎么改)
BAD_PATH_RULES = [
    (r"(^|/)MDK-ARM/Objects/",  "Keil 编译产物",        'git rm -r --cached MDK-ARM/Objects'),
    (r"(^|/)MDK-ARM/Listings/", "Keil 列表文件",        'git rm -r --cached MDK-ARM/Listings'),
    (r"(^|/)MDK-ARM/DebugConfig/", "Keil 调试配置",     'git rm -r --cached MDK-ARM/DebugConfig'),
    (r"/_pkg/",                 "安装包目录",           'git rm -r --cached _pkg'),
    (r"\.(exe|msi|iso)$",       "安装包/可执行文件",     "安装包放网盘，不入库"),
    (r"\.(o|axf|d|crf|dep|lnp|map|lst|iex|sct|bak)$", "编译产物", "编译产物不入库"),
    (r"\.uvguix\.",             "Keil 个人界面配置",    "个人配置不入库"),
    (r"(^|/)JLinkLog\.txt$",    "J-Link 日志",         "日志不入库"),
    (r"(^|/)JLinkSettings\.ini$", "J-Link 本地配置",    "本地配置不入库"),
    (r"\.log$",                 "日志文件",             "日志不入库"),
    (r"__pycache__/",           "Python 缓存",         "缓存不入库"),
    (r"\.pyc$",                 "Python 缓存",         "缓存不入库"),
    (r"(^|/)\.venv/",           "Python 虚拟环境",     "虚拟环境不入库"),
    (r"(^|/)(build[^/]*|dist)/", "打包/构建输出目录",    'git rm -r --cached dist'),
    (r"~[$].*\.(pptx|docx|xlsx|ppt|doc|xls)$", "Office 锁文件",
     "这是打开文件时生成的临时文件，关掉 Office 后删除并 git rm --cached 它"),
    (r"\.(pem|key|p12|pfx)$",   "私钥/证书",           "私钥绝不入库"),
    (r"(^|/)(id_rsa|id_ed25519)(\.|$)", "SSH 私钥",     "私钥绝不入库"),
]

MAX_FILE_MB = 20      # 超过就拦
WARN_FILE_MB = 10     # 超过只提醒

ALLOWED_TYPES = ["feat", "fix", "release", "hotfix", "refactor", "perf",
                 "docs", "test", "build", "chore", "hw", "revert"]
TYPE_HINT = " / ".join(ALLOWED_TYPES)

MSG_BAD_BODY = {"update", "updata", "modify", "change", "test",
                "修改", "更新", "改完", "改好了", "提交", "代码", "1", "111", "."}
MAX_SUBJECT_LEN = 50

BRANCH_RE = re.compile(
    r"^(main|master|dev"
    r"|release/v?\d+\.\d+(\.\d+)?"
    r"|(feature|fix|hotfix|test|hw)/[a-z0-9][a-z0-9\-]{1,40})$")

SECRET_PATTERNS = [
    (re.compile(r"-----BEGIN [A-Z ]*PRIVATE KEY-----"), "私钥内容"),
    (re.compile(r"(?i)\b(password|passwd|pwd)\s*[=:]\s*[\"'][^\"'\s]{4,}[\"']"), "明文密码"),
    (re.compile(r"(?i)\b(api[_-]?key|access[_-]?token|secret[_-]?key)\s*[=:]\s*[\"'][^\"'\s]{8,}[\"']"), "明文密钥"),
    (re.compile(r"(?i)\bsk-[A-Za-z0-9]{16,}"), "疑似 API 密钥"),
]

TEXT_EXT = {".c", ".h", ".s", ".py", ".ps1", ".bat", ".cmd", ".sh", ".html",
            ".js", ".json", ".md", ".txt", ".ini", ".uvprojx", ".uvoptx",
            ".gitignore", ".gitattributes", ".cfg", ".yml", ".yaml", ".iss"}

# ---------------------------------------------------------------------------
_errors = []   # (归类, 位置, 怎么改)
_warns = []
_ok = []


def git(*args):
    try:
        r = subprocess.run(["git"] + list(args), cwd=ROOT or None,
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        return r.stdout.decode("utf-8", "replace").strip()
    except Exception:
        return ""


def err(group, where, how=""):
    _errors.append((group, where, how))


def warn(group, where, how=""):
    _warns.append((group, where, how))


def ok(msg):
    _ok.append(msg)
    print("  [OK] " + msg)


# ---------------------------------------------------------------------------
def files_staged():
    return [x for x in git("diff", "--cached", "--name-only",
                           "--diff-filter=ACMR").splitlines() if x.strip()]


def files_tracked():
    return [x for x in git("ls-files").splitlines() if x.strip()]


def check_hygiene(files, label):
    print("\n[1/6] 不该进仓库的文件（%s，共 %d 个）" % (label, len(files)))
    n0 = len(_errors)
    for f in files:
        for pat, group, how in BAD_PATH_RULES:
            if re.search(pat, f, re.IGNORECASE):
                err(group, f, how)
                break
    if len(_errors) == n0:
        ok("没有编译产物 / 安装包 / 锁文件 / 密钥")


def check_sizes(files):
    print("\n[2/6] 文件大小")
    n0 = len(_errors) + len(_warns)
    for f in files:
        p = os.path.join(ROOT, f)
        if not os.path.isfile(p):
            continue
        mb = os.path.getsize(p) / 1024.0 / 1024.0
        if mb > MAX_FILE_MB:
            err("大文件（>%dMB）" % MAX_FILE_MB, f, "放网盘/制品库；确需版本管理就用 Git LFS")
        elif mb > WARN_FILE_MB:
            warn("文件偏大（>%dMB）" % WARN_FILE_MB, "%s (%.1fMB)" % (f, mb), "确认是否必须入库")
    if len(_errors) + len(_warns) == n0:
        ok("没有超过 %dMB 的文件" % MAX_FILE_MB)


def check_secrets(files):
    print("\n[3/6] 密码 / 私钥")
    n0 = len(_errors)
    for f in files:
        if os.path.splitext(f)[1].lower() not in TEXT_EXT:
            continue
        p = os.path.join(ROOT, f)
        if not os.path.isfile(p) or os.path.getsize(p) > 512 * 1024:
            continue
        try:
            with io.open(p, "r", encoding="utf-8", errors="ignore") as fh:
                for no, line in enumerate(fh, 1):
                    for rx, desc in SECRET_PATTERNS:
                        if rx.search(line):
                            err("疑似" + desc, "%s:%d" % (f, no),
                                "删掉明文，改占位符（例如 #define WIFI_PASS \"\"）")
                            break
        except Exception:
            pass
    if len(_errors) == n0:
        ok("没有明文密码 / 私钥")


def check_message(msg):
    print("\n[4/6] 提交信息")
    lines = [x for x in msg.replace("\r\n", "\n").split("\n") if x.strip()]
    if not lines:
        err("提交信息格式", "(空)", "写一句：feat(模块): 做了什么")
        return
    subj = lines[0].strip()
    # 合并 / 回滚 / 压缩提交由 git 自动生成，不按格式要求
    if re.match(r"^(Merge|Revert)\b", subj) or subj.startswith(("fixup!", "squash!")):
        ok("提交信息（由 git 自动生成，跳过格式检查）：" + subj)
        return
    m = re.match(r"^([a-zA-Z]+)(\([a-zA-Z0-9_\-]+\))?(!)?: (.+)$", subj)
    if not m:
        err("提交信息格式", subj,
            "格式：类型(模块): 说明    例如 feat(adc64): 增加 FPC12 原始值读取")
        return
    t = m.group(1).lower()
    if t not in ALLOWED_TYPES:
        err("提交类型不在白名单", t, "只能用：" + TYPE_HINT)
        return
    body = m.group(4).strip()
    if len(body) < 2 or body.lower() in MSG_BAD_BODY:
        err("提交说明没有信息量", subj, "写具体：feat(adc64): 增加 FPC12 原始值读取")
        return
    if len(subj) > MAX_SUBJECT_LEN:
        err("首行太长", "%s（%d 字）" % (subj, len(subj)),
            "首行只写结论，控制在 %d 字内，细节放正文" % MAX_SUBJECT_LEN)
        return
    if subj.endswith("。"):
        err("首行句号结尾", subj, "去掉句号")
        return
    ok("提交信息合规：" + subj)


def check_branch(name):
    print("\n[5/6] 分支名")
    if not name or name == "HEAD":
        err("分支名", "(读不到当前分支)", "确认在仓库里、不是 detached HEAD")
        return
    if BRANCH_RE.match(name):
        ok("分支名合规：" + name)
    else:
        err("分支名不符合规范", name,
            "只能用 main / dev / release/v2.3 / feature/xxx / fix/xxx / hotfix/xxx\n"
            "        小写字母+数字+连字符，不要中文、空格、人名")


def check_project(files):
    print("\n[6/6] 项目约定")
    n0 = len(_errors) + len(_warns)

    # (a) 新增的 .c/.h 必须已经加进 Keil 工程
    uv = []
    for dirpath, dirnames, names in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in (".git", "dist", "build")]
        for n in names:
            if n.endswith(".uvprojx"):
                uv.append(os.path.join(dirpath, n))
    if uv:
        blob = ""
        for p in uv:
            try:
                with io.open(p, "r", encoding="utf-8", errors="ignore") as fh:
                    blob += fh.read()
            except Exception:
                pass
        for f in files:
            # 只看 .c：Keil 工程文件里只列源文件，头文件是靠 include 路径找的，列不列都正常
            if not f.lower().endswith(".c"):
                continue
            if os.path.basename(f).lower() == "main.c":
                continue
            if os.path.exists(os.path.join(ROOT, f)) and os.path.basename(f) not in blob:
                warn("源文件没加进 Keil 工程", f,
                     "Keil 里右键 Group → Add Existing Files，加进去再提交")

    # (b) 中断服务程序里不能调延时
    for f in files:
        if not f.lower().endswith(".c"):
            continue
        p = os.path.join(ROOT, f)
        if not os.path.isfile(p):
            continue
        try:
            with io.open(p, "r", encoding="utf-8", errors="ignore") as fh:
                src = fh.read()
        except Exception:
            continue
        for m in re.finditer(r"void\s+(\w*(?:IRQHandler|SysTick_Handler))\s*\([^)]*\)", src):
            i = src.find("{", m.end())
            if i < 0:
                continue
            depth, j = 0, i
            while j < len(src):
                if src[j] == "{":
                    depth += 1
                elif src[j] == "}":
                    depth -= 1
                    if depth == 0:
                        break
                j += 1
            if re.search(r"board_delay_(us|ms)\s*\(", src[i:j]):
                err("中断里调用了延时", "%s → %s" % (f, m.group(1)),
                    "中断里不能延时（会拖垮 1kHz 时基），逻辑挪到主循环")

    if len(_errors) + len(_warns) == n0:
        ok("新增源文件已入工程；中断里没有延时")


def check_release(tag):
    print("\n[发版检查] 版本号三处一致")
    ver = ""
    p = os.path.join(ROOT, "Core", "Inc", "config.h")
    if os.path.isfile(p):
        with io.open(p, "r", encoding="utf-8", errors="ignore") as fh:
            m = re.search(r'#define\s+FW_VERSION\s+"([^"]+)"', fh.read())
            if m:
                ver = m.group(1)
    print("  config.h  FW_VERSION = %s" % (ver or "(没找到)"))
    print("  git tag              = %s" % (tag or "(没给)"))

    t = (tag or "").lstrip("vV")
    if tag and ver and t != ver:
        err("版本号不一致", "tag=%s  config.h=%s" % (tag, ver),
            "先把 config.h 改成 %s（单独一个 chore 提交），再打 tag" % t)
    elif tag and ver:
        ok("tag 与 FW_VERSION 一致")

    rp = os.path.join(ROOT, "README.md")
    if tag and os.path.isfile(rp):
        with io.open(rp, "r", encoding="utf-8", errors="ignore") as fh:
            txt = fh.read()
        if tag not in txt and ("v" + t) not in txt:
            warn("README 没记录这个版本", tag,
                 "在 README「版本历史」补一行：%s · 日期 · 变更摘要" % tag)
        else:
            ok("README 版本历史里有 " + tag)


# ---------------------------------------------------------------------------
def summary():
    print("\n" + "=" * 62)
    if not _errors:
        if _warns:
            print("结论：通过（%d 项提醒，不拦提交）" % len(_warns))
            for group, where, how in (_warns if SHOW_ALL else _warns[:8]):
                print("  ! %s —— %s" % (group, where))
                if how:
                    print("      %s" % how)
            if not SHOW_ALL and len(_warns) > 8:
                print("  …… 还有 %d 项（加 --all 看全部）" % (len(_warns) - 8))
        else:
            print("结论：全部通过 ✔")
        print("=" * 62)
        return 0

    # 按问题归类
    groups = {}
    for group, where, how in _errors:
        groups.setdefault((group, how), []).append(where)

    total = sum(len(v) for v in groups.values())
    print("结论：发现 %d 个问题，分成 %d 类" % (total, len(groups)))
    print("-" * 62)
    for idx, ((group, how), wheres) in enumerate(groups.items(), 1):
        print("%d) %s   × %d 个" % (idx, group, len(wheres)))
        show = wheres if SHOW_ALL else wheres[:3]
        for w in show:
            print("     例：%s" % w)
        if not SHOW_ALL and len(wheres) > 3:
            print("     …… 还有 %d 个同类（加 --all 看全部）" % (len(wheres) - 3))
        if how:
            print("     怎么改：%s" % how)
        print("")
    print("=" * 62)
    print("有错误，先改完再提交。")
    return 1


def main(argv):
    global ROOT
    ROOT = git("rev-parse", "--show-toplevel")
    if not ROOT:
        print("不在 git 仓库里（或者没装 git）")
        return 1
    ROOT = ROOT.replace("/", os.sep)

    if "--staged" in argv:
        files = files_staged()
        print("仓库：%s" % ROOT)
        if not files:
            print("暂存区是空的，没东西要查")
            return 0
        print("本次要提交 %d 个文件" % len(files))
        check_hygiene(files, "暂存区")
        check_sizes(files)
        check_secrets(files)
        check_project(files)
        return summary()

    if "--message" in argv:
        i = argv.index("--message")
        arg = argv[i + 1] if i + 1 < len(argv) else ""
        if arg and os.path.isfile(arg):          # 钩子传的是文件路径
            with io.open(arg, "r", encoding="utf-8", errors="ignore") as fh:
                arg = fh.read()
        print("仓库：%s" % ROOT)
        check_message(arg)
        return summary()

    if "--branch" in argv:
        i = argv.index("--branch")
        name = argv[i + 1] if i + 1 < len(argv) else git("rev-parse", "--abbrev-ref", "HEAD")
        print("仓库：%s" % ROOT)
        check_branch(name)
        return summary()

    if "--release" in argv:
        i = argv.index("--release")
        print("仓库：%s" % ROOT)
        check_release(argv[i + 1] if i + 1 < len(argv) else "")
        return summary()

    # -- 云效流水线模式：检查"相对基线分支的本次改动" --
    if "--ci" in argv:
        i = argv.index("--ci")
        base = argv[i + 1] if i + 1 < len(argv) else "origin/main"
        print("仓库：%s" % ROOT)
        print("流水线模式：对比基线 %s" % base)

        files = []
        head = git("rev-parse", "--short", "HEAD")
        # 先按 <base>...HEAD 比（合并请求里最准）
        if git("rev-parse", "--verify", "--quiet", base):
            out = git("diff", "--name-only", "--diff-filter=ACMR", "%s...HEAD" % base)
            files = [x for x in out.splitlines() if x.strip()]
            print("基线 %s 存在，本次改动 %d 个文件" % (base, len(files)))
        else:
            # 浅克隆里可能没有基线分支，退化成"最近一次提交"
            out = git("diff", "--name-only", "--diff-filter=ACMR", "HEAD~1", "HEAD")
            files = [x for x in out.splitlines() if x.strip()]
            print("!! 找不到基线 %s（可能需要 git fetch），退化为只查最近一次提交（%s）"
                  % (base, head))

        if not files:
            print("\n本次没有文件改动，跳过。")
            return 0

        check_hygiene(files, "本次改动")
        check_sizes(files)
        check_secrets(files)
        check_project(files)
        return summary()

    # 默认：整仓体检
    files = files_tracked()
    branch = git("rev-parse", "--abbrev-ref", "HEAD")
    print("仓库：%s" % ROOT)
    print("分支：%s   已跟踪文件：%d 个" % (branch or "(未知)", len(files)))

    print("\n[0/6] 基线文件")
    for name in (".gitignore", ".gitattributes"):
        if os.path.isfile(os.path.join(ROOT, name)):
            ok("有 " + name)
        else:
            err("缺少基线文件", name, "新仓初始化第一件事就是建它，漏了会把编译产物全传上去")

    check_branch(branch)
    check_hygiene(files, "已跟踪文件")
    check_sizes(files)
    check_secrets(files)
    check_project(files)
    return summary()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
