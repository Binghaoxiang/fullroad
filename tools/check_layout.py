from pptx import Presentation
from pptx.util import Emu
p = Presentation(r"激光通信自动对准系统_工作汇报.pptx")
SW, SH = p.slide_width, p.slide_height
bad = []
for i, s in enumerate(p.slides, 1):
    for sh in s.shapes:
        try:
            l, t, w, h = sh.left, sh.top, sh.width, sh.height
        except Exception:
            continue
        if l is None: continue
        if l < -Emu(10000) or t < -Emu(10000) or l + w > SW + Emu(10000) or t + h > SH + Emu(10000):
            bad.append((i, sh.shape_type, round(l/914400,2), round(t/914400,2),
                        round((l+w)/914400,2), round((t+h)/914400,2)))
print("slides:", len(p.slides.__iter__.__self__._sldIdLst))
print("slide size (in):", round(SW/914400,2), "x", round(SH/914400,2))
print("out-of-bounds shapes:", len(bad))
for b in bad[:12]:
    print("  ", b)
# 文字溢出粗估：单行字符数超过框宽的可疑项
print("\n--- 长文本块检查（>120 字符的文本框）---")
for i, s in enumerate(p.slides, 1):
    for sh in s.shapes:
        if sh.has_text_frame and len(sh.text) > 120:
            print("  slide %02d  %d chars  %.1f x %.1f in" %
                  (i, len(sh.text), sh.width/914400, sh.height/914400))
