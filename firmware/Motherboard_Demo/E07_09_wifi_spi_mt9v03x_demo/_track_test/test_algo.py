#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""赛道识别算法的数值验证：构造各种赛道图像，检查识别结果是否合理。
真值统一用算法本帧算出的阈值来二值化，保证对比公平。

运行：python test_algo.py
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from algo_port import *

PASS, FAIL = [], []


def check(name, cond, detail=''):
    (PASS if cond else FAIL).append(name)
    print(('  [ OK ] ' if cond else '  [FAIL] ') + name + (('  -> ' + detail) if detail else ''))


def truth_bounds(img, row, th):
    """真值：该行按同一阈值二值化后最长白段的范围"""
    best = (0, -1, -1)
    cur = 0
    start = -1
    for c in range(W + 1):
        white = c < W and img[row][c] >= th
        if white:
            if cur == 0:
                start = c
            cur += 1
            if cur > best[0]:
                best = (cur, start, c)
        else:
            cur = 0
    return best


def max_jump(arr, lo, hi):
    return max(abs(arr[i] - arr[i + 1]) for i in range(lo, hi))


def run_case(title, img, verbose_rows=(20, 50, 90, 115), show=True):
    a = TrackAlgo()
    t0 = time.time()
    x1, x2, x3 = a.process(img)
    dt = (time.time() - t0) * 1000
    if show:
        print('\n=== %s ===' % title)
        print('  threshold=%d width=%d cross_rows=%d type=%d L_lost=%d R_lost=%d (%.0f ms on PC)'
              % (a.threshold, a.width_est, a.cross_rows, a.track_type, a.left_lost, a.right_lost, dt))
        for r in verbose_rows:
            if r >= H:
                continue
            tb = truth_bounds(img, r, a.threshold)
            print('    row %3d : x1=%3d x2=%3d x3=%3d   | 真值 %s'
                  % (r, x1[r], x2[r], x3[r], ('(%d..%d w=%d)' % (tb[1], tb[2], tb[0])) if tb[0] else '无白色'))
    return a


def base_checks(title, a, img):
    bad_order = [r for r in range(H) if a.x1[r] >= a.x3[r]]
    check('%s: 左边界 < 右边界' % title, not bad_order, '异常行=%s' % bad_order[:5])
    bad_mid = [r for r in range(H) if not (a.x1[r] <= a.x2[r] <= a.x3[r])]
    check('%s: 中线在左右之间' % title, not bad_mid, '异常行=%s' % bad_mid[:5])
    mj = max_jump(a.x2, 0, H - 1)
    check('%s: 中线平滑' % title, mj <= 12, 'max jump=' + str(mj))


print('################ 赛 道 识 别 验 证 ################')

# ---------------- 1. 直道 ----------------
img = make_image('straight')
a = run_case('直道', img)
base_checks('直道', a, img)
for r in (60, 90, 115):
    tb = truth_bounds(img, r, a.threshold)
    check('直道 row%d 左边线误差<=3 (真值%d 实测%d)' % (r, tb[1], a.x1[r]), abs(a.x1[r] - tb[1]) <= 3)
    check('直道 row%d 右边线误差<=3 (真值%d 实测%d)' % (r, tb[2], a.x3[r]), abs(a.x3[r] - tb[2]) <= 3)

# ---------------- 2. 左弯 / 右弯 ----------------
for kind, nm, direction in (('curve_left', '左弯', -1), ('curve_right', '右弯', +1)):
    img = make_image(kind)
    a = run_case(nm, img)
    base_checks(nm, a, img)
    errs = []
    for r in range(20, 100, 5):
        tb = truth_bounds(img, r, a.threshold)
        if tb[0] >= 12:
            errs.append(abs(a.x1[r] - tb[1]))
            errs.append(abs(a.x3[r] - tb[2]))
    avg = sum(errs) / len(errs)
    check('%s: 边线平均误差<=4px (%.1f)' % (nm, avg), avg <= 4)
    far_c, near_c = a.x2[10], a.x2[110]
    check('%s: 远端中线偏向%s (%d vs %d)' % (nm, nm[0], far_c, near_c),
          (far_c < near_c) if direction < 0 else (far_c > near_c))

# ---------------- 3. 十字 ----------------
img = make_image('cross')
a = run_case('十字（行 62-74 为横向赛道）', img)
check('十字: 识别到十字 (cross_rows=%d)' % a.cross_rows, a.cross_rows > 0)
mj = max_jump(a.x2, 45, 95)
check('十字: 十字区域中线连续 (max jump=%d)' % mj, mj <= 12)
tb = truth_bounds(img, 100, a.threshold)
check('十字: 十字下方边线正常 (实测%d,%d 真值%d,%d)' % (a.x1[100], a.x3[100], tb[1], tb[2]),
      abs(a.x1[100] - tb[1]) <= 4 and abs(a.x3[100] - tb[2]) <= 4)
tb = truth_bounds(img, 30, a.threshold)
check('十字: 十字上方边线也正常 (实测%d,%d 真值%d,%d)' % (a.x1[30], a.x3[30], tb[1], tb[2]),
      abs(a.x1[30] - tb[1]) <= 5 and abs(a.x3[30] - tb[2]) <= 5)

# ---------------- 4. 斜入十字 (车不正对) ----------------
img = make_image('cross', center=70)
a = run_case('十字(偏左)', img)
check('十字(偏左): 识别到十字 (cross_rows=%d)' % a.cross_rows, a.cross_rows > 0)
base_checks('十字(偏左)', a, img)

# ---------------- 4b. 十字位置变化 ----------------
for band, nm in (((40, 50), '十字在远处'), ((90, 100), '十字在近处'),
                 ((100, 108), '十字很靠近车'), ((20, 26), '十字在很远')):
    img = make_image('cross', band=band)
    a = run_case('十字位置: %s (行%d-%d)' % (nm, band[0], band[1]), img,
                 verbose_rows=(band[1] - 5, band[1] + 3))
    check('%s: 识别到十字 (cross_rows=%d)' % (nm, a.cross_rows), a.cross_rows > 0)
    base_checks(nm, a, img)

# ---------------- 4c. 弯道中带十字 ----------------
img = make_image('cross', center=120, band=(62, 74))
a = run_case('右弯中带十字', img)
check('右弯中带十字: 识别到十字 (cross_rows=%d)' % a.cross_rows, a.cross_rows > 0)
base_checks('右弯中带十字', a, img)

# ---------------- 4d. S 弯 ----------------
img = make_image('s')
a = run_case('S 弯', img)
base_checks('S 弯', a, img)
errs = []
for r in range(20, 100, 5):
    tb = truth_bounds(img, r, a.threshold)
    if tb[0] >= 12:
        errs.append(abs(a.x1[r] - tb[1]))
        errs.append(abs(a.x3[r] - tb[2]))
avg = sum(errs) / len(errs)
check('S 弯: 边线平均误差<=4px (%.1f)' % avg, avg <= 4)

# ---------------- 5. 超宽直道(原算法会误判为十字) ----------------
img = make_image('straight', width=190, center=94)
a = run_case('超宽直道(远端占满整行)', img)
check('超宽直道: 不误判为十字 (cross_rows=%d)' % a.cross_rows, a.cross_rows == 0)
base_checks('超宽直道', a, img)

# ---------------- 6. 椒盐噪点(滤波效果) ----------------
img = make_image('noisydots', seed=7, noise=5)
a = run_case('赛道内黑斑 + 背景白点(椒盐噪点)', img)
base_checks('椒盐噪点', a, img)
errs = []
for r in range(20, 116, 10):
    tb = truth_bounds(img, r, a.threshold)
    if tb[0] >= 12:
        errs.append(abs(a.x1[r] - tb[1]))
        errs.append(abs(a.x3[r] - tb[2]))
avg = sum(errs) / len(errs)
check('椒盐噪点: 边线平均误差<=4px (%.1f)' % avg, avg <= 4)

# ---------------- 7. 低对比度 / 全黑 / 全白 ----------------
img = [[30] * W for _ in range(H)]
a = run_case('全黑(无赛道)', img)
check('全黑: 类型=LOST (%d)' % a.track_type, a.track_type == TRACK_TYPE_LOST)
check('全黑: 输出贴边直线', all(a.x1[r] == 0 and a.x3[r] == W - 1 for r in range(H)))

img = [[240] * W for _ in range(H)]
a = run_case('全白(整幅都是赛道)', img)
check('全白: 不崩且边线合法', all(a.x1[r] < a.x3[r] for r in range(H)))

img = make_image('straight', brightness=110, bg=95)     # 中值滤波后对比度约 15
a = run_case('极低对比度', img)
check('低对比度: 退回固定阈值 (%d)' % a.threshold, a.threshold == TRACK_FIXED_THRESHOLD)

# ---------------- 8. 光照整体变化(固定阈值的死穴) ----------------
for bright, bgv, nm in ((90, 20, '暗光'), (250, 120, '强光'), (140, 40, '中光')):
    img = make_image('straight', brightness=bright, bg=bgv)
    a = run_case('光照变化: %s (赛道%d/背景%d)' % (nm, bright, bgv), img)
    if a.track_type == TRACK_TYPE_LOST:
        check('%s: 不应该丢线' % nm, False, 'type=LOST th=%d' % a.threshold)
        continue
    errs = []
    for r in range(40, 116, 10):
        tb = truth_bounds(img, r, a.threshold)
        if tb[0] >= 12:
            errs.append(abs(a.x2[r] - (tb[1] + tb[2]) // 2))
    if errs:
        avg = sum(errs) / len(errs)
        check('%s: 中线平均误差<=4px (%.1f, th=%d)' % (nm, avg, a.threshold), avg <= 4)

# ---------------- 9. 窄赛道 ----------------
img = make_image('straight', width=40)
a = run_case('窄赛道(宽40)', img)
base_checks('窄赛道', a, img)
tb = truth_bounds(img, 100, a.threshold)
check('窄赛道: 边线正确 (实测%d,%d 真值%d,%d)' % (a.x1[100], a.x3[100], tb[1], tb[2]),
      abs(a.x1[100] - tb[1]) <= 3 and abs(a.x3[100] - tb[2]) <= 3)

print('\n================ 汇总 ================')
print('通过 %d 项, 失败 %d 项' % (len(PASS), len(FAIL)))
if FAIL:
    print('失败项:')
    for f in FAIL:
        print('  - ' + f)
    sys.exit(1)
print('全部通过')
