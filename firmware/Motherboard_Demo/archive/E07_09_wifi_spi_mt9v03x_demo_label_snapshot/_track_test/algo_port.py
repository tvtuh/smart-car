#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.c 赛道识别算法的等价 Python 移植，仅用于在 PC 上做数值验证。
与 user/main.c.new 中的函数一一对应：
    image_otsu_threshold() / image_binarize() / row_find_longest_run()
    row_is_cross_area()    / slope_median()      / track_find_boundary()
凡是 C 里出现的整型截断、位压缩、限幅，这里都用相同的算法复现。
"""

W = 188
H = 120
IMAGE_BYTES = (W + 7) // 8

# ---------------- 可调参数(与 C 保持一致) ----------------
TRACK_FIXED_THRESHOLD = 150
TRACK_THRESHOLD_MIN = 60
TRACK_THRESHOLD_MAX = 210
TRACK_THRESHOLD_OFFSET = 0
TRACK_MIN_CONTRAST = 25
TRACK_MIN_BLACK_PIXELS = 20
TRACK_MIN_WHITE_PIXELS = 20
TRACK_MIN_RUN_WIDTH = 8
TRACK_MAX_TRACK_WIDTH = 184
TRACK_START_OFFSET = 3
TRACK_SEED_SCAN_ROWS = 10
TRACK_SEED_MAX_ROW = 44
TRACK_WIN_BASE = 6
TRACK_WIN_EXTRA = 2
TRACK_WIN_MAX = 64
TRACK_MAX_CENTER_JUMP = 22
TRACK_MAX_EDGE_JUMP = 26
TRACK_MAX_SLOPE = 4
TRACK_LOST_LIMIT = 24
TRACK_CROSS_WHITE_PERCENT = 93
TRACK_CROSS_CONFIRM_ROWS = 1
TRACK_CROSS_MIN_WIDTH_RATIO = 118
TRACK_CROSS_EXIT_RATIO = 145
TRACK_CROSS_LOOKAHEAD = 8
TRACK_CROSS_LOOKAHEAD_RATIO = 150
TRACK_CROSS_MAX_JUMP = 8
TRACK_FAR_WIDTH_RATIO = 200
TRACK_FAR_WIDTH_MAX = TRACK_MAX_TRACK_WIDTH
TRACK_CROSS_BOTH_SIDES = 15
TRACK_OTSU_SAMPLE_ROW = 4
TRACK_OTSU_SAMPLE_COL = 3
TRACK_GRAY_SHIFT = 3
TRACK_GRAY_LEVELS = 256 >> TRACK_GRAY_SHIFT
TRACK_SLOPE_HIST = 3

TRACK_TYPE_NORMAL, TRACK_TYPE_CROSS, TRACK_TYPE_LOST = 0, 1, 2

u8 = lambda v: v & 0xFF
i16 = lambda v: ((v + 0x8000) & 0xFFFF) - 0x8000


def LIMIT(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class TrackAlgo:
    def __init__(self):
        self.image_copy = [[0] * W for _ in range(H)]
        self.image_bin = [[0] * IMAGE_BYTES for _ in range(H)]
        self.image_white_count = [0] * H
        self.x1 = [0] * H
        self.x2 = [0] * H
        self.x3 = [0] * H
        self.threshold = TRACK_FIXED_THRESHOLD
        self.width_est = 0
        self.cross_rows = 0
        self.left_lost = 0
        self.right_lost = 0
        self.track_type = TRACK_TYPE_NORMAL
        self.boundary_tmp = [0] * H
        self.m_prev_left = 0
        self.m_prev_right = 0
        self.m_seed_width = 0
        self.m_seed_row = 0
        self.m_left_pos_hist = [0] * TRACK_SLOPE_HIST
        self.m_right_pos_hist = [0] * TRACK_SLOPE_HIST
        self.m_hist_cnt = 0

    # ------------------------------------------------------------------
    def IMAGE_BIN_GET(self, row, col):
        return (self.image_bin[row][col >> 3] >> (col & 0x07)) & 0x01

    def boundary_median_filter(self, boundary):
        for i in range(H):
            self.boundary_tmp[i] = boundary[i]
        for i in range(1, H - 1):
            a, b, c = self.boundary_tmp[i - 1], self.boundary_tmp[i], self.boundary_tmp[i + 1]
            if a > b:
                a, b = b, a
            if b > c:
                b, c = c, b
            if a > b:
                a, b = b, a
            boundary[i] = b
        boundary[0] = self.boundary_tmp[0]
        boundary[H - 1] = self.boundary_tmp[H - 1]

    # ------------------------------------------------------------------
    def image_otsu_threshold(self):
        hist = [0] * TRACK_GRAY_LEVELS
        total = sum_g = sum_b = w_b = 0
        min_gray, max_gray = 255, 0
        for row in range(0, H, TRACK_OTSU_SAMPLE_ROW):
            for col in range(0, W, TRACK_OTSU_SAMPLE_COL):
                gray = self.image_copy[row][col]
                hist[gray >> TRACK_GRAY_SHIFT] += 1
                total += 1
                sum_g += gray
                if gray < min_gray:
                    min_gray = gray
                if gray > max_gray:
                    max_gray = gray
        if (max_gray - min_gray) < TRACK_MIN_CONTRAST:
            return TRACK_FIXED_THRESHOLD
        best_eta, threshold = 0, TRACK_FIXED_THRESHOLD
        for i in range(TRACK_GRAY_LEVELS):
            w_b += hist[i]
            if w_b == 0:
                continue
            w_f = total - w_b
            if w_f == 0:
                break
            sum_b += i * hist[i]
            mean_b = sum_b // w_b
            mean_f = (sum_g - sum_b) // w_f
            var_between = mean_b - mean_f if mean_b > mean_f else mean_f - mean_b
            eta = w_b * w_f * var_between * var_between
            if eta > best_eta:
                best_eta, threshold = eta, i
        black_cnt = sum(hist[:threshold + 1])
        white_cnt = total - black_cnt
        if black_cnt < TRACK_MIN_BLACK_PIXELS or white_cnt < TRACK_MIN_WHITE_PIXELS:
            return TRACK_FIXED_THRESHOLD
        threshold = threshold << TRACK_GRAY_SHIFT
        threshold = LIMIT(threshold, TRACK_THRESHOLD_MIN, TRACK_THRESHOLD_MAX)
        threshold = threshold + TRACK_THRESHOLD_OFFSET
        threshold = LIMIT(threshold, 1, 254)
        return threshold

    # ------------------------------------------------------------------
    def image_binarize(self, threshold):
        """纵向 3 点中值滤波 + 二值化 + 位压缩 + 白点统计（与 C 版一致）"""
        line = [self.image_copy[0][:], self.image_copy[0][:], self.image_copy[0][:]]
        b0, b1, b2 = 0, 1, 2
        for row in range(H):
            if row > 0:
                if row + 1 <= H - 1:
                    line[b2] = self.image_copy[row + 1][:]
                else:
                    line[b2] = self.image_copy[row][:]
            white_cnt, bits, bit_mask = 0, 0, 0x01
            for col in range(W):
                a, b, c = line[b0][col], line[b1][col], line[b2][col]
                if a > b:
                    a, b = b, a
                if b > c:
                    b, c = c, b
                if a > b:
                    a, b = b, a
                mid = b
                if mid >= threshold:
                    bits |= bit_mask
                    white_cnt += 1
                bit_mask = u8(bit_mask << 1)
                if bit_mask == 0:
                    self.image_bin[row][col >> 3] = u8(bits)
                    bits, bit_mask = 0, 0x01
            if bit_mask != 0x01:
                self.image_bin[row][(W - 1) >> 3] = u8(bits)
            self.image_white_count[row] = white_cnt
            b0, b1, b2 = b1, b2, b0

    # ------------------------------------------------------------------
    def row_find_longest_run(self, row, col_start, col_end):
        if col_start < 0:
            col_start = 0
        if col_end > W - 1:
            col_end = W - 1
        if col_start > col_end:
            return 0, -1, -1
        start = -1
        best_start = best_end = -1
        best_len = cur_len = 0
        prev_white = 0
        for col in range(col_start, col_end + 1):
            white = self.IMAGE_BIN_GET(row, col)
            if white:
                if not prev_white:
                    start, cur_len = col, 0
                cur_len += 1
                if cur_len > best_len:
                    best_len, best_start, best_end = cur_len, start, col
            prev_white = white
        return best_len, best_start, best_end

    def row_is_cross_area(self, row):
        return 1 if (self.image_white_count[row] * 100) >= (W * TRACK_CROSS_WHITE_PERCENT) else 0

    def cross_lookahead(self, row, normal_width=None):
        """十字判定：当前行是否是横向赛道(白段)的下沿——下方若干行都不比它更宽"""
        cur = self.image_white_count[row]
        for i in range(1, TRACK_CROSS_MAX_JUMP + 1):
            r = row + i
            if r > H - 1:
                break
            if self.image_white_count[r] >= cur:
                return 0
        return 1

    def cross_expected_width(self, row, seed_row=None, seed_width=None):
        """透视宽度上限：该行正常赛道最多能有多宽"""
        if self.m_seed_row <= 0:
            return TRACK_MAX_TRACK_WIDTH
        dist = self.m_seed_row - row
        if dist <= 0:
            return TRACK_MAX_TRACK_WIDTH
        far_width = (self.m_seed_width * TRACK_FAR_WIDTH_RATIO) // 100
        far_width = (far_width * dist) // self.m_seed_row
        if far_width < self.m_seed_width:
            far_width = self.m_seed_width
        if far_width > TRACK_MAX_TRACK_WIDTH:
            far_width = TRACK_MAX_TRACK_WIDTH
        return far_width

    # ------------------------------------------------------------------
    def slope_median(self, prev, hist):
        d = [i16(prev - hist[0]) if i == 0 else i16(hist[i - 1] - hist[i]) for i in range(TRACK_SLOPE_HIST)]
        d.sort()
        return d[TRACK_SLOPE_HIST // 2]

    # ------------------------------------------------------------------
    def track_find_boundary(self):
        self.cross_rows = 0
        self.track_type = TRACK_TYPE_NORMAL
        self.left_lost = 0
        self.right_lost = 0
        lost_cnt = 0
        found = 0
        cross_flag = 0
        cross_confirm = 0
        cross_row_cnt = 0
        self.m_hist_cnt = 0

        for row in range(H):
            self.x1[row], self.x2[row], self.x3[row] = 0, W // 2, W - 1

        # 1.1 在下方 TRACK_SEED_SCAN_ROWS 行里挑最长白段最长的一行作为基准
        found = 0
        best_run_len = 0
        best = (0, -1, -1)
        row = H - 1 - TRACK_START_OFFSET
        for _ in range(TRACK_SEED_SCAN_ROWS):
            if row < TRACK_SEED_MAX_ROW:
                break
            rl, rs, re = self.row_find_longest_run(row, 0, W - 1)
            if TRACK_MIN_RUN_WIDTH <= rl <= TRACK_MAX_TRACK_WIDTH and (not found or rl > best_run_len):
                found = 1
                best_run_len = rl
                best = (row, rs, re)
            row -= 1
        # 1.2 下方找不到就继续往上找
        if not found:
            row = H - 1 - TRACK_START_OFFSET
            while row >= 0:
                rl, rs, re = self.row_find_longest_run(row, 0, W - 1)
                if TRACK_MIN_RUN_WIDTH <= rl <= TRACK_MAX_TRACK_WIDTH:
                    found = 1
                    best_run_len = rl
                    best = (row, rs, re)
                    break
                row -= 1
        if not found:
            self.track_type = TRACK_TYPE_LOST
            self.width_est = 0
            return

        seed_row, run_start, run_end = best
        run_len = best_run_len
        self.m_seed_row = seed_row
        left, right = run_start, run_end
        for i in range(seed_row + 1, H):
            self.x1[i] = u8(left)
            self.x2[i] = u8((left + right) // 2)
            self.x3[i] = u8(right)
        width = LIMIT(run_len, TRACK_MIN_RUN_WIDTH, TRACK_MAX_TRACK_WIDTH)
        self.width_est = width
        self.m_seed_width = width
        for i in range(TRACK_SLOPE_HIST):
            self.m_left_pos_hist[i] = left
            self.m_right_pos_hist[i] = right
        self.m_hist_cnt = TRACK_SLOPE_HIST
        self.m_prev_left, self.m_prev_right = left, right

        for row in range(seed_row, -1, -1):
            # 2.1 预测
            if self.m_hist_cnt >= TRACK_SLOPE_HIST:
                left_slope = self.slope_median(self.m_prev_left, self.m_left_pos_hist)
                right_slope = self.slope_median(self.m_prev_right, self.m_right_pos_hist)
            else:
                left_slope = right_slope = 0
            left_slope = LIMIT(left_slope, -TRACK_MAX_SLOPE, TRACK_MAX_SLOPE)
            right_slope = LIMIT(right_slope, -TRACK_MAX_SLOPE, TRACK_MAX_SLOPE)
            left_pred = self.m_prev_left + left_slope
            right_pred = self.m_prev_right + right_slope

            # 2.2 十字判定
            run_len, run_start, run_end = self.row_find_longest_run(row, 0, W - 1)
            if (self.row_is_cross_area(row)
                    and run_len > TRACK_MIN_RUN_WIDTH
                    and (self.m_prev_left - run_start) > TRACK_CROSS_BOTH_SIDES
                    and (run_end - self.m_prev_right) > TRACK_CROSS_BOTH_SIDES
                    and self.cross_lookahead(row)):
                if cross_confirm < TRACK_CROSS_CONFIRM_ROWS:
                    cross_confirm += 1
            elif not cross_flag:
                cross_confirm = 0
            if not cross_flag and cross_confirm >= TRACK_CROSS_CONFIRM_ROWS:
                cross_flag = 1
                cross_row_cnt = 0

            if cross_flag:
                # 2.3 十字外推补线
                left = i16(LIMIT(self.m_prev_left + left_slope, 0, W - 1))
                right = i16(LIMIT(self.m_prev_right + right_slope, 0, W - 1))
                if left >= right:
                    center = (left + right) // 2
                    left = center - TRACK_MIN_RUN_WIDTH // 2
                    right = center + TRACK_MIN_RUN_WIDTH // 2
                if (run_len >= TRACK_MIN_RUN_WIDTH
                        and run_len * 100 < width * TRACK_CROSS_EXIT_RATIO):
                    cross_flag = 0
                    cross_confirm = 0
                    if run_start < self.m_prev_left + TRACK_MAX_EDGE_JUMP:
                        left = run_start
                    if run_end > self.m_prev_right - TRACK_MAX_EDGE_JUMP:
                        right = run_end
                else:
                    cross_row_cnt += 1
                    if cross_row_cnt > TRACK_LOST_LIMIT or run_len < TRACK_MIN_RUN_WIDTH:
                        cross_flag = 0
                        cross_confirm = 0
                        self.track_type = TRACK_TYPE_LOST
            else:
                # 2.4 正常区域
                win = LIMIT(TRACK_WIN_BASE + lost_cnt * TRACK_WIN_EXTRA, TRACK_WIN_BASE, TRACK_WIN_MAX)
                run_len, run_start, run_end = self.row_find_longest_run(row, left_pred - win, left_pred + win)
                left_ok = 1
                if run_len < TRACK_MIN_RUN_WIDTH:
                    left_ok = 0
                elif run_start > left_pred + TRACK_MAX_EDGE_JUMP or run_start < left_pred - TRACK_MAX_EDGE_JUMP:
                    left_ok = 0
                if left_ok:
                    left = run_start
                else:
                    left = left_pred
                    self.left_lost += 1

                run_len, run_start, run_end = self.row_find_longest_run(row, right_pred - win, right_pred + win)
                right_ok = 1
                if run_len < TRACK_MIN_RUN_WIDTH:
                    right_ok = 0
                elif run_end > right_pred + TRACK_MAX_EDGE_JUMP or run_end < right_pred - TRACK_MAX_EDGE_JUMP:
                    right_ok = 0
                if right_ok:
                    right = run_end
                else:
                    right = right_pred
                    self.right_lost += 1

                lost_cnt = max(self.left_lost, self.right_lost)
                lost_cnt = min(lost_cnt, TRACK_LOST_LIMIT)

                if TRACK_MIN_RUN_WIDTH < (right - left) < TRACK_MAX_TRACK_WIDTH:
                    width = (width * 3 + (right - left)) // 4
                    # 用透视上限约束，避免进入十字前那几行宽白段把宽度估计抬高
                    width = LIMIT(width, TRACK_MIN_RUN_WIDTH, self.cross_expected_width(row))
                else:
                    right = left + width
                    if right > W - 1:
                        right = W - 1

            # 2.5 保护与输出
            left = LIMIT(left, 0, W - 1)
            right = LIMIT(right, 0, W - 1)
            if left >= right:
                center = (left + right) // 2
                left = LIMIT(center - TRACK_MIN_RUN_WIDTH // 2, 0, W - 1)
                right = LIMIT(center + TRACK_MIN_RUN_WIDTH // 2, 0, W - 1)

            self.x1[row] = u8(left)
            self.x3[row] = u8(right)
            self.x2[row] = u8((left + right) // 2)
            # 只统计"进入十字补线状态"的行，不能用 row_is_cross_area 统计
            if cross_flag:
                self.cross_rows += 1

            if (not cross_flag) or run_len <= TRACK_MAX_TRACK_WIDTH:
                if self.m_hist_cnt >= TRACK_SLOPE_HIST:
                    for i in range(TRACK_SLOPE_HIST - 1, 0, -1):
                        self.m_left_pos_hist[i] = self.m_left_pos_hist[i - 1]
                        self.m_right_pos_hist[i] = self.m_right_pos_hist[i - 1]
                else:
                    self.m_hist_cnt += 1
                self.m_left_pos_hist[0] = left
                self.m_right_pos_hist[0] = right

            self.m_prev_left, self.m_prev_right = left, right

        self.boundary_median_filter(self.x1)
        self.boundary_median_filter(self.x3)
        for row in range(H):
            self.x2[row] = u8((self.x1[row] + self.x3[row]) // 2)

        if self.track_type != TRACK_TYPE_LOST:
            if self.cross_rows > 0 or self.left_lost > TRACK_LOST_LIMIT or self.right_lost > TRACK_LOST_LIMIT:
                self.track_type = TRACK_TYPE_CROSS
            else:
                self.track_type = TRACK_TYPE_NORMAL
        self.width_est = width

    # ------------------------------------------------------------------
    def process(self, image):
        self.image_copy = [r[:] for r in image]
        self.threshold = self.image_otsu_threshold()
        self.image_binarize(self.threshold)
        self.track_find_boundary()
        return self.x1, self.x2, self.x3


# ======================================================================
# 测试图像生成
# ======================================================================
import random


def make_image(kind, width=70, center=94, brightness=200, bg=30, noise=3, seed=1,
               band=(62, 74)):
    """生成与 C 端输入等价的 188x120 灰度图。
    注意：真实摄像头画面的噪点是空间相关的低频噪声，不是孤立椒盐点，
    所以这里的噪声幅度小且逐像素独立；椒盐噪点由 noisydots 单独测试。
    band 指定横向赛道所在的行区间。"""
    rnd = random.Random(seed)
    img = [[max(0, min(255, bg + rnd.randint(-noise, noise))) for _ in range(W)] for _ in range(H)]
    for row in range(H):
        t = row / (H - 1)                      # 0=最远 1=最近
        # 透视：越近越宽
        w_row = int(width * (0.45 + 0.55 * t))
        c_row = int(center)
        if kind in ('straight', 'noisydots'):
            pass
        elif kind == 'curve_left':             # 赛道向左弯
            c_row = int(center - 55 * (1 - t) ** 2)
        elif kind == 'curve_right':
            c_row = int(center + 55 * (1 - t) ** 2)
        elif kind == 's':
            c_row = int(center + 40 * ((1 - t) ** 2 - (1 - t)))
        for col in range(W):
            if kind == 'cross' and band[0] <= row <= band[1]:   # 横向赛道
                inside = True
            else:
                inside = abs(col - c_row) <= w_row // 2
            if inside:
                img[row][col] = max(0, min(255, brightness + rnd.randint(-noise, noise)))
    if kind == 'noisydots':                    # 赛道内部黑斑 + 背景白点(椒盐噪点)
        for _ in range(60):
            r = rnd.randrange(30, H)
            c = rnd.randrange(0, W)
            img[r][c] = 20
        for _ in range(120):
            r = rnd.randrange(0, H)
            c = rnd.randrange(0, W)
            img[r][c] = 235
    return img
