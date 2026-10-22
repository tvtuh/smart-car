"""Numeric simulation of the current right-edge and cross-bridge formulas.

Run: python _track_test/test_right_cross_sim.py
This uses synthetic edge rows, not a captured camera frame or MCU execution.
"""

from pathlib import Path
import sys
from unittest import TestCase, main


HEIGHT = 120
WIDTH = 188


def cdiv(a, b):
    return (abs(a) // abs(b)) * (1 if (a >= 0) == (b >= 0) else -1)


def right_predict(edge, lost, row):
    first = -1
    samples = []
    for y in range(row + 1, HEIGHT):
        if lost[y] or edge[y] <= 1 or edge[y] >= WIDTH - 2:
            if first >= 0:
                break
            continue
        if first < 0:
            first = y
            if first - row > 60:
                return None
        samples.append((y - first, edge[y]))
        if len(samples) == 12:
            break
    n = len(samples)
    if n < 8:
        return None
    sum_t = sum(t for t, _ in samples)
    sum_x = sum(x for _, x in samples)
    sum_tt = sum(t * t for t, _ in samples)
    sum_tx = sum(t * x for t, x in samples)
    denominator = n * sum_tt - sum_t * sum_t
    slope_q = cdiv((n * sum_tx - sum_t * sum_x) * 256, denominator)
    if abs(slope_q) > 3 * 256:
        return None
    base_q = cdiv(sum_x * 256 - slope_q * sum_t, n)
    if any(abs(x * 256 - (base_q + slope_q * t)) > 3 * 256 for t, x in samples):
        return None
    value = cdiv(base_q + slope_q * (row - first) + 128, 256)
    if not 2 <= value < WIDTH - 2 or abs(value - edge[first]) > 40:
        return None
    return value, slope_q


def fill_gap(edge, lost, a, b, bridge=None):
    edge = edge[:]
    xa = edge[a - 1] if a > 0 else None
    xb = edge[b + 1] if b < HEIGHT - 1 else None
    if bridge is not None and xa is not None and xb is not None:
        prediction = right_predict(edge, lost, a)
        top, bottom = bridge
        if prediction is not None and top > 0 and bottom == a - 1 and abs(xa - prediction[0]) > 12:
            upper_y = top - 1
            projected_upper = xb + cdiv(prediction[1] * (upper_y - b - 1), 256)
            if b + 1 - upper_y <= 70 and not lost[upper_y] and abs(edge[upper_y] - projected_upper) <= 8:
                for y in range(top, a):
                    edge[y] = xb + cdiv(prediction[1] * (y - b - 1), 256)
                xa = None
    if xa is not None and xb is not None:
        for y in range(a, b + 1):
            edge[y] = xa + cdiv((xb - xa) * (y - a + 1), b - a + 2)
    elif xb is not None:
        trend = right_predict(edge, lost, a)
        if trend is None:
            slope_q = 0
            for y in range(b + 2, min(HEIGHT, b + 7)):
                if not lost[y]:
                    slope_q = cdiv(edge[y] - xb, y - b - 1) * 256
                    break
        else:
            slope_q = trend[1]
        for y in range(a, b + 1):
            edge[y] = max(0, min(WIDTH - 1, xb + cdiv(slope_q * (y - b - 1), 256)))
    return edge


def lower_slope(y):
    return 120 + (y + 1) // 2


class RightCrossSimulation(TestCase):
    def test_top_loss_follows_lower_slope(self):
        edge = [lower_slope(y) if y >= 56 else lower_slope(56) for y in range(HEIGHT)]
        lost = [y < 56 for y in range(HEIGHT)]
        result = fill_gap(edge, lost, 0, 55)
        self.assertEqual(right_predict(edge, lost, 0)[0], 120)
        self.assertEqual(result[0], 120)
        self.assertEqual(result[20], 130)
        print("top loss: y=0,20,55,56 ->", [result[y] for y in (0, 20, 55, 56)])

    def test_matching_upper_edge_joins_cross(self):
        edge = [lower_slope(y) if y < 25 or y >= 90 else 165 for y in range(HEIGHT)]
        lost = [35 <= y <= 89 for y in range(HEIGHT)]
        predicted = right_predict(edge, lost, 35)
        result = fill_gap(edge, lost, 35, 89, bridge=(25, 34))
        self.assertIsNotNone(predicted)
        self.assertLess(predicted[0], 145)
        self.assertEqual(result[35], 138)
        self.assertEqual(result[25], 132)
        self.assertLessEqual(abs(result[24] - result[25]), 1)
        self.assertLessEqual(abs(result[34] - result[35]), 1)
        self.assertLess(result[35], result[50])
        self.assertLess(result[50], result[70])
        print("matching cross: predicted y=35 ->", predicted[0],
              "; filled y=24,25,34,35,50,70,89 ->",
              [result[y] for y in (24, 25, 34, 35, 50, 70, 89)])

    def test_mismatched_upper_edge_is_not_overwritten(self):
        edge = [165 if y < 90 else lower_slope(y) for y in range(HEIGHT)]
        lost = [35 <= y <= 89 for y in range(HEIGHT)]
        result = fill_gap(edge, lost, 35, 89, bridge=(25, 34))
        self.assertEqual([result[y] for y in (25, 35, 50, 89)], [165] * 4)

    def test_normal_two_end_gap_keeps_interpolation(self):
        edge = [165 if y < 90 else lower_slope(y) for y in range(HEIGHT)]
        lost = [35 <= y <= 89 for y in range(HEIGHT)]
        result = fill_gap(edge, lost, 35, 89)
        self.assertEqual([result[y] for y in (35, 50, 70, 89)], [165] * 4)

    def test_cross_drift_cap_flattens_long_extrapolation(self):
        anchor = lower_slope(90)
        slope_q = cdiv((lower_slope(96) - anchor) * 256, 6)
        result = {}
        for y in (89, 70, 50, 35):
            shift = max(-20, min(20, cdiv(slope_q * (y - 90), 256)))
            result[y] = anchor + shift
        self.assertEqual(result[50], result[35])
        print("cross one-end bridge: y=89,70,50,35 ->",
              [result[y] for y in (89, 70, 50, 35)])


def save_plot(path):
    import matplotlib.pyplot as plt

    edge = [lower_slope(y) if y < 25 or y >= 90 else 165 for y in range(HEIGHT)]
    lost = [35 <= y <= 89 for y in range(HEIGHT)]
    before = fill_gap(edge, lost, 35, 89)
    after = fill_gap(edge, lost, 35, 89, bridge=(25, 34))
    fig, axes = plt.subplots(1, 2, figsize=(8, 5), sharex=True, sharey=True)
    for ax, values, title in zip(axes, (before, after), ("Before", "After")):
        ax.axhspan(25, 34, color="0.85", label="cross-bar region")
        ax.plot(values, range(HEIGHT), color="#e4ad00", linewidth=2.5)
        ax.scatter([values[35], values[89]], [35, 89], s=20, color="black")
        ax.set(title=title, xlim=(110, 185), ylim=(119, 0), xlabel="right-edge x (pixels)")
        ax.grid(alpha=0.2)
    axes[0].set_ylabel("image row (0 = top)")
    fig.suptitle("Synthetic cross-exit edge rows, not a camera frame")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    if "--plot" in sys.argv:
        sys.argv.remove("--plot")
        output = Path(__file__).with_name("right_cross_sim.png")
        save_plot(output)
        print("plot:", output)
    main(verbosity=2)
