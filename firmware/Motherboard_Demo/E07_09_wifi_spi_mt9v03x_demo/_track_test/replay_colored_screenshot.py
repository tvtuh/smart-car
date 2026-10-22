"""Replay an assistant screenshot after removing its colored track overlay.

This is a proxy frame, not a raw MT9V03X capture. The removal mask is kept
separate from the input so the effect of the original red centerline is visible.
"""

from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw

from simulate_photos import HEIGHT, RUNNER, WIDTH, build_runner


SOURCE = Path.home() / "AppData/Local/Temp/codex-clipboard-4c06c15e-d5bd-4187-9ff4-9a1b84445bc2.png"
OUTPUT = Path(__file__).parent / "photo_sim" / "colored_screenshot_replay"
CROP = (15, 21, 969, 630)
SCALE = 4


def run_frame(gray, *args):
    result = subprocess.run((str(RUNNER), *args), input=gray.tobytes(),
                            capture_output=True, check=True)
    if len(result.stdout) != HEIGHT * 4:
        raise RuntimeError(f"Unexpected C output length: {len(result.stdout)}")
    lines = [result.stdout[i * HEIGHT:(i + 1) * HEIGHT] for i in range(4)]
    return lines, result.stderr.decode("ascii", errors="replace").strip()


def render(gray, lines):
    left, center, right, _ = lines
    rgb = np.repeat(gray[:, :, None], 3, axis=2)
    for y in range(HEIGHT):
        for x, color in ((left[y], (0, 255, 0)),
                         (center[y], (255, 0, 0)),
                         (right[y], (255, 255, 0))):
            if x < WIDTH:
                rgb[y, x] = color
    return Image.fromarray(rgb).resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST)


def main():
    build_runner()
    source = cv2.imread(str(SOURCE), cv2.IMREAD_COLOR)
    if source is None:
        raise FileNotFoundError(SOURCE)
    x0, y0, x1, y1 = CROP
    crop = source[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # The road is grayscale. Saturated pixels belong to the displayed lines.
    colored = ((hsv[:, :, 1] >= 55) & (hsv[:, :, 2] >= 85)).astype(np.uint8) * 255
    mask = cv2.dilate(colored, np.ones((3, 3), np.uint8), iterations=2)
    restored = cv2.inpaint(crop, mask, 5, cv2.INPAINT_TELEA)

    raw_gray = cv2.resize(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY),
                          (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    clean_gray = cv2.resize(cv2.cvtColor(restored, cv2.COLOR_BGR2GRAY),
                            (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)

    raw_lines, raw_diag = run_frame(raw_gray)
    clean_lines, clean_diag = run_frame(clean_gray)
    auto_lines, auto_diag = run_frame(clean_gray, "--auto-ring")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    Image.fromarray(mask).save(OUTPUT / "overlay_mask.png")
    Image.fromarray(clean_gray).resize((WIDTH * SCALE, HEIGHT * SCALE),
                                      Image.Resampling.NEAREST).save(OUTPUT / "clean_gray.png")
    raw_overlay = render(raw_gray, raw_lines)
    clean_overlay = render(clean_gray, clean_lines)
    raw_overlay.save(OUTPUT / "unremoved_terminal.png")
    clean_overlay.save(OUTPUT / "cleaned_terminal.png")
    render(clean_gray, auto_lines).save(OUTPUT / "auto_ring_terminal.png")

    sheet = Image.new("RGB", (WIDTH * SCALE * 3, HEIGHT * SCALE + 34), "white")
    cleaned = Image.fromarray(clean_gray).resize((WIDTH * SCALE, HEIGHT * SCALE),
                                                Image.Resampling.NEAREST).convert("RGB")
    for i, panel in enumerate((raw_overlay, cleaned, clean_overlay)):
        sheet.paste(panel, (i * WIDTH * SCALE, 34))
    labels = ("Overlay present", "Overlay removed", "C algorithm replay")
    draw = ImageDraw.Draw(sheet)
    for i, label in enumerate(labels):
        draw.text((i * WIDTH * SCALE + 8, 9), label, fill="black")
    sheet.save(OUTPUT / "comparison.png")

    masked_small = cv2.resize(mask, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA) > 64
    newly_white = int(np.count_nonzero(masked_small & (raw_gray < 130) & (clean_gray >= 130)))
    print(f"crop={CROP}, colored_mask_pixels={np.count_nonzero(mask)}, false_black_corrected={newly_white}")
    for name, lines, diag, gray in (("unremoved", raw_lines, raw_diag, raw_gray),
                                   ("cleaned", clean_lines, clean_diag, clean_gray),
                                   ("auto_ring", auto_lines, auto_diag, clean_gray)):
        left, center, right, lost = lines
        black = sum(1 for line in (left, center, right)
                    for y, x in enumerate(line) if x < WIDTH and gray[y, x] < 130)
        print(name, diag, "black_line_points=", black)
        print("  y20/40/60/80/100/119:",
              [(y, left[y], center[y], right[y], lost[y])
               for y in (20, 40, 60, 80, 100, 119)])
    print("comparison=", OUTPUT / "comparison.png")


if __name__ == "__main__":
    main()
