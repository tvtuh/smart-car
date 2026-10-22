"""Replay cropped assistant screenshots as grayscale proxy frames.

The screenshots already contain colored overlays, so these are regression
proxies, not raw MT9V03X camera frames.
"""

from pathlib import Path
import subprocess

from PIL import Image, ImageFilter

from simulate_photos import HEIGHT, OUTPUT, RUNNER, SCALE, WIDTH, build_runner


SCREENSHOTS = Path.home() / "Pictures" / "Screenshots"
CASES = (
    ("cross_far", "屏幕截图 2026-10-01 224753.png", (8, 49, 1045, 712), 7),
    ("cross_near", "屏幕截图 2026-10-01 224832.png", (54, 63, 1090, 725), 7),
    ("cross_mid", "屏幕截图 2026-10-01 225614.png", (48, 50, 688, 449), 5),
)


def run_case(name, filename, crop, median_size):
    camera = (Image.open(SCREENSHOTS / filename).convert("RGB")
              .crop(crop).convert("L")
              .filter(ImageFilter.MedianFilter(median_size))
              .resize((WIDTH, HEIGHT), Image.Resampling.BOX))
    gray = camera.tobytes()
    result = subprocess.run((str(RUNNER),), input=gray, capture_output=True, check=True)
    assert len(result.stdout) == 4 * HEIGHT
    diagnostics = result.stderr.decode("ascii")
    if name == "cross_far":
        assert "reject_upper=1" in diagnostics, diagnostics
    elif name == "cross_near":
        assert "from_top=1" in diagnostics, diagnostics
    else:
        assert "two_end=1" in diagnostics, diagnostics
    left, center, right, lost = (
        result.stdout[i * HEIGHT:(i + 1) * HEIGHT] for i in range(4))
    black = [(y, x) for line in (left, center, right)
             for y, x in enumerate(line) if x < WIDTH and gray[y * WIDTH + x] < 130]
    assert not black, (name, black[:5])

    overlay = Image.merge("RGB", (camera, camera, camera))
    pixels = overlay.load()
    for y in range(HEIGHT):
        for x, color in ((left[y], (0, 255, 0)), (center[y], (255, 0, 0)),
                         (right[y], (255, 255, 0))):
            if x < WIDTH:
                pixels[x, y] = color
    OUTPUT.mkdir(exist_ok=True)
    camera.resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST).save(
        OUTPUT / f"{name}_proxy_gray.png")
    overlay.resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST).save(
        OUTPUT / f"{name}_proxy_terminal.png")
    print(name, diagnostics.strip())
    print("  y40/60/100/119:",
          [(left[y], center[y], right[y], lost[y]) for y in (40, 60, 100, 119)])
    return left, center, right


def main():
    build_runner()
    far, near, middle = (run_case(*case) for case in CASES)
    assert far[0][100] < 60, "distant cross left edge jumped to road center"
    assert all(abs(near[0][y] - near[0][y - 1]) <= 8 for y in range(46, HEIGHT)), (
        "bottom-white cross left edge jumped to a side patch")
    assert abs(near[0][119] - near[0][46]) <= 20, (
        "bottom-white cross did not follow the stable upper edge")
    assert near[2][60] > near[2][46] and near[2][119] == 255, (
        "right edge should follow its slope, then leave the field of view")
    assert middle[0][30] > middle[0][50] > middle[0][80]


if __name__ == "__main__":
    main()
