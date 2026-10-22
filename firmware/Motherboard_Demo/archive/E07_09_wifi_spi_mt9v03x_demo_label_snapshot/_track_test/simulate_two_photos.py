"""Render two portrait track photos through the current host C pipeline.

Usage: python simulate_two_photos.py photo1.jpg photo2.jpg
These are phone-photo proxies, not MT9V03X frames; the crop is an assumption.
"""

from pathlib import Path
import subprocess
import sys

from PIL import Image

from simulate_photos import HEIGHT, OUTPUT, RUNNER, SCALE, WIDTH, build_runner


def run(name, path, top):
    crop = (0, top, 960, top + 613)
    source = Image.open(path).convert("RGB")
    camera = source.crop(crop).resize((WIDTH, HEIGHT), Image.Resampling.BOX).convert("L")
    result = subprocess.run((str(RUNNER),), input=camera.tobytes(),
                            capture_output=True, check=True)
    if len(result.stdout) != HEIGHT * 4:
        raise RuntimeError("unexpected host output size")

    left = result.stdout[:HEIGHT]
    center = result.stdout[HEIGHT:2 * HEIGHT]
    right = result.stdout[2 * HEIGHT:3 * HEIGHT]
    lost = result.stdout[3 * HEIGHT:]
    overlay = Image.merge("RGB", (camera, camera, camera))
    pixels = overlay.load()
    for y in range(HEIGHT):
        for x, color in ((left[y], (0, 255, 0)),
                         (center[y], (255, 0, 0)),
                         (right[y], (255, 255, 0))):
            if x < WIDTH:
                pixels[x, y] = color

    OUTPUT.mkdir(exist_ok=True)
    camera.resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST).save(
        OUTPUT / f"{name}_gray.png")
    overlay.resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST).save(
        OUTPUT / f"{name}_terminal.png")
    print(name, "crop=", crop, result.stderr.decode("ascii").strip())
    print("  rows 20/50/80/110 L/C/R/lost:",
          [(left[y], center[y], right[y], lost[y]) for y in (20, 50, 80, 110)])


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: python simulate_two_photos.py photo1.jpg photo2.jpg")
    build_runner()
    for name, path in zip(("curve_a", "curve_b"), map(Path, sys.argv[1:])):
        run(name, path, 250)
        run(name + "_near", path, 400)
