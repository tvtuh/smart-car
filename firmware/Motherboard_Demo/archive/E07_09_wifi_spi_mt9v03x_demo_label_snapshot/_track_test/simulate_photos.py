"""Run the firmware's edge/cross C pipeline on proxy camera views.

The input is a crop of each phone photo, not an MT9V03X frame. The result is
single-frame only; ring processing is disabled like in the firmware.
"""

from pathlib import Path
import shutil
import subprocess
import sys

from PIL import Image


WIDTH = 188
HEIGHT = 120
SCALE = 4
DOWNLOADS = Path.home() / "Downloads"
OUTPUT = Path(__file__).parent / "photo_sim"
RUNNER = Path.home() / "AppData/Local/Temp/codex_smartcar_build_verification/host_runner.exe"
PROJECT = Path(__file__).parent.parent
COMPILE_SOURCES = (PROJECT / "_track_test/host_runner.c", PROJECT / "code/track_edge.c",
                   PROJECT / "code/cross_road.c")
SOURCES = COMPILE_SOURCES + (PROJECT / "code/ring_road.c",)

CASES = (
    ("cross", "*20261001204051*237_180.jpg", (0, 80, 1280, 900)),
    ("right_ring", "*20261001204053*238_180.jpg", (0, 190, 1707, 1280)),
    ("left_bend", "*20261001204054*239_180.jpg", (0, 140, 1280, 960)),
)


def build_runner():
    gcc = shutil.which("gcc") or "D:/Dev-Cpp/MinGW64/bin/gcc.exe"
    if RUNNER.is_file() and all(RUNNER.stat().st_mtime >= p.stat().st_mtime for p in SOURCES):
        return
    RUNNER.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run((str(gcc), "-std=c99", "-O0", "-finput-charset=GBK",
                    "-I", str(PROJECT / "_track_test/host_stub"),
                    "-I", str(PROJECT / "code"),
                    *(str(p) for p in COMPILE_SOURCES), "-o", str(RUNNER)), check=True)


def render(name, pattern, crop, force_corner=None):
    matches = sorted(DOWNLOADS.glob(pattern))
    cached = OUTPUT / f"{name}_gray.png"
    if len(matches) == 1:
        source = Image.open(matches[0]).convert("RGB")
        camera = source.crop(crop).resize((WIDTH, HEIGHT), Image.Resampling.BOX).convert("L")
    elif len(matches) == 0 and cached.is_file():
        camera = Image.open(cached).convert("L").resize((WIDTH, HEIGHT), Image.Resampling.NEAREST)
    else:
        raise RuntimeError(f"expected one photo or cached grayscale frame for {name}")
    command = (str(RUNNER),) if force_corner is None else (
        str(RUNNER), "--force-right-ring", str(force_corner))
    process = subprocess.run(command, input=camera.tobytes(),
                             capture_output=True, check=True)
    if len(process.stdout) != HEIGHT * 4:
        raise RuntimeError(f"{name}: unexpected C output length {len(process.stdout)}")

    left = process.stdout[0:HEIGHT]
    center = process.stdout[HEIGHT:2 * HEIGHT]
    right = process.stdout[2 * HEIGHT:3 * HEIGHT]
    lost = process.stdout[3 * HEIGHT:4 * HEIGHT]

    def longest_run(mask):
        best = (0, -1, -1)
        start = -1
        for y in range(HEIGHT + 1):
            active = y < HEIGHT and bool(lost[y] & mask)
            if active and start < 0:
                start = y
            elif not active and start >= 0:
                if y - start > best[0]:
                    best = (y - start, start, y - 1)
                start = -1
        return best

    overlay = Image.merge("RGB", (camera, camera, camera))
    pixels = overlay.load()
    for y in range(HEIGHT):
        for x, color in ((left[y], (0, 255, 0)),
                         (center[y], (255, 0, 0)),
                         (right[y], (255, 255, 0))):
            if 0 <= x < WIDTH:
                pixels[x, y] = color

    OUTPUT.mkdir(exist_ok=True)
    camera.resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST).save(
        OUTPUT / f"{name}_gray.png")
    overlay.resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST).save(
        OUTPUT / f"{name}_terminal.png")

    print(name, "crop=", crop, "threshold=130", process.stderr.decode("ascii").strip(),
          "lost_rows=", sum(bool(v & 3) for v in lost),
          "right_run=", longest_run(2))
    print("  rows 20/50/80/110 (left, center, right):",
          [(left[y], center[y], right[y]) for y in (20, 50, 80, 110)])


if __name__ == "__main__":
    build_runner()
    for case in CASES:
        render(*case)
    if "--force-right-ring" in sys.argv:
        _, ring_pattern, ring_crop = CASES[1]
        render("right_ring_forced_101", ring_pattern, ring_crop, force_corner=101)
        render("right_ring_forced_90", ring_pattern, ring_crop, force_corner=90)
