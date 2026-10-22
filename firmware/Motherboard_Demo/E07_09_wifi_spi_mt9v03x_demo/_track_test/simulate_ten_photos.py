"""Replay ten phone photos through the current C edge/cross pipeline.

Each photo is a single-frame proxy, not an MT9V03X capture. All photos use
the same full-width crop and the same 130 gray-level threshold.
"""

from pathlib import Path
import argparse
import subprocess

from PIL import Image, ImageDraw

from simulate_photos import HEIGHT, RUNNER, WIDTH, build_runner


DOWNLOADS = Path.home() / "Downloads"
OUTPUT = Path(__file__).parent / "photo_sim" / "ten_photos_20261002"
SUFFIXES = (
    "250", "251", "252", "253", "254",
    "255", "256", "257", "258", "259",
)
SCALE = 4
CROP_TOP = 80


def source_for(suffix):
    matches = list(DOWNLOADS.glob(f"微信图片_20261002*_{suffix}_180.jpg"))
    if len(matches) != 1:
        raise RuntimeError(f"Expected one photo for {suffix}, got {len(matches)}")
    return matches[0]


def process(suffix):
    source = Image.open(source_for(suffix)).convert("RGB")
    width, height = source.size
    crop_height = round(width * HEIGHT / WIDTH)
    top = min(CROP_TOP, height - crop_height)
    crop = (0, top, width, top + crop_height)
    camera = source.crop(crop).resize((WIDTH, HEIGHT), Image.Resampling.BOX).convert("L")
    result = subprocess.run((str(RUNNER),), input=camera.tobytes(),
                            capture_output=True, check=True)
    if len(result.stdout) != HEIGHT * 4:
        raise RuntimeError(f"{suffix}: C returned {len(result.stdout)} bytes")

    left, center, right, lost = (
        result.stdout[i * HEIGHT:(i + 1) * HEIGHT] for i in range(4))
    pixels = camera.load()
    black_points = sum(1 for line in (left, center, right)
                       for y, x in enumerate(line) if x < WIDTH and pixels[x, y] < 130)

    overlay = Image.merge("RGB", (camera, camera, camera))
    drawn = overlay.load()
    for y in range(HEIGHT):
        for x, color in ((left[y], (0, 255, 0)),
                         (center[y], (255, 0, 0)),
                         (right[y], (255, 255, 0))):
            if x < WIDTH:
                drawn[x, y] = color

    OUTPUT.mkdir(parents=True, exist_ok=True)
    camera.resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST).save(
        OUTPUT / f"{suffix}_gray.png")
    panel = overlay.resize((WIDTH * SCALE, HEIGHT * SCALE), Image.Resampling.NEAREST)
    panel.save(OUTPUT / f"{suffix}_terminal.png")
    diagnostics = result.stderr.decode("ascii", errors="replace").strip().replace("\n", " ")
    print(suffix, "crop=", crop, diagnostics,
          "valid_left=", sum(x < WIDTH for x in left),
          "valid_right=", sum(x < WIDTH for x in right),
          "black_line_points=", black_points)
    return panel


def contact_sheet(name, items):
    padding = 8
    label_height = 26
    cell_width = WIDTH * SCALE
    cell_height = HEIGHT * SCALE + label_height
    sheet = Image.new("RGB", (cell_width * 2 + padding * 3,
                              cell_height * 3 + padding * 4), "white")
    draw = ImageDraw.Draw(sheet)
    for i, (suffix, panel) in enumerate(items):
        col, row = i % 2, i // 2
        x = padding + col * (cell_width + padding)
        y = padding + row * (cell_height + padding)
        draw.text((x + 4, y + 5), f"Photo {suffix}", fill="black")
        sheet.paste(panel, (x, y + label_height))
    sheet.save(OUTPUT / f"{name}_terminal_contact_sheet.png")


def main():
    global CROP_TOP, OUTPUT
    parser = argparse.ArgumentParser()
    parser.add_argument("--top", type=int, default=80)
    args = parser.parse_args()
    CROP_TOP = args.top
    if CROP_TOP != 80:
        OUTPUT = OUTPUT.parent / f"ten_photos_20261002_top{CROP_TOP}"
    build_runner()
    panels = [(suffix, process(suffix)) for suffix in SUFFIXES]
    contact_sheet("250_to_254", panels[:5])
    contact_sheet("255_to_259", panels[5:])
    print("output=", OUTPUT)


if __name__ == "__main__":
    main()
