"""Check whether the one-frame classifications survive small crop changes."""

import subprocess

from PIL import Image

from simulate_photos import CASES, DOWNLOADS, HEIGHT, RUNNER, WIDTH, build_runner


build_runner()
for name, pattern, base in CASES:
    source = Image.open(next(DOWNLOADS.glob(pattern))).convert("RGB")
    for shift in (-40, 0, 40):
        crop = (base[0], base[1] + shift, base[2], base[3])
        gray = source.crop(crop).resize((WIDTH, HEIGHT), Image.Resampling.BOX).convert("L")
        process = subprocess.run((str(RUNNER),), input=gray.tobytes(),
                                 capture_output=True, check=True)
        print(name, "top_shift=", shift, process.stderr.decode("ascii").strip())
