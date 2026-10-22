"""Exercise the real C tracking pipeline with simple synthetic grayscale roads."""

import subprocess

from simulate_photos import HEIGHT, RUNNER, WIDTH, build_runner


def run_case(name, road):
    image = bytearray(WIDTH * HEIGHT)
    for y, runs in enumerate(road):
        for left, right in runs:
            image[y * WIDTH + left:y * WIDTH + right + 1] = bytes([220]) * (right - left + 1)

    result = subprocess.run((str(RUNNER),), input=image, capture_output=True, check=True)
    assert len(result.stdout) == 4 * HEIGHT, name
    left = result.stdout[:HEIGHT]
    center = result.stdout[HEIGHT:2 * HEIGHT]
    right = result.stdout[2 * HEIGHT:3 * HEIGHT]
    lost = result.stdout[3 * HEIGHT:]
    for y in range(HEIGHT):
        for x in (left[y], center[y], right[y]):
            assert x == 255 or (x < WIDTH and image[y * WIDTH + x] >= 130), (name, y, x)
    return left, center, right, lost


def main():
    build_runner()

    # A white patch at x=0 is not the road under the camera center.
    separate = [[(0, 20), (60, 130)] for _ in range(HEIGHT)]
    left, _, right, _ = run_case("separate-left-white", separate)
    assert (left[119], right[119], left[40]) == (60, 130, 60)

    # The selected lane really reaches x=0, then curves away from it.
    curve = [[(max(0, 60 - y), 110)] for y in range(HEIGHT)]
    left, _, _, _ = run_case("border-then-curve", curve)
    assert (left[119], left[60], left[40], left[10]) == (0, 0, 20, 50)

    # A crossbar joins white regions; the driving boundary must not jump to x=0.
    cross = [[(60, 130)] for _ in range(HEIGHT)]
    for y in range(50, 56):
        cross[y] = [(0, WIDTH - 1)]
    left, _, right, _ = run_case("merged-crossbar", cross)
    assert all(59 <= left[y] <= 60 and 129 <= right[y] <= 130 for y in range(48, 58))

    # A real two-sided cross should keep the bridge's slope through the white bar.
    slanted = [[(75, 125) if y < 50 else (60, 130)] for y in range(HEIGHT)]
    for y in range(50, 56):
        slanted[y] = [(0, WIDTH - 1)]
    left, _, right, _ = run_case("slanted-crossbar", slanted)
    assert 60 < left[53] < 75 and 125 < right[53] < 130
    assert left[50] > left[55] and right[50] < right[55]

    # Even a full black row is a hard stop, never a license to bridge over black.
    blocked = [[(60, 130)] for _ in range(HEIGHT)]
    blocked[50] = []
    left, center, right, lost = run_case("black-break", blocked)
    assert all((left[y], center[y], right[y]) == (255, 255, 255) for y in range(51))
    assert all((lost[y] & 3) == 3 for y in range(51))

    print("5 white-lane guard cases passed")


if __name__ == "__main__":
    main()
