"""Turns met_sim's PPMs into a PNG and an animated GIF for each case, and a contact sheet of them all.

    python3 render.py OUT

OUT/<case>/picture.ppm is the picture without the animations, f0000.ppm on the frames with them. Writes
OUT/<case>.png, OUT/<case>.gif and OUT/stills.png, all at twice the panel's size, then drops the PPMs, which
come to gigabytes.
"""

import shutil
import sys
from pathlib import Path

from PIL import Image, ImageDraw

SCALE = 2
COLS = 4


def big(im):
    return im.resize((im.width * SCALE, im.height * SCALE), Image.NEAREST)


def main(out):
    out = Path(out)
    stills = []
    for case in sorted(p for p in out.iterdir() if p.is_dir()):
        pic = Image.open(case / "picture.ppm").convert("RGB")
        big(pic).save(out / f"{case.name}.png")
        stills.append((case.name, pic))
        frames = sorted(case.glob("f*.ppm"))
        if not frames:
            shutil.rmtree(case)
            continue
        fps = int((case / "fps").read_text()) if (case / "fps").exists() else 10
        ims = [big(Image.open(f).convert("RGB")) for f in frames]
        ims[0].save(
            out / f"{case.name}.gif",
            save_all=True,
            append_images=ims[1:],
            duration=1000 // fps,
            loop=0,
            optimize=False,
        )
        shutil.rmtree(case)
    if not stills:
        return
    w, h = 320, 240
    rows = (len(stills) + COLS - 1) // COLS
    sheet = Image.new("RGB", (COLS * w, rows * (h + 14)), "black")
    draw = ImageDraw.Draw(sheet)
    for i, (name, pic) in enumerate(stills):
        x, y = i % COLS * w, i // COLS * (h + 14)
        sheet.paste(pic, (x, y + 14))
        draw.text((x + 4, y + 1), name, fill="white")
    sheet.save(out / "stills.png")
    print(f"{len(stills)} cases in {out}, the contact sheet is {out / 'stills.png'}")


if __name__ == "__main__":
    main(sys.argv[1])
