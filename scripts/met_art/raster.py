"""Rasterizes the canvas's SVG layers into 320x240 RGBA layers, the way the panel's picture is framed.

The artboard is 1600x1000 (16:10), the panel 320x240 (4:3). The artboard is scaled by exactly 0.2 both ways, so
its proportions and its golden-ratio layout stay as drawn, and sits 20 rows down: its golden lines (0.2 x 382 and
618, plus 20 = 96 and 144) then fall within 4 rows of the panel's own (92 and 148). build.py fills the 20 rows of
sky above it and continues the foreground for the 20 below.
"""

import io
import json
import sys

import cairosvg
import numpy as np
from PIL import Image

W, H = 320, 240
SX = SY = 0.2  # the artboard, unstretched
OY = 20  # rows of sky above it
SS = 3  # supersampling


def render_svg(path, w, h):
    """The SVG at exactly w x h pixels."""
    with open(path, encoding="utf-8") as f:
        svg = f.read().replace("<svg ", '<svg preserveAspectRatio="none" ', 1)
    png = cairosvg.svg2png(
        bytestring=svg.encode(),
        output_width=max(1, round(w)),
        output_height=max(1, round(h)),
    )
    return Image.open(io.BytesIO(png)).convert("RGBA")


def terrain(path, l):
    """A layer where the artboard has it: placed on a supersampled canvas, then box-filtered down."""
    big = Image.new("RGBA", (W * SS, H * SS), (0, 0, 0, 0))
    im = render_svg(path, l["w"] * SX * SS, l["h"] * SY * SS)
    big.alpha_composite(im, (round(l["x"] * SX * SS), round((l["y"] * SY + OY) * SS)))
    return big


def standing(path, l, anchor=None):
    """A layer standing with its bottom middle at anchor (artboard px), else where the artboard has it."""
    ax, ay = anchor if anchor else (l["x"] + l["w"] / 2, l["y"] + l["h"])
    big = Image.new("RGBA", (W * SS, H * SS), (0, 0, 0, 0))
    im = render_svg(path, l["w"] * SX * SS, l["h"] * SY * SS)
    big.alpha_composite(
        im, (round((ax - l["w"] / 2) * SX * SS), round(((ay - l["h"]) * SY + OY) * SS))
    )
    return big


def down(big):
    """Premultiplied box filter from SSx to 1x, so edges get the right colour and coverage."""
    a = np.asarray(big, dtype=np.float64) / 255
    rgb, al = a[..., :3] * a[..., 3:], a[..., 3:]
    sh = (H, SS, W, SS)
    rgb = rgb.reshape(sh + (3,)).mean(axis=(1, 3))
    al = al.reshape(sh + (1,)).mean(axis=(1, 3))
    col = np.where(al > 1e-6, rgb / np.maximum(al, 1e-6), 0)
    return np.concatenate([col, al], axis=-1)  # float RGBA, straight alpha


if __name__ == "__main__":
    spec_path, svg_dir, out = sys.argv[1:4]
    with open(spec_path) as f:
        spec = json.load(f)
    layers = {}
    for l in spec["layers"]:
        path = f"{svg_dir}/{l['file']}"
        big = (
            terrain(path, l)
            if l["kind"] == "terrain"
            else standing(path, l, l.get("anchor"))
        )
        layers[l["name"]] = down(big).astype(np.float32)
        print(
            f"{l['name']:24s} {l['kind']:8s} coverage {layers[l['name']][..., 3].sum():8.0f}"
        )
    np.savez_compressed(out, **layers)
