"""Takes the weather overlay's characters from the panel firmware's fonts, so the forecast over the picture is in
the same Noto Sans Medium as the clock band over it. Writes packages/code/met/met_font.h.

    python3 scripts/met_font/build.py FIRMWARE/ui/fonts packages/code/met/met_font.h

FIRMWARE is a checkout of flexit-ci600-firmware. Its fonts are LVGL's, 4 bits a pixel. The picture has 16 colours,
so a pixel is the text's where it's at least half covered and the picture's elsewhere: hard edges, no anti-aliasing.
"""

import re
import sys
from pathlib import Path

# The fonts as the clock band has them (ui_saver.c): the clock in n40, the date in f14, the temperatures in f16.
# f12 for the wave height in the few rows under the date.
# '^' is written for the degree sign and '~' for the minus, an en dash, as the panel shows them.
FONTS = {
    "BIG": ("font_n40.c", "0123456789~^"),
    "MEDIUM": ("font_f16.c", "0123456789.~^ms/ "),
    "SMALL": ("font_f14.c", "0123456789~^"),
    "TINY": ("font_f12.c", "0123456789.m "),
}
CODE = {"^": 0xB0, "~": 0x2013}
THRESHOLD = 8  # of 15


def parse(path):
    src = Path(path).read_text(encoding="utf-8")
    bitmap = src[src.index("glyph_bitmap[]") :]
    bitmap = bitmap[: bitmap.index("};")]
    # glyph ids in order, from the comments before each glyph's bytes
    codes = [int(c, 16) for c in re.findall(r"/\* U\+([0-9A-F]+) ", bitmap)]
    data = bytes(int(b, 16) for b in re.findall(r"0x([0-9a-fA-F]{1,2})\b", bitmap))
    dsc = re.findall(
        r"\.bitmap_index = (\d+), \.adv_w = (\d+), \.box_w = (\d+), \.box_h = (\d+), "
        r"\.ofs_x = (-?\d+), \.ofs_y = (-?\d+)",
        src,
    )[1:]  # id 0 is reserved
    line_height = int(re.search(r"\.line_height = (\d+)", src).group(1))
    base_line = int(re.search(r"\.base_line = (\d+)", src).group(1))
    glyphs = {}
    for code, d in zip(codes, dsc, strict=True):
        index, adv, w, h, ox, oy = (int(v) for v in d)
        rows = []
        for y in range(h):
            bits = 0
            for x in range(w):
                n = y * w + x  # the pixels follow on from row to row, two to a byte
                b = data[index + n // 2]
                a = b >> 4 if n % 2 == 0 else b & 15
                if a >= THRESHOLD:
                    bits |= 1 << (w - 1 - x)
            rows.append(bits)
        glyphs[code] = (round(adv / 16), w, h, ox, oy, rows)
    return line_height, base_line, glyphs


def main(fonts_dir, out):
    rows, tables = [], []
    for name, (file, chars) in FONTS.items():
        line_height, base_line, glyphs = parse(Path(fonts_dir) / file)
        entries = []
        for ch in chars:
            adv, w, h, ox, oy, bits = glyphs[CODE.get(ch, ord(ch))]
            if w > 32:
                raise SystemExit(f"{file}: {ch!r} is {w} px wide, a row holds 32")
            entries.append(f"{{'{ch}', {adv}, {w}, {h}, {ox}, {oy}, {len(rows)}}}")
            rows += bits
        tables.append((name, file, line_height, base_line, entries))
    w = []
    w.append("// The weather overlay's characters, from the panel firmware's fonts: Noto Sans Medium as its clock band\n")
    w.append("// has it. Made by scripts/met_font/build.py, don't edit. Plain C++ without ESPHome, see met_scene.h.\n")
    w.append("#pragma once\n#include <cstdint>\n\nnamespace flexit::panel::met {\n\n")
    w.append("// A character: its advance, its box w x h, the box's offset from the pen (y up from the baseline), and\n")
    w.append("// its rows in ROWS from row, leftmost pixel in the highest of w bits\n")
    w.append("struct FontGlyph {\n  char c;\n  uint8_t adv, w, h;\n  int8_t x, y;\n  uint16_t row;\n};\n")
    w.append("struct Font {\n  const FontGlyph *glyphs;\n  uint8_t n;\n")
    w.append("  uint8_t line_height, base_line;  // base_line up from the line's bottom\n};\n\n")
    w.append("// clang-format off: generated\n")
    w.append(f"inline constexpr uint32_t FONT_ROWS[{len(rows)}] = {{\n")
    for i in range(0, len(rows), 8):
        w.append("    " + ", ".join(f"0x{r:X}" for r in rows[i : i + 8]) + ",\n")
    w.append("};\n")
    for name, file, line_height, base_line, entries in tables:
        w.append(f"// {file}\n")
        w.append(f"inline constexpr FontGlyph {name}_GLYPHS[] = {{\n")
        for e in entries:
            w.append(f"    {e},\n")
        w.append("};\n")
        w.append(
            f"inline constexpr Font FONT_{name} = {{{name}_GLYPHS, {len(entries)}, {line_height}, {base_line}}};\n"
        )
    w.append("// clang-format on\n\n}  // namespace flexit::panel::met\n")
    Path(out).write_text("".join(w), encoding="utf-8")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
