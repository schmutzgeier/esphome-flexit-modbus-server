"""Turns the rasterized layers into what met_scene.h draws from: a map of material slots for the landscape, the
aurora's three bands, the figures as small sprites, and a few spots (chimney, door, eaves). Writes met_art.h.

    python3 build.py layers.json layers320.npz met_art.h [preview.png]

Each pixel of the landscape gets the slot of the topmost layer covering it, split by the artboard's own shading
(the mountain's ridge bands, the heath's distance). met_scene.h colours each slot for the light, the season and the
weather, and uses the artboard's colour at night.
"""

import json
import sys

import numpy as np
from PIL import Image

W, H = 320, 240
ART_Y0, ART_Y1 = 20, 220  # the artboard's rows, see raster.py

# The material slots, in the order met_scene.h's Slot enum has them. depth: 0 the mountain, 1 the far land and
# the bay, 2 near. Fog covers depth 0 first and depth 1 when it's thick, like the original's far and middle hills.
SLOTS = [
    ("SKY", -1),
    ("MTN_RIM", 0),  # the ridge's rim, lit by the aurora in the artboard
    ("MTN_A", 0),  # ridge bands, darkest (top) to lightest (in the mist at the foot)
    ("MTN_B", 0),
    ("MTN_C", 0),
    ("SEA_FAR", 1),
    ("SEA_NEAR", 1),
    ("ROCK_LIGHT", 1),
    ("ROCK_DARK", 1),
    ("HEATH_FAR", 1),
    ("HEATH_MID", 1),
    ("HEATH_NEAR", 2),
    ("HEATH_FORE", 2),
    ("GRANITE_FAR", 1),
    ("GRANITE", 2),
    ("PATH", 2),
    ("SHEEP", 1),
    ("SHEEP_DARK", 1),
    ("BLACK", 2),  # the dog, the cabin's roof and frames
    ("CABIN_WALL", 2),
    ("CABIN_TRIM", 2),
    ("WINDOW", 2),
    ("SPILL", 2),  # heath in the windows' light
    ("JUNIPER", 2),
    ("HEATHER", 2),
    ("COTTON", 2),
    ("TREE", 2),  # the spruces by the cabin
    ("BIRCH", 2),  # the birch's crown, coloured for the season
    ("BIRCH_BARK", 2),
    ("BIRCH_TWIG", 2),  # its branches, in the crown's colour while it's in leaf
]
S = {name: i for i, (name, _) in enumerate(SLOTS)}

ROLE = {
    "Sky": "sky",
    "Stars": None,
    "Moon": None,
    "Aurora horizon glow": "aurora",
    "Aurora upper band": "aurora",
    "Aurora main band": "aurora",
    "Mountain": "mountain",
    "Mountain mist": None,
    "Bay": "sea",
    "Rocky island": "rock",
    "Land": "land",
    "Far heath 1": "heath_far",
    "Far heath 2": "heath_far",
    "Far haze": None,
    "Granite dome": "granite_far",
    "Heath rise": "heath_mid",
    "Cabin hill": "heath_near",
    "Footpath": "path",
    "Cabin": "cabin",
    "Chimney smoke": None,
    "Buhund": "dog",
    "Spruce": "tree",
    "Birch": "birch",
    "Shore bank": "rock",
}
FIGURES = {
    "Climber": "CLIMBER",
    "Kayak": "KAYAKER",
    "Runner": "RUNNER",
    "Skier (climber)": "SKIER_CLIMBER",
    "Skier (kayaker)": "SKIER_KAYAKER",
    "Runner (kayaker)": "RUNNER_KAYAKER",
}


def role_of(name):
    if name in ROLE:
        return ROLE[name]
    for prefix, r in [
        ("Sheep", "sheep"),
        ("Grass", None),
        ("Boulder 1", "granite_far"),
        ("Boulder", "granite"),
        ("Rock slab", "granite"),
        ("Juniper", "juniper"),
        ("Heather", "heather"),
        ("Cotton grass", "cotton"),
        ("Near heath", "heath_near"),
        ("Foreground heath", "heath_fore"),
    ]:
        if name.startswith(prefix):
            return r
    if name in FIGURES:
        return "figure"
    raise SystemExit(f"no role for layer {name!r}")


def q4(c):
    """A float RGB colour as 0xRRGGBB at 4 bits per channel, like met_scene.h's q()."""
    r, g, b = (int((max(0.0, min(1.0, v)) * 255 + 8) // 17 * 17) for v in c)
    return r << 16 | g << 8 | b


def lum(c):
    return c[..., 0] * 0.299 + c[..., 1] * 0.587 + c[..., 2] * 0.114


def slot_of(role, name, c, y, cabin_glow):
    """The slot for a pixel the layer `name` owns, c its colour in the artboard (with everything over it)."""
    lu = float(lum(c))
    if role == "mountain":
        if c[1] - c[2] > 0.02 and c[1] > 0.3:
            return S["MTN_RIM"]
        return S["MTN_A"] if lu < 0.14 else S["MTN_B"] if lu < 0.22 else S["MTN_C"]
    if role == "sea":
        return S["SEA_FAR"] if y < 192 else S["SEA_NEAR"]
    if role == "rock":
        return S["ROCK_LIGHT"] if lu >= 0.3 else S["ROCK_DARK"]
    if role == "land":
        return (
            S["HEATH_FAR"]
            if lu >= 0.18
            else S["HEATH_MID"]
            if y < 200
            else S["HEATH_FORE"]
        )
    if role in ("heath_far", "heath_mid", "heath_near", "heath_fore"):
        s = S[role.upper()]
        if s == S["HEATH_NEAR"] and cabin_glow >= 0.1:
            return S["SPILL"]
        return s
    if role == "granite_far":
        return S["GRANITE_FAR"]
    if role == "granite":
        return S["GRANITE"]
    if role == "path":
        return S["PATH"]
    if role == "sheep":
        return S["SHEEP"] if lu >= 0.25 else S["SHEEP_DARK"]
    if role == "dog":
        return S["BLACK"]
    if role == "cabin":
        if c[0] - c[2] > 0.25 and lu > 0.4:
            return S["WINDOW"]
        if lu < 0.065:
            return S["BLACK"]
        if lu > 0.2:
            return S["CABIN_TRIM"]
        return S["CABIN_WALL"]
    if role in ("juniper", "heather", "cotton", "tree"):
        return S[role.upper()]
    if role == "birch":
        # c is the birch's own colour here, so its crown's edges stay crown
        if c[1] > c[0] + 0.03 and c[1] > c[2]:
            return S["BIRCH"]
        if lu >= 0.35:
            return S["BIRCH_BARK"]
        return (
            S["BLACK"] if c[2] > c[0] + 0.02 else S["BIRCH_TWIG"]
        )  # a mark, or a branch
    raise ValueError(role)


def rle_rows(grid, y0, y1):
    """Rows y0..y1-1 as (value, run) byte pairs, runs of at most 255. Returns the bytes and each row's offset."""
    data, offs = [], []
    for y in range(y0, y1):
        offs.append(len(data))
        row = grid[y]
        x = 0
        while x < W:
            v, n = int(row[x]), 1
            while x + n < W and row[x + n] == v and n < 255:
                n += 1
            data += [v, n]
            x += n
    offs.append(len(data))
    return data, offs


def figure(a, threshold=0.45):
    """A figure's pixels: 1 its dark body, 2 its light (headlamp, rim). Its box, and where the lamp is."""
    al = a[..., 3]
    on = al >= threshold
    ys, xs = np.where(on)
    x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
    px = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
    lamp = None
    best = 0.0
    for y in range(y0, y1):
        for x in range(x0, x1):
            if not on[y, x]:
                continue
            lu = float(lum(a[y, x, :3]))
            px[y - y0, x - x0] = 2 if lu > 0.5 else 1
            if lu > best and lu > 0.5:
                best, lamp = lu, (x, y)
    return {
        "x": int(x0),
        "y": int(y0),
        "w": int(x1 - x0),
        "h": int(y1 - y0),
        "px": px,
        "lamp": lamp,
    }


def main():
    spec_path, npz, out = sys.argv[1:4]
    preview = sys.argv[4] if len(sys.argv) > 4 else None
    with open(spec_path) as f:
        spec = json.load(f)
    L = np.load(npz)
    layers = spec["layers"]

    # The artboard as drawn, without the figures, the moon, the stars and the aurora (those are drawn live)
    comp = np.zeros((H, W, 3))
    for l in layers:
        r = role_of(l["name"])
        if r in ("figure", "aurora") or l["name"] in ("Moon", "Stars"):
            continue
        a = L[l["name"]]
        al = a[..., 3:]
        comp = comp * (1 - al) + a[..., :3] * al
    cabin_glow = L["Cabin"][..., 3]

    slot = np.zeros((H, W), dtype=np.uint8)
    owner = np.full((H, W), -1)
    for i, l in enumerate(layers):
        r = role_of(l["name"])
        if r in (None, "sky", "aurora", "figure"):
            continue
        thr = 0.3 if r == "path" else 0.5
        m = L[l["name"]][..., 3] >= thr
        owner[m] = i
    for y in range(H):
        for x in range(W):
            i = owner[y, x]
            if i < 0:
                continue
            name = layers[i]["name"]
            r = role_of(name)
            slot[y, x] = slot_of(
                r,
                name,
                L[name][y, x, :3] if r == "birch" else comp[y, x],
                y,
                cabin_glow[y, x] if y >= 150 else 0,
            )

    # The 20 rows under the artboard: its bottom row's heath, bay and shore carried on down, without the things
    # standing on it
    keep = {
        S["SEA_FAR"],
        S["SEA_NEAR"],
        S["ROCK_DARK"],
        S["ROCK_LIGHT"],
        S["HEATH_FORE"],
        S["HEATH_NEAR"],
    }
    last = ART_Y1 - 1
    for x in range(W):
        k = int(slot[last, x])
        k = k if k in keep else S["HEATH_FORE"]
        slot[ART_Y1:, x] = k
        comp[ART_Y1:, x] = (
            comp[last, x]
            if k == slot[last, x]
            else comp[last][slot[last] == S["HEATH_FORE"]].mean(axis=0)
        )

    # Night colours: each slot's average in the artboard
    night = []
    for k, (name, _) in enumerate(SLOTS):
        m = slot == k
        night.append(q4(comp[m].mean(axis=0)) if m.any() and k else 0)
    # The dog goes indoors in heavy rain, so it's a sprite too: its pixels as drawn (still counted in the black's
    # colour above), and the land under it from the layers behind
    dog_i = next(i for i, l in enumerate(layers) if role_of(l["name"]) == "dog")
    dog = owner == dog_i
    bare = np.zeros((H, W, 3))  # the artboard without the dog, for what's under it
    for l in layers:
        r = role_of(l["name"])
        if r in ("figure", "aurora", "dog") or l["name"] in ("Moon", "Stars"):
            continue
        al = L[l["name"]][..., 3:]
        bare = bare * (1 - al) + L[l["name"]][..., :3] * al
    for y, x in zip(*np.where(dog)):
        under = [
            i
            for i, l in enumerate(layers[:dog_i])
            if role_of(l["name"]) not in (None, "sky", "aurora", "figure")
            and L[l["name"]][y, x, 3] >= (0.3 if role_of(l["name"]) == "path" else 0.5)
        ]
        name = layers[under[-1]]["name"] if under else None
        slot[y, x] = (
            slot_of(
                role_of(name), name, bare[y, x], y, cabin_glow[y, x] if y >= 150 else 0
            )
            if name
            else 0
        )
    # The birch's crown is bare in winter, half its pixels showing what's behind it: the slot of the layer under each
    birch_i = next(i for i, l in enumerate(layers) if role_of(l["name"]) == "birch")
    back = np.zeros((H, W, 3))
    for l in layers[:birch_i]:
        if role_of(l["name"]) in ("figure", "aurora") or l["name"] in ("Moon", "Stars"):
            continue
        al = L[l["name"]][..., 3:]
        back = back * (1 - al) + L[l["name"]][..., :3] * al
    crown = slot == S["BIRCH"]
    bys, bxs = np.where(crown)
    bx0, by0 = int(bxs.min()), int(bys.min())
    birch_under = np.zeros((bys.max() + 1 - by0, bxs.max() + 1 - bx0), dtype=np.uint8)
    for y, x in zip(bys, bxs):
        under = [
            i
            for i, l in enumerate(layers[:birch_i])
            if role_of(l["name"]) not in (None, "sky", "aurora", "figure")
            and L[l["name"]][y, x, 3] >= 0.5
        ]
        name = layers[under[-1]]["name"] if under else None
        birch_under[y - by0, x - bx0] = (
            slot_of(role_of(name), name, back[y, x], y, 0) if name else 0
        )
    # The aurora's bands from its layers' coverage: 1 the faint top, 2 the body, 3 the bright lower edge
    A = np.zeros((H, W))
    for n in ("Aurora upper band", "Aurora main band"):  # not its glow along the ridge
        al = L[n][..., 3]
        A = A + al * (1 - A)
    # softened across first, so the posterized rays read as the curtain's folds rather than as noise
    k = np.array([1, 2, 3, 2, 1], dtype=float)
    k /= k.sum()
    A = np.apply_along_axis(lambda r: np.convolve(r, k, mode="same"), 1, A)
    aur = np.where(
        A >= 0.72, 3, np.where(A >= 0.36, 2, np.where(A >= 0.16, 1, 0))
    ).astype(np.uint8)
    # Above the artboard the rays fade out at their own heights instead of stopping at its top edge
    for x in range(W):
        if aur[ART_Y0, x]:
            n = (x * 2654435761 >> 7) % ART_Y0
            aur[ART_Y0 - n : ART_Y0, x] = 1
    aur[slot != 0] = 0  # only where there's sky
    ays = np.where(aur.any(axis=1))[0]
    a0, a1 = int(ays.min()), int(ays.max()) + 1

    gys = np.where((slot != 0).any(axis=1))[0]
    g0 = int(gys.min())
    ground, goffs = rle_rows(slot, g0, H)
    aurora, aoffs = rle_rows(aur, a0, a1)
    mtn = (slot >= S["MTN_RIM"]) & (slot <= S["MTN_C"])
    ridge = [
        int(np.argmax(mtn[:, x]))
        if mtn[:, x].any()
        else int(np.argmax(slot[:, x] != 0))
        for x in range(W)
    ]
    top = [int(np.argmax(slot[:, x] != 0)) for x in range(W)]
    depth = np.array([d for _, d in SLOTS])[slot]
    land = [
        int(np.argmax(depth[:, x] >= 1)) for x in range(W)
    ]  # the ground in front of the mountain

    figs = {
        FIGURES[l["name"]]: figure(L[l["name"]]) for l in layers if l["name"] in FIGURES
    }
    ys, xs = np.where(dog)
    figs["DOG"] = {
        "x": int(xs.min()),
        "y": int(ys.min()),
        "w": int(xs.max() - xs.min() + 1),
        "h": int(ys.max() - ys.min() + 1),
        "px": dog[ys.min() : ys.max() + 1, xs.min() : xs.max() + 1].astype(np.uint8),
        "lamp": None,
    }
    for key, fg in figs.items():  # as far away as what's under its feet, for the fog
        x, y = fg["x"] + fg["w"] // 2, min(H - 1, fg["y"] + fg["h"] - (key != "DOG"))
        fg["depth"] = (
            int(depth[y, x]) if slot[y, x] else 0
        )  # the dog's feet stand on the hill below the mountain

    # Spots on the cabin, from the artboard's cabin drawing
    def spot(x, y):
        return round(x * 0.2), round(y * 0.2 + ART_Y0)

    spots = {
        "CHIMNEY": spot(1537, 531),
        "DOOR": spot(1477, 650),
        "EAVE_L": spot(1390, 525),
        "EAVE_R": spot(1574, 565),
        "WINDOW": spot(1434, 596),
    }
    # where the cabin ends and the trees beside it begin, halfway down its wall: the deer comes out of them there
    y = (spots["EAVE_R"][1] + spots["DOOR"][1]) // 2
    cabin = [S[k] for k in ("CABIN_WALL", "CABIN_TRIM", "BLACK", "WINDOW")]
    x = spots["DOOR"][0]
    while x < W - 1 and slot[y, x] in cabin:
        x += 1
    spots["TREES"] = (x, y)

    with open(out, "w") as f:
        w = f.write
        w(
            "// The landscape for met_scene.h, made by scripts/met_art/build.py from the SVG layers next to it.\n"
        )
        w("// Generated, don't edit: change the SVGs or build.py and run it again.\n")
        w("#pragma once\n#include <cstdint>\n\nnamespace flexit::panel::met::art {\n\n")
        w("// clang-format off: generated\n")
        w("enum Slot : uint8_t {\n")
        for name, _ in SLOTS:
            w(f"  S_{name},\n")
        w("  SLOTS\n};\n")
        w("// Each slot's colour in the artboard, the scene at night\n")
        w(
            "static const uint32_t NIGHT[SLOTS] = {"
            + ", ".join(f"0x{c:06X}" for c in night)
            + "};\n"
        )
        w("// 0 the mountain, 1 the far land and the bay, 2 near. -1 is the sky\n")
        w(
            "static const int8_t DEPTH[SLOTS] = {"
            + ", ".join(str(d) for _, d in SLOTS)
            + "};\n\n"
        )

        def arr(ctype, name, vals, per=24):
            w(f"static const {ctype} {name}[{len(vals)}] = {{\n")
            for i in range(0, len(vals), per):
                w("    " + ", ".join(str(v) for v in vals[i : i + per]) + ",\n")
            w("};\n")

        w(
            f"// The landscape from row GROUND_Y0 down, each row as (slot, run) pairs\nstatic constexpr int GROUND_Y0 = {g0};\n"
        )
        arr("uint16_t", "GROUND_ROW", goffs)
        arr("uint8_t", "GROUND", ground, 32)
        w(
            "\n// The aurora's bands, rows AURORA_Y0 to AURORA_Y1 - 1, as (band, run) pairs\n"
        )
        w(f"static constexpr int AURORA_Y0 = {a0}, AURORA_Y1 = {a1};\n")
        arr("uint16_t", "AURORA_ROW", aoffs)
        arr("uint8_t", "AURORA", aurora, 32)
        w(
            "\n// First row of the mountain in each column, and of anything in front of the sky\n"
        )
        arr("uint8_t", "RIDGE", ridge, 32)
        arr("uint8_t", "TOP", top, 32)
        w(
            "// First row of the land, the bay or anything else in front of the mountain\n"
        )
        arr("uint8_t", "LAND", land, 32)
        w(
            "\n// The figures: 2 bits a pixel, 1 dark, 2 light (headlamp, rim), rows of w\n"
        )
        w(
            "struct Figure {\n  int16_t x, y;\n  uint8_t w, h;\n  int16_t lamp_x, lamp_y;  // -1: no headlamp\n"
        )
        w(
            "  int8_t depth;  // of the ground it's on, as DEPTH\n  const uint8_t *px;\n};\n"
        )
        for key, fg in figs.items():
            bits = []
            for row in fg["px"]:
                acc, n = 0, 0
                for v in row:
                    acc |= int(v) << (2 * (n % 4))
                    n += 1
                    if n % 4 == 0:
                        bits.append(acc)
                        acc = 0
                if n % 4:
                    bits.append(acc)
            arr("uint8_t", f"{key}_PX", bits, 32)
            lx, ly = fg["lamp"] if fg["lamp"] else (-1, -1)
            w(
                f"static const Figure {key} = {{{fg['x']}, {fg['y']}, {fg['w']}, {fg['h']}, {lx}, {ly}, {fg['depth']}, {key}_PX}};\n"
            )
        w(
            "\n// The birch's crown in the box BIRCH_X, BIRCH_Y, BIRCH_W x BIRCH_H: the slot behind each pixel, 0 the\n"
            "// sky, for when it's bare\n"
        )
        bh, bw = birch_under.shape
        w(
            f"static constexpr int BIRCH_X = {bx0}, BIRCH_Y = {by0}, BIRCH_W = {bw}, BIRCH_H = {bh};\n"
        )
        arr("uint8_t", "BIRCH_UNDER", [int(v) for v in birch_under.flatten()], bw)
        w("\n// Spots on the cabin\n")
        for k, (x, y) in spots.items():
            w(f"static constexpr int {k}_X = {x}, {k}_Y = {y};\n")
        w("// clang-format on\n\n}  // namespace flexit::panel::met::art\n")

    print(
        f"ground rows {g0}..{H - 1}: {len(ground)} bytes; aurora rows {a0}..{a1 - 1}: {len(aurora)} bytes"
    )
    print(
        "figures",
        {
            k: (v["x"], v["y"], v["w"], v["h"], v["lamp"], v["depth"])
            for k, v in figs.items()
        },
    )
    print("land beside the cabin", land[268:282])
    print("spots", spots)
    for k, (name, _) in enumerate(SLOTS):
        print(
            f"  {k:2d} {name:12s} {int((slot == k).sum()):6d} px  night 0x{night[k]:06X}"
        )

    if preview:
        pal = np.array(
            [[(c >> 16) & 255, (c >> 8) & 255, c & 255] for c in night], dtype=np.uint8
        )
        img = (comp * 255).astype(np.uint8).copy()
        g = slot != 0
        img[g] = pal[slot[g]]
        Image.fromarray(img).resize((W * 3, H * 3), Image.NEAREST).save(preview)


if __name__ == "__main__":
    main()
