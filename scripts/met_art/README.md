# The weather picture's landscape

`packages/code/panel/met_scene.h` draws the weather over a landscape that comes from the SVG layers here, an
artboard of 1600x1000. `met_art.h` next to it is made from them; don't edit it, change the layers and make it again:

```sh
pip install cairosvg numpy pillow
python3 scripts/met_art/raster.py scripts/met_art/layers.json scripts/met_art/svg /tmp/layers320.npz
python3 scripts/met_art/build.py scripts/met_art/layers.json /tmp/layers320.npz packages/code/panel/met_art.h
```

- `layers.json` lists the layers back to front with where they sit on the artboard. A layer's name picks what it
  is in `build.py` (`ROLE`), so a new rock is `Boulder 5` and a new patch of heath `Near heath 4`.
- The artboard is scaled by exactly 0.2 into rows 20 to 219 of the 320x240 picture, so its proportions and
  golden-ratio layout stay as drawn. The rows above are sky, the ones below carry the foreground on.
- Each pixel of the land gets a material slot (mountain band, bay, heath near or far, granite, cabin wall, window,
  ...). At night `met_scene.h` uses the artboard's own colour for each slot; by day, at sunset, in snow, fog and
  rain it uses its own colours for them (`SEASON_DAY`, `day_colour()`).
- The aurora is the artboard's, in three bands: its faint top, its body and its bright lower edge. The sky, the sun,
  the moon, the stars, clouds, rain and snow are drawn live, as before.
- The people are their own small sprites: `Climber`, `Kayak`, and `Runner`, `Runner (kayaker)`,
  `Skier (climber)` and `Skier (kayaker)` for rain, gales and snow. Feet go where `anchor` says. Fog covers them
  as it covers the ground they stand on. The `Buhund` is one too, as drawn, so it can go indoors in heavy rain; the
  land behind it comes from the layers under it. The reindeer and the walrus are drawn in `met_scene.h`.
- A spruce stands behind the cabin's roof (`Spruce`) and a birch beside the cabin (`Birch`). The birch's crown
  is one slot that `met_scene.h` colours for the time of year: bare, first leaves, summer green, yellow, its last
  leaves. Bare, its branches show and most of the crown shows what's behind it, which `build.py` keeps for each of
  its pixels (`BIRCH_UNDER`).
