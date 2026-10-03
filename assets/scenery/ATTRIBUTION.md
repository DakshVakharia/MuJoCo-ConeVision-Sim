# Scenery asset attribution

All scenery meshes are derived from Kenney (https://kenney.nl) asset packs, released under
**Creative Commons Zero (CC0 1.0)** - public domain, no attribution required (given here anyway).
License texts copied from the packs: `LICENSE-kenney-*.txt`.

| Pack | Source URL | License | Used for |
|---|---|---|---|
| Kenney Nature Kit 2.1 | https://kenney.nl/assets/nature-kit | CC0 | trees, bushes, stump, log stack |
| Kenney Racing Kit | https://kenney.nl/assets/racing-kit | CC0 | barriers, fence, billboard, light post, flag, grandstand |
| Kenney Survival Kit | https://kenney.nl/assets/survival-kit | CC0 | barrel, tent, signpost, bucket, rocks |

Direct downloads used (Oct 2026):
- https://kenney.nl/media/pages/assets/nature-kit/37ac38a37b-1677698939/kenney_nature-kit.zip
- https://kenney.nl/media/pages/assets/racing-kit/933b8fd9fd-1677580949/kenney_racing-kit.zip
- https://kenney.nl/media/pages/assets/survival-kit/4065a8185b-1712149243/kenney_survival-kit.zip

Conversion: `src/conevision_sim/scene/asset_import.py` splits each model by colour into
`meshes/<name>_<k>.obj` (Z-up, centred, base at z=0, scaled to metres) and writes `catalog.json`
(per-part rgba, size, footprint radius). Colours of the nature kit were remapped to natural greens/browns.
Tyre stacks and hay bales are generated procedurally (no download).
