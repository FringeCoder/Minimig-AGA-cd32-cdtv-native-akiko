# T7 — running the vAmigaTS VPOS suite on the core

T7 in `core-accuracy-todo.md` is a measurement, not a change. Upstream's own
figures after `06f30af` were **268 of 304 probe rows reading 0**, so 36 rows
still disagree, and `ersy1` is correct on only 14 of 16. Those numbers are
upstream's, taken on upstream's core. Nobody has taken them on ours.

This is the cheapest open item in the backlog: no RTL edit, no fit, no reference
Amiga. vAmigaTS ships photographs of the same programs running on real A500
hardware, so scoring is done by eye against a published image.

The result decides whether T9 (NTSC line length) and the other beam-counter
items are worth spending timing margin on. If our rows are already close to
upstream's, they are not.

## Where everything is

- Suite checkout: `externalgithub/vAmigaTS` (sparse, `Agnus/Registers/VPOS`
  only — the full repo is 18257 files and clones slowly; it was cloned with
  `--filter=blob:none --no-checkout` and a cone sparse-checkout).
- On the MiSTer: `/media/fat/games/Amiga/vAmigaTS-VPOS/`, 30 ADFs, 27 MB.
- Reference photos live beside each test's source, as
  `Agnus/Registers/VPOS/<test>/<test>_A500_<chipset>.JPG`. The `.raw` and
  `.tiff` beside them are vAmiga's own digital dumps, not hardware.

Each ADF is bootable bare metal. No Workbench, no ROM beyond Kickstart, nothing
to install.

## Configure the core to match the reference

**This is the part that will invalidate the run if it is got wrong**, in exactly
the way the `boot_rom_uae=none` skew did in `chip-ram-shortfall.md`: the
references are photographs of an **A500**, so the core has to be an A500 too.

- Machine: **A500**.
- Chipset: `config_chipset_msg` offers `OCS-A500`, `OCS-A1000`, `ECS`, `AGA`.
  Pick the one matching the reference image you are scoring against — most
  tests ship both an `A500_ECS` and an `A500_OCS` photo, and they are not the
  same picture.
- Do not score an AGA run against these images. These programs target OCS/ECS,
  and a difference on AGA is not necessarily a defect.

Note the A500 uses fx68k, so `ss_ctrl`'s peek window is unavailable during
these runs (`ss_supported = |cpucfg`). Nothing here needs it.

## What each group shows

From the suite's own README:

- **probe1 – probe13** — probe VHPOSR at 16 different locations and display the
  result. 13 tests × 16 probes is where the "304 rows" figure comes from. This
  is the group T7 is actually about; run it first.
- **ersy1, ersy2** — as the probe tests, with ERSY modified. `ersy1` sets and
  resets ERSY within one scanline (no effect on hardware); `ersy2` crosses a
  scanline with ERSY set, which corrupts the display on a real machine because
  HSYNC is not generated. `ersy1` is the one T7 names as 14 of 16.
- **vprobe1 – vprobe4** — probe VPOSR across the vertical boundary, where vpos
  goes `$FF` → `$100`.
- **cycle01v, cycle01vh, cycleD9v, cycleD9vh** — VPOSR/VHPOSR read in line `$FF`
  cycle `$DF`, and line `$100` cycle `$01`.
- **vhpos1 – vhpos5** — VPOSR, VHPOSR and the CIAB TOD line counter sampled in
  the VBLANK handler, drawn as colour bars. `vhpos4` and `vhpos5` add a VPOSW
  write that forces short frames.
- **lof1, lof2** — force a long and a short frame via the LOF bit in VPOSW, then
  display the highest line number of that frame.

## Reference coverage

Not every test ships both chipsets. Score against what exists:

| Tests | ECS ref | OCS ref |
|---|---|---|
| probe1–probe13, vhpos1–vhpos5, lof1 | yes | yes |
| cycle01v, cycle01vh, cycleD9v, cycleD9vh | yes | `OCS5` |
| ersy1, ersy2 | `ECS6` | `OCS5` |
| lof2, vprobe1–vprobe4 | — | yes, OCS only |

`ECS6` / `OCS5` are the chip revisions the photo was taken on.

## Scoring sheet

Fill this in as the tests are run. For the probe group, record how many of the
16 probes match, so the total is directly comparable to upstream's 304.

| Test | Chipset run | Matches | Notes |
|---|---|---|---|
| probe1 | | / 16 | |
| probe2 | | / 16 | |
| probe3 | | / 16 | |
| probe4 | | / 16 | |
| probe5 | | / 16 | |
| probe6 | | / 16 | |
| probe7 | | / 16 | |
| probe8 | | / 16 | |
| probe9 | | / 16 | |
| probe10 | | / 16 | |
| probe11 | | / 16 | |
| probe12 | | / 16 | |
| probe13 | | / 16 | |
| **probe total** | | **/ 208** | upstream: 268/304 across their row set |
| ersy1 | | / 16 | upstream: 14 of 16 |
| ersy2 | | | expect display corruption; that is correct behaviour |
| vprobe1 | | | OCS only |
| vprobe2 | | | OCS only |
| vprobe3 | | | OCS only |
| vprobe4 | | | OCS only |
| cycle01v | | | |
| cycle01vh | | | |
| cycleD9v | | | |
| cycleD9vh | | | |
| vhpos1 | | | |
| vhpos2 | | | |
| vhpos3 | | | |
| vhpos4 | | | |
| vhpos5 | | | |
| lof1 | | | |
| lof2 | | | OCS only |

## Two things that will produce a confident wrong answer

1. **Scoring the wrong chipset photo.** Most tests ship two, and they differ.
   Write down which one each row was scored against.
2. **Reading a scaled display.** `winuae-fidelity-is-not-enough.md` is the
   standing warning here — the MiSTer's scaler has vetoed correct chipset
   timing before. If a row looks marginally off rather than plainly different,
   suspect the display path before the beam counter.
