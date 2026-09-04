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

## The suite publishes numbers, not only photographs

Every probe-shaped test ships an `expected:` table in its own source, right
after the `info:` string. probe1's is

    expected:
        dc.w    $E08B,$E28D,$E48F,$E691,$E891,$EA97,$EC97,$EE99
        dc.w    $F09B,$F29D,$F49C,$F69C,$F8A1,$FAA2,$FCA2,$FEA6

and it matches `probe1_A500_ECS.JPG` value for value. **Score against these
rather than against the photographs.** Reading sixteen hex words off a CRT with
a phone is how a wrong digit gets recorded as a wrong core, and on 2026-09-03 a
misread of one value cost a working hypothesis: VHPOSR reports `vpos[7:0]`, so
`$31` is line 49 *or* line 305, and it was first written down as line 49.

Twenty-two of the thirty tests carry a table. `cycle01v`, `cycle01vh`,
`cycleD9v`, `cycleD9vh`, `lof1`, `lof2` and `vhpos1`-`vhpos5` do not -- those are
photograph-scored, and `vhpos1`-`vhpos5` draw colour bars rather than numbers.

Note what each table is a table OF. The `probe`, `ersy` and `cycle` tests read
**VHPOSR**, so an entry is `{vpos[7:0], hpos[8:1]}`. The `vprobe` tests read
**VPOSR**, so bit 15 is LOF and the low bits are `vpos[10:8]` -- which is why
their tables are all `$8000` and `$8001`, and why they are the tests that would
catch a lost vertical high bit.

| Test | Expected, slots 0-F |
|---|---|
| `probe1` | `$E08B $E28D $E48F $E691 $E891 $EA97 $EC97 $EE99 $F09B $F29D $F49C $F69C $F8A1 $FAA2 $FCA2 $FEA6` |
| `probe2` | `$E8BE $E8C8 $E8D2 $E8DC $E903 $E90D $E917 $E921 $E92B $E935 $E93F $E949 $E953 $E95D $E967 $E971` |
| `probe3` | `$E8C0 $E8CA $E8D4 $E8DE $E905 $E90F $E919 $E923 $E92D $E937 $E941 $E94B $E955 $E95F $E969 $E973` |
| `probe4` | `$E8C2 $E8CC $E8D6 $E8E0 $E907 $E911 $E91B $E925 $E92F $E939 $E943 $E94D $E957 $E961 $E96B $E975` |
| `probe5` | `$FFBA $FFC4 $FFCE $FFD8 $FFE2 $0009 $0013 $001D $0027 $0031 $003B $0045 $004F $0059 $0063 $006D` |
| `probe6` | `$FFBE $FFC8 $FFD2 $FFDC $0003 $000D $0017 $0021 $002B $0035 $003F $0049 $0053 $005D $0067 $0071` |
| `probe7` | `$FFBE $FFC8 $FFD2 $FFDC $0003 $000D $0017 $0021 $002B $0035 $003F $0049 $0053 $005D $0067 $0071` |
| `probe8` | `$38BC $38C6 $38D0 $38DA $3801 $0011 $001B $0025 $002F $0039 $0043 $004D $0057 $0061 $006B $0075` |
| `probe9` | `$38BC $38C6 $38D0 $38DA $3801 $0011 $001B $0025 $002F $0039 $0043 $004D $0057 $0061 $006B $0075` |
| `probe10` | `$38C2 $38CC $38D6 $38E0 $000D $0017 $0021 $002B $0035 $003F $0049 $0053 $005D $0067 $0071 $007B` |
| `probe11` | `$37BC $37C6 $37D0 $37DA $3701 $0011 $001B $0025 $002F $0039 $0043 $004D $0057 $0061 $006B $0075` |
| `probe12` | `$37BC $37C6 $37D0 $37DA $3701 $0011 $001B $0025 $002F $0039 $0043 $004D $0057 $0061 $006B $0075` |
| `probe13` | `$37C2 $37CC $37D6 $37E0 $000D $0017 $0021 $002B $0035 $003F $0049 $0053 $005D $0067 $0071 $007B` |
| `ersy1` | `$E06F $E271 $E477 $E679 $E873 $EA75 $EC7B $EE7D $F073 $F275 $F476 $F67A $F870 $FA76 $FC7A $FE7C` |
| `ersy2` | `$40BF $40C9 $40D3 $40DD $4000 $4000 $4000 $4000 $4000 $4000 $4000 $4000 $4000 $4000 $4000 $4000` |
| `vprobe1` | `$8000 x5, then $8001 x11` |
| `vprobe2` | `$8000 x5, then $8001 x11` |
| `vprobe3` | `$8000 x4, then $8001 x12` |
| `vprobe4` | `$8000 x4, then $8001 x12` |

`probe7` is identical to `probe6`, and `probe9` to `probe8`, and `probe12` to
`probe11`. That is the suite's own doing -- the pairs differ in how they get to
the same place -- so identical results for a pair are expected, not a copy-paste
mistake in this table.

`ersy2`'s last twelve entries being `$4000` is the point of that test: crossing
a scanline with ERSY set stops HSYNC on real hardware, so the probes after the
crossing read a horizontal position of zero. A core that keeps counting through
it produces sixteen plausible values and fails silently.

## What was run on 2026-09-03, and where it got to

Four tests, A500 profile, on the seed 4 netlist.

| Test | Result |
|---|---|
| `vhpos1` | matches the OCS reference -- one white bar, ~20 bars, right extent |
| `probe1` | slot 0 `$3118`, slots 1-F `$0000` |
| `ersy1` | slot 0 `$3142`, slots 1-F `$0000` |
| `probe2` | sixteen values, correct `$0A` step and line wrap, but `$31xx` |

The rest of the sheet below is unfilled on purpose: with probe1 in that state
the per-row counts would not mean what upstream's 304 means, so scoring the
remaining tests would produce a number that cannot be compared to anything.

Two benches came out of it, both in CI, and **both pass**:

- `rtl/sim/beamcounter/tb_beamcounter_vposr_sweep.sv` -- every line of a PAL and
  an NTSC frame reads back its own line number, including the lines these tests
  probe. The readback is not the fault.
- `rtl/sim/copper/tb_copper_wait.sv` -- probe1's own list, sixteen WAITs at
  `$E051`-`$FE6F`, releases all sixteen MOVEs on lines 224-254. The copper's
  WAIT is not the fault either.

## What was run on 2026-09-04

**The A1200 differential, and why it is void.** A500 uses fx68k, A1200 and CD32
use TG68K, so the same ADF on both was meant to separate a CPU-specific
interrupt latency from everything else. It does not work: `probe1` on A1200
renders its title correctly positioned -- better than A500, which duplicated it
and rolled the frame -- but prints no value rows at all, and the background is a
noise field rather than blue. `probe2` on A1200 renders text for roughly 100 ms,
about five frames, and then the display dies. Both programs misbehave on AGA, in
different ways, so nothing can be attributed to the CPU through them. These are
OCS/ECS programs and vAmigaTS ships A500 references only; running them on AGA is
legitimate as a differential but only while the program still functions there.

**The copper bench grew two more cases, and both pass.**

- probe1's list again with the real `agnus_bitplanedma` competing, configured
  exactly as `probe.i` does. Bitplane contention is not the fault.
- probe1's list writing `INTREQ $8004` through `paula_intcontroller`. All
  sixteen become a level 1 request, each serviced before the next. The interrupt
  controller is not the fault.

## Where that leaves it

Eliminated, each with a bench in CI: the VPOSR/VHPOSR readback, the copper's
WAIT, bitplane DMA contention, and the copper-to-Paula interrupt path.

**Do not assume the CPU next.** It was the obvious remaining suspect and the
hardware argues against it: on A500 `probe2` delivered all sixteen, and its
sixteen INTREQ writes are back-to-back, far tighter than probe1's two-lines
apart. A core that services sixteen in rapid succession does not miss fifteen
spread over thirty lines.

What remains is integration -- full `agnus.v` arbitration with the CPU and
refresh competing for slots, which no bench here models. Refresh takes the first
slots of every line ahead of everything.

**Three harness faults were fixed along the way, and every one of them produced
output that read as a finding.** They are documented at the sites and in the
commits, but the pattern is worth carrying: a MOVE routed to the wrong register
still passed the WAIT assertions; two writers on one bus made every host
register write vanish and reported "copper raised level 1 0 times"; and
`agnus_bitplanedma.v` has no `timescale of its own while writing every register
with `<= #1`, so compiled after a file that sets one it silently ignores every
write. The last two each independently made the bitplane contention case
vacuous, and a vacuous pass is indistinguishable from a real one. That case now
counts bitplane DMA cycles and fails below 1000; the real figure is 8000 per
frame against 0 when either fault is present.

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
