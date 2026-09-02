# AmigaCD core: accuracy and cleanup TODO

Written 2026-08-31 after the `74d6ce0` sync. Every item cites a file and line or
an upstream commit. Nothing here is inferred from behaviour alone.

**Re-verified 2026-08-31 (second pass)** against core `46b33f5` and userspace
`a02de37`, fifteen commits after this was written. Every open item below was
re-checked against the current code and its citations still resolve. What
changed: T0's numbers were superseded by a seed choice and are corrected below,
and T1, T16, T18, T21 and T21a are now done.

## How we verify, given no reference Amiga

We cannot measure against real hardware. That rules out the method upstream uses
for chipset timing work, so every item below names what we *can* check it with:

- **SIM** — an Icarus bench in CI. Our primary instrument, and the one that has
  actually caught things: the light pen latch bench showed the old code taking
  one distinct VPOSR value across 260 lines, and it caught upstream's VHPOSR
  decrement the moment it landed. Nine bench directories run in CI today.
- **SUITE** — a test program run on the core, scored against the *suite's own
  published reference values*. vAmigaTS ships expected output, so this needs no
  reference Amiga — only the MiSTer and someone to read the screen.
- **TITLE** — regression observation against known-good software. Flink,
  Castlevania AGA, Cannon Fodder, Jim Power, save/restore, physical disc.
- **FIT** — Quartus slacks. Both edges, always; a hold violation fails at every
  clock frequency.

Anything with no verification route is marked as such and ranked accordingly.

---

## T0 — Timing headroom is thin, but it is not the blocker  [FIT]

**Corrected 2026-08-31. The reading this section was written on has since been
superseded by a seed choice, and the conclusion drawn from it was wrong.**

`docs/sdram-timing-headroom.md` (2026-08-06) established that this core has no
headroom, that `sd_addr` was the repeated destination of the binding path, and
it designated a fix: split the chip and CPU address loads and select at the
output.

That fix was done — our `5cb32ea perf(sdram): split sd_addr into chip and CPU
halves, select at the output` — and the problem returned anyway. This section
first recorded **setup −0.347, hold −0.401**, TNS −3.303 / −1.104 at the tip of
`sync/minimig-upstream-74d6ce0`, and concluded that the merge's chipset commits
might have to be dropped.

**They did not.** The merge closed on seed 10 — `1a4c33e quartus: seed 10 for
the 74d6ce0 netlist` — and `output_files/Minimig.sta.rpt` now reads:

    Worst-case setup slack is 0.117
    Worst-case hold slack is 0.234

Both positive, TNS 0.000 on every domain, worst-case domain
`emu|pll|...|counter[0].output_counter|divclk`. So all four chipset commits are
affordable as they stand, `7ce2980` included, and nothing needs re-fitting.

What survives is the premise, not the blocker. **+0.117 ns of setup margin is
thin, and it took a sweep to find it** — seed 1 in that same sweep was rejected
at +0.134 setup / −0.495 hold. Every HOT item still needs its own fit before it
can be believed. But do not treat T0 as gating them, and do not rank other work
by how much slack it buys.

Actions:
1. **Open.** Update `docs/sdram-timing-headroom.md:107` — it still says the
   designated fix is "not attempted. This is the one to reach for when the
   problem returns". It was attempted, it landed, the problem returned, and a
   seed then absorbed it. That whole sequence is worth more than the original
   prediction.
2. **Done 2026-08-31.** `quartus_sta -t` with `report_timing -setup -npaths 10`
   against the seed 10 netlist. Ten seconds, no re-fit. **The destination is not
   `sd_addr`, and it is not in `sdram_ctrl` at all** — neither appears anywhere
   in the worst ten. Seven of the ten end inside `ss_ctrl`:

    0.117  f2sdram~FF_3780                     -> ss_ctrl|state.S_L_KICK_CHK
    0.122  f2sdram~FF_3780                     -> ss_ctrl|state.S_L_KICK_CHK
    0.146  f2sdram~FF_3777                     -> ss_ctrl|state.S_L_KICK_CHK
    0.151  f2sdram~FF_3777                     -> ss_ctrl|state.S_L_KICK_CHK
    0.219  ss_ctrl|hdr_length[12]              -> ss_ctrl|rd_idx[1]
    0.219  ss_ctrl|hdr_length[12]              -> ss_ctrl|rd_idx[2]
    0.230  ss_ctrl|hdr_length[12]              -> ss_ctrl|rd_idx[23]
    0.263  ddram_ctrl|a2065_ddram_arbiter|busy -> f2sdram~FF_1381
    0.286  f2sdram~FF_3780                     -> ss_ctrl|state.S_L_MAGIC
    0.307  ss_ctrl|rd_data[52]                 -> ss_ctrl|rd_idx[1]

   Hold is comfortable and unrelated: worst +0.234 on an `IIR_filter` tap, with
   `ss_ctrl|kick_pairs[12]` third at +0.247.

   Two of the three shapes here are the save state controller's restore-side
   load path — the DDR3 read arriving at the Kickstart-check and magic-check
   states, and the header/index arithmetic feeding `rd_idx`. **That is T17's
   subject matter, which promotes T17 from tidiness to the targeted fix.**
   `docs/sdram-timing-headroom.md` has been updated with the same finding, since
   its whole `sd_addr` framing predates it.
3. ~~Only then decide whether the four chipset commits in the pending merge are
   affordable.~~ **Answered: they fit on seed 10.**

---

## T1 — CIA CNT: guard the revert in CI  [SIM] — [DONE 2026-08-31]

**Decided 2026-08-31: we are NOT reporting this upstream.** So the revert is
permanent, it will be re-applied on every future Minimig-AGA sync, and the only
question left is how to stop it being missed.

Background. `ciaa.v:20-27` has listed the gap since 2005:

    // counter inputs for timer A and B other then 'E' clock

Upstream `b013ce3` added the counting logic but not the wiring — `minimig.v`
still drives both CIAs with `.cnt_in(1'b1)`, so `cnt_rise` never pulses and
selecting INMODE stops the timer dead. It broke Flink; reverted 2026-08-27, and
again 2026-08-31 when the merge brought it back. Confirmed by TITLE both ways.

Carrying a revert forever is a fine decision, but "remember to check" is not a
control. It has already been forgotten once between two syncs five days apart,
and the failure is near-silent: a title starts and then hangs, with every data
path healthy, which is the hardest possible thing to attribute.

**Done.** `rtl-sim.yml` now carries a "CIA count source is eclk" step that fails
the build if either source is not the eclk form:

    rtl/cia_timera.v:48   assign count = eclk;
    rtl/cia_timerb.v:47   assign count = tmcr[6] ? tmra_ovf : eclk;

The revert `6df82c1` moved both lines — they were `:52` and `:51` when this was
written — so the step matches on the assign text, not on line numbers. The
failure message and the comment above the step explain *why* rather than
reporting a missing string, and say explicitly not to delete the step as stale:
the next person to hit it will be mid-merge and would otherwise assume exactly
that.

The alternative — wiring CIA-A CNT to a synthesised keyboard clock so the
upstream commit can be taken properly — stays open but unscheduled. It is
speculative work on a cold path with no way to validate the result here, and it
buys nothing that the guard does not.

## T2 — `hbstrt_reg` is the wrong width  [SIM] — [WONTFIX 2026-09-01]

`agnus_beamcounter.v:229`, `:257`:

    reg [ 8:0] hbstrt_reg; // not correct size, this should have [10:0]
    HBSTRT [8:1] : hbstrt_reg <= {data_in[ 7:0], 1'b0}; // TODO fix this

Programmed horizontal blank start truncates in ECS `varbeamen` mode. Small,
self-admitted, and testable in simulation: extend the existing beamcounter bench
to write HBSTRT above 9 bits and check the blanking edge lands where asked. The
compare at `:540` is against `hpos`, so widening the register means revisiting
that too.

**Attempted, shipped, broke the picture, reverted. Do not try this again without
reading the whole of `update_hblank()`.**

The reasoning looked solid. `custom.cpp` masks HBSTRT and HBSTOP to `0x7ff`
where the other four horizontal registers get `0xff`, so these two are the only
ones carrying anything above bit 7, and `drawing.cpp` builds a
half-colour-clock position from bit 10:

    denise_phbstrt_lores = (denise_phbstrt << 1) |
                           ((hbstrt_denise_reg >> 10) & 1);

`hpos` counts half colour clocks, so that looks like precisely the resolution we
can represent. The write became `{data_in[7:0], data_in[10]}`, a bench asserted
the edge moved, CI passed, and the fit closed.

**On hardware it blurred the picture in PAL and drew the OSD twice with a
horizontal offset.** Flink switching to NTSC on the fly made both correct, which
is what identified it: the NTSC long-line work is inactive in PAL, so the fault
had to be the other change that moves a blanking edge.

The line that was missed is the one above the block quoted: it runs only inside
`if (exthblankon_aga)`, and its else branch sets every programmed position to
`-1`. That is Denise's extended-HBLANK path, AGA only, and this core does not
implement it. What `hbstrt_reg` actually drives is the Agnus-side programmed
blanking — WinUAE's `agnus_phblank` — which compares against colour clocks and
nothing finer:

    hbstrt_cck = hbstrt & 0xff;
    if (hhp == hbstrt_cck) { agnus_phblank = true; ... }

So the original `{data_in[7:0], 1'b0}` was right all along, and bit 10 shifted
every programmed blanking edge by half a lores pixel.

The 2005 comment that started this — "not correct size, this should have
[10:0]" — is about storing the raw register so the extra bits reach the Denise
path. Storing them is harmless. Using them in this comparison is not, and there
is nothing here to use them for until extended HBLANK exists.

The bench now asserts the opposite and is the guard: bit 10, bits 9:8, bits
15:11 and the whole upper byte must all leave the edge where the low byte put
it. Its header explains the trap, because the next person will read the same
function and reach the same wrong conclusion.

**Lesson worth keeping beyond this item:** a bench that asserts the behaviour
you just implemented proves only that you implemented it. Both the bench and the
implementation came from the same misreading, so they agreed with each other and
neither could catch it. What caught it was hardware.

## T3 — The last bench outside CI  [SIM] — [DONE 2026-08-31]

`rtl/sim/chipset` is the only one of ten bench directories not in the workflow.
It is ModelSim-only, which is exactly how the two CDTV benches silently stopped
compiling at the `b265a3b` merge and stayed broken until last week.

Either port it to Icarus and add it, or delete it. Leaving a bench that nothing
runs is how the last two rotted.

**Deleted, and the premise above was wrong.** It is not a ModelSim-only bench
waiting to be ported — its DUT does not exist in this branch at all. `6f8abca`
added the bench and its two runners without `rtl/chipset_bus_trace.v`, which
`git log --all` finds only on the unmerged `hybris-blit-vpos-trace` and
`hybris-blt-dest-trace` branches, and nothing in `rtl/` or `Minimig.sv`
references `chipset_bus_trace` or `uio_cs_trace`. `run_chipset_trace.do`
compiles a file that is not there, so it has never run under any simulator here.

The bench is sound — five tests over the ring's drain order, sentinels and wrap
— and is preserved in history and on both of those branches, where the module
also lives. Restore the directory alongside the module if the trace ring is ever
merged. Vendoring a sim-only copy of the module was considered and rejected: a
CI step passing forever against logic the design does not contain is the same
dead weight in a new shape.

---

## T4 — Paula channel modulation (ADKCON attach bits) is absent  [SIM] — [DONE 2026-08-31]

Nothing to do with attaching hardware — this is Paula using one audio channel to
modulate the next. `paula_audio_channel.v:1` says "attached modes are not
supported"; the phrase is the Amiga's, and it misleads, hence the longer title.

Confirmed by grep rather than by that comment alone: `adkcon` is stored at
`paula.v:138` and read back, but **bits 0..7 are decoded nowhere in `rtl/`**.

The semantics, from WinUAE `audio.cpp:1643`:

    int audav = adkcon & (0x01 << nr);   // bits 0-3: attach VOLUME
    int audap = adkcon & (0x10 << nr);   // bits 4-7: attach PERIOD

and `audio_update_adkmasks`, which shows what it costs:

    unsigned long t = adkcon | (adkcon >> 4);
    audio_channel[0].data.adk_mask = (((t >> 0) & 1) - 1);

If either attach bit is set for a channel, its mask goes to zero: the modulating
channel is silenced and its sample data becomes the volume or period of the next
channel. Two channels spent for one richer voice, with modulation running at the
audio DMA rate instead of at whatever rate the CPU manages — which is the point,
since register-driven envelopes and sweeps are audibly stepped.

Today music that uses it fails in both directions at once: the modulator is
heard when it should be silent, and the modulation never happens. Nothing is
logged.

Cold path, self-contained, DSP and BRAM headroom available, and benchable: drive
ADKCON, feed the modulator known data, check the modulated channel's period and
volume follow it and that the modulator itself goes quiet. Best value-per-risk
item on this list.

No specific title is named here on purpose — the mechanism is established from
the register decode, but no software has been verified against it on this core.

**Done.** The timing came from `loaddat()`'s two call sites rather than the
function alone: the period write hangs off WinUAE's 2->3 state transition and
the volume write off 3->2, which are our `AUDIO_STATE_3 -> 4` and `4 -> 3`,
those states being the high and low sample of the fetched word. Silencing is
`audio_update_adkmasks()`, which builds its mask from `adkcon | (adkcon >> 4)`
with no special case for the last channel — so channel 3 is silenced by its own
attach bits even though `loaddat()` returns early for it and it modulates
nothing. That asymmetry is deliberate and is asserted.

Two things deliberately not carried over, both recorded in the commit: WinUAE's
period clamp, because this core does not clamp a CPU write to AUDxPER either and
clamping only the modulated path would make the two disagree; and suppressing
the modulator's own sample buffer load, because the output is masked to zero
regardless.

Bench: `rtl/sim/paula/tb_paula_audio_attach.sv`, in CI, eleven checks. Note also
T14 — with attach implemented, ADKCON not being cleared at boot stopped being
dormant, and that has been fixed on the userspace side.

## T5 — The rest of the 2005 CIA list  [SIM] — [DONE 2026-08-31]

Still true, from the same header:

- CIA-B serial data register — `ciab.v:263` hardwires `.ser(1'b0)`, `:134`
  confirms "not implemented in this simplified version".
- Port B for CIA A — `ciaa.v:340` "simplified, mostly unused in Amiga".
- PB6/PB7 toggling by timer A/B — `cia_timera.v:52-53`, `cia_timerb.v:51-52`.

All cold-path and small. The serial register has the most software exposure.

**All three done.** The serial register looked like it needed the CNT pin, which
would have meant undoing T1's revert, and it does not: WinUAE `cia.cpp` shifts
inside the timer A underflow path, gated on
`(cr & (CR_SPMODE | CR_RUNMODE)) == CR_SPMODE`, because in output mode the CIA
*generates* CNT rather than receiving it. CIA-B's CNT goes to the expansion bus
and is unconnected on a stock Amiga, so input mode has no source on real hardware
either. Output mode is implemented, input mode is explicitly inert, and SP now
reaches the interrupt controller instead of `.ser(1'b0)`. T1's revert and its
guard are untouched.

CIA-A port B had no output register at all -- a PRB write was dropped and a read
returned the pins whatever DDRB said. PB6/PB7 are now driveable by the timers
(PBON, and OUTMODE choosing pulse or toggle). Note what those two pins are on
CIA-B: /SEL3 and /MTR. A program setting PBON there drives drive-select and motor
from a timer, which is exactly what a real Amiga does. Faithful, not safe.

None of the new state is captured in `ss_state`, and `rtl/ss_state.vh` explains
why: 19 bits of padding are free before the state section lengthens, the new
state is 23, and lengthening it moves every section after it -- the 1.2 case in
`ss_ctrl.v`'s version notes, which would refuse every save file on disk.

Bench: `rtl/sim/cia/tb_cia_leftovers.sv`, in CI, eleven checks.

## T6 — `HHPOSR` is not implemented at all  [SIM] — [DONE 2026-08-31, RE-ENABLED 2026-09-02]

No occurrence anywhere in `rtl/`. ECS register, limited exposure, but WinUAE
implements it and light-pen-aware code reads it. Cheap, and easy to bench.

**Done.** WinUAE returns the light pen latch when one is armed and `hhpos`
otherwise, masked to `0xff`, and refuses the read without ECS Agnus. `hhpos` is
assigned `agnus_hpos` every colour clock except in BEAMCON0 DUAL mode, where it
free-runs and HHPOSW (`$1D8`) reseeds it. This core has no DUAL mode, so HHPOSR
is exactly the horizontal half of VHPOSR — same decrement, same ERSY case at the
wrap, same freeze — and is implemented by sharing that expression rather than
duplicating the counter.

HHPOSW stays undecoded: with `hhpos` not free-running there is nothing for a
write to hold, and a register that accepted a value and then ignored it would be
worse than one that is absent.

Bench: `rtl/sim/beamcounter/tb_beamcounter_hhposr.sv`, in CI. It asserts the
VHPOSR equality, which holds only while DUAL mode is absent — so if DUAL is ever
added, that bench is where it will say so.

---

## T7 — Characterise what the beam counter still gets wrong  [SUITE]

`06f30af` improved the readback but did not close the class. Upstream's own
measurement, from their commit message:

    before   191 of 304 probe rows read +1 CCK, 76 read 0, one reads -226
    after    268 of 304 read 0

**36 of 304 rows still fail**, and `ersy1` is correct on only 14 of 16.

This is a measurement, not a change, and it needs no reference Amiga: run the
vAmigaTS VPOS suite on the core and score against the values vAmigaTS itself
publishes. Do this before any other beam-counter work — it decides whether T9
and the other HOT items are worth their timing cost.

## T8 — Audit address decodes for over-breadth  [SIM]

`74d6ce0` narrowed Gary's custom-register decode from all of `$C0-$DF` to
`$DFxxxx`. That is a class: a decode that is too wide silently claims addresses
belonging to something else, and the symptom appears in the victim, not the
culprit.

Audit `gayle.v`, `cart.v`, `cdtv_bridge.v`, `akiko.v` against the hardware maps.
`gayle.v:55` already records one deliberate omission (`$DA8000` IDE INTREQ, "not
implemented as scsi.device doesn't use it") — fine, but it should be the only
one.

**Audited 2026-08-31. No over-broad decode found beyond the one `74d6ce0` had
already fixed.** Recording the result so nobody audits it twice.

Checked: `gary.v`'s selects (the generators for all of the above), `gayle.v`'s
internal nibble decodes, `cart.v`, and the `akiko.v` / `cdtv_bridge.v` register
decodes. Every one is either an exact nibble compare or matches the device.

- `sel_rtc` looks wrong at first — 64 KB for sixteen nibble registers — and is
  not. WinUAE `memory.cpp` maps `clock_bank` at `0xDC` for one bank with
  `startmask 0xdc0000`, so the clock really does mirror across the whole
  `$DC0000-$DCFFFF`. That is the device, not the decode.
- **There is a real overlap, and it is resolved downstream rather than in the
  decode.** `sel_cdtv_nvram` (`$DC8000-$DCFFFF`) sits inside `sel_rtc`, and both
  assert together. `cpu_wrapper`'s `cpu_din` mux takes `cdtv_selack` ahead of the
  chip bus and `sel_cdtv_nvram` is one of its terms, so an NVRAM read wins and
  the clock's contribution to the wired-OR at `minimig.v:1364` is discarded. It
  works, but nothing said so; `gary.v` now does. Anything else put in
  `$DC1000-$DCFFFF` without that selack short-circuit gets its data OR'd with a
  clock nibble, and the symptom appears in the new device.
- Two comments were wrong rather than the logic. `gayle.v`'s port comment said
  `$DExxxx` when `gary.v` gives it only `$DE1xxx` — a twelve-bit decode, so
  `sel_gayleid`'s re-check of `addr[15:12]` is redundant rather than load
  bearing. And `sel_rtg` was commented `$B8xxxxx`, one x too many. Both fixed.

---

## T9 — NTSC line length is wrong for every NTSC title  [SUITE + FIT] — [REVERTED ON HARDWARE 2026-09-02]

`agnus_beamcounter.v:88` and `:318`:

    parameter HTOTAL_VAL = 8'd227 - 8'd1;  // NTSC 227.5 CCKs is not supported
    reg long_line;  // (actually long lines are not supported yet)

NTSC alternates 227 and 228 CCK lines to average 227.5; the core does 227 flat.
The `long_line` register exists and toggles — nothing consumes it.

Largest single accuracy gap here, affecting every NTSC title's raster timing,
with the scaffolding already present. Also squarely in the hot path, so it is
gated on T0. `VSSTOP_VAL = 5` is the same story vertically: "PAL vsync width:
2.5 lines (NTSC: 3 lines - not implemented)".

**Done, but not yet fitted.** The toggle condition here was already right and
already matched WinUAE's; what was missing was the other half,
`maxhpos = maxhpos_short + lol`. So `htotal` keeps meaning the short-line length
and a new `htotal_cck` adds `long_line` to it.

One thing that would have made this half-work: `htotal_out` had to carry the
effective length rather than the short one, because `agnus.v:483` derives the DMA
slot lookahead wrap from `htotal[8:1]` and would otherwise have wrapped the slot
grid one colour clock early on every long line. VPOSW bit 7 now writes
`long_line` too, which it did not before -- WinUAE `custom.cpp:7712`.

Bench: `rtl/sim/beamcounter/tb_beamcounter_longline.sv`, in CI. Eight NTSC lines
sum to 1812 last-colour-clock values, which is the 227.5 average, and PAL,
BEAMCON0 PAL and LOLDIS are each held flat.

**It went to hardware, and it had to be switched back off.**  [2026-09-02]

On a PAL machine running Flink the picture was blurred and the OSD was drawn
twice with a horizontal offset. Three fixes were reasoned out of WinUAE across
two days -- the HBSTRT bit 10 revert, the VPOSW reset, and a re-read of the
toggle condition -- and none of them changed the fault. What identified it was
restoring `amigacd.rbf.pre-accuracy-0901` from the MiSTer's `_Test/` directory
and A/B testing on the machine: that core was clean immediately. One core swap,
no compile, and it succeeded where three rounds of reading had failed. Do that
first next time.

Two things were wrong, and only one of them is a bug in the usual sense.

**`pal` in this module is BEAMCON0 bit 5, not the machine's video setting.** It
resets to `~ntsc` and is then overwritten by any program that writes BEAMCON0.
Flink sets VARBEAMEN and does not set bit 5 -- which a program that programs
every beam register explicitly has no reason to do -- so `pal` went to 0 on a
PAL machine and the alternation ran. Every other PAL line became 228 colour
clocks instead of 227.

**And that is what WinUAE does too.** The guard at `custom.cpp:10959` is exactly
`!(new_beamcon0 & BEAMCON0_PAL) && !(new_beamcon0 & BEAMCON0_LOLDIS)`, and
`setmaxhpos()` adds `lol` to a programmed `maxhpos` unconditionally. The
implementation was faithful. It was still wrong for this platform: WinUAE
renders into a host window that resamples freely, and the MiSTer feeds a
fixed-rate scaler that cannot absorb an alternating line length. The blur is the
scaler resampling; the doubled OSD is the same thing at OSD scale.

So the accuracy is real and unshippable as it stands. `LONG_LINES` now defaults
to `1'b0` (and `HHPOSR_DECODE` with it, only to keep the next hardware test to
one variable), which makes the synthesised video path bit-identical to the core
confirmed good. `rtl/sim/beamcounter/tb_beamcounter_longline.sv` overrides the
parameter to 1, so the feature stays covered in simulation.

Anyone picking this up: the open question is not whether the RTL matches WinUAE.
It does. It is whether the MiSTer video pipeline can be made to accept a
227.5-colour-clock line at all -- and that is a question for the scaler and
`video.cpp`, not for `agnus_beamcounter.v`. Until that is answered, leave the
parameter off.

`VSSTOP_VAL` is untouched and still open.

## T10 — Is the STRHOR hack still right after `7ce2980`?  [SIM] — [DONE 2026-08-31]

`agnus.v:532`: `assign strhor_paula = hpos==(6*2+1) ? 1'b1 : 1'b0; //hack`

`7ce2980` has just advanced the DMA slot grid a colour clock **and moved STRHOR
with it**. Whether this hand-tuned constant is still correct after that is an
open question, and it would surface as a subtle raster artefact rather than a
crash — the hardest kind to notice.

**Answered, and the premise was a misreading.** `7ce2980` moved `strhor_denise`
to `hpos_slot` and deliberately left `strhor_paula` alone. Its own message says
so: "hde and strhor_paula are deliberately left on the raw counter ...
strhor_paula is an existing hack that is out of scope here." Denise had to move
because it has no `hde` port and takes its whole horizontal phase from that
strobe; Paula's does not.

Checked for a race as well, since the grid moving under a strobe that did not is
the obvious way this could still bite. `strhor_paula` latches Paula's per-line
DMA request registers, and the earliest consumer is the channel 0 audio slot at
`hpos_slot` `9'b0001_0010_1` in `agnus_audiodma.v` -- colour clock 18 on the
grid, 17 raw, against the strobe at raw colour clock 6. Eleven colour clocks of
margin; a one colour clock shift does not close it.

The constant is still a hand-tuned hack with no recorded derivation. That is a
separate question from whether `7ce2980` disturbed it, and it did not. Written up
at the site.

## T11 — Bitplane pointer write delay  [SIM] — HOT — [SCOPED 2026-08-31, NOT ATTEMPTED]

`agnus_bitplanedma.v:223`: "TODO high bitplane pointer probably needs a delay
(writing to pointer doesn't seem to take effect next cycle ...)".

**Scoped against WinUAE, and it is not a delay on this register.** The behaviour
is in the inherited `TODO` file, from Toni Wilen: writing BPLxPT when exactly the
next cycle has DMA to the matching BPLxDAT, the write goes nowhere, and the same
is true of sprite and blitter registers.

WinUAE models it as a four-slot RGA pipeline -- `custom.cpp`'s
`rga_pipe[(slot + rga_slot_first_offset) & 3]`, with `write_rga_update()`
capturing the pointer into the pipeline ahead of the access, and even modelling
the collision case ("DMA address pointer conflict causes both old and new address
to become old OR new"). A write landing after the capture is not seen by that
access, which is the "goes nowhere".

Reproducing that means a pipelined address capture on every DMA channel --
bitplane, sprite, blitter, audio, disk -- because the TODO says the same is true
of all of them. That is a chipset-wide rearchitecture in the hot path, on a
design with 0.117 ns of setup margin, and the software that demonstrates it is
the TLC PowerTrax demo, which cannot be run here.

**Not attempted, deliberately.** A one-line delay on this register would not
reproduce the behaviour and would perturb the fetch for nothing. Recorded at the
site so the next person does not start there.

## T12 — Superhires scroller select  [SIM] — HOT — [BLOCKED: needs a display]

`denise_bitplane_shifter.v:113`: `sh_select = {aga, scroll[0], 1'b1};` with "MSB
bit should probably be 0, this is a hack for kickstart screen", and `:108` "TODO
test if this is correct".

**WinUAE cannot settle this one, checked 2026-08-31.** The superhires scroller is
Minimig's own compensation for a Denise pipeline that is not cycle exact -- an
eight-deep sub-pixel delay line with a selected tap. WinUAE has no corresponding
structure, because it computes pixel positions directly rather than delaying a
stream to line them up. So there is nothing to compare a tap against: the right
tap is a property of THIS implementation's latency, and the only instrument that
can read it is the display.

Re-tagged from SIM to blocked. A bench can only assert whatever tap is written
into it, which is not a test.

One observation for whoever gets to a screen, recorded at the site: the lores
case selects `{aga, scroll[1:0]}`, an AGA base of 4 plus the scroll, while the
hires case selects `{aga, scroll[0], 1'b1}`, an AGA base of 5 or 7. The bases
differ. Whether that is the bug or the compensation is exactly the question, and
guessing at it is how the light pen trigger cost two rounds before someone
measured it.

---

## Ours, not upstream's

## T13 — The SNAC light gun trigger is not wired  [TITLE] — [DONE 2026-08-31]

`support/lightpen/amiga_lightpen.cpp:133` — the button routing was deliberately
put in the shared module so "the SNAC gun can use the same route **when its
trigger is wired up**". It never was. The USB GunCon path works; the user-port
one aims but cannot fire.

Small userspace change, and the one light-pen item that is actually testable —
unlike the position path, which is blocked on a gun that locks to the display.

## T14 — Three registers the boot ROM never clears  [TITLE] — [DONE 2026-08-31]

`support/minimig/minimig_boot.cpp:318`, `:321`, `:349` — CLXCON (`$dff098`),
ADKCON (`$dff09e`) and BPLCON3 (`$dff106`) are commented out with bare TODOs
while their neighbours are written. Anything relying on a clean boot state gets
whatever the previous title left behind. Note T4 is the ADKCON one: if audio
attach is implemented, leaving ADKCON uninitialised at boot becomes a live bug
rather than a dormant one.

## T15 — `minimig_share` file actions  [TITLE] — [DONE 2026-08-31]

`minimig_share.cpp:851`, `:858` — `ACTION_SET_PROTECT` and `ACTION_SET_COMMENT`
log "unimplemented". Directory sharing works; file attribute changes silently do
nothing.

## T16 — Deferred CDDA pump teardown  [TITLE] — [DONE 2026-08-31]

Phase 33 happened. `akiko_cd32.cpp:916` tears down an in-flight pump, `:1159`
arms it from `cd_cdda_lba_next/end`, and `:1688` is the streaming section.

Only two stale comments survived it, both now corrected: `:2686` claimed the
teardown was still to come, and the `cd_audio_timeout` legend at `:285` still
called the `-1` arm a "placeholder until Phase 33". The teardown is `cmd_stop()`
clearing `cd_cdda_lba_next/end` and `cd_cdda_drv`; the `-1` to `-2` advance
those comments wrapped was always correct.

## T17 — Trim the save state diagnostic scaffolding  [SIM + FIT] — [INVESTIGATED 2026-08-31, NOT TRIMMED]

`ss_ctrl.v` is 2,530 lines, and the save state notes already flag the peek
window, fault and interrupt latches and free-running counters as large and
trimmable. That was tidiness when it was written.

With T0 established it is more than tidiness: **removing logic that exists only
to diagnose a bug which has since been fixed is the cheapest slack available.**
It costs no capability, and unlike every other timing idea on this list it
cannot break behaviour, because behaviour does not depend on it.

Keep whatever the ssdiag sub-channel in `support/minimig/minimig_ssdiag.cpp`
still consumes; drop the rest, and re-fit to measure what came back.

**Promoted 2026-08-31 on new evidence.** T0 action 2 measured the current worst
setup paths, and seven of the ten end inside `ss_ctrl` — the worst of them all
at +0.117, from the DDR3 read into `state.S_L_KICK_CHK`. So this is not slack
gathered from wherever it happens to be lying about; it is the binding logic.
Two shapes to aim at:

- the restore-side load states, where a DDR3 read feeds `S_L_KICK_CHK` and
  `S_L_MAGIC` — four of the worst ten, including both of the top two
- the header and index arithmetic, `hdr_length[12]` into `rd_idx[*]` and
  `rd_data[52]` into `rd_idx[1]` — four more

Neither is diagnostic scaffolding as such, so the trim alone may not reach them.
Measure after trimming before deciding whether the load path also needs a
pipeline stage. T22's bench is what makes any of this safe to believe.

### What the trim actually found  [2026-08-31]

**There is nothing to trim, and the premise that this is "the cheapest slack
available" is wrong.** Recorded in full because it is a conclusion nobody should
have to reach twice.

The reasoning was: `ss_ctrl.v` is 2,530 lines, much of it diagnostics for a bug
fixed in 2026-08, so removing it costs no capability and cannot break behaviour.
Both halves are true. What does not follow is that any of it is *costing*
anything.

- Of the module's seventy ports, exactly two had no consumer in `Minimig.sv`:
  `peek_acks` and `peek_timeout`. **Quartus was therefore already removing them
  and the counters behind them**, so deleting them buys nothing that has not
  already been bought.
- And they are not dead. `rtl/tb/ss_ctrl_tb.v` asserts on both -- "peek did not
  time out" and "peek counted its acks", the latter expecting exactly 8. They
  are the bench's observation points into the peek DMA path. Deleting them
  removes two real assertions in exchange for nothing.
- Everything else -- the peek window, `pc_snapshot`, `kick_pair`, `dbg_idx`, the
  state and change-count latches -- is consumed by
  `support/minimig/minimig_ssdiag.cpp`, which this item says to keep.

So the only lever left is to stop *consuming* it: delete the ssdiag sub-channel
and then the scaffolding behind it. That is a product decision about whether the
save state diagnostics are still wanted, not a tidy-up, and it should be taken
deliberately rather than as a way to find timing.

None of which would reach the binding path anyway. T0 action 2 put seven of the
ten worst setup paths inside `ss_ctrl`, and they are the restore-side load states
and the header/index arithmetic into `rd_idx` -- the load path itself, not
anything diagnostic. If `ss_ctrl` has to give up time, it is that path that needs
a pipeline stage, and T22's bench is what would make such a change safe to
believe.

**One trap found on the way.** `rtl/ss_ctrl.v` exists in both repos as
byte-identical mirrors; only the userspace copy has the benches, in `rtl/tb/`.
A change applied to the core copy alone builds and fits perfectly and silently
loses its test coverage. Change both, and run `rtl/tb/ss_ctrl_tb.v` after.

## T18 — The save state "not captured" list is stale  [no code] — [DONE 2026-08-31]

The record from 2026-08-18 lists AGA colour banks, Akiko/CD state and fast RAM
as "not captured at all". **All three have since been implemented:**

- `denise_colortable.v:21` — "The colour table is 256 entries deep -- eight AGA
  banks", with an 8-bit `ss_clut_addr` and 32-bit data
- `akiko.v` — 61 `ss_` references, plus `rtl/sim/akiko/tb_akiko_savestate.sv`
- `ss_ctrl.v:158-254` — Zorro II fast RAM, an 8 MB window, and a conditional
  payload section in both directions

Someone reading that list would go and re-implement finished work.

**Corrected.** Note the stale record is not a file in either repo — it is the
`savestate-restore-state-2026-08-18` memory file, whose "not captured at all"
bullet has been replaced with the three citations above and a "do not go and
re-implement them" warning. The title list was corrected in the same pass (Jim
Power, Cannon Fodder, Flink and Castlevania AGA, not just Arabian Nights), and
its stale −0.058 ns `cpu_cache_new` timing note was replaced with the seed 10
numbers from T0. "The Kickstart gate can become strict now that the fingerprint
matches" is genuinely still open and stays on that list.

## T19 — Doc staleness  [no code]

Beyond T0's update to `sdram-timing-headroom.md`: `docs/upstream-pins.md` was
found this week with no `MiSTer-devel/Main_MiSTer` row at all and a Minimig-AGA
pin two syncs out of date. Both fixed. Worth a habit of checking it at the top
of every sync rather than the bottom.

---

## T21 — CI has no syntax gate, so a parse error costs a 35-minute fit  [SIM] — [DONE 2026-08-31]

Every step in `rtl-sim.yml` compiles a small subset of files for one bench.
**Nothing ever parses `Minimig.sv`, and most of `rtl/` is never parsed at all.**
A plain syntax error therefore passes CI in full and surfaces only in Quartus.

That is not hypothetical. On 2026-08-27 a missing comma in a `Minimig.sv` port
list produced

    Error (10170): Verilog HDL syntax error at Minimig.sv(1728) near text: "."
    Quartus Prime Full Compilation was unsuccessful. 4 errors

18 seconds into a fit that had been waited on, when the same error is visible in
about two seconds from a standalone parse.

Measured across the whole tree, so this is known to work rather than assumed:

    clean = 48    unknown-module-only = 35    syntax errors = 1

The 35 are the expected consequence of parsing a module standalone and are
filtered by matching only on `syntax error`, `has already been declared` and
`Errors in port declarations`. The recipe is the one used by hand throughout the
2026-08 work:

    iverilog -g2012 -t null -o /dev/null <file> 2>&1

**Done.** `syntax_check.sh` at the repo root, wired in as the first step of
`rtl-sim.yml` before the benches. It covers `rtl/**` (excluding `rtl/sim/`) plus
the root sources, which is broader than the survey above: **99 files, 56 clean,
43 unknown-module-only, 0 syntax errors** once T21a was fixed. Verified to fail
the way it should by injecting a stray `.` into the `cdda` instantiation, which
it reported as `Minimig.sv:2879: syntax error` with exit 1.

### T21a — `rtl/cdda.v` has a parameter with no default  [DONE 2026-08-31]

The single syntax hit above is real, not a false positive:

    module cdda #(parameter CLK_RATE)      // rtl/cdda.v:2

Quartus accepts it; Icarus rejects it under both `-g2012` and `-g2005-sv`. There
is exactly one instantiation, `Minimig.sv:2878` (`cdda #(28375160)`), so giving
it `= 0` costs nothing and makes the file parse anywhere. Needed before T21 can
be clean, and worth doing regardless for portability.

**Done** — `parameter CLK_RATE = 0`. The instantiation still passes the real
value explicitly, so nothing takes the default; it exists only so the file
parses standalone.

## T22 — `cpu_wrapper.v` has no bench  [SIM] — [DONE 2026-08-31]

Of the modules surveyed it is the only one with no simulation coverage at all,
and it is not a quiet corner. It holds:

- the **save state CPU park** — `ss_bus_settled & ~ss_cpu_hold`, which was very
  nearly deleted during the `74d6ce0` merge, where upstream's side of the
  conflict carried a plainer `clkena_p_base` and taking theirs would have
  removed it silently
- the **stock-speed throttle** the CPU Turbo row drives, whose cooldown constant
  upstream has already had wrong by a factor of two (`148931d`)
- the consumer of the **chip-slot guard** from `114ab43`

The park is the part most worth pinning down. Its own comment explains that
reading `ss_at_boundary` live rather than latched releases the CPU while
`ss_ctrl` is still writing chip RAM — a restore that lands on half-rewritten
memory, indistinguishable from the mid-instruction restore the park exists to
prevent. That is subtle, it is load-bearing for save states, and a future merge
will eventually break it again the way this one almost did.

A bench that freezes and resumes across a range of boundary conditions would
have caught the merge conflict automatically instead of relying on someone
reading both sides carefully.

**Done.** `rtl/sim/cpuwrap/tb_cpu_wrapper_park.sv`, in CI, twenty checks over all
three: the park latching and surviving the boundary going away, releasing on
`ss_arm`, not carrying into a second park, and not deadlocking a CPU armed while
never reaching a boundary; the bus-settled gate and each of the four sources
that can settle it; the cooldown being exactly four cycles under stock speed and
absent otherwise; and `chipreq` asserting for a plain chip access but not while
either bridge has it.

Both CPU cores are stubbed — `rtl/sim/cpuwrap/cpu_core_stubs.v`. None of this
needs a working 68000, only control over `busstate` and `ss_at_boundary`.

Verified by mutation rather than by assertion alone: reverting the park to a
live gate, setting the cooldown back to 9, and dropping `~cdtv_selack` from
`chipreq` fails five of the twenty checks, and the live-gate mutation shows the
CPU running freely through the whole window a restore needs it stopped.

## T20 — The legacy seed scripts ranked on setup alone  [DONE 2026-08-31]

Recorded because the failure mode is worth remembering, not because work
remains.

`seed_sweep.sh`, `refit_until_good.sh` and `refit_one.sh` all extracted
"Worst-case setup slack" and never looked at hold. `refit_until_good.sh` was the
worst of the three: it stopped at the first seed clearing a setup target, so it
did not merely rank wrong, it stopped looking.

This is not hypothetical. Seed 16 once topped a list at +0.113 setup while
carrying -0.346 hold and nearly shipped on the better number; and in the
2026-08-31 sweep the first candidate was seed 1 at **+0.134 setup / -0.495
hold**, which those scripts would have selected, assembled, and called good.

Fixed: the two sweepers now delegate to `seed_sweep_both.sh`, which requires
both slacks positive. `refit_one.sh` keeps its job — fit one named seed and
produce a bitstream — but reports both slacks, waits for any in-flight Quartus
run, and exits non-zero when either is negative.

They delegate rather than being deleted because the names are in older notes and
in muscle memory. Typing the old name should do the right thing, not fail and
invite someone to recover the original from git.

---

## Order

**Re-ranked 2026-08-31 (second pass).** T1, T16, T18, T21 and T21a are done. T0
turned out not to be a gate, which is what most of the original ranking hung
off: nothing below is ordered by how much timing slack it buys any more.

1. **T17** — promoted, on evidence gathered after the first re-rank. Seven of
   the ten worst setup paths now end inside `ss_ctrl`, so trimming the
   diagnostic scaffolding is no longer generic slack-hunting: it is work on the
   logic that is actually binding. It still cannot change behaviour, which makes
   it the only timing item on this list with no accuracy risk.
2. **T3, T19** — the remaining little-or-no-code items. T3 is a decision rather
   than a task: port `rtl/sim/chipset` to Icarus and add it to the workflow —
   which is cheaper now that a parse gate runs ahead of the benches — or delete
   it. Leaving it is how the last two rotted.
3. **T2, T4, T6** — cold path, evidenced, simulation-verifiable. T4 is the best
   value here.
4. **T22** — before the next upstream sync rather than after. Its value is
   catching the merge that quietly removes the save state park, and that is only
   useful if it exists beforehand. It also overlaps T17: both are about
   `ss_ctrl`, and a bench that freezes and resumes is what makes trimming it
   safe to believe.
5. **T7** — measurement only; it justifies or kills T9 and the other HOT work.
6. **T13, T14, T15** — our own loose ends, all TITLE-verifiable.
7. **T17** — previously ranked as "the cheapest move against T0". With T0 not a
   blocker this is ordinary cleanup: still worth doing, still the cheapest slack
   if a HOT item later needs some, but no longer a prerequisite for anything.
8. **T5, T8** — steady cold-path accuracy work.
9. **T9, T10, T11, T12** — each needs its own fit, and T7 in hand first.

## What is not on this list

Anything justified by MiSTer having a bigger FPGA alone. There is area headroom
— 70% ALMs, 50% BRAM, 62% DSP — but area is not what this design is short of,
and `docs/sdram-timing-headroom.md` established that a year of evidence ago.
Check the current slacks before proposing anything on that basis.
