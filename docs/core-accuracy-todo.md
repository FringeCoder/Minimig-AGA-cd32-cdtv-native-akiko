# AmigaCD core: accuracy and cleanup TODO

Written 2026-08-31 after the `74d6ce0` sync. Every item cites a file and line or
an upstream commit. Nothing here is inferred from behaviour alone.

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

## T0 — Timing headroom is the blocker for everything else  [FIT]

**This gates every HOT item below. Do not start one until this is understood.**

`docs/sdram-timing-headroom.md` (2026-08-06) already established that this core
has no headroom, that `sd_addr` was the repeated destination of the binding
path, and it designated a fix: split the chip and CPU address loads and select
at the output.

**That fix was done** — our `5cb32ea perf(sdram): split sd_addr into chip and
CPU halves, select at the output` — and the problem has returned anyway. Current
state, tip of `sync/minimig-upstream-74d6ce0`: **setup −0.347, hold −0.401**,
TNS −3.303 / −1.104. The last four builds needed two seed sweeps to close, and
seed 1 was rejected in one sweep for a −0.495 hold violation behind a healthy
setup number.

Actions:
1. Update `docs/sdram-timing-headroom.md` — it still says the designated fix is
   "not attempted. This is the one to reach for when the problem returns". It
   was attempted, it landed, and the problem returned regardless. Recording that
   is worth more than the original prediction.
2. Identify the current binding path from `Minimig.sta.rpt` and write it up the
   way that doc did. The destination may no longer be `sd_addr`.
3. Only then decide whether the four chipset commits in the pending merge are
   affordable. `7ce2980` (DMA slot grid advanced a colour clock) is the prime
   suspect and the obvious first thing to drop and re-fit.

---

## T1 — CIA CNT: guard the revert in CI  [SIM]

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

**Action: add a CI assertion.** A grep step in `rtl-sim.yml` that fails the
build if either count source is not the eclk form:

    rtl/cia_timera.v   assign count = eclk;
    rtl/cia_timerb.v   assign count = tmcr[6] ? tmra_ovf : eclk;

Seconds to run, and it converts a thing someone must remember into a thing the
pipeline refuses. Word the failure message so it explains *why* rather than just
reporting a missing string — the next person to hit it will be mid-merge and
will otherwise assume the assertion is stale and delete it.

The alternative — wiring CIA-A CNT to a synthesised keyboard clock so the
upstream commit can be taken properly — stays open but unscheduled. It is
speculative work on a cold path with no way to validate the result here, and it
buys nothing that the guard does not.

## T2 — `hbstrt_reg` is the wrong width  [SIM]

`agnus_beamcounter.v:229`, `:257`:

    reg [ 8:0] hbstrt_reg; // not correct size, this should have [10:0]
    HBSTRT [8:1] : hbstrt_reg <= {data_in[ 7:0], 1'b0}; // TODO fix this

Programmed horizontal blank start truncates in ECS `varbeamen` mode. Small,
self-admitted, and testable in simulation: extend the existing beamcounter bench
to write HBSTRT above 9 bits and check the blanking edge lands where asked. The
compare at `:540` is against `hpos`, so widening the register means revisiting
that too.

## T3 — The last bench outside CI  [SIM]

`rtl/sim/chipset` is the only one of ten bench directories not in the workflow.
It is ModelSim-only, which is exactly how the two CDTV benches silently stopped
compiling at the `b265a3b` merge and stayed broken until last week.

Either port it to Icarus and add it, or delete it. Leaving a bench that nothing
runs is how the last two rotted.

---

## T4 — Paula channel modulation (ADKCON attach bits) is absent  [SIM]

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

## T5 — The rest of the 2005 CIA list  [SIM]

Still true, from the same header:

- CIA-B serial data register — `ciab.v:263` hardwires `.ser(1'b0)`, `:134`
  confirms "not implemented in this simplified version".
- Port B for CIA A — `ciaa.v:340` "simplified, mostly unused in Amiga".
- PB6/PB7 toggling by timer A/B — `cia_timera.v:52-53`, `cia_timerb.v:51-52`.

All cold-path and small. The serial register has the most software exposure.

## T6 — `HHPOSR` is not implemented at all  [SIM]

No occurrence anywhere in `rtl/`. ECS register, limited exposure, but WinUAE
implements it and light-pen-aware code reads it. Cheap, and easy to bench.

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

---

## T9 — NTSC line length is wrong for every NTSC title  [SUITE + FIT] — HOT

`agnus_beamcounter.v:88` and `:318`:

    parameter HTOTAL_VAL = 8'd227 - 8'd1;  // NTSC 227.5 CCKs is not supported
    reg long_line;  // (actually long lines are not supported yet)

NTSC alternates 227 and 228 CCK lines to average 227.5; the core does 227 flat.
The `long_line` register exists and toggles — nothing consumes it.

Largest single accuracy gap here, affecting every NTSC title's raster timing,
with the scaffolding already present. Also squarely in the hot path, so it is
gated on T0. `VSSTOP_VAL = 5` is the same story vertically: "PAL vsync width:
2.5 lines (NTSC: 3 lines - not implemented)".

## T10 — Is the STRHOR hack still right after `7ce2980`?  [SIM] — HOT

`agnus.v:532`: `assign strhor_paula = hpos==(6*2+1) ? 1'b1 : 1'b0; //hack`

`7ce2980` has just advanced the DMA slot grid a colour clock **and moved STRHOR
with it**. Whether this hand-tuned constant is still correct after that is an
open question, and it would surface as a subtle raster artefact rather than a
crash — the hardest kind to notice.

## T11 — Bitplane pointer write delay  [SIM] — HOT

`agnus_bitplanedma.v:223`: "TODO high bitplane pointer probably needs a delay
(writing to pointer doesn't seem to take effect next cycle ...)".

## T12 — Superhires scroller select  [SIM] — HOT

`denise_bitplane_shifter.v:113`: `sh_select = {aga, scroll[0], 1'b1};` with "MSB
bit should probably be 0, this is a hack for kickstart screen", and `:108` "TODO
test if this is correct".

---

## Ours, not upstream's

## T13 — The SNAC light gun trigger is not wired  [TITLE]

`support/lightpen/amiga_lightpen.cpp:133` — the button routing was deliberately
put in the shared module so "the SNAC gun can use the same route **when its
trigger is wired up**". It never was. The USB GunCon path works; the user-port
one aims but cannot fire.

Small userspace change, and the one light-pen item that is actually testable —
unlike the position path, which is blocked on a gun that locks to the display.

## T14 — Three registers the boot ROM never clears  [TITLE]

`support/minimig/minimig_boot.cpp:318`, `:321`, `:349` — CLXCON (`$dff098`),
ADKCON (`$dff09e`) and BPLCON3 (`$dff106`) are commented out with bare TODOs
while their neighbours are written. Anything relying on a clean boot state gets
whatever the previous title left behind. Note T4 is the ADKCON one: if audio
attach is implemented, leaving ADKCON uninitialised at boot becomes a live bug
rather than a dormant one.

## T15 — `minimig_share` file actions  [TITLE]

`minimig_share.cpp:851`, `:858` — `ACTION_SET_PROTECT` and `ACTION_SET_COMMENT`
log "unimplemented". Directory sharing works; file attribute changes silently do
nothing.

## T16 — Deferred CDDA pump teardown  [TITLE]

`akiko_cd32.cpp:2686` — "Phase 33 will tear down the CDDA pump here. For now
we...". Someone should confirm whether Phase 33 happened or whether this is
permanent.

## T17 — Trim the save state diagnostic scaffolding  [SIM + FIT]

`ss_ctrl.v` is 2,530 lines, and the save state notes already flag the peek
window, fault and interrupt latches and free-running counters as large and
trimmable. That was tidiness when it was written.

With T0 established it is more than tidiness: **removing logic that exists only
to diagnose a bug which has since been fixed is the cheapest slack available.**
It costs no capability, and unlike every other timing idea on this list it
cannot break behaviour, because behaviour does not depend on it.

Do this before attempting any HOT accuracy item. Keep whatever the ssdiag
sub-channel in `support/minimig/minimig_ssdiag.cpp` still consumes; drop the
rest, and re-fit to measure what came back.

## T18 — The save state "not captured" list is stale  [no code]

The record from 2026-08-18 lists AGA colour banks, Akiko/CD state and fast RAM
as "not captured at all". **All three have since been implemented:**

- `denise_colortable.v:21` — "The colour table is 256 entries deep -- eight AGA
  banks", with an 8-bit `ss_clut_addr` and 32-bit data
- `akiko.v` — 61 `ss_` references, plus `rtl/sim/akiko/tb_akiko_savestate.sv`
- `ss_ctrl.v:158-254` — Zorro II fast RAM, an 8 MB window, and a conditional
  payload section in both directions

Someone reading that list would go and re-implement finished work. Correct it.
Also still listed and worth re-checking: "the Kickstart gate can become strict
now that the fingerprint matches", and "only Arabian Nights verified" — since
then Jim Power, Cannon Fodder, Flink and Castlevania AGA have all been exercised.

## T19 — Doc staleness  [no code]

Beyond T0's update to `sdram-timing-headroom.md`: `docs/upstream-pins.md` was
found this week with no `MiSTer-devel/Main_MiSTer` row at all and a Minimig-AGA
pin two syncs out of date. Both fixed. Worth a habit of checking it at the top
of every sync rather than the bottom.

---

## T21 — CI has no syntax gate, so a parse error costs a 35-minute fit  [SIM]

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

Add it as a first step in `rtl-sim.yml`, before the benches — it is seconds of
runtime and it fails the cheap way.

### T21a — `rtl/cdda.v` has a parameter with no default

The single syntax hit above is real, not a false positive:

    module cdda #(parameter CLK_RATE)      // rtl/cdda.v:2

Quartus accepts it; Icarus rejects it under both `-g2012` and `-g2005-sv`. There
is exactly one instantiation, `Minimig.sv:2878` (`cdda #(28375160)`), so giving
it `= 0` costs nothing and makes the file parse anywhere. Needed before T21 can
be clean, and worth doing regardless for portability.

## T22 — `cpu_wrapper.v` has no bench  [SIM]

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

1. **T21 + T21a** — minutes of work, and it stops a whole class of error costing
   half an hour each. Do this first simply because everything else below is
   cheaper once it exists.
2. **T0** — gates every HOT item, and is the honest blocker on the pending
   merge. **T17 is the cheapest move against it** and should be tried first,
   since it removes logic rather than adding any.
3. **T1, T3, T18, T19** — small or no code. T1 belongs alongside T21 in practice:
   both are CI gates that turn something a person must remember into something
   the pipeline enforces, and T1's failure mode has already escaped once.
   T18 stops someone re-implementing finished work.
4. **T2, T4, T6** — cold path, evidenced, simulation-verifiable. T4 is the best
   value here.
5. **T7** — measurement only; it justifies or kills T9 and the other HOT work.
6. **T22** — before the next upstream sync rather than after. Its value is
   catching the merge that quietly removes the save state park, and that is only
   useful if it exists beforehand.
7. **T13, T14, T15, T16** — our own loose ends, all TITLE-verifiable.
8. **T5, T8** — steady cold-path accuracy work.
9. **T9, T10, T11, T12** — only with T0 resolved and T7 in hand.

## What is not on this list

Anything justified by MiSTer having a bigger FPGA alone. There is area headroom
— 70% ALMs, 50% BRAM, 62% DSP — but area is not what this design is short of,
and `docs/sdram-timing-headroom.md` established that a year of evidence ago.
Check the current slacks before proposing anything on that basis.
