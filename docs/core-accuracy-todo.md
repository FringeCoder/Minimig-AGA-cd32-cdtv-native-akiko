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
2. **Open.** Identify the current binding path and write it up the way that doc
   did. Note `Minimig.sta.rpt` is a summary report and carries no per-path
   detail — this needs a `quartus_sta` run with `report_timing -setup`, which is
   minutes against the existing netlist, not a re-fit. The destination is very
   likely no longer `sd_addr`: the last per-path capture we have,
   `setup_paths.rpt` from 2026-08-17, shows
   `ciab:CIAB1|regportb[6]` to `sdram_ctrl:ram1|sd_cas`.
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

## T16 — Deferred CDDA pump teardown  [TITLE] — [DONE 2026-08-31]

Phase 33 happened. `akiko_cd32.cpp:916` tears down an in-flight pump, `:1159`
arms it from `cd_cdda_lba_next/end`, and `:1688` is the streaming section.

Only two stale comments survived it, both now corrected: `:2686` claimed the
teardown was still to come, and the `cd_audio_timeout` legend at `:285` still
called the `-1` arm a "placeholder until Phase 33". The teardown is `cmd_stop()`
clearing `cd_cdda_lba_next/end` and `cd_cdda_drv`; the `-1` to `-2` advance
those comments wrapped was always correct.

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

**Re-ranked 2026-08-31 (second pass).** T1, T16, T18, T21 and T21a are done. T0
turned out not to be a gate, which is what most of the original ranking hung
off: nothing below is ordered by how much timing slack it buys any more.

1. **T3, T19** — the remaining little-or-no-code items. T3 is a decision rather
   than a task: port `rtl/sim/chipset` to Icarus and add it to the workflow —
   which is cheaper now that a parse gate runs ahead of the benches — or delete
   it. Leaving it is how the last two rotted.
2. **T0 actions 1 and 2** — the `sdram-timing-headroom.md` update, and the
   `quartus_sta` capture of the current binding path. Minutes each, and action 2
   is what says where the slack actually is now rather than where it was in
   2026-08.
3. **T2, T4, T6** — cold path, evidenced, simulation-verifiable. T4 is the best
   value here.
4. **T22** — before the next upstream sync rather than after. Its value is
   catching the merge that quietly removes the save state park, and that is only
   useful if it exists beforehand.
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
