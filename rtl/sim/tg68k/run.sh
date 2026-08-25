#!/usr/bin/env bash
# Build and run the TG68K savestate restore testbench under GHDL.
#
# Run from the repository root:
#     rtl/sim/tg68k/run.sh
#
# All three GHDL flags are load-bearing on this source: -fsynopsys for
# std_logic_unsigned, -fexplicit for the overloaded "=", --std=93c for the
# vintage. --ieee-asserts=disable only silences the Synopsys packages'
# metavalue warnings, which the kernel produces by the thousand at time zero
# and which say nothing about the design.
#
# The testbench reads rtl/sim/tg68k/tg68k_ss_prog.hex relative to the working
# directory, which is why this has to run from the root.
set -euo pipefail

WORKDIR=${WORKDIR:-build/ghdl}
GHDL=${GHDL:-ghdl}
FLAGS=(--workdir="$WORKDIR" --std=93c -fsynopsys -fexplicit -frelaxed)

mkdir -p "$WORKDIR"

"$GHDL" -a "${FLAGS[@]}" \
	rtl/tg68k/TG68K_Pack.vhd \
	rtl/tg68k/TG68K_ALU.vhd \
	rtl/tg68k/TG68KdotC_Kernel.vhd \
	rtl/sim/tg68k/tg68k_ss_tb.vhd

"$GHDL" -e "${FLAGS[@]}" tg68k_ss_tb
"$GHDL" -r "${FLAGS[@]}" tg68k_ss_tb --ieee-asserts=disable "$@" | tee "$WORKDIR/tg68k_ss_tb.log"

grep -q "RUN: PASS" "$WORKDIR/tg68k_ss_tb.log"
