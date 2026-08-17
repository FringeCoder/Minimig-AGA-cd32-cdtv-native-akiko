#!/usr/bin/env bash
#
# Build and run tb_ss_regbus_mux under Icarus. Run from anywhere:
#
#     bash rtl/sim/ssmux/run.sh
#
# Exits non-zero unless the bench prints RUN: PASS.
#
# The file list is here rather than in the workflow because it is long: this
# bench elaborates the real minimig and everything under it. Two modules are
# stood in for -- see sim_stubs.v -- and neither is on the register bus.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RTL="$(cd "$HERE/../.." && pwd)"
OUT="$HERE/build"

mkdir -p "$OUT"

# The pin boilerplate is generated from minimig.v's port list. Fail loudly if
# the checked-in copy no longer matches the module, rather than simulating a
# stale one.
python3 "$HERE/gen_minimig_pins.py" --check

iverilog -g2012 -o "$OUT/tb_ss_regbus_mux" -I"$HERE" -s tb_ss_regbus_mux \
	"$HERE/tb_ss_regbus_mux.sv" \
	"$HERE/sim_stubs.v" \
	"$RTL/sim/cache/dpram_sim.v" \
	"$RTL/amiga_clk.v" \
	"$RTL/ss_regshadow.v" \
	"$RTL/minimig.v" \
	"$RTL"/agnus*.v \
	"$RTL"/paula*.v \
	"$RTL"/denise*.v \
	"$RTL"/cia*.v \
	"$RTL/gary.v" \
	"$RTL/gayle.v" \
	"$RTL/ide.v" \
	"$RTL/cart.v" \
	"$RTL/userio.v" \
	"$RTL/minimig_bankmapper.v" \
	"$RTL/minimig_m68k_bridge.v" \
	"$RTL/minimig_sram_bridge.v" \
	"$RTL/minimig_syscontrol.v" \
	"$RTL"/MiSTerFloppy*.v \
	"$RTL/cdtv_bridge.v" \
	"$RTL/cdtv_nvram.v" \
	"$RTL"/A2065/a2065*.v

cd "$OUT"
vvp "$OUT/tb_ss_regbus_mux" | tee "$OUT/out.log"
grep -q "RUN: PASS" "$OUT/out.log"
