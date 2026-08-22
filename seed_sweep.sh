#!/usr/bin/env bash
# Placement seed sweep.
#
# This design fits within a few tenths of a nanosecond of its constraint, and
# three builds today came out NEGATIVE -- Quartus is deterministic, so the same
# source refits identically and only the seed moves the placement. Rather than
# reseed reactively each time a build fails, sweep once and keep the best.
#
# Synthesis is run ONCE: the seed only affects fitting, so each candidate is
# fit + timing + assemble, which is about two thirds the cost of a full compile.
# Every candidate's .rbf is kept, so the winner needs no re-fit to produce one.
set -u
Q=/c/intelFPGA_lite/17.0/quartus/bin64
OUT=seed_sweep
mkdir -p "$OUT"
RESULTS="$OUT/results.txt"

# Wait for any fit already in flight -- the one running now is seed 9 and its
# result counts as a data point like any other.
while tasklist 2>/dev/null | grep -qiE "quartus_(map|fit|asm|sta)"; do sleep 30; done

if [ -f output_files/Minimig.sta.rpt ]; then
    s=$(grep -oE "Worst-case setup slack is [-0-9.]+" output_files/Minimig.sta.rpt | tail -1 | grep -oE "[-0-9.]+$")
    echo "seed 9 (in-flight build)  slack ${s}" | tee -a "$RESULTS"
    [ -f output_files/Minimig.rbf ] && cp output_files/Minimig.rbf "$OUT/Minimig_seed9.rbf"
fi

# Synthesis once. Everything after this is placement only.
"$Q/quartus_map" Minimig > "$OUT/map.log" 2>&1 || { echo "SYNTHESIS FAILED" | tee -a "$RESULTS"; exit 1; }

for SEED in 1 2 3 4 5 6 7 10 11 12 13 14 15 16 17 18; do
    "$Q/quartus_fit" --seed=$SEED Minimig > "$OUT/fit_$SEED.log" 2>&1
    if [ $? -ne 0 ]; then echo "seed $SEED  FIT FAILED" | tee -a "$RESULTS"; continue; fi
    "$Q/quartus_sta" Minimig > "$OUT/sta_$SEED.log" 2>&1
    s=$(grep -oE "Worst-case setup slack is [-0-9.]+" output_files/Minimig.sta.rpt | tail -1 | grep -oE "[-0-9.]+$")
    "$Q/quartus_asm" Minimig > "$OUT/asm_$SEED.log" 2>&1
    [ -f output_files/Minimig.rbf ] && cp output_files/Minimig.rbf "$OUT/Minimig_seed$SEED.rbf"
    echo "seed $SEED  slack ${s:-unknown}" | tee -a "$RESULTS"
done

echo "== sweep complete ==" | tee -a "$RESULTS"
sort -k4 -g -r "$RESULTS" 2>/dev/null | head -5 | tee -a "$RESULTS"
