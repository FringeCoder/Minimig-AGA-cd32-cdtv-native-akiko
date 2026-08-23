#!/usr/bin/env bash
# Re-fit the CURRENT netlist across candidate seeds, stopping at the first with
# real margin.
#
# Synthesis is not re-run: the seed only affects placement, and the map database
# in this tree already matches the source. The seed order is the ranking from
# the earlier full sweep, which is a prior rather than a promise -- that sweep
# measured a different netlist, which is exactly why its best seed came out
# negative here.
set -u
Q=/c/intelFPGA_lite/17.0/quartus/bin64
OUT=seed_sweep
mkdir -p "$OUT"
R="$OUT/refit.txt"
: > "$R"
TARGET=0.15

for SEED in 10 16 4 18 14 7 2 11 15 1; do
    "$Q/quartus_fit" --seed=$SEED Minimig > "$OUT/refit_fit_$SEED.log" 2>&1 || { echo "seed $SEED FIT FAILED" | tee -a "$R"; continue; }
    "$Q/quartus_sta" Minimig > "$OUT/refit_sta_$SEED.log" 2>&1
    s=$(grep -oE "Worst-case setup slack is [-0-9.]+" output_files/Minimig.sta.rpt | tail -1 | grep -oE "[-0-9.]+$")
    echo "seed $SEED  slack ${s:-unknown}" | tee -a "$R"
    ok=$(awk -v a="${s:-0}" -v t="$TARGET" 'BEGIN{print (a>=t)?1:0}')
    if [ "$ok" = "1" ]; then
        "$Q/quartus_asm" Minimig > "$OUT/refit_asm_$SEED.log" 2>&1
        cp output_files/Minimig.rbf "$OUT/Minimig_good_seed$SEED.rbf"
        echo "GOOD: seed $SEED at $s -- rbf saved" | tee -a "$R"
        exit 0
    else
        # Keep every positive build: if nothing clears the target, the best
        # measured seed still needs a bitstream and refitting it costs another
        # fifteen minutes for a placement already computed once.
        ok2=$(awk -v a="${s:-0}" 'BEGIN{print (a>0)?1:0}')
        if [ "$ok2" = "1" ]; then
            "$Q/quartus_asm" Minimig > "$OUT/refit_asm_$SEED.log" 2>&1
            cp output_files/Minimig.rbf "$OUT/Minimig_pos_seed$SEED.rbf"
        fi
    fi
done
echo "no seed reached $TARGET" | tee -a "$R"
