#!/usr/bin/env bash
# Placement seed sweep that ranks on BOTH slacks.
#
# The three scripts beside this one (seed_sweep.sh, refit_until_good.sh,
# refit_one.sh) all extract "Worst-case setup slack" and rank on it alone. That
# is how seed 16 once came out top of the list at +0.113 setup while carrying
# -0.346 hold, and very nearly got deployed on the strength of the better
# number. A hold violation is not a slower-clock problem -- it fails at every
# frequency -- so a build is only a candidate when BOTH slacks are positive.
#
# Synthesis is not re-run. The seed moves placement only, and the map database
# in this tree already matches the current source (a full compile just ran), so
# each candidate is fit + sta + asm.
#
# Seed order is the ranking from the earlier full sweep. That measured a
# different netlist, so it is a prior and not a promise -- its best seed came
# out negative on the next netlist it was tried against. Seed 14 is skipped: it
# is the current one and already measured at -0.091 / +0.244 on this netlist.
set -u
Q=/c/intelFPGA_lite/17.0/quartus/bin64
OUT=seed_sweep
mkdir -p "$OUT"
R="$OUT/both_slacks.txt"
TARGET=0.10          # both slacks must clear this to stop early

# Never two fits at once: they share db/ and output_files/ and interleave
# silently, and results from such a window cannot be trusted at all.
while tasklist 2>/dev/null | grep -qiE "quartus_(map|fit|asm|sta)"; do sleep 30; done

echo "== sweep started $(date '+%Y-%m-%d %H:%M') ==" | tee -a "$R"
echo "== baseline for this netlist: seed 14  setup -0.091  hold +0.244 ==" | tee -a "$R"

best_seed=""; best_min=-99

for SEED in 1 10 16 4 18 7 2 11 15 5 13 17 6 12 3; do
    "$Q/quartus_fit" --seed=$SEED Minimig > "$OUT/both_fit_$SEED.log" 2>&1 \
        || { echo "seed $SEED  FIT FAILED" | tee -a "$R"; continue; }
    "$Q/quartus_sta" Minimig > "$OUT/both_sta_$SEED.log" 2>&1

    # Read both slacks from THIS run's report.
    su=$(grep -oE "Worst-case setup slack is [-0-9.]+" output_files/Minimig.sta.rpt | tail -1 | grep -oE "[-0-9.]+$")
    ho=$(grep -oE "Worst-case hold slack is [-0-9.]+"  output_files/Minimig.sta.rpt | tail -1 | grep -oE "[-0-9.]+$")
    su=${su:-0}; ho=${ho:-0}
    mn=$(awk -v a="$su" -v b="$ho" 'BEGIN{print (a<b)?a:b}')

    printf "seed %-3s setup %-8s hold %-8s worst %s\n" "$SEED" "$su" "$ho" "$mn" | tee -a "$R"

    both_pos=$(awk -v a="$su" -v b="$ho" 'BEGIN{print (a>0 && b>0)?1:0}')
    if [ "$both_pos" = "1" ]; then
        # Keep every build where both are positive: refitting a placement
        # already computed costs another quarter hour for nothing.
        "$Q/quartus_asm" Minimig > "$OUT/both_asm_$SEED.log" 2>&1
        cp output_files/Minimig.rbf "$OUT/Minimig_both_seed$SEED.rbf"
        better=$(awk -v a="$mn" -v b="$best_min" 'BEGIN{print (a>b)?1:0}')
        [ "$better" = "1" ] && { best_min=$mn; best_seed=$SEED; }

        clears=$(awk -v a="$su" -v b="$ho" -v t="$TARGET" 'BEGIN{print (a>=t && b>=t)?1:0}')
        if [ "$clears" = "1" ]; then
            echo "GOOD: seed $SEED  setup $su  hold $ho  -- both clear $TARGET, rbf saved" | tee -a "$R"
            exit 0
        fi
    fi
done

if [ -n "$best_seed" ]; then
    echo "no seed cleared $TARGET on both; best both-positive was seed $best_seed (worst slack $best_min), rbf saved" | tee -a "$R"
else
    echo "NO seed produced two positive slacks -- this change does not fit, and that is not a seed problem" | tee -a "$R"
fi
