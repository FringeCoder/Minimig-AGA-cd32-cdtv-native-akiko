#!/usr/bin/env bash
# Re-fit ONE named seed and assemble it. "Fit exactly this seed and give me the
# bitstream" is worth having as its own thing, so this survives where the two
# sweep scripts beside it did not.
#
# It used to print the setup slack and nothing else, which is how a build with a
# hold violation can look fine in the terminal. A hold violation is not a
# slower-clock problem -- it fails at every frequency -- so this now reports
# BOTH and exits non-zero if either is negative. The .rbf is still written
# either way; you may well want a failing build to test with, but you should
# have to notice that it fails.
set -u
Q=/c/intelFPGA_lite/17.0/quartus/bin64
OUT=seed_sweep
SEED=${1:?usage: refit_one.sh <seed>}
mkdir -p "$OUT"

# Never two fits at once: they share db/ and output_files/ and interleave
# silently, and nothing from such a window can be trusted.
while tasklist 2>/dev/null | grep -qiE "quartus_(map|fit|asm|sta)"; do
    echo "waiting for an in-flight Quartus run..."
    sleep 30
done

"$Q/quartus_fit" --seed="$SEED" Minimig > "$OUT/one_fit_$SEED.log" 2>&1 || {
    echo "seed $SEED: FIT FAILED, see $OUT/one_fit_$SEED.log"; exit 1; }
"$Q/quartus_sta" Minimig > "$OUT/one_sta_$SEED.log" 2>&1

# Read both from THIS run's report.
su=$(grep -oE "Worst-case setup slack is [-0-9.]+" output_files/Minimig.sta.rpt | tail -1 | grep -oE "[-0-9.]+$")
ho=$(grep -oE "Worst-case hold slack is [-0-9.]+"  output_files/Minimig.sta.rpt | tail -1 | grep -oE "[-0-9.]+$")
su=${su:-0}; ho=${ho:-0}

"$Q/quartus_asm" Minimig > "$OUT/one_asm_$SEED.log" 2>&1
cp output_files/Minimig.rbf "$OUT/Minimig_final_seed$SEED.rbf"

printf "seed %s  setup %s  hold %s  -- rbf saved to %s/Minimig_final_seed%s.rbf\n" \
       "$SEED" "$su" "$ho" "$OUT" "$SEED"

bad=$(awk -v a="$su" -v b="$ho" 'BEGIN{print (a<=0 || b<=0) ? 1 : 0}')
if [ "$bad" = "1" ]; then
    echo "*** THIS BUILD DOES NOT MEET TIMING. Do not ship it. ***"
    exit 2
fi
