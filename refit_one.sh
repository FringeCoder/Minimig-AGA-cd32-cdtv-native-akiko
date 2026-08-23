#!/usr/bin/env bash
# Re-fit one known seed and assemble it. Used after refit_until_good.sh has
# ranked the candidates but saved none of them -- it only kept a build that
# cleared its target, and when nothing does, the best measured seed still has to
# be rebuilt to get an .rbf. Fixed below for next time; kept as its own script
# because "fit exactly this seed and give me the bitstream" is worth having.
set -u
Q=/c/intelFPGA_lite/17.0/quartus/bin64
SEED=${1:?seed}
"$Q/quartus_fit" --seed=$SEED Minimig > seed_sweep/one_fit_$SEED.log 2>&1 || exit 1
"$Q/quartus_sta" Minimig > seed_sweep/one_sta_$SEED.log 2>&1
s=$(grep -oE "Worst-case setup slack is [-0-9.]+" output_files/Minimig.sta.rpt | tail -1 | grep -oE "[-0-9.]+$")
"$Q/quartus_asm" Minimig > seed_sweep/one_asm_$SEED.log 2>&1
cp output_files/Minimig.rbf seed_sweep/Minimig_final_seed$SEED.rbf
echo "seed $SEED slack $s -- rbf saved"
