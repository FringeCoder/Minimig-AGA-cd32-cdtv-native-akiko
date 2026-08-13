#!/usr/bin/env bash
# Regenerate tg68k_ss_prog.hex from tg68k_ss_prog.s.
#
# The .hex is committed because CI has no 68k assembler and the testbench needs
# the image at elaboration time. Run this after editing the .s, and commit both.
#
# vasm lives at C:\Tools\vbcc_win_x64\vbcc\bin\vasmm68k_mot on the dev machine;
# override with VASM=... anywhere else.
set -euo pipefail

cd "$(dirname "$0")"

VASM=${VASM:-/c/Tools/vbcc_win_x64/vbcc/bin/vasmm68k_mot}

"$VASM" -Fbin -o tg68k_ss_prog.bin tg68k_ss_prog.s

# One 16-bit big-endian word per line, which is what the testbench's loader
# reads. od prints bytes; paste pairs them back up.
od -An -tx1 -v tg68k_ss_prog.bin \
	| tr -s ' ' '\n' | grep -v '^$' | paste - - | tr -d '\t' \
	> tg68k_ss_prog.hex

rm -f tg68k_ss_prog.bin
wc -l < tg68k_ss_prog.hex
