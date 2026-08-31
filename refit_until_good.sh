#!/usr/bin/env bash
# SUPERSEDED. This now delegates to seed_sweep_both.sh.
#
# The original stopped at the first seed whose SETUP slack cleared a target, and
# never looked at hold. That is the more dangerous of the two old scripts,
# because stopping early on a partial measurement is exactly how a build with a
# hold violation gets picked and then trusted -- it does not merely rank wrong,
# it stops looking.
#
# Concretely, in the 2026-08-31 sweep the first candidate was seed 1 at +0.134
# setup and -0.495 hold. This script would have taken it, assembled it, printed
# "GOOD", and exited 0.
#
# seed_sweep_both.sh keeps the same early-exit behaviour but requires BOTH
# slacks positive and clear of its target before it stops, keeps every
# both-positive bitstream on the way, and says plainly when nothing qualifies
# instead of reporting a least-bad seed.
#
# Kept as a delegating wrapper rather than deleted: the name appears in older
# notes and commit messages, and it should do the right thing when typed.
set -u
here=$(cd "$(dirname "$0")" && pwd)
echo "refit_until_good.sh is superseded -- running seed_sweep_both.sh instead."
echo "  (it requires BOTH slacks positive before it calls a seed good)"
exec bash "$here/seed_sweep_both.sh" "$@"
