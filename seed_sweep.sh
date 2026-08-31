#!/usr/bin/env bash
# SUPERSEDED. This now delegates to seed_sweep_both.sh.
#
# The original ranked candidates on "Worst-case setup slack" alone. That is how
# seed 16 once topped a list at +0.113 setup while carrying -0.346 hold and very
# nearly got deployed on the strength of the better number, and it is not a
# historical curiosity: in the 2026-08-31 sweep seed 1 came back at +0.134 setup
# with -0.495 hold, which this script would have selected and assembled.
#
# A hold violation is not a slower-clock problem. It fails at every frequency,
# so a build is only a candidate when BOTH slacks are positive.
#
# Kept as a delegating wrapper rather than deleted, because the name is in
# people's fingers and in older notes. Typing it should do the right thing, not
# fail with "no such file" and invite someone to fish the old one out of git.
set -u
here=$(cd "$(dirname "$0")" && pwd)
echo "seed_sweep.sh is superseded -- running seed_sweep_both.sh instead."
echo "  (it records both slacks and rejects a build that fails either)"
exec bash "$here/seed_sweep_both.sh" "$@"
