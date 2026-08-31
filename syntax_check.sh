#!/usr/bin/env bash
#
# Parse every synthesisable Verilog/SystemVerilog source standalone and fail on
# a real syntax error.
#
# Why this exists: every step in .github/workflows/rtl-sim.yml compiles a small
# subset of files for one bench, so most of rtl/ -- and all of Minimig.sv -- is
# never parsed by CI at all. A plain syntax error therefore passes the whole
# pipeline and surfaces only in Quartus. On 2026-08-27 a missing comma in a
# Minimig.sv port list produced
#
#     Error (10170): Verilog HDL syntax error at Minimig.sv(1728) near text: "."
#
# 18 seconds into a fit that had been waited on. The same error is visible in
# about two seconds from a standalone parse.
#
# Parsing a file standalone necessarily reports unknown modules and unresolved
# hierarchical names, because the rest of the design is not on the command line.
# Those are expected and are ignored. Only these are treated as failures:
#
#     syntax error
#     has already been declared
#     Errors in port declarations
#
# rtl/sim/ is excluded: benches are compiled with their real dependencies by the
# bench steps below this one, and they legitimately reference DUT internals.

set -uo pipefail

IVERILOG=${IVERILOG:-iverilog}
NULLOUT=${TMPDIR:-/tmp}/syntax_check.$$
trap 'rm -f "$NULLOUT"' EXIT

mapfile -t FILES < <(
  { find rtl -name '*.v' -o -name '*.sv'; ls *.v *.sv 2>/dev/null; } \
    | grep -v '/sim/' | sort -u
)

clean=0
unknown_only=0
bad=0

for f in "${FILES[@]}"; do
  out=$("$IVERILOG" -g2012 -t null -o "$NULLOUT" "$f" 2>&1)
  if [ -z "$out" ]; then
    clean=$((clean + 1))
    continue
  fi
  if echo "$out" | grep -qE 'syntax error|has already been declared|Errors in port declarations'; then
    bad=$((bad + 1))
    echo "FAIL $f"
    echo "$out" | grep -E 'syntax error|has already been declared|Errors in port declarations' | sed 's/^/    /'
  else
    unknown_only=$((unknown_only + 1))
  fi
done

echo
echo "files parsed        : ${#FILES[@]}"
echo "clean               : $clean"
echo "unknown-module only : $unknown_only"
echo "syntax errors       : $bad"

if [ "$bad" -ne 0 ]; then
  echo
  echo "A file above does not parse standalone. Quartus may still accept it, but"
  echo "this is the cheap place to find out -- fix it here rather than 35 minutes"
  echo "into a fit."
  exit 1
fi
