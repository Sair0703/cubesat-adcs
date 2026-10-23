#!/usr/bin/env bash
# Boots the flight software on an emulated Cortex-M3 and checks the mission results.
set -u
OUT=$(timeout 60 qemu-system-arm -M lm3s6965evb -nographic \
      -semihosting-config enable=on,target=native -kernel build/adcs.elf 2>/dev/null | tr -d '\r')
STATUS=$?
echo "$OUT" | tail -12
echo "----"
fail=0
metric() { echo "$OUT" | grep "$1" | grep -oE -- '-?[0-9]+\.[0-9]+' | head -1; }
within() {  # name value min max
    if awk -v v="$2" -v lo="$3" -v hi="$4" 'BEGIN{exit !(v!="" && v>=lo && v<=hi)}'; then
        echo "PASS  $1 = $2 (expected $3..$4)"; else echo "FAIL  $1 = '$2' (expected $3..$4)"; fail=1; fi
}
[ $STATUS -eq 0 ] && echo "PASS  firmware exited cleanly" || { echo "FAIL  exit status $STATUS"; fail=1; }
echo "$OUT" | grep -q "MISSION COMPLETE" && echo "PASS  mission completed" || { echo "FAIL  mission did not complete"; fail=1; }
echo "$OUT" | grep -q "SAFE .*SUN_STUCK" && echo "PASS  entered SAFE mode on sun sensor fault" || { echo "FAIL  no SAFE mode"; fail=1; }
within "detumble time (s)"      "$(metric 'detumble time')"   0 20
within "slew settle time (s)"   "$(metric 'slew settled')"    0 25
within "overshoot (deg)"        "$(metric 'overshoot')"       0 2
within "fault detection (s)"    "$(metric 'fault detected')"  0 1
within "recovery (s)"           "$(metric 'recovered in')"    0 20
within "final error (deg)"      "$(metric 'final pointing')" -1 1
if echo "$OUT" | grep -q "FAULT\*\|UNEXPECTED"; then echo "FAIL  CPU fault"; fail=1; else echo "PASS  no CPU faults"; fi
exit $fail
