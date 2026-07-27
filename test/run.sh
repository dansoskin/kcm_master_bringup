#!/usr/bin/env bash
# Host tests for the pure-maths parts of the firmware. No hardware, no HAL.
#   bash test/run.sh
set -euo pipefail

cd "$(dirname "$0")"
CC=${CC:-gcc}
CFLAGS="-Wall -Wextra -O2 -I../Core/Inc"

fail=0

echo "== plate_kinematics =="
$CC $CFLAGS test_plate_kinematics.c ../Core/Src/plate_kinematics.c -lm -o test_plate.exe
./test_plate.exe || fail=1

echo
if [ "$fail" -ne 0 ]; then
    echo "FAILED"
    exit 1
fi
echo "all host tests passed"
