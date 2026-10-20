#!/bin/sh
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
binary=$(mktemp /tmp/apogee-imu-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
cc -std=c11 -Wall -Wextra -Werror -Wno-unused-parameter \
    -I"$repo/tests/host/include" \
    -I"$repo/components/telemetry/include" \
    -I"$repo/components/lsm6dsv/include" \
    -I"$repo/components/lsm6dsv/vendor" \
    "$repo/tests/host/test_lsm6dsv_pipeline.c" \
    "$repo/components/lsm6dsv/vendor/lsm6dsv320x_reg.c" \
    -lm -o "$binary"
"$binary"
