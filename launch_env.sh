#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.7"
fi

export STAGING_ROOT="/data/safe_staging"

# If a split-warp chestnut big model was compiled by SConscript, use it instead of the
# model manager bundle. The split-warp path warps on QCOM and ships 0.39 MB to AMD,
# vs the fused path shipping 7.47 MB raw NV12 — needed to fit the 50 ms budget on comma 3X.
BIG_PKL="$PWD/openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl"
if [ -f "$BIG_PKL" ] && lsusb 2>/dev/null | grep -q "3801:0001"; then
  export COMBINED_MODEL_PKL="$BIG_PKL"
fi
