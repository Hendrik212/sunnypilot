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

# Ioniq 6: drive on Cheddar (commaai PR 39255, compiled on the device on Nov 9) instead of the
# model-manager bundle. It lived only in the device's copy of this file and a deploy dropped it:
# routes 283-28e (Nov 19-22) ran Kumars Vibe. modeld ignores it when the pkl is missing (falls
# back to the bundle) and on the chestnut path (modeld_v2/modeld.py _override_pkl).
export COMBINED_MODEL_PKL="/data/media/0/models/cheddar_build/driving_cheddar_pr39255_tinygrad.pkl"
export COMBINED_MODEL_GENERATION=12
