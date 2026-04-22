#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_TAG="${RUN_TAG:-baseline_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${OUT_DIR:-$ROOT_DIR/baseline_runs/$RUN_TAG}"

SIM_TIME="${SIM_TIME:-60}"
NUM_UES="${NUM_UES:-30}"
RNG_RUN="${RNG_RUN:-42}"
SATURATE="${SATURATE:-false}"
STATIC_UES="${STATIC_UES:-false}"
UE_SPEED="${UE_SPEED:-3}"
REWARD_PENALTY_WEIGHT="${REWARD_PENALTY_WEIGHT:-0.2}"

FILES_TO_PROTECT=(
  "cell_metrics.csv"
  "cio_actions.csv"
  "maddpg_actions.csv"
  "reward_curve.csv"
  "train_diag.csv"
  "stagnation_check.csv"
  "ue_trajectory.csv"
  "maddpg_models"
  "masac_models"
)

BACKUP_DIR="$(mktemp -d "$ROOT_DIR/.baseline_backup.XXXXXX")"

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  set +e

  for path in "${FILES_TO_PROTECT[@]}"; do
    rm -rf "$ROOT_DIR/$path"
    if [ -e "$BACKUP_DIR/$path" ]; then
      mv "$BACKUP_DIR/$path" "$ROOT_DIR/$path"
    fi
  done

  rm -rf "$BACKUP_DIR"
  exit "$rc"
}

trap cleanup EXIT INT TERM

mkdir -p "$OUT_DIR"

for path in "${FILES_TO_PROTECT[@]}"; do
  if [ -e "$ROOT_DIR/$path" ]; then
    mv "$ROOT_DIR/$path" "$BACKUP_DIR/$path"
  fi
done

CMD="./ns3 run \"TestSONXappLB_MASAC --baseline=true --simTime=${SIM_TIME} --numUes=${NUM_UES} --rngRun=${RNG_RUN} --ueSpeed=${UE_SPEED} --rewardPenaltyWeight=${REWARD_PENALTY_WEIGHT} --staticUes=${STATIC_UES}\""
if [ "$SATURATE" = "true" ]; then
  CMD="./ns3 run \"TestSONXappLB_MASAC --baseline=true --saturate=true --simTime=${SIM_TIME} --numUes=${NUM_UES} --rngRun=${RNG_RUN} --ueSpeed=${UE_SPEED} --rewardPenaltyWeight=${REWARD_PENALTY_WEIGHT} --staticUes=${STATIC_UES}\""
fi

echo "=== Isolated Baseline Run ==="
echo "  OutputDir : $OUT_DIR"
echo "  SimTime   : ${SIM_TIME}s"
echo "  NumUEs    : ${NUM_UES}"
echo "  RNG_RUN   : ${RNG_RUN}"
echo "  Saturate  : ${SATURATE}"
echo "  StaticUEs : ${STATIC_UES}"
echo "  UESpeed   : ${UE_SPEED}"
echo "  Penalty   : ${REWARD_PENALTY_WEIGHT}"
echo "============================="

cd "$ROOT_DIR"
eval "$CMD" | tee "$OUT_DIR/console.log"

for path in "${FILES_TO_PROTECT[@]}"; do
  if [ -e "$ROOT_DIR/$path" ]; then
    cp -a "$ROOT_DIR/$path" "$OUT_DIR/"
  fi
done

echo "Baseline outputs saved to: $OUT_DIR"
