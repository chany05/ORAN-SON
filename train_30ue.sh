#!/bin/bash
set -euo pipefail

# Curriculum training for MASAC:
#   1) Static random placement pretraining
#   2) Dynamic mobility finetuning
#
# Override defaults with environment variables if needed, e.g.
#   STATIC_EPISODES=30 DYNAMIC_EPISODES=10 SIM_TIME=30 ./train_30ue.sh

STATIC_EPISODES="${STATIC_EPISODES:-20}"
DYNAMIC_EPISODES="${DYNAMIC_EPISODES:-5}"
SIM_TIME="${SIM_TIME:-20}"
NUM_UES="${NUM_UES:-30}"
UE_SPEED="${UE_SPEED:-0.5}"

TOTAL_EPISODES=$((STATIC_EPISODES + DYNAMIC_EPISODES))
EP_INDEX=0

run_phase() {
    local phase_label="$1"
    local extra_args="$2"
    local episodes="$3"

    for (( local_ep=1; local_ep<=episodes; local_ep++ )); do
        EP_INDEX=$((EP_INDEX + 1))
        START=$(date +%s)
        echo "========== Episode ${EP_INDEX} / ${TOTAL_EPISODES} (${phase_label} ${local_ep}/${episodes}) =========="

        if [ "$EP_INDEX" -eq 1 ]; then
            ./ns3 run "TestSONXappLB_MASAC --simTime=${SIM_TIME} --numUes=${NUM_UES} ${extra_args}" 2>&1
        else
            ./ns3 run "TestSONXappLB_MASAC --simTime=${SIM_TIME} --numUes=${NUM_UES} --loadPretrained=true ${extra_args} --saturate=true" 2>&1
            ./ns3 run "TestSONXappLB_MASAC --simTime=${SIM_TIME} --numUes=${NUM_UES} --loadPretrained=true ${extra_args}" 2>&1
        fi

        END=$(date +%s)
        ELAPSED=$(( END - START ))
        echo "  >> ${ELAPSED}s ($(( ELAPSED / 60 ))m$(( ELAPSED % 60 ))s)"
        echo ""
    done
}

echo "=== MASAC Curriculum Training ==="
echo "  Static episodes : ${STATIC_EPISODES}"
echo "  Dynamic episodes: ${DYNAMIC_EPISODES}"
echo "  SimTime         : ${SIM_TIME}s"
echo "  NumUEs          : ${NUM_UES}"
echo "  Static phase    : ConstantPosition"
echo "  Dynamic phase   : RandomDirection2d (${UE_SPEED} m/s)"
echo "================================="

run_phase "static" "--staticUes=true" "${STATIC_EPISODES}"
run_phase "dynamic" "--ueSpeed=${UE_SPEED}" "${DYNAMIC_EPISODES}"

echo "=== Done ==="
