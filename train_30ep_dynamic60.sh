#!/bin/bash
set -euo pipefail

TRAIN_EPISODES="${TRAIN_EPISODES:-30}"
EVAL_INTERVAL="${EVAL_INTERVAL:-3}"
SIM_TIME="${SIM_TIME:-60}"
EVAL_SIM_TIME="${EVAL_SIM_TIME:-60}"
NUM_UES="${NUM_UES:-30}"
UE_SPEED="${UE_SPEED:-3}"
REWARD_PENALTY_WEIGHT="${REWARD_PENALTY_WEIGHT:-0.2}"
RNG_RUN="${RNG_RUN:-42}"
LOAD_EXISTING="${LOAD_EXISTING:-false}"

echo "=== MASAC Dynamic Train/Eval ==="
echo "  TrainEpisodes       : ${TRAIN_EPISODES}"
echo "  EvalInterval        : ${EVAL_INTERVAL} train -> 1 eval"
echo "  TrainSimTime        : ${SIM_TIME}s"
echo "  EvalSimTime         : ${EVAL_SIM_TIME}s"
echo "  NumUEs              : ${NUM_UES}"
echo "  UEMobility          : RandomDirection2d (${UE_SPEED} m/s)"
echo "  RewardScale         : throughput / 10"
echo "  RewardPenaltyWeight : ${REWARD_PENALTY_WEIGHT}"
echo "  RNG_RUN             : ${RNG_RUN}"
echo "  LoadExisting        : ${LOAD_EXISTING}"
echo "================================"

for (( ep=1; ep<=TRAIN_EPISODES; ep++ )); do
    START=$(date +%s)
    echo "========== Train Episode ${ep} / ${TRAIN_EPISODES} =========="

    if [ "${ep}" -eq 1 ] && [ "${LOAD_EXISTING}" != "true" ]; then
        ./ns3 run "TestSONXappLB_MASAC --simTime=${SIM_TIME} --numUes=${NUM_UES} --ueSpeed=${UE_SPEED} --rewardPenaltyWeight=${REWARD_PENALTY_WEIGHT} --rngRun=${RNG_RUN}" 2>&1
    else
        ./ns3 run "TestSONXappLB_MASAC --simTime=${SIM_TIME} --numUes=${NUM_UES} --ueSpeed=${UE_SPEED} --rewardPenaltyWeight=${REWARD_PENALTY_WEIGHT} --rngRun=${RNG_RUN} --loadPretrained=true" 2>&1
    fi

    END=$(date +%s)
    ELAPSED=$(( END - START ))
    echo "  >> train elapsed: ${ELAPSED}s ($(( ELAPSED / 60 ))m$(( ELAPSED % 60 ))s)"
    echo ""

    if (( ep % EVAL_INTERVAL == 0 )); then
        EVAL_INDEX=$(( ep / EVAL_INTERVAL ))
        START=$(date +%s)
        echo "---------- Eval Episode ${EVAL_INDEX} (after train ${ep}) ----------"
        ./ns3 run "TestSONXappLB_MASAC --simTime=${EVAL_SIM_TIME} --numUes=${NUM_UES} --ueSpeed=${UE_SPEED} --rewardPenaltyWeight=${REWARD_PENALTY_WEIGHT} --rngRun=${RNG_RUN} --loadPretrained=true --inferenceOnly=true" 2>&1
        END=$(date +%s)
        ELAPSED=$(( END - START ))
        echo "  >> eval elapsed: ${ELAPSED}s ($(( ELAPSED / 60 ))m$(( ELAPSED % 60 ))s)"
        echo ""
    fi
done

echo "=== Done ==="
