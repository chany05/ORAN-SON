#!/bin/bash
# MADDPG Training: 30 UEs, simTime=60, ACT_REG=0.05, real-time seed

EPISODES=30
SIM_TIME=60

echo "=== MASAC Training: ${EPISODES} eps, 30 UEs, simTime=${SIM_TIME}s ==="

for ep in $(seq 1 $EPISODES); do
    START=$(date +%s)
    echo "========== Episode $ep / $EPISODES =========="

    if [ $ep -eq 1 ]; then
        ./ns3 run "TestSONXappLB_MASAC --simTime=$SIM_TIME" 2>&1
    else
        ./ns3 run "TestSONXappLB_MASAC --simTime=$SIM_TIME --loadPretrained=true" 2>&1
        ./ns3 run "TestSONXappLB_MASAC --simTime=$SIM_TIME --saturate=true --loadPretrained=true" 2>&1
    fi

    END=$(date +%s)
    ELAPSED=$(( END - START ))
    echo "  >> ${ELAPSED}s ($(( ELAPSED / 60 ))m$(( ELAPSED % 60 ))s)"
    echo ""
done
echo "=== Done ==="
