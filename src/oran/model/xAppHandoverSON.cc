#include "xAppHandoverSON.h"

#include "ns3/E2AP.h"
#include "ns3/core-module.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <sstream>
#include <numeric>
#include <random>

using namespace ns3;
using namespace oran;

NS_LOG_COMPONENT_DEFINE("xAppHandoverSON");

xAppHandoverSON::xAppHandoverSON(float sonPeriodicitySec, bool initiateHandovers,
                                 bool loadPretrained, bool inferenceOnly,
                                 double simStopTime, bool baseline)
    : xAppHandover(),
      m_sonPeriodicitySec(sonPeriodicitySec),
      m_initiateHandovers(initiateHandovers),
      m_edgeRsrpThreshold(-79.26),
      m_loadThreshold(12.0),
      m_rsrqThreshold(-15.0),
      m_cqiThreshold(11),
      m_txPower(32.0),
      m_frequency(2.12e9),
      m_dlBandwidthPrb(100)
{
    NS_LOG_FUNCTION(this);

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&xAppHandoverSON::HandoverSucceeded, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&xAppHandoverSON::HandoverStarted, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                    MakeCallback(&xAppHandoverSON::HandoverFailed, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback(&xAppHandoverSON::ConnectionEstablished, this));

    m_loadPretrained = loadPretrained;
    m_inferenceOnly  = inferenceOnly;
    m_simStopTime    = simStopTime;
    m_baseline       = baseline;

    if (m_useMADDPG && !m_baseline)
    {
        InitMADDPG();
    }
    InitCsvLoggers();

    Simulator::Schedule(Seconds(m_sonPeriodicitySec),
                        &xAppHandoverSON::PeriodicSONCheck,
                        this);
}

void
xAppHandoverSON::PeriodicSONCheck()
{
    NS_LOG_FUNCTION(this);

    CollectKPMs();

    // Direct copy (no EMA smoothing)
    for (auto& [cellId, cell] : m_cellContexts)
    {
        auto& s = m_smoothed[cellId];
        s.ueCount = cell.ueCount;
        s.edgeUeCount = cell.edgeUeCount;
        s.dlThp = cell.totalThroughputDl;
        s.ulThp = cell.totalThroughputUl;
        s.initialized = true;
    }

    if (m_stepCount % 5 == 0)
    {
        LogCellMetrics();
    }
    NS_LOG_LOGIC("cell 1: " << m_cellContexts[1].ueCount
        << " cell 2: " << m_cellContexts[2].ueCount
        << " cell 3: " << m_cellContexts[3].ueCount);

    std::cout << "\n[TICK] PeriodicSONCheck(MADDPG) - Step: " << m_stepCount
              << " | UE count: " << m_ueContexts.size() << std::endl;

    if (m_baseline)
    {
        // Baseline: CIO=0, TXP=32 고정, 메트릭만 기록
        CollectCellThroughput();
        m_stepCount++;
    }
    else if (m_useMADDPG)
    {
        StepMADDPG();

        if (!m_inferenceOnly)
        {
            TrainMADDPG();
        }

        m_stepCount++;

        if (m_stepCount % 100 == 0)
        {
            SaveModels();
        }
    }
    else
    {
        if (m_initiateHandovers)
        {
            CalculateLoadScores();
            E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");
            for (auto& [key, ue] : m_ueContexts)
            {
                if (m_imsiInHandover.find(key) != m_imsiInHandover.end())
                    continue;

                uint16_t targetCell = MakeSONDecision(key);
                if (targetCell != std::numeric_limits<uint16_t>::max() &&
                    targetCell != ue.servingCellId)
                {
                    std::string srcEndpoint = "/E2Node/"
                        + std::to_string(ue.servingCellId) + "/";
                    ric->E2SmRcSendHandoverControlRequest(
                        ue.rnti, targetCell, srcEndpoint);
                    m_imsiInHandover.emplace(key, 0);
                }
            }
        }
        CollectCellThroughput();
    }

    Simulator::Schedule(Seconds(m_sonPeriodicitySec),
                        &xAppHandoverSON::PeriodicSONCheck,
                        this);
}

// =============================================================================
// 데이터 수집
// =============================================================================
void
xAppHandoverSON::CollectKPMs()
{
    NS_LOG_FUNCTION(this);
    m_ueContexts.clear();
    m_cellContexts.clear();
    CollectCellKpms();
    CollectRsrpRsrq();
    CollectTargetRsrq();
    CollectCqi();

    CalculateEdgeUEs();
    PurgeStaleUeContexts();

    CollectUeCount();
    CollectCellThroughput();
}

void
xAppHandoverSON::CollectRsrpRsrq()
{
    NS_LOG_FUNCTION(this);
    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");

    auto rsrpMap = ric->QueryKpmMetric("/KPM/HO.SrcCellQual.RSRP");
    for (auto& e2nodeMeasurements : rsrpMap)
    {
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (!measurement.measurements.contains("RNTI") ||
                !measurement.measurements.contains("CELLID"))
                continue;

            uint16_t rnti = measurement.measurements["RNTI"];
            uint16_t cellId = measurement.measurements["CELLID"];
            double rsrp = measurement.measurements["VALUE"];

            UeKey key = MakeUeKey(cellId, rnti);
            m_ueContexts[key].rnti = rnti;
            m_ueContexts[key].servingCellId = cellId;
            m_ueContexts[key].servingRsrp = rsrp;
        }
    }

    auto rsrqMap = ric->QueryKpmMetric("/KPM/HO.SrcCellQual.RSRQ");
    for (auto& e2nodeMeasurements : rsrqMap)
    {
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (!measurement.measurements.contains("RNTI") ||
                !measurement.measurements.contains("CELLID"))
                continue;

            uint16_t rnti = measurement.measurements["RNTI"];
            uint16_t cellId = measurement.measurements["CELLID"];

            UeKey key = MakeUeKey(cellId, rnti);
            if (m_ueContexts.find(key) != m_ueContexts.end() &&
                m_ueContexts[key].servingRsrq == 0)
            {
                m_ueContexts[key].servingRsrq = measurement.measurements["VALUE"];
            }
        }
    }
}

void
xAppHandoverSON::CollectTargetRsrq()
{
    NS_LOG_FUNCTION(this);
    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");

    auto rsrqMap = ric->QueryKpmMetric("/KPM/HO.TrgtCellQual.RSRQ");
    for (auto& e2nodeMeasurements : rsrqMap)
    {
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (!measurement.measurements.contains("RNTI") ||
                !measurement.measurements.contains("CELLID") ||
                !measurement.measurements.contains("TARGET"))
                continue;

            uint16_t rnti = measurement.measurements["RNTI"];
            uint16_t cellId = measurement.measurements["CELLID"];
            uint16_t targetCellId = measurement.measurements["TARGET"];
            double rsrq = measurement.measurements["VALUE"];

            UeKey key = MakeUeKey(cellId, rnti);
            if (m_ueContexts.find(key) != m_ueContexts.end())
            {
                if (m_ueContexts[key].neighborRsrq.find(targetCellId) ==
                    m_ueContexts[key].neighborRsrq.end())
                {
                    m_ueContexts[key].neighborRsrq[targetCellId] = rsrq;
                }
            }
        }
    }
}

void
xAppHandoverSON::CollectCqi()
{
    NS_LOG_FUNCTION(this);
    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");

    auto cqiMap = ric->QueryKpmMetric("/KPM/CARR.WBCQIDist.Bin");
    for (auto& e2nodeMeasurements : cqiMap)
    {
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (!measurement.measurements.contains("RNTI") ||
                !measurement.measurements.contains("CELLID"))
                continue;

            uint16_t rnti = measurement.measurements["RNTI"];
            uint16_t cellId = measurement.measurements["CELLID"];

            UeKey key = MakeUeKey(cellId, rnti);
            if (m_ueContexts.find(key) != m_ueContexts.end())
            {
                m_ueContexts[key].cqi = measurement.measurements["VALUE"];
            }
        }
    }
}

void
xAppHandoverSON::CollectCellKpms()
{
    NS_LOG_FUNCTION(this);
    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");
    if (!ric) return;

    auto ueCountMap = ric->QueryKpmMetric("/KPM/DRB.UEActiveDl.QCI");
    for (auto& [endpoint, measurements] : ueCountMap)
    {
        std::string mostRecentTimestamp("");
        for (auto& m : measurements)
        {
            if (mostRecentTimestamp.empty())
                mostRecentTimestamp = m.timestamp;
            if (mostRecentTimestamp != m.timestamp)
                continue;

            if (!m.measurements.contains("CELLID") ||
                !m.measurements.contains("VALUE"))
                continue;

            uint16_t cellId = m.measurements["CELLID"];
            int ueCount = m.measurements["VALUE"];

            if (m_cellContexts.find(cellId) == m_cellContexts.end())
            {
                m_cellContexts[cellId] = CellContext();
                m_cellContexts[cellId].cellId = cellId;
            }
            m_cellContexts[cellId].ueCount = ueCount;
        }
    }

    auto txPwrMap = ric->QueryKpmMetric("/KPM/CARR.AvgTxPwr");
    for (auto& [endpoint, measurements] : txPwrMap)
    {
        std::string mostRecentTimestamp("");
        for (auto& m : measurements)
        {
            if (mostRecentTimestamp.empty())
                mostRecentTimestamp = m.timestamp;
            if (mostRecentTimestamp != m.timestamp)
                continue;
            if (!m.measurements.contains("CELLID") ||
                !m.measurements.contains("VALUE"))
                continue;

            uint16_t cellId = m.measurements["CELLID"];
            double txPower = m.measurements["VALUE"];

            if (m_cellContexts.find(cellId) == m_cellContexts.end())
            {
                m_cellContexts[cellId] = CellContext();
                m_cellContexts[cellId].cellId = cellId;
            }
            m_cellContexts[cellId].txPower = txPower;
        }
    }
}

void
xAppHandoverSON::CollectUeCount()
{
    NS_LOG_FUNCTION(this);
    for (auto& [key, ue] : m_ueContexts)
    {
        uint16_t cellId = ue.servingCellId;
        if (m_cellContexts.find(cellId) == m_cellContexts.end())
        {
            m_cellContexts[cellId] = CellContext();
            m_cellContexts[cellId].cellId = cellId;
        }
        if (ue.isEdge)
        {
            m_cellContexts[cellId].edgeUeCount++;
        }
    }
}

void
xAppHandoverSON::CollectCellThroughput()
{
    NS_LOG_FUNCTION(this);
    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");
    if (!ric) return;

    for (auto& [cellId, cell] : m_cellContexts)
    {
        cell.totalThroughputDl = 0.0;
        cell.totalThroughputUl = 0.0;
    }

    // DL
    auto dlMap = ric->QueryKpmMetric("/KPM/DRB.IpVolDl.QCI");
    std::map<uint16_t, double> latestDlBytes;
    for (auto& [endpoint, measurements] : dlMap)
    {
        for (auto& m : measurements)
        {
            if (!m.measurements.contains("CELLID") ||
                !m.measurements.contains("VALUE"))
                continue;
            uint16_t cellId = m.measurements["CELLID"];
            double cumBytes = m.measurements["VALUE"];
            latestDlBytes[cellId] = std::max(latestDlBytes[cellId], cumBytes);
        }
    }
    for (auto& [cellId, cumBytes] : latestDlBytes)
    {
        double prev = m_prevCellDlBytes.count(cellId) ? m_prevCellDlBytes[cellId] : -1.0;
        m_prevCellDlBytes[cellId] = cumBytes;
        if (prev < 0) continue;
        double delta = cumBytes - prev;
        if (delta < 0) delta = 0;
        auto it = m_cellContexts.find(cellId);
        if (it != m_cellContexts.end())
            it->second.totalThroughputDl += delta;
    }

    // UL
    auto ulMap = ric->QueryKpmMetric("/KPM/DRB.IpVolUl.QCI");
    std::map<uint16_t, double> latestUlBytes;
    for (auto& [endpoint, measurements] : ulMap)
    {
        for (auto& m : measurements)
        {
            if (!m.measurements.contains("CELLID") ||
                !m.measurements.contains("VALUE"))
                continue;
            uint16_t cellId = m.measurements["CELLID"];
            double cumBytes = m.measurements["VALUE"];
            latestUlBytes[cellId] = std::max(latestUlBytes[cellId], cumBytes);
        }
    }
    for (auto& [cellId, cumBytes] : latestUlBytes)
    {
        double prev = m_prevCellUlBytes.count(cellId) ? m_prevCellUlBytes[cellId] : 0.0;
        m_prevCellUlBytes[cellId] = cumBytes;
        if (prev < 0) continue;
        double delta = cumBytes - prev;
        if (delta < 0) delta = 0;
        auto it = m_cellContexts.find(cellId);
        if (it != m_cellContexts.end())
            it->second.totalThroughputUl += delta;
    }

    // PRB utilization
    auto prbMap = ric->QueryKpmMetric("/KPM/RRU.PrbTotDl");
    for (auto& [endpoint, measurements] : prbMap)
    {
        for (auto& m : measurements)
        {
            if (!m.measurements.contains("CELLID") ||
                !m.measurements.contains("PRB_UTIL")) continue;
            uint16_t cellId = m.measurements["CELLID"];
            double util = m.measurements["PRB_UTIL"];
            auto it = m_cellContexts.find(cellId);
            if (it != m_cellContexts.end())
                it->second.prbUtilDl = util;
        }
    }

    // bytes → kbps
    for (auto& [cellId, cell] : m_cellContexts)
    {
        if (cell.totalThroughputDl > 0 || cell.totalThroughputUl > 0)
        {
            cell.totalThroughputDl = (cell.totalThroughputDl * 8.0) / 1000.0 / m_sonPeriodicitySec;
            cell.totalThroughputUl = (cell.totalThroughputUl * 8.0) / 1000.0 / m_sonPeriodicitySec;

            std::cout << "[IpVol] Cell" << cellId
                << " DL=" << (cell.totalThroughputDl / 1e3) << "Mbps"
                << " UL=" << (cell.totalThroughputUl / 1e3) << "Mbps"
                << " UEs=" << cell.ueCount << std::endl;

            m_lastThroughputDl[cellId] = cell.totalThroughputDl;
            m_lastThroughputUl[cellId] = cell.totalThroughputUl;
        }
        else
        {
            cell.totalThroughputDl = 0;
            cell.totalThroughputUl = 0;
        }
    }
}

// =============================================================================
// Edge UE 계산
// =============================================================================
void
xAppHandoverSON::CalculateEdgeUEs()
{
    NS_LOG_FUNCTION(this);
    for (auto& [rnti, ue] : m_ueContexts)
    {
        ue.isEdge = (ue.servingRsrp < m_edgeRsrpThreshold);
    }
}

// =============================================================================
// 부하 계산
// =============================================================================
void
xAppHandoverSON::CalculateLoadScores()
{
    NS_LOG_FUNCTION(this);
    for (auto& [cellId, cell] : m_cellContexts)
    {
        cell.loadScore = cell.ueCount * 2.0
                       + cell.edgeUeCount * 3.0;
    }
}

bool
xAppHandoverSON::IsCellOverloaded(uint16_t cellId)
{
    auto it = m_cellContexts.find(cellId);
    if (it == m_cellContexts.end())
        return false;
    return it->second.loadScore > m_loadThreshold;
}

// =============================================================================
// 의사결정
// =============================================================================
uint16_t
xAppHandoverSON::MakeSONDecision(UeKey key)
{
    NS_LOG_FUNCTION(this);

    auto it = m_ueContexts.find(key);
    if (it == m_ueContexts.end())
        return std::numeric_limits<uint16_t>::max();

    UEContext& ue = it->second;

    if (IsCellOverloaded(ue.servingCellId) && ue.isEdge)
        return FindLeastLoadedNeighbor(key);

    if (ue.servingRsrq < m_rsrqThreshold)
        return FindBestRsrqCell(key);

    if (ue.cqi < m_cqiThreshold && ue.cqi > 0)
        return FindLeastLoadedNeighbor(key);

    return std::numeric_limits<uint16_t>::max();
}

uint16_t
xAppHandoverSON::FindLeastLoadedNeighbor(UeKey key)
{
    NS_LOG_FUNCTION(this);
    auto ueIt = m_ueContexts.find(key);
    if (ueIt == m_ueContexts.end())
        return std::numeric_limits<uint16_t>::max();

    uint16_t servingCell = ueIt->second.servingCellId;
    uint16_t bestCell = std::numeric_limits<uint16_t>::max();
    double minLoad = std::numeric_limits<double>::max();

    for (auto& [cellId, cell] : m_cellContexts)
    {
        if (cellId != servingCell && cell.loadScore < minLoad)
        {
            minLoad = cell.loadScore;
            bestCell = cellId;
        }
    }
    return bestCell;
}

uint16_t
xAppHandoverSON::FindBestRsrqCell(UeKey key)
{
    NS_LOG_FUNCTION(this);
    auto ueIt = m_ueContexts.find(key);
    if (ueIt == m_ueContexts.end())
        return std::numeric_limits<uint16_t>::max();

    UEContext& ue = ueIt->second;
    if (ue.neighborRsrq.empty())
        return FindLeastLoadedNeighbor(key);

    uint16_t bestCell = std::numeric_limits<uint16_t>::max();
    double bestRsrq = -std::numeric_limits<double>::max();

    for (auto& [targetCellId, rsrq] : ue.neighborRsrq)
    {
        if (rsrq > bestRsrq)
        {
            bestRsrq = rsrq;
            bestCell = targetCellId;
        }
    }

    if (bestRsrq <= ue.servingRsrq)
        return std::numeric_limits<uint16_t>::max();

    return bestCell;
}

void
xAppHandoverSON::PurgeStaleUeContexts()
{
    for (auto staleIt = m_staleKeys.begin(); staleIt != m_staleKeys.end(); )
    {
        auto ueIt = m_ueContexts.find(*staleIt);
        if (ueIt != m_ueContexts.end())
        {
            m_ueContexts.erase(ueIt);
            staleIt = m_staleKeys.erase(staleIt);
        }
        else
        {
            staleIt = m_staleKeys.erase(staleIt);
        }
    }
}

// =============================================================================
// 핸드오버 의사결정 (E2SM-RC 콜백)
// =============================================================================
void
xAppHandoverSON::HandoverDecision(Json& payload)
{
    NS_LOG_FUNCTION(this);

    if (E2AP::RetrieveInstanceWithEndpoint(GetRootEndpoint())->GetNode() !=
        E2AP::RetrieveInstanceWithEndpoint("/E2Node/0")->GetNode())
    {
        NS_ABORT_MSG("Trying to run a xApp on a E2Node is a no-no");
    }

    uint16_t requestingRnti = payload["RNTI"];
    uint16_t requestedTargetCellId = payload["Target Primary Cell ID"];

    uint16_t servingCellId = 0;
    if (payload.contains("Source Primary Cell ID"))
        servingCellId = payload["Source Primary Cell ID"];
    else if (payload.contains("CELLID"))
    {
        servingCellId = payload["CELLID"];
        servingCellId++;
    }

    if (servingCellId == 0)
    {
        payload["Target Primary Cell ID"] = std::numeric_limits<uint16_t>::max();
        m_decision_history.push_back({requestingRnti, requestedTargetCellId,
                                      std::numeric_limits<uint16_t>::max()});
        return;
    }

    UeKey key = MakeUeKey(servingCellId, requestingRnti);
    uint16_t decidedTargetCellId = requestedTargetCellId;

    if (m_initiateHandovers)
    {
        uint16_t sonDecision = MakeSONDecision(key);
        if (sonDecision != std::numeric_limits<uint16_t>::max())
            decidedTargetCellId = sonDecision;
    }

    if (m_imsiInHandover.find(key) != m_imsiInHandover.end())
        decidedTargetCellId = std::numeric_limits<uint16_t>::max();

    if (decidedTargetCellId != std::numeric_limits<uint16_t>::max())
        m_imsiInHandover.emplace(key, 0);

    payload["Target Primary Cell ID"] = decidedTargetCellId;
    m_decision_history.push_back({requestingRnti, requestedTargetCellId, decidedTargetCellId});
}

// =============================================================================
// 핸드오버 콜백
// =============================================================================
void
xAppHandoverSON::HandoverStarted(std::string context, uint64_t imsi, uint16_t cellid,
                         uint16_t rnti, uint16_t targetCellId)
{
    UeKey key = MakeUeKey(cellid, rnti);
    m_imsiInHandover[key] = imsi;
}

void
xAppHandoverSON::HandoverSucceeded(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    NS_LOG_FUNCTION(this);
    for (auto it = m_imsiInHandover.begin(); it != m_imsiInHandover.end(); ++it)
    {
        if (it->second == imsi)
        {
            m_staleKeys.insert(it->first);
            m_imsiInHandover.erase(it);
            break;
        }
    }
}

void
xAppHandoverSON::HandoverFailed(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    for (auto it = m_imsiInHandover.begin(); it != m_imsiInHandover.end(); ++it)
    {
        if (it->second == imsi) { m_imsiInHandover.erase(it); break; }
    }
}

void
xAppHandoverSON::ConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    for (auto it = m_imsiInHandover.begin(); it != m_imsiInHandover.end(); ++it)
    {
        if (it->second == imsi) { m_imsiInHandover.erase(it); break; }
    }
}

// =============================================================================
// MADDPG 초기화
// =============================================================================
void
xAppHandoverSON::InitMADDPG()
{
    NS_LOG_FUNCTION(this);

    m_cellIds = {1, 2, 3};
    m_neighborMap[1] = {2, 3};
    m_neighborMap[2] = {1, 3};
    m_neighborMap[3] = {1, 2};

    int64_t totalObsDim = NUM_AGENTS * OBS_DIM;
    int64_t totalActDim = NUM_AGENTS * ACT_DIM;  // each agent: [CIO, TXP]

    for (auto cellId : m_cellIds)
    {
        AgentConfig config;
        config.cellId = cellId;
        config.obsDim = OBS_DIM;
        config.actDim = ACT_DIM;  // [CIO, TXP]
        config.neighborCellIds = m_neighborMap[cellId];

        m_agents.push_back(std::make_unique<MADDPGAgent>(
            config, totalObsDim, totalActDim, ACTOR_LR, CRITIC_LR, MAX_ACTION));
    }

    m_replayBuffer = std::make_unique<ReplayBuffer>(BUFFER_SIZE);

    NS_LOG_UNCOND("[MADDPG] Initialized: " << NUM_AGENTS << " agents, "
        << "Obs=" << OBS_DIM << " totalActDim=" << totalActDim
        << " Network=256-256, Epsilon=" << m_epsilon);

    if (m_loadPretrained)
    {
        LoadModels();
    }

    // Inference: Actor를 eval 모드로 전환 (RunningMeanStd가 running stats 사용)
    if (m_inferenceOnly)
    {
        for (auto& agent : m_agents)
            agent->GetActor()->eval();
    }
}

// =============================================================================
// 관측 벡터 (EMA smoothing 적용, 5MHz 정규화)
// =============================================================================
torch::Tensor
xAppHandoverSON::BuildObservation(uint16_t cellId)
{
    std::vector<float> obs(OBS_DIM, 0.0f);

    auto cellIt = m_cellContexts.find(cellId);
    if (cellIt == m_cellContexts.end())
        return torch::zeros({OBS_DIM});

    // 1회 순회로 UE 통계 수집
    float sumCqi = 0.0f;
    uint32_t cqiCount = 0;
    uint32_t edgeCount = 0;
    uint32_t ueWithData = 0;

    for (auto& [key, ue] : m_ueContexts)
    {
        if (ue.servingCellId == cellId)
        {
            ueWithData++;
            sumCqi += ue.cqi;
            cqiCount++;
            if (ue.isEdge) edgeCount++;
        }
    }

    const auto& sm = m_smoothed[cellId];
    float avgCqi = (cqiCount > 0) ? sumCqi / cqiCount : 0.0f;
    double smoothTotal = 0.0;
    for (auto& [cid, s] : m_smoothed) smoothTotal += s.ueCount;

    obs[0] = avgCqi / 15.0f;                                                    // AvgCqi normalized
    obs[1] = static_cast<float>(sm.dlThp / 1e3) / 5.0f;                        // DL Thp / 5Mbps (5MHz BW)
    obs[2] = (sm.ueCount > 0.5) ? static_cast<float>(sm.edgeUeCount / sm.ueCount) : 0.0f;  // EdgeRatio
    obs[3] = (smoothTotal > 0.5) ? static_cast<float>(sm.ueCount / smoothTotal) : 0.0f;     // UeRatio

    return torch::from_blob(obs.data(), {OBS_DIM}, torch::kFloat32).clone();
}

// =============================================================================
// 보상 함수: perUeThp (raw kbps)
// =============================================================================
std::vector<double>
xAppHandoverSON::ComputeRewards()
{
    std::vector<double> thps;
    std::vector<double> prbUtils;
    std::vector<double> ueCounts;
    double totalUEs = 0.0;

    for (auto cellId : m_cellIds)
    {
        auto sit = m_smoothed.find(cellId);
        auto cit = m_cellContexts.find(cellId);
        if (sit != m_smoothed.end() && sit->second.initialized)
        {
            thps.push_back(sit->second.dlThp + sit->second.ulThp);  // raw kbps
            ueCounts.push_back(sit->second.ueCount);
            totalUEs += sit->second.ueCount;
        }
        else
        {
            thps.push_back(0.0);
            ueCounts.push_back(0.0);
        }
        prbUtils.push_back(cit != m_cellContexts.end() ? cit->second.prbUtilDl : 0.0);
    }

    // Reward: total throughput (normalized) - UE std penalty
    double totalThp = std::accumulate(thps.begin(), thps.end(), 0.0);
    double normalizedThp = totalThp / 2e4;  // scale to ~0-1 range

    // UE std penalty: penalize uneven UE distribution across cells
    double stdUeNorm = 0.0;
    if (totalUEs > 0) {
        double meanUe = totalUEs / ueCounts.size();
        double ueVar = 0.0;
        for (double u : ueCounts) ueVar += (u - meanUe) * (u - meanUe);
        stdUeNorm = std::sqrt(ueVar / ueCounts.size()) / totalUEs;
    }

    constexpr double UE_STD_PENALTY = 0.5;
    double normalizedReward = normalizedThp - UE_STD_PENALTY * stdUeNorm;

    std::vector<double> rewards;
    for (size_t i = 0; i < thps.size(); i++)
    {
        rewards.push_back(normalizedReward);
    }

    // For logging
    double meanPrb = std::accumulate(prbUtils.begin(), prbUtils.end(), 0.0) / prbUtils.size();
    double prbVar = 0.0;
    for (double p : prbUtils) prbVar += (p - meanPrb) * (p - meanPrb);
    double stdPrb = std::sqrt(prbVar / prbUtils.size());

    {
        double now = Simulator::Now().GetSeconds();
        std::ostringstream oss;
        oss << now << "," << m_stepCount;
        for (size_t i = 0; i < rewards.size(); i++)
            oss << "," << rewards[i];
        oss << "," << stdPrb << "," << stdUeNorm;
        m_rewardCurveBuf.push_back(oss.str());
    }

    return rewards;
}

// =============================================================================
// MADDPG 스텝: 관측 → 행동 → CIO 적용
// =============================================================================
void
xAppHandoverSON::StepMADDPG()
{
    NS_LOG_FUNCTION(this);

    // ── 현재 관측 수집 ──
    std::vector<torch::Tensor> currentObs;
    for (size_t i = 0; i < m_agents.size(); i++)
    {
        currentObs.push_back(BuildObservation(m_agents[i]->GetConfig().cellId));
    }

    // ── 이전 전이를 replay buffer에 저장 ──
    std::vector<double> stepRewards;
    if (m_hasPrevStep)
    {
        stepRewards = ComputeRewards();
        double now = Simulator::Now().GetSeconds();
        bool isDone = (m_simStopTime - now <= m_sonPeriodicitySec * 2.0);

        if (!m_inferenceOnly)
        {
            Experience exp;
            exp.obs = m_prevObs;
            exp.acts = m_prevActs;
            exp.rewards = stepRewards;
            exp.nextObs = currentObs;
            exp.done = isDone;
            m_replayBuffer->Push(std::move(exp));
        }

        if (m_stepCount % 10 == 0)
        {
            NS_LOG_UNCOND("[MADDPG] Step=" << m_stepCount
                << " Rewards=[" << stepRewards[0] << ", "
                << stepRewards[1] << ", " << stepRewards[2] << "]"
                << " BufferSize=" << m_replayBuffer->Size());
        }
    }

    // ── 행동 선택 (epsilon-greedy + Gaussian noise, Python MADDPG와 동일) ──
    std::vector<torch::Tensor> currentActs;

    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");
    if (!ric) return;

    for (size_t i = 0; i < m_agents.size(); i++)
    {
        torch::Tensor act;
        uint16_t srcCell = m_agents[i]->GetConfig().cellId;

        if (m_inferenceOnly)
        {
            // Inference: deterministic (Beta mode)
            act = m_agents[i]->SelectAction(currentObs[i], /*deterministic=*/true);
        }
        else if (m_uniformDist(m_rng) <= m_epsilon)
        {
            // Epsilon-greedy: random action ∈ [-1, 1] for each dim
            std::vector<float> randAct(ACT_DIM);
            for (int d = 0; d < ACT_DIM; d++)
                randAct[d] = static_cast<float>(m_uniformDist(m_rng) * 2.0 - 1.0);
            act = torch::from_blob(randAct.data(), {ACT_DIM}, torch::kFloat32).clone();
            std::cout << "[ACT-MADDPG] Step=" << m_stepCount
                << " Cell" << srcCell << " RANDOM eps=" << m_epsilon << std::endl;
        }
        else
        {
            // Agent action + Gaussian noise
            act = m_agents[i]->SelectAction(currentObs[i], /*deterministic=*/true);
            auto noise = torch::randn_like(act) * m_explNoise;
            act = torch::clamp(act + noise, -1.0, 1.0);
        }

        currentActs.push_back(act);

        auto actData = act.accessor<float, 1>();

        // ── action[0] = self CIO, action[1] = TXP ──
        int selfCioDB = std::max(-5, std::min(5,
            static_cast<int>(std::round(actData[0] * 5.0))));
        m_cellCio[srcCell] = selfCioDB;

        double txpOffset = static_cast<double>(actData[1]) * 4.0;
        double txpApplied = m_txPower + txpOffset;
        txpApplied = std::max(26.0, std::min(38.0, txpApplied));

        // TXP 적용
        std::string ep = "/E2Node/" + std::to_string(srcCell) + "/";
        ric->E2SmRcSendTxPowerControlRequest(txpApplied, ep);

        // ── 콘솔 로그 ──
        std::cout << "[ACT-MADDPG] Step=" << m_stepCount
            << " Cell" << srcCell << " selfCIO=" << selfCioDB
            << "dB TXP=" << txpApplied << "dBm raw=[" << actData[0]
            << "," << actData[1] << "]" << std::endl;

        // ── CSV logging (every 5 steps) ──
        if (m_stepCount % 5 == 0)
        {
            double now = Simulator::Now().GetSeconds();
            double cellThp = 0.0;
            auto cellIt = m_cellContexts.find(srcCell);
            if (cellIt != m_cellContexts.end())
                cellThp = cellIt->second.totalThroughputDl + cellIt->second.totalThroughputUl;

            std::ostringstream oss;
            oss << now << "," << srcCell
                << "," << actData[0] << "," << actData[1]
                << "," << selfCioDB << "," << txpApplied
                << "," << cellThp;
            m_maddpgActionsBuf.push_back(oss.str());
        }
    }

    // ── Apply effective CIO: CIO(src→dst) = CIO_dst - CIO_src ──
    for (auto srcCellId : m_cellIds)
    {
        auto nit = m_neighborMap.find(srcCellId);
        if (nit == m_neighborMap.end()) continue;
        Json cioList = Json::array();
        int srcCio = m_cellCio.count(srcCellId) ? m_cellCio[srcCellId] : 0;
        for (auto dstCellId : nit->second)
        {
            int dstCio = m_cellCio.count(dstCellId) ? m_cellCio[dstCellId] : 0;
            int effectiveCio = std::max(-5, std::min(5, dstCio - srcCio));
            Json entry;
            entry["CELL_ID"] = dstCellId;
            entry["CIO_VALUE"] = effectiveCio * 2;  // IE units
            cioList.push_back(entry);

            if (m_stepCount % 5 == 0)
            {
                double now = Simulator::Now().GetSeconds();
                std::ostringstream oss;
                oss << now << "," << srcCellId << "," << dstCellId << ","
                    << effectiveCio << "," << (effectiveCio * 2);
                m_cioActionsBuf.push_back(oss.str());
            }
        }
        std::string epCio = "/E2Node/" + std::to_string(srcCellId) + "/";
        ric->E2SmRcSendCioControlRequest(cioList, epCio);
    }

    // ── Epsilon decay (Python MADDPG와 동일) ──
    if (!m_inferenceOnly && m_epsilon > m_epsilonEnd)
    {
        m_epsilon *= m_epsilonDecay;
    }

    // ── 고착화 체크 ──
    if (!stepRewards.empty())
    {
        CheckAndLogStagnation(currentActs, stepRewards);
    }

    // ── 다음 스텝을 위해 저장 ──
    m_prevObs = currentObs;
    m_prevActs = currentActs;
    m_hasPrevStep = true;
}

// =============================================================================
// MADDPG 학습 (Python MADDPG와 동일한 구조)
// =============================================================================
void
xAppHandoverSON::TrainMADDPG()
{
    NS_LOG_FUNCTION(this);

    static constexpr size_t WARMUP_STEPS = 200;
    if (m_replayBuffer->Size() < WARMUP_STEPS)
    {
        NS_LOG_UNCOND("[MADDPG] Warmup: " << m_replayBuffer->Size()
            << "/" << WARMUP_STEPS);
        return;
    }

    auto batch = m_replayBuffer->Sample(BATCH_SIZE);
    size_t B = batch.size();

    // ── 배치 텐서 구성 ──
    std::vector<torch::Tensor> obsBatch(NUM_AGENTS);
    std::vector<torch::Tensor> actsBatch(NUM_AGENTS);
    std::vector<torch::Tensor> nextObsBatch(NUM_AGENTS);
    std::vector<torch::Tensor> rewardBatch(NUM_AGENTS);
    auto doneBatch = torch::zeros({(long)B, 1});

    for (int a = 0; a < NUM_AGENTS; a++)
    {
        std::vector<torch::Tensor> oVec, aVec, noVec;
        rewardBatch[a] = torch::zeros({(long)B, 1});

        for (size_t b = 0; b < B; b++)
        {
            oVec.push_back(batch[b].obs[a]);
            aVec.push_back(batch[b].acts[a]);
            noVec.push_back(batch[b].nextObs[a]);
            rewardBatch[a][b][0] = batch[b].rewards[a];
            if (a == 0) doneBatch[b][0] = batch[b].done ? 1.0f : 0.0f;
        }
        obsBatch[a] = torch::stack(oVec);
        actsBatch[a] = torch::stack(aVec);
        nextObsBatch[a] = torch::stack(noVec);
    }

    auto allObs     = torch::cat(obsBatch, 1);
    auto allActs    = torch::cat(actsBatch, 1);
    auto allNextObs = torch::cat(nextObsBatch, 1);

    // Target 행동 계산
    std::vector<torch::Tensor> targetNextActs;
    {
        torch::NoGradGuard noGrad;
        for (int a = 0; a < NUM_AGENTS; a++)
        {
            auto act = m_agents[a]->SelectAction(nextObsBatch[a], true);
            targetNextActs.push_back(act);
        }
    }
    auto allTargetNextActs = torch::cat(targetNextActs, 1);

    // ── 에이전트별 업데이트 ──
    for (int i = 0; i < NUM_AGENTS; i++)
    {
        auto& agent = m_agents[i];
        // ─ Critic 업데이트 ─
        torch::Tensor targetQ;
        {
            torch::NoGradGuard noGrad;
            targetQ = agent->GetTargetCritic()->forward(allNextObs, allTargetNextActs);
            targetQ = rewardBatch[i] + GAMMA * (1.0 - doneBatch) * targetQ;
        }

        auto currentQ = agent->GetCritic()->forward(allObs, allActs);
        auto criticLoss = torch::mse_loss(currentQ, targetQ);

        agent->GetCriticOpt().zero_grad();
        criticLoss.backward();
        torch::nn::utils::clip_grad_norm_(agent->GetCritic()->parameters(), 0.5);
        agent->GetCriticOpt().step();

        // ─ Actor 업데이트 (action regularization to prevent tanh saturation) ─
        auto currAct = agent->GetActor()->forward(obsBatch[i]);
        std::vector<torch::Tensor> policyActs;
        for (int j = 0; j < NUM_AGENTS; j++)
        {
            if (j == i)
                policyActs.push_back(currAct);
            else
                policyActs.push_back(actsBatch[j].detach());
        }
        auto allPolicyActs = torch::cat(policyActs, 1);

        constexpr float ACT_REG = 0.01f;
        auto actorLoss = (-agent->GetCritic()->forward(allObs.detach(), allPolicyActs)
                         + ACT_REG * (currAct * currAct).sum(-1, /*keepdim=*/true)).mean();

        agent->GetActorOpt().zero_grad();
        actorLoss.backward();
        torch::nn::utils::clip_grad_norm_(agent->GetActor()->parameters(), 0.5);
        agent->GetActorOpt().step();

        // ─ Target Soft Update ─
        agent->SoftUpdateTargets(TAU_SOFT);

        if (m_stepCount % 10 == 0)
        {
            float curActorLoss  = actorLoss.item<float>();
            float curCriticLoss = criticLoss.item<float>();

            // ── Gradient Norm ──
            double actorGradNorm = 0.0, criticGradNorm = 0.0;
            for (auto& p : agent->GetActor()->parameters())
                if (p.grad().defined()) actorGradNorm += p.grad().norm().item<double>();
            for (auto& p : agent->GetCritic()->parameters())
                if (p.grad().defined()) criticGradNorm += p.grad().norm().item<double>();

            // ── Weight Norm (magnitude) ──
            double actorWeightNorm = 0.0, criticWeightNorm = 0.0;
            for (auto& p : agent->GetActor()->parameters())
                actorWeightNorm += p.norm().item<double>();
            for (auto& p : agent->GetCritic()->parameters())
                criticWeightNorm += p.norm().item<double>();

            // ── Action Distribution: mean, std, saturation ratio (|a|>0.95) ──
            float actMean = currAct.mean().item<float>();
            float actStd  = currAct.std().item<float>();
            float satRatio = (currAct.abs() > 0.95).to(torch::kFloat).mean().item<float>();

            std::cout << "[MADDPG Train] Step=" << m_stepCount
                      << " | Agent=" << i
                      << " | CriticLoss=" << curCriticLoss
                      << " | ActorLoss=" << curActorLoss
                      << " | Epsilon=" << m_epsilon
                      << " | GradA=" << actorGradNorm
                      << " | GradC=" << criticGradNorm
                      << " | SatR=" << satRatio
                      << std::endl;

            // ── CSV 로깅 ──
            std::ostringstream oss;
            oss << Simulator::Now().GetSeconds()
                << "," << m_stepCount << "," << i
                << "," << curActorLoss << "," << curCriticLoss
                << "," << actorGradNorm << "," << criticGradNorm
                << "," << actorWeightNorm << "," << criticWeightNorm
                << "," << actMean << "," << actStd << "," << satRatio;
            m_trainDiagBuf.push_back(oss.str());
        }
    }

    // ── 500스텝마다 고착화 체크 ──
    if (m_stepCount % 500 == 0)
    {
        std::cout << "\n===== [STAGNATION CHECK] Step=" << m_stepCount << " =====" << std::endl;
        for (size_t i = 0; i < m_agents.size(); i++)
        {
            auto actor = m_agents[i]->GetActor();

            // Weight norm
            double wNorm = 0.0;
            for (auto& p : actor->parameters()) wNorm += p.norm().item<double>();

            // Weight 변화량: 현재 vs target
            double targetDiff = 0.0;
            auto tgtActor = m_agents[i]->GetTargetActor();
            auto srcParams = actor->parameters();
            auto tgtParams = tgtActor->parameters();
            auto srcIt = srcParams.begin();
            auto tgtIt = tgtParams.begin();
            for (; srcIt != srcParams.end() && tgtIt != tgtParams.end(); ++srcIt, ++tgtIt)
                targetDiff += (*srcIt - *tgtIt).abs().mean().item<double>();

            // 테스트 행동 분포
            auto testObs = BuildObservation(m_agents[i]->GetConfig().cellId);
            actor->eval();
            auto testAct = actor->forward(testObs.unsqueeze(0)).squeeze(0);
            actor->train();
            float actMean = testAct.mean().item<float>();
            float actStd = testAct.std().item<float>();
            float satRatio = (testAct.abs() > 0.95).to(torch::kFloat).mean().item<float>();

            std::cout << "  Agent" << i
                      << " | wNorm=" << wNorm
                      << " | tgtDiff=" << targetDiff
                      << " | actMean=" << actMean
                      << " | actStd=" << actStd
                      << " | satR=" << satRatio
                      << std::endl;
        }
        std::cout << "=========================================\n" << std::endl;
    }
}

// =============================================================================
// 모델 저장/로드
// =============================================================================
void
xAppHandoverSON::SaveModels(const std::string& dir)
{
    NS_LOG_FUNCTION(this);
    if (m_baseline) return;

    if (!m_inferenceOnly)
    {
        std::string cmd = "mkdir -p " + dir;
        system(cmd.c_str());

        for (size_t i = 0; i < m_agents.size(); i++)
        {
            std::string prefix = dir + "/agent_" + std::to_string(i);

            torch::save(m_agents[i]->GetActor(), prefix + "_actor.pt");
            torch::save(m_agents[i]->GetCritic(), prefix + "_critic.pt");
            torch::save(m_agents[i]->GetTargetActor(), prefix + "_target_actor.pt");
            torch::save(m_agents[i]->GetTargetCritic(), prefix + "_target_critic.pt");

        }

        std::string bufferPath = dir + "/replay_buffer.pt";
        m_replayBuffer->Save(bufferPath);

        NS_LOG_UNCOND("[MADDPG] Models & Buffer saved to " << dir
            << "/ (step=" << m_stepCount << ", BufferSize=" << m_replayBuffer->Size() << ")");

        std::ofstream metaFile(dir + "/meta.txt");
        metaFile << m_stepCount << "\n" << m_epsilon << std::endl;
        metaFile.close();
    }
}

void
xAppHandoverSON::LoadModels(const std::string& dir)
{
    NS_LOG_FUNCTION(this);
    std::cout << "[MADDPG] Loading models..." << std::endl;

    for (size_t i = 0; i < m_agents.size(); i++)
    {
        std::string prefix = dir + "/agent_" + std::to_string(i);

        try
        {
            torch::load(m_agents[i]->GetActor(), prefix + "_actor.pt");
            torch::load(m_agents[i]->GetCritic(), prefix + "_critic.pt");
            torch::load(m_agents[i]->GetTargetActor(), prefix + "_target_actor.pt");
            torch::load(m_agents[i]->GetTargetCritic(), prefix + "_target_critic.pt");

        }
        catch (const std::exception& e)
        {
            NS_LOG_UNCOND("[MADDPG] Failed to load agent " << i
                << " from " << dir << ": " << e.what());
            return;
        }
    }

    std::string bufferPath = dir + "/replay_buffer.pt";
    try {
        m_replayBuffer->Load(bufferPath, OBS_DIM, 0, NUM_AGENTS);
        NS_LOG_UNCOND("[MADDPG] Buffer loaded. Size: " << m_replayBuffer->Size());
    } catch (const std::exception& e) {
        NS_LOG_WARN("[MADDPG] Failed to load buffer: " << e.what());
    }

    NS_LOG_UNCOND("[MADDPG] Models loaded from " << dir << "/");
    std::ifstream metaFile(dir + "/meta.txt");
    if (metaFile.is_open())
    {
        metaFile >> m_stepCount;
        if (metaFile >> m_epsilon)
        {
            NS_LOG_UNCOND("[MADDPG] Restored epsilon=" << m_epsilon);
        }
        metaFile.close();
        NS_LOG_UNCOND("[MADDPG] Restored stepCount=" << m_stepCount);
    }
}

// =============================================================================
// CSV 로깅 (buffered)
// =============================================================================
void
xAppHandoverSON::InitCsvLoggers()
{
    // EP1 학습 시작(loadPretrained=false, inference=false)만 헤더+덮어쓰기
    // 그 외(EP2+, inference)는 헤더 없이 append
    if (!m_loadPretrained && !m_inferenceOnly)
    {
        m_cellMetricsBuf.push_back("time_s,cellId,ueCount,edgeUeCount,cellDlThp_kbps,cellUlThp_kbps,avgCqi,txPower_dBm,prbUtilDl");
        m_cioActionsBuf.push_back("time_s,srcCellId,neighborCellId,cioDB,cioIE");
        m_maddpgActionsBuf.push_back("time_s,cellId,cioRaw,txpRaw,selfCioDB,txpApplied_dBm,cellThp_kbps,alpha");
        m_rewardCurveBuf.push_back("time_s,step,reward_cell1,reward_cell2,reward_cell3,prb_std,ue_std");
        m_stagnationBuf.push_back("time_s,step,cellId,cio_l2_change,max_cio_change,txp_raw_curr,txp_raw_prev,txp_raw_change,txp_dBm_curr,txp_dBm_prev,ue_count_prev,ue_count_curr,ue_count_change,reward_prev,reward_curr,reward_change,consecutive_stagnant_steps,cio_stagnant,txp_stagnant,is_stagnant");
        m_trainDiagBuf.push_back("time_s,step,agent,actor_loss,critic_loss,actor_grad_norm,critic_grad_norm,actor_weight_norm,critic_weight_norm,act_mean,act_std,sat_ratio");
    }
}

void
xAppHandoverSON::LogCellMetrics()
{
    double now = Simulator::Now().GetSeconds();
    for (auto& [cellId, cell] : m_cellContexts)
    {
        float avgCqi = 0.0f;
        uint32_t cqiCount = 0;
        for (auto& [key, ue] : m_ueContexts)
        {
            if (ue.servingCellId == cellId) { avgCqi += ue.cqi; cqiCount++; }
        }
        if (cqiCount > 0) avgCqi /= cqiCount;

        std::ostringstream oss;
        oss << now << "," << cellId << ","
            << cell.ueCount << "," << cell.edgeUeCount << ","
            << cell.totalThroughputDl << "," << cell.totalThroughputUl << ","
            << avgCqi << "," << cell.txPower << ","
            << cell.prbUtilDl;
        m_cellMetricsBuf.push_back(oss.str());
    }
}

// =============================================================================
// CSV 버퍼 → 파일 일괄 쓰기 (시뮬레이션 종료 시 1회)
// =============================================================================
void
xAppHandoverSON::FlushCsvLogs()
{
    auto writeFile = [](const std::string& filename, const std::vector<std::string>& buf, bool append) {
        if (buf.empty()) return;
        std::ofstream f(filename, append ? std::ios::app : std::ios::out);
        for (const auto& line : buf)
            f << line << "\n";
        f.close();
    };

    bool append = (m_loadPretrained || m_inferenceOnly);
    writeFile("cell_metrics.csv",     m_cellMetricsBuf,     append);
    writeFile("cio_actions.csv",      m_cioActionsBuf,      append);
    writeFile("maddpg_actions.csv",   m_maddpgActionsBuf,   append);
    writeFile("reward_curve.csv",     m_rewardCurveBuf,     append);
    writeFile("train_diag.csv",      m_trainDiagBuf,       append);
    writeFile("stagnation_check.csv", m_stagnationBuf,      append);

    NS_LOG_UNCOND("[MADDPG] CSV logs flushed ("
        << m_cellMetricsBuf.size() << " cell_metrics, "
        << m_cioActionsBuf.size() << " cio_actions, "
        << m_rewardCurveBuf.size() << " reward_curve rows)");
}

// =============================================================================
// 고착화 체크 (CIO + TXP)
// =============================================================================
void
xAppHandoverSON::CheckAndLogStagnation(
    const std::vector<torch::Tensor>& currentActs,
    const std::vector<double>& rewards)
{
    double now = Simulator::Now().GetSeconds();
    bool allStagnant = true;

    for (size_t i = 0; i < m_agents.size(); i++)
    {
        uint16_t cellId = m_agents[i]->GetConfig().cellId;
        auto actData = currentActs[i].accessor<float, 1>();
        int64_t actDim = currentActs[i].size(0);

        std::vector<float> currAct(actDim);
        for (int64_t d = 0; d < actDim; d++)
            currAct[d] = actData[d];

        // CIO = action[0], TXP = action[1]
        float txpRaw = (actDim > 1) ? currAct[1] : 0.0f;
        double txpApplied = m_txPower + static_cast<double>(txpRaw) * 4.0;
        txpApplied = std::max(26.0, std::min(38.0, txpApplied));

        double cioL2Change = -1.0;
        double maxCioChange = -1.0;
        double txpRawChange = -1.0;
        float prevTxpRaw = txpRaw;

        auto prevIt = m_prevRawActions.find(cellId);
        if (prevIt != m_prevRawActions.end() && (int64_t)prevIt->second.size() == actDim)
        {
            // CIO change (action[0] only)
            double diff = std::abs(currAct[0] - prevIt->second[0]);
            cioL2Change = diff;
            maxCioChange = diff;
            // TXP change
            if (actDim > 1)
            {
                prevTxpRaw = prevIt->second[1];
                txpRawChange = std::abs(txpRaw - prevTxpRaw);
            }
        }

        double prevTxpApplied = m_txPower + static_cast<double>(prevTxpRaw) * 4.0;
        prevTxpApplied = std::max(26.0, std::min(38.0, prevTxpApplied));

        uint32_t ueCountCurr = 0;
        auto cellIt = m_cellContexts.find(cellId);
        if (cellIt != m_cellContexts.end()) ueCountCurr = cellIt->second.ueCount;

        int ueCountChange = 0;
        uint32_t ueCountPrev = ueCountCurr;
        auto uePrevIt = m_prevUeCounts.find(cellId);
        if (uePrevIt != m_prevUeCounts.end())
        {
            ueCountPrev = uePrevIt->second;
            ueCountChange = static_cast<int>(ueCountCurr) - static_cast<int>(ueCountPrev);
        }

        double rewardCurr = (i < rewards.size()) ? rewards[i] : 0.0;
        double rewardPrev = rewardCurr;
        double rewardChange = 0.0;
        if (i < m_prevRewards.size())
        {
            rewardPrev = m_prevRewards[i];
            rewardChange = std::abs(rewardCurr - rewardPrev);
        }

        bool cioStagnant = (cioL2Change >= 0.0 && cioL2Change < STAGNATION_ACTION_THRESH);
        bool txpStagnant = (txpRawChange >= 0.0 && txpRawChange < STAGNATION_ACTION_THRESH);
        bool isStagnant = cioStagnant && txpStagnant
                       && (rewardChange < STAGNATION_REWARD_THRESH);
        if (!isStagnant) allStagnant = false;

        {
            std::ostringstream oss;
            oss << now << "," << m_stepCount << "," << cellId << ","
                << cioL2Change << "," << maxCioChange << ","
                << txpRaw << "," << prevTxpRaw << "," << txpRawChange << ","
                << txpApplied << "," << prevTxpApplied << ","
                << ueCountPrev << "," << ueCountCurr << "," << ueCountChange << ","
                << rewardPrev << "," << rewardCurr << "," << rewardChange << ","
                << m_stagnantSteps << ","
                << (cioStagnant ? 1 : 0) << ","
                << (txpStagnant ? 1 : 0) << ","
                << (isStagnant ? 1 : 0);
            m_stagnationBuf.push_back(oss.str());
        }

        m_prevRawActions[cellId] = currAct;
        m_prevUeCounts[cellId] = ueCountCurr;
    }

    if (allStagnant && !m_prevRewards.empty())
        m_stagnantSteps++;
    else
        m_stagnantSteps = 0;

    m_prevRewards = rewards;

    if (m_stagnantSteps >= 10 && m_stagnantSteps % 10 == 0)
    {
        NS_LOG_UNCOND("[STAGNATION-MADDPG] WARNING: " << m_stagnantSteps
            << " consecutive stagnant steps at t=" << now << "s");
    }
}
