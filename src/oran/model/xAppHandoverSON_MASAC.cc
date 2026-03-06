#include "xAppHandoverSON_MASAC.h"

#include "ns3/E2AP.h"
#include "ns3/core-module.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <numeric>
#include <random>

using namespace ns3;
using namespace oran;

NS_LOG_COMPONENT_DEFINE("xAppHandoverSON_MASAC");

xAppHandoverSON_MASAC::xAppHandoverSON_MASAC(float sonPeriodicitySec, bool initiateHandovers,
                                             bool loadPretrained, bool inferenceOnly,
                                             double simStopTime)
    : xAppHandover(),
      m_sonPeriodicitySec(sonPeriodicitySec),
      m_initiateHandovers(initiateHandovers),
      m_edgeRsrpThreshold(-85.28),
      m_loadThreshold(12.0),
      m_rsrqThreshold(-15.0),
      m_cqiThreshold(11),
      m_txPower(32.0),
      m_frequency(2.12e9),
      m_dlBandwidthPrb(100)
{
    NS_LOG_FUNCTION(this);

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&xAppHandoverSON_MASAC::HandoverSucceeded, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&xAppHandoverSON_MASAC::HandoverStarted, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                    MakeCallback(&xAppHandoverSON_MASAC::HandoverFailed, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback(&xAppHandoverSON_MASAC::ConnectionEstablished, this));

    m_loadPretrained = loadPretrained;
    m_inferenceOnly  = inferenceOnly;
    m_simStopTime    = simStopTime;

    if (m_useMADDPG)
    {
        InitMASAC();
    }
    InitCsvLoggers();

    Simulator::Schedule(Seconds(m_sonPeriodicitySec),
                        &xAppHandoverSON_MASAC::PeriodicSONCheck,
                        this);
}

void
xAppHandoverSON_MASAC::PeriodicSONCheck()
{
    NS_LOG_FUNCTION(this);

    CollectKPMs();

    // EMA smoothing of cell metrics
    for (auto& [cellId, cell] : m_cellContexts)
    {
        auto& s = m_smoothed[cellId];
        if (!s.initialized)
        {
            s.ueCount = cell.ueCount;
            s.edgeUeCount = cell.edgeUeCount;
            s.dlThp = cell.totalThroughputDl;
            s.ulThp = cell.totalThroughputUl;
            s.initialized = true;
        }
        else
        {
            s.ueCount     = EMA_ALPHA * cell.ueCount     + (1.0 - EMA_ALPHA) * s.ueCount;
            s.edgeUeCount = EMA_ALPHA * cell.edgeUeCount + (1.0 - EMA_ALPHA) * s.edgeUeCount;
            s.dlThp       = EMA_ALPHA * cell.totalThroughputDl + (1.0 - EMA_ALPHA) * s.dlThp;
            s.ulThp       = EMA_ALPHA * cell.totalThroughputUl + (1.0 - EMA_ALPHA) * s.ulThp;
        }
    }

    if (m_stepCount % 5 == 0)
    {
        LogCellMetrics();
    }
    NS_LOG_LOGIC("cell 1: " << m_cellContexts[1].ueCount
        << " cell 2: " << m_cellContexts[2].ueCount
        << " cell 3: " << m_cellContexts[3].ueCount);
    NS_LOG_LOGIC("[SON] === Periodic Check === UEs=" << m_ueContexts.size()
        << " Cells=" << m_cellContexts.size());

    std::cout << "\n[TICK] PeriodicSONCheck(MASAC) - Step: " << m_stepCount
              << " | UE count: " << m_ueContexts.size() << std::endl;

    if (m_useMADDPG)
    {
        StepMASAC();

        if (!m_inferenceOnly)
        {
            TrainMASAC();
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
                        &xAppHandoverSON_MASAC::PeriodicSONCheck,
                        this);
}

// =============================================================================
// 데이터 수집 (same as original)
// =============================================================================
void
xAppHandoverSON_MASAC::CollectKPMs()
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
xAppHandoverSON_MASAC::CollectRsrpRsrq()
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
            if (m_ueContexts.find(key) != m_ueContexts.end())
                continue;
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
xAppHandoverSON_MASAC::CollectTargetRsrq()
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
xAppHandoverSON_MASAC::CollectCqi()
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
xAppHandoverSON_MASAC::CollectCellKpms()
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
            if (mostRecentTimestamp.empty()) mostRecentTimestamp = m.timestamp;
            if (mostRecentTimestamp != m.timestamp) continue;
            if (!m.measurements.contains("CELLID") || !m.measurements.contains("VALUE")) continue;
            uint16_t cellId = m.measurements["CELLID"];
            int ueCount = m.measurements["VALUE"];
            if (m_cellContexts.find(cellId) == m_cellContexts.end())
            {
                m_cellContexts[cellId] = CellContext_MASAC();
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
            if (mostRecentTimestamp.empty()) mostRecentTimestamp = m.timestamp;
            if (mostRecentTimestamp != m.timestamp) continue;
            if (!m.measurements.contains("CELLID") || !m.measurements.contains("VALUE")) continue;
            uint16_t cellId = m.measurements["CELLID"];
            double txPower = m.measurements["VALUE"];
            if (m_cellContexts.find(cellId) == m_cellContexts.end())
            {
                m_cellContexts[cellId] = CellContext_MASAC();
                m_cellContexts[cellId].cellId = cellId;
            }
            m_cellContexts[cellId].txPower = txPower;
        }
    }
}

void
xAppHandoverSON_MASAC::CollectUeCount()
{
    NS_LOG_FUNCTION(this);
    for (auto& [key, ue] : m_ueContexts)
    {
        uint16_t cellId = ue.servingCellId;
        if (m_cellContexts.find(cellId) == m_cellContexts.end())
        {
            m_cellContexts[cellId] = CellContext_MASAC();
            m_cellContexts[cellId].cellId = cellId;
        }
        if (ue.isEdge)
        {
            m_cellContexts[cellId].edgeUeCount++;
        }
    }
}

void
xAppHandoverSON_MASAC::CollectCellThroughput()
{
    NS_LOG_FUNCTION(this);
    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");
    if (!ric) return;

    for (auto& [cellId, cell] : m_cellContexts)
    {
        cell.totalThroughputDl = 0.0;
        cell.totalThroughputUl = 0.0;
    }

    auto dlMap = ric->QueryKpmMetric("/KPM/DRB.IpVolDl.QCI");
    std::map<uint16_t, double> latestDlBytes;
    for (auto& [endpoint, measurements] : dlMap)
    {
        for (auto& m : measurements)
        {
            if (!m.measurements.contains("CELLID") || !m.measurements.contains("VALUE")) continue;
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

    auto ulMap = ric->QueryKpmMetric("/KPM/DRB.IpVolUl.QCI");
    std::map<uint16_t, double> latestUlBytes;
    for (auto& [endpoint, measurements] : ulMap)
    {
        for (auto& m : measurements)
        {
            if (!m.measurements.contains("CELLID") || !m.measurements.contains("VALUE")) continue;
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

    auto prbMap = ric->QueryKpmMetric("/KPM/RRU.PrbTotDl");
    for (auto& [endpoint, measurements] : prbMap)
    {
        for (auto& m : measurements)
        {
            if (!m.measurements.contains("CELLID") || !m.measurements.contains("PRB_UTIL")) continue;
            uint16_t cellId = m.measurements["CELLID"];
            double util = m.measurements["PRB_UTIL"];
            auto it = m_cellContexts.find(cellId);
            if (it != m_cellContexts.end())
                it->second.prbUtilDl = util;
        }
    }

    for (auto& [cellId, cell] : m_cellContexts)
    {
        double dlDelta = cell.totalThroughputDl;
        double ulDelta = cell.totalThroughputUl;
        if (dlDelta > 0 || ulDelta > 0)
        {
            cell.totalThroughputDl = (cell.totalThroughputDl * 8.0) / 1000.0 / m_sonPeriodicitySec;
            cell.totalThroughputUl = (cell.totalThroughputUl * 8.0) / 1000.0 / m_sonPeriodicitySec;
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
// Edge UE / Load
// =============================================================================
void
xAppHandoverSON_MASAC::CalculateEdgeUEs()
{
    NS_LOG_FUNCTION(this);
    for (auto& [rnti, ue] : m_ueContexts)
    {
        ue.isEdge = (ue.servingRsrp < m_edgeRsrpThreshold);
    }
}

void
xAppHandoverSON_MASAC::CalculateLoadScores()
{
    NS_LOG_FUNCTION(this);
    for (auto& [cellId, cell] : m_cellContexts)
    {
        cell.loadScore = cell.ueCount * 2.0 + cell.edgeUeCount * 3.0;
    }
}

bool
xAppHandoverSON_MASAC::IsCellOverloaded(uint16_t cellId)
{
    auto it = m_cellContexts.find(cellId);
    if (it == m_cellContexts.end()) return false;
    return it->second.loadScore > m_loadThreshold;
}

// =============================================================================
// 의사결정 (규칙 기반 fallback)
// =============================================================================
uint16_t
xAppHandoverSON_MASAC::MakeSONDecision(UeKey key)
{
    auto it = m_ueContexts.find(key);
    if (it == m_ueContexts.end()) return std::numeric_limits<uint16_t>::max();
    auto& ue = it->second;
    if (IsCellOverloaded(ue.servingCellId) && ue.isEdge)
        return FindLeastLoadedNeighbor(key);
    if (ue.servingRsrq < m_rsrqThreshold)
        return FindBestRsrqCell(key);
    if (ue.cqi < m_cqiThreshold && ue.cqi > 0)
        return FindLeastLoadedNeighbor(key);
    return std::numeric_limits<uint16_t>::max();
}

uint16_t
xAppHandoverSON_MASAC::FindLeastLoadedNeighbor(UeKey key)
{
    auto ueIt = m_ueContexts.find(key);
    if (ueIt == m_ueContexts.end()) return std::numeric_limits<uint16_t>::max();
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
xAppHandoverSON_MASAC::FindBestRsrqCell(UeKey key)
{
    auto ueIt = m_ueContexts.find(key);
    if (ueIt == m_ueContexts.end()) return std::numeric_limits<uint16_t>::max();
    auto& ue = ueIt->second;
    if (ue.neighborRsrq.empty()) return FindLeastLoadedNeighbor(key);
    uint16_t bestCell = std::numeric_limits<uint16_t>::max();
    double bestRsrq = -std::numeric_limits<double>::max();
    for (auto& [targetCellId, rsrq] : ue.neighborRsrq)
    {
        if (rsrq > bestRsrq) { bestRsrq = rsrq; bestCell = targetCellId; }
    }
    if (bestRsrq <= ue.servingRsrq) return std::numeric_limits<uint16_t>::max();
    return bestCell;
}

void
xAppHandoverSON_MASAC::PurgeStaleUeContexts()
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
// 핸드오버 콜백
// =============================================================================
void
xAppHandoverSON_MASAC::HandoverDecision(Json& payload)
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

void
xAppHandoverSON_MASAC::HandoverStarted(std::string context, uint64_t imsi, uint16_t cellid,
                         uint16_t rnti, uint16_t targetCellId)
{
    UeKey key = MakeUeKey(cellid, rnti);
    m_imsiInHandover[key] = imsi;
}

void
xAppHandoverSON_MASAC::HandoverSucceeded(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    for (auto it = m_imsiInHandover.begin(); it != m_imsiInHandover.end(); ++it)
    {
        if (it->second == imsi) { m_staleKeys.insert(it->first); m_imsiInHandover.erase(it); break; }
    }
}

void
xAppHandoverSON_MASAC::HandoverFailed(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    for (auto it = m_imsiInHandover.begin(); it != m_imsiInHandover.end(); ++it)
    {
        if (it->second == imsi) { m_imsiInHandover.erase(it); break; }
    }
}

void
xAppHandoverSON_MASAC::ConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    for (auto it = m_imsiInHandover.begin(); it != m_imsiInHandover.end(); ++it)
    {
        if (it->second == imsi) { m_imsiInHandover.erase(it); break; }
    }
}

// =============================================================================
// MASAC 초기화
// =============================================================================
void
xAppHandoverSON_MASAC::InitMASAC()
{
    NS_LOG_FUNCTION(this);

    m_cellIds = {1, 2, 3};
    m_neighborMap[1] = {2, 3};
    m_neighborMap[2] = {1, 3};
    m_neighborMap[3] = {1, 2};

    int64_t totalObsDim = NUM_AGENTS * OBS_DIM;
    int64_t totalActDim = NUM_AGENTS * 2;  // each agent: [CIO_self, TXP]

    for (auto cellId : m_cellIds)
    {
        AgentConfig_MASAC config;
        config.cellId = cellId;
        config.obsDim = OBS_DIM;
        config.actDim = 2;  // [CIO_self, TXP]
        config.neighborCellIds = m_neighborMap[cellId];

        m_agents.push_back(std::make_unique<MASACAgent>(
            config, totalObsDim, totalActDim, ACTOR_LR, CRITIC_LR));
    }

    m_replayBuffer = std::make_unique<ReplayBuffer_MASAC>(BUFFER_SIZE);

    NS_LOG_UNCOND("[MASAC] Initialized: " << NUM_AGENTS << " agents, "
        << "Obs=" << OBS_DIM << " totalActDim=" << totalActDim);

    if (m_loadPretrained)
    {
        LoadModels();
    }
}

// =============================================================================
// 관측 벡터 (same as original)
// =============================================================================
torch::Tensor
xAppHandoverSON_MASAC::BuildObservation(uint16_t cellId)
{
    std::vector<float> obs(OBS_DIM, 0.0f);

    auto cellIt = m_cellContexts.find(cellId);
    if (cellIt == m_cellContexts.end())
        return torch::zeros({OBS_DIM});

    const auto& cell = cellIt->second;

    float avgCqi = 0.0f;
    uint32_t cqiCount = 0;
    uint32_t edgeCount = 0;
    uint32_t ueWithData = 0;

    for (auto& [key, ue] : m_ueContexts)
    {
        if (ue.servingCellId == cellId)
        {
            ueWithData++;
            avgCqi += ue.cqi;
            cqiCount++;
            if (ue.isEdge) edgeCount++;
        }
    }
    if (cqiCount > 0) avgCqi /= cqiCount;
    obs[0] = avgCqi / 15.0f;
    obs[1] = static_cast<float>(cell.totalThroughputDl / 1e3) / 5.0f;
    obs[2] = (ueWithData > 0) ? static_cast<float>(edgeCount) / ueWithData : 0.0f;
    uint32_t totalUEs = m_ueContexts.size();
    obs[3] = (totalUEs > 0) ? static_cast<float>(cell.ueCount) / static_cast<float>(totalUEs) : 0.0f;

    return torch::from_blob(obs.data(), {OBS_DIM}, torch::kFloat32).clone();
}

// =============================================================================
// 보상 함수 (same as original)
// =============================================================================
std::vector<double>
xAppHandoverSON_MASAC::ComputeRewards()
{
    // EMA-smoothed values for thp, ueCount; raw prbUtil
    std::vector<double> thps;
    std::vector<double> prbUtils;
    std::vector<double> ueCounts;
    double totalUEs = 0.0;
    thps.reserve(m_cellIds.size());
    prbUtils.reserve(m_cellIds.size());
    ueCounts.reserve(m_cellIds.size());

    for (auto cellId : m_cellIds)
    {
        auto sit = m_smoothed.find(cellId);
        auto cit = m_cellContexts.find(cellId);
        if (sit != m_smoothed.end() && sit->second.initialized)
        {
            thps.push_back((sit->second.dlThp + sit->second.ulThp) / 1e3 / 5.0);
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

    // Reward: total cell throughput (incentivizes filling idle PRBs via load balancing)
    double w_thp = 10.0;

    std::vector<double> rewards;
    for (size_t i = 0; i < thps.size(); i++)
    {
        double r = w_thp * thps[i];
        rewards.push_back(r);
    }

    // For logging only
    double meanPrb = std::accumulate(prbUtils.begin(), prbUtils.end(), 0.0) / prbUtils.size();
    double prbVar = 0.0;
    for (double p : prbUtils) prbVar += (p - meanPrb) * (p - meanPrb);
    double stdPrb = std::sqrt(prbVar / prbUtils.size());
    double stdUeNorm = 0.0;
    if (totalUEs > 0) {
        double meanUe = totalUEs / ueCounts.size();
        double ueVar = 0.0;
        for (double u : ueCounts) ueVar += (u - meanUe) * (u - meanUe);
        stdUeNorm = std::sqrt(ueVar / ueCounts.size()) / totalUEs;
    }

    if (m_rewardCurveCsv.is_open())
    {
        double now = Simulator::Now().GetSeconds();
        m_rewardCurveCsv << now << "," << m_stepCount;
        for (size_t i = 0; i < rewards.size(); i++)
            m_rewardCurveCsv << "," << rewards[i];
        m_rewardCurveCsv << "," << stdPrb << "," << stdUeNorm << "\n";
    }

    return rewards;
}

// =============================================================================
// MASAC 스텝: 관측 → 행동 → CIO/TXP 적용
// =============================================================================
void
xAppHandoverSON_MASAC::StepMASAC()
{
    NS_LOG_FUNCTION(this);

    // ── 현재 관측 수집 (1회 순회로 모든 셀 obs 동시 계산) ──
    std::vector<torch::Tensor> currentObs;
    {
        // 셀별 집계를 위한 임시 구조체
        struct CellObs { float sumCqi = 0; uint32_t cqiCount = 0; uint32_t edgeCount = 0; uint32_t ueWithData = 0; };
        std::map<uint16_t, CellObs> cellObsMap;
        for (auto cellId : m_cellIds) cellObsMap[cellId] = {};

        // 1회 순회로 모든 셀의 UE 통계 수집
        for (auto& [key, ue] : m_ueContexts)
        {
            auto it = cellObsMap.find(ue.servingCellId);
            if (it != cellObsMap.end())
            {
                it->second.ueWithData++;
                it->second.sumCqi += ue.cqi;
                it->second.cqiCount++;
                if (ue.isEdge) it->second.edgeCount++;
            }
        }

        uint32_t totalUEs = m_ueContexts.size();
        for (size_t i = 0; i < m_agents.size(); i++)
        {
            uint16_t cellId = m_agents[i]->GetConfig().cellId;
            std::vector<float> obs(OBS_DIM, 0.0f);
            auto cellIt = m_cellContexts.find(cellId);
            if (cellIt == m_cellContexts.end())
            {
                currentObs.push_back(torch::zeros({OBS_DIM}));
                continue;
            }
            const auto& co = cellObsMap[cellId];
            const auto& sm = m_smoothed[cellId];
            float avgCqi = (co.cqiCount > 0) ? co.sumCqi / co.cqiCount : 0.0f;
            double smoothTotal = 0.0;
            for (auto& [cid, s] : m_smoothed) smoothTotal += s.ueCount;
            obs[0] = avgCqi / 15.0f;
            obs[1] = static_cast<float>(sm.dlThp / 1e3) / 5.0f;
            obs[2] = (sm.ueCount > 0.5) ? static_cast<float>(sm.edgeUeCount / sm.ueCount) : 0.0f;
            obs[3] = (smoothTotal > 0.5) ? static_cast<float>(sm.ueCount / smoothTotal) : 0.0f;
            currentObs.push_back(torch::from_blob(obs.data(), {OBS_DIM}, torch::kFloat32).clone());
        }
    }

    // ── 이전 전이를 n-step buffer에 저장 후 replay buffer로 flush ──
    std::vector<double> stepRewards;
    if (m_hasPrevStep)
    {
        stepRewards = ComputeRewards();
        double now = Simulator::Now().GetSeconds();
        bool isDone = (m_simStopTime - now <= m_sonPeriodicitySec * 2.0);

        if (!m_inferenceOnly)
        {
            // n-step 버퍼에 현재 전이 추가
            NStepTransition trans;
            trans.obs = m_prevObs;
            trans.acts = m_prevActs;
            trans.rewards = stepRewards;
            m_nStepBuffer.push_back(std::move(trans));

            // n-step 버퍼가 N_STEP개 모이면 replay buffer에 push
            if ((int)m_nStepBuffer.size() >= N_STEP || isDone)
            {
                // 가장 오래된 전이의 obs/acts를 시작점으로 사용
                const auto& oldest = m_nStepBuffer.front();

                // n-step discounted reward 계산
                std::vector<double> nStepRewards(NUM_AGENTS, 0.0);
                for (int k = 0; k < (int)m_nStepBuffer.size(); k++)
                {
                    double discount = std::pow(GAMMA, k);
                    for (int a = 0; a < NUM_AGENTS; a++)
                    {
                        nStepRewards[a] += discount * m_nStepBuffer[k].rewards[a];
                    }
                }

                Experience_MASAC exp;
                exp.obs = oldest.obs;
                exp.acts = oldest.acts;
                exp.rewards = nStepRewards;
                exp.nextObs = currentObs;  // s_{t+n}
                exp.done = isDone;
                m_replayBuffer->Push(std::move(exp));

                // 가장 오래된 전이 제거 (sliding window)
                m_nStepBuffer.pop_front();
            }

            // 에피소드 종료 시 남은 전이도 flush
            if (isDone)
            {
                while (!m_nStepBuffer.empty())
                {
                    const auto& oldest = m_nStepBuffer.front();
                    std::vector<double> nStepRewards(NUM_AGENTS, 0.0);
                    for (int k = 0; k < (int)m_nStepBuffer.size(); k++)
                    {
                        double discount = std::pow(GAMMA, k);
                        for (int a = 0; a < NUM_AGENTS; a++)
                        {
                            nStepRewards[a] += discount * m_nStepBuffer[k].rewards[a];
                        }
                    }

                    Experience_MASAC exp;
                    exp.obs = oldest.obs;
                    exp.acts = oldest.acts;
                    exp.rewards = nStepRewards;
                    exp.nextObs = currentObs;
                    exp.done = true;
                    m_replayBuffer->Push(std::move(exp));

                    m_nStepBuffer.pop_front();
                }
            }
        }

        if (m_stepCount % 10 == 0)
        {
            NS_LOG_UNCOND("[MASAC] Step=" << m_stepCount
                << " Rewards=[" << stepRewards[0] << ", "
                << stepRewards[1] << ", " << stepRewards[2] << "]"
                << " BufferSize=" << m_replayBuffer->Size());
        }
    }

    // ── 행동 선택 (no epsilon-greedy, SAC stochastic policy) ──
    std::vector<torch::Tensor> currentActs;

    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");
    if (!ric) return;

    for (size_t i = 0; i < m_agents.size(); i++)
    {
        torch::Tensor act;
        uint16_t srcCell = m_agents[i]->GetConfig().cellId;
        int64_t agentActDim = m_agents[i]->GetConfig().actDim;

        if (m_inferenceOnly)
        {
            // Inference: deterministic (tanh(μ))
            act = m_agents[i]->SelectAction(currentObs[i], /*deterministic=*/true);
        }
        else
        {
            // Training: stochastic (reparameterized Gaussian + tanh)
            act = m_agents[i]->SelectAction(currentObs[i], /*deterministic=*/false);
        }

        currentActs.push_back(act);

        auto actData = act.accessor<float, 1>();
        const auto& neighbors = m_agents[i]->GetConfig().neighborCellIds;

        // ── Per-cell CIO: action[0] = self CIO, action[1] = TXP ──
        int selfCioDB = std::max(-5, std::min(5,
            static_cast<int>(std::round(actData[0] * 5.0))));
        m_cellCio[srcCell] = selfCioDB;

        std::string endpoint = "/E2Node/" + std::to_string(srcCell) + "/";
        double txpOffset = static_cast<double>(actData[1]) * 4.0;
        double txpApplied = std::max(26.0, std::min(38.0, m_txPower + txpOffset));
        ric->E2SmRcSendTxPowerControlRequest(txpApplied, endpoint);

        // ── 콘솔 로그 ──
        std::cout << "[ACT-MASAC] Step=" << m_stepCount
            << " Cell" << srcCell << " selfCIO=" << selfCioDB
            << "dB TXP=" << txpApplied << "dBm raw=["
            << actData[0] << "," << actData[1] << "]" << std::endl;

        // ── CSV logging (every 5 steps) ──
        if (m_stepCount % 5 == 0)
        {
            double now = Simulator::Now().GetSeconds();
            double cellThp = 0.0;
            auto cellIt = m_cellContexts.find(srcCell);
            if (cellIt != m_cellContexts.end())
                cellThp = cellIt->second.totalThroughputDl + cellIt->second.totalThroughputUl;

            double alpha_val = m_agents[i]->GetLogAlpha().exp().item<double>();
            m_maddpgActionsCsv << now << "," << srcCell
                << "," << actData[0] << "," << actData[1]
                << "," << selfCioDB << "," << txpApplied
                << "," << cellThp << "," << alpha_val << "\n";
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

            // CSV logging
            if (m_stepCount % 5 == 0 && m_cioActionsCsv.is_open())
            {
                double now = Simulator::Now().GetSeconds();
                m_cioActionsCsv << now << ","
                    << srcCellId << "," << dstCellId << ","
                    << effectiveCio << "," << (effectiveCio * 2) << "\n";
            }
        }
        std::string ep = "/E2Node/" + std::to_string(srcCellId) + "/";
        ric->E2SmRcSendCioControlRequest(cioList, ep);
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
// MASAC 학습
// =============================================================================
// SAC 특성:
//   - Twin Critic (Q1, Q2) → min(Q1,Q2)로 overestimation 방지
//   - No target actor → 현재 actor의 sample()로 target action
//   - Critic loss: MSE(Q1, target) + MSE(Q2, target)
//   - Actor loss: (α*log_π - min(Q1,Q2)).mean()
//   - Alpha loss: -(log_α * (log_π + H_target)).mean()
void
xAppHandoverSON_MASAC::TrainMASAC()
{
    NS_LOG_FUNCTION(this);

    if (m_replayBuffer->Size() < BATCH_SIZE)
    {
        NS_LOG_UNCOND("[MASAC] Warmup: " << m_replayBuffer->Size()
            << "/" << BATCH_SIZE);
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

    // ── Target 행동 계산 (from current actor, no target actor in SAC) ──
    std::vector<torch::Tensor> targetNextActs;
    std::vector<torch::Tensor> targetNextLogProbs;
    {
        torch::NoGradGuard noGrad;
        for (int a = 0; a < NUM_AGENTS; a++)
        {
            auto [act, logp] = m_agents[a]->GetActor()->sample(nextObsBatch[a]);
            targetNextActs.push_back(act);
            targetNextLogProbs.push_back(logp);
        }
    }
    auto allTargetNextActs = torch::cat(targetNextActs, 1);

    // ── 에이전트별 업데이트 ──
    for (int i = 0; i < NUM_AGENTS; i++)
    {
        auto& agent = m_agents[i];
        auto alpha = agent->GetLogAlpha().exp().detach();

        // ─ Twin Critic 업데이트 ─
        torch::Tensor targetQ;
        {
            torch::NoGradGuard noGrad;
            auto [tq1, tq2] = agent->GetTargetCritic()->forward(allNextObs, allTargetNextActs);
            auto minTargetQ = torch::min(tq1, tq2);
            double gammaN = std::pow(GAMMA, N_STEP);
            targetQ = rewardBatch[i] + gammaN * (1.0 - doneBatch)
                    * (minTargetQ - alpha * targetNextLogProbs[i]);
        }

        auto [q1, q2] = agent->GetCritic()->forward(allObs, allActs);
        auto criticLoss = torch::mse_loss(q1, targetQ) + torch::mse_loss(q2, targetQ);

        agent->GetCriticOpt().zero_grad();
        criticLoss.backward();
        torch::nn::utils::clip_grad_norm_(agent->GetCritic()->parameters(), 0.5);
        agent->GetCriticOpt().step();

        // ─ Actor 업데이트 ─
        auto [currAct, currLogProb] = agent->GetActor()->sample(obsBatch[i]);
        std::vector<torch::Tensor> policyActs;
        for (int j = 0; j < NUM_AGENTS; j++)
        {
            if (j == i)
                policyActs.push_back(currAct);
            else
                policyActs.push_back(actsBatch[j].detach());
        }
        auto allPolicyActs = torch::cat(policyActs, 1);

        auto [pq1, pq2] = agent->GetCritic()->forward(allObs.detach(), allPolicyActs);
        auto minQ = torch::min(pq1, pq2);
        auto actorLoss = (alpha * currLogProb - minQ).mean();

        agent->GetActorOpt().zero_grad();
        actorLoss.backward();
        torch::nn::utils::clip_grad_norm_(agent->GetActor()->parameters(), 0.5);
        agent->GetActorOpt().step();

        // ─ Alpha 업데이트 ─
        auto alphaLoss = -(agent->GetLogAlpha()
            * (currLogProb.detach() + agent->GetTargetEntropy())).mean();
        agent->GetAlphaOpt().zero_grad();
        alphaLoss.backward();
        agent->GetAlphaOpt().step();

        // ─ Target Critic Soft Update (no target actor) ─
        agent->SoftUpdateTargets(TAU_SOFT);

        if (m_stepCount % 10 == 0)
        {
            float curActorLoss  = actorLoss.item<float>();
            float curCriticLoss = criticLoss.item<float>();
            float alphaVal = agent->GetLogAlpha().exp().item<float>();

            std::cout << "[MASAC Train] Step=" << m_stepCount
                      << " | Agent=" << i
                      << " | CriticLoss=" << curCriticLoss
                      << " | ActorLoss=" << curActorLoss
                      << " | Alpha=" << alphaVal
                      << std::endl;
        }
    }
}

// =============================================================================
// 모델 저장/로드
// =============================================================================
void
xAppHandoverSON_MASAC::SaveModels(const std::string& dir)
{
    NS_LOG_FUNCTION(this);

    if (!m_inferenceOnly)
    {
        std::string cmd = "mkdir -p " + dir;
        system(cmd.c_str());

        for (size_t i = 0; i < m_agents.size(); i++)
        {
            std::string prefix = dir + "/agent_" + std::to_string(i);

            torch::save(m_agents[i]->GetActor(), prefix + "_actor.pt");
            torch::save(m_agents[i]->GetCritic(), prefix + "_critic.pt");
            torch::save(m_agents[i]->GetTargetCritic(), prefix + "_target_critic.pt");
            torch::save(std::vector<torch::Tensor>{m_agents[i]->GetLogAlpha()},
                        prefix + "_log_alpha.pt");
        }

        std::string bufferPath = dir + "/replay_buffer.pt";
        m_replayBuffer->Save(bufferPath);

        NS_LOG_UNCOND("[MASAC] Models & Buffer saved to " << dir
            << "/ (step=" << m_stepCount << ", BufferSize=" << m_replayBuffer->Size() << ")");

        std::ofstream metaFile(dir + "/meta.txt");
        metaFile << m_stepCount << std::endl;
        metaFile.close();
    }
}

void
xAppHandoverSON_MASAC::LoadModels(const std::string& dir)
{
    NS_LOG_FUNCTION(this);
    std::cout << "[MASAC] Loading models..." << std::endl;

    for (size_t i = 0; i < m_agents.size(); i++)
    {
        std::string prefix = dir + "/agent_" + std::to_string(i);

        try
        {
            torch::load(m_agents[i]->GetActor(), prefix + "_actor.pt");
            torch::load(m_agents[i]->GetCritic(), prefix + "_critic.pt");
            torch::load(m_agents[i]->GetTargetCritic(), prefix + "_target_critic.pt");

            try {
                std::vector<torch::Tensor> alpha_data;
                torch::load(alpha_data, prefix + "_log_alpha.pt");
                if (!alpha_data.empty())
                    m_agents[i]->GetLogAlpha().data().copy_(alpha_data[0]);
            } catch (...) {}
        }
        catch (const std::exception& e)
        {
            NS_LOG_UNCOND("[MASAC] Failed to load agent " << i
                << " from " << dir << ": " << e.what());
            return;
        }
    }

    std::string bufferPath = dir + "/replay_buffer.pt";
    try {
        m_replayBuffer->Load(bufferPath, OBS_DIM, 0, NUM_AGENTS);
        NS_LOG_UNCOND("[MASAC] Buffer loaded. Size: " << m_replayBuffer->Size());
    } catch (const std::exception& e) {
        NS_LOG_WARN("[MASAC] Failed to load buffer: " << e.what());
    }

    NS_LOG_UNCOND("[MASAC] Models loaded from " << dir << "/");
    std::ifstream metaFile(dir + "/meta.txt");
    if (metaFile.is_open())
    {
        metaFile >> m_stepCount;
        metaFile.close();
        NS_LOG_UNCOND("[MASAC] Restored stepCount=" << m_stepCount);
    }
}

// =============================================================================
// CSV 로깅
// =============================================================================
void
xAppHandoverSON_MASAC::InitCsvLoggers()
{
    if (m_loadPretrained | m_inferenceOnly)
    {
        m_cellMetricsCsv.open("cell_metrics.csv", std::ios::app);
        m_cioActionsCsv.open("cio_actions.csv", std::ios::app);
        m_maddpgActionsCsv.open("maddpg_actions.csv", std::ios::app);
        m_rewardCurveCsv.open("reward_curve.csv", std::ios::app);
        m_stagnationCsv.open("stagnation_check.csv", std::ios::app);
    }
    else
    {
        m_cellMetricsCsv.open("cell_metrics.csv");
        m_cellMetricsCsv << "time_s,cellId,ueCount,edgeUeCount,"
            << "cellDlThp_kbps,cellUlThp_kbps,"
            << "avgCqi,txPower_dBm,prbUtilDl" << std::endl;

        m_cioActionsCsv.open("cio_actions.csv");
        m_cioActionsCsv << "time_s,srcCellId,neighborCellId,cioDB,cioIE" << std::endl;

        m_maddpgActionsCsv.open("maddpg_actions.csv");
        m_maddpgActionsCsv << "time_s,cellId,cioRaw,txpRaw,selfCioDB,txpApplied_dBm,cellThp_kbps,alpha" << std::endl;

        m_rewardCurveCsv.open("reward_curve.csv");
        m_rewardCurveCsv << "time_s,step,reward_cell1,reward_cell2,reward_cell3,prb_std" << std::endl;

        m_stagnationCsv.open("stagnation_check.csv");
        m_stagnationCsv << "time_s,step,cellId,"
            << "cio_l2_change,max_cio_change,"
            << "txp_raw_curr,txp_raw_prev,txp_raw_change,"
            << "txp_dBm_curr,txp_dBm_prev,"
            << "ue_count_prev,ue_count_curr,ue_count_change,"
            << "reward_prev,reward_curr,reward_change,"
            << "consecutive_stagnant_steps,"
            << "cio_stagnant,txp_stagnant,is_stagnant" << std::endl;
    }
}

void
xAppHandoverSON_MASAC::LogCellMetrics()
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

        m_cellMetricsCsv
            << now << "," << cellId << ","
            << cell.ueCount << "," << cell.edgeUeCount << ","
            << cell.totalThroughputDl << "," << cell.totalThroughputUl << ","
            << avgCqi << "," << cell.txPower << ","
            << cell.prbUtilDl << "\n";
    }
}

// =============================================================================
// 고착화 체크
// =============================================================================
void
xAppHandoverSON_MASAC::CheckAndLogStagnation(
    const std::vector<torch::Tensor>& currentActs,
    const std::vector<double>& rewards)
{
    if (!m_stagnationCsv.is_open()) return;

    double now = Simulator::Now().GetSeconds();
    bool allStagnant = true;

    for (size_t i = 0; i < m_agents.size(); i++)
    {
        uint16_t cellId = m_agents[i]->GetConfig().cellId;
        auto actData = currentActs[i].accessor<float, 1>();
        int64_t actDim = currentActs[i].size(0);
        const auto& neighbors = m_agents[i]->GetConfig().neighborCellIds;
        int64_t cioEndIdx = static_cast<int64_t>(neighbors.size());

        std::vector<float> currAct(actDim);
        for (int64_t d = 0; d < actDim; d++)
            currAct[d] = actData[d];

        float txpRaw = currAct[cioEndIdx];
        double txpApplied = m_txPower + static_cast<double>(txpRaw) * 4.0;
        txpApplied = std::max(26.0, std::min(38.0, txpApplied));

        double cioL2Change = -1.0;
        double maxCioChange = -1.0;
        double txpRawChange = -1.0;
        float prevTxpRaw = txpRaw;

        auto prevIt = m_prevRawActions.find(cellId);
        if (prevIt != m_prevRawActions.end() && (int64_t)prevIt->second.size() == actDim)
        {
            cioL2Change = 0.0;
            maxCioChange = 0.0;
            for (int64_t d = 0; d < cioEndIdx; d++)
            {
                double diff = std::abs(currAct[d] - prevIt->second[d]);
                cioL2Change += diff * diff;
                maxCioChange = std::max(maxCioChange, diff);
            }
            cioL2Change = std::sqrt(cioL2Change);
            prevTxpRaw = prevIt->second[cioEndIdx];
            txpRawChange = std::abs(txpRaw - prevTxpRaw);
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

        bool cioStagnant = false;
        bool txpStagnant = false;
        if (cioL2Change >= 0.0)
        {
            cioStagnant = (cioL2Change < STAGNATION_ACTION_THRESH);
            txpStagnant = (txpRawChange < STAGNATION_ACTION_THRESH);
        }
        bool isStagnant = cioStagnant && txpStagnant
                       && (rewardChange < STAGNATION_REWARD_THRESH);
        if (!isStagnant) allStagnant = false;

        m_stagnationCsv << now << "," << m_stepCount << "," << cellId << ","
            << cioL2Change << "," << maxCioChange << ","
            << txpRaw << "," << prevTxpRaw << "," << txpRawChange << ","
            << txpApplied << "," << prevTxpApplied << ","
            << ueCountPrev << "," << ueCountCurr << "," << ueCountChange << ","
            << rewardPrev << "," << rewardCurr << "," << rewardChange << ","
            << m_stagnantSteps << ","
            << (cioStagnant ? 1 : 0) << ","
            << (txpStagnant ? 1 : 0) << ","
            << (isStagnant ? 1 : 0) << "\n";

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
        NS_LOG_UNCOND("[STAGNATION-MASAC] WARNING: " << m_stagnantSteps
            << " consecutive stagnant steps at t=" << now << "s");
    }
}
