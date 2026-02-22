#include "xAppHandoverSON.h"

#include "ns3/E2AP.h"
#include "ns3/core-module.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <numeric>   // std::iota
#include <random>

using namespace ns3;
using namespace oran;

NS_LOG_COMPONENT_DEFINE("xAppHandoverSON");

xAppHandoverSON::xAppHandoverSON(float sonPeriodicitySec, bool initiateHandovers) ////여기서 각 파라미터 조정, m_loadThreshold 등의 파라미터를 RL을 통해 나온 값으로 대체하면 됨
    : xAppHandover(),
      m_sonPeriodicitySec(sonPeriodicitySec),
      m_initiateHandovers(initiateHandovers),
      //m_cellRadius(289.0),
      //m_edgeThreshold(0.7),
      m_edgeRsrpThreshold(-85.28), // ★ 237.74m 물리적 거리에 상응하는 RSRP 임계값
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
                    
    // ── MADDPG 초기화 ──
    if (m_useMADDPG)
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

    // 1. KPM 수집 (기존 로직 그대로)
    CollectKPMs();
    if (m_stepCount % 10 == 0)
    {
        LogCellMetrics();  // ★ 추가
    }
    NS_LOG_LOGIC("cell 1: " << m_cellContexts[1].ueCount
        << " cell 2: " << m_cellContexts[2].ueCount
        << " cell 3: " << m_cellContexts[3].ueCount);
    NS_LOG_LOGIC("[SON] === Periodic Check === UEs=" << m_ueContexts.size()
        << " Cells=" << m_cellContexts.size());

    std::cout << "\n[TICK] PeriodicSONCheck 불림 - Step: " << m_stepCount 
              << " | UE 수: " << m_ueContexts.size() << std::endl;
    for (auto& [key, ue] : m_ueContexts)
    {
        // 변경 후
        NS_LOG_UNCOND("[SON]   UE RNTI=" << ue.rnti
            << " Cell=" << ue.servingCellId
            << " RSRP=" << ue.servingRsrp
            << " RSRQ=" << ue.servingRsrq
            << " CQI=" << ue.cqi
            << " isEdge=" << ue.isEdge);

        std::cout << "[SON]   UE RNTI=" << ue.rnti
            << " Cell=" << ue.servingCellId
            << " RSRP=" << ue.servingRsrp
            << " RSRQ=" << ue.servingRsrq
            << " CQI=" << ue.cqi
            << " isEdge=" << ue.isEdge << std::endl;
    }

    if (m_useMADDPG)
    {
        // ── MADDPG 경로 ──
        // 3. 관측 구성 → Actor 행동 선택 → CIO 적용
        StepMADDPG();

        // 4. 배치 학습
        if (!m_inferenceOnly)
        {
            //if (m_stepCount % 4 == 0)
            TrainMADDPG();
        }

        m_stepCount++;
        
        // 100스텝마다 모델 저장 (체크포인트)
        if (m_stepCount % 100 == 0)
        {
            SaveModels();
        }
    }
    else
    {
        // ── 기존 규칙 기반 경로 ──
        

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

                    NS_LOG_UNCOND("[SON] Handover initiated RNTI=" << ue.rnti
                        << " from Cell=" << ue.servingCellId
                        << " to Cell=" << targetCell);
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
    //CollectThroughput();

    CalculateEdgeUEs();
    CollectUeCount();
    CollectCellThroughput();  // ★ 추가: 셀 단위 DL+UL throughput 집계

    PurgeStaleUeContexts();
}

void
xAppHandoverSON::CollectRsrpRsrq()
{
    NS_LOG_FUNCTION(this);
    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");

    // RSRP
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

    // RSRQ
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
            if (m_ueContexts.find(key) != m_ueContexts.end() &&
                m_ueContexts[key].cqi == 0)
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

    // UE Count
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
    // TxPower
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

    // ── DL ──
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

        if (prev < 0) continue;  // 첫 읽기 — baseline 설정만

        double delta = cumBytes - prev;
        if (delta < 0) delta = 0;

        auto it = m_cellContexts.find(cellId);
        if (it != m_cellContexts.end())
            it->second.totalThroughputDl += delta;
    }

    // ── UL ──
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

    // bytes → kbps (SON 주기 기반)
    for (auto& [cellId, cell] : m_cellContexts)
    {
        double dlDelta = cell.totalThroughputDl;  // 아직 bytes 상태
        double ulDelta = cell.totalThroughputUl;
        if (dlDelta > 0 || ulDelta > 0)
        {
            cell.totalThroughputDl = (cell.totalThroughputDl * 8.0) / 1000.0 / m_sonPeriodicitySec;
            cell.totalThroughputUl = (cell.totalThroughputUl * 8.0) / 1000.0 / m_sonPeriodicitySec;

            std::cout << "[IpVol] Cell" << cellId
                << " DL=" << (cell.totalThroughputDl / 1e3) << "Mbps"
                << " UL=" << (cell.totalThroughputUl / 1e3) << "Mbps"
                << " UEs=" << cell.ueCount << std::endl;
            NS_LOG_LOGIC("[KPM-Vol] Cell" << cellId
                << " DL=" << (cell.totalThroughputDl / 1e3) << "Mbps"
                << " UL=" << (cell.totalThroughputUl / 1e3) << "Mbps");

            // 캐시 저장
            m_lastThroughputDl[cellId] = cell.totalThroughputDl;
            m_lastThroughputUl[cellId] = cell.totalThroughputUl;

            double dlBytes = cell.totalThroughputDl * m_sonPeriodicitySec * 1000.0 / 8.0;
            double ulBytes = cell.totalThroughputUl * m_sonPeriodicitySec * 1000.0 / 8.0;

            std::cout << "[IpVol] Cell" << cellId
                << " DL=" << (cell.totalThroughputDl / 1e3) << "Mbps"
                << " (" << (dlBytes / 1e3) << "KB)"
                << " UL=" << (cell.totalThroughputUl / 1e3) << "Mbps"
                << " UEs=" << cell.ueCount
                << " edge=" << cell.edgeUeCount << std::endl;
        }
        else
        {
            // delta=0이면 이전 값 유지
            cell.totalThroughputDl = m_lastThroughputDl[cellId];
            cell.totalThroughputUl = m_lastThroughputUl[cellId];

            std::cout << "[IpVol] Cell" << cellId
                << " DL=" << (cell.totalThroughputDl / 1e3) << "Mbps"
                << " UL=" << (cell.totalThroughputUl / 1e3) << "Mbps"
                << " UEs=" << cell.ueCount << std::endl;
        }
        

        double offeredMbps = cell.ueCount * 8.192;
        double actualMbps = cell.totalThroughputDl / 1e3;
        std::cout << "[SAT] Cell" << cellId
            << " UEs=" << cell.ueCount
            << " offered=" << offeredMbps << "Mbps"
            << " actual=" << actualMbps << "Mbps"
            << " ratio=" << (offeredMbps > 0 ? actualMbps / offeredMbps : 0)
            << std::endl;
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
        // 수집된 RSRP가 임계값(-118 dBm)보다 작으면 Edge UE
        ue.isEdge = (ue.servingRsrp < m_edgeRsrpThreshold);

        // 디버깅을 위해 로그
        NS_LOG_DEBUG("[SON-EDGE] RNTI=" << ue.rnti 
            << " Cell=" << ue.servingCellId 
            << " RSRP=" << ue.servingRsrp 
            << " dBm | isEdge=" << (ue.isEdge ? "True" : "False"));
    }
}
/*
double
xAppHandoverSON::FriisDistanceEstimate(double rsrp_dBm, double txPower_dBm, double freq_Hz, uint16_t rnti, uint16_t cellId)
{
    Json cellInfo = E2AP::QueryCellInfo(cellId);
    double nRE;
    if (!cellInfo.empty())
    {
        int prb = cellInfo["DL_BANDWIDTH_PRB"];
        // TxPower는 전체 대역 전력 → RE 당 전력으로 보정
        nRE = prb * 12.0;
    }
    else
    {
        nRE = m_dlBandwidthPrb * 12.0;  // fallback
    }
    double txPowerPerRE_dBm = txPower_dBm - 10.0 * std::log10(nRE);

    double pathLoss = txPowerPerRE_dBm - rsrp_dBm;
    double systemLoss = 1.0; // 시스템 손실이 없다고 가정
    double c = 3.0e8;
    double lambda = c / freq_Hz;
    double d = lambda / std::sqrt(systemLoss) * std::pow(10, pathLoss / 20.0) / (4.0 * M_PI);
    NS_LOG_LOGIC("[SON-DBG] RNTI=" << rnti << " RSRP=" << rsrp_dBm
        << " d=" << d << " m");
    return d;
}*/

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
                       + cell.edgeUeCount * 3.0
                       - cell.avgCqi * 0.0;
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

    // ===== 디버그 추가 =====
    bool overloaded = IsCellOverloaded(ue.servingCellId);
    auto cellIt = m_cellContexts.find(ue.servingCellId);
    double loadScore = (cellIt != m_cellContexts.end()) ? cellIt->second.loadScore : -1;
    int ueCount = (cellIt != m_cellContexts.end()) ? cellIt->second.ueCount : -1;

    NS_LOG_UNCOND("[SON-DBG] RNTI=" << ue.rnti
        << " Cell=" << ue.servingCellId
        << " CQI=" << ue.cqi
        << " RSRQ=" << ue.servingRsrq
        << " isEdge=" << ue.isEdge
        << " overloaded=" << overloaded
        << " loadScore=" << loadScore
        << " ueCount=" << ueCount
        << " loadThreshold=" << m_loadThreshold
        << " cqiThreshold=" << m_cqiThreshold
        << " rsrqThreshold=" << m_rsrqThreshold);
    // ===== 디버그 끝 =====

    // 조건 1: 셀 과부하 + Edge UE → MLB
    if (IsCellOverloaded(ue.servingCellId) && ue.isEdge)
    {
        NS_LOG_INFO("SON: MLB triggered for RNTI " << ue.rnti);
        return FindLeastLoadedNeighbor(key);
    }

    // 조건 2: RSRQ 불량 ->> 뭔가 여기서 핑퐁 발생할것 같은 느낌
    if (ue.servingRsrq < m_rsrqThreshold)
    {
        NS_LOG_INFO("SON: RSRQ handover triggered for RNTI " << ue.rnti);
        return FindBestRsrqCell(key);
    }

    // 조건 3: CQI 불량 -> 여기도 핑퐁 위험
    if (ue.cqi < m_cqiThreshold && ue.cqi > 0)
    {
        NS_LOG_INFO("SON: CQI handover triggered for RNTI " << ue.rnti);
        return FindLeastLoadedNeighbor(key);
    }

    return std::numeric_limits<uint16_t>::max();
}

uint16_t
xAppHandoverSON::FindLeastLoadedNeighbor(UeKey key)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Finding least loaded neighbor for UE key: " << key);
    auto ueIt = m_ueContexts.find(key);
    if (ueIt == m_ueContexts.end())
    {
        NS_LOG_INFO("UE context not found.");
        return std::numeric_limits<uint16_t>::max();
    }

    uint16_t servingCell = ueIt->second.servingCellId;
    uint16_t bestCell = std::numeric_limits<uint16_t>::max();
    double minLoad = std::numeric_limits<double>::max();

    for (auto& [cellId, cell] : m_cellContexts)
    {
        NS_LOG_INFO("Checking Cell " << cellId << "servingCell: " << servingCell << " loadScore: " << cell.loadScore);
        if (cellId != servingCell && cell.loadScore < minLoad)
        {
            NS_LOG_INFO("Checking neighbor Cell " << cellId << " with load " << cell.loadScore);
            minLoad = cell.loadScore;
            bestCell = cellId;
            NS_LOG_INFO("New best cell found: Cell " << bestCell << " with load " << minLoad);
        }
    }
    NS_LOG_INFO("Least loaded neighbor for UE key " << key << " is Cell " << bestCell << " with load " << minLoad);
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

    // 이웃 셀 RSRQ 데이터가 없으면 fallback
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

    // 타겟 RSRQ가 현재 서빙 셀보다 나아야 핸드오버 의미가 있음
    if (bestRsrq <= ue.servingRsrq)
        return std::numeric_limits<uint16_t>::max();

    return bestCell;
}
// =============================================================================
// CIO sending
// =============================================================================
void
xAppHandoverSON::ApplyCioActions(const std::vector<double>& cioActions,
                          const std::vector<uint16_t>& cellIds,
                          const std::vector<std::string>& enbEndpoints)
{
    std::map<uint16_t, int8_t> cellCioMap;
    for (size_t i = 0; i < cellIds.size(); i++)
    {
        int8_t cio = static_cast<int8_t>(std::round(cioActions[i] * 24.0));
        cio = std::max((int8_t)-24, std::min((int8_t)24, cio));
        cellCioMap[cellIds[i]] = cio;
    }

    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");
    if (!ric) return;

    for (size_t i = 0; i < cellIds.size(); i++)
    {
        Json cioList = Json::array();
        for (auto& [cellId, cioValue] : cellCioMap)
        {
            if (cellId == cellIds[i]) continue;
            Json entry;
            entry["CELL_ID"] = cellId;
            entry["CIO_VALUE"] = cioValue;
            cioList.push_back(entry);
        }

        std::string srcEndpoint = enbEndpoints[i];
        ric->E2SmRcSendCioControlRequest(cioList, srcEndpoint);

        NS_LOG_INFO("[xApp-CIO] Sent CIO_LIST to " << srcEndpoint
                    << " entries=" << cioList.size());
    }
}

void
xAppHandoverSON::PurgeStaleUeContexts()
{
    // 1차: staleKeys에 정확히 매칭되는 것 제거
    for (auto staleIt = m_staleKeys.begin(); staleIt != m_staleKeys.end(); )
    {
        auto ueIt = m_ueContexts.find(*staleIt);
        if (ueIt != m_ueContexts.end())
        {
            NS_LOG_UNCOND("[SON-PURGE] Matched stale Cell=" 
                << ueIt->second.servingCellId
                << " RNTI=" << ueIt->second.rnti);
            m_ueContexts.erase(ueIt);
            staleIt = m_staleKeys.erase(staleIt);  // 매칭 성공 → 목록에서도 제거
        }
        else
        {
            staleIt = m_staleKeys.erase(staleIt);  // KPM에 없음 → 이미 정리됨
        }
    }
    /*
    // 2차: 그래도 남아있으면 ueCount 기반 fallback
    for (auto& [cellId, cellCtx] : m_cellContexts)
    {
        std::vector<UeKey> cellUeKeys;
        for (auto& [key, ue] : m_ueContexts)
        {
            if (ue.servingCellId == cellId)
                cellUeKeys.push_back(key);
        }

        uint32_t kpmCount = cellUeKeys.size();
        if (kpmCount <= cellCtx.ueCount)
            continue;

        std::sort(cellUeKeys.begin(), cellUeKeys.end(),
            [this](const UeKey& a, const UeKey& b) {
                return m_ueContexts[a].servingRsrp < m_ueContexts[b].servingRsrp;
            });

        uint32_t toRemove = kpmCount - cellCtx.ueCount;
        for (uint32_t i = 0; i < toRemove; i++)
        {
            NS_LOG_UNCOND("[SON-PURGE] Fallback Cell=" << cellId
                << " RNTI=" << m_ueContexts[cellUeKeys[i]].rnti
                << " RSRP=" << m_ueContexts[cellUeKeys[i]].servingRsrp);
            m_ueContexts.erase(cellUeKeys[i]);
        }
    }
    */
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

    if (m_initiateHandovers)  // ← 가드 추가
    {
        uint16_t sonDecision = MakeSONDecision(key);
        if (sonDecision != std::numeric_limits<uint16_t>::max())
            decidedTargetCellId = sonDecision;
    }

    // 이미 핸드오버 중이면 거부
    if (m_imsiInHandover.find(key) != m_imsiInHandover.end())
        decidedTargetCellId = std::numeric_limits<uint16_t>::max();

    // 핸드오버 마킹
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
            m_staleKeys.insert(it->first);  // ★ (srcCellId, srcRnti) 기록
            NS_LOG_UNCOND("[SON-STALE] Marked stale key Cell=" 
                << (it->first >> 16) << " RNTI=" << (it->first & 0xFFFF));
            m_imsiInHandover.erase(it);
            break;
        }
    }
}

void
xAppHandoverSON::HandoverFailed(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    NS_LOG_FUNCTION(this);
    for (auto it = m_imsiInHandover.begin(); it != m_imsiInHandover.end(); ++it)
    {
        if (it->second == imsi)
        {
            m_imsiInHandover.erase(it);
            break;
        }
    }
}

void
xAppHandoverSON::ConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    for (auto it = m_imsiInHandover.begin(); it != m_imsiInHandover.end(); ++it)
    {
        if (it->second == imsi)
        {
            m_imsiInHandover.erase(it);
            break;
        }
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

    // 논문: Critic = 전체 에이전트 state + action 결합
    // totalObsDim = 3 × 4 = 12, totalActDim = 3 × 2 = 6
    int64_t totalObsDim = NUM_AGENTS * OBS_DIM;
    int64_t totalActDim = NUM_AGENTS * ACT_DIM;

    for (auto cellId : m_cellIds)
    {
        AgentConfig config;
        config.cellId = cellId;
        config.obsDim = OBS_DIM;
        config.actDim = ACT_DIM;
        config.neighborCellIds = m_neighborMap[cellId];

        m_agents.push_back(std::make_unique<MADDPGAgent>(
            config, totalObsDim, totalActDim, LR, MAX_ACTION));
    }

    m_replayBuffer = std::make_unique<ReplayBuffer>(BUFFER_SIZE);

    NS_LOG_UNCOND("[MADDPG] Initialized: " << NUM_AGENTS << " agents, "
        << "Obs=" << OBS_DIM << " Act=" << ACT_DIM
        << " CriticInput=" << (totalObsDim + totalActDim)
        << " MaxAction=" << MAX_ACTION);

    if (m_loadPretrained)
    {
        LoadModels();
    }
}

// =============================================================================
// MADDPG 관측 벡터 구성
// =============================================================================
// 왜 필요한가: Actor 네트워크의 입력. CellContext(항상 정확한 ueCount)와
// per-UE 데이터(일부 핸드오버 직후 누락 가능)를 조합하여 8차원 벡터 생성.
//
// 벡터 구성:
//   [0] ueCount / totalUEs           — 이 셀의 부하 비율
//   [1] totalDlThp / MAX_THR         — DL throughput 정규화
//   [2] avgCqi / 15.0                — 평균 채널 품질
//   [3] edgeRatio                    — edge UE 비율 (핸드오버 후보 비율)
//   [4] neighbor1_ueCount / totalUEs — 이웃1 부하
//   [5] neighbor1_avgCqi / 15.0      — 이웃1 채널 품질
//   [6] neighbor2_ueCount / totalUEs — 이웃2 부하
//   [7] neighbor2_avgCqi / 15.0      — 이웃2 채널 품질
//
// 숫자 예시 (eNB2, 40 UE 중 20개 연결, DL 8Mbps, avgCQI 12, edge 30%):
//   [20/40, 8e6/20e6, 12/15, 0.3, 10/40, 13/15, 10/40, 14/15]
//   = [0.5, 0.4, 0.8, 0.3, 0.25, 0.87, 0.25, 0.93]

// 논문 Eq.(2): s_i = [C_i, T_i, N_i, E_i]
// 논문 코드: eNB_state = [AvgCqi, Throughput, FarUes, ServedUes]
// 정규화 없이 raw 값 사용 (논문 코드와 동일)
torch::Tensor
xAppHandoverSON::BuildObservation(uint16_t cellId)
{
    std::vector<float> obs(OBS_DIM, 0.0f);

    auto cellIt = m_cellContexts.find(cellId);
    if (cellIt == m_cellContexts.end())
    {
        NS_LOG_WARN("[MADDPG] CellContext missing for cell " << cellId);
        return torch::zeros({OBS_DIM});
    }

    const auto& cell = cellIt->second;

    // [0] AvgCqi — C_i
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
    obs[0] = avgCqi;

    // [1] Throughput — T_i (Mbps)
    obs[1] = static_cast<float>(cell.totalThroughputDl / 1e3);

    // [2] FarUes — E_i (edge UE 비율)
    obs[2] = (ueWithData > 0)
        ? static_cast<float>(edgeCount) / ueWithData : 0.0f;

    // [3] ServedUes — N_i (UE 수)
    obs[3] = static_cast<float>(cell.ueCount);

    return torch::from_blob(obs.data(), {OBS_DIM}, torch::kFloat32).clone();
}

// =============================================================================
// MADDPG 보상 함수
// =============================================================================
// 왜 필요한가: CIO 조절 행동의 품질을 평가하는 신호.
// cooperative 세팅 → 모든 에이전트가 동일한 보상.
//
// r = α × min_i(thp_i) + β × avg_i(thp_i)
//
//   min(thp): 가장 약한 셀의 성능을 끌어올림 → 공평성(fairness)
//   avg(thp): 전체 네트워크 효율 유지 → 효율성(efficiency)
//
// 숫자 예시 (정규화 전, Mbps):
//   [Cell1: 8, Cell2: 3, Cell3: 7] → min=3, avg=6
//   r = 0.5×(3/20) + 0.5×(6/20) = 0.075 + 0.15 = 0.225
//
//   학습 후 균등 분배:
//   [Cell1: 6, Cell2: 6, Cell3: 6] → min=6, avg=6
//   r = 0.5×(6/20) + 0.5×(6/20) = 0.15 + 0.15 = 0.3  (개선됨)

// 논문 Eq.(7): r_i = Σ R_k (per-agent 셀 throughput)
// 논문 코드: R_rewards = Throughput[2:5], 각 에이전트에 자기 셀 throughput 할당
std::vector<double>
xAppHandoverSON::ComputeRewards()
{
    std::vector<double> rewards;
    for (auto cellId : m_cellIds)
    {
        auto it = m_cellContexts.find(cellId);
        if (it != m_cellContexts.end())
        {
            // DL + UL 합산 (Mbps)
            double totalThp = (it->second.totalThroughputDl
                             + it->second.totalThroughputUl) / 1e3;
            rewards.push_back(totalThp);
        }
        else
        {
            rewards.push_back(0.0);
        }
    }
    return rewards;
}

// =============================================================================
// MADDPG 스텝: 관측 → 행동 → CIO 적용
// =============================================================================
// 매 SON 주기마다:
//   1) 이전 스텝의 전이(transition)를 replay buffer에 저장
//   2) 현재 관측으로 Actor가 행동 선택
//   3) 행동을 CIO dB로 변환 → E2SM-RC로 전송
//
// 행동 → CIO 변환:
//   cio_dB = action × MAX_CIO_DB  (action ∈ [-1,1], MAX_CIO_DB = 6.0)
//   cio_IE = round(cio_dB / 0.5)  (3GPP TS 36.331 §6.3.6: q-OffsetCell, 0.5dB 단위)
//   cio_IE ∈ [-12, +12]
//
// 숫자 예시: Actor 출력 0.5 → 0.5×6.0 = 3.0dB → IE = round(3.0/0.5) = 6

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
    if (m_hasPrevStep)
    {
        auto rewards = ComputeRewards();
        Experience exp;
        exp.obs = m_prevObs;
        exp.acts = m_prevActs;
        exp.rewards = rewards;
        exp.nextObs = currentObs;
        exp.done = false;
        m_replayBuffer->Push(std::move(exp));

        if (m_stepCount % 10 == 0)
        {
            NS_LOG_UNCOND("[MADDPG] Step=" << m_stepCount
                << " Rewards=[" << rewards[0] << ", "
                << rewards[1] << ", " << rewards[2] << "]"
                << " BufferSize=" << m_replayBuffer->Size()
                << " Epsilon=" << m_epsilon);

                       
            // NS_LOG_UNCOND 대신 std::cout 사용!
            std::cout << "🚀 [MADDPG] Step=" << m_stepCount
                    << " | Reward=" << rewards[0] << ", " << rewards[1] << ", " << rewards[2]
                    << " | BufferSize=" << m_replayBuffer->Size() << std::endl;
            
        }
    }

    // ── 행동 선택 ──
    std::vector<torch::Tensor> currentActs;

    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");
    if (!ric) return;

    // 논문 코드: epsilon-greedy + 가우시안 노이즈
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
    static std::uniform_real_distribution<double> randomAction(-MAX_ACTION, MAX_ACTION);
    std::normal_distribution<double> noiseDist(0.0, MAX_ACTION * EXPL_NOISE);

    for (size_t i = 0; i < m_agents.size(); i++)
    {
        torch::Tensor act;
        uint16_t srcCell = m_agents[i]->GetConfig().cellId;

        if (!m_inferenceOnly && uniformDist(rng) <= m_epsilon)
        {
            // ε-greedy: 랜덤 행동
            std::vector<float> randAct(ACT_DIM);
            for (int d = 0; d < ACT_DIM; d++)
                randAct[d] = static_cast<float>(randomAction(rng));
            act = torch::from_blob(randAct.data(), {ACT_DIM}, torch::kFloat32).clone();
        }
        else if (!m_inferenceOnly)
        {
            // Actor + 가우시안 노이즈
            act = m_agents[i]->SelectAction(currentObs[i]);
            std::vector<float> noise(ACT_DIM);
            for (int d = 0; d < ACT_DIM; d++)
                noise[d] = static_cast<float>(noiseDist(rng));
            auto noiseTensor = torch::from_blob(noise.data(), {ACT_DIM}, torch::kFloat32).clone();
            act = torch::clamp(act + noiseTensor,
                              -static_cast<float>(MAX_ACTION),
                               static_cast<float>(MAX_ACTION));
        }
        else
        {
            // Inference: 노이즈 없이 Actor만
            act = m_agents[i]->SelectAction(currentObs[i]);
        }

        currentActs.push_back(act);

        auto actData = act.accessor<float, 1>();
        const auto& neighbors = m_agents[i]->GetConfig().neighborCellIds;

        // ── [0] CIO 변환 ──
        // 논문 코드: env_action1 = round(action[0] * 0.5, 0) → 정수 dB
        // 3GPP TS 36.331 §6.3.6: q-OffsetCell
        // 글로벌 CIO: 모든 이웃에 동일 값 적용
        //
        // 숫자 예시: action[0]=4.2 → 4.2×0.5=2.1 → round=2 dB → IE=4
        //           action[0]=-6.0 → -6.0×0.5=-3.0 → -3 dB → IE=-6
        int cioDB = static_cast<int>(std::round(actData[0] * 0.5));
        cioDB = std::max(-6, std::min(6, cioDB));
        int cioIE = cioDB * 2;  // dB → IE (0.5dB 단위): dB / 0.5 = dB × 2

        Json cioList = Json::array();
        for (auto nId : neighbors)
        {
            Json entry;
            entry["CELL_ID"] = nId;
            entry["CIO_VALUE"] = cioIE;
            cioList.push_back(entry);
        }
        std::string endpoint = "/E2Node/" + std::to_string(srcCell) + "/";
        ric->E2SmRcSendCioControlRequest(cioList, endpoint);

        // ── [1] TXP 변환 ──
        // 논문 코드: env_action2 = round(action[1], 4) → dBm offset
        // 기본 TXP에 offset 적용, clamp [30, 46]
        //
        // 숫자 예시: action[1]=3.5 → TXP=46+3.5=49.5 → clamp→46.0
        //           action[1]=-5.0 → TXP=46-5.0=41.0
        double rawTxpOffset = static_cast<double>(actData[1]);
        
        // ★ 논문과 100% 동일하게 소수점 4자리 반올림 (Python np.round(val, 4) 완벽 모사)
        double paperTxpOffset = std::round(rawTxpOffset * 10000.0) / 10000.0;
        
        double txpApplied = m_txPower + paperTxpOffset;
        txpApplied = std::max(20.0, std::min(46.0, txpApplied)); // 논문의 한계선 적용

        ric->E2SmRcSendTxPowerControlRequest(txpApplied, endpoint);

        // 기존 로그 아래에 추가
        NS_LOG_UNCOND("[MADDPG] Cell" << srcCell
            << " CIO=" << cioDB << "dB(IE=" << cioIE << ")"
            << " TXP=" << txpApplied << "dBm(offset=" << paperTxpOffset<< ")");
        if (m_stepCount % 10 == 0)
        {
            // ★ CSV 로깅
            double now = Simulator::Now().GetSeconds();

            // CIO 로그 — 각 이웃별로 기록
            for (auto nId : neighbors)
            {
                m_cioActionsCsv << now << ","
                    << srcCell << "," << nId << ","
                    << cioDB << "," << cioIE << std::endl;
            }

            // MADDPG 행동 로그
            double cellThp = 0.0;
            auto cellIt = m_cellContexts.find(srcCell);
            if (cellIt != m_cellContexts.end())
                cellThp = cellIt->second.totalThroughputDl + cellIt->second.totalThroughputUl;

            m_maddpgActionsCsv << now << ","
                << srcCell << ","
                << actData[0] << "," << actData[1] << ","
                << cioDB << "," << txpApplied << ","
                << cellThp << "," << m_epsilon << std::endl;
        }
    }

    // ── epsilon 감소 ──
    if (m_epsilon > EPSILON_END)
    {
        m_epsilon *= EPSILON_DECAY;
    }

    // ── 다음 스텝을 위해 저장 ──
    m_prevObs = currentObs;
    m_prevActs = currentActs;
    m_hasPrevStep = true;
}

// =============================================================================
// MADDPG 학습
// =============================================================================
// 각 에이전트 i에 대해:
//
// [Critic 업데이트]
//   y = r + γ × Q'_i(x', a'_1, ..., a'_N)   (a'_j = μ'_j(o'_j), Target Actor)
//   L = MSE(Q_i(x, a_1, ..., a_N), y)
//
// [Actor 업데이트]  
//   J = -E[ Q_i(x, a_1, ..., μ_i(o_i), ..., a_N) ]
//   에이전트 i의 행동만 현재 Actor 출력으로 교체,
//   나머지는 배치에서 가져온 기존 행동(detach) 사용.
//
// [Target Soft Update]
//   θ' ← τθ + (1-τ)θ'    (τ = 0.01)
//
// 출처: Lowe et al., "Multi-Agent Actor-Critic for Mixed
//       Cooperative-Competitive Environments", NeurIPS 2017

void
xAppHandoverSON::TrainMADDPG()
{
    NS_LOG_FUNCTION(this);

    if (m_replayBuffer->Size() < BATCH_SIZE)
    {
        NS_LOG_UNCOND("[MADDPG] Warmup: " << m_replayBuffer->Size()
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
        }
        obsBatch[a] = torch::stack(oVec);       // (B, 4)
        actsBatch[a] = torch::stack(aVec);       // (B, 2)
        nextObsBatch[a] = torch::stack(noVec);   // (B, 4)
    }

    // 전체 결합 (dim=1 — 논문 코드 버그 수정)
    auto allObs     = torch::cat(obsBatch, 1);       // (B, 12)
    auto allActs    = torch::cat(actsBatch, 1);       // (B, 6)
    auto allNextObs = torch::cat(nextObsBatch, 1);    // (B, 12)

    // Target 행동 계산
    std::vector<torch::Tensor> targetNextActs;
    {
        torch::NoGradGuard noGrad;
        for (int a = 0; a < NUM_AGENTS; a++)
            targetNextActs.push_back(
                m_agents[a]->GetTargetActor()->forward(nextObsBatch[a]));
    }
    auto allTargetNextActs = torch::cat(targetNextActs, 1);  // (B, 6)

    // ── 에이전트별 업데이트 ──
    for (int i = 0; i < NUM_AGENTS; i++)
    {
        auto& agent = m_agents[i];

        // ─ Critic 업데이트 — 논문 Eq.(8)(9) ─
        torch::Tensor targetQ;
        {
            torch::NoGradGuard noGrad;
            targetQ = agent->GetTargetCritic()->forward(allNextObs, allTargetNextActs);
            targetQ = rewardBatch[i] + GAMMA * targetQ;
        }

        auto currentQ = agent->GetCritic()->forward(allObs, allActs);
        auto criticLoss = torch::mse_loss(currentQ, targetQ);

        agent->GetCriticOpt().zero_grad();
        criticLoss.backward();
        agent->GetCriticOpt().step();

        // ─ Actor 업데이트 — 논문 Eq.(10) ─
        std::vector<torch::Tensor> policyActs;
        for (int j = 0; j < NUM_AGENTS; j++)
        {
            if (j == i)
                policyActs.push_back(agent->GetActor()->forward(obsBatch[i]));
            else
                policyActs.push_back(actsBatch[j].detach());
        }
        auto allPolicyActs = torch::cat(policyActs, 1);

        auto actorLoss = -agent->GetCritic()->forward(
            allObs.detach(), allPolicyActs).mean();

        agent->GetActorOpt().zero_grad();
        actorLoss.backward();
        agent->GetActorOpt().step();

        // ─ Target Soft Update — 논문 Eq.(11)(12) ─
        agent->SoftUpdateTargets(TAU_SOFT);

        if (m_stepCount % 10 == 0)
        {
            NS_LOG_UNCOND("[MADDPG] Train Agent=" << i
                << " CriticLoss=" << criticLoss.item<float>()
                << " ActorLoss=" << actorLoss.item<float>());
            std::cout << "🔥 [MADDPG Train] Step=" << m_stepCount
                      << " | Agent=" << i
                      << " | CriticLoss=" << criticLoss.item<float>()
                      << " | ActorLoss=" << actorLoss.item<float>() << std::endl;
        }
    }
}

// =============================================================================
// 모델 저장/로드
// =============================================================================
// 왜 필요한가: MADDPG 학습은 warmup + 수백 스텝이 필요.
// 매 실행마다 처음부터 학습하면 비효율적.
// 학습된 Actor/Critic 가중치를 파일로 저장하면:
//   1) 시뮬레이션 중단 후 이어서 학습 가능
//   2) 학습 완료된 모델로 평가(inference)만 실행 가능
//
// 저장 구조:
//   maddpg_models/
//     agent_0_actor.pt
//     agent_0_critic.pt
//     agent_0_target_actor.pt
//     agent_0_target_critic.pt
//     agent_1_actor.pt
//     ...

void
xAppHandoverSON::SaveModels(const std::string& dir)
{
    NS_LOG_FUNCTION(this);

    // 디렉토리 생성
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
    // ★ 버퍼 저장 로직 추가
    std::string bufferPath = dir + "/replay_buffer.pt";
    m_replayBuffer->Save(bufferPath);

    NS_LOG_UNCOND("[MADDPG] Models & Buffer saved to " << dir
        << "/ (step=" << m_stepCount << ", BufferSize=" << m_replayBuffer->Size() << ")");
    
    // 모델 저장 후
    std::ofstream metaFile(dir + "/meta.txt");
    metaFile << m_epsilon << std::endl;
    metaFile << m_stepCount << std::endl;
    metaFile.close();

}

void
xAppHandoverSON::LoadModels(const std::string& dir)
{
    NS_LOG_FUNCTION(this);
    std::cout << "loaded" << std::endl;

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
            NS_LOG_UNCOND("[MADDPG] Starting with fresh weights for agent " << i);
            return;
        }
    }

    // ★ 버퍼 불러오기 로직 추가
    std::string bufferPath = dir + "/replay_buffer.pt";
    try {
        m_replayBuffer->Load(bufferPath, OBS_DIM, ACT_DIM, NUM_AGENTS);
        NS_LOG_UNCOND("[MADDPG] Buffer loaded successfully. Size: " << m_replayBuffer->Size());
    } catch (const std::exception& e) {
        NS_LOG_WARN("[MADDPG] Failed to load buffer from " << bufferPath << ": " << e.what());
    }

    NS_LOG_UNCOND("[MADDPG] Models & Buffer loaded from " << dir << "/");
    std::ifstream metaFile(dir + "/meta.txt");
    if (metaFile.is_open())
    {
        metaFile >> m_epsilon;
        metaFile >> m_stepCount;
        metaFile >> m_episode;
        metaFile.close();
        m_episode++;  // 새 실행 = 새 에피소드
        NS_LOG_UNCOND("[MADDPG] Restored epsilon=" << m_epsilon
            << " stepCount=" << m_stepCount
            << " episode=" << m_episode);
    }
}

void
xAppHandoverSON::InitCsvLoggers()
{
    if (m_loadPretrained)
    {
        // 이어쓰기
        m_cellMetricsCsv.open("cell_metrics.csv", std::ios::app);
        m_cioActionsCsv.open("cio_actions.csv", std::ios::app);
        m_maddpgActionsCsv.open("maddpg_actions.csv", std::ios::app);
    }
    else
    {
        // 새로 작성
        m_cellMetricsCsv.open("cell_metrics.csv");
        m_cellMetricsCsv << "episode,time_s,cellId,ueCount,edgeUeCount,"
            << "cellDlThp_kbps,cellUlThp_kbps,"
            << "avgCqi,txPower_dBm" << std::endl;

        m_cioActionsCsv.open("cio_actions.csv");
        m_cioActionsCsv << "episode,time_s,srcCellId,neighborCellId,cioDB,cioIE" << std::endl;

        m_maddpgActionsCsv.open("maddpg_actions.csv");
        m_maddpgActionsCsv << "episode,time_s,cellId,cioRawAction,txpRawAction,"
            << "cioDB,txpApplied_dBm,cellThp_kbps,epsilon" << std::endl;
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
            if (ue.servingCellId == cellId)
            {
                avgCqi += ue.cqi;
                cqiCount++;
            }
        }
        if (cqiCount > 0) avgCqi /= cqiCount;

        m_cellMetricsCsv << m_episode << ","
            << now << ","
            << cellId << ","
            << cell.ueCount << ","
            << cell.edgeUeCount << ","
            << cell.totalThroughputDl << ","
            << cell.totalThroughputUl << ","
            << avgCqi << ","
            << cell.txPower << std::endl;
    }
}