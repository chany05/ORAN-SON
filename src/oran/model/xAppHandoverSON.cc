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
    if (m_stepCount % 1 == 0)
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
    /*
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
    */
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
    PurgeStaleUeContexts();

    CollectUeCount();
    CollectCellThroughput();  // ★ 추가: 셀 단위 DL+UL throughput 집계

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

    // DL/UL volume 후에 추가
    // PRB utilization 수집 (구간값 — delta 불필요)
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
            {
                it->second.prbUtilDl = util;
            }
        }
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
            cell.totalThroughputDl = 0;
            cell.totalThroughputUl = 0;

            std::cout << "[IpVol] Cell" << cellId
                << " DL=" << (cell.totalThroughputDl / 1e3) << "Mbps"
                << " UL=" << (cell.totalThroughputUl / 1e3) << "Mbps"
                << " UEs=" << cell.ueCount << std::endl;
        }
        

        double offeredMbps = cell.ueCount * 0.82;
        double actualMbps = cell.totalThroughputDl / 1e3;
        std::cout << "[SAT] Cell" << cellId
            << " UEs=" << cell.ueCount
            << " offered=" << offeredMbps << "Mbps"
            << " actual=" << actualMbps << "Mbps"
            << " ratio=" << (offeredMbps > 0 ? actualMbps / offeredMbps : 0)
            << " PRB util=" << cell.prbUtilDl
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

    // actDim = 이웃 셀 수 + 1(TXP) — 이웃 수에 따라 자동 결정
    // totalActDim = Σ(각 에이전트의 actDim)
    int64_t totalObsDim = NUM_AGENTS * OBS_DIM;
    int64_t totalActDim = 0;
    for (auto cellId : m_cellIds)
    {
        totalActDim += static_cast<int64_t>(m_neighborMap[cellId].size()) + 1;
    }

    for (auto cellId : m_cellIds)
    {
        AgentConfig config;
        config.cellId = cellId;
        config.obsDim = OBS_DIM;
        config.actDim = static_cast<int64_t>(m_neighborMap[cellId].size()) + 1;  // per-neighbor CIO + TXP
        config.neighborCellIds = m_neighborMap[cellId];

        m_agents.push_back(std::make_unique<MADDPGAgent>(
            config, totalObsDim, totalActDim, ACTOR_LR, CRITIC_LR, MAX_ACTION));
    }

    m_replayBuffer = std::make_unique<ReplayBuffer>(BUFFER_SIZE);

    NS_LOG_UNCOND("[MADDPG] Initialized: " << NUM_AGENTS << " agents, "
        << "Obs=" << OBS_DIM
        << " ActDim=[");
    for (size_t i = 0; i < m_agents.size(); i++)
    {
        NS_LOG_UNCOND("  Cell " << m_agents[i]->GetConfig().cellId
            << ": actDim=" << m_agents[i]->GetConfig().actDim
            << " (neighbors=" << m_agents[i]->GetConfig().neighborCellIds.size() << ")");
    }
    NS_LOG_UNCOND("] totalActDim=" << totalActDim
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
    obs[0] = avgCqi / 15.0f;  // ★ 정규화: CQI 0~15 → [0,1]

    // [1] Per-cell Throughput — T_i (정규화: Mbps/30 → ~[0,1])
    obs[1] = static_cast<float>(cell.totalThroughputDl / 1e3) / 30.0f;  // ★ per-cell, 40UE 포화 기준

    // [2] FarUes — E_i (edge UE 비율, 이미 0~1)
    obs[2] = (ueWithData > 0)
        ? static_cast<float>(edgeCount) / ueWithData : 0.0f;

    // [3] UE count — N_i (정규화: ueCount/totalUEs → [0,1])
    uint32_t totalUEs = m_ueContexts.size();
    obs[3] = (totalUEs > 0)
        ? static_cast<float>(cell.ueCount) / static_cast<float>(totalUEs) : 0.0f;

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
    // 1) 셀별 per-cell throughput 수집
    std::vector<double> thps;
    for (auto cellId : m_cellIds)
    {
        auto it = m_cellContexts.find(cellId);
        double thp = 0.0;
        if (it != m_cellContexts.end())
        {
            thp = (it->second.totalThroughputDl + it->second.totalThroughputUl)
                / 1e3 / 30.0;  // per-cell throughput 정규화 (~0~1)
        }
        std::cout << "cellId: " << cellId << " thp: " << thp << std::endl;
        thps.push_back(thp);
    }

    // 2) PRB 균형 패널티
    std::vector<double> prbUtils;
    for (auto cellId : m_cellIds)
    {
        auto it = m_cellContexts.find(cellId);
        prbUtils.push_back(it != m_cellContexts.end() ? it->second.prbUtilDl : 0.0);
    }
    double meanPrb = std::accumulate(prbUtils.begin(), prbUtils.end(), 0.0) / prbUtils.size();
    double prbVar = 0.0;
    for (double p : prbUtils) prbVar += (p - meanPrb) * (p - meanPrb);
    double stdPrb = std::sqrt(prbVar / prbUtils.size());

    // 3) UE 분배 균형 패널티
    std::vector<double> ueCounts;
    double totalUEs = 0.0;
    for (auto cellId : m_cellIds)
    {
        auto it = m_cellContexts.find(cellId);
        double ue = (it != m_cellContexts.end()) ? (double)it->second.ueCount : 0.0;
        ueCounts.push_back(ue);
        totalUEs += ue;
    }
    double meanUe = totalUEs / ueCounts.size();
    double ueVar = 0.0;
    for (double u : ueCounts) ueVar += (u - meanUe) * (u - meanUe);
    double stdUe = std::sqrt(ueVar / ueCounts.size());
    double stdUeNorm = (totalUEs > 0) ? stdUe / totalUEs : 0.0;  // 0~1 정규화

    std::cout << " stdPrb: " << stdPrb
              << " stdUe: " << stdUe << " stdUeNorm: " << stdUeNorm << std::endl;

    // 4) 최종 reward: per-cell thp - PRB 패널티 - UE 균형 패널티
    double w_thp   = 3.0;   // THP 가중치
    double alpha   = 2.0;   // PRB 균형 가중치
    double beta    = 10.0;  // UE 분배 균형 가중치
    std::vector<double> rewards;
    for (size_t i = 0; i < thps.size(); i++)
    {
        double r = w_thp * thps[i] - alpha * stdPrb - beta * stdUeNorm;
        rewards.push_back(r);
        std::cout << " reward: " << r << std::endl;
    }

    // ★ reward curve 로깅
    if (m_rewardCurveCsv.is_open())
    {
        double now = Simulator::Now().GetSeconds();
        m_rewardCurveCsv << now << "," << m_stepCount;
        for (size_t i = 0; i < rewards.size(); i++)
            m_rewardCurveCsv << "," << rewards[i];
        m_rewardCurveCsv << "," << stdPrb << "," << stdUeNorm << std::endl;
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

    // ── 이전 전이를 replay buffer에 저장 (학습 모드만) ──
    std::vector<double> stepRewards;  // 고착화 체크에서도 사용
    if (m_hasPrevStep)
    {
        stepRewards = ComputeRewards();

        if (!m_inferenceOnly)
        {
            Experience exp;
            exp.obs = m_prevObs;
            exp.acts = m_prevActs;
            exp.rewards = stepRewards;
            exp.nextObs = currentObs;
            // 시뮬레이션 종료 직전이면 done=true → target Q에서 γ×Q'(next) 제거
            double now = Simulator::Now().GetSeconds();
            exp.done = (m_simStopTime - now <= m_sonPeriodicitySec * 2.0);
            m_replayBuffer->Push(std::move(exp));
        }

        if (m_stepCount % 10 == 0)
        {
            NS_LOG_UNCOND("[MADDPG] Step=" << m_stepCount
                << " Rewards=[" << stepRewards[0] << ", "
                << stepRewards[1] << ", " << stepRewards[2] << "]"
                << " BufferSize=" << m_replayBuffer->Size()
                << " Epsilon=" << m_epsilon);


            // NS_LOG_UNCOND 대신 std::cout 사용!
            std::cout << "🚀 [MADDPG] Step=" << m_stepCount
                    << " | Reward=" << stepRewards[0] << ", " << stepRewards[1] << ", " << stepRewards[2]
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
        int64_t agentActDim = m_agents[i]->GetConfig().actDim;

        if (!m_inferenceOnly && uniformDist(rng) <= m_epsilon)
        {
            // ε-greedy: 랜덤 행동
            std::vector<float> randAct(agentActDim);
            for (int64_t d = 0; d < agentActDim; d++)
                randAct[d] = static_cast<float>(randomAction(rng));
            act = torch::from_blob(randAct.data(), {agentActDim}, torch::kFloat32).clone();
        }
        else if (!m_inferenceOnly)
        {
            // Actor + 가우시안 노이즈
            act = m_agents[i]->SelectAction(currentObs[i]);
            std::vector<float> noise(agentActDim);
            for (int64_t d = 0; d < agentActDim; d++)
                noise[d] = static_cast<float>(noiseDist(rng));
            auto noiseTensor = torch::from_blob(noise.data(), {agentActDim}, torch::kFloat32).clone();
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
        int txpIdx = static_cast<int>(neighbors.size());  // TXP는 마지막 인덱스

        // ── per-neighbor CIO 변환 ──
        // action[0..N-1] ∈ [-1, 1] → 각 이웃별 독립 CIO
        // 3GPP TS 36.331 §6.3.6: q-OffsetCell IE = dB × 2 (0.5dB 단위)
        Json cioList = Json::array();
        for (size_t n = 0; n < neighbors.size(); n++)
        {
            int cioDB = static_cast<int>(std::round(actData[n] * 5.0));
            cioDB = std::max(-5, std::min(5, cioDB));
            int cioIE = cioDB * 2;

            Json entry;
            entry["CELL_ID"] = neighbors[n];
            entry["CIO_VALUE"] = cioIE;
            cioList.push_back(entry);
        }
        std::string endpoint = "/E2Node/" + std::to_string(srcCell) + "/";
        ric->E2SmRcSendCioControlRequest(cioList, endpoint);

        // ── TXP 변환 (마지막 action 차원) ──
        // action[N] ∈ [-1, 1] → ×4 → ±4 dBm offset from base
        double txpOffset = static_cast<double>(actData[txpIdx]) * 4.0;
        double txpApplied = m_txPower + txpOffset;
        txpApplied = std::max(26.0, std::min(38.0, txpApplied));

        ric->E2SmRcSendTxPowerControlRequest(txpApplied, endpoint);

        // ── 콘솔 로그 ──
        std::cout << "[ACT] Step=" << m_stepCount
            << " Cell" << srcCell << " CIO=[";
        for (size_t n = 0; n < neighbors.size(); n++)
        {
            int cioDB = static_cast<int>(std::round(actData[n] * 5.0));
            cioDB = std::max(-5, std::min(5, cioDB));
            std::cout << " " << neighbors[n] << ":" << cioDB << "dB";
            if (n + 1 < neighbors.size()) std::cout << ", ";
        }
        std::cout << "] TXP=" << txpApplied << "dBm(off=" << txpOffset << ")"
            << " raw=[";
        for (int64_t d = 0; d < agentActDim; d++)
        {
            std::cout << actData[d];
            if (d + 1 < agentActDim) std::cout << ",";
        }
        std::cout << "]" << std::endl;

        //if (m_stepCount % 10 == 0)
        {
            double now = Simulator::Now().GetSeconds();

            // CIO 로그 — 이웃별 독립 CIO 기록
            for (size_t n = 0; n < neighbors.size(); n++)
            {
                int cioDB = static_cast<int>(std::round(actData[n] * 5.0));
                cioDB = std::max(-5, std::min(5, cioDB));
                int cioIE = cioDB * 2;
                m_cioActionsCsv << now << ","
                    << srcCell << "," << neighbors[n] << ","
                    << cioDB << "," << cioIE << std::endl;
            }

            // MADDPG 행동 로그 — 모든 raw action + 변환값
            double cellThp = 0.0;
            auto cellIt = m_cellContexts.find(srcCell);
            if (cellIt != m_cellContexts.end())
                cellThp = cellIt->second.totalThroughputDl + cellIt->second.totalThroughputUl;

            m_maddpgActionsCsv << now << "," << srcCell;
            // raw actions
            for (int64_t d = 0; d < agentActDim; d++)
                m_maddpgActionsCsv << "," << actData[d];
            // converted CIO dB values
            for (size_t n = 0; n < neighbors.size(); n++)
            {
                int cioDB = static_cast<int>(std::round(actData[n] * 5.0));
                cioDB = std::max(-5, std::min(5, cioDB));
                m_maddpgActionsCsv << "," << cioDB;
            }
            m_maddpgActionsCsv << "," << txpApplied
                << "," << cellThp << "," << m_epsilon << std::endl;
        }
    }

    // ── 고착화 체크 (이전 reward와 현재 action 비교) ──
    if (!stepRewards.empty())
    {
        CheckAndLogStagnation(currentActs, stepRewards);
    }

    // ── epsilon 감소 (학습 모드만) ──
    if (!m_inferenceOnly && m_epsilon > EPSILON_END)
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
    auto doneBatch = torch::zeros({(long)B, 1});  // done 마스크

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
            // done=1이면 γ×Q'(next)=0 → target Q = reward만
            targetQ = rewardBatch[i] + GAMMA * (1.0 - doneBatch) * targetQ;
        }

        auto currentQ = agent->GetCritic()->forward(allObs, allActs);
        auto criticLoss = torch::mse_loss(currentQ, targetQ);

        agent->GetCriticOpt().zero_grad();
        criticLoss.backward();
        torch::nn::utils::clip_grad_norm_(agent->GetCritic()->parameters(), 0.5);
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
        torch::nn::utils::clip_grad_norm_(agent->GetActor()->parameters(), 0.5);
        agent->GetActorOpt().step();

        // ─ Target Soft Update — 논문 Eq.(11)(12) ─
        agent->SoftUpdateTargets(TAU_SOFT);

        if (m_stepCount % 10 == 0)
        {
            float curActorLoss  = actorLoss.item<float>();
            float curCriticLoss = criticLoss.item<float>();
            NS_LOG_UNCOND("[MADDPG] Train Agent=" << i
                << " CriticLoss=" << curCriticLoss
                << " ActorLoss=" << curActorLoss);

            static float prevActorLoss[NUM_AGENTS] = {0.f, 0.f, 0.f};
            float actorLossDelta = curActorLoss - prevActorLoss[i];
            prevActorLoss[i] = curActorLoss;

            auto actStd = policyActs[i].detach().std(0);
            float actStdMean = actStd.mean().item<float>();

            auto qValues = currentQ.detach();
            float qMean = qValues.mean().item<float>();
            float qStd  = qValues.std().item<float>();

            auto actVar = policyActs[i].detach().var(0);
            auto perDimEntropy = 0.5 * torch::log(2.0 * M_PI * M_E * (actVar + 1e-8));
            float entropy = perDimEntropy.sum().item<float>();

            std::cout << "[MADDPG Train] Step=" << m_stepCount
                      << " | Agent=" << i
                      << " | CriticLoss=" << curCriticLoss
                      << " | ActorLoss=" << curActorLoss
                      << " | ActorLossDelta=" << actorLossDelta
                      << " | ActionStd=" << actStdMean
                      << " | Q_mean=" << qMean
                      << " | Q_std=" << qStd
                      << " | ActionEntropy=" << entropy
                      << std::endl;
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
        
    if(!m_inferenceOnly){
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
        m_replayBuffer->Load(bufferPath, OBS_DIM, 0 /*unused*/, NUM_AGENTS);
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
        metaFile.close();
        NS_LOG_UNCOND("[MADDPG] Restored epsilon=" << m_epsilon
            << " stepCount=" << m_stepCount);
    }
}

void
xAppHandoverSON::InitCsvLoggers()
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
        // 동적 헤더: 이웃 수에 따라 CIO 컬럼 자동 생성
        size_t maxNeighbors = 0;
        for (auto& [cellId, neighbors] : m_neighborMap)
            maxNeighbors = std::max(maxNeighbors, neighbors.size());
        m_maddpgActionsCsv << "time_s,cellId";
        for (size_t n = 0; n < maxNeighbors; n++)
            m_maddpgActionsCsv << ",cioRaw_n" << n;
        m_maddpgActionsCsv << ",txpRaw";
        for (size_t n = 0; n < maxNeighbors; n++)
            m_maddpgActionsCsv << ",cioDB_n" << n;
        m_maddpgActionsCsv << ",txpApplied_dBm,cellThp_kbps,epsilon" << std::endl;

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

        m_cellMetricsCsv 
            << now << ","
            << cellId << ","
            << cell.ueCount << ","
            << cell.edgeUeCount << ","
            << cell.totalThroughputDl << ","
            << cell.totalThroughputUl << ","
            << avgCqi << ","
            << cell.txPower << ","
            << cell.prbUtilDl << std::endl;
    }
}

// =============================================================================
// 고착화(Stagnation) 체크 및 CSV 로깅
// =============================================================================
// 매 스텝마다 에이전트별로:
//   1) CIO 변화량: 이전 CIO raw action과의 L2 norm (TXP 제외)
//   2) TXP 변화량: TXP raw action 변화 (마지막 차원, 단독 추적)
//   3) TXP 실제 적용값: raw → dBm 변환값
//   4) UE 분포 변화량, Reward 변화량
//   5) 연속 고착 스텝: CIO/TXP 각각 독립 추적
void
xAppHandoverSON::CheckAndLogStagnation(
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
        int64_t cioEndIdx = static_cast<int64_t>(neighbors.size());  // TXP는 마지막

        // 현재 action을 vector로 변환
        std::vector<float> currAct(actDim);
        for (int64_t d = 0; d < actDim; d++)
            currAct[d] = actData[d];

        // 현재 TXP raw 및 적용값
        float txpRaw = currAct[cioEndIdx];
        double txpApplied = m_txPower + static_cast<double>(txpRaw) * 4.0;
        txpApplied = std::max(26.0, std::min(38.0, txpApplied));

        // --- CIO / TXP 변화량 분리 계산 ---
        double cioL2Change = -1.0;
        double maxCioChange = -1.0;
        double txpRawChange = -1.0;
        float prevTxpRaw = txpRaw;

        auto prevIt = m_prevRawActions.find(cellId);
        if (prevIt != m_prevRawActions.end() && (int64_t)prevIt->second.size() == actDim)
        {
            // CIO 차원만 (0 ~ cioEndIdx-1)
            cioL2Change = 0.0;
            maxCioChange = 0.0;
            for (int64_t d = 0; d < cioEndIdx; d++)
            {
                double diff = std::abs(currAct[d] - prevIt->second[d]);
                cioL2Change += diff * diff;
                maxCioChange = std::max(maxCioChange, diff);
            }
            cioL2Change = std::sqrt(cioL2Change);

            // TXP 차원 (마지막)
            prevTxpRaw = prevIt->second[cioEndIdx];
            txpRawChange = std::abs(txpRaw - prevTxpRaw);
        }

        // TXP 이전 적용값
        double prevTxpApplied = m_txPower + static_cast<double>(prevTxpRaw) * 4.0;
        prevTxpApplied = std::max(26.0, std::min(38.0, prevTxpApplied));

        // --- UE 분포 변화량 ---
        uint32_t ueCountCurr = 0;
        auto cellIt = m_cellContexts.find(cellId);
        if (cellIt != m_cellContexts.end())
            ueCountCurr = cellIt->second.ueCount;

        int ueCountChange = 0;
        uint32_t ueCountPrev = ueCountCurr;
        auto uePrevIt = m_prevUeCounts.find(cellId);
        if (uePrevIt != m_prevUeCounts.end())
        {
            ueCountPrev = uePrevIt->second;
            ueCountChange = static_cast<int>(ueCountCurr) - static_cast<int>(ueCountPrev);
        }

        // --- Reward 변화량 ---
        double rewardCurr = (i < rewards.size()) ? rewards[i] : 0.0;
        double rewardPrev = rewardCurr;
        double rewardChange = 0.0;
        if (i < m_prevRewards.size())
        {
            rewardPrev = m_prevRewards[i];
            rewardChange = std::abs(rewardCurr - rewardPrev);
        }

        // --- 고착 판정 (CIO, TXP 독립) ---
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

        // --- CSV 기록 ---
        m_stagnationCsv << now << "," << m_stepCount << "," << cellId << ","
            << cioL2Change << "," << maxCioChange << ","
            << txpRaw << "," << prevTxpRaw << "," << txpRawChange << ","
            << txpApplied << "," << prevTxpApplied << ","
            << ueCountPrev << "," << ueCountCurr << "," << ueCountChange << ","
            << rewardPrev << "," << rewardCurr << "," << rewardChange << ","
            << m_stagnantSteps << ","
            << (cioStagnant ? 1 : 0) << ","
            << (txpStagnant ? 1 : 0) << ","
            << (isStagnant ? 1 : 0) << std::endl;

        // 이전 값 갱신
        m_prevRawActions[cellId] = currAct;
        m_prevUeCounts[cellId] = ueCountCurr;
    }

    // 전체 에이전트가 고착이면 카운터 증가, 아니면 리셋
    if (allStagnant && !m_prevRewards.empty())
        m_stagnantSteps++;
    else
        m_stagnantSteps = 0;

    // 이전 reward 갱신
    m_prevRewards = rewards;

    // 콘솔 경고: 연속 고착이 길어지면 알림
    if (m_stagnantSteps >= 10 && m_stagnantSteps % 10 == 0)
    {
        NS_LOG_UNCOND("[STAGNATION] WARNING: " << m_stagnantSteps
            << " consecutive stagnant steps detected at t=" << now << "s");
    }
}