#include "xAppHandoverSON.h"

#include "ns3/E2AP.h"
#include "ns3/core-module.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using namespace ns3;
using namespace oran;

NS_LOG_COMPONENT_DEFINE("xAppHandoverSON");

xAppHandoverSON::xAppHandoverSON(float sonPeriodicitySec, bool initiateHandovers) ////여기서 각 파라미터 조정, m_loadThreshold 등의 파라미터를 RL을 통해 나온 값으로 대체하면 됨
    : xAppHandover(),
      m_sonPeriodicitySec(sonPeriodicitySec),
      m_initiateHandovers(initiateHandovers),
      m_cellRadius(40.0),
      m_edgeThreshold(0.8),
      m_loadThreshold(3.0),
      m_rsrqThreshold(-15.0),
      m_cqiThreshold(11),
      m_txPower(46.0),
      m_frequency(2.12e9)
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

    Simulator::Schedule(Seconds(m_sonPeriodicitySec),
                        &xAppHandoverSON::PeriodicSONCheck,
                        this);
}

// =============================================================================
// 주기적 SON 체크
// =============================================================================
void
xAppHandoverSON::PeriodicSONCheck()
{
    NS_LOG_FUNCTION(this);

    // 1. KPM 수집
    CollectKPMs();

    // ===== 디버그: 수집된 데이터 출력 =====
    NS_LOG_UNCOND("[SON] === Periodic Check === UEs=" << m_ueContexts.size()
        << " Cells=" << m_cellContexts.size());

    for (auto& [key, ue] : m_ueContexts)
    {
        NS_LOG_UNCOND("[SON]   UE RNTI=" << ue.rnti
            << " Cell=" << ue.servingCellId
            << " RSRP=" << ue.servingRsrp
            << " RSRQ=" << ue.servingRsrq
            << " CQI=" << ue.cqi
            << " DL=" << ue.throughputDl << "kbps"
            << " UL=" << ue.throughputUl << "kbps");
    }
    // ===== 디버그 끝 =====

    // 2. Edge UE 계산
    CalculateEdgeUEs();

    // 3. 부하 점수 계산
    CalculateLoadScores();

    // 4. 핸드오버 의사결정 및 실행
    if (m_initiateHandovers)
    {
        E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");

        for (auto& [key, ue] : m_ueContexts)
        {
            if (m_imsiInHandover.find(key) != m_imsiInHandover.end())
                continue;

            uint16_t targetCell = MakeSONDecision(key); //키값은 servingCellId와 rnti로 구성되어 있음 -> 두 연결 구성의 고유 식별 번호

            if (targetCell != std::numeric_limits<uint16_t>::max() &&
                targetCell != ue.servingCellId)
            {
                std::string srcEndpoint = "/E2Node/" + std::to_string(ue.servingCellId) + "/";
                ric->E2SmRcSendHandoverControlRequest(ue.rnti, targetCell, srcEndpoint);
                m_imsiInHandover.emplace(key, 0); //더미데이터를 넣음으로써 핸드오버 중복 실행 방지 -> 얘는 건들이지 말란뜻

                NS_LOG_UNCOND("[SON] Handover initiated RNTI=" << ue.rnti
                    << " from Cell=" << ue.servingCellId
                    << " to Cell=" << targetCell);
            }
        }
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

    CollectRsrpRsrq();
    CollectTargetRsrq();   // 추가
    CollectCqi();
    CollectThroughput();
    CollectUeCount();
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
        std::string mostRecentTimestamp("");
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (mostRecentTimestamp.empty())
                mostRecentTimestamp = measurement.timestamp;
            if (mostRecentTimestamp != measurement.timestamp)
                continue;

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

    // RSRQ
    auto rsrqMap = ric->QueryKpmMetric("/KPM/HO.SrcCellQual.RSRQ");
    for (auto& e2nodeMeasurements : rsrqMap)
    {
        std::string mostRecentTimestamp("");
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (mostRecentTimestamp.empty())
                mostRecentTimestamp = measurement.timestamp;
            if (mostRecentTimestamp != measurement.timestamp)
                continue;

            if (!measurement.measurements.contains("RNTI") ||
                !measurement.measurements.contains("CELLID"))
                continue;

            uint16_t rnti = measurement.measurements["RNTI"];
            uint16_t cellId = measurement.measurements["CELLID"];

            UeKey key = MakeUeKey(cellId, rnti);
            if (m_ueContexts.find(key) != m_ueContexts.end())
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
        std::string mostRecentTimestamp("");
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (mostRecentTimestamp.empty())
                mostRecentTimestamp = measurement.timestamp;
            if (mostRecentTimestamp != measurement.timestamp)
                continue;

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
                m_ueContexts[key].neighborRsrq[targetCellId] = rsrq;
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
        std::string mostRecentTimestamp("");
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (mostRecentTimestamp.empty())
                mostRecentTimestamp = measurement.timestamp;
            if (mostRecentTimestamp != measurement.timestamp)
                continue;

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
xAppHandoverSON::CollectThroughput()
{
    NS_LOG_FUNCTION(this);
    E2AP* ric = (E2AP*)E2AP::RetrieveInstanceWithEndpoint("/E2Node/0");

    // DL
    auto dlMap = ric->QueryKpmMetric("/KPM/DRB.IpThpDl.QCI");
    for (auto& e2nodeMeasurements : dlMap)
    {
        std::string mostRecentTimestamp("");
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (mostRecentTimestamp.empty())
                mostRecentTimestamp = measurement.timestamp;
            if (mostRecentTimestamp != measurement.timestamp)
                continue;

            if (!measurement.measurements.contains("RNTI") ||
                !measurement.measurements.contains("CELLID"))
                continue;

            uint16_t rnti = measurement.measurements["RNTI"];
            uint16_t cellId = measurement.measurements["CELLID"];

            UeKey key = MakeUeKey(cellId, rnti);
            if (m_ueContexts.find(key) != m_ueContexts.end())
            {
                m_ueContexts[key].throughputDl = measurement.measurements["VALUE"];
            }
        }
    }

    // UL
    auto ulMap = ric->QueryKpmMetric("/KPM/DRB.IpThpUl.QCI");
    for (auto& e2nodeMeasurements : ulMap)
    {
        std::string mostRecentTimestamp("");
        for (auto& measurement : e2nodeMeasurements.second)
        {
            if (mostRecentTimestamp.empty())
                mostRecentTimestamp = measurement.timestamp;
            if (mostRecentTimestamp != measurement.timestamp)
                continue;

            if (!measurement.measurements.contains("RNTI") ||
                !measurement.measurements.contains("CELLID"))
                continue;

            uint16_t rnti = measurement.measurements["RNTI"];
            uint16_t cellId = measurement.measurements["CELLID"];

            UeKey key = MakeUeKey(cellId, rnti);
            if (m_ueContexts.find(key) != m_ueContexts.end())
            {
                m_ueContexts[key].throughputUl = measurement.measurements["VALUE"];
            }
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
        m_cellContexts[cellId].ueCount++;
        if (ue.isEdge)
        {
            m_cellContexts[cellId].edgeUeCount++;
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

    for (auto& [key, ue] : m_ueContexts)
    {
        double distance = FriisDistanceEstimate(ue.servingRsrp, m_txPower, m_frequency);
        ue.isEdge = (distance > m_cellRadius * m_edgeThreshold);
    }
}

double
xAppHandoverSON::FriisDistanceEstimate(double rsrp_dBm, double txPower_dBm, double freq_Hz)
{
    double pathLoss = txPower_dBm - rsrp_dBm;
    double c = 3.0e8;
    double lambda = c / freq_Hz;
    double d = lambda * std::pow(10.0, pathLoss / 20.0) / (4.0 * M_PI);
    return d;
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

    // SON 로직으로 대체 결정
    uint16_t sonDecision = MakeSONDecision(key);
    if (sonDecision != std::numeric_limits<uint16_t>::max())
        decidedTargetCellId = sonDecision;

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
