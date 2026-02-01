#include "xAppHandoverMlpackKmeans.h"

#include "ns3/E2AP.h"
#include "ns3/core-module.h"

#include <mlpack/core.hpp>
#include <mlpack/methods/kmeans/allow_empty_clusters.hpp>
#include <mlpack/methods/kmeans/kmeans.hpp>
#include <mlpack/prereqs.hpp>

#include <array>
#include <cstdlib>
#include <limits>
#include <map>
#include <string>
#include <vector>

using namespace ns3;
using namespace oran;

NS_LOG_COMPONENT_DEFINE("xAppHandoverMlpackKmeans");

namespace
{
using UeKey = uint32_t;

// UE key = (servingCellId, rnti)
static inline UeKey
MakeUeKey(uint16_t servingCellId, uint16_t rnti)
{
    return (static_cast<UeKey>(servingCellId) << 16) | static_cast<UeKey>(rnti);
}

static inline uint16_t
KeyToServingCell(UeKey k)
{
    return static_cast<uint16_t>((k >> 16) & 0xFFFF);
}

static inline uint16_t
KeyToRnti(UeKey k)
{
    return static_cast<uint16_t>(k & 0xFFFF);
}
} // namespace

xAppHandoverMlpackKmeans::xAppHandoverMlpackKmeans(bool kmeansEmptyPolicy,
                                                   float clusteringPeriodicitySec,
                                                   bool initiateHandovers)
    : xAppHandover(),
      m_kmeansKeepEmptyPolicy(kmeansEmptyPolicy),
      m_clusteringPeriodicitySec(clusteringPeriodicitySec),
      m_initiateHandovers(initiateHandovers)
{
    NS_LOG_FUNCTION(this);

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&xAppHandoverMlpackKmeans::HandoverSucceeded, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&xAppHandoverMlpackKmeans::HandoverStarted, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                    MakeCallback(&xAppHandoverMlpackKmeans::HandoverFailed, this));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback(&xAppHandoverMlpackKmeans::ConnectionEstablished, this));
    Simulator::Schedule(Seconds(m_clusteringPeriodicitySec),
                        &xAppHandoverMlpackKmeans::PeriodicClustering,
                        this);
};

void
xAppHandoverMlpackKmeans::PeriodicClustering()
{
    NS_LOG_FUNCTION(this);

    E2AP* ric = (E2AP*)static_cast<const E2AP*>(xApp::RetrieveInstanceWithEndpoint("/E2Node/0"));

    // 원본 rntis(map<rnti, col>) -> UEKey(map<UeKey, col>)
    std::map<UeKey, uint16_t> ues;

    std::array<std::string, 4> kpmMetrics = {//"/KPM/HO.SrcCellQual.RSRP",
                                             "/KPM/HO.SrcCellQual.RSRQ",
                                             //"/KPM/HO.TrgtCellQual.RSRP",
                                             "/KPM/HO.TrgtCellQual.RSRQ"};

    std::map<uint16_t, uint16_t> cells;
    uint16_t i_ue = 0;
    uint16_t i_cell = 0;

    // Count the number of eNodeBs/E2Nodes and the number of UEs
    for (auto kpmMetric : kpmMetrics)
    {
        auto metricMap = ric->QueryKpmMetric(kpmMetric);

        if (metricMap.size() == 0)
        {
            continue;
        }

        for (auto& e2nodeMeasurements : metricMap)
        {
            std::string cellIdStr(e2nodeMeasurements.first.begin() +
                                      e2nodeMeasurements.first.find_last_of("/") + 1,
                                  e2nodeMeasurements.first.end());
            uint16_t endpointCellId = std::atoi(cellIdStr.c_str());
            if (cells.find(endpointCellId) == cells.end())
            {
                cells[endpointCellId] = i_cell++;
            }

            std::string mostRecentTimestamp("");
            for (auto& measurementDeque : e2nodeMeasurements.second)
            {
                if (mostRecentTimestamp == "")
                {
                    mostRecentTimestamp = measurementDeque.timestamp;
                }
                if (mostRecentTimestamp != measurementDeque.timestamp)
                {
                    // Skip old measurements
                    continue;
                }

                uint16_t rnti = measurementDeque.measurements["RNTI"];

                // UEKey를 만들려면 servingCellId가 필요함.
                if (kpmMetric == "/KPM/HO.SrcCellQual.RSRQ")
                {
                    if (!measurementDeque.measurements.contains("CELLID"))
                    {
                        continue;
                    }
                    uint16_t servingCellId = measurementDeque.measurements["CELLID"];
                    servingCellId++; 

                    UeKey key = MakeUeKey(servingCellId, rnti);
                    if (ues.find(key) == ues.end())
                    {
                        ues[key] = i_ue++;
                    }
                }
            }
        }
    }

    // If there is nothing to cluster, end early
    if (ues.size() == 0 || cells.size() == 0)
    {
        Simulator::Schedule(Seconds(1), &xAppHandoverMlpackKmeans::PeriodicClustering, this);
        return;
    }

    // Rows are cells and columns are UEs
    arma::mat dataset = arma::zeros(cells.size(), ues.size());

    // 원본: m_rntiToCurrentCellId.clear();
    // 변경: UEKey 기반으로 current cell 저장
    m_rntiToCurrentCellId.clear();

    // Collate data into an armadillo matrix for processing
    for (auto kpmMetric : kpmMetrics)
    {
        auto metricMap = ric->QueryKpmMetric(kpmMetric);

        if (metricMap.size() == 0)
        {
            continue;
        }

        for (auto& e2nodeMeasurements : metricMap)
        {
            // Trgt metric에서 CELLID가 없을 수 있으므로,
            // endpoint cellId는 "serving eNB(=RNTI 소속)"로 사용 가능
            std::string endpointCellIdStr(e2nodeMeasurements.first.begin() +
                                              e2nodeMeasurements.first.find_last_of("/") + 1,
                                          e2nodeMeasurements.first.end());
            uint16_t endpointCellId = std::atoi(endpointCellIdStr.c_str());

            std::string mostRecentTimestamp("");
            for (auto& measurementDeque : e2nodeMeasurements.second)
            {
                if (mostRecentTimestamp == "")
                {
                    mostRecentTimestamp = measurementDeque.timestamp;
                }
                if (mostRecentTimestamp != measurementDeque.timestamp)
                {
                    // Skip old measurements
                    continue;
                }

                uint16_t rnti = measurementDeque.measurements["RNTI"];

                if (kpmMetric == "/KPM/HO.SrcCellQual.RSRQ")
                {
                    if (!measurementDeque.measurements.contains("CELLID"))
                    {
                        continue;
                    }

                    uint16_t servingCellId = measurementDeque.measurements["CELLID"];
                    servingCellId++; // 원본 보존

                    if (cells.find(servingCellId) == cells.end())
                        continue;

                    UeKey key = MakeUeKey(servingCellId, rnti);

                    auto itUe = ues.find(key);
                    if (itUe == ues.end())
                        continue;

                    uint16_t ue_offset = itUe->second;

                    // current cell 갱신
                    m_rntiToCurrentCellId[key] = servingCellId;

                    uint16_t cellid_offset = cells.at(servingCellId);
                    dataset.at(cellid_offset, ue_offset) = measurementDeque.measurements["VALUE"];
                }
                else
                {
                    // Trgt metric: measurement["TARGET"]는 target cellId(대개 1-based)
                    // serving cell(=RNTI 소속)은 endpointCellId로 삼는다 (원본과 가장 유사한 최소 변경).
                    uint16_t servingCellId = endpointCellId;
                    if (servingCellId == 0)
                        continue;

                    UeKey key = MakeUeKey(servingCellId, rnti);

                    auto itUe = ues.find(key);
                    if (itUe == ues.end())
                    {
                        // UE 세트는 Src metric 기반으로 만들었으므로, 없으면 스킵
                        continue;
                    }

                    uint16_t ue_offset = itUe->second;

                    uint16_t targetCellId = measurementDeque.measurements["TARGET"];
                    if (cells.find(targetCellId) == cells.end())
                        continue;

                    uint16_t cellid_offset = cells.at(targetCellId);
                    dataset.at(cellid_offset, ue_offset) = measurementDeque.measurements["VALUE"];
                }
            }
        }
    }

    // Prepare to run K-means
    arma::Row<size_t> assignments;
    arma::mat centroids;
    if (m_kmeansKeepEmptyPolicy)
    {
        mlpack::kmeans::KMeans<mlpack::metric::EuclideanDistance,
                               mlpack::kmeans::SampleInitialization,
                               mlpack::kmeans::AllowEmptyClusters,
                               mlpack::kmeans::NaiveKMeans,
                               arma::mat>
            k;
        k.Cluster(dataset, cells.size(), assignments, centroids);
    }
    else
    {
        mlpack::kmeans::KMeans<> k;
        k.Cluster(dataset, cells.size(), assignments, centroids);
    }

    // Print dataset with each column representing measurements from a UE
    std::cout << dataset << std::endl;

    // Print assigned clusters to each UE
    std::cout << assignments << std::endl;

    m_rntiToClusteredCellId.clear();

    // Translate assigned clusters into cellIds
    int i = 0;
    for (auto& ue : ues) // ue.first = UeKey
    {
        uint16_t decidedCellId = static_cast<uint16_t>(-1);
        for (auto& cell : cells)
        {
            if (cell.second == (uint16_t)assignments.at(i))
            {
                decidedCellId = cell.first;
                break;
            }
        }
        m_rntiToClusteredCellId[ue.first] = decidedCellId;
        i++;
    }

    if (m_initiateHandovers)
    {
        // Command handovers
        for (auto& kv : m_rntiToCurrentCellId) // key=UeKey, value=connectedCell
        {
            UeKey key = kv.first;

            // connected cell은 map 값도 있고, key에서 뽑아도 됨. (둘 일치가 정상)
            uint16_t connectedCell = kv.second;
            uint16_t rnti = KeyToRnti(key);

            // Skip UEs that were not assigned to cluster
            auto ueToCluster = m_rntiToClusteredCellId.find(key);
            if (ueToCluster == m_rntiToClusteredCellId.end())
                continue;

            // Skip UEs that are already in the target cluster cell id
            if (ueToCluster->second == connectedCell)
                continue;

            // Skip UEs already in handover
            if (m_imsiInHandover.find(key) != m_imsiInHandover.end())
            {
                continue;
            }

            // Spoof RIC CONTROL REQUEST with source eNB endpoint (where the UE is currently connected)
            std::string spoofed_src_endpoint = "/E2Node/" + std::to_string(connectedCell) + "/";
            ric->E2SmRcSendHandoverControlRequest(rnti,
                                                  ueToCluster->second,
                                                  spoofed_src_endpoint);

            // mark in-handover (IMSI는 HandoverStarted에서 채움)
            m_imsiInHandover.emplace(key, 0);
        }
    }

    Simulator::Schedule(Seconds(m_clusteringPeriodicitySec),
                        &xAppHandoverMlpackKmeans::PeriodicClustering,
                        this);
}

void
xAppHandoverMlpackKmeans::HandoverDecision(Json& payload)
{
    NS_LOG_FUNCTION(this);

    // Check if we are not receiving invalid payloads
    if (E2AP::RetrieveInstanceWithEndpoint(GetRootEndpoint())->GetNode() !=
        E2AP::RetrieveInstanceWithEndpoint("/E2Node/0")->GetNode())
    {
        NS_ABORT_MSG("Trying to run a xApp on a E2Node is a no-no");
    }

    // Read inputs from the json
    uint16_t requestingRnti = payload["RNTI"];
    uint16_t requestedTargetCellId = payload["Target Primary Cell ID"];

    // serving cell은 E2SM-RC에서 내려준 "Source Primary Cell ID"를 사용 (이미 1-based 가정)
    uint16_t servingCellId = 0;
    if (payload.contains("Source Primary Cell ID"))
    {
        servingCellId = payload["Source Primary Cell ID"];
    }
    else if (payload.contains("CELLID"))
    {
        // fallback only: old behavior
        servingCellId = payload["CELLID"];
        servingCellId++; // only here
    }

    // serving cell 없으면 UEKey를 만들 수 없으므로 reject
    if (servingCellId == 0)
    {
        payload["Target Primary Cell ID"] = std::numeric_limits<uint16_t>::max();
        m_decision_history.push_back({requestingRnti, requestedTargetCellId,
                                      std::numeric_limits<uint16_t>::max()});
        return;
    }

    UeKey key = MakeUeKey(servingCellId, requestingRnti);

    // Do the processing
    uint16_t decidedTargetCellId = requestedTargetCellId;

    // Use K-means clustering results
    if (m_rntiToClusteredCellId.find(key) != m_rntiToClusteredCellId.end())
        decidedTargetCellId = m_rntiToClusteredCellId.at(key);

    // If the result is invalid (same as current cell or unknown), use 0xFFFF to reject the handover
    if (m_rntiToCurrentCellId.find(key) == m_rntiToCurrentCellId.end() ||
        decidedTargetCellId == m_rntiToCurrentCellId.at(key))
        decidedTargetCellId = std::numeric_limits<uint16_t>::max();

    // Check if it is already in handover
    if (m_imsiInHandover.find(key) != m_imsiInHandover.end())
        decidedTargetCellId = std::numeric_limits<uint16_t>::max();

    // If not in handover, mark it with an empty IMSI field
    if (decidedTargetCellId != std::numeric_limits<uint16_t>::max())
        m_imsiInHandover.emplace(key, 0);

    // Then write the outputs to the json
    payload["Target Primary Cell ID"] = decidedTargetCellId;
    m_decision_history.push_back({requestingRnti, requestedTargetCellId, decidedTargetCellId});
}

void
xAppHandoverMlpackKmeans::HandoverStarted(std::string context,
                                         uint64_t imsi,
                                         uint16_t cellid,
                                         uint16_t rnti,
                                         uint16_t targetCellId)
{
    // "source eNB RRC"에서 콜백이 오므로 cellid는 source cell로 보는게 자연스러움
    UeKey key = MakeUeKey(cellid, rnti);
    m_imsiInHandover[key] = imsi; // overwrite ok
}

void
xAppHandoverMlpackKmeans::HandoverSucceeded(std::string context,
                                           uint64_t imsi,
                                           uint16_t cellid,
                                           uint16_t rnti)
{
    NS_LOG_FUNCTION(this);
    std::cout << "yay" << std::endl; // reward predictor

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
xAppHandoverMlpackKmeans::HandoverFailed(std::string context,
                                        uint64_t imsi,
                                        uint16_t cellid,
                                        uint16_t rnti)
{
    NS_LOG_FUNCTION(this);
    std::cout << "nay" << std::endl; // punish predictor

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
xAppHandoverMlpackKmeans::ConnectionEstablished(std::string context,
                                               uint64_t imsi,
                                               uint16_t cellid,
                                               uint16_t rnti)
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

