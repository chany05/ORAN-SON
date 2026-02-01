#ifndef NS3_XAPP_HANDOVER_MLPACK_KMEANS_H
#define NS3_XAPP_HANDOVER_MLPACK_KMEANS_H

#include "ns3/xAppHandover.h"

#include <cstdint>   // <-- 추가 (uint32_t)
#include <deque>
#include <map>
#include <tuple>

/**
 * \ingroup oran
 * ns3::oran::xAppHandoverMlpackKmeans declaration.
 */

namespace ns3
{
namespace oran
{
/**
 * \ingroup oran
 * \class xAppHandoverMlpackKmeans
 * \brief An implementation of a Handover xApp using
 * MlPack's implementation of K-Means clustering
 */
class xAppHandoverMlpackKmeans : public xAppHandover
{
  public:
    xAppHandoverMlpackKmeans(bool kmeansEmptyPolicy = false,
                             float clusteringPeriodicitySec = 1.0,
                             bool initiateHandovers = false);

    void HandoverDecision(Json& payload);

    void HandoverSucceeded(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);

    void HandoverFailed(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);

    void HandoverStarted(std::string context,
                         uint64_t imsi,
                         uint16_t cellid,
                         uint16_t rnti,
                         uint16_t targetCellId);

    void ConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);

    void PeriodicClustering();

  private:
    // UE key = (servingCellId, rnti). servingCellId is 1-based here by convention.
    using UeKey = uint32_t;

    std::deque<std::tuple<uint16_t, uint16_t, uint16_t>>
        m_decision_history; ///< Keeps track of previous decisions

    std::map<UeKey, uint16_t>
        m_rntiToClusteredCellId; ///< Map of UEKey to their clustered cell ID

    std::map<UeKey, uint16_t>
        m_rntiToCurrentCellId; ///< Map of UEKey to their currently connected cell ID

    bool m_kmeansKeepEmptyPolicy;     ///< Flag indicates if the policy to allow empty clusters is enabled
    float m_clusteringPeriodicitySec; ///< Periodicity to re-cluster UEs
    bool m_initiateHandovers;         ///< Flag indicates if the xApp initiates the handovers

    std::map<UeKey, uint64_t>
        m_imsiInHandover; ///< Map of UEKey in handover and their IMSIs (0 means pending)
};
} // namespace oran
} // namespace ns3

#endif // NS3_XAPP_HANDOVER_MLPACK_KMEANS_H

