#ifndef NS3_XAPP_SON_H
#define NS3_XAPP_SON_H

#include "ns3/xAppHandover.h"

#include <cstdint>
#include <deque>
#include <map>
#include <tuple>

namespace ns3
{
namespace oran
{

struct UEContext
{
    uint16_t rnti;
    uint16_t servingCellId;
    double servingRsrp;
    double servingRsrq;
    int cqi;
    double throughputDl;
    double throughputUl;
    bool isEdge;
    std::map<uint16_t, double> neighborRsrq;  // targetCellId → RSRQ
};

struct CellContext
{
    uint16_t cellId;
    double avgCqi;
    double totalThroughputDl;
    double totalThroughputUl;
    uint32_t ueCount;
    uint32_t edgeUeCount;
    double loadScore;
};

class xAppHandoverSON : public xAppHandover
{
  public:
    xAppHandoverSON(float sonPeriodicitySec = 1.0,
            bool initiateHandovers = false);

    void HandoverDecision(Json& payload) override;

    // 콜백
    void HandoverSucceeded(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);
    void HandoverFailed(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);
    void HandoverStarted(std::string context, uint64_t imsi, uint16_t cellid,
                         uint16_t rnti, uint16_t targetCellId);
    void ConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);

    // 주기적 SON 체크
    void PeriodicSONCheck();

  private:
    using UeKey = uint32_t;

    static inline UeKey MakeUeKey(uint16_t servingCellId, uint16_t rnti)
    {
        return (static_cast<UeKey>(servingCellId) << 16) | static_cast<UeKey>(rnti);
    }

    static inline uint16_t KeyToServingCell(UeKey k)
    {
        return static_cast<uint16_t>((k >> 16) & 0xFFFF);
    }

    static inline uint16_t KeyToRnti(UeKey k)
    {
        return static_cast<uint16_t>(k & 0xFFFF);
    }

    // 데이터 수집
    void CollectKPMs();
    void CollectRsrpRsrq();
    void CollectCqi();
    void CollectThroughput();
    void CollectUeCount();

    // Edge UE 계산
    void CalculateEdgeUEs();
    double FriisDistanceEstimate(double rsrp_dBm, double txPower_dBm, double freq_Hz);

    // 부하 계산
    void CalculateLoadScores();
    bool IsCellOverloaded(uint16_t cellId);

    // 의사결정
    uint16_t MakeSONDecision(UeKey key);
    uint16_t FindLeastLoadedNeighbor(UeKey key);
    uint16_t FindBestRsrqCell(UeKey key);
    void CollectTargetRsrq();

    // 상태 저장
    std::map<UeKey, UEContext> m_ueContexts;
    std::map<uint16_t, CellContext> m_cellContexts;
    std::deque<std::tuple<uint16_t, uint16_t, uint16_t>> m_decision_history;
    std::map<UeKey, uint64_t> m_imsiInHandover;

    // 설정 파라미터
    float m_sonPeriodicitySec;
    bool m_initiateHandovers;
    double m_cellRadius;
    double m_edgeThreshold;
    double m_loadThreshold;
    double m_rsrqThreshold;
    double m_cqiThreshold;
    double m_txPower;
    double m_frequency;
};

} // namespace oran
} // namespace ns3

#endif // NS3_XAPP_SON_H