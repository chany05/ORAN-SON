#ifndef NS3_XAPP_SON_H
#define NS3_XAPP_SON_H
#include "ns3/xAppHandover.h"
#include <torch/torch.h>
#include <cstdint>
#include <deque>
#include <map>
#include <tuple>
#include <set>
#include <vector>
#include <random>
#include <memory>
#include <numeric>

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
    std::map<uint16_t, double> neighborRsrq;
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
    double txPower;
};

// ─── MADDPG 컴포넌트 (FineBalancer 논문 사양) ────────────

// Actor: state → action [-max_action, +max_action]
// 논문: 256-256-actDim, tanh × max_action
struct ActorNetImpl : torch::nn::Module
{
    ActorNetImpl(int64_t obsDim, int64_t actDim, double maxAction = 6.4999)
        : m_maxAction(maxAction)
    {
        fc1 = register_module("fc1", torch::nn::Linear(obsDim, 256));
        fc2 = register_module("fc2", torch::nn::Linear(256, 256));
        fc3 = register_module("fc3", torch::nn::Linear(256, actDim));
    }

    torch::Tensor forward(torch::Tensor obs)
    {
        auto x = torch::relu(fc1->forward(obs));
        x = torch::relu(fc2->forward(x));
        return m_maxAction * torch::tanh(fc3->forward(x));
    }

    torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
    double m_maxAction;
};
TORCH_MODULE(ActorNet);

// Critic: 전체 에이전트 state + action → Q-value
// 논문: (totalStateDim + totalActDim) → 256-256-1
struct CriticNetImpl : torch::nn::Module
{
    CriticNetImpl(int64_t totalStateDim, int64_t totalActDim)
    {
        fc1 = register_module("fc1", torch::nn::Linear(totalStateDim + totalActDim, 256));
        fc2 = register_module("fc2", torch::nn::Linear(256, 256));
        fc3 = register_module("fc3", torch::nn::Linear(256, 1));
    }

    torch::Tensor forward(torch::Tensor state, torch::Tensor action)
    {
        auto x = torch::cat({state, action}, 1);
        x = torch::relu(fc1->forward(x));
        x = torch::relu(fc2->forward(x));
        return fc3->forward(x);
    }

    torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
};
TORCH_MODULE(CriticNet);

// Experience
struct Experience
{
    std::vector<torch::Tensor> obs;      // [nAgents] 각 (obsDim,)
    std::vector<torch::Tensor> acts;     // [nAgents] 각 (actDim,)
    std::vector<double> rewards;         // [nAgents] per-agent 보상
    std::vector<torch::Tensor> nextObs;  // [nAgents] 각 (obsDim,)
    bool done;
};

// Replay Buffer
class ReplayBuffer
{
  public:
    ReplayBuffer(size_t capacity) : m_capacity(capacity), m_rng(std::random_device{}()) {}

    void Push(Experience exp)
    {
        if (m_buffer.size() >= m_capacity) m_buffer.pop_front();
        m_buffer.push_back(std::move(exp));
    }

    std::vector<Experience> Sample(size_t batchSize)
    {
        std::vector<size_t> indices(m_buffer.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), m_rng);
        size_t n = std::min(batchSize, m_buffer.size());
        std::vector<Experience> batch;
        for (size_t i = 0; i < n; i++) batch.push_back(m_buffer[indices[i]]);
        return batch;
    }

    size_t Size() const { return m_buffer.size(); }

    // ★ 추가: 버퍼 저장
    void Save(const std::string& filename)
    {
        std::vector<torch::Tensor> all_data;
        for (const auto& exp : m_buffer) {
            // 모든 텐서를 일렬로 붙여서 저장
            for (auto& o : exp.obs) all_data.push_back(o);
            for (auto& a : exp.acts) all_data.push_back(a);
            for (auto& no : exp.nextObs) all_data.push_back(no);
            // double형 reward도 Tensor로 변환해서 저장
            for (auto r : exp.rewards) all_data.push_back(torch::tensor(r));
        }
        torch::save(all_data, filename);
    }

    // ★ 추가: 버퍼 불러오기
    void Load(const std::string& filename, int obsDim, int actDim, int nAgents)
    {
        std::vector<torch::Tensor> all_data;
        torch::load(all_data, filename);

        m_buffer.clear();
        int elements_per_exp = nAgents * 4; // obs, acts, nextObs, rewards 각각 nAgents개
        
        for (size_t i = 0; i < all_data.size(); i += elements_per_exp) {
            Experience exp;
            for (int j = 0; j < nAgents; j++) exp.obs.push_back(all_data[i + j]);
            for (int j = 0; j < nAgents; j++) exp.acts.push_back(all_data[i + nAgents + j]);
            for (int j = 0; j < nAgents; j++) exp.nextObs.push_back(all_data[i + 2 * nAgents + j]);
            for (int j = 0; j < nAgents; j++) exp.rewards.push_back(all_data[i + 3 * nAgents + j].item<double>());
            exp.done = false;
            m_buffer.push_back(exp);
        }
    }
    
  private:
    size_t m_capacity;
    std::deque<Experience> m_buffer;
    std::mt19937 m_rng;
};

// 에이전트별 설정
struct AgentConfig
{
    uint16_t cellId;
    int64_t obsDim;
    int64_t actDim;
    std::vector<uint16_t> neighborCellIds;
};

// MADDPGAgent
class MADDPGAgent
{
  public:
    MADDPGAgent(const AgentConfig& config,
                int64_t totalObsDim, int64_t totalActDim,
                double lr = 3e-4, double maxAction = 6.4999)
        : m_config(config), m_maxAction(maxAction)
    {
        m_actor = ActorNet(config.obsDim, config.actDim, maxAction);
        m_critic = CriticNet(totalObsDim, totalActDim);
        m_targetActor = ActorNet(config.obsDim, config.actDim, maxAction);
        m_targetCritic = CriticNet(totalObsDim, totalActDim);

        HardCopyParams(m_actor, m_targetActor);
        HardCopyParams(m_critic, m_targetCritic);

        m_actorOpt = std::make_shared<torch::optim::Adam>(
            m_actor->parameters(), torch::optim::AdamOptions(lr));
        m_criticOpt = std::make_shared<torch::optim::Adam>(
            m_critic->parameters(), torch::optim::AdamOptions(lr));
    }

    torch::Tensor SelectAction(torch::Tensor obs)
    {
        torch::NoGradGuard noGrad;
        return m_actor->forward(obs);
    }

    void SoftUpdateTargets(double tau)
    {
        SoftUpdate(m_actor, m_targetActor, tau);
        SoftUpdate(m_critic, m_targetCritic, tau);
    }

    ActorNet& GetActor()         { return m_actor; }
    ActorNet& GetTargetActor()   { return m_targetActor; }
    CriticNet& GetCritic()       { return m_critic; }
    CriticNet& GetTargetCritic() { return m_targetCritic; }
    torch::optim::Adam& GetActorOpt()  { return *m_actorOpt; }
    torch::optim::Adam& GetCriticOpt() { return *m_criticOpt; }
    const AgentConfig& GetConfig() const { return m_config; }
    double GetMaxAction() const { return m_maxAction; }

  private:
    template <typename M>
    void HardCopyParams(M& src, M& tgt)
    {
        torch::NoGradGuard noGrad;
        auto sp = src->named_parameters();
        auto tp = tgt->named_parameters();
        for (auto& p : sp) tp[p.key()].copy_(p.value());
    }

    template <typename M>
    void SoftUpdate(M& src, M& tgt, double tau)
    {
        torch::NoGradGuard noGrad;
        auto sp = src->named_parameters();
        auto tp = tgt->named_parameters();
        for (auto& p : sp)
            tp[p.key()].copy_(tau * p.value() + (1.0 - tau) * tp[p.key()]);
    }

    AgentConfig m_config;
    double m_maxAction;
    ActorNet m_actor{nullptr}, m_targetActor{nullptr};
    CriticNet m_critic{nullptr}, m_targetCritic{nullptr};
    std::shared_ptr<torch::optim::Adam> m_actorOpt, m_criticOpt;
};

// ─── xApp 본체 ─────────────────────────────────────────

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

    // MADDPG 모델 저장/로드
    void SaveModels(const std::string& dir = "maddpg_models");
    void LoadModels(const std::string& dir = "maddpg_models");

    void LogCioAction(uint16_t srcCell, uint16_t neighborCell, int cioDB, int cioIE);
    void LogMaddpgAction(uint16_t cellId, float cioAction, float txpAction,
                        double txpApplied, double reward, double epsilon);
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
    void CollectCellKpms();
    void CollectCellThroughput();   // ★ 셀 throughput 수집 (DL+UL)
    void PurgeStaleUeContexts();
    void CollectTargetRsrq();

    // Edge UE 계산
    void CalculateEdgeUEs();
    double FriisDistanceEstimate(double rsrp_dBm, double txPower_dBm, double freq_Hz, uint16_t rnti, uint16_t cellId);

    // 부하 계산 (규칙 기반 — m_useMADDPG=false일 때)
    void CalculateLoadScores();
    bool IsCellOverloaded(uint16_t cellId);

    // 의사결정 (규칙 기반)
    uint16_t MakeSONDecision(UeKey key);
    uint16_t FindLeastLoadedNeighbor(UeKey key);
    uint16_t FindBestRsrqCell(UeKey key);

    void ApplyCioActions(const std::vector<double>& cioActions,
                          const std::vector<uint16_t>& cellIds,
                          const std::vector<std::string>& enbEndpoints);

    // 상태 저장
    std::map<UeKey, UEContext> m_ueContexts;
    std::map<uint16_t, CellContext> m_cellContexts;
    std::deque<std::tuple<uint16_t, uint16_t, uint16_t>> m_decision_history;
    std::map<UeKey, uint64_t> m_imsiInHandover;
    std::set<UeKey> m_staleKeys;

    // 설정 파라미터
    float m_sonPeriodicitySec;
    bool m_initiateHandovers;
    //double m_cellRadius;
    //double m_edgeThreshold;
    double m_loadThreshold;
    double m_rsrqThreshold;
    double m_cqiThreshold;
    double m_txPower;
    double m_frequency;
    double m_edgeRsrpThreshold;
    uint16_t m_dlBandwidthPrb;

    // ── MADDPG (FineBalancer 논문 사양) ──────────────
    bool m_useMADDPG = true;
    bool m_loadPretrained = false;
    bool m_inferenceOnly = false;

    static constexpr int    NUM_AGENTS   = 3;
    static constexpr int    OBS_DIM      = 4;       // [AvgCqi, Thp, FarUes, ServedUes]
    static constexpr int    ACT_DIM      = 2;       // [CIO, TXP]
    static constexpr double MAX_ACTION   = 6.4999;
    static constexpr size_t BUFFER_SIZE  = 100000;
    static constexpr size_t BATCH_SIZE   = 256;
    static constexpr double GAMMA        = 0.99;
    static constexpr double TAU_SOFT     = 0.005;
    static constexpr double LR           = 3e-4;

    // 탐색
    double m_epsilon = 1.0;
    static constexpr double EPSILON_END   = 1e-15;
    static constexpr double EPSILON_DECAY = 0.9999;
    static constexpr double EXPL_NOISE    = 0.1;

    // 에이전트 + 버퍼
    std::vector<std::unique_ptr<MADDPGAgent>> m_agents;
    std::unique_ptr<ReplayBuffer> m_replayBuffer;

    // 이전 스텝 저장
    std::vector<torch::Tensor> m_prevObs;
    std::vector<torch::Tensor> m_prevActs;
    bool m_hasPrevStep = false;
    uint64_t m_stepCount = 0;

    // 셀 토폴로지
    std::vector<uint16_t> m_cellIds;
    std::map<uint16_t, std::vector<uint16_t>> m_neighborMap;
    uint32_t m_totalUEs = 40;

    // MADDPG 함수
    void InitMADDPG();
    torch::Tensor BuildObservation(uint16_t cellId);
    std::vector<double> ComputeRewards();
    void StepMADDPG();
    void TrainMADDPG();

    //csv logging
    std::ofstream m_cellMetricsCsv;
    std::ofstream m_cioActionsCsv;
    std::ofstream m_maddpgActionsCsv;

    void InitCsvLoggers();
    void LogCellMetrics();

};

} // namespace oran
} // namespace ns3

#endif // NS3_XAPP_SON_H