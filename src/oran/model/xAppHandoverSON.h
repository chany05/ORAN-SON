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
    double txPower;
};

// ─── MADDPG 컴포넌트 ────────────────────────────────────

// Actor: 관측 → 행동 [-1,1] (deterministic policy)
// 왜 필요한가: 각 에이전트(eNB)의 로컬 관측만으로 CIO 행동을 출력.
// tanh 출력 → MAX_CIO_DB로 스케일링하여 실제 dB 값으로 변환.
struct ActorNetImpl : torch::nn::Module
{
    ActorNetImpl(int64_t obsDim, int64_t actDim, int64_t hidden = 64)
    {
        fc1 = register_module("fc1", torch::nn::Linear(obsDim, hidden));
        fc2 = register_module("fc2", torch::nn::Linear(hidden, hidden));
        fc3 = register_module("fc3", torch::nn::Linear(hidden, actDim));
        // 출력층 가중치를 작게 → 초기 행동이 0 근처 (CIO ≈ 0dB로 시작)
        torch::nn::init::uniform_(fc3->weight, -3e-3, 3e-3);
        torch::nn::init::uniform_(fc3->bias, -3e-3, 3e-3);
    }

    torch::Tensor forward(torch::Tensor obs)
    {
        auto x = torch::relu(fc1->forward(obs));
        x = torch::relu(fc2->forward(x));
        return torch::tanh(fc3->forward(x)); // [-1, 1]
    }

    torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
};
TORCH_MODULE(ActorNet);

// Critic: 전체 관측 + 전체 행동 → Q-value (스칼라)
// 왜 필요한가: CTDE(Centralized Training, Decentralized Execution)의 핵심.
// 학습 시에만 모든 에이전트의 정보를 보고 Q값을 추정하여
// 각 Actor의 gradient를 계산하는 데 사용.
// 실행 시에는 Actor만 사용하므로 통신 불필요.
struct CriticNetImpl : torch::nn::Module
{
    CriticNetImpl(int64_t totalObsDim, int64_t totalActDim, int64_t hidden = 64)
    {
        fc1 = register_module("fc1", torch::nn::Linear(totalObsDim + totalActDim, hidden));
        fc2 = register_module("fc2", torch::nn::Linear(hidden, hidden));
        fc3 = register_module("fc3", torch::nn::Linear(hidden, 1));
        torch::nn::init::uniform_(fc3->weight, -3e-3, 3e-3);
        torch::nn::init::uniform_(fc3->bias, -3e-3, 3e-3);
    }

    torch::Tensor forward(torch::Tensor allObs, torch::Tensor allActs)
    {
        auto x = torch::cat({allObs, allActs}, /*dim=*/1);
        x = torch::relu(fc1->forward(x));
        x = torch::relu(fc2->forward(x));
        return fc3->forward(x); // (batch, 1)
    }

    torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
};
TORCH_MODULE(CriticNet);

// OU Noise — Ornstein-Uhlenbeck 프로세스
// 왜 필요한가: DDPG 계열은 결정론적 정책이므로 탐색(exploration)을 위해
// 시간 상관성 있는 노이즈를 행동에 더함.
// dx = θ(μ - x)dt + σ√dt · N(0,1)
// θ(theta): 평균 회귀 속도, σ(sigma): 노이즈 크기
class OUNoise
{
  public:
    OUNoise(int dim, double mu = 0.0, double theta = 0.15,
            double sigma = 0.2, double dt = 1.0)
        : m_dim(dim), m_mu(mu), m_theta(theta), m_sigma(sigma), m_dt(dt),
          m_rng(std::random_device{}()), m_dist(0.0, 1.0)
    {
        m_state.assign(dim, mu);
    }

    std::vector<double> Sample()
    {
        for (int i = 0; i < m_dim; i++)
        {
            double dx = m_theta * (m_mu - m_state[i]) * m_dt
                      + m_sigma * std::sqrt(m_dt) * m_dist(m_rng);
            m_state[i] += dx;
        }
        return m_state;
    }

    void Reset() { m_state.assign(m_dim, m_mu); }
    void SetSigma(double s) { m_sigma = s; }

  private:
    int m_dim;
    double m_mu, m_theta, m_sigma, m_dt;
    std::vector<double> m_state;
    std::mt19937 m_rng;
    std::normal_distribution<double> m_dist;
};

// Experience — replay buffer에 저장되는 하나의 전이(transition)
// (obs_all, act_all, reward, nextObs_all, done)
struct Experience
{
    std::vector<torch::Tensor> obs;     // [nAgents] 각 (obsDim,)
    std::vector<torch::Tensor> acts;    // [nAgents] 각 (actDim,)
    double reward;
    std::vector<torch::Tensor> nextObs; // [nAgents] 각 (obsDim,)
    bool done;
};

// Replay Buffer — off-policy 학습의 핵심
// 왜 필요한가: 과거 경험을 저장 → 랜덤 샘플링으로 시간적 상관성 제거 → 학습 안정화
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

// MADDPGAgent — 에이전트 하나(eNB 하나)에 대응
// Actor(로컬 정책) + Critic(중앙 Q함수) + 각각의 Target 네트워크
class MADDPGAgent
{
  public:
    MADDPGAgent(const AgentConfig& config,
                int64_t totalObsDim, int64_t totalActDim,
                double actorLr = 1e-4, double criticLr = 1e-3)
        : m_config(config),
          m_noise(config.actDim, 0.0, 0.15, 0.2, 1.0)
    {
        m_actor = ActorNet(config.obsDim, config.actDim);
        m_critic = CriticNet(totalObsDim, totalActDim);
        m_targetActor = ActorNet(config.obsDim, config.actDim);
        m_targetCritic = CriticNet(totalObsDim, totalActDim);

        // Hard copy: Target ← Online (초기 동기화)
        HardCopyParams(m_actor, m_targetActor);
        HardCopyParams(m_critic, m_targetCritic);

        m_actorOpt = std::make_shared<torch::optim::Adam>(
            m_actor->parameters(), torch::optim::AdamOptions(actorLr));
        m_criticOpt = std::make_shared<torch::optim::Adam>(
            m_critic->parameters(), torch::optim::AdamOptions(criticLr));
    }

    torch::Tensor SelectAction(torch::Tensor obs, bool explore)
    {
        torch::NoGradGuard noGrad;
        auto action = m_actor->forward(obs);
        if (explore)
        {
            auto noise = m_noise.Sample();
            auto nt = torch::from_blob(noise.data(),
                {(long)noise.size()}, torch::kFloat64).to(torch::kFloat32);
            action = torch::clamp(action + nt, -1.0, 1.0);
        }
        return action;
    }

    // Soft update: θ' ← τθ + (1-τ)θ'
    // 왜 필요한가: Target을 천천히 추종시켜 TD 타겟의 급변 방지 → 학습 안정화
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
    OUNoise& GetNoise() { return m_noise; }

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
    ActorNet m_actor{nullptr}, m_targetActor{nullptr};
    CriticNet m_critic{nullptr}, m_targetCritic{nullptr};
    std::shared_ptr<torch::optim::Adam> m_actorOpt, m_criticOpt;
    OUNoise m_noise;
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
    void SaveModels(const std::string& dir = "maddpg_models");
    void LoadModels(const std::string& dir = "maddpg_models");

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

    void PurgeStaleUeContexts();

    // Edge UE 계산
    void CalculateEdgeUEs();
    double FriisDistanceEstimate(double rsrp_dBm, double txPower_dBm, double freq_Hz, uint16_t rnti, uint16_t cellId);

    // 부하 계산
    void CalculateLoadScores();
    bool IsCellOverloaded(uint16_t cellId);

    // 의사결정
    uint16_t MakeSONDecision(UeKey key);
    uint16_t FindLeastLoadedNeighbor(UeKey key);
    uint16_t FindBestRsrqCell(UeKey key);
    void CollectTargetRsrq();
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
    double m_cellRadius;
    double m_edgeThreshold;
    double m_loadThreshold;
    double m_rsrqThreshold;
    double m_cqiThreshold;
    double m_txPower;
    double m_frequency;
    uint16_t m_dlBandwidthPrb;  ///< DL bandwidth in PRBs   

    // ── MADDPG ──────────────────────────────────────────
    // 모드 전환: true면 MADDPG가 CIO 결정, false면 기존 규칙 기반
    bool m_useMADDPG = true;

    // 하이퍼파라미터
    static constexpr int    NUM_AGENTS   = 3;
    static constexpr int    OBS_DIM      = 8;   // 자기 셀 4 + 이웃 2셀 × 2
    static constexpr int    ACT_DIM      = 2;   // 이웃 2셀에 대한 CIO
    static constexpr double MAX_CIO_DB   = 6.0; // CIO 최대 ±6 dB
    static constexpr size_t BUFFER_SIZE  = 100000;
    static constexpr size_t BATCH_SIZE   = 64;
    static constexpr size_t WARMUP_STEPS = 200;  // 학습 시작 전 최소 경험
    static constexpr double GAMMA        = 0.95;
    static constexpr double TAU_SOFT     = 0.01;
    static constexpr double ACTOR_LR     = 1e-4;
    static constexpr double CRITIC_LR    = 1e-3;
    static constexpr double REWARD_ALPHA = 0.5;  // min(thp) 가중치
    static constexpr double REWARD_BETA  = 0.5;  // avg(thp) 가중치

    // 에이전트 + 버퍼
    std::vector<std::unique_ptr<MADDPGAgent>> m_agents;
    std::unique_ptr<ReplayBuffer> m_replayBuffer;

    // 이전 스텝 저장 (experience 구성용)
    std::vector<torch::Tensor> m_prevObs;
    std::vector<torch::Tensor> m_prevActs;
    bool m_hasPrevStep = false;
    uint64_t m_stepCount = 0;

    // 셀 토폴로지
    std::vector<uint16_t> m_cellIds;
    std::map<uint16_t, std::vector<uint16_t>> m_neighborMap;
    uint32_t m_totalUEs = 40;

    // MADDPG 전용 함수
    void InitMADDPG();
    torch::Tensor BuildObservation(uint16_t cellId);
    double ComputeReward();
    void StepMADDPG();
    void TrainMADDPG();
    bool m_loadPretrained = true;  // true면 기존 모델 로드 후 학습 계속
    bool m_inferenceOnly = false;  // true면 학습 안 하고 Actor만 실행.
};

} // namespace oran
} // namespace ns3

#endif // NS3_XAPP_SON_H