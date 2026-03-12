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

struct UEContext {
    uint16_t rnti = 0;
    uint16_t servingCellId = 0;
    double servingRsrp = 0;
    double servingRsrq = 0;
    int cqi = 0;
    bool isEdge = false;
    std::map<uint16_t, double> neighborRsrq;
};

struct CellContext
{
    uint16_t cellId;
    double avgCqi;
    double totalThroughputDl;
    double totalThroughputUl;
    double prbUtilDl = 0.0;
    uint32_t ueCount;
    uint32_t edgeUeCount;
    double loadScore;
    double txPower;
};

// ─── RunningMeanStd: EMA 기반 observation 정규화 ─────────────
struct RunningMeanStd : torch::nn::Module
{
    RunningMeanStd(int64_t dim, double alpha = 0.01)
        : m_alpha(alpha)
    {
        // register_buffer: 학습 파라미터가 아님, 저장/로드는 됨
        m_mean = register_buffer("running_mean", torch::zeros({dim}));
        m_var  = register_buffer("running_var",  torch::ones({dim}));
        m_count = register_buffer("count", torch::zeros({1}));
    }

    torch::Tensor normalize(torch::Tensor x)
    {
        if (is_training() && x.size(0) > 1)
        {
            auto batchMean = x.mean(0);
            auto batchVar  = x.var(0, /*unbiased=*/false);
            m_mean = (1.0 - m_alpha) * m_mean + m_alpha * batchMean.detach();
            m_var  = (1.0 - m_alpha) * m_var  + m_alpha * batchVar.detach();
            m_count += 1;
        }
        return (x - m_mean) / (m_var.sqrt() + 1e-8);
    }

    double m_alpha;
    torch::Tensor m_mean, m_var, m_count;
};

// ─── MADDPG + Tanh Actor (RunningMeanStd + LeakyReLU) ────────────
// Actor: obs → RunningMeanStd → LeakyReLU → LeakyReLU → tanh → action ∈ [-1, +1]
struct ActorNetImpl : torch::nn::Module
{
    ActorNetImpl(int64_t obsDim, int64_t actDim, double /*maxAction*/ = 1.0)
    {
        obsNorm = register_module("obs_norm", std::make_shared<RunningMeanStd>(obsDim));
        fc1 = register_module("fc1", torch::nn::Linear(obsDim, 256));
        fc2 = register_module("fc2", torch::nn::Linear(256, 256));
        fc3 = register_module("fc3", torch::nn::Linear(256, actDim));

        // He initialization for hidden layers
        torch::nn::init::kaiming_uniform_(fc1->weight, std::sqrt(5));
        torch::nn::init::zeros_(fc1->bias);
        torch::nn::init::kaiming_uniform_(fc2->weight, std::sqrt(5));
        torch::nn::init::zeros_(fc2->bias);
        // Larger init for output layer so actions start non-trivial
        torch::nn::init::uniform_(fc3->weight, -0.3, 0.3);
        torch::nn::init::uniform_(fc3->bias, -0.1, 0.1);
    }

    torch::Tensor forward(torch::Tensor obs)
    {
        auto x = obsNorm->normalize(obs);
        x = torch::leaky_relu(fc1->forward(x), 0.01);
        x = torch::leaky_relu(fc2->forward(x), 0.01);
        return torch::tanh(fc3->forward(x));
    }

    std::shared_ptr<RunningMeanStd> obsNorm{nullptr};
    torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
};
TORCH_MODULE(ActorNet);

// Critic: 전체 에이전트 state + action → Q-value (256-256-1, Python과 동일)
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
        x = torch::leaky_relu(fc1->forward(x));
        x = torch::leaky_relu(fc2->forward(x));
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

    void Save(const std::string& filename)
    {
        std::vector<torch::Tensor> all_data;
        for (const auto& exp : m_buffer) {
            for (auto& o : exp.obs) all_data.push_back(o);
            for (auto& a : exp.acts) all_data.push_back(a);
            for (auto& no : exp.nextObs) all_data.push_back(no);
            for (auto r : exp.rewards) all_data.push_back(torch::tensor(r));
        }
        torch::save(all_data, filename);
    }

    void Load(const std::string& filename, int obsDim, int actDim, int nAgents)
    {
        std::vector<torch::Tensor> all_data;
        torch::load(all_data, filename);
        m_buffer.clear();
        int elements_per_exp = nAgents * 4;
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

// MADDPGAgent (Tanh + BatchNorm)
class MADDPGAgent
{
  public:
    MADDPGAgent(const AgentConfig& config,
                int64_t totalObsDim, int64_t totalActDim,
                double actorLr = 3e-4, double criticLr = 3e-4, double maxAction = 1.0)
        : m_config(config), m_maxAction(maxAction)
    {
        m_actor = ActorNet(config.obsDim, config.actDim, maxAction);
        m_critic = CriticNet(totalObsDim, totalActDim);
        m_targetActor = ActorNet(config.obsDim, config.actDim, maxAction);
        m_targetCritic = CriticNet(totalObsDim, totalActDim);

        HardCopyParams(m_actor, m_targetActor);
        HardCopyParams(m_critic, m_targetCritic);

        m_actorOpt = std::make_shared<torch::optim::Adam>(
            m_actor->parameters(), torch::optim::AdamOptions(actorLr));
        m_criticOpt = std::make_shared<torch::optim::Adam>(
            m_critic->parameters(), torch::optim::AdamOptions(criticLr));
    }

    torch::Tensor SelectAction(torch::Tensor obs, bool deterministic = true)
    {
        torch::NoGradGuard noGrad;
        m_actor->eval();  // BN eval mode for inference
        auto input = obs.dim() == 1 ? obs.unsqueeze(0) : obs;  // (4,) → (1,4)
        auto act = m_actor->forward(input);
        m_actor->train(); // back to train mode
        return act.squeeze(0);  // (1,2) → (2,)
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
        // Copy BN running stats (buffers)
        auto sb = src->named_buffers();
        auto tb = tgt->named_buffers();
        for (auto& b : sb) tb[b.key()].copy_(b.value());
    }

    template <typename M>
    void SoftUpdate(M& src, M& tgt, double tau)
    {
        torch::NoGradGuard noGrad;
        auto sp = src->named_parameters();
        auto tp = tgt->named_parameters();
        for (auto& p : sp)
            tp[p.key()].copy_(tau * p.value() + (1.0 - tau) * tp[p.key()]);
        // Copy BN running stats (buffers) directly
        auto sb = src->named_buffers();
        auto tb = tgt->named_buffers();
        for (auto& b : sb) tb[b.key()].copy_(b.value());
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
    xAppHandoverSON(float sonPeriodicitySec = 0.5,
            bool initiateHandovers = false,
            bool loadPretrained = false,
            bool inferenceOnly = false,
            double simStopTime = 256.0,
            bool baseline = false);
    void HandoverDecision(Json& payload) override;

    // 콜백
    void HandoverSucceeded(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);
    void HandoverFailed(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);
    void HandoverStarted(std::string context, uint64_t imsi, uint16_t cellid,
                         uint16_t rnti, uint16_t targetCellId);
    void ConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);

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
    void CollectUeCount();
    void CollectCellKpms();
    void CollectCellThroughput();
    void PurgeStaleUeContexts();
    void CollectTargetRsrq();

    void CalculateEdgeUEs();
    void CalculateLoadScores();
    bool IsCellOverloaded(uint16_t cellId);

    uint16_t MakeSONDecision(UeKey key);
    uint16_t FindLeastLoadedNeighbor(UeKey key);
    uint16_t FindBestRsrqCell(UeKey key);

    // 상태 저장
    std::map<UeKey, UEContext> m_ueContexts;
    std::map<uint16_t, CellContext> m_cellContexts;
    std::deque<std::tuple<uint16_t, uint16_t, uint16_t>> m_decision_history;
    std::map<UeKey, uint64_t> m_imsiInHandover;
    std::set<UeKey> m_staleKeys;

    // 설정 파라미터
    float m_sonPeriodicitySec;
    bool m_initiateHandovers;
    double m_loadThreshold;
    double m_rsrqThreshold;
    double m_cqiThreshold;
    double m_txPower;
    double m_frequency;
    double m_edgeRsrpThreshold;
    std::map<uint16_t, double> m_lastThroughputDl;
    std::map<uint16_t, double> m_lastThroughputUl;
    uint16_t m_dlBandwidthPrb;
    std::map<uint16_t, double> m_prevCellDlBytes;
    std::map<uint16_t, double> m_prevCellUlBytes;
    std::ofstream m_rewardCurveCsv;

    // ── CSV buffering ──
    std::vector<std::string> m_cellMetricsBuf;
    std::vector<std::string> m_cioActionsBuf;
    std::vector<std::string> m_maddpgActionsBuf;
    std::vector<std::string> m_rewardCurveBuf;
    std::vector<std::string> m_stagnationBuf;
    std::vector<std::string> m_trainDiagBuf;   // gradient/weight/activation diagnostics
public:
    void FlushCsvLogs();
private:

    // ── MADDPG ──────────────
    bool m_useMADDPG = true;
    bool m_loadPretrained = false;
    bool m_inferenceOnly = false;
    bool m_baseline = false;  // true: CIO=0, TXP=32 고정 (no MADDPG action)

    static constexpr int    NUM_AGENTS   = 3;
    static constexpr int    OBS_DIM      = 4;       // [AvgCqi/15, Thp/5Mbps, EdgeRatio, UeRatio]
    static constexpr int    ACT_DIM      = 2;       // [selfCIO, TXP]
    static constexpr double MAX_ACTION   = 1.0;
    static constexpr size_t BUFFER_SIZE  = 100000;   // Python: 1e6
    static constexpr size_t BATCH_SIZE   = 64;
    static constexpr double GAMMA        = 0.99;
    static constexpr double TAU_SOFT     = 0.005;
    static constexpr double ACTOR_LR     = 3e-4;    // Python과 동일
    static constexpr double CRITIC_LR    = 3e-4;

    // Epsilon-greedy exploration (Python MADDPG와 동일)
    double m_epsilon        = 1.0;
    double m_epsilonEnd     = 1e-15;
    double m_epsilonDecay   = 0.9999;
    double m_explNoise      = 0.1;    // Gaussian noise scale

    // 에이전트 + 버퍼
    std::vector<std::unique_ptr<MADDPGAgent>> m_agents;
    std::unique_ptr<ReplayBuffer> m_replayBuffer;

    // 이전 스텝 저장
    std::vector<torch::Tensor> m_prevObs;
    std::vector<torch::Tensor> m_prevActs;
    bool m_hasPrevStep = false;
    uint64_t m_stepCount = 0;
    double m_simStopTime = 256.0;

    // 셀 토폴로지
    std::vector<uint16_t> m_cellIds;
    std::map<uint16_t, std::vector<uint16_t>> m_neighborMap;
    uint32_t m_totalUEs = 40;

    // EMA smoothing for noisy cell metrics
    static constexpr double EMA_ALPHA = 0.3;
    struct SmoothedCellMetrics {
        double ueCount = 0.0;
        double edgeUeCount = 0.0;
        double dlThp = 0.0;
        double ulThp = 0.0;
        bool initialized = false;
    };
    std::map<uint16_t, SmoothedCellMetrics> m_smoothed;

    // Self CIO: m_selfCio[cellId] = CIO dB (동일 값이 모든 이웃에 적용)
    std::map<uint16_t, int> m_selfCio;

    // Random engine for epsilon-greedy
    std::mt19937 m_rng{std::random_device{}()};
    std::uniform_real_distribution<double> m_uniformDist{0.0, 1.0};
    std::normal_distribution<double> m_normalDist{0.0, 1.0};

    // MADDPG 함수
    void InitMADDPG();
    torch::Tensor BuildObservation(uint16_t cellId);
    std::vector<double> ComputeRewards();
    void StepMADDPG();
    void TrainMADDPG();

    // csv logging
    std::ofstream m_cellMetricsCsv;
    std::ofstream m_cioActionsCsv;
    std::ofstream m_maddpgActionsCsv;

    void InitCsvLoggers();
    void LogCellMetrics();

    // 고착화 체크
    std::ofstream m_stagnationCsv;
    std::map<uint16_t, std::vector<float>> m_prevRawActions;
    std::map<uint16_t, uint32_t> m_prevUeCounts;
    std::vector<double> m_prevRewards;
    uint32_t m_stagnantSteps = 0;
    static constexpr double STAGNATION_ACTION_THRESH = 0.02;
    static constexpr double STAGNATION_REWARD_THRESH = 0.01;
    void CheckAndLogStagnation(const std::vector<torch::Tensor>& currentActs,
                               const std::vector<double>& rewards);
};

} // namespace oran
} // namespace ns3

#endif // NS3_XAPP_SON_H
