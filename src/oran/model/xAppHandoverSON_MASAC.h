#ifndef NS3_XAPP_SON_MASAC_H
#define NS3_XAPP_SON_MASAC_H
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
#include <utility>

namespace ns3
{
namespace oran
{

// Reuse shared types from original header
struct UEContext_MASAC {
    uint16_t rnti = 0;
    uint16_t servingCellId = 0;
    double servingRsrp = 0;
    double servingRsrq = 0;
    int cqi = 0;
    bool isEdge = false;
    std::map<uint16_t, double> neighborRsrq;
};

struct CellContext_MASAC
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

// ─── MASAC 컴포넌트 ────────────

// ─── RunningMeanStd: EMA 기반 observation 정규화 ─────────────
struct RunningMeanStd_MASAC : torch::nn::Module
{
    RunningMeanStd_MASAC(int64_t dim, double alpha = 0.01)
        : m_alpha(alpha)
    {
        m_mean = register_buffer("running_mean", torch::zeros({dim}));
        m_var  = register_buffer("running_var",  torch::ones({dim}));
        m_count = register_buffer("count", torch::zeros({1}));
    }

    torch::Tensor normalize(torch::Tensor x)
    {
        // Update running stats only for batched inputs shaped like [B, feat].
        // Single observations arrive as [feat] during action selection.
        if (is_training() && x.dim() >= 2 && x.size(0) > 1)
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

// SACActorNet: Gaussian policy with tanh squashing + obs normalization
struct SACActorNetImpl : torch::nn::Module
{
    SACActorNetImpl(int64_t obsDim, int64_t actDim)
    {
        obsNorm = register_module("obs_norm", std::make_shared<RunningMeanStd_MASAC>(obsDim));
        fc1 = register_module("fc1", torch::nn::Linear(obsDim, 64));
        fc2 = register_module("fc2", torch::nn::Linear(64, 64));
        fc_mean   = register_module("fc_mean",   torch::nn::Linear(64, actDim));
        fc_logstd = register_module("fc_logstd", torch::nn::Linear(64, actDim));

        // He initialization for hidden layers
        torch::nn::init::kaiming_uniform_(fc1->weight, std::sqrt(5));
        torch::nn::init::zeros_(fc1->bias);
        torch::nn::init::kaiming_uniform_(fc2->weight, std::sqrt(5));
        torch::nn::init::zeros_(fc2->bias);
        // Output heads: small init so μ starts near 0 (avoid tanh saturation)
        torch::nn::init::uniform_(fc_mean->weight, -0.3, 0.3);
        torch::nn::init::uniform_(fc_mean->bias, -0.08, 0.08);
        // log_std bias = -1.0 → initial std ≈ 0.37 (moderate exploration, avoid tanh saturation)
        torch::nn::init::uniform_(fc_logstd->weight, -3e-3, 3e-3);
        torch::nn::init::uniform_(fc_logstd->bias, -4e-3, 4e-3);
    }

    // Stochastic: reparameterization trick a = tanh(μ + σ*ε)
    std::pair<torch::Tensor, torch::Tensor> sample(torch::Tensor obs)
    {
        auto x = obsNorm->normalize(obs);
        x = torch::leaky_relu(fc1->forward(x));
        x = torch::leaky_relu(fc2->forward(x));
        auto mu = fc_mean->forward(x);
        auto log_std = torch::clamp(fc_logstd->forward(x), -20.0, 2.0);
        auto std = torch::exp(log_std);

        auto eps = torch::randn_like(std);
        auto u = mu + std * eps;       // pre-squashing
        auto action = torch::tanh(u);  // squashed action

        // log_prob with Jacobian correction for tanh squashing
        auto log_prob = -0.5 * (std::log(2.0 * M_PI) + 2.0 * log_std + eps * eps)
                       - torch::log(1.0 - action * action + 1e-3);
        log_prob = log_prob.sum(-1, /*keepdim=*/true);

        return {action, log_prob};
    }

    // Deterministic: tanh(μ)
    torch::Tensor deterministic(torch::Tensor obs)
    {
        auto x = obsNorm->normalize(obs);
        x = torch::leaky_relu(fc1->forward(x));
        x = torch::leaky_relu(fc2->forward(x));
        return torch::tanh(fc_mean->forward(x));
    }

    torch::Tensor forward(torch::Tensor obs) { return deterministic(obs); }

    std::shared_ptr<RunningMeanStd_MASAC> obsNorm{nullptr};
    torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc_mean{nullptr}, fc_logstd{nullptr};
};
TORCH_MODULE(SACActorNet);

// TwinCriticNet: Q1, Q2 독립 네트워크 (각 128-128-1)
// min(Q1,Q2)로 overestimation 방지
struct TwinCriticNetImpl : torch::nn::Module
{
    TwinCriticNetImpl(int64_t totalStateDim, int64_t totalActDim)
    {
        q1_fc1 = register_module("q1_fc1", torch::nn::Linear(totalStateDim + totalActDim, 64));
        q1_fc2 = register_module("q1_fc2", torch::nn::Linear(64, 64));
        q1_fc3 = register_module("q1_fc3", torch::nn::Linear(64, 1));

        q2_fc1 = register_module("q2_fc1", torch::nn::Linear(totalStateDim + totalActDim, 64));
        q2_fc2 = register_module("q2_fc2", torch::nn::Linear(64, 64));
        q2_fc3 = register_module("q2_fc3", torch::nn::Linear(64, 1));

        // He initialization
        for (auto* layer : {&q1_fc1, &q1_fc2, &q2_fc1, &q2_fc2}) {
            torch::nn::init::kaiming_uniform_((*layer)->weight, std::sqrt(5));
            torch::nn::init::zeros_((*layer)->bias);
        }
        // Small init for output layers
        for (auto* layer : {&q1_fc3, &q2_fc3}) {
            torch::nn::init::uniform_((*layer)->weight, -3e-3, 3e-3);
            torch::nn::init::uniform_((*layer)->bias, -3e-3, 3e-3);
        }
    }

    std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor state, torch::Tensor action)
    {
        auto x = torch::cat({state, action}, 1);

        auto q1 = torch::leaky_relu(q1_fc1->forward(x));
        q1 = torch::leaky_relu(q1_fc2->forward(q1));
        q1 = q1_fc3->forward(q1);

        auto q2 = torch::leaky_relu(q2_fc1->forward(x));
        q2 = torch::leaky_relu(q2_fc2->forward(q2));
        q2 = q2_fc3->forward(q2);

        return {q1, q2};
    }

    torch::nn::Linear q1_fc1{nullptr}, q1_fc2{nullptr}, q1_fc3{nullptr};
    torch::nn::Linear q2_fc1{nullptr}, q2_fc2{nullptr}, q2_fc3{nullptr};
};
TORCH_MODULE(TwinCriticNet);

// Experience (same as original)
struct Experience_MASAC
{
    std::vector<torch::Tensor> obs;
    std::vector<torch::Tensor> acts;
    std::vector<double> rewards;
    std::vector<torch::Tensor> nextObs;
    bool done;
};

// Replay Buffer (same as original)
class ReplayBuffer_MASAC
{
  public:
    ReplayBuffer_MASAC(size_t capacity) : m_capacity(capacity), m_rng(std::random_device{}()) {}

    void Push(Experience_MASAC exp)
    {
        if (m_buffer.size() >= m_capacity) m_buffer.pop_front();
        m_buffer.push_back(std::move(exp));
    }

    std::vector<Experience_MASAC> Sample(size_t batchSize)
    {
        std::vector<size_t> indices(m_buffer.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), m_rng);
        size_t n = std::min(batchSize, m_buffer.size());
        std::vector<Experience_MASAC> batch;
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
            Experience_MASAC exp;
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
    std::deque<Experience_MASAC> m_buffer;
    std::mt19937 m_rng;
};

// AgentConfig (same as original)
struct AgentConfig_MASAC
{
    uint16_t cellId;
    int64_t obsDim;
    int64_t actDim;
    std::vector<uint16_t> neighborCellIds;
};

// MASACAgent: no target actor (SAC 특성), target critic만 유지
class MASACAgent
{
  public:
    MASACAgent(const AgentConfig_MASAC& config,
               int64_t totalObsDim, int64_t totalActDim,
               double actorLr = 7e-5, double criticLr = 1e-4)
        : m_config(config),
          m_targetEntropy(-1.0)  // -actDim/2 for 2D action space
    {
        m_actor = SACActorNet(config.obsDim, config.actDim);
        m_critic = TwinCriticNet(totalObsDim, totalActDim);
        // No target actor (SAC characteristic)
        m_targetCritic = TwinCriticNet(totalObsDim, totalActDim);

        HardCopyParams(m_critic, m_targetCritic);

        m_actorOpt = std::make_shared<torch::optim::Adam>(
            m_actor->parameters(), torch::optim::AdamOptions(actorLr));
        m_criticOpt = std::make_shared<torch::optim::Adam>(
            m_critic->parameters(), torch::optim::AdamOptions(criticLr));

        // Entropy temperature α (start very small to avoid μ→0 collapse)
        m_logAlpha = torch::full({1}, -2.0, torch::requires_grad(true));
        m_alphaOpt = std::make_shared<torch::optim::Adam>(
            std::vector<torch::Tensor>{m_logAlpha}, torch::optim::AdamOptions(3e-4));
    }

    torch::Tensor SelectAction(torch::Tensor obs, bool deterministic = true)
    {
        torch::NoGradGuard noGrad;
        if (deterministic) {
            const bool wasTraining = m_actor->is_training();
            m_actor->eval();
            auto act = m_actor->deterministic(obs);
            if (wasTraining)
            {
                m_actor->train();
            }
            else
            {
                m_actor->eval();
            }
            return act;
        }
        auto [action, log_prob] = m_actor->sample(obs);
        return action;
    }

    void SoftUpdateTargets(double tau)
    {
        // Only target critic (no target actor in SAC)
        SoftUpdate(m_critic, m_targetCritic, tau);
    }

    SACActorNet& GetActor()              { return m_actor; }
    TwinCriticNet& GetCritic()           { return m_critic; }
    TwinCriticNet& GetTargetCritic()     { return m_targetCritic; }
    torch::optim::Adam& GetActorOpt()    { return *m_actorOpt; }
    torch::optim::Adam& GetCriticOpt()   { return *m_criticOpt; }
    const AgentConfig_MASAC& GetConfig() const { return m_config; }
    torch::Tensor& GetLogAlpha()         { return m_logAlpha; }
    torch::optim::Adam& GetAlphaOpt()    { return *m_alphaOpt; }
    double GetTargetEntropy() const      { return m_targetEntropy; }

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

    AgentConfig_MASAC m_config;
    double m_targetEntropy;
    SACActorNet m_actor{nullptr};
    TwinCriticNet m_critic{nullptr}, m_targetCritic{nullptr};
    std::shared_ptr<torch::optim::Adam> m_actorOpt, m_criticOpt;
    torch::Tensor m_logAlpha;
    std::shared_ptr<torch::optim::Adam> m_alphaOpt;
};

// ─── xApp 본체 (MASAC variant) ─────────────────────────

class xAppHandoverSON_MASAC : public xAppHandover
{
  public:
    xAppHandoverSON_MASAC(float sonPeriodicitySec = 1.0,
            bool initiateHandovers = false,
            bool loadPretrained = false,
            bool inferenceOnly = false,
            double simStopTime = 256.0);
    void HandoverDecision(Json& payload) override;

    // 콜백
    void HandoverSucceeded(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);
    void HandoverFailed(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);
    void HandoverStarted(std::string context, uint64_t imsi, uint16_t cellid,
                         uint16_t rnti, uint16_t targetCellId);
    void ConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti);

    void PeriodicSONCheck();

    void SaveModels(const std::string& dir = "masac_models");
    void LoadModels(const std::string& dir = "masac_models");

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
    std::map<UeKey, UEContext_MASAC> m_ueContexts;
    std::map<uint16_t, CellContext_MASAC> m_cellContexts;
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
    std::vector<std::string> m_trainDiagBuf;
public:
    void FlushCsvLogs();
private:

    // ── MASAC ──────────────
    bool m_useMADDPG = true;
    bool m_loadPretrained = false;
    bool m_inferenceOnly = false;

    static constexpr int    NUM_AGENTS   = 3;
    static constexpr int    OBS_DIM      = 4;
    static constexpr double MAX_ACTION   = 1.0;
    static constexpr size_t BUFFER_SIZE  = 100000;
    static constexpr size_t BATCH_SIZE   = 64;
    static constexpr double GAMMA        = 0.99;
    static constexpr double TAU_SOFT     = 0.005;
    static constexpr double ACTOR_LR     = 3e-4;
    static constexpr double CRITIC_LR    = 3e-4;
    // 에이전트 + 버퍼
    std::vector<std::unique_ptr<MASACAgent>> m_agents;
    std::unique_ptr<ReplayBuffer_MASAC> m_replayBuffer;

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

    // Self CIO: m_selfCio[cellId] = CIO dB (동일 값이 모든 이웃에 적용)
    std::map<uint16_t, int> m_selfCio;

    // MASAC 함수
    void InitMASAC();
    torch::Tensor BuildObservation(uint16_t cellId);
    std::vector<double> ComputeRewards();
    void StepMASAC();
    void TrainMASAC();

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

#endif // NS3_XAPP_SON_MASAC_H
