# O-RAN Multi-Agent RL Load Balancing Module

NS-3 기반 O-RAN xApp 모듈. 3셀 LTE 토폴로지에서 Multi-Agent RL(MASAC, MADDPG)을 이용하여 CIO(Cell Individual Offset)와 TxPower를 조정하고, 셀 간 부하를 분산합니다.

---

## Architecture

### Class Hierarchy

```
ns3::Application
  └── PubSubInfra            JSON pub/sub over NS-3 sockets
      ├── E2AP                E2SM-KPM (측정) + E2SM-RC (제어)
      └── xApp
          └── xAppHandover
              ├── xAppHandoverSON          MADDPG agent
              ├── xAppHandoverSON_MASAC    Multi-Agent SAC agent
              ├── xAppHandoverMaxRsrq      Heuristic baseline
              └── xAppHandoverMlpackKmeans K-means baseline (mlpack)
```

### Data Flow

```
eNB (KPM) ──E2AP──> xApp (RL Agent)
                        │
                  BuildObservation()
                        │
                  Actor Network
                        │
                  [CIO, TxPower]
                        │
                  E2SM-RC ──> eNB (적용)
```

---

## MASAC (Multi-Agent Soft Actor-Critic)

### Observation Space (per agent, 4D)

| Index | Feature | Normalization |
|-------|---------|--------------|
| 0 | AvgCqi | / 15.0 |
| 1 | DL Throughput | / 5 Mbps |
| 2 | Edge UE Ratio | edgeCount / ueCount |
| 3 | UE Ratio | cellUeCount / totalUeCount |

> **Note**: 현재 각 agent는 **자기 셀만** 관측합니다. 이웃 셀 부하를 모르므로 CIO 방향 결정에 한계가 있습니다. Centralized critic은 전체 관측(3x4=12D)을 보지만, actor는 로컬 4D만 입력받습니다. -> 이 부분을 어떻게 처리할지 결정이 필요해보임

### Action Space (per agent, 3D)

| Index | Action | Raw Range | Mapped Range |
|-------|--------|-----------|-------------|
| 0 | CIO → neighbor1 | [-1, 1] | round(raw × 5) → [-5, +5] dB |
| 1 | CIO → neighbor2 | [-1, 1] | round(raw × 5) → [-5, +5] dB |
| 2 | TxPower offset | [-1, 1] | base(32) + raw×4 → [26, 38] dBm |

### Network Architecture

**Actor (SACActorNet)** — Gaussian policy + tanh squashing
```
obs(4D) → RunningMeanStd(EMA α=0.01)
        → Linear(4, 64) + LeakyReLU
        → Linear(64, 64) + LeakyReLU
        ├→ fc_mean(64, 3)    → μ
        └→ fc_logstd(64, 3)  → log σ (clamped [-20, 2])

Stochastic:  a = tanh(μ + σ·ε),  ε ~ N(0,I)
Deterministic: a = tanh(μ)
```

**Twin Critic (TwinCriticNet)** — Centralized, global state + joint action
```
input = cat(all_obs(12D), all_acts(9D)) = 21D

Q1: Linear(21, 64) → LeakyReLU → Linear(64, 64) → LeakyReLU → Linear(64, 1)
Q2: (same structure, independent weights)

Output: min(Q1, Q2)  — overestimation 방지
```

### Reward Function

```
totalThp = Σ (smoothed_DL_Thp + smoothed_UL_Thp)  per cell
normalizedReward = totalThp / 20,000

UE_STD_PENALTY = 0.5
stdUeNorm = std(ueCount_per_cell) / totalUEs

reward = normalizedThp - UE_STD_PENALTY × stdUeNorm
```

모든 agent가 동일한 공유 보상을 받습니다 (cooperative).

### Training Algorithm

```
1. Collect transition → N-step buffer (N=3)
2. Compute n-step return: R = Σ_{k=0}^{N-1} γ^k · r_k
3. Push (obs_t, acts_t, R, obs_{t+N}, done) to replay buffer (100K)
4. Sample batch (64) from replay buffer
5. Per agent:
   a. Twin Critic update:
      targetQ = R + γ^3 · (1-done) · (min(TQ1,TQ2) - α·log_π)
      loss = MSE(Q1, targetQ) + MSE(Q2, targetQ)
   b. Actor update (entropy-regularized):
      loss = (α·log_π - min(Q1,Q2)).mean()
   c. Alpha (entropy temperature) update:
      loss = -(log_α · (log_π + H_target)).mean()
   d. Soft target update: θ_tgt ← τ·θ + (1-τ)·θ_tgt  (critic only)
```

### Hyperparameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| NUM_AGENTS | 3 | One per cell |
| BUFFER_SIZE | 100,000 | Replay buffer capacity |
| BATCH_SIZE | 64 | Training batch |
| GAMMA | 0.99 | Discount factor |
| TAU | 0.005 | Soft update rate |
| ACTOR_LR | 3e-4 | Actor learning rate |
| CRITIC_LR | 3e-4 | Critic learning rate |
| N_STEP | 3 | N-step returns |
| EMA_ALPHA | 0.3 | Cell metrics smoothing |
| H_target | -1.5 | Target entropy (-actDim/2) |
| Initial α | ~0.135 | exp(-2.0) |
| SON Period | 1.0s | Decision interval |
| Train Freq | Every 3 steps | = 3.0s |

### Exploration

SAC는 stochastic policy 자체가 exploration:
- **학습**: reparameterized Gaussian + tanh (자동 exploration)
- **추론**: deterministic tanh(μ) (exploitation only)
- **Entropy temperature α**: 자동 조절 — H_target = -1.5에 수렴

---

## MADDPG (Multi-Agent Deep Deterministic Policy Gradient)

### Actor, vs MASAC 차이

| 항목 | MASAC | MADDPG |
|------|-------|--------|
| Policy | Gaussian + tanh (stochastic) | Deterministic tanh |
| Hidden | 64-64 | 256-256 |
| Critic | Twin Q (min 사용) | Single Q |
| Target Network | Critic only | Actor + Critic |
| Exploration | Entropy regularization (자동) | ε-greedy + Gaussian noise |
| Actor Regularization | Entropy term | L2 (λ=0.01) |
| Gradient Clip | Actor 1.0, Critic 0.5 | Both 0.5 |

### MADDPG Exploration

```
if random() < ε:     → uniform random action
else:                → Actor(obs) + N(0, 0.1)

ε: 1.0 → 1e-15 (decay 0.9999/step ≈ 100K steps)
```

---

## Test Scenarios

### TestSONXappLB_MASAC

MASAC 학습/추론 시나리오. 3셀 LTE 토폴로지에서 30 UE 부하 분산.

```bash
# 기본 학습 (256초)
./ns3 run "TestSONXappLB_MASAC"

# 추론 모드
./ns3 run "TestSONXappLB_MASAC --inferenceOnly=true --loadPretrained=true"

# Cell1 과부하 시나리오
./ns3 run "TestSONXappLB_MASAC --saturate=true"

# 짧은 테스트
./ns3 run "TestSONXappLB_MASAC --simTime=30 --numUes=30"
```

**CMD Arguments**:

| Argument | Default | Description |
|----------|---------|-------------|
| `--simTime` | 256 | Simulation time (s) |
| `--numUes` | 30 | Total UE count |
| `--rngRun` | 42 | RNG seed (42 = time-based) |
| `--inferenceOnly` | false | 학습 없이 추론만 |
| `--loadPretrained` | false | 저장된 모델 로드 |
| `--saturate` | false | Gaussian 집중 배치 (σ=185m) |

**Configuration**:
- PRB: 50 (10 MHz)
- Traffic: 512B/1ms = 4.1 Mbps/UE
- Handover: A3-RSRP (Hysteresis=0, TTT=0)
- UE mobility: 3 m/s RandomDirection
- UE 배치: Normal → 균등 랜덤(100~900), Saturate → Gaussian(σ=185m)

**CSV Outputs**:
- `cell_metrics.csv` — 셀별 KPM (CQI, THP, PRB, UE count)
- `cio_actions.csv` — CIO 결정 이력
- `maddpg_actions.csv` — 에이전트 action 상세 (raw + mapped + alpha)
- `reward_curve.csv` — 보상 곡선
- `ue_trajectory.csv` — UE 위치/셀 이력

---

### TestSONXappLB_MADDPG

MADDPG 학습/추론 시나리오. MASAC과 동일 토폴로지.

```bash
# 기본 학습
./ns3 run "TestSONXappLB_MADDPG"

# No-RL 베이스라인 (CIO=0, TxP=32 고정)
./ns3 run "TestSONXappLB_MADDPG --baseline=true"
```

**CMD Arguments**:

| Argument | Default | Description |
|----------|---------|-------------|
| `--simTime` | 256 | Simulation time (s) |
| `--numUes` | 30 | Total UE count |
| `--rngRun` | 42 | RNG seed |
| `--inferenceOnly` | false | 추론 전용 |
| `--loadPretrained` | false | 모델 로드 |
| `--saturate` | false | Cell1 집중 (σ=150m) |
| `--baseline` | false | No-RL 베이스라인 |

**Configuration**:
- PRB: 25 (5 MHz)
- Traffic: 160B/1ms = 1.28 Mbps/UE
- 나머지 MASAC과 동일

---

### TestThpSaturation

RL 없이 순수 셀 THP 포화 측정. 셀별 UE 수를 지정하여 포화 곡선 생성.

```bash
# 셀당 5 UE (균등)
./ns3 run "TestThpSaturation --uesCell1=5 --uesCell2=5 --uesCell3=5"

# Cell1 과부하
./ns3 run "TestThpSaturation --uesCell1=20 --uesCell2=5 --uesCell3=5"

# 패킷 크기 변경 (포화 조절)
./ns3 run "TestThpSaturation --uesCell1=10 --uesCell2=10 --uesCell3=10 --pktSize=512"

# 균등 증가 sweep
for N in 3 5 8 10 13 15; do
  ./ns3 run "TestThpSaturation --uesCell1=$N --uesCell2=$N --uesCell3=$N --pktSize=512"
done
```

**CMD Arguments**:

| Argument | Default | Description |
|----------|---------|-------------|
| `--uesCell1` | 5 | Cell 1 UE 수 |
| `--uesCell2` | 5 | Cell 2 UE 수 |
| `--uesCell3` | 5 | Cell 3 UE 수 |
| `--simTime` | 5 | Simulation time (s) |
| `--rngRun` | 1 | RNG seed |
| `--txPower` | 32 | eNB TX power (dBm) |
| `--pktSize` | 160 | UDP packet size (bytes) |
| `--pktInterval` | 1 | UDP packet interval (ms) |

**Configuration**:
- PRB: 50 (MASAC과 동일)
- Handover: **NoOp** (순수 THP 측정)
- UE 배치: 각 eNB 중심 Gaussian(σ=80m), 반경 150m 클램프
- UE mobility: 3 m/s RandomDirection
- THP 측정: 2초 워밍업 후 0.5초 간격

**CSV Output**: `thp_saturation_3cell_new.csv`
- Columns: `uesCell1, uesCell2, uesCell3, totalUes, cell{N}_thp, cell{N}_ues, totalThp, perUeThp`

**Knee Point 분석 결과** (30 UE, c1+c2+c3=30):

| pktSize | Per-UE Load | Knee Point (per cell) | 최적 분배 | 최악 분배 | Gap |
|---------|-------------|----------------------|----------|----------|-----|
| 160B | 1.28 Mbps | 미포화 (15 UE에서도 증가) | 10/8/12 | 14/15/1 | 23% |
| 512B | 4.1 Mbps | ~5 UEs/cell | 10/10/10 | 14/15/1 | 25% |
| 1024B | 8.2 Mbps | ~3 UEs/cell | 10/10/10 | 14/15/1 | 35% |

512B에서 균등 분배(10/10/10)가 최적 — MASAC 학습 환경과 일치.

---

## 3-Cell Topology

```
         Cell1 (250, 356)
            ╲           ╱
             ╲   500m  ╱
              ╲       ╱
               ╲     ╱
      Cell2 (750, 356)
                 ╲
          500m    ╲  500m
                   ╲
            Cell3 (500, 789)

Simulation area: 50~950 × 50~950 m
```

---

## Build

```bash
# LibTorch required at $HOME/libs/libtorch
./ns3 configure --enable-examples
./ns3 build oran

# Run
./ns3 run TestSONXappLB_MASAC
./ns3 run TestThpSaturation
```

## Dependencies

- NS-3 (v3-dev) with modules: core, network, internet, lte, mobility, applications
- LibTorch (PyTorch C++ API) at `$HOME/libs/libtorch`
- C++17 compiler
- (Optional) mlpack + armadillo for K-means baseline

