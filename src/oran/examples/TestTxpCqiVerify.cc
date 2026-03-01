/**
 * TXP-CQI 검증 시나리오 (NS-3 간섭 테스트 구조 기반)
 *
 * 토폴로지:
 *   eNB1 (0, 0) ─── UE0 (300, 0) ─── eNB2 (500, 0) ─── UE1 (500, 10)
 *   서빙셀         관측 대상          이웃셀 (간섭원)    트래픽 생성용
 *
 * eNB2 TXP를 단계적으로 올리면서 UE0의 SINR/CQI 변화 관측
 */

#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"

#include <fstream>
#include <iomanip>

using namespace ns3;

static std::ofstream g_csv;
static double g_currentNeighborTxp = 20.0;
static Ptr<LteEnbNetDevice> g_neighborEnbDev;

static int SinrToCqi(double sinrDb)
{
    static const double thresholds[] = {
        -6.7, -4.7, -2.3, 0.2, 2.4, 4.3, 5.9, 8.1, 10.3, 11.7, 14.1, 16.3, 18.7, 21.0, 22.7
    };
    for (int i = 14; i >= 0; i--)
    {
        if (sinrDb >= thresholds[i])
            return i + 1;
    }
    return 0;
}

void
ReportRsrpSinr(std::string context, uint16_t cellId, uint16_t rnti,
                double rsrp, double sinr, uint8_t ccId)
{
    if (cellId != 1) return;

    double now = Simulator::Now().GetSeconds();
    double sinrDb = 10.0 * std::log10(std::max(sinr, 1e-10));
    int cqi = SinrToCqi(sinrDb);
    double rsrpDbm = 10.0 * std::log10(std::max(rsrp, 1e-30) * 1000.0);

    g_csv << std::fixed << std::setprecision(3)
          << now << "," << cellId << "," << rnti << ","
          << rsrpDbm << "," << sinrDb << "," << cqi << ","
          << g_currentNeighborTxp << std::endl;

    static double lastPrint = -1.0;
    if (now - lastPrint >= 1.0)
    {
        lastPrint = now;
        std::cout << "[t=" << std::setprecision(1) << now << "s]"
                  << " RSRP=" << std::setprecision(1) << rsrpDbm << "dBm"
                  << " SINR=" << std::setprecision(2) << sinrDb << "dB"
                  << " CQI=" << cqi
                  << " | eNB2 TXP=" << g_currentNeighborTxp << "dBm"
                  << std::endl;
    }
}

void
ChangeNeighborTxp(double newTxpDbm)
{
    g_currentNeighborTxp = newTxpDbm;
    g_neighborEnbDev->GetPhy()->SetTxPower(newTxpDbm);

    std::cout << "\n======== [TXP CHANGE] eNB2 TXP → " << newTxpDbm << " dBm"
              << " at t=" << Simulator::Now().GetSeconds() << "s ========\n" << std::endl;
}

int
main(int argc, char* argv[])
{
    double enb1Txp = 32.0;
    double enbDistance = 500.0;
    double ueRatio = 0.6;
    double simTime = 40.0;

    CommandLine cmd;
    cmd.AddValue("enb1Txp", "Serving cell TX power (dBm)", enb1Txp);
    cmd.AddValue("enbDistance", "Distance between eNBs (m)", enbDistance);
    cmd.AddValue("ueRatio", "UE position ratio from eNB1", ueRatio);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.Parse(argc, argv);

    double ueX = enbDistance * ueRatio;

    std::cout << "=== TXP-CQI Verification (Interference Test Style) ===" << std::endl;
    std::cout << "  eNB1: (0,0) TXP=" << enb1Txp << " dBm (fixed)" << std::endl;
    std::cout << "  eNB2: (" << enbDistance << ",0) TXP=20→38 dBm (varying)" << std::endl;
    std::cout << "  UE0:  (" << ueX << ",0) dist_eNB1=" << ueX << "m dist_eNB2=" << (enbDistance-ueX) << "m" << std::endl;
    std::cout << "  Handover: DISABLED" << std::endl;
    std::cout << "======================================================\n" << std::endl;

    g_csv.open("txp_cqi_verify.csv");
    g_csv << "time_s,cellId,rnti,rsrp_dBm,sinr_dB,cqi,neighbor_txp_dBm" << std::endl;

    // ── NS-3 간섭 테스트와 동일한 설정 ──
    Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(false));
    Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue(false));
    Config::SetDefault("ns3::LteAmc::AmcModel", EnumValue(LteAmc::PiroEW2010));
    Config::SetDefault("ns3::LteAmc::Ber", DoubleValue(0.00005));
    Config::SetDefault("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue(false));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    // ★ 핵심: FriisSpectrum + UsePdsch + NoEPC
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisSpectrumPropagationLossModel"));
    lteHelper->SetAttribute("UseIdealRrc", BooleanValue(false));
    lteHelper->SetAttribute("UsePdschForCqiGeneration", BooleanValue(true));

    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
    lteHelper->SetSchedulerAttribute("UlCqiFilter", EnumValue(FfMacScheduler::PUSCH_UL_CQI));

    // ── 노드 생성 ──
    NodeContainer enbNodes;
    enbNodes.Create(2);
    NodeContainer ueNodes1, ueNodes2;
    ueNodes1.Create(1);  // UE0: eNB1 소속 (관측 대상)
    ueNodes2.Create(1);  // UE1: eNB2 소속 (간섭 트래픽)
    NodeContainer allNodes = NodeContainer(enbNodes, ueNodes1, ueNodes2);

    // ── 위치 배치 ──
    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0.0, 0.0, 0.0));              // eNB1
    posAlloc->Add(Vector(enbDistance, 0.0, 0.0));       // eNB2
    posAlloc->Add(Vector(ueX, 0.0, 0.0));              // UE0: 3:2 지점
    posAlloc->Add(Vector(enbDistance, 10.0, 0.0));      // UE1: eNB2 바로 옆

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(posAlloc);
    mobility.Install(allNodes);

    // ── LTE 디바이스 ──
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(enb1Txp));
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);

    // eNB2 초기 TXP 설정
    g_neighborEnbDev = DynamicCast<LteEnbNetDevice>(enbDevs.Get(1));
    g_neighborEnbDev->GetPhy()->SetTxPower(20.0);

    NetDeviceContainer ueDevs1 = lteHelper->InstallUeDevice(ueNodes1);
    NetDeviceContainer ueDevs2 = lteHelper->InstallUeDevice(ueNodes2);

    // ── UE 연결 (EPC 없이) ──
    lteHelper->Attach(ueDevs1, enbDevs.Get(0));
    lteHelper->Attach(ueDevs2, enbDevs.Get(1));

    // ── 데이터 베어러 활성화 (EPC 없이) ──
    EpsBearer bearer(EpsBearer::GBR_CONV_VOICE);
    lteHelper->ActivateDataRadioBearer(ueDevs1, bearer);
    lteHelper->ActivateDataRadioBearer(ueDevs2, bearer);

    // ── SINR 트레이스 ──
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
                    MakeCallback(&ReportRsrpSinr));

    // ── TXP 단계적 변경 ──
    double txpValues[] = {20.0, 24.0, 28.0, 32.0, 36.0, 38.0};
    int numSteps = sizeof(txpValues) / sizeof(txpValues[0]);
    for (int i = 0; i < numSteps; i++)
    {
        Simulator::Schedule(Seconds(3.0 + i * 5.0), &ChangeNeighborTxp, txpValues[i]);
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    g_csv.close();
    std::cout << "\n=== Results saved to txp_cqi_verify.csv ===" << std::endl;

    Simulator::Destroy();
    return 0;
}
