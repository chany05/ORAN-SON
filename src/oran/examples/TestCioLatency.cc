/**
 * CIO 반영 지연 측정 시나리오
 *
 * 3셀 토폴로지에서:
 * 1. t=5s: Cell1→Cell2 CIO를 +10 (큰 값)으로 설정
 * 2. 매 100ms마다 각 셀 UE 수를 로깅
 * 3. CIO 적용 후 UE 분포가 실제로 바뀌는 시점을 관찰
 *
 * 콘솔 출력으로 CIO 적용 → 핸드오버 완료까지 지연 확인
 */

#include "ns3/E2AP.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-ue-rrc.h>

#include <map>
#include <string>

NS_LOG_COMPONENT_DEFINE("TestCioLatency");

using namespace ns3;
using namespace oran;

// ─── 글로벌 상태 ───
static double g_cioApplyTime = -1.0;
static int g_cell1UesBefore = -1;
static bool g_firstHoAfterCio = true;

// ─── UE count per cell ───
void
LogUeDistribution(NodeContainer ueNodes, NetDeviceContainer ueLteDevs, uint16_t numUes)
{
    std::map<uint16_t, int> cellUeCount;
    for (uint16_t i = 0; i < numUes; i++)
    {
        auto ueDev = DynamicCast<LteUeNetDevice>(ueLteDevs.Get(i));
        if (ueDev && ueDev->GetRrc())
        {
            uint16_t cellId = ueDev->GetRrc()->GetCellId();
            cellUeCount[cellId]++;
        }
    }

    double now = Simulator::Now().GetSeconds();
    std::cout << "[UE-DIST] t=" << std::fixed << std::setprecision(1) << now << "s";
    for (int c = 1; c <= 3; c++)
    {
        std::cout << " | Cell" << c << "=" << cellUeCount[c];
    }

    if (g_cioApplyTime > 0)
    {
        double elapsed = now - g_cioApplyTime;
        std::cout << "  (CIO적용후 +" << std::setprecision(1) << elapsed << "s)";
    }
    std::cout << std::endl;
}

// ─── 핸드오버 콜백 ───
void
NotifyHandoverStartEnb(std::string context, uint64_t imsi, uint16_t cellid,
                        uint16_t rnti, uint16_t targetCellId)
{
    double now = Simulator::Now().GetSeconds();
    std::cout << "[HO-START] t=" << std::fixed << std::setprecision(3) << now
              << "s IMSI=" << imsi << " Cell" << cellid << "→Cell" << targetCellId;
    if (g_cioApplyTime > 0)
    {
        double delay = now - g_cioApplyTime;
        std::cout << "  (CIO적용후 +" << std::setprecision(3) << delay << "s)";
        if (g_firstHoAfterCio)
        {
            std::cout << " ★ FIRST HO AFTER CIO";
            g_firstHoAfterCio = false;
        }
    }
    std::cout << std::endl;
}

void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    double now = Simulator::Now().GetSeconds();
    std::cout << "[HO-OK] t=" << std::fixed << std::setprecision(3) << now
              << "s IMSI=" << imsi << " →Cell" << cellid;
    if (g_cioApplyTime > 0)
    {
        std::cout << "  (CIO적용후 +" << std::setprecision(3) << (now - g_cioApplyTime) << "s)";
    }
    std::cout << std::endl;
}

void
NotifyHandoverEndErrorUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    double now = Simulator::Now().GetSeconds();
    std::cout << "[HO-ERR] t=" << std::fixed << std::setprecision(3) << now
              << "s IMSI=" << imsi << " Cell" << cellid << std::endl;
}

// ─── CIO 적용 함수 ───
void
ApplyCio(Ptr<LteEnbRrc> rrc, uint16_t neighborCellId, int8_t cioValue)
{
    double now = Simulator::Now().GetSeconds();
    g_cioApplyTime = now;
    g_firstHoAfterCio = true;

    uint16_t myCellId = rrc->ComponentCarrierToCellId(0);
    std::cout << "\n========================================" << std::endl;
    std::cout << "[CIO-SET] t=" << std::fixed << std::setprecision(3) << now
              << "s Cell" << myCellId << "→Cell" << neighborCellId
              << " CIO=" << (int)cioValue << " (" << cioValue * 0.5 << "dB)" << std::endl;
    std::cout << "========================================\n" << std::endl;

    rrc->SetCellIndividualOffset(neighborCellId, cioValue);
    rrc->SendCioMeasConfigToAllUes();
}

int
main(int argc, char* argv[])
{
    double simTime = 30.0;
    double cioApplyAt = 5.0;
    int32_t cioValue = 6;      // +3dB (IE units, *0.5=dB)
    uint16_t numberOfUes = 30;
    uint32_t rngRun = 1;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("cioApplyAt", "Time to apply CIO (s)", cioApplyAt);
    cmd.AddValue("cioValue", "CIO value in IE units (x0.5=dB)", cioValue);
    cmd.AddValue("numUes", "Number of UEs", numberOfUes);
    cmd.AddValue("rngRun", "RNG run", rngRun);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(rngRun);
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    uint16_t numberOfEnbs = 3;

    std::cout << "=== CIO Latency Test ===" << std::endl;
    std::cout << "  UEs: " << numberOfUes << std::endl;
    std::cout << "  CIO apply at: " << cioApplyAt << "s" << std::endl;
    std::cout << "  CIO value: " << (int)cioValue << " IE (" << cioValue * 0.5 << "dB)" << std::endl;
    std::cout << "========================" << std::endl;

    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(20)));
    Config::SetDefault("ns3::UdpClient::PacketSize", UintegerValue(2048));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(0));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024));
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(80));
    Config::SetDefault("ns3::LteEnbRrc::EpsBearerToRlcMapping", EnumValue(LteEnbRrc::RLC_UM_ALWAYS));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkEnablePcap", BooleanValue(false));
    lteHelper->SetEpcHelper(epcHelper);

    lteHelper->SetSchedulerType("ns3::PfFfMacScheduler");
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisSpectrumPropagationLossModel"));
    lteHelper->SetSpectrumChannelType("ns3::MultiModelSpectrumChannel");
    lteHelper->SetEnbAntennaModelType("ns3::IsotropicAntennaModel");
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(25));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(25));

    // A3 핸드오버 (CIO가 영향을 미치는 알고리즘)
    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.0));
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(0)));

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Mb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // eNB 배치 (MASAC 시나리오와 동일)
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
    enbPositionAlloc->Add(Vector(250.0, 356.0, 0));
    enbPositionAlloc->Add(Vector(750.0, 356.0, 0));
    enbPositionAlloc->Add(Vector(500.0, 789.0, 0));
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    // UE: Cell1-Cell2 경계(x=500, y=356) 주변에 배치 (정지)
    // 경계 근처 UE가 CIO 변화에 민감하게 반응
    Ptr<ListPositionAllocator> ueAlloc = CreateObject<ListPositionAllocator>();
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    rng->SetAttribute("Min", DoubleValue(-1.0));
    rng->SetAttribute("Max", DoubleValue(1.0));
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        // Cell1(250,356)~Cell2(750,356) 사이 경계 근처 ±100m
        double x = 500.0 + rng->GetValue() * 100.0;
        double y = 356.0 + rng->GetValue() * 80.0;
        ueAlloc->Add(Vector(x, y, 0));
    }

    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator(ueAlloc);
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");  // 정지!
    ueMobility.Install(ueNodes);

    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(32.0));
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueLteDevs);

    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        lteHelper->AttachToClosestEnb(ueLteDevs.Get(i), enbLteDevs);
    }

    // 트래픽 설정
    uint16_t dlPort = 10000;
    uint16_t ulPort = 20000;
    for (uint32_t u = 0; u < numberOfUes; ++u)
    {
        Ptr<Node> ue = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

        ++dlPort;
        ++ulPort;
        ApplicationContainer clientApps, serverApps;

        UdpClientHelper dlClient(ueIpIfaces.GetAddress(u), dlPort);
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(1)));
        dlClient.SetAttribute("PacketSize", UintegerValue(115));
        clientApps.Add(dlClient.Install(remoteHost));
        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        serverApps.Add(dlSink.Install(ue));

        EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        tft->Add(dlpf);
        lteHelper->ActivateDedicatedEpsBearer(ueLteDevs.Get(u), bearer, tft);

        serverApps.Start(Seconds(0));
        clientApps.Start(Seconds(0));
    }

    lteHelper->AddX2Interface(enbNodes);

    // 핸드오버 콜백
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                    MakeCallback(&NotifyHandoverEndErrorUe));

    // UE 분포 로깅 (매 0.5초)
    for (double t = 1.0; t < simTime; t += 0.5)
    {
        Simulator::Schedule(Seconds(t), &LogUeDistribution, ueNodes, ueLteDevs, numberOfUes);
    }

    // ─── CIO 적용: Cell1→Cell2 +CIO, Cell2→Cell1 -CIO ───
    // Cell1 UE들이 Cell2를 더 선호하도록 (effective CIO = CIO_dst - CIO_src)
    auto enbDev1 = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(0));
    auto enbDev2 = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(1));
    Ptr<LteEnbRrc> rrc1 = enbDev1->GetRrc();
    Ptr<LteEnbRrc> rrc2 = enbDev2->GetRrc();
    // Cell1에서 Cell2 방향 CIO 양수 = Cell2를 선호
    Simulator::Schedule(Seconds(cioApplyAt), &ApplyCio, rrc1, (uint16_t)2, (int8_t)cioValue);
    // Cell2에서 Cell1 방향 CIO 음수 = Cell1을 비선호
    Simulator::Schedule(Seconds(cioApplyAt + 0.001), &ApplyCio, rrc2, (uint16_t)1, (int8_t)(-cioValue));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "\n=== Summary ===" << std::endl;
    if (g_cioApplyTime > 0)
    {
        std::cout << "CIO applied at: " << g_cioApplyTime << "s" << std::endl;
    }

    return 0;
}
