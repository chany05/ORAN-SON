/**
 * THP 포화 테스트 — MASAC 학습 시나리오와 동일 조건
 *
 * 3셀 토폴로지, 동일 트래픽/이동성, 핸드오버 없음 (NoOp)
 * UE 수만 변경하며 총 셀 THP 측정
 */

#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/config-store-module.h"

#include <fstream>
#include <iomanip>
#include <numeric>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TestThpSaturation");

// Per-cell PacketSink 기반 DL throughput
struct CellSinks
{
    ApplicationContainer sinks;
    uint64_t prevBytes = 0;
    std::vector<double> thpSamples;
};

static std::map<uint16_t, CellSinks> g_cellSinks;

void
MeasureThp(double interval)
{
    double now = Simulator::Now().GetSeconds();
    double totalThp = 0;

    std::cout << "[THP] t=" << std::fixed << std::setprecision(1) << now << "s";

    for (auto& [cellId, cs] : g_cellSinks)
    {
        uint64_t totalBytes = 0;
        for (uint32_t i = 0; i < cs.sinks.GetN(); i++)
        {
            auto sink = DynamicCast<PacketSink>(cs.sinks.Get(i));
            if (sink) totalBytes += sink->GetTotalRx();
        }

        if (cs.prevBytes > 0)
        {
            double thpKbps = (totalBytes - cs.prevBytes) * 8.0 / (interval * 1000.0);
            cs.thpSamples.push_back(thpKbps);
            totalThp += thpKbps;
            std::cout << "  Cell" << cellId << "=" << std::setprecision(0) << thpKbps;
        }
        cs.prevBytes = totalBytes;
    }

    if (totalThp > 0)
        std::cout << "  total=" << std::setprecision(0) << totalThp << " kbps";
    std::cout << std::endl;

    Simulator::Schedule(Seconds(interval), &MeasureThp, interval);
}

int
main(int argc, char* argv[])
{
    uint16_t numUes = 10;
    double simTime = 20.0;
    uint32_t rngRun = 1;
    double enbTxPower = 32.0;

    CommandLine cmd;
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("rngRun", "RNG run", rngRun);
    cmd.AddValue("txPower", "eNB TX power (dBm)", enbTxPower);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(rngRun);

    uint16_t numberOfEnbs = 3;

    std::cout << "=== THP Saturation Test (3-cell, MASAC-identical) ===" << std::endl;
    std::cout << "  eNBs: " << numberOfEnbs << "  UEs: " << numUes
              << "  TxPower: " << enbTxPower << "dBm" << std::endl;

    // ─── MASAC 시나리오와 100% 동일한 설정 ───
    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(20)));
    Config::SetDefault("ns3::UdpClient::PacketSize", UintegerValue(2048));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(0));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024));
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(80));
    Config::SetDefault("ns3::LteEnbRrc::EpsBearerToRlcMapping",
                       EnumValue(LteEnbRrc::RLC_UM_ALWAYS));
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(enbTxPower));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkEnablePcap", BooleanValue(false));
    lteHelper->SetEpcHelper(epcHelper);

    lteHelper->SetSchedulerType("ns3::PfFfMacScheduler");
    lteHelper->SetAttribute("PathlossModel",
                           StringValue("ns3::FriisSpectrumPropagationLossModel"));
    lteHelper->SetSpectrumChannelType("ns3::MultiModelSpectrumChannel");
    lteHelper->SetEnbAntennaModelType("ns3::IsotropicAntennaModel");
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(25));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(25));

    // 핸드오버 없음 — 순수 THP 포화만 측정
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");

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

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                               Ipv4Mask("255.0.0.0"), 1);

    // ─── 3셀 토폴로지 (MASAC과 동일) ───
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator>();
    enbPos->Add(Vector(250.0, 356.0, 0));
    enbPos->Add(Vector(750.0, 356.0, 0));
    enbPos->Add(Vector(500.0, 789.0, 0));
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPos);
    enbMobility.Install(enbNodes);

    // ─── UE: 100~900m 랜덤 배치, 1m/s 이동 (MASAC과 동일) ───
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    Ptr<RandomRectanglePositionAllocator> ueAllocator =
        CreateObject<RandomRectanglePositionAllocator>();
    Ptr<UniformRandomVariable> xPos = CreateObject<UniformRandomVariable>();
    xPos->SetAttribute("Min", DoubleValue(100.0));
    xPos->SetAttribute("Max", DoubleValue(900.0));
    ueAllocator->SetX(xPos);
    Ptr<UniformRandomVariable> yPos = CreateObject<UniformRandomVariable>();
    yPos->SetAttribute("Min", DoubleValue(100.0));
    yPos->SetAttribute("Max", DoubleValue(900.0));
    ueAllocator->SetY(yPos);

    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator(ueAllocator);
    ueMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(50, 950, 50, 950)),
        "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.1]"));
    ueMobility.Install(ueNodes);

    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);

    // AttachToClosestEnb (MASAC과 동일)
    for (uint16_t i = 0; i < numUes; i++)
    {
        lteHelper->AttachToClosestEnb(ueDevs.Get(i), enbDevs);
    }

    lteHelper->AddX2Interface(enbNodes);

    // ─── DL + UL 트래픽 (MASAC과 동일) ───
    uint16_t dlPort = 10000;
    uint16_t ulPort = 20000;
    // 초기 UE→셀 매핑을 기록하여 per-cell sink 그룹 생성
    // (NoOp HO이므로 이동해도 셀 변경 없음)
    for (uint32_t u = 0; u < numUes; ++u)
    {
        Ptr<Node> ue = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

        ++dlPort;
        ++ulPort;

        // DL
        UdpClientHelper dlClient(ueIpIfaces.GetAddress(u), dlPort);
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(20)));
        dlClient.SetAttribute("PacketSize", UintegerValue(2048));
        ApplicationContainer dlClientApps = dlClient.Install(remoteHost);

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        ApplicationContainer dlSinkApps = dlSink.Install(ue);

        // UL
        Ipv4Address remoteAddr = internetIpIfaces.GetAddress(1);
        UdpClientHelper ulClient(remoteAddr, ulPort);
        ulClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        ulClient.SetAttribute("Interval", TimeValue(MilliSeconds(20)));
        ulClient.SetAttribute("PacketSize", UintegerValue(2048));
        ApplicationContainer ulClientApps = ulClient.Install(ue);

        PacketSinkHelper ulSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        ApplicationContainer ulSinkApps = ulSink.Install(remoteHost);

        EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        tft->Add(dlpf);
        EpcTft::PacketFilter ulpf;
        ulpf.remotePortStart = ulPort;
        ulpf.remotePortEnd = ulPort;
        tft->Add(ulpf);
        lteHelper->ActivateDedicatedEpsBearer(ueDevs.Get(u), bearer, tft);

        dlClientApps.Start(Seconds(0));
        ulClientApps.Start(Seconds(0));
        dlSinkApps.Start(Seconds(0));
        ulSinkApps.Start(Seconds(0));

        // DL sink만 per-cell 그룹에 추가 (THP 측정용)
        // 셀 ID는 가장 가까운 eNB 기준
        auto ueDev = DynamicCast<LteUeNetDevice>(ueDevs.Get(u));
        uint16_t cellId = ueDev->GetRrc()->GetCellId();
        g_cellSinks[cellId].sinks.Add(dlSinkApps);
    }

    // UE 분포 출력
    std::cout << "Initial UE distribution:";
    for (auto& [cellId, cs] : g_cellSinks)
        std::cout << "  Cell" << cellId << "=" << cs.sinks.GetN();
    std::cout << std::endl;

    // THP 측정 시작 (3초 워밍업 후, 1초 간격)
    Simulator::Schedule(Seconds(3.0), &MeasureThp, 1.0);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // ─── 결과 요약 ───
    std::cout << "\n=== RESULT ===" << std::endl;
    double totalAvgThp = 0;

    for (auto& [cellId, cs] : g_cellSinks)
    {
        if (cs.thpSamples.size() < 3) continue;
        // 첫 2개 제외
        double sum = 0;
        int cnt = 0;
        for (size_t i = 2; i < cs.thpSamples.size(); i++)
        {
            sum += cs.thpSamples[i];
            cnt++;
        }
        double avg = cnt > 0 ? sum / cnt : 0;
        totalAvgThp += avg;
        std::cout << "  Cell" << cellId << ": " << std::fixed << std::setprecision(0)
                  << avg << " kbps (" << cs.sinks.GetN() << " UEs, "
                  << std::setprecision(0) << avg / std::max(1u, cs.sinks.GetN())
                  << " kbps/UE)" << std::endl;
    }

    std::cout << "  TOTAL: " << std::setprecision(0) << totalAvgThp << " kbps"
              << "  (" << numUes << " UEs, "
              << std::setprecision(0) << totalAvgThp / numUes << " kbps/UE)" << std::endl;

    // CSV
    std::ofstream csv("thp_saturation_3cell_new.csv", std::ios::app);
    if (csv.tellp() == 0)
    {
        csv << "numUes";
        for (auto& [cellId, cs] : g_cellSinks)
            csv << ",cell" << cellId << "_thp,cell" << cellId << "_ues";
        csv << ",totalThp,perUeThp" << std::endl;
    }
    csv << numUes;
    for (auto& [cellId, cs] : g_cellSinks)
    {
        double sum = 0; int cnt = 0;
        for (size_t i = 2; i < cs.thpSamples.size(); i++) { sum += cs.thpSamples[i]; cnt++; }
        double avg = cnt > 0 ? sum / cnt : 0;
        csv << "," << std::fixed << std::setprecision(1) << avg << "," << cs.sinks.GetN();
    }
    csv << "," << std::setprecision(1) << totalAvgThp
        << "," << std::setprecision(1) << totalAvgThp / numUes << std::endl;

    Simulator::Destroy();
    return 0;
}
