//
// TestSONXapp.cc
//
// xAppHandoverSON 테스트 시나리오
// - eNB 3개, UE 4개 (셀 1에 집중 → 부하 불균형 유도)
// - UDP DL/UL 트래픽 (throughput KPM 검증)
// - SON 주기적 KPM 수집 + 자체 핸드오버 의사결정
//

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include "ns3/E2AP.h"
#include "ns3/xAppHandoverSON.h"

using namespace ns3;
using namespace oran;

NS_LOG_COMPONENT_DEFINE("TestSONXapp");

void
NotifyHandoverStartEnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    NS_LOG_UNCOND("Inter-eNB handover (i.e., X2) for cellId " << targetCellId);
}

int
main(int argc, char* argv[])
{
    // =========================================================================
    // 시뮬레이션 파라미터
    // =========================================================================
    uint16_t numEnbs = 3;
    uint16_t numUes = 4;
    double simTime = 20.0;
    double sonPeriodSec = 1.0;
    bool sonInitiateHandovers = true;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("sonPeriod", "SON check period (s)", sonPeriodSec);
    cmd.AddValue("sonHandover", "SON initiates handovers", sonInitiateHandovers);
    cmd.Parse(argc, argv);

    // =========================================================================
    // LTE Helper
    // =========================================================================
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger",
                                              TimeValue(MilliSeconds(256)));

    // =========================================================================
    // 노드 생성
    // =========================================================================
    NodeContainer enbNodes;
    enbNodes.Create(numEnbs);

    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // =========================================================================
    // eNB 위치: 삼각형 배치
    //   eNB1 (0,0)  ----  eNB2 (500,0)
    //          \          /
    //           eNB3 (250,433)
    // =========================================================================
    Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator>();
    enbPos->Add(Vector(0.0, 0.0, 30.0));
    enbPos->Add(Vector(500.0, 0.0, 30.0));
    enbPos->Add(Vector(250.0, 433.0, 30.0));

    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPos);
    enbMobility.Install(enbNodes);

    // =========================================================================
    // UE 위치: 셀 1에 3개 집중, 셀 2에 1개 (부하 불균형)
    // =========================================================================
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector(50.0, 10.0, 1.5));      // UE 0: eNB 1 근처
    uePos->Add(Vector(30.0, -20.0, 1.5));     // UE 1: eNB 1 근처
    uePos->Add(Vector(80.0, 5.0, 1.5));       // UE 2: eNB 1 근처
    uePos->Add(Vector(480.0, 10.0, 1.5));     // UE 3: eNB 2 근처

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.SetPositionAllocator(uePos);
    ueMobility.Install(ueNodes);

    // =========================================================================
    // LTE 디바이스 설치 + X2
    // =========================================================================
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);
    lteHelper->AddX2Interface(enbNodes);

    // =========================================================================
    // 인터넷 + EPC 연결
    // =========================================================================
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> sgw = epcHelper->GetSgwNode();

    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
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
    remoteHostStaticRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // UE IP
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(
                ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(
            epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // =========================================================================
    // UE Attach (가장 가까운 eNB에 자동 연결)
    // =========================================================================
    lteHelper->Attach(ueDevs);

    // =========================================================================
    // UDP DL 트래픽: remoteHost → UE
    // =========================================================================
    uint16_t dlPort = 1234;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        ApplicationContainer sinkApp = dlSink.Install(ueNodes.Get(u));
        sinkApp.Start(Seconds(0.5));

        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(20)));
        dlClient.SetAttribute("MaxPackets", UintegerValue(100000));
        dlClient.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer clientApp = dlClient.Install(remoteHost);
        clientApp.Start(Seconds(2.0));
        clientApp.Stop(Seconds(simTime - 1.0));

        EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        lteHelper->ActivateDedicatedEpsBearer(ueDevs.Get(u), bearer,
                                              EpcTft::Default());
    }

    // =========================================================================
    // UDP UL 트래픽: UE → remoteHost
    // =========================================================================
    uint16_t ulPort = 2000;

    PacketSinkHelper ulSink("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ApplicationContainer ulSinkApp = ulSink.Install(remoteHost);
    ulSinkApp.Start(Seconds(0.5));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        UdpClientHelper ulClient(remoteHostAddr, ulPort);
        ulClient.SetAttribute("Interval", TimeValue(MilliSeconds(50)));
        ulClient.SetAttribute("MaxPackets", UintegerValue(100000));
        ulClient.SetAttribute("PacketSize", UintegerValue(512));
        ApplicationContainer ulApp = ulClient.Install(ueNodes.Get(u));
        ulApp.Start(Seconds(2.5));
        ulApp.Stop(Seconds(simTime - 1.0));
    }

    // =========================================================================
    // 핸드오버 알림 콜백
    // =========================================================================
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartEnb));

    // =========================================================================
    // E2AP 설정 (RIC + E2 Nodes)
    // =========================================================================
    E2AP e2t;   // RIC
    E2AP e2n1;  // eNB 1
    E2AP e2n2;  // eNB 2
    E2AP e2n3;  // eNB 3

    sgw->AddApplication(&e2t);
    enbNodes.Get(0)->AddApplication(&e2n1);
    enbNodes.Get(1)->AddApplication(&e2n2);
    enbNodes.Get(2)->AddApplication(&e2n3);

    // 변경 후: 각 E2Node에 시간차 부여
    Simulator::Schedule(Seconds(0.5), &E2AP::Connect, &e2t);
    Simulator::Schedule(Seconds(1.0), &E2AP::Connect, &e2n1);
    Simulator::Schedule(Seconds(1.5), &E2AP::Connect, &e2n2);
    Simulator::Schedule(Seconds(2.0), &E2AP::Connect, &e2n3);

    Simulator::Schedule(Seconds(3.0), &E2AP::RegisterDefaultEndpoints, &e2n1);
    Simulator::Schedule(Seconds(3.5), &E2AP::RegisterDefaultEndpoints, &e2n2);
    Simulator::Schedule(Seconds(4.0), &E2AP::RegisterDefaultEndpoints, &e2n3);

    Simulator::Schedule(Seconds(5.0), &E2AP::SubscribeToDefaultEndpoints,
                        &e2t, std::ref(e2n1));
    Simulator::Schedule(Seconds(5.5), &E2AP::SubscribeToDefaultEndpoints,
                        &e2t, std::ref(e2n2));
    Simulator::Schedule(Seconds(6.0), &E2AP::SubscribeToDefaultEndpoints,
                        &e2t, std::ref(e2n3));

    // =========================================================================
    // xAppHandoverSON 등록
    // =========================================================================
    xAppHandoverSON sonxapp(sonPeriodSec, sonInitiateHandovers);
    sgw->AddApplication(&sonxapp);

    // =========================================================================
    // 실행
    // =========================================================================
    Simulator::Stop(Seconds(simTime));

    NS_LOG_UNCOND("=== TestSONXapp Start ===");
    NS_LOG_UNCOND("  eNBs: " << numEnbs);
    NS_LOG_UNCOND("  UEs: " << numUes);
    NS_LOG_UNCOND("  SimTime: " << simTime << "s");
    NS_LOG_UNCOND("  SON Period: " << sonPeriodSec << "s");
    NS_LOG_UNCOND("  SON Handover: " << (sonInitiateHandovers ? "ON" : "OFF"));
    NS_LOG_UNCOND("========================");

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}