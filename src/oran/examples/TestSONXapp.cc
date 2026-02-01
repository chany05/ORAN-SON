//
// Created by Gabriel Ferreira(@gabrielcarvfer) on 1/11/22.
//
// Modified: UE 4개 + eNB reachablity Test (SON xApp 테스트)
//

#include "ns3/E2AP.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/xAppHandoverSON.h"

#include <map>
#include <string>

NS_LOG_COMPONENT_DEFINE("TestHandoverSONXapp");

using namespace ns3;
using namespace oran;

void
NotifyConnectionEstablishedUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " UE IMSI " << imsi << ": connected to CellId " << cellid
              << " with RNTI " << rnti << std::endl;
}

void
NotifyHandoverStartUe(std::string context,
                      uint64_t imsi,
                      uint16_t cellid,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    std::cout << context << " UE IMSI " << imsi << ": previously connected to CellId " << cellid
              << " with RNTI " << rnti << ", doing handover to CellId " << targetCellId
              << std::endl;
}

void
NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " UE IMSI " << imsi << ": successful handover to CellId " << cellid
              << " with RNTI " << rnti << std::endl;
}

void
NotifyConnectionEstablishedEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " eNB CellId " << cellid << ": successful connection of UE with IMSI "
              << imsi << " RNTI " << rnti << std::endl;
}

void
NotifyHandoverStartEnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    std::cout << context << " eNB CellId " << cellid << ": start handover of UE with IMSI " << imsi
              << " RNTI " << rnti << " to CellId " << targetCellId << std::endl;
}

void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " eNB CellId " << cellid << ": completed handover of UE with IMSI "
              << imsi << " RNTI " << rnti << std::endl;
}

int
main()
{
    // Testes de conexão de nós
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    uint16_t numberOfUes = 4;
    uint16_t numberOfEnbs = 3;
    uint16_t numBearersPerUe = 1;
    double distance = 400.0; // m
    double enbTxPowerDbm = 2.0;

    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkEnablePcap", BooleanValue(false));
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");

    lteHelper->SetHandoverAlgorithmType(
        "ns3::NoOpHandoverAlgorithm");

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> sgw = epcHelper->GetSgwNode();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Create the Internet
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
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    /*
     * Network topology:
     *
     *   eNB1 (400,200)     eNB2 (600,200)     eNB3 (800,200)
     *     |                   |                   |
     *   UE0 (400,500)       UE3 (600,500)
     *   UE1 (380,480)
     *   UE2 (420,520)
     *
     *   Cell 1에 UE 3개 집중 (부하 불균형), Cell 2에 UE 1개
     *   Cell 3은 비어있음
     */

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // eNB 위치: 일직선 배치 (원본 패턴 유지)
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 1; i < numberOfEnbs + 1; i++)
    {
        Vector enbPosition(distance * (i + 1), distance, 0);
        enbPositionAlloc->Add(enbPosition);
    }
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    // UE 위치: 셀 1 근처에 3개, 셀 2 근처에 1개 (정적)
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    /*
    uePositionAlloc->Add(Vector(distance * 2, distance + 300, 0));       // UE0: eNB1 근처
    uePositionAlloc->Add(Vector(distance * 2 - 20, distance + 280, 0)); // UE1: eNB1 근처
    uePositionAlloc->Add(Vector(distance * 2 + 20, distance + 320, 0)); // UE2: eNB1 근처
    uePositionAlloc->Add(Vector(distance * 3, distance + 300, 0));       // UE3: eNB2 근처
    */
    // UE2를 셀 경계(eNB1과 eNB2 중간)에 배치 → CQI 저하 유도
    uePositionAlloc->Add(Vector(distance * 2, distance + 300, 0));       // UE0: eNB1 근처 (CQI 높음)
    uePositionAlloc->Add(Vector(distance * 3, distance + 300, 0)); // UE1: eNB1 근처 (CQI 높음)
    uePositionAlloc->Add(Vector(distance * 3, distance + 300, 0));    // UE2: eNB1-eNB2 중간 (CQI 낮음)
    uePositionAlloc->Add(Vector(distance * 3, distance + 300, 0));       // UE3: eNB2 근처 (CQI 높음)
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.SetPositionAllocator(uePositionAlloc);
    ueMobility.Install(ueNodes);

    // Install LTE Devices in eNB and UEs
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(enbTxPowerDbm));
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

    // UE 0,1,2 → eNB1에 연결, UE 3 → eNB2에 연결
    for (uint16_t i = 0; i < 3; i++)
    {
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
    }
    lteHelper->Attach(ueLteDevs.Get(3), enbLteDevs.Get(1));

    NS_LOG_LOGIC("setting up applications");

    uint16_t dlPort = 10000;
    uint16_t ulPort = 20000;

    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable>();
    startTimeSeconds->SetAttribute("Min", DoubleValue(0));
    startTimeSeconds->SetAttribute("Max", DoubleValue(0.010));

    for (uint32_t u = 0; u < numberOfUes; ++u)
    {
        Ptr<Node> ue = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

        for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
            ++dlPort;
            ++ulPort;

            ApplicationContainer clientApps;
            ApplicationContainer serverApps;

            NS_LOG_LOGIC("installing UDP DL app for UE " << u);
            UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(u), dlPort);
            clientApps.Add(dlClientHelper.Install(remoteHost));
            PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            serverApps.Add(dlPacketSinkHelper.Install(ue));

            NS_LOG_LOGIC("installing UDP UL app for UE " << u);
            UdpClientHelper ulClientHelper(remoteHostAddr, ulPort);
            clientApps.Add(ulClientHelper.Install(ue));
            PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                                InetSocketAddress(Ipv4Address::GetAny(), ulPort));
            serverApps.Add(ulPacketSinkHelper.Install(remoteHost));

            Ptr<EpcTft> tft = Create<EpcTft>();
            EpcTft::PacketFilter dlpf;
            dlpf.localPortStart = dlPort;
            dlpf.localPortEnd = dlPort;
            tft->Add(dlpf);
            EpcTft::PacketFilter ulpf;
            ulpf.remotePortStart = ulPort;
            ulpf.remotePortEnd = ulPort;
            tft->Add(ulpf);
            EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
            lteHelper->ActivateDedicatedEpsBearer(ueLteDevs.Get(u), bearer, tft);

            Time startTime = Seconds(startTimeSeconds->GetValue());
            serverApps.Start(startTime);
            clientApps.Start(startTime);
        }
    }

    // Add X2 interface
    lteHelper->AddX2Interface(enbNodes);

    // eNB 라우팅 (원본 패턴 유지)
    for (unsigned i = 1; i < numberOfEnbs; i++)
    {
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(enbNodes.Get(i)->GetObject<Ipv4>());
        remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("10.0.0.6"),
                                                   Ipv4Mask("255.255.255.252"),
                                                   1);
    }

    // 콜백
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkUe));

    // =========================================================================
    // E2AP + SON xApp (원본 패턴 유지)
    // =========================================================================
    E2AP e2t;
    NS_ASSERT(e2t.GetInstanceID() == 0);
    NS_ASSERT(e2t.GetRootEndpoint() == "/E2Node/0");

    E2AP e2n1;
    NS_ASSERT(e2n1.GetInstanceID() == 1);
    NS_ASSERT(e2n1.GetRootEndpoint() == "/E2Node/1");

    NS_ASSERT(E2AP::RetrieveInstanceWithEndpoint("/E2Node/0") == static_cast<PubSubInfra*>(&e2t));

    // SON xApp: 주기 1초, 자체 핸드오버 ON
    xAppHandoverSON sonxapp(1.0, true);

    sgw->AddApplication(&e2t);
    sgw->AddApplication(&sonxapp);

    enbNodes.Get(0)->AddApplication(&e2n1);

    Simulator::Schedule(Seconds(0.5), &E2AP::Connect, &e2t);
    Simulator::Schedule(Seconds(1.0), &E2AP::Connect, &e2n1);
    Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n1);
    Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n1);

    E2AP e2n2;
    enbNodes.Get(1)->AddApplication(&e2n2);
    Simulator::Schedule(Seconds(1.0), &E2AP::Connect, &e2n2);
    Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n2);
    Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n2);

    E2AP e2n3;
    enbNodes.Get(2)->AddApplication(&e2n3);
    Simulator::Schedule(Seconds(1.0), &E2AP::Connect, &e2n3);
    Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n3);
    Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n3);

    // =========================================================================
    // 수동 핸드오버 시나리오:
    //   t=6s: UE2를 Cell1 → Cell2로 이동 (부하 분산 테스트)
    //   t=8s: UE2를 Cell2 → Cell3로 이동
    //   t=10s: UE2를 Cell3 → Cell1로 복귀
    //   t=12s: UE0를 Cell1 → Cell3로 이동
    //   t=14s: UE0를 Cell3 → Cell1로 복귀
    // =========================================================================
    /*
    lteHelper->HandoverRequest(Seconds(6.0),
                               ueLteDevs.Get(2),
                               enbLteDevs.Get(0),
                               enbLteDevs.Get(1));

    lteHelper->HandoverRequest(Seconds(8.0),
                               ueLteDevs.Get(2),
                               enbLteDevs.Get(1),
                               enbLteDevs.Get(2));

    lteHelper->HandoverRequest(Seconds(10.0),
                               ueLteDevs.Get(2),
                               enbLteDevs.Get(2),
                               enbLteDevs.Get(0));

    lteHelper->HandoverRequest(Seconds(12.0),
                               ueLteDevs.Get(0),
                               enbLteDevs.Get(0),
                               enbLteDevs.Get(2));

    lteHelper->HandoverRequest(Seconds(14.0),
                               ueLteDevs.Get(0),
                               enbLteDevs.Get(2),
                               enbLteDevs.Get(0));
        */
    Simulator::Stop(Seconds(30.0));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}