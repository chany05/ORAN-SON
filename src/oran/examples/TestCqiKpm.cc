#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/E2AP.h"

using namespace ns3;
using namespace oran;

NS_LOG_COMPONENT_DEFINE("TestCqiKpm");

int main()
{
    // 기본 설정
    uint16_t numEnbs = 1;
    uint16_t numUes = 1;
    double simTime = 5.0;

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    // 핸드오버 알고리즘 비활성화
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");

    // 노드 생성
    NodeContainer enbNodes;
    enbNodes.Create(numEnbs);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // eNB 위치
    Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator>();
    enbPos->Add(Vector(0, 0, 0));
    
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPos);
    enbMobility.Install(enbNodes);

    // UE 위치
    // UE 위치 (이동하면서 CQI 변화 관찰)                                   // <-- 변경 시작
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);

    // UE 0: 셀 경계에서 시작, eNB 2 방향으로 이동
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(200, 300, 0));
    ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(50, 0, 0));
                                                                         // <-- 변경 끝

    // LTE 설치
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);

    // IP 스택
    InternetStackHelper internet;
    internet.Install(ueNodes);
    epcHelper->AssignUeIpv4Address(ueDevs);

    // UE 연결
    lteHelper->Attach(ueDevs.Get(0), enbDevs.Get(0));

    // 트래픽 생성 (중요!)
    uint16_t dlPort = 1234;
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> remoteHost = CreateObject<Node>();
    internet.Install(remoteHost);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    NetDeviceContainer internetDevs = p2p.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIfaces = ipv4.Assign(internetDevs);

    // UDP 트래픽
    UdpClientHelper dlClient(epcHelper->GetUeDefaultGatewayAddress(), dlPort);
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    dlClient.SetAttribute("MaxPackets", UintegerValue(1000));
    ApplicationContainer clientApps = dlClient.Install(remoteHost);
    clientApps.Start(Seconds(1.0));

    // E2AP 설정
    Ptr<Node> sgw = epcHelper->GetSgwNode();

    E2AP e2t;  // RIC
    E2AP e2n1; // eNB 1
                                                                              // <-- e2n2 제거
    sgw->AddApplication(&e2t);
    enbNodes.Get(0)->AddApplication(&e2n1);
                                                                              // <-- 제거
    Simulator::Schedule(Seconds(0.5), &E2AP::Connect, &e2t);
    Simulator::Schedule(Seconds(1.0), &E2AP::Connect, &e2n1);
                                                                              // <-- 제거
    Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n1);
                                                                              // <-- 제거
    Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n1);
                                                                              // <-- 제거

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}