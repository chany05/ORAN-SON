//
// Created by Gabriel Ferreira(@gabrielcarvfer) on 1/11/22.
//
// Modified: UE 12개를 eNB1에 몰아넣기 — 극단적 부하 불균형 + SON 부하분산 테스트
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

using namespace ns3;
using namespace oran;

class Registry
{
  public:
    /**
     *  \brief Type of registry entry
     */
    enum registerType
    {
        HANDOVER_CANCELLED_RIC = 0,
        HANDOVER_TRIGGERED_ENB,
        HANDOVER_START_ENB,
        HANDOVER_OK_ENB,
        HANDOVER_START_UE,
        HANDOVER_OK_UE,
        HANDOVER_ERROR_UE,
        CONNECTION_RECONFIGURATION_ENB,
        CONNECTION_ESTABLISHED_ENB,
        CONNECTION_ERROR_ENB,
        CONNECTION_START_UE,
        CONNECTION_ESTABLISHED_UE,
        CONNECTION_ERROR_UE,
        DISCONNECTION_UE,
    };

    /**
     *  \brief Map between registry type to string
     */
    static inline std::map<enum registerType, std::string> registerTypeStr = {
        {HANDOVER_CANCELLED_RIC, "HANDOVER_CANCELLED_RIC"},
        {HANDOVER_TRIGGERED_ENB, "HANDOVER_TRIGGERED_ENB"},
        {HANDOVER_START_ENB, "HANDOVER_START_ENB"},
        {HANDOVER_OK_ENB, "HANDOVER_OK_ENB"},
        {HANDOVER_START_UE, "HANDOVER_START_UE"},
        {HANDOVER_OK_UE, "HANDOVER_OK_UE"},
        {HANDOVER_ERROR_UE, "HANDOVER_ERROR_UE"},
        {CONNECTION_RECONFIGURATION_ENB, "CONNECTION_RECONFIGURATION_ENB"},
        {CONNECTION_ESTABLISHED_ENB, "CONNECTION_ESTABLISHED_ENB"},
        {CONNECTION_ERROR_ENB, "CONNECTION_ERROR_ENB"},
        {CONNECTION_START_UE, "CONNECTION_START_UE"},
        {CONNECTION_ESTABLISHED_UE, "CONNECTION_ESTABLISHED_UE"},
        {CONNECTION_ERROR_UE, "CONNECTION_ERROR_UE"},
        {DISCONNECTION_UE, "DISCONNECTION_UE"},
    };

    /**
     *  \brief Type of registry entry
     *  \param [in] imsi Network subscriber identifier
     *  \param [in] cellId Cell ID of E2Node/eNB
     *  \param [in] rnti UE temporary identifier
     *  \param [in] trgtCellId Target cell ID
     *  \param [in] type Register type
     */
    Registry(uint64_t imsi,
             uint16_t cellId,
             uint16_t rnti,
             uint16_t trgtCellId,
             enum registerType type)
        : m_timestamp(Simulator::Now()),
          m_imsi(imsi),
          m_srcCellId(cellId),
          m_rnti(rnti),
          m_trgtCellId(trgtCellId),
          m_type(type)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const Registry& registry);

  private:
    Time m_timestamp;         ///< Time of event
    uint64_t m_imsi;          ///< Unique identifier of subscriber
    uint16_t m_srcCellId;     ///< Source cell ID
    uint16_t m_rnti;          ///< UE temporary identifier
    uint16_t m_trgtCellId;    ///< Target cell ID
    enum registerType m_type; ///< Type of register
};

/**
 *  \brief Type of registry entry
 *  \param [in, out] os Output stream
 *  \param [in] registry Registry to serialize
 *  \return output stream
 */
std::ostream&
operator<<(std::ostream& os, const Registry& registry)
{
    os << registry.m_timestamp.GetNanoSeconds() << "," << registry.m_imsi << ","
       << registry.m_srcCellId << "," << registry.m_rnti << "," << registry.m_trgtCellId << ","
       << Registry::registerTypeStr.at(registry.m_type) << ",";
    return os;
}

std::vector<Registry>
    simulationRegistry; ///< Vector containing registries collected through the simulation


void
NotifyConnectionEstablishedUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " UE IMSI " << imsi << ": connected to CellId " << cellid
              << " with RNTI " << rnti << std::endl;
    simulationRegistry.emplace_back(imsi,
                                    cellid,
                                    rnti,
                                    cellid,
                                    Registry::CONNECTION_ESTABLISHED_UE);
}

/**
 * \brief Callback function when a handover is started in the UE
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 * \param [in] targetCellId The destination cell ID
 */
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
    simulationRegistry.emplace_back(imsi, cellid, rnti, targetCellId, Registry::HANDOVER_START_UE);
}

/**
 * \brief Callback function when a handover is successful in the UE
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 */
void
NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " UE IMSI " << imsi << ": successful handover to CellId " << cellid
              << " with RNTI " << rnti << std::endl;
    simulationRegistry.emplace_back(imsi, cellid, rnti, cellid, Registry::HANDOVER_OK_UE);
}

/**
 * \brief Callback function when a connection is established in the eNB
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 */
void
NotifyConnectionEstablishedEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " eNB CellId " << cellid << ": successful connection of UE with IMSI "
              << imsi << " RNTI " << rnti << std::endl;
    simulationRegistry.emplace_back(imsi,
                                    cellid,
                                    rnti,
                                    cellid,
                                    Registry::CONNECTION_ESTABLISHED_ENB);
}

/**
 * \brief Callback function when a connectionis reconfigured in the eNB
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 */
void
NotifyConnectionReconfigurationEnb(std::string context,
                                   uint64_t imsi,
                                   uint16_t cellid,
                                   uint16_t rnti)
{
    std::cout << context << " UE IMSI " << imsi << ": requires a reconfiguration to CellId "
              << cellid << " with RNTI " << rnti << std::endl;
    simulationRegistry.emplace_back(imsi,
                                    cellid,
                                    rnti,
                                    cellid,
                                    Registry::CONNECTION_RECONFIGURATION_ENB);
}

/**
 * \brief Callback function when a handover is started in the eNB
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 * \param [in] targetCellId The destination cell ID
 */
void
NotifyHandoverStartEnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    std::cout << context << " eNB CellId " << cellid << ": start handover of UE with IMSI " << imsi
              << " RNTI " << rnti << " to CellId " << targetCellId << std::endl;
    simulationRegistry.emplace_back(imsi, cellid, rnti, targetCellId, Registry::HANDOVER_START_ENB);
}

/**
 * \brief Callback function when a handover is cancelled in the eNB
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 * \param [in] targetCellId The destination cell ID
 */
void
NotifyHandoverCancelledEnb(std::string context,
                           uint64_t imsi,
                           uint16_t cellid,
                           uint16_t rnti,
                           uint16_t targetCellId)
{
    std::cout << context << " eNB CellId " << cellid << ": RIC cancelled handover of UE with IMSI "
              << imsi << " RNTI " << rnti << " to CellId " << targetCellId << std::endl;
    simulationRegistry.emplace_back(imsi,
                                    cellid,
                                    rnti,
                                    targetCellId,
                                    Registry::HANDOVER_CANCELLED_RIC);
}

/**
 * \brief Callback function when a handover is triggered in the eNB
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 * \param [in] targetCellId The destination cell ID
 */
void
NotifyHandoverTriggeredEnb(std::string context,
                           uint64_t imsi,
                           uint16_t cellid,
                           uint16_t rnti,
                           uint16_t targetCellId)
{
    std::cout << context << " eNB CellId " << cellid
              << ": handover triggered RIC handover control of UE with IMSI " << imsi << " RNTI "
              << rnti << " to CellId " << targetCellId << std::endl;
    simulationRegistry.emplace_back(imsi,
                                    cellid,
                                    rnti,
                                    targetCellId,
                                    Registry::HANDOVER_TRIGGERED_ENB);
}

/**
 * \brief Callback function when a handover is finalized in the eNB
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 */
void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " eNB CellId " << cellid << ": completed handover of UE with IMSI "
              << imsi << " RNTI " << rnti << std::endl;
    simulationRegistry.emplace_back(imsi, cellid, rnti, cellid, Registry::HANDOVER_OK_ENB);
}

/**
 * \brief Callback function when a handover fails in the UE
 * \param [in] context The context from the call
 * \param [in] imsi The subscriber permanent ID associated to the UE
 * \param [in] cellid The cell ID
 * \param [in] rnti The temporary ID from the UE that failed to handover
 */
void
NotifyHandoverEndErrorUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " eNB CellId " << cellid << ": completed handover of UE with IMSI "
              << imsi << " RNTI " << rnti << std::endl;
    simulationRegistry.emplace_back(imsi, cellid, rnti, cellid, Registry::HANDOVER_ERROR_UE);
}

void 
UeStateTransition(std::string context, uint64_t imsi, uint16_t cellId,
                        uint16_t rnti, LteUeRrc::State oldState, LteUeRrc::State newState)
{
    NS_LOG_INFO("[RRC] IMSI=" << imsi << " Cell=" << cellId
              << " RNTI=" << rnti
              << " " << oldState << " → " << newState);
    // CONNECTED에서 벗어날 때만 DISCONNECTION 기록
    if (oldState == LteUeRrc::CONNECTED_NORMALLY && newState != LteUeRrc::CONNECTED_NORMALLY)
    {
        simulationRegistry.emplace_back(imsi, cellId, rnti, cellId, Registry::DISCONNECTION_UE);
    }
}

int
main()
{
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    uint16_t numberOfUes = 40;
    uint16_t numberOfEnbs = 3;
    // distance 변수 제거, isd 사용
    uint16_t numBearersPerUe = 1;
    double enbTxPowerDbm = 46.0;
    std::string output_csv_filename = "outputSONLB.csv";

    std::cout << "=== Load Balancing Test ===" << std::endl;
    std::cout << "  eNBs: " << numberOfEnbs << std::endl;
    std::cout << "  UEs: " << numberOfUes << " (all on eNB1)" << std::endl;
    std::cout << "  SimTime: 30s" << std::endl;
    std::cout << "  SON Period: 4s, Handover: ON" << std::endl;
    std::cout << "==========================" << std::endl;

    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkEnablePcap", BooleanValue(false));
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");

    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(1.0));   // 3dB
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(256)));

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> sgw = epcHelper->GetSgwNode();

    // Remote Host
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
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    /*
     * Topology:
     *
     *   eNB1 (800,400)       eNB2 (1200,400)       eNB3 (1600,400)
     *
     *   UE 12개: 전부 eNB2(1200,400) 근처에 물리적으로 배치
     *            하지만 전부 eNB2에 강제 연결 → 과부하
     *   → SON이 CQI 저하 감지 → eNB2/eNB3로 핸드오버 기대
     */

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // eNB 위치
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
    // === 변경 (hexagonal 정삼각형, ISD=500m) ===
    double isd = 200.0;
    // eNB1: 꼭짓점 위
    // eNB2: 왼쪽 아래
    // eNB3: 오른쪽 아래
    double cx = 500.0;  // 삼각형 중심 x
    double cy = 644.0;  // 삼각형 중심 y (= 500 + isd/√3 ≈ 500 + 288.7 * 0.5)
    double height = isd * std::sqrt(3.0) / 2.0;  // 삼각형 높이 = 173m

    enbPositionAlloc->Add(Vector(cx, cy + height / 3.0, 0));           // eNB1 (500, 788)
    enbPositionAlloc->Add(Vector(cx - isd / 2.0, cy - height * 2.0 / 3.0, 0)); // eNB2 (250, 355)
    enbPositionAlloc->Add(Vector(cx + isd / 2.0, cy - height * 2.0 / 3.0, 0)); // eNB3 (750, 355)
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    // UE 위치: 전부 eNB2(1200,400) 근처에 격자 배치
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    // UE 배치: eNB2 (250, 355) 근처, 반경 30~150m
    double ueCenterX = 400.0;   // eNB2 x
    double ueCenterY = 528.0;   // eNB2 y
    double minRadius = 30.0;
    double maxRadius = 40.0;   // 150 → 80으로 축소

    Ptr<UniformRandomVariable> randRadius = CreateObject<UniformRandomVariable>();
    randRadius->SetAttribute("Min", DoubleValue(minRadius));
    randRadius->SetAttribute("Max", DoubleValue(maxRadius));

    Ptr<UniformRandomVariable> randAngle = CreateObject<UniformRandomVariable>();
    randAngle->SetAttribute("Min", DoubleValue(0.0));
    randAngle->SetAttribute("Max", DoubleValue(2.0 * M_PI));

    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        double r = randRadius->GetValue();
        double theta = randAngle->GetValue();
        double ueX = ueCenterX + r * std::cos(theta);
        double ueY = ueCenterY + r * std::sin(theta);
        uePositionAlloc->Add(Vector(ueX, ueY, 0));
    }
    MobilityHelper ueMobility;
    // 삼각형 전체를 커버하는 범위
    // eNB1 (cx, cy+h/3), eNB2 (cx-isd/2, cy-2h/3), eNB3 (cx+isd/2, cy-2h/3)
    // 여유를 줘서 eNB 배치 영역 + 100m 패딩

    double minX = (cx - isd / 2.0) - 100.0;   // eNB2 x - 100
    double maxX = (cx + isd / 2.0) + 100.0;   // eNB3 x + 100
    double minY = (cy - height * 2.0 / 3.0) - 100.0;  // eNB2,3 y - 100
    double maxY = (cy + height / 3.0) + 100.0;         // eNB1 y + 100

    ueMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
    "Bounds", RectangleValue(Rectangle(minX, maxX, minY, maxY)),
    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=25.0]"),  // 3 m/s (빠른 보행)
    "Direction", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185]"),
    "Distance", DoubleValue(30.0));  // 30m마다 방향 전환
    ueMobility.SetPositionAllocator(uePositionAlloc);
    ueMobility.Install(ueNodes);

    // Install LTE Devices
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(enbTxPowerDbm));
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    // IP stack
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

    // ★ UE 12개 전부 eNB1에 강제 연결 (극단적 과부하)
    // 강제 연결 복구 (eNB2, 인덱스 1)
    for (uint16_t i = 0; i < numberOfUes; i++) {
        Ptr<NetDevice> ueDev = ueLteDevs.Get(i);
        Ptr<NetDevice> enbDev = enbLteDevs.Get(1);  // eNB2
        Simulator::Schedule(Seconds(3.0 + i * 0.1), [lteHelper, ueDev, enbDev]() {
            lteHelper->Attach(ueDev, enbDev);
        });
    }

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

    // X2 interface
    lteHelper->AddX2Interface(enbNodes);

    // eNB 라우팅
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
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                    MakeCallback(&NotifyHandoverEndErrorUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverCancelled",
                    MakeCallback(&NotifyHandoverCancelledEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverTriggered",
                    MakeCallback(&NotifyHandoverTriggeredEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
                    MakeCallback(&UeStateTransition));
    

    // =========================================================================
    // E2AP + SON xApp
    // =========================================================================
    E2AP e2t;
    E2AP e2n1;

    // SON xApp: 주기 1초, 자체 핸드오버 ON
    xAppHandoverSON sonxapp(3.0, false);

    sgw->AddApplication(&e2t);
    sgw->AddApplication(&sonxapp);

    enbNodes.Get(0)->AddApplication(&e2n1);

    Simulator::Schedule(Seconds(1.0), &E2AP::Connect, &e2t);
    Simulator::Schedule(Seconds(1.5), &E2AP::Connect, &e2n1);
    Simulator::Schedule(Seconds(2.0), &E2AP::SendE2SetupRequest, &e2n1);
    //Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n1);
    //Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n1);

    E2AP e2n2;
    enbNodes.Get(1)->AddApplication(&e2n2);
    Simulator::Schedule(Seconds(1.5), &E2AP::Connect, &e2n2);
    Simulator::Schedule(Seconds(2.0), &E2AP::SendE2SetupRequest, &e2n2);
    //Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n2);
    //Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n2);

    E2AP e2n3;
    enbNodes.Get(2)->AddApplication(&e2n3);
    Simulator::Schedule(Seconds(1.0), &E2AP::Connect, &e2n3);
    Simulator::Schedule(Seconds(2.0), &E2AP::SendE2SetupRequest, &e2n3);
    //Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n3);
    //Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n3);

    // 수동 핸드오버 없음 — SON 자체 부하분산만

    // TestSONXappLB.cc에서 Simulator::Stop 직전에
    Simulator::Schedule(Seconds(599.0), [&sonxapp]() {
        sonxapp.SaveModels();
    });
    Simulator::Stop(Seconds(600.0));
    Simulator::Run();
    std::ofstream csvOutput(output_csv_filename);
    csvOutput << "Time (ns),IMSI,SrcCellId,RNTI,TrgtCellId,Type," << std::endl;
    for (auto& entry : simulationRegistry)
    {
        csvOutput << entry << std::endl;
    }
    csvOutput.close();
    Simulator::Destroy();
    return 0;
}