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
#include "ns3/config-store-module.h" 
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-ue-rrc.h>

#include <map>
#include <string>
#include <ctime>

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
    /*
    std::cout << context << " UE IMSI " << imsi << ": previously connected to CellId " << cellid
              << " with RNTI " << rnti << ", doing handover to CellId " << targetCellId
              << std::endl;*/
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
    /*
    std::cout << context << " UE IMSI " << imsi << ": successful handover to CellId " << cellid
              << " with RNTI " << rnti << std::endl;*/
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
    /*
    std::cout << context << " eNB CellId " << cellid << ": start handover of UE with IMSI " << imsi
              << " RNTI " << rnti << " to CellId " << targetCellId << std::endl;*/
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
    /*
    std::cout << context << " eNB CellId " << cellid << ": RIC cancelled handover of UE with IMSI "
              << imsi << " RNTI " << rnti << " to CellId " << targetCellId << std::endl;*/
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
    /*
    std::cout << context << " eNB CellId " << cellid
              << ": handover triggered RIC handover control of UE with IMSI " << imsi << " RNTI "
              << rnti << " to CellId " << targetCellId << std::endl;
    */
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
    /*
    std::cout << context << " eNB CellId " << cellid << ": completed handover of UE with IMSI "
              << imsi << " RNTI " << rnti << std::endl;*/
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
    RngSeedManager::SetSeed(1);
    //RngSeedManager::SetRun(1);  // 고정값
    RngSeedManager::SetRun(static_cast<uint64_t>(std::time(nullptr)));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    // ── 변경 ──
    uint16_t numberOfUes = 40;
    uint16_t numberOfEnbs = 3;
    uint16_t numBearersPerUe = 2;
    double enbTxPowerDbm = 32.0;
    std::string output_csv_filename = "outputSONLB.csv";

    std::cout << "=== Load Balancing Test ===" << std::endl;
    std::cout << "  eNBs: " << numberOfEnbs << std::endl;
    std::cout << "  UEs: " << numberOfUes << " (all on eNB1)" << std::endl;
    std::cout << "  SimTime: 256s" << std::endl;
    std::cout << "  SON Period: 1s, Handover: ON" << std::endl;
    std::cout << "==========================" << std::endl;

    // ── 변경: 논문 트래픽 ──
    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::UdpClient::PacketSize", UintegerValue(512));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024));
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(80));
    Config::SetDefault("ns3::LteEnbRrc::EpsBearerToRlcMapping",EnumValue(LteEnbRrc::RLC_UM_ALWAYS));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkEnablePcap", BooleanValue(false));
    lteHelper->SetEpcHelper(epcHelper);

    // ── 변경: 스케줄러, 경로손실, 대역폭 ──
    lteHelper->SetSchedulerType("ns3::PfFfMacScheduler");
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisPropagationLossModel"));
    lteHelper->SetSpectrumChannelType("ns3::MultiModelSpectrumChannel");
    lteHelper->SetEnbAntennaModelType("ns3::IsotropicAntennaModel");
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(100));

    // ── 변경: 핸드오버 파라미터 ──
    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.0));
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(0)));

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> sgw = epcHelper->GetSgwNode();

    // Remote Host
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

    /*
     * Topology: Hexagonal (equilateral triangle) layout, ISD = 500 m
     *
     *                eNB3 (500, 717)
     *               /              \
     *              /    coverage    \
     *             /      area       \
     *   eNB1 (250, 284) -------- eNB2 (750, 284)
     *
     *   Center: (500, 428)
     *   UE: uniform random in bounding box [100, 900] x [100, 900]
     *   Mobility bounds: [50, 950] x [50, 950]
     */

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // ── 변경: eNB 정삼각형 배치, ISD = 500 m ──
    // 삼각형 centroid = (500, 500), UE 분포 중심과 일치
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
    enbPositionAlloc->Add(Vector(250.0,  356.0, 0));   // eNB1: 왼쪽 아래
    enbPositionAlloc->Add(Vector(750.0,  356.0, 0));   // eNB2: 오른쪽 아래
    enbPositionAlloc->Add(Vector(500.0,  789.0, 0));   // eNB3: 위 꼭짓점
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    // ── 변경: UE 균등 분포, 삼각형 커버리지를 포함하는 사각 영역 ──
    MobilityHelper ueMobility;
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

    ueMobility.SetPositionAllocator(ueAllocator);
    ueMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(50, 950, 50, 950)),
        "Speed", StringValue("ns3::ConstantRandomVariable[Constant=3]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.1]"));
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
// 모든 단말을 시작과 동시에 가장 가까운 기지국(eNB)에 자동으로 연결
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        lteHelper->AttachToClosestEnb(ueLteDevs.Get(i), enbLteDevs);
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
    xAppHandoverSON sonxapp(1.0, false);
    //lteHelper->EnableRlcTraces();
    //lteHelper->EnableRlcTraces();
    //lteHelper->EnableMacTraces();  // ← 추가    
    //lteHelper->EnableDlPhyTraces();
    sgw->AddApplication(&e2t);
    sgw->AddApplication(&sonxapp);

    enbNodes.Get(0)->AddApplication(&e2n1);

    Simulator::Schedule(Seconds(0.1), &E2AP::Connect, &e2t);
    Simulator::Schedule(Seconds(0.2), &E2AP::Connect, &e2n1);
    Simulator::Schedule(Seconds(0.3), &E2AP::SendE2SetupRequest, &e2n1);
    //Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n1);
    //Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n1);

    E2AP e2n2;
    enbNodes.Get(1)->AddApplication(&e2n2);
    Simulator::Schedule(Seconds(0.2), &E2AP::Connect, &e2n2);
    Simulator::Schedule(Seconds(0.3), &E2AP::SendE2SetupRequest, &e2n2);
    //Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n2);
    //Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n2);

    E2AP e2n3;
    enbNodes.Get(2)->AddApplication(&e2n3);
    Simulator::Schedule(Seconds(0.2), &E2AP::Connect, &e2n3);
    Simulator::Schedule(Seconds(0.3), &E2AP::SendE2SetupRequest, &e2n3);
    //Simulator::Schedule(Seconds(2.0), &E2AP::RegisterDefaultEndpoints, &e2n3);
    //Simulator::Schedule(Seconds(2.5), &E2AP::SubscribeToDefaultEndpoints, &e2t, e2n3);
    
    // 수동 핸드오버 없음 — SON 자체 부하분산만
    /// UE 위치 트래커
    std::ofstream ueTrajCsv("ue_trajectory.csv");
    ueTrajCsv << "time_s,ueIndex,x,y,servingCellId" << std::endl;
    ueTrajCsv.close();

    // 0.5초마다 모든 UE 위치 기록
    for (double t = 0.0; t < 256.0; t += 0.5)
    {
        Simulator::Schedule(Seconds(t), [&ueNodes, &ueLteDevs, numberOfUes]() {
            std::ofstream csv("ue_trajectory.csv", std::ios::app);
            double now = Simulator::Now().GetSeconds();

            for (uint16_t i = 0; i < numberOfUes; i++)
            {
                Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
                Vector pos = mob->GetPosition();

                // serving cell 조회
                Ptr<LteUeNetDevice> ueDev =
                    DynamicCast<LteUeNetDevice>(ueLteDevs.Get(i));
                uint16_t cellId = 0;
                if (ueDev && ueDev->GetRrc())
                    cellId = ueDev->GetRrc()->GetCellId();

                csv << now << "," << i << ","
                    << pos.x << "," << pos.y << ","
                    << cellId << std::endl;
            }
            csv.close();
        });
    }
    
    // TestSONXappLB.cc에서 Simulator::Stop 직전에
    Simulator::Schedule(Seconds(255.0), [&sonxapp]() {
        sonxapp.SaveModels();
    });
    Simulator::Stop(Seconds(256.0));
    Simulator::Run();
    std::ofstream csvOutput(output_csv_filename);
    /*
    csvOutput << "Time (ns),IMSI,SrcCellId,RNTI,TrgtCellId,Type," << std::endl;
    for (auto& entry : simulationRegistry)
    {
        csvOutput << entry << std::endl;
    }
    csvOutput.close();
    */
    Simulator::Destroy();
    return 0;
}