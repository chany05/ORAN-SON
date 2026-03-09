//
// MADDPG (Beta Distribution) variant of TestSONXappLB
// Same scenario as MASAC: 3 cells, 40 UEs, 5MHz BW
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

NS_LOG_COMPONENT_DEFINE("TestHandoverSONXapp_MADDPG");

using namespace ns3;
using namespace oran;

class Registry_MADDPG
{
  public:
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

    Registry_MADDPG(uint64_t imsi, uint16_t cellId, uint16_t rnti,
                    uint16_t trgtCellId, enum registerType type)
        : m_timestamp(Simulator::Now()),
          m_imsi(imsi),
          m_srcCellId(cellId),
          m_rnti(rnti),
          m_trgtCellId(trgtCellId),
          m_type(type)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const Registry_MADDPG& registry);

  private:
    Time m_timestamp;
    uint64_t m_imsi;
    uint16_t m_srcCellId;
    uint16_t m_rnti;
    uint16_t m_trgtCellId;
    enum registerType m_type;
};

std::ostream&
operator<<(std::ostream& os, const Registry_MADDPG& registry)
{
    os << registry.m_timestamp.GetNanoSeconds() << "," << registry.m_imsi << ","
       << registry.m_srcCellId << "," << registry.m_rnti << "," << registry.m_trgtCellId << ","
       << Registry_MADDPG::registerTypeStr.at(registry.m_type) << ",";
    return os;
}

std::vector<Registry_MADDPG> simulationRegistry_MADDPG;

void
NotifyConnectionEstablishedUe_MADDPG(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " UE IMSI " << imsi << ": connected to CellId " << cellid
              << " with RNTI " << rnti << std::endl;
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, cellid,
                                           Registry_MADDPG::CONNECTION_ESTABLISHED_UE);
}

void
NotifyHandoverStartUe_MADDPG(std::string context, uint64_t imsi, uint16_t cellid,
                              uint16_t rnti, uint16_t targetCellId)
{
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, targetCellId,
                                           Registry_MADDPG::HANDOVER_START_UE);
}

void
NotifyHandoverEndOkUe_MADDPG(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, cellid,
                                           Registry_MADDPG::HANDOVER_OK_UE);
}

void
NotifyConnectionEstablishedEnb_MADDPG(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " eNB CellId " << cellid << ": connection of UE IMSI "
              << imsi << " RNTI " << rnti << std::endl;
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, cellid,
                                           Registry_MADDPG::CONNECTION_ESTABLISHED_ENB);
}

void
NotifyConnectionReconfigurationEnb_MADDPG(std::string context, uint64_t imsi,
                                          uint16_t cellid, uint16_t rnti)
{
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, cellid,
                                           Registry_MADDPG::CONNECTION_RECONFIGURATION_ENB);
}

void
NotifyHandoverStartEnb_MADDPG(std::string context, uint64_t imsi, uint16_t cellid,
                               uint16_t rnti, uint16_t targetCellId)
{
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, targetCellId,
                                           Registry_MADDPG::HANDOVER_START_ENB);
}

void
NotifyHandoverCancelledEnb_MADDPG(std::string context, uint64_t imsi, uint16_t cellid,
                                   uint16_t rnti, uint16_t targetCellId)
{
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, targetCellId,
                                           Registry_MADDPG::HANDOVER_CANCELLED_RIC);
}

void
NotifyHandoverTriggeredEnb_MADDPG(std::string context, uint64_t imsi, uint16_t cellid,
                                   uint16_t rnti, uint16_t targetCellId)
{
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, targetCellId,
                                           Registry_MADDPG::HANDOVER_TRIGGERED_ENB);
}

void
NotifyHandoverEndOkEnb_MADDPG(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, cellid,
                                           Registry_MADDPG::HANDOVER_OK_ENB);
}

void
NotifyHandoverEndErrorUe_MADDPG(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << context << " eNB CellId " << cellid << ": HO error UE IMSI "
              << imsi << " RNTI " << rnti << std::endl;
    simulationRegistry_MADDPG.emplace_back(imsi, cellid, rnti, cellid,
                                           Registry_MADDPG::HANDOVER_ERROR_UE);
}

void
UeStateTransition_MADDPG(std::string context, uint64_t imsi, uint16_t cellId,
                          uint16_t rnti, LteUeRrc::State oldState, LteUeRrc::State newState)
{
    if (oldState == LteUeRrc::CONNECTED_NORMALLY && newState != LteUeRrc::CONNECTED_NORMALLY)
    {
        simulationRegistry_MADDPG.emplace_back(imsi, cellId, rnti, cellId,
                                               Registry_MADDPG::DISCONNECTION_UE);
    }
}

int
main(int argc, char* argv[])
{
    bool inferenceOnly  = false;
    bool loadPretrained = false;
    bool saturate       = false;
    bool baseline       = false;
    double simTime      = 256.0;
    uint32_t rngRun     = 42;

    CommandLine cmd;
    cmd.AddValue("inferenceOnly",  "Inference only (no training)", inferenceOnly);
    cmd.AddValue("loadPretrained", "Load pretrained models",       loadPretrained);
    cmd.AddValue("saturate",       "Saturate Cell1 with 30 UEs",   saturate);
    cmd.AddValue("simTime",        "Simulation duration (s)",       simTime);
    cmd.AddValue("baseline",       "Baseline: no MADDPG, CIO=0 TXP=32", baseline);
    cmd.AddValue("rngRun",         "RNG run number (same seed = same UE mobility)", rngRun);
    cmd.Parse(argc, argv);

    if (inferenceOnly) loadPretrained = true;

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(rngRun);
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    uint16_t numberOfUes = 40;
    uint16_t numberOfEnbs = 3;
    uint16_t numBearersPerUe = 2;
    double enbTxPowerDbm = 32.0;

    std::cout << "=== MADDPG (Beta) Load Balancing Test ===" << std::endl;
    std::cout << "  eNBs: " << numberOfEnbs << std::endl;
    std::cout << "  UEs: " << numberOfUes << std::endl;
    std::cout << "  SimTime: " << simTime << "s" << std::endl;
    std::cout << "  SON Period: 0.5s" << std::endl;
    std::cout << "  InferenceOnly: " << (inferenceOnly ? "YES" : "NO") << std::endl;
    std::cout << "  LoadPretrained: " << (loadPretrained ? "YES" : "NO") << std::endl;
    std::cout << "  Saturate: " << (saturate ? "YES (Cell1 heavy)" : "NO") << std::endl;
    std::cout << "=========================================" << std::endl;

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

    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.0));
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(0)));

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> sgw = epcHelper->GetSgwNode();

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

    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
    enbPositionAlloc->Add(Vector(250.0, 356.0, 0));
    enbPositionAlloc->Add(Vector(750.0, 356.0, 0));
    enbPositionAlloc->Add(Vector(500.0, 789.0, 0));
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    MobilityHelper ueMobility;

    if (saturate)
    {
        Ptr<ListPositionAllocator> satAlloc = CreateObject<ListPositionAllocator>();
        Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
        rng->SetAttribute("Min", DoubleValue(0.0));
        rng->SetAttribute("Max", DoubleValue(1.0));

        for (int i = 0; i < 30; i++) {
            double x = 250.0 + (rng->GetValue() - 0.5) * 160.0;
            double y = 356.0 + (rng->GetValue() - 0.5) * 160.0;
            x = std::max(50.0, std::min(950.0, x));
            y = std::max(50.0, std::min(950.0, y));
            satAlloc->Add(Vector(x, y, 0));
        }
        for (int i = 0; i < 5; i++) {
            double x = 750.0 + (rng->GetValue() - 0.5) * 160.0;
            double y = 356.0 + (rng->GetValue() - 0.5) * 160.0;
            x = std::max(50.0, std::min(950.0, x));
            y = std::max(50.0, std::min(950.0, y));
            satAlloc->Add(Vector(x, y, 0));
        }
        for (int i = 0; i < 5; i++) {
            double x = 500.0 + (rng->GetValue() - 0.5) * 160.0;
            double y = 789.0 + (rng->GetValue() - 0.5) * 160.0;
            x = std::max(50.0, std::min(950.0, x));
            y = std::max(50.0, std::min(950.0, y));
            satAlloc->Add(Vector(x, y, 0));
        }
        std::cout << "[SATURATE] 30 UEs @ Cell1, 5 @ Cell2, 5 @ Cell3" << std::endl;
        ueMobility.SetPositionAllocator(satAlloc);
    }
    else
    {
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
    }

    ueMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(50, 950, 50, 950)),
        "Speed", StringValue("ns3::ConstantRandomVariable[Constant=3]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.1]"));
    ueMobility.Install(ueNodes);

    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(enbTxPowerDbm));
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        lteHelper->AttachToClosestEnb(ueLteDevs.Get(i), enbLteDevs);
    }

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

            UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(u), dlPort);
            dlClientHelper.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
            dlClientHelper.SetAttribute("Interval", TimeValue(MilliSeconds(1)));
            dlClientHelper.SetAttribute("PacketSize", UintegerValue(1400));
            clientApps.Add(dlClientHelper.Install(remoteHost));
            PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            serverApps.Add(dlPacketSinkHelper.Install(ue));

            UdpClientHelper ulClientHelper(remoteHostAddr, ulPort);
            ulClientHelper.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
            ulClientHelper.SetAttribute("Interval", TimeValue(MilliSeconds(2)));
            ulClientHelper.SetAttribute("PacketSize", UintegerValue(1400));
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

    lteHelper->AddX2Interface(enbNodes);

    for (unsigned i = 1; i < numberOfEnbs; i++)
    {
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(enbNodes.Get(i)->GetObject<Ipv4>());
        remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("10.0.0.6"),
                                                   Ipv4Mask("255.255.255.252"), 1);
    }

    // 콜백
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedEnb_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedUe_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartEnb_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartUe_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkUe_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                    MakeCallback(&NotifyHandoverEndErrorUe_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverCancelled",
                    MakeCallback(&NotifyHandoverCancelledEnb_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverTriggered",
                    MakeCallback(&NotifyHandoverTriggeredEnb_MADDPG));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
                    MakeCallback(&UeStateTransition_MADDPG));

    // E2AP + MADDPG xApp
    E2AP e2t;
    E2AP e2n1;

    xAppHandoverSON sonxapp(0.5, false, loadPretrained, inferenceOnly, simTime, baseline);
    sgw->AddApplication(&e2t);
    sgw->AddApplication(&sonxapp);

    enbNodes.Get(0)->AddApplication(&e2n1);
    Simulator::Schedule(Seconds(0.1), &E2AP::Connect, &e2t);
    Simulator::Schedule(Seconds(0.2), &E2AP::Connect, &e2n1);
    Simulator::Schedule(Seconds(0.3), &E2AP::SendE2SetupRequest, &e2n1);

    E2AP e2n2;
    enbNodes.Get(1)->AddApplication(&e2n2);
    Simulator::Schedule(Seconds(0.2), &E2AP::Connect, &e2n2);
    Simulator::Schedule(Seconds(0.3), &E2AP::SendE2SetupRequest, &e2n2);

    E2AP e2n3;
    enbNodes.Get(2)->AddApplication(&e2n3);
    Simulator::Schedule(Seconds(0.2), &E2AP::Connect, &e2n3);
    Simulator::Schedule(Seconds(0.3), &E2AP::SendE2SetupRequest, &e2n3);

    // UE trajectory CSV logging
    {
        std::ofstream ueTrajCsv("ue_trajectory.csv");
        ueTrajCsv << "time_s,ueIndex,x,y,servingCellId" << std::endl;
        ueTrajCsv.close();

        for (double t = 0.0; t < simTime; t += 0.5)
        {
            Simulator::Schedule(Seconds(t), [&ueNodes, &ueLteDevs, numberOfUes]() {
                std::ofstream csv("ue_trajectory.csv", std::ios::app);
                double now = Simulator::Now().GetSeconds();
                for (uint16_t i = 0; i < numberOfUes; i++)
                {
                    Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
                    Vector pos = mob->GetPosition();
                    Ptr<LteUeNetDevice> ueDev =
                        DynamicCast<LteUeNetDevice>(ueLteDevs.Get(i));
                    uint16_t cellId = 0;
                    if (ueDev && ueDev->GetRrc())
                        cellId = ueDev->GetRrc()->GetCellId();
                    csv << now << "," << i << ","
                        << pos.x << "," << pos.y << ","
                        << cellId << "\n";
                }
                csv.close();
            });
        }
    }

    Simulator::Schedule(Seconds(simTime - 1.0), [&sonxapp]() {
        sonxapp.SaveModels();
        sonxapp.FlushCsvLogs();
    });
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
