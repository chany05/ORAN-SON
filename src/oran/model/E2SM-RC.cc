//
// Created by Gabriel Ferreira (@gabrielcarvfer) on 27/10/22.
//

#include "E2SM-RC-control-types.h"
#include "E2SM-RC-indication-types.h"

#include <ns3/lte-enb-rrc.h>

static inline uint16_t
ParseCellIdFromEndpoint(const std::string& endpoint)
{
    std::string s = endpoint;
    if (!s.empty() && s.back() == '/')
    {
        s.pop_back();
    }
    auto pos = s.find_last_of('/');
    if (pos == std::string::npos || pos + 1 >= s.size())
    {
        return 0;
    }
    return static_cast<uint16_t>(std::atoi(s.substr(pos + 1).c_str()));
}

void
E2AP::HandleE2SmRcIndicationPayload(std::string& src_endpoint,
                                    std::string& dest_endpoint,
                                    Json& payload)
{
    NS_LOG_FUNCTION(this);

    E2SM_RC_RIC_INDICATION_HEADER indicationHeader;
    NS_ASSERT(payload.contains("HEADER"));
    from_json(payload["HEADER"], indicationHeader);

    switch (indicationHeader.format)
    {
    case RIC_INDICATION_HEADER_FORMAT_1: {
        // REPORT services
        // uint16_t eventTriggerConditionId =
        // indicationHeader.contents.format_1.eventTriggerConditionID;
    }
    break;
    case RIC_INDICATION_HEADER_FORMAT_2: {
        // INSERT services
        uint16_t rnti = indicationHeader.contents.format_2.RNTI;
        switch (indicationHeader.contents.format_2.RICInsertStyleType)
        {
        case RIC_INSERT_SERVICE_STYLES::RADIO_BEARER_CONTROL_REQUEST::VALUE: {
            switch (indicationHeader.contents.format_2.InsertIndicationID)
            {
            case RIC_INSERT_SERVICE_STYLES::RADIO_BEARER_CONTROL_REQUEST::
                DRB_QOS_CONFIGURATION_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_BEARER_CONTROL_REQUEST::
                QOS_FLOW_MAPPING_CONFIGURATION_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_BEARER_CONTROL_REQUEST::
                LOGICAL_CHANNEL_CONFIGURATION_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_BEARER_CONTROL_REQUEST::
                RADIO_ADMISSION_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_BEARER_CONTROL_REQUEST::
                DRB_TERMINATION_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_BEARER_CONTROL_REQUEST::
                DRB_SPLIT_RATION_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_BEARER_CONTROL_REQUEST::
                PDCP_DUPLICATION_CONTROL_REQUEST::VALUE: {
            }
            break;
            default:
                NS_ABORT_MSG("Unknown Radio Bearer Control Request Indication ID");
            }
        }
        break;
        case RIC_INSERT_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL_REQUEST::VALUE: {
            switch (indicationHeader.contents.format_2.InsertIndicationID)
            {
            case RIC_INSERT_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL_REQUEST::
                DRX_PARAMETER_CONFIGURATION_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL_REQUEST::
                SR_PERIODICITY_CONFIGURATION_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL_REQUEST::
                SPS_PARAMETERS_CONFIGURATION_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL_REQUEST::
                CONFIGURED_GRANT_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL_REQUEST::
                CQI_TABLE_CONFIGURATION_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL_REQUEST::
                SLICE_LEVEL_PRB_QUOTA_REQUEST::VALUE: {
            }
            break;
            default:
                NS_ABORT_MSG("Unknown Radio Resource Allocation Control Request Indication ID");
            }
        }
        break;
        case RIC_INSERT_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL_REQUEST::VALUE: {
            switch (indicationHeader.contents.format_2.InsertIndicationID)
            {
            case RIC_INSERT_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL_REQUEST::
                HANDOVER_CONTROL_REQUEST::VALUE: {
                // RAN parameters from 8.4.4.1
                if (!payload["MESSAGE"].contains("Target Primary Cell ID"))
                {
                    // todo: send RIC_CONTROL_FAILURE
                    return;
                }
                // UE wants to switch to a different cell
                uint16_t ueToHandover = indicationHeader.contents.format_2.RNTI;
                uint16_t requestedTargetCell = payload["MESSAGE"]["Target Primary Cell ID"];

                // Measure time spent on xApp
                auto startTimeXapp = std::chrono::high_resolution_clock::now();

                // Set target cell to the requested by default
                uint16_t targetCell = requestedTargetCell;
                std::function<void(Json&)> handoverHandler = GetEndpointCallback("/Action/HO");
                if (handoverHandler)
                {
                    Json temp;
                    temp["RNTI"] = ueToHandover;
                    temp["Target Primary Cell ID"] = requestedTargetCell;
                     // === ADD (2 lines) ===
		    temp["SRC_ENDPOINT"] = src_endpoint;
		    temp["Source Primary Cell ID"] = ParseCellIdFromEndpoint(src_endpoint);
		    // =====================
                    handoverHandler(temp);
                    targetCell = temp["Target Primary Cell ID"];
                }
                // Measure time spent on xApp
                auto endTimeXapp = std::chrono::high_resolution_clock::now();
                uint64_t nsDelayXapp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           endTimeXapp - startTimeXapp)
                                           .count();

                // Send CONNECTED_MODE_MOBILITY_CONTROL::HANDOVER_CONTROL
                E2SmRcSendHandoverControl(rnti, targetCell, src_endpoint, nsDelayXapp);
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL_REQUEST::
                CONDITIONAL_HANDOVER_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL_REQUEST::
                DUAL_ACTIVE_PROTOCOL_STACK_HANDOVER_CONTROL_REQUEST::VALUE: {
            }
            break;
            default:
                NS_ABORT_MSG("Unknown Radio Resource Allocation Control Request Indication ID");
            }
        }
        break;
        case RIC_INSERT_SERVICE_STYLES::RADIO_ACCESS_CONTROL_REQUEST::VALUE: {
            switch (indicationHeader.contents.format_2.InsertIndicationID)
            {
            case RIC_INSERT_SERVICE_STYLES::RADIO_ACCESS_CONTROL_REQUEST::
                UE_ADMISSION_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_ACCESS_CONTROL_REQUEST::
                RACH_BACKOFF_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_ACCESS_CONTROL_REQUEST::
                ACCESS_BARRING_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_ACCESS_CONTROL_REQUEST::RRC_CONNECTION_RELEASE::
                VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::RADIO_ACCESS_CONTROL_REQUEST::RRC_CONNECTION_REJECT::
                VALUE: {
            }
            break;
            default:
                NS_ABORT_MSG("Unknown Radio Resource Allocation Control Request Indication ID");
            }
        }
        break;
        case RIC_INSERT_SERVICE_STYLES::DUAL_CONNECTIVITY_CONTROL_REQUEST::VALUE: {
            switch (indicationHeader.contents.format_2.InsertIndicationID)
            {
            case RIC_INSERT_SERVICE_STYLES::DUAL_CONNECTIVITY_CONTROL_REQUEST::
                DC_SECONDARY_NODE_ADDITION_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::DUAL_CONNECTIVITY_CONTROL_REQUEST::
                DC_SECONDARY_NODE_MODIFICATION_AND_RELEASE_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::DUAL_CONNECTIVITY_CONTROL_REQUEST::
                DC_PSCELL_CHANGE_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::DUAL_CONNECTIVITY_CONTROL_REQUEST::
                DC_SECONDARY_NODE_CHANGE_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::DUAL_CONNECTIVITY_CONTROL_REQUEST::
                DC_DRB_TERMINATION_CONTROL_REQUEST::VALUE: {
            }
            break;
            default:
                NS_ABORT_MSG("Unknown Radio Resource Allocation Control Request Indication ID");
            }
        }
        break;
        case RIC_INSERT_SERVICE_STYLES::CARRIER_AGGREGATION_CONTROL_REQUEST::VALUE: {
            switch (indicationHeader.contents.format_2.InsertIndicationID)
            {
            case RIC_INSERT_SERVICE_STYLES::CARRIER_AGGREGATION_CONTROL_REQUEST::
                CA_SECONDARY_CELL_ADDITION_CONTROL_REQUEST::VALUE: {
            }
            break;
            case RIC_INSERT_SERVICE_STYLES::CARRIER_AGGREGATION_CONTROL_REQUEST::
                CA_SECONDARY_CELL_MODIFICATION_AND_RELEASE_CONTROL_REQUEST::VALUE: {
            }
            break;
            default:
                NS_ABORT_MSG("Unknown Radio Resource Allocation Control Request Indication ID");
            }
        }
        break;
        case RIC_INSERT_SERVICE_STYLES::IDLE_MODE_MOBILITY_CONTROL_REQUEST::VALUE: {
            switch (indicationHeader.contents.format_2.InsertIndicationID)
            {
            case RIC_INSERT_SERVICE_STYLES::IDLE_MODE_MOBILITY_CONTROL_REQUEST::
                CELL_RESELECTION_PRIORITY_REQUEST::VALUE: {
            }
            break;
            default:
                NS_ABORT_MSG("Unknown Radio Resource Allocation Control Request Indication ID");
            }
        }
        break;
        default:
            NS_ABORT_MSG("Unknown RIC Insert Style type");
        }
    }
    break;
    case RIC_INDICATION_HEADER_FORMAT_3: {
        // todo: INSERT Multiple Actions Control Request
    }
    break;
    default:
        NS_ABORT_MSG("Unknown RIC Indication Header Format");
    }
}

void
handoverTriggeringTrace(Ptr<LteEnbRrc> rrc, uint16_t rnti, uint16_t cellId)
{
    if (rrc->HasUeManager(rnti))
    {
        Ptr<UeManager> ueManager = rrc->GetUeManager(rnti);
        rrc->m_handoverTriggeredTrace(
            ueManager->GetImsi(),
            rrc->ComponentCarrierToCellId(ueManager->GetComponentCarrierId()),
            rnti,
            cellId);
    }
    else
    {
        rrc->m_handoverTriggeredTrace(std::numeric_limits<uint64_t>::max(),
                                      rrc->GetFirstCellId().value(),
                                      rnti,
                                      cellId);
    }
}

void
handoverCancelledTrace(Ptr<LteEnbRrc> rrc, uint16_t rnti, uint16_t cellId)
{
    if (rrc->HasUeManager(rnti))
    {
        // try to find the current cell ID based on the UeManager
        Ptr<UeManager> ueManager = rrc->GetUeManager(rnti);
        // try to find the current cell ID based on the UeManager
        rrc->m_handoverCancelledTrace(
            ueManager->GetImsi(),
            rrc->ComponentCarrierToCellId(ueManager->GetComponentCarrierId()),
            rnti,
            cellId);
    }
    else
    {
        rrc->m_handoverCancelledTrace(std::numeric_limits<uint64_t>::max(),
                                      rrc->GetFirstCellId().value(),
                                      rnti,
                                      cellId);
    }
}

void
E2AP::HandleE2SmRcControlRequest(std::string& src_endpoint,
                                 std::string& dest_endpoint,
                                 Json& payload)
{
    NS_LOG_FUNCTION(this);

    E2SM_RC_RIC_CONTROL_HEADER controlHeader;
    NS_ASSERT(payload.contains("HEADER"));
    from_json(payload["HEADER"], controlHeader);

    switch (controlHeader.format)
    {
    case RC_CONTROL_HEADER_FORMAT_1:
        switch (controlHeader.contents.format_1.RICControlStyleType)
        {
        case RIC_CONTROL_SERVICE_STYLES::RADIO_BEARER_CONTROL::VALUE:
            break;
        case RIC_CONTROL_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL::VALUE:
            ///////////////////////////////////TXP////////////////////////////////////////
            //in Uplink power control - control service - action ID 10
            switch (controlHeader.contents.format_1.ControlActionID)
                {
                case RIC_CONTROL_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL::
                    DL_TX_POWER_CONTROL::VALUE:
                {
                    Ptr<LteEnbRrc> rrc = GetRrc();
                    if (!rrc)
                    {
                        NS_LOG_ERROR("[E2SM-RC] DL_TX_POWER_CONTROL: no RRC");
                        return;
                    }

                    NS_ASSERT_MSG(payload.contains("MESSAGE"),
                                "DL_TX_POWER_CONTROL missing MESSAGE");

                    // MESSAGE 예시: { "TX_POWER": 23 }
                    // 단위: dBm, 범위: -60 ~ 50 (3GPP TS 36.331 §6.3.1)
                    if (payload["MESSAGE"].contains("TX_POWER"))
                    {
                        int8_t txPower = payload["MESSAGE"]["TX_POWER"];
                        
                        if (txPower < -60 || txPower > 50)
                        {
                            NS_LOG_WARN("[E2SM-RC] TX_POWER out of range: " << (int)txPower);
                            return;
                        }

                        uint16_t cellId = rrc->ComponentCarrierToCellId(0);
                        rrc->SetDlTxPower(txPower);  // ← lte-enb-rrc에 구현 필요
                        
                        NS_LOG_INFO("[E2SM-RC] DL TXP | Cell=" << cellId
                                    << " TxPower=" << (int)txPower << " dBm");
                    }

                    // RIC_CONTROL_ACKNOWLEDGE
                    Json ackMsg;
                    ackMsg["DEST_ENDPOINT"] = src_endpoint;
                    ackMsg["PAYLOAD"]["TYPE"] = RIC_CONTROL_ACKNOWLEDGE;
                    ackMsg["PAYLOAD"]["SERVICE_MODEL"] = E2SM_RC;
                    SendPayload(ackMsg);
                }
                break;

                default:
                    NS_LOG_WARN("[E2SM-RC] Unimplemented RRAC action: "
                                << (int)controlHeader.contents.format_1.ControlActionID);
                    break;
                }
                break;
            break;
        case RIC_CONTROL_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL::VALUE:
            switch (controlHeader.contents.format_1.ControlActionID)
            {
            case RIC_CONTROL_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL::HANDOVER_CONTROL::
                VALUE: {
                auto handoverRequestIt = m_pendingRequestsPerRnti.find("HO");
                if (handoverRequestIt == m_pendingRequestsPerRnti.end())
                {
                    m_pendingRequestsPerRnti.emplace("HO", std::map<uint16_t, Json>{});
                    handoverRequestIt = m_pendingRequestsPerRnti.find("HO");
                }
                uint16_t rnti = controlHeader.contents.format_1.RNTI;
                auto UeRntiIt = handoverRequestIt->second.find(rnti);
                if (UeRntiIt == handoverRequestIt->second.end())
                {
                    // Received handover control request directly from the RIC
                    handoverRequestIt->second.emplace(rnti, payload);
                    uint16_t cellId = payload["MESSAGE"]["Target Primary Cell ID"];

                    // Trace the handover triggering just to compare with the eNB initiated version
                    Ptr<LteEnbRrc> rrc = GetRrc();
                    handoverTriggeringTrace(rrc, rnti, cellId);

                    if (cellId != std::numeric_limits<uint16_t>::max())
                    {
                        // IMSI=MAX 상황의 근본 원인: UE manager가 없음
                        if (!rrc->HasUeManager(rnti))
                        {
                            NS_LOG_WARN(GetRootEndpoint()
                                        << " drop RIC HO control: no UE manager for RNTI=" << rnti
                                        << " targetCellId=" << cellId);
                            return;
                        }
                        rrc->DoTriggerHandover(rnti, cellId);
                    }
                    else
                    {
                        handoverCancelledTrace(rrc, rnti, cellId);
                    }
                }
                else
                {
                    UeRntiIt->second = payload;
                }
            }
            break;
            case RIC_CONTROL_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL::
                CONDITIONAL_HANDOVER_CONTROL::VALUE:
            case RIC_CONTROL_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL::
                DUAL_ACTIVE_PROTOCOL_STACK_HANDOVER_CONTROL::VALUE:
            default:
                NS_ASSERT("Unimplemented controls");
                break;
            }
            break;
        case RIC_CONTROL_SERVICE_STYLES::RADIO_ACCESS_CONTROL::VALUE:
            break;
        case RIC_CONTROL_SERVICE_STYLES::DUAL_CONNECTIVITY_CONTROL::VALUE:
            break;
        case RIC_CONTROL_SERVICE_STYLES::CARRIER_AGGREGATION_CONTROL::VALUE:
            break;
        case RIC_CONTROL_SERVICE_STYLES::IDLE_MODE_MOBILITY_CONTROL::VALUE:
            break;
        case RIC_CONTROL_SERVICE_STYLES::UE_INFORMATION_AND_ASSIGNMENT::VALUE:
            break;
        case RIC_CONTROL_SERVICE_STYLES::MEASUREMENT_REPORTING_CONFIGURATION_CONTROL::VALUE:
            switch (controlHeader.contents.format_1.ControlActionID)
            {
            // O-RAN E2SM-RC R004 v09.00 §7.6.11, Action ID 2
            // Modify MR Configuration: cellIndividualOffset (RAN Param ID 113)
            case RIC_CONTROL_SERVICE_STYLES::MEASUREMENT_REPORTING_CONFIGURATION_CONTROL::
                MODIFY_MR_CONFIGURATION::VALUE: {

                Ptr<LteEnbRrc> rrc = GetRrc();
                if (!rrc)
                {
                    NS_LOG_ERROR("[E2SM-RC] MODIFY_MR_CONFIG: no RRC");
                    return;
                }

                NS_ASSERT_MSG(payload.contains("MESSAGE"),
                            "MODIFY_MR_CONFIGURATION missing MESSAGE");

                if (payload["MESSAGE"].contains("CIO_LIST"))
                {
                    uint16_t myCellId = rrc->ComponentCarrierToCellId(0);
                    int validCount = 0;

                    for (auto& entry : payload["MESSAGE"]["CIO_LIST"])
                    {
                        uint16_t neighborCellId = entry["CELL_ID"];
                        int8_t cioValue = entry["CIO_VALUE"];

                        if (cioValue < -24 || cioValue > 24)
                        {
                            NS_LOG_WARN("[E2SM-RC] CIO out of range for cell "
                                        << neighborCellId << ": " << (int)cioValue);
                            continue;
                        }

                        rrc->SetCellIndividualOffset(neighborCellId, cioValue);
                        validCount++;

                        NS_LOG_INFO("[E2SM-RC] CIO | Cell=" << myCellId
                                    << " Neighbor=" << neighborCellId
                                    << " CIO=" << (int)cioValue
                                    << " (" << cioValue * 0.5 << " dB)");
                    }

                    // 리스트 전체 설정 후 한번만 MeasConfig 전송
                    if (validCount > 0)
                    {
                        rrc->SendCioMeasConfigToAllUes();
                    }
                }

                // RIC_CONTROL_ACKNOWLEDGE
                Json ackMsg;
                ackMsg["DEST_ENDPOINT"] = src_endpoint;
                ackMsg["PAYLOAD"]["TYPE"] = RIC_CONTROL_ACKNOWLEDGE;
                ackMsg["PAYLOAD"]["SERVICE_MODEL"] = E2SM_RC;
                SendPayload(ackMsg);
            }
            break;

            default:
                NS_LOG_WARN("[E2SM-RC] Unimplemented MR Config action: "
                            << (int)controlHeader.contents.format_1.ControlActionID);
                break;
            }
            break;
        case RIC_CONTROL_SERVICE_STYLES::MULTIPLE_ACTIONS_CONTROL::VALUE:
            break;
        default:
            NS_ASSERT("Unknown RIC Control Style");
            break;
        }
        break;
    case RC_CONTROL_HEADER_FORMAT_2:
    case RC_CONTROL_HEADER_FORMAT_3:
    default:
        NS_ASSERT("Unsupported RIC Control Request header format");
        break;
    }
}

void
E2AP::E2SmRcSendHandoverControlRequest(uint16_t rnti, uint16_t targetCell, std::string src_endpoint)
{
    E2SM_RC_RIC_INDICATION_HEADER hdr;
    hdr.format = ns3::oran::RIC_INDICATION_HEADER_FORMAT_2;
    hdr.contents.format_2.RNTI = rnti;
    hdr.contents.format_2.RICInsertStyleType =
        RIC_INSERT_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL_REQUEST::VALUE;
    hdr.contents.format_2.InsertIndicationID = RIC_INSERT_SERVICE_STYLES::
        CONNECTED_MODE_MOBILITY_CONTROL_REQUEST::HANDOVER_CONTROL_REQUEST::VALUE;
    Json json_hdr;
    to_json(json_hdr, hdr);
    Json HANDOVER_CONTROL_REQUEST_MSG;
    HANDOVER_CONTROL_REQUEST_MSG["DEST_ENDPOINT"] = "/E2Node/0";
    HANDOVER_CONTROL_REQUEST_MSG["PAYLOAD"]["TYPE"] = RIC_INDICATION;
    HANDOVER_CONTROL_REQUEST_MSG["PAYLOAD"]["SERVICE_MODEL"] = E2SM_RC;
    HANDOVER_CONTROL_REQUEST_MSG["PAYLOAD"]["HEADER"] = json_hdr;
    HANDOVER_CONTROL_REQUEST_MSG["PAYLOAD"]["MESSAGE"]["Target Primary Cell ID"] =
        targetCell; // starts counting from 1

    // We can spoof the SRC_ENDPOINT in case this request
    // is sent by the xApp in the name of an eNB
    if (src_endpoint.size() > 0)
    {
        HANDOVER_CONTROL_REQUEST_MSG["SRC_ENDPOINT"] = src_endpoint;
    }

    // Send indication with control request
    SendPayload(HANDOVER_CONTROL_REQUEST_MSG);
}

void
E2AP::E2SmRcSendCioControlRequest(Json cioList, std::string destination_endpoint)
{
    NS_LOG_FUNCTION(this << destination_endpoint);

    E2SM_RC_RIC_CONTROL_HEADER hdr;
    hdr.format = RC_CONTROL_HEADER_FORMAT_1;
    hdr.contents.format_1.RNTI = 0;  // Cell-level control
    hdr.contents.format_1.RICControlStyleType =
        RIC_CONTROL_SERVICE_STYLES::MEASUREMENT_REPORTING_CONFIGURATION_CONTROL::VALUE;
    hdr.contents.format_1.ControlActionID =
        RIC_CONTROL_SERVICE_STYLES::MEASUREMENT_REPORTING_CONFIGURATION_CONTROL::
            MODIFY_MR_CONFIGURATION::VALUE;
    hdr.contents.format_1.RicDecision = RC_ACCEPT;

    Json json_hdr;
    to_json(json_hdr, hdr);

    Json msg;
    msg["DEST_ENDPOINT"] = destination_endpoint;
    msg["PAYLOAD"]["TYPE"] = RIC_CONTROL_REQUEST;
    msg["PAYLOAD"]["SERVICE_MODEL"] = E2SM_RC;
    msg["PAYLOAD"]["HEADER"] = json_hdr;
    msg["PAYLOAD"]["MESSAGE"]["CIO_LIST"] = cioList;

    SendPayload(msg);
}

void
E2AP::E2SmRcSendTxPowerControlRequest(double txPowerDbm,
                                       std::string destination_endpoint)
{
    NS_LOG_FUNCTION(this << txPowerDbm << destination_endpoint);

    E2SM_RC_RIC_CONTROL_HEADER hdr;
    hdr.format = RC_CONTROL_HEADER_FORMAT_1;
    hdr.contents.format_1.RNTI = 0;  // Cell-level control
    hdr.contents.format_1.RICControlStyleType =
        RIC_CONTROL_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL::VALUE;
    hdr.contents.format_1.ControlActionID =
        RIC_CONTROL_SERVICE_STYLES::RADIO_RESOURCE_ALLOCATION_CONTROL::
            DL_TX_POWER_CONTROL::VALUE;
    hdr.contents.format_1.RicDecision = RC_ACCEPT;

    Json json_hdr;
    to_json(json_hdr, hdr);

    Json msg;
    msg["DEST_ENDPOINT"] = destination_endpoint;
    msg["PAYLOAD"]["TYPE"] = RIC_CONTROL_REQUEST;
    msg["PAYLOAD"]["SERVICE_MODEL"] = E2SM_RC;
    msg["PAYLOAD"]["HEADER"] = json_hdr;
    msg["PAYLOAD"]["MESSAGE"]["TX_POWER"] = txPowerDbm;

    SendPayload(msg);
}

void
E2AP::E2SmRcSendHandoverControl(uint16_t rnti,
                                uint16_t targetCell,
                                std::string& destination_endpoint,
                                double xAppDelayNs)
{
    // Send CONNECTED_MODE_MOBILITY_CONTROL::HANDOVER_CONTROL
    E2SM_RC_RIC_CONTROL_HEADER hdr;
    hdr.format = ns3::oran::RC_CONTROL_HEADER_FORMAT_1;
    hdr.contents.format_1.RNTI = rnti;
    hdr.contents.format_1.RICControlStyleType =
        RIC_CONTROL_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL::VALUE;
    hdr.contents.format_1.ControlActionID =
        RIC_CONTROL_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL::HANDOVER_CONTROL::VALUE;
    hdr.contents.format_1.RicDecision =
        targetCell != std::numeric_limits<uint16_t>::max() ? RC_ACCEPT : RC_REJECT;
    Json json_hdr;
    to_json(json_hdr, hdr);

    Json HANDOVER_CONTROL_MSG;
    HANDOVER_CONTROL_MSG["DEST_ENDPOINT"] = destination_endpoint;
    HANDOVER_CONTROL_MSG["PAYLOAD"]["TYPE"] = RIC_CONTROL_REQUEST;
    HANDOVER_CONTROL_MSG["PAYLOAD"]["SERVICE_MODEL"] = E2SM_RC;
    HANDOVER_CONTROL_MSG["PAYLOAD"]["HEADER"] = json_hdr;
    HANDOVER_CONTROL_MSG["PAYLOAD"]["MESSAGE"]["Target Primary Cell ID"] = targetCell;

    // Send indication with control request with delay to account for the xApp processing time
    Simulator::Schedule(NanoSeconds(xAppDelayNs), &E2AP::SendPayload, this, HANDOVER_CONTROL_MSG);
}

std::optional<uint16_t>
E2AP::E2SmRcHandoverControl(uint16_t rnti, uint16_t cellId, LteEnbRrc& rrc)
{
    auto handoverRequestIt = m_pendingRequestsPerRnti.find("HO");
    if (handoverRequestIt == m_pendingRequestsPerRnti.end())
    {
        m_pendingRequestsPerRnti.emplace("HO", std::map<uint16_t, Json>{});
        handoverRequestIt = m_pendingRequestsPerRnti.find("HO");
    }
    auto UeRntiIt = handoverRequestIt->second.find(rnti);
    if (UeRntiIt == handoverRequestIt->second.end())
    {
        // In case there is no pending request, we need to send a control indication,
        // then wait for a response
        E2SmRcSendHandoverControlRequest(rnti, cellId);

        // Create a pending request entry for the current rnti
        handoverRequestIt->second.emplace(rnti, Json{});

        // Record start time for handover control request
        handoverTriggeringTrace(&rrc, rnti, cellId);

        // Skip the end of this function
        return {};
    }

    if (!UeRntiIt->second.contains("HEADER"))
    {
        return {};
    }

    // Retrieve RIC Control Request message payload
    Json ricControlRequest = UeRntiIt->second;

    // Remove pending message entry
    handoverRequestIt->second.erase(UeRntiIt->first);

    E2SM_RC_RIC_CONTROL_HEADER controlHeader;
    NS_ASSERT(ricControlRequest.contains("HEADER"));
    from_json(ricControlRequest["HEADER"], controlHeader);

    switch (controlHeader.format)
    {
    case RC_CONTROL_HEADER_FORMAT_1:
        if (controlHeader.contents.format_1.RicDecision == RC_REJECT)
        {
            NS_LOG_FUNCTION("RIC Rejected Handover Request from UE " + std::to_string(rnti) +
                            " to Cell " + std::to_string(cellId));
            handoverCancelledTrace(&rrc, rnti, cellId);
            return std::numeric_limits<uint16_t>::max();
        }
        if (controlHeader.contents.format_1.RICControlStyleType !=
            RIC_CONTROL_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL::VALUE)
        {
            NS_ASSERT("Incorrect RIC Control Style");
            // todo: control failure
        }
        if (controlHeader.contents.format_1.ControlActionID !=
            RIC_CONTROL_SERVICE_STYLES::CONNECTED_MODE_MOBILITY_CONTROL::HANDOVER_CONTROL::VALUE)
        {
            NS_ASSERT("Incorrect RIC Control Action ID");
            // todo: control failure
        }
        break;
    case RC_CONTROL_HEADER_FORMAT_2:
        if (controlHeader.contents.format_2.RicDecision == RC_REJECT)
        {
            NS_LOG_FUNCTION("RIC Rejected Handover Request from UE " + std::to_string(rnti) +
                            " to Cell " + std::to_string(cellId));
        }
        // connect to the current CellId
        break;
    default:
        NS_ASSERT("Unknown RC Control Header format");
    }

    // todo: implement the proper way
    // If we succeeded, check if the target cell still is the same
    // E2SM_RC_RIC_CONTROL_MESSAGE controlMessage;
    // NS_ASSERT(UeRntiIt->second["MESSAGE"]);
    // from_json (UeRntiIt->second["MESSAGE"], controlMessage);
    // if (controlMessage.format != E2SM_RC_RIC_CONTROL_MESSAGE_FORMAT_1)
    //  {
    //    NS_ASSERT("Incorrect Control Message format for Handover");
    //  }
    // else
    //  {
    //    cellId =
    //    controlMessage.contents.format_1.sequence_of_ran_parameters.find(RAN_PARAMETER_ID)
    //  }
    //
    // temporary hack
    cellId = ricControlRequest["MESSAGE"]["Target Primary Cell ID"];
    return cellId;
}



