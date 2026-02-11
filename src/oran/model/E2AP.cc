//
// Created by gabriel on 29/09/22.
//

#include "E2AP.h"

using namespace ns3;
using namespace oran;

NS_LOG_COMPONENT_DEFINE("E2AP");
std::map<uint16_t, Json> E2AP::s_cellInfoStorage;

Ptr<LteEnbRrc>
E2AP::GetRrc()
{
    if (m_rrc == nullptr)
    {
        Ptr<Node> currentNode = GetNode();
        auto val = PointerValue();
        currentNode->GetDevice(0)->GetAttribute("LteEnbRrc", val);
        m_rrc = val.Get<LteEnbRrc>();
    }
    return m_rrc;
}

void
E2AP::HandlePayload(std::string src_endpoint, std::string dest_endpoint, Json payload)
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT_MSG(payload.contains("TYPE"),
                  "Payload from " + src_endpoint + " addressed to " + dest_endpoint +
                      "does not contain a message type.");
    ORAN_MESSAGE_TYPES msg_type = payload.find("TYPE").value();

    // Check if we are not receiving invalid payloads
    if (GetRootEndpoint() == "/E2Node/0")
    {
        // E2T can't receive messages that should have originated on the RIC (himself)
        if (msg_type < RIC_SUBSCRIPTION_RESPONSE)
        {
            std::stringstream ss;
            ss << "E2T received message addressed to himself. Type=" << oran_msg_str.at(msg_type);
            NS_ASSERT(ss.str().c_str());
        }
    }
    else
    {
        // E2 nodes can't receive messages that should have originated on E2 Nodes (himselves)
        if (RIC_SUBSCRIPTION_RESPONSE <= msg_type && msg_type <= E2_NODE_CONFIGURATION_UPDATE)
        {
            std::stringstream ss;
            ss << GetRootEndpoint()
               << " received message addressed to himself. Type=" << oran_msg_str[msg_type];
            NS_ASSERT(ss.str().c_str());
        }
    }

    NS_LOG_FUNCTION("Handling payload with type " + oran_msg_str.at(msg_type) + " : " +
                    to_string(payload));

    // Handle all O-RAN messages
    switch (msg_type)
    {
    // O-RAN WG3 E2AP v2.02 8.2.1.2
    // RIC initiated
    case RIC_SUBSCRIPTION_REQUEST: {
        NS_LOG_FUNCTION(GetRootEndpoint() + " subscribing " + src_endpoint + " to " +
                        to_string(payload["RAN Function"]));

        if (payload["RIC Subscription Details"]["RIC Event Trigger Format"] !=
            EVENT_TRIGGER_PERIODIC)
        {
            NS_ABORT_MSG("Inexistent RIC event trigger format");
        }

        // Fetch data from request
        uint32_t period =
            payload["RIC Subscription Details"]["RIC Event Trigger Definition"]["Period"];
        std::string periodic_endpoint_to_report = payload["RAN Function"];

        auto it = m_endpointPeriodicityAndBuffer.find(periodic_endpoint_to_report);

        // Setup event loop
        if (it != m_endpointPeriodicityAndBuffer.end())
        {
            NS_LOG_FUNCTION(GetRootEndpoint() + " failed to subscribe " + src_endpoint + " to " +
                            to_string(payload["RAN Function"]));
            Json RIC_SUBSCRIPTION_FAILURE_MESSAGE;
            RIC_SUBSCRIPTION_FAILURE_MESSAGE["DEST_ENDPOINT"] = src_endpoint;
            RIC_SUBSCRIPTION_FAILURE_MESSAGE["PAYLOAD"]["TYPE"] = RIC_SUBSCRIPTION_FAILURE;
            RIC_SUBSCRIPTION_FAILURE_MESSAGE["PAYLOAD"]["RAN Function"] =
                periodic_endpoint_to_report;
            SendPayload(RIC_SUBSCRIPTION_FAILURE_MESSAGE);
            return;
        }

        // Register event loop to send payload
        EventId event = Simulator::Schedule(MilliSeconds(period),
                                            &E2AP::PeriodicReport,
                                            this,
                                            src_endpoint,
                                            period,
                                            periodic_endpoint_to_report);
        PeriodicReportStruct entry{period,
                                   event,
                                   src_endpoint,
                                   SystemWallClockTimestamp().ToString(),
                                   {}};
        m_endpointPeriodicityAndBuffer.emplace(periodic_endpoint_to_report, entry);

        // todo: handle actions
        //  payload["RIC Subscription Details"]["Sequence of Actions"];

        NS_LOG_FUNCTION(GetRootEndpoint() + " subscribed " + src_endpoint + " to " +
                        to_string(payload["RAN Function"]));
        Json RIC_SUBSCRIPTION_RESPONSE_MESSAGE;
        RIC_SUBSCRIPTION_RESPONSE_MESSAGE["DEST_ENDPOINT"] = src_endpoint;
        RIC_SUBSCRIPTION_RESPONSE_MESSAGE["PAYLOAD"]["TYPE"] = RIC_SUBSCRIPTION_RESPONSE;
        RIC_SUBSCRIPTION_RESPONSE_MESSAGE["PAYLOAD"]["RAN Function"] = periodic_endpoint_to_report;
        SendPayload(RIC_SUBSCRIPTION_RESPONSE_MESSAGE);
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.1.2
    // E2 initiated
    case RIC_SUBSCRIPTION_RESPONSE: {
        NS_LOG_FUNCTION(dest_endpoint + " was successfully subscribed to " +
                        to_string(payload["RAN Function"]));

        // Store registered endpoint on the RIC
        sSubscribeToEndpoint(payload["RAN Function"], dest_endpoint);
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.1.3
    // E2 initiated
    case RIC_SUBSCRIPTION_FAILURE: {
        NS_LOG_FUNCTION(dest_endpoint + " failed to subscribed to " +
                        to_string(payload["RAN Function"]));
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.2.2
    // RIC initiated
    case RIC_SUBSCRIPTION_DELETE_REQUEST: {
        std::string endpointToUnsubscribe = payload["RAN Function"];
        NS_LOG_FUNCTION(GetRootEndpoint() + " unsubscribing " + src_endpoint + " from " +
                        endpointToUnsubscribe);
        auto endpointIt = m_endpointPeriodicityAndBuffer.find(endpointToUnsubscribe);
        if (endpointIt == m_endpointPeriodicityAndBuffer.end())
        {
            // Failure, send failure message
            NS_LOG_FUNCTION(GetRootEndpoint() + " doesn't have a subscription to " +
                            endpointToUnsubscribe);
            Json RIC_SUBSCRIPTION_DELETE_FAILURE_MESSAGE;
            RIC_SUBSCRIPTION_DELETE_FAILURE_MESSAGE["DEST_ENDPOINT"] = src_endpoint;
            RIC_SUBSCRIPTION_DELETE_FAILURE_MESSAGE["PAYLOAD"]["TYPE"] =
                RIC_SUBSCRIPTION_DELETE_FAILURE;
            RIC_SUBSCRIPTION_DELETE_FAILURE_MESSAGE["PAYLOAD"]["RAN Function"] =
                endpointToUnsubscribe;
            SendPayload(RIC_SUBSCRIPTION_DELETE_FAILURE_MESSAGE);
        }
        else
        {
            NS_LOG_FUNCTION(GetRootEndpoint() + " have the subscription to " +
                            endpointToUnsubscribe);

            // Cancel periodic event
            Simulator::Cancel(endpointIt->second.eventId);

            // Remove entry
            m_endpointPeriodicityAndBuffer.erase(endpointIt);

            NS_LOG_FUNCTION(GetRootEndpoint() + " removed the subscription to " +
                            endpointToUnsubscribe);

            // Successful, send response message
            Json RIC_SUBSCRIPTION_DELETE_RESPONSE_MESSAGE;
            RIC_SUBSCRIPTION_DELETE_RESPONSE_MESSAGE["DEST_ENDPOINT"] = src_endpoint;
            RIC_SUBSCRIPTION_DELETE_RESPONSE_MESSAGE["PAYLOAD"]["TYPE"] =
                RIC_SUBSCRIPTION_DELETE_RESPONSE;
            RIC_SUBSCRIPTION_DELETE_RESPONSE_MESSAGE["PAYLOAD"]["RAN Function"] =
                endpointToUnsubscribe;
            SendPayload(RIC_SUBSCRIPTION_DELETE_RESPONSE_MESSAGE);
        }
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.2.2
    // E2 initiated
    case RIC_SUBSCRIPTION_DELETE_RESPONSE: {
        NS_LOG_FUNCTION(dest_endpoint + " was successfully unsubscribed to " +
                        to_string(payload["RAN Function"]));

        // Remove registry of subscribed endpoint from the RIC
        sUnsubscribeToEndpoint(payload["RAN Function"], dest_endpoint);
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.2.3
    // E2 initiated
    case RIC_SUBSCRIPTION_DELETE_FAILURE: {
        NS_LOG_FUNCTION(dest_endpoint + " was not unsubscribed from " +
                        to_string(payload["RAN Function"]));
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.2A.2
    // E2 initiated
    case RIC_SUBSCRIPTION_DELETE_REQUIRED: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.3.2
    // E2 initiated
    case RIC_INDICATION: {
        NS_LOG_FUNCTION(GetRootEndpoint() +
                        " parsing RIC INDICATION message: " + to_string(payload));
        // Indications are complicated, handle them in a dedicated function
        HandleIndicationPayload(src_endpoint, dest_endpoint, payload);
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.4.2
    // RIC initiated
    case RIC_CONTROL_REQUEST: {
        NS_ASSERT(payload.contains("SERVICE_MODEL"));
        E2_SERVICE_MODELS model = payload["SERVICE_MODEL"];
        switch (model)
        {
        case E2_SERVICE_MODELS::E2SM_RC:
            HandleE2SmRcControlRequest(src_endpoint, dest_endpoint, payload);
            break;
        default:
            NS_ASSERT("Unknown service model for message type RIC Control Request.");
        }
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.4.2
    // E2 initiated
    case RIC_CONTROL_ACKNOWLEDGE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.2.4.3
    // E2 initiated
    case RIC_CONTROL_FAILURE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.1.2
    // E2 initiated
    // O-RAN WG3 E2AP v2.02 8.3.1.2
    // E2 initiated
    case E2_SETUP_REQUEST: {
        HandleE2SetupRequest(src_endpoint, payload);
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.1.2
    // RIC initiated
    case E2_SETUP_RESPONSE: {
        NS_LOG_UNCOND("[E2Setup] " << GetRootEndpoint() << " Setup accepted by RIC"
            << " | Accepted: " << payload["RAN_FUNCTIONS_ACCEPTED"].size());
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.1.3
    // RIC initiated
    case E2_SETUP_FAILURE: {
        NS_LOG_ERROR("[E2Setup] " << GetRootEndpoint()
            << " Setup FAILED | Cause: " << payload["CAUSE"]);
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.2.2
    // RIC or E2 initiated
    case RESET_REQUEST: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.2.3
    // RIC or E2 initiated
    case RESET_RESPONSE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.3.2
    // RIC or E2 initiated
    case ERROR_INDICATION: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.4.2
    // RIC initiated
    case RIC_SERVICE_QUERY: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.4.2
    // E2 initiated
    case RIC_SERVICE_UPDATE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.4.2
    // RIC initiated
    case RIC_SERVICE_UPDATE_ACKNOWLEDGE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.4.3
    // RIC initiated
    case RIC_SERVICE_UPDATE_FAILURE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.5.2
    // E2 initiated
    case E2_NODE_CONFIGURATION_UPDATE: {
        bool successful = true;
        bool temp = true;
        std::vector<std::string> failed_addition_list{};
        std::vector<std::vector<std::string>> failed_update_list{};
        std::vector<std::string> failed_removal_list{};

        if (payload.contains("COMPONENT_CONFIGURATION_ADDITION_LIST"))
        {
            for (std::string new_endpoint :
                 payload.find("COMPONENT_CONFIGURATION_ADDITION_LIST").value())
            {
                temp = true & sRegisterEndpoint(src_endpoint, new_endpoint);
                if (!temp)
                {
                    failed_addition_list.push_back(new_endpoint);
                }
                successful &= temp;
            }
        }
        if (payload.contains("COMPONENT_CONFIGURATION_UPDATE_LIST"))
        {
            for (std::vector<std::string> endpoints :
                 payload.find("COMPONENT_CONFIGURATION_UPDATE_LIST").value())
            {
                temp = true & sUpdateEndpoint(src_endpoint, endpoints[0], endpoints[1]);
                if (!temp)
                {
                    failed_update_list.push_back(endpoints);
                }
                successful &= temp;
            }
        }
        if (payload.contains("COMPONENT_CONFIGURATION_REMOVAL_LIST"))
        {
            for (std::string old_endpoint :
                 payload.find("COMPONENT_CONFIGURATION_REMOVAL_LIST").value())
            {
                temp = true & sRemoveEndpoint(src_endpoint, old_endpoint);
                if (!temp)
                {
                    failed_removal_list.push_back({src_endpoint});
                }
                successful &= temp;
            }
        }
        if (successful)
        {
            Json E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE_MESSAGE;
            E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE_MESSAGE["DEST_ENDPOINT"] = src_endpoint;
            E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE_MESSAGE["PAYLOAD"]["TYPE"] =
                E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE;
            if (payload.contains("COMPONENT_CONFIGURATION_ADDITION_LIST"))
            {
                E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE_MESSAGE
                ["PAYLOAD"]["COMPONENT_CONFIGURATION_ADDITION_LIST"] =
                    payload.find("COMPONENT_CONFIGURATION_ADDITION_LIST").value();
            }
            if (payload.contains("COMPONENT_CONFIGURATION_UPDATE_LIST"))
            {
                E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE_MESSAGE
                ["PAYLOAD"]["COMPONENT_CONFIGURATION_UPDATE_LIST"] =
                    payload.find("COMPONENT_CONFIGURATION_UPDATE_LIST").value();
            }
            if (payload.contains("COMPONENT_CONFIGURATION_REMOVAL_LIST"))
            {
                E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE_MESSAGE
                ["PAYLOAD"]["COMPONENT_CONFIGURATION_REMOVAL_LIST"] =
                    payload.find("COMPONENT_CONFIGURATION_REMOVAL_LIST").value();
            }
            SendPayload(E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE_MESSAGE);
        }
        else
        {
            Json E2_NODE_CONFIGURATION_UPDATE_FAILURE_MESSAGE;
            E2_NODE_CONFIGURATION_UPDATE_FAILURE_MESSAGE["DEST_ENDPOINT"] = src_endpoint;
            E2_NODE_CONFIGURATION_UPDATE_FAILURE_MESSAGE["PAYLOAD"]["TYPE"] =
                E2_NODE_CONFIGURATION_UPDATE_FAILURE;
            if (payload.contains("COMPONENT_CONFIGURATION_ADDITION_LIST"))
            {
                E2_NODE_CONFIGURATION_UPDATE_FAILURE_MESSAGE
                ["PAYLOAD"]["COMPONENT_CONFIGURATION_ADDITION_LIST"] = failed_addition_list;
            }
            if (payload.contains("COMPONENT_CONFIGURATION_UPDATE_LIST"))
            {
                E2_NODE_CONFIGURATION_UPDATE_FAILURE_MESSAGE
                ["PAYLOAD"]["COMPONENT_CONFIGURATION_UPDATE_LIST"] = failed_update_list;
            }
            if (payload.contains("COMPONENT_CONFIGURATION_REMOVAL_LIST"))
            {
                E2_NODE_CONFIGURATION_UPDATE_FAILURE_MESSAGE
                ["PAYLOAD"]["COMPONENT_CONFIGURATION_REMOVAL_LIST"] = failed_removal_list;
            }
            SendPayload(E2_NODE_CONFIGURATION_UPDATE_FAILURE_MESSAGE);
        }
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.5.2
    // RIC initiated
    case E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE: {
        // todo: handle successful case
        NS_LOG_FUNCTION("Received E2_NODE_CONFIGURATION_UPDATE_ACKNOWLEDGE from RIC: " +
                        to_string(payload));
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.5.3
    // RIC initiated
    case E2_NODE_CONFIGURATION_UPDATE_FAILURE: {
        // todo: handle failure case
        NS_LOG_FUNCTION("Received E2_NODE_CONFIGURATION_UPDATE_FAILURE from RIC: " +
                        to_string(payload));
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.6.2
    // RIC initiated
    case E2_CONNECTION_UPDATE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.6.2
    // E2 initiated
    case E2_CONNECTION_UPDATE_ACKNOWLEDGE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.6.3
    // E2 initiated
    case E2_CONNECTION_UPDATE_FAILURE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.7.2
    // RIC or E2 initiated
    case E2_REMOVAL_REQUEST: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.7.2
    // RIC or E2 initiated
    case E2_REMOVAL_RESPONSE: {
    }
    break;
    // O-RAN WG3 E2AP v2.02 8.3.7.3
    // RIC or E2 initiated
    case E2_REMOVAL_FAILURE: {
    }
    break;
    default:
        std::cout << this->GetRootEndpoint()
                  << " failed to handle payload with RPC=" << payload["RPC"] << std::endl;
    }
}

void
E2AP::SendPayload(Json payload)
{
    // Add source endpoint
    if (!payload.contains("SRC_ENDPOINT"))
    {
        payload["SRC_ENDPOINT"] = GetRootEndpoint();
    }

    // Replace destination endpoint with the corresponding root endpoint
    if (payload.contains("DEST_ENDPOINT"))
    {
        payload["DEST_ENDPOINT"] = getEndpointRoot(payload["DEST_ENDPOINT"]);
    }

    if (GetRootEndpoint() != "/E2Node/0")
    {
        NS_LOG_FUNCTION("Sending the payload of type " +
                        oran_msg_str.at(payload.at("PAYLOAD").at("TYPE")) +
                        " to the RIC: " + to_string(payload));
        m_socket->SendTo(encodeJsonToPacket(payload), 0, m_node0Address);
    }
    else
    {
        // If we are on the RIC, we can handle things locally if the payload is addressed to us
        if (payload["DEST_ENDPOINT"] == "/E2Node/0")
        {
            NS_LOG_FUNCTION("RIC handling the payload of type " +
                            oran_msg_str.at(payload.at("PAYLOAD").at("TYPE")) +
                            " locally: " + to_string(payload));
            HandlePayload(payload["SRC_ENDPOINT"], GetRootEndpoint(), payload.at("PAYLOAD"));
        }
        // Or we need to forward to the correct node
        else
        {
            NS_LOG_FUNCTION("RIC forwarding the payload of type " +
                            oran_msg_str.at(payload.at("PAYLOAD").at("TYPE")) + " to " +
                            to_string(payload["DEST_ENDPOINT"]) + ": " + to_string(payload));
            m_socket->SendTo(encodeJsonToPacket(payload),
                             0,
                             getAddressFromEndpointRoot(payload["DEST_ENDPOINT"]));
        }
    }
}

void
E2AP::RegisterEndpoint(std::string endpoint)
{
    NS_LOG_FUNCTION(this);
    Json E2_NODE_CONFIGURATION_UPDATE_MESSAGE;
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["DEST_ENDPOINT"] = "/E2Node/0";
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["PAYLOAD"]["TYPE"] = E2_NODE_CONFIGURATION_UPDATE;
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["PAYLOAD"]["COMPONENT_CONFIGURATION_ADDITION_LIST"] =
        std::vector<std::string>{endpoint};
    SendPayload(E2_NODE_CONFIGURATION_UPDATE_MESSAGE);
}

void
E2AP::UpdateEndpoint(std::string old_endpoint, std::string new_endpoint)
{
    NS_LOG_FUNCTION(this);
    Json E2_NODE_CONFIGURATION_UPDATE_MESSAGE;
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["DEST_ENDPOINT"] = "/E2Node/0";
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["PAYLOAD"]["TYPE"] = E2_NODE_CONFIGURATION_UPDATE;
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["PAYLOAD"]["COMPONENT_CONFIGURATION_UPDATE_LIST"] =
        std::vector<std::vector<std::string>>{{old_endpoint, new_endpoint}};
    SendPayload(E2_NODE_CONFIGURATION_UPDATE_MESSAGE);
}

void
E2AP::RemoveEndpoint(std::string endpoint)
{
    NS_LOG_FUNCTION(this);
    Json E2_NODE_CONFIGURATION_UPDATE_MESSAGE;
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["DEST_ENDPOINT"] = "/E2Node/0";
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["PAYLOAD"]["TYPE"] = E2_NODE_CONFIGURATION_UPDATE;
    E2_NODE_CONFIGURATION_UPDATE_MESSAGE["PAYLOAD"]["COMPONENT_CONFIGURATION_REMOVAL_LIST"] =
        std::vector<std::string>{endpoint};
    SendPayload(E2_NODE_CONFIGURATION_UPDATE_MESSAGE);
}

void
E2AP::SubscribeToEndpoint(std::string endpoint)
{
    SubscribeToEndpointPeriodic(endpoint, 1000);
}

void
E2AP::SubscribeToEndpointPeriodic(std::string endpoint, uint32_t periodicity_ms)
{
    NS_LOG_FUNCTION(GetRootEndpoint() + " subscribing to endpoint " + endpoint);
    Json RIC_SUBSCRIPTION_REQUEST_MESSAGE;
    RIC_SUBSCRIPTION_REQUEST_MESSAGE["DEST_ENDPOINT"] = endpoint;
    RIC_SUBSCRIPTION_REQUEST_MESSAGE["PAYLOAD"]["TYPE"] = RIC_SUBSCRIPTION_REQUEST;
    RIC_SUBSCRIPTION_REQUEST_MESSAGE["PAYLOAD"]["RAN Function"] = endpoint;
    RIC_SUBSCRIPTION_REQUEST_MESSAGE["PAYLOAD"]["RIC Subscription Details"];
    RIC_SUBSCRIPTION_REQUEST_MESSAGE["PAYLOAD"]["RIC Subscription Details"]
                                    ["RIC Event Trigger Format"] = EVENT_TRIGGER_PERIODIC;
    RIC_SUBSCRIPTION_REQUEST_MESSAGE["PAYLOAD"]["RIC Subscription Details"]
                                    ["RIC Event Trigger Definition"]["Period"] = periodicity_ms;
    RIC_SUBSCRIPTION_REQUEST_MESSAGE["PAYLOAD"]["RIC Subscription Details"]
                                    ["Sequence of Actions"]; // todo
    SendPayload(RIC_SUBSCRIPTION_REQUEST_MESSAGE);
}

void
E2AP::UnsubscribeToEndpoint(std::string endpoint)
{
    NS_LOG_FUNCTION(GetRootEndpoint() + " unsubscribing to endpoint " + endpoint);
    Json RIC_SUBSCRIPTION_DELETE_REQUEST_MESSAGE;
    RIC_SUBSCRIPTION_DELETE_REQUEST_MESSAGE["DEST_ENDPOINT"] = endpoint;
    RIC_SUBSCRIPTION_DELETE_REQUEST_MESSAGE["PAYLOAD"]["TYPE"] = RIC_SUBSCRIPTION_DELETE_REQUEST;
    RIC_SUBSCRIPTION_DELETE_REQUEST_MESSAGE["PAYLOAD"]["RAN Function"] = endpoint;
    SendPayload(RIC_SUBSCRIPTION_DELETE_REQUEST_MESSAGE);
}

void
E2AP::PublishToEndpointSubscribers(std::string endpoint, Json json)
{
    // NS_LOG_FUNCTION(this << complete_endpoint << to_string (json));
    if (endpoint.find(GetRootEndpoint()) == std::string::npos)
    {
        endpoint = buildEndpoint(GetRootEndpoint(), endpoint);
    }

    // Do not push report if endpoint is not registered
    auto endpointIt = RetrieveSubscribersOfEndpoint(endpoint);
    if (!endpointIt.has_value())
    {
        NS_LOG_FUNCTION(this << "Endpoint not subscribed:" << endpoint);
        return;
    }

    // Published content is sent by the periodic reporting in E2Nodes
    std::string kpm = getSubEndpoint(GetRootEndpoint(),
                                     endpoint); // Remove endpointRoot from full KPM endpoint

    auto it = m_endpointPeriodicityAndBuffer.find(endpoint);
    if (it == m_endpointPeriodicityAndBuffer.end())
    {
        NS_LOG_FUNCTION(this << "Endpoint " + endpoint + " is not subscribed");
        return;
    }
    auto periodicMeasurement =
        PeriodicMeasurementStruct{SystemWallClockTimestamp().ToString(), json};
    it->second.measurements.push_front(periodicMeasurement);
    /*
    NS_LOG_UNCOND("[E2AP] " << json["MEASID"] << " | RNTI=" << json["RNTI"] << " | CELLID=" << json["CELLID"]
                                 << " | ENDPOINT=" << kpm
                                 << " | TIMESTAMP=" << periodicMeasurement.timestamp
                                 << " | VALUE=" << to_string(json["VALUE"]));
            */                       
}

// O-RAN WG3 E2SM KPM v2.00.03 7.3.2
// Trigger timer: only for REPORT, not INSERT/POLICY
// ["periodic"] = KPM interval

// O-RAN WG3 E2SM RC v01.02 7.3.1
std::vector<
    std::tuple<enum RIC_EVENT_TRIGGER_DEFINITION_STYLES, std::string, std::pair<uint8_t, uint8_t>>>
    RIC_EVENT_TRIGGER_DEFINITION_SUPPORTED_RIC_SERVICE_STYLE{
        {MESSAGE_EVENT, "REPORT", {1, 1}},
        {MESSAGE_EVENT, "POLICY", {1, 7}},
        {CALL_PROCESS_BREAKPOINT, "REPORT", {2, 2}},
        {CALL_PROCESS_BREAKPOINT, "INSERT", {1, 7}},
        {CALL_PROCESS_BREAKPOINT, "POLICY", {1, 8}},
        {E2_NODE_INFORMATION_CHANGE, "REPORT", {3, 3}},
        {UE_INFORMATION_CHANGE, "REPORT", {4, 4}},
        {ON_DEMAND, "REPORT", {5, 5}},
    };

// O-RAN WG3 E2SM KPM v2.00.03 7.4.1
// REPORT services
enum REPORT_KPM_SERVICES
{
    E2_NODE_MEASUREMENT = 1,
    E2_NODE_MEASUREMENT_FOR_SINGLE_UE,
    E2_NODE_CONDITIONAL_MEASUREMENT_FOR_UES,
    E2_NODE_CONDITIONAL_MEASUREMENT_SET_FOR_UES,
    E2_NODE_MEASUREMENT_FOR_MULTIPLE_UES
};

// REPORTed measurements
// 5G https://www.etsi.org/deliver/etsi_TS/128500_128599/128552/16.10.00_60/ts_128552v161000p.pdf
// 4G https://www.etsi.org/deliver/etsi_TS/132400_132499/132425/14.01.00_60/ts_132425v140100p.pdf

void
E2AP::RegisterDefaultEndpoints()
{
    NS_LOG_FUNCTION(this);
    for (auto& kpmEndpoints : m_defaultEndpointsKpmTypeAndUnit)
    {
        if (std::get<2>(kpmEndpoints.second) == IMPLEMENTED)
        {
            RegisterEndpoint("/KPM/" + kpmEndpoints.first);
        }
    }
}

void
E2AP::SubscribeToDefaultEndpoints(const E2AP& e2NodeToSubscribeTo)
{
    NS_LOG_FUNCTION(this);
    for (auto& kpmEndpoints : m_defaultEndpointsKpmTypeAndUnit)
    {
        if (std::get<2>(kpmEndpoints.second) == IMPLEMENTED)
        {
            SubscribeToEndpoint(e2NodeToSubscribeTo.GetRootEndpoint() + "/KPM/" +
                                kpmEndpoints.first);
        }
    }
}

void
E2AP::HandleIndicationPayload(std::string& src_endpoint, std::string& dest_endpoint, Json& payload)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(payload.contains("SERVICE_MODEL"));
    enum E2_SERVICE_MODELS service_model = payload["SERVICE_MODEL"];
    switch (service_model)
    {
    case E2SM_KPM:
        HandleE2SmKpmIndicationPayload(src_endpoint, dest_endpoint, payload);
        break;
    case E2SM_RC:
        HandleE2SmRcIndicationPayload(src_endpoint, dest_endpoint, payload);
        break;
    default:
        NS_ABORT_MSG("Unsupported KPM indication format");
    }
}

void
E2AP::SendE2SetupRequest()
{
    NS_LOG_FUNCTION(this);

    Ptr<LteEnbRrc> rrc = GetRrc();
    if (!rrc)
    {
        NS_LOG_WARN("[E2Setup] No RRC available, cannot send E2 Setup Request");
        return;
    }

    // === Cell Info (표준 정보만) ===
    uint16_t cellId = rrc->ComponentCarrierToCellId(0);
    uint16_t dlBandwidth = rrc->GetDlBandwidth();
    uint16_t ulBandwidth = rrc->GetUlBandwidth();
    uint32_t dlEarfcn = rrc->GetDlEarfcn();
    uint32_t ulEarfcn = rrc->GetUlEarfcn();

    Json cellInfo;
    cellInfo["CELLID"] = cellId;
    cellInfo["DL_BANDWIDTH_PRB"] = dlBandwidth;
    cellInfo["UL_BANDWIDTH_PRB"] = ulBandwidth;
    cellInfo["DL_EARFCN"] = dlEarfcn;
    cellInfo["UL_EARFCN"] = ulEarfcn;

    // === RAN Function List ===
    Json ranFunctionList = Json::array();

    // E2SM-KPM: IMPLEMENTED 메트릭 목록
    Json kpmFunction;
    kpmFunction["RAN_FUNCTION_ID"] = 1;
    kpmFunction["SERVICE_MODEL"] = "E2SM_KPM";
    Json kpmEndpoints = Json::array();
    for (auto& [name, info] : m_defaultEndpointsKpmTypeAndUnit)
    {
        if (std::get<2>(info) == IMPLEMENTED)
        {
            kpmEndpoints.push_back(name);
        }
    }
    kpmFunction["ENDPOINTS"] = kpmEndpoints;
    ranFunctionList.push_back(kpmFunction);

    // E2SM-RC: 지원하는 Control Style 목록
    Json rcFunction;
    rcFunction["RAN_FUNCTION_ID"] = 2;
    rcFunction["SERVICE_MODEL"] = "E2SM_RC";
    rcFunction["SUPPORTED_STYLES"] = Json::array({
        "CONNECTED_MODE_MOBILITY_CONTROL"
    });
    ranFunctionList.push_back(rcFunction);

    // === E2 Setup Request 메시지 구성 ===
    Json msg;
    msg["DEST_ENDPOINT"] = "/E2Node/0";
    msg["PAYLOAD"]["TYPE"] = E2_SETUP_REQUEST;
    msg["PAYLOAD"]["GLOBAL_E2_NODE_ID"] = GetRootEndpoint();
    msg["PAYLOAD"]["CELL_INFO"] = cellInfo;
    msg["PAYLOAD"]["RAN_FUNCTION_LIST"] = ranFunctionList;

    NS_LOG_UNCOND("[E2Setup] " << GetRootEndpoint()
        << " → RIC | CellId=" << cellId
        << " DL_BW=" << dlBandwidth << "PRB"
        << " DL_EARFCN=" << dlEarfcn
        << " KPMs=" << kpmEndpoints.size());

    SendPayload(msg);
}

void
E2AP::HandleE2SetupRequest(std::string& src_endpoint, Json& payload)
{
    NS_LOG_FUNCTION(this);

    // === Failure 체크 ===

    // 1. 필수 필드 검증
    if (!payload.contains("CELL_INFO") ||
        !payload["CELL_INFO"].contains("CELLID") ||
        !payload["CELL_INFO"].contains("DL_BANDWIDTH_PRB") ||
        !payload["CELL_INFO"].contains("DL_EARFCN"))
    {
        NS_LOG_WARN("[E2Setup-RIC] Missing required fields from " << src_endpoint);
        Json failureMsg;
        failureMsg["DEST_ENDPOINT"] = src_endpoint;
        failureMsg["PAYLOAD"]["TYPE"] = E2_SETUP_FAILURE;
        failureMsg["PAYLOAD"]["CAUSE"] = "MISSING_REQUIRED_FIELDS";
        SendPayload(failureMsg);
        return;
    }

    uint16_t cellId = payload["CELL_INFO"]["CELLID"];

    // 2. 중복 Cell ID 체크
    if (s_cellInfoStorage.find(cellId) != s_cellInfoStorage.end())
    {
        NS_LOG_WARN("[E2Setup-RIC] Duplicate CellId=" << cellId << " from " << src_endpoint);
        Json failureMsg;
        failureMsg["DEST_ENDPOINT"] = src_endpoint;
        failureMsg["PAYLOAD"]["TYPE"] = E2_SETUP_FAILURE;
        failureMsg["PAYLOAD"]["CAUSE"] = "DUPLICATE_CELL_ID";
        SendPayload(failureMsg);
        return;
    }

    // === Cell Info 저장 ===
    s_cellInfoStorage[cellId] = payload["CELL_INFO"];

    NS_LOG_UNCOND("[E2Setup-RIC] Registered CellId=" << cellId
        << " DL_BW=" << payload["CELL_INFO"]["DL_BANDWIDTH_PRB"]
        << " DL_EARFCN=" << payload["CELL_INFO"]["DL_EARFCN"]);

    // === RAN Function List 처리 ===
    Json acceptedList = Json::array();
    Json rejectedList = Json::array();

    if (payload.contains("RAN_FUNCTION_LIST"))
    {
        for (auto& ranFunc : payload["RAN_FUNCTION_LIST"])
        {
            int funcId = ranFunc["RAN_FUNCTION_ID"];
            std::string model = ranFunc["SERVICE_MODEL"];

            if (model == "E2SM_KPM")
            {
                // KPM 엔드포인트 자동 등록 + 구독
                if (ranFunc.contains("ENDPOINTS"))
                {
                    for (auto& kpmName : ranFunc["ENDPOINTS"])
                    {
                        std::string endpoint = "/KPM/" + kpmName.get<std::string>();
                        std::string fullEndpoint = buildEndpoint(src_endpoint, endpoint);

                        // eNB 측 엔드포인트 등록 (E2 Node Configuration Update와 동일 로직)
                        sRegisterEndpoint(src_endpoint, fullEndpoint);

                        NS_LOG_FUNCTION("[E2Setup-RIC] Registered KPM endpoint: " << fullEndpoint);
                    }

                    // RIC이 해당 eNB의 모든 KPM에 구독
                    for (auto& kpmName : ranFunc["ENDPOINTS"])
                    {
                        std::string endpoint = src_endpoint + "/KPM/" + kpmName.get<std::string>();
                        SubscribeToEndpointPeriodic(endpoint, 1000);
                    }
                }
                acceptedList.push_back(funcId);
            }
            else if (model == "E2SM_RC")
            {
                // RC는 저장만 — 런타임에 Indication/Control로 동작
                acceptedList.push_back(funcId);
            }
            else
            {
                // 알 수 없는 서비스 모델 → Reject
                Json rejected;
                rejected["RAN_FUNCTION_ID"] = funcId;
                rejected["CAUSE"] = "UNSUPPORTED_SERVICE_MODEL";
                rejectedList.push_back(rejected);
            }
        }
    }

    // === E2 Setup Response 전송 ===
    Json responseMsg;
    responseMsg["DEST_ENDPOINT"] = src_endpoint;
    responseMsg["PAYLOAD"]["TYPE"] = E2_SETUP_RESPONSE;
    responseMsg["PAYLOAD"]["RAN_FUNCTIONS_ACCEPTED"] = acceptedList;
    if (!rejectedList.empty())
    {
        responseMsg["PAYLOAD"]["RAN_FUNCTIONS_REJECTED"] = rejectedList;
    }

    NS_LOG_UNCOND("[E2Setup-RIC] → " << src_endpoint
        << " | Accepted=" << acceptedList.size()
        << " Rejected=" << rejectedList.size());

    SendPayload(responseMsg);
}
std::map<std::string, std::deque<PeriodicMeasurementStruct>>
E2AP::QueryKpmMetric(std::string metric)
{
    auto it = m_kpmToEndpointStorage.find(metric);
    if (it == m_kpmToEndpointStorage.end())
        return std::map<std::string, std::deque<PeriodicMeasurementStruct>>();
    auto result = it->second;     // 복사
    
    // ★ 읽은 후 클리어
    for (auto& [endpoint, deque] : it->second)
    {
        deque.clear();
    }
    
    return result;
}

Json
E2AP::QueryCellInfo(uint16_t cellId)
{
    auto it = s_cellInfoStorage.find(cellId);
    if (it != s_cellInfoStorage.end())
        return it->second;
    return Json();
}

std::vector<uint16_t>
E2AP::GetRegisteredCellIds()
{
    std::vector<uint16_t> ids;
    for (auto& [id, info] : s_cellInfoStorage)
    {
        ids.push_back(id);
    }
    return ids;
}

#include "E2SM-KPM.cc"
#include "E2SM-RC.cc"
