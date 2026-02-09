#include <iostream>
#include <string>
#include <vector>
#include <mqtt/async_client.h>
#include <algorithm> 

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

using json = nlohmann::json;

#define AMR_SIZE_X  15.0  
#define AMR_SIZE_Y  5.0 
#define AMR_SIZE_Z  2.0 

#define AMR_COUNT 1

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("fms_client");

const int QOS = 1;

const std::string FACTS_TOPIC = "agv/v2/ZENIXROBOTICS/0000/instantActions";
const std::string STATE_TOPIC = "agv/v2/ZENIXROBOTICS/0000/state";
const std::string CONNECTION_TOPIC = "agv/v2/ZENIXROBOTICS/0000/connection";

std::vector<std::string> VISUALIZATION_TOPICS;
std::vector<std::string> ORDER_TOPICS;

bool initialized = false;

// ============================================================================
// Order JSON 생성 함수
// ============================================================================

std::string createInitialOrder()
{
    nlohmann::json order;
    
    order["headerId"] = 0;
    order["timestamp"] = "2025-12-29T01:42:19.305756100Z";
    order["version"] = "2.0.0";
    order["manufacturer"] = "ZENIXROBOTICS";
    order["serialNumber"] = "0000";
    order["orderId"] = "93181054-240d-4c64-a3a7-768a522ebd93-0";
    order["orderUpdateId"] = 0;
    
    // Nodes
    order["nodes"] = nlohmann::json::array();
    
    nlohmann::json node1;
    node1["nodeId"] = "PW1001_REAR";
    node1["sequenceId"] = 0;
    node1["released"] = true;
    node1["actions"] = nlohmann::json::array();
    node1["nodePosition"] = {
        {"x", 396.0},
        {"y", 451.0},
        {"mapId", "default_map"},
        {"theta", 1.5707963267948966},
        {"allowedDeviationXY", 1.0},
        {"allowedDeviationTheta", 3.141592653589793}
    };
    order["nodes"].push_back(node1);
    
    nlohmann::json node2;
    node2["nodeId"] = "PW1001_subE_REAR";
    node2["sequenceId"] = 2;
    node2["released"] = true;
    node2["actions"] = nlohmann::json::array();
    node2["nodePosition"] = {
        {"x", 396.0},
        {"y", 469.0},
        {"mapId", "default_map"},
        {"theta", 1.5707963267948966},
        {"allowedDeviationXY", 1.0},
        {"allowedDeviationTheta", 0.17453292519943295}
    };
    order["nodes"].push_back(node2);
    
    nlohmann::json node3;
    node3["nodeId"] = "PPB023_REAR";
    node3["sequenceId"] = 4;
    node3["released"] = false;
    node3["actions"] = nlohmann::json::array();
    node3["nodePosition"] = {
        {"x", 396.0},
        {"y", 519.5},
        {"mapId", "default_map"},
        {"theta", 1.5707963267948966},
        {"allowedDeviationXY", 1.0},
        {"allowedDeviationTheta", 0.17453292519943295}
    };
    order["nodes"].push_back(node3);
    
    // Edges
    order["edges"] = nlohmann::json::array();
    
    nlohmann::json edge1;
    edge1["edgeId"] = "PW1001_REAR --- PW1001_subE_REAR";
    edge1["sequenceId"] = 1;
    edge1["released"] = true;
    edge1["startNodeId"] = "PW1001_REAR";
    edge1["endNodeId"] = "PW1001_subE_REAR";
    edge1["actions"] = nlohmann::json::array();
    edge1["maxSpeed"] = 35.0;
    order["edges"].push_back(edge1);
    
    nlohmann::json edge2;
    edge2["edgeId"] = "PW1001_subE_REAR --- PPB023_REAR";
    edge2["sequenceId"] = 3;
    edge2["released"] = false;
    edge2["startNodeId"] = "PW1001_subE_REAR";
    edge2["endNodeId"] = "PPB023_REAR";
    edge2["actions"] = nlohmann::json::array();
    edge2["maxSpeed"] = 35.0;
    order["edges"].push_back(edge2);
    
    return order.dump(2);
}

std::string createUpdateOrder()
{
    nlohmann::json order;
    
    order["headerId"] = 1;
    order["timestamp"] = "2025-12-29T01:42:19.424330Z";
    order["version"] = "2.0.0";
    order["manufacturer"] = "ZENIXROBOTICS";
    order["serialNumber"] = "0000";
    order["orderId"] = "93181054-240d-4c64-a3a7-768a522ebd93-0";  // ← 동일한 orderId
    order["orderUpdateId"] = 1;  // ← orderUpdateId 증가
    
    // Nodes (seq=0 노드 제외, seq=2부터 시작)
    order["nodes"] = nlohmann::json::array();
    
    nlohmann::json node2;
    node2["nodeId"] = "PW1001_subE_REAR";
    node2["sequenceId"] = 2;
    node2["released"] = true;
    node2["actions"] = nlohmann::json::array();
    node2["nodePosition"] = {
        {"x", 396.0},
        {"y", 469.0},
        {"mapId", "default_map"},
        {"theta", 1.5707963267948966},
        {"allowedDeviationXY", 1.0},
        {"allowedDeviationTheta", 0.17453292519943295}
    };
    order["nodes"].push_back(node2);
    
    nlohmann::json node3;
    node3["nodeId"] = "PPB023_REAR";
    node3["sequenceId"] = 4;
    node3["released"] = true;  // ← released 변경됨
    node3["actions"] = nlohmann::json::array();
    node3["nodePosition"] = {
        {"x", 396.0},
        {"y", 519.5},
        {"mapId", "default_map"},
        {"theta", 1.5707963267948966},
        {"allowedDeviationXY", 1.0},
        {"allowedDeviationTheta", 0.17453292519943295}
    };
    order["nodes"].push_back(node3);
    
    // Edges (seq=1 엣지 제외, seq=3부터 시작)
    order["edges"] = nlohmann::json::array();
    
    nlohmann::json edge2;
    edge2["edgeId"] = "PW1001_subE_REAR --- PPB023_REAR";
    edge2["sequenceId"] = 3;
    edge2["released"] = true;  // ← released 변경됨
    edge2["startNodeId"] = "PW1001_subE_REAR";
    edge2["endNodeId"] = "PPB023_REAR";
    edge2["actions"] = nlohmann::json::array();
    edge2["maxSpeed"] = 1.0;
    order["edges"].push_back(edge2);
    
    return order.dump(2);
}

void publishOrderEdgesAsLines(const std::string& payload,
                             rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                             rclcpp::Node::SharedPtr node)
{
    static visualization_msgs::msg::Marker line_marker;

    if(!initialized)
    {
        line_marker.header.frame_id = "map";
        line_marker.ns = "fms_order_edges";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        line_marker.scale.x = 0.3;
        line_marker.color.r = 0.5;
        line_marker.color.g = 0.5;
        line_marker.color.b = 1.0;
        line_marker.color.a = 1.0;
        
        initialized = true;
        std::cout << "initialized!!! " << std::endl;
    }        

    auto j = nlohmann::json::parse(payload);

    if (!j.contains("nodes") || !j.contains("edges"))
    {
        std::cerr << "Missing nodes or edges in payload" << std::endl;
        return;
    }

    struct NodeInfo {
        std::string nodeId;
        double x, y, theta;
    };

    std::unordered_map<std::string, NodeInfo> node_map;
    for (const auto& node : j["nodes"])
    {
        NodeInfo n;
        n.nodeId = node.value("nodeId", "");
        if (node.contains("nodePosition"))
        {
            n.x = node["nodePosition"].value("x", 0.0);
            n.y = node["nodePosition"].value("y", 0.0);
            n.theta = node["nodePosition"].value("theta", 0.0);
        }
        node_map[n.nodeId] = n;
    }

    // 기존 점들 초기화 (갱신 시 누적 방지)
    line_marker.points.clear();
    line_marker.header.stamp = node->get_clock()->now();    

    int cnt = 0;
    for (const auto& edge : j["edges"])
    {
        static int arc_idx = 0;
        static int line_idx = 0;
        std::string start_id = edge.value("startNodeId", "");
        std::string end_id = edge.value("endNodeId", "");
        std::string ceter_id = edge.value("centerNodeId", "");

        if (node_map.find(start_id) == node_map.end() || 
            node_map.find(end_id) == node_map.end())
            continue;

        const NodeInfo& start_node = node_map[start_id];
        const NodeInfo& end_node = node_map[end_id];

        if (edge.contains("centerNodeId") && node_map.find(ceter_id) != node_map.end())
        {
            const NodeInfo& center_node = node_map[ceter_id];
            double cx = center_node.x;
            double cy = center_node.y;

            std::cout << " turnCenter coord: " << cnt << " " << center_node.nodeId << " " <<  cx << " " << cy << std::endl;

            double radius = std::hypot(start_node.x - cx, start_node.y - cy);

            double start_angle = atan2(start_node.y - cy, start_node.x - cx);
            double end_angle   = atan2(end_node.y - cy, end_node.x - cx);

            double dtheta = end_angle - start_angle;
            if (dtheta > M_PI) dtheta -= 2*M_PI;
            else if (dtheta < -M_PI) dtheta += 2*M_PI;

            int steps = 20;
            visualization_msgs::msg::Marker arc_marker;
            for (int i = 0; i <= steps; i++)
            {
                arc_marker.header.frame_id = "map";
                arc_marker.header.stamp = node->get_clock()->now();
                arc_marker.ns = "fms_order_arcs";
                arc_marker.id = arc_idx++;
                arc_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                arc_marker.action = visualization_msgs::msg::Marker::ADD;

                arc_marker.scale.x = 0.3;
                arc_marker.color.r = 1.0;
                arc_marker.color.g = 0.0;
                arc_marker.color.b = 0.0;
                arc_marker.color.a = 1.0;

                double theta = start_angle + dtheta * (static_cast<double>(i)/steps);
                geometry_msgs::msg::Point p;
                p.x = cx + radius * cos(theta);
                p.y = cy + radius * sin(theta);
                p.z = 0.0;
                arc_marker.points.push_back(p);
            }
            marker_pub->publish(arc_marker);
        }
        else
        {
            // 직선 edge
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = start_node.x;
            p_start.y = start_node.y;
            p_start.z = 0.0;

            p_end.x = end_node.x;
            p_end.y = end_node.y;
            p_end.z = 0.0;

            std::cout << " line coord : " << p_start.x << ", " << p_start.y << " " << p_end.x << " " << p_end.y << std::endl;

            line_marker.header.frame_id = "map";
            line_marker.ns = "fms_order_edges";
            line_marker.id = line_idx++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            line_marker.action = visualization_msgs::msg::Marker::ADD;

            line_marker.scale.x = 0.3;
            line_marker.color.r = 0.5;
            line_marker.color.g = 0.5;
            line_marker.color.b = 1.0;
            line_marker.color.a = 1.0;            

            line_marker.points.push_back(p_start);
            line_marker.points.push_back(p_end);
        }
        cnt++;
    }
    marker_pub->publish(line_marker);
}

// MQTT 콜백 핸들러 클래스 정의
class Callback : public virtual mqtt::callback
{
public:
    Callback(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub,
             const std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>& marker_pubs,
             rclcpp::Node::SharedPtr node)
        : pose_pub_(pose_pub), marker_pubs_(marker_pubs), node_(node), factsheet_received_(false)
    {
        for (size_t i = 0; i < marker_pubs_.size(); ++i)
        {
            visualization_msgs::msg::Marker marker_msg;
            marker_msg.header.frame_id = "map";
            marker_msg.ns = "amr_model_" + std::to_string(i);
            marker_msg.id = 0;
            marker_msg.type = visualization_msgs::msg::Marker::CUBE;
            marker_msg.action = visualization_msgs::msg::Marker::ADD;
            marker_msg.color.a = 0.8;
            marker_msg.color.r = 0.0;
            marker_msg.color.g = 1.0;
            marker_msg.color.b = 0.0;
            initialized_markers_.push_back(marker_msg);
            usleep(10000);
        }
    }

    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::string topic = msg->get_topic();
        std::string payload_ = msg->to_string();

        for (size_t i = 0; i < AMR_COUNT; ++i)
        {
            if (topic == VISUALIZATION_TOPICS[i])
            {
                try
                {
                    auto j = json::parse(payload_);
                    auto pose_json = j.at("pose");
                    double x = pose_json.at("x").get<double>();
                    double y = pose_json.at("y").get<double>();
                    double theta = pose_json.at("theta").get<double>();

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.stamp = node_->get_clock()->now();
                    pose.header.frame_id = "map";
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 1.0;

                    tf2::Quaternion q;
                    q.setRPY(0, 0, theta);
                    pose.pose.orientation = tf2::toMsg(q);

                    pose_pub_->publish(pose);

                    auto& marker_msg = initialized_markers_[i];
                    marker_msg.header.stamp = node_->get_clock()->now();
                    marker_msg.pose = pose.pose;

                    marker_msg.scale.x = AMR_SIZE_X;
                    marker_msg.scale.y = AMR_SIZE_Y;
                    marker_msg.scale.z = AMR_SIZE_Z;

                    marker_pubs_[i]->publish(marker_msg);
                }
                catch (...)
                {
                    std::cerr << "[ERROR] Failed to parse visualization json for AMR " << i << std::endl;
                }
            }
        }
        
        if(topic == STATE_TOPIC)
        {
            std::cout << std::endl;
            std::cout << "recv State_TOPIC : " << payload_ << std::endl;
        }

        if (topic == CONNECTION_TOPIC)
        {
            std::cout << "recv CONNECTION_TOPIC : " << payload_ << std::endl;
        }

        if (topic == FACTS_TOPIC && !factsheet_received_)
        {
            std::cout << "recv FACTS_TOPIC!!! : " << payload_ << std::endl;
            try
            {
                auto j = json::parse(payload_);
                auto physical_params = j.at("physicalParameters");
                double width = physical_params.at("width").get<double>();
                double length = physical_params.at("length").get<double>();
                double height = physical_params.at("heightMax").get<double>();

                auto& marker_msg = initialized_markers_[0];

                marker_msg.scale.x = length;
                marker_msg.scale.y = width;
                marker_msg.scale.z = height;

                marker_msg.pose.position.z = height / 2.0;

                factsheet_received_ = true;
                std::cout << "[ROS2] Received factsheet and set marker dimensions." << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << "[ERROR] Failed to parse factsheet json: " << e.what() << std::endl;
            }
        }
    }

    void connection_lost(const std::string &cause) override
    {
        std::cout << "[MQTT] Connection lost";
        if (!cause.empty())
            std::cout << ", cause: " << cause;
        std::cout << std::endl;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_pubs_;
    std::vector<visualization_msgs::msg::Marker> initialized_markers_;
    rclcpp::Node::SharedPtr node_;
    bool factsheet_received_;
};

std::string getCurrentTimestampISO8601() 
{
    return "2025-11-04T05:57:00.00Z"; 
}

static uint32_t factsheet_request_id = 0; 
const std::string AGV_MANUFACTURER = "ZENIXROBOTICS";
const std::string AGV_SERIAL_NUMBER = "0000";

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("fms_mqtt_publish");
    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    auto edge_marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("fms_order_edges_marker", 10);

    for(int i = 0; i < AMR_COUNT; ++i) 
    {
        VISUALIZATION_TOPICS.push_back("agv/agvs/amr_" + std::to_string(i) + "/visualization");
        ORDER_TOPICS.push_back("agv/agvs/amr_" + std::to_string(i) + "/order");
    }

    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_pubs;

    for (int i = 0; i < AMR_COUNT; ++i)
    {
        std::string marker_topic = "agv_" + std::to_string(i) + "_marker";
        marker_pubs.push_back(node->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10));
    }

    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    sleep(1);

    Callback cb(pose_pub, marker_pubs, node);
    client.set_callback(cb);

    try
    {
        std::cout << "Connecting to the MQTT server..." << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "Connected!" << std::endl;

        // AMR 구독
        for (int i = 0; i < AMR_COUNT; ++i)
        {
            std::stringstream ss;
            ss << std::setw(4) << std::setfill('0') << i;              
            std::string topic_filter = "agv/v2/ZENIXROBOTICS/" + ss.str() + "/#";
            client.subscribe(topic_filter, QOS)->wait();
            std::cout << "Subscribed to topic: " << topic_filter << std::endl;
        }

        // Factsheet 요청
        {
            std::cout << "\n========================================" << std::endl;
            std::cout << "Publishing factsheet request..." << std::endl;
            std::cout << "========================================\n" << std::endl;

            nlohmann::json instant_action_msg;
            
            instant_action_msg["headerId"] = ++factsheet_request_id;
            instant_action_msg["timestamp"] = getCurrentTimestampISO8601();
            instant_action_msg["version"] = "2.1.0";
            instant_action_msg["manufacturer"] = AGV_MANUFACTURER; 
            instant_action_msg["serialNumber"] = AGV_SERIAL_NUMBER;
            
            instant_action_msg["actions"] = nlohmann::json::array();
            
            nlohmann::json factsheet_request_action;
            factsheet_request_action["actionType"] = "factsheetRequest"; 
            factsheet_request_action["actionId"] = "FMS_FS_REQ_" + std::to_string(factsheet_request_id);
            factsheet_request_action["blockingType"] = "NONE";
            
            instant_action_msg["actions"].push_back(factsheet_request_action);

            std::string payload_req_factsheet = instant_action_msg.dump(4);

            auto instant_msg = mqtt::make_message(FACTS_TOPIC, payload_req_factsheet);
            instant_msg->set_qos(QOS);
            client.publish(instant_msg)->wait();
            
            std::cout << "Factsheet request published!" << std::endl;
        }

        sleep(2);

        // ============================================================================
        // Step 1: Initial Order 발행
        // ============================================================================
        {
            std::stringstream ss;
            ss << std::setw(4) << std::setfill('0') << 0;
            std::string order_topic = "agv/v2/ZENIXROBOTICS/" + ss.str() + "/order";

            std::cout << "\n========================================" << std::endl;
            std::cout << "Step 1: Publishing INITIAL Order" << std::endl;
            std::cout << "  orderId: 93181054-240d-4c64-a3a7-768a522ebd93-0" << std::endl;
            std::cout << "  orderUpdateId: 0" << std::endl;
            std::cout << "  nodes: seq 0, 2, 4" << std::endl;
            std::cout << "  edges: seq 1, 3" << std::endl;
            std::cout << "========================================\n" << std::endl;

            std::string initial_order = createInitialOrder();
            
            auto order_msg = mqtt::make_message(order_topic, initial_order);
            order_msg->set_qos(QOS);
            auto token = client.publish(order_msg);
            token->wait();
            
            if (token->is_complete()) 
            {
                std::cout << "✅ Initial Order published successfully!" << std::endl;
            }
            else
            {
                std::cerr << "❌ Initial Order publish failed!" << std::endl;
            }

            // RViz 경로 표시
            publishOrderEdgesAsLines(initial_order, edge_marker_pub, node);
        }

        // ============================================================================
        // Step 2: AGV 이동 대기 (5초)
        // ============================================================================
        {
            std::cout << "\n========================================" << std::endl;
            std::cout << "Step 2: Waiting for AGV to move..." << std::endl;
            std::cout << "  AGV should be moving from seq=0 to seq=2" << std::endl;
            std::cout << "  Waiting 5 seconds..." << std::endl;
            std::cout << "========================================\n" << std::endl;

            for (int i = 5; i > 0; --i)
            {
                std::cout << "  " << i << " seconds remaining..." << std::endl;
                sleep(1);
            }
        }

        // ============================================================================
        // Step 3: Update Order 발행
        // ============================================================================
        {
            std::stringstream ss;
            ss << std::setw(4) << std::setfill('0') << 0;
            std::string order_topic = "agv/v2/ZENIXROBOTICS/" + ss.str() + "/order";

            std::cout << "\n========================================" << std::endl;
            std::cout << "Step 3: Publishing UPDATE Order" << std::endl;
            std::cout << "  orderId: 93181054-240d-4c64-a3a7-768a522ebd93-0 (SAME)" << std::endl;
            std::cout << "  orderUpdateId: 1 (INCREASED)" << std::endl;
            std::cout << "  nodes: seq 2, 4 (seq 0 removed)" << std::endl;
            std::cout << "  edges: seq 3 (seq 1 removed)" << std::endl;
            std::cout << "  released: true (changed from false)" << std::endl;
            std::cout << "========================================\n" << std::endl;

            std::string update_order = createUpdateOrder();
            
            std::cout << "Update Order JSON:" << std::endl;
            std::cout << update_order << std::endl;
            std::cout << std::endl;
            
            auto order_msg = mqtt::make_message(order_topic, update_order);
            order_msg->set_qos(QOS);
            auto token = client.publish(order_msg);
            token->wait();
            
            if (token->is_complete()) 
            {
                std::cout << "✅ Update Order published successfully!" << std::endl;
            }
            else
            {
                std::cerr << "❌ Update Order publish failed!" << std::endl;
            }
        }

        // ============================================================================
        // Step 4: 결과 관찰
        // ============================================================================
        {
            std::cout << "\n========================================" << std::endl;
            std::cout << "Step 4: Observing results..." << std::endl;
            std::cout << "  Expected behavior:" << std::endl;
            std::cout << "    1. AGV receives Update Order" << std::endl;
            std::cout << "    2. orderUpdateId increases: 0 → 1" << std::endl;
            std::cout << "    3. Path merges: keeps seq=0, updates seq≥2" << std::endl;
            std::cout << "    4. AGV continues to seq=4 without stopping" << std::endl;
            std::cout << "  Waiting 10 seconds to observe..." << std::endl;
            std::cout << "========================================\n" << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "Test completed!" << std::endl;
        std::cout << "Check AGV logs for:" << std::endl;
        std::cout << "  - [Vda5050Protocol] ORDER UPDATE detected" << std::endl;
        std::cout << "  - [Vda5050Protocol] Merging update 1..." << std::endl;
        std::cout << "  - [Vda5050Protocol] Merged path now has..." << std::endl;
        std::cout << "========================================\n" << std::endl;

        rclcpp::spin(node);

        client.disconnect()->wait();
        std::cout << "Disconnected." << std::endl;
    }
    catch (const mqtt::exception& exc)
    {
        std::cerr << "Error: " << exc.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}