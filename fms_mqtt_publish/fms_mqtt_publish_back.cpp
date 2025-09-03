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
#include <fstream>
#include <sstream>

using json = nlohmann::json;

#define AMR_SIZE_X  3.0  
#define AMR_SIZE_Y  2.0 
#define AMR_SIZE_Z  2.0 

#define AMR_COUNT 5

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("fms_client");

const int QOS = 1;

const std::string VISUALIZATION_TOPICS[AMR_COUNT] = {
    "vda5050/agvs/amr_0/visualization",
    "vda5050/agvs/amr_1/visualization",
    "vda5050/agvs/amr_2/visualization",
    "vda5050/agvs/amr_3/visualization",
    "vda5050/agvs/amr_4/visualization"
};

const std::string ORDER_TOPICS[AMR_COUNT] = {
    "vda5050/agvs/amr_0/order",
    "vda5050/agvs/amr_1/order",
    "vda5050/agvs/amr_2/order",
    "vda5050/agvs/amr_3/order",
    "vda5050/agvs/amr_4/order"
};

const std::string FACTS_TOPIC = "vda5050/agvs/amr_0/instantActions";

const std::string FACTSHEET_REQUEST_INSTANT_ACTION = R"({
    "headerId": "factsheet_request_1",
    "timestamp": 1650000000,
    "version": "2.1.0",
    "manufacturer": "FMSManufacturer",
    "serialNumber": "fms_001",
    "instantActions": [
        {
            "actionType": "factsheetRequest",
            "actionId": "req_001"
        }
    ]
})";

struct NodeInfo 
{
    std::string nodeId;
    int sequenceId;
    double x;
    double y;
    double theta;

    NodeInfo() : sequenceId(0), x(0), y(0), theta(0) {}
};

bool initialized = false;

void publishOrderEdgesAsLines(const std::string& payload,
                             rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                             rclcpp::Node::SharedPtr node)
{
    static visualization_msgs::msg::Marker line_marker;

    if(!initialized)
    {
        line_marker.header.frame_id = "map";
        // line_marker.header.stamp = node->get_clock()->now();
        line_marker.ns = "fms_order_edges";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        line_marker.scale.x = 0.5;
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

    // visualization_msgs::msg::Marker line_marker;

    line_marker.header.stamp = node->get_clock()->now();    

    for (const auto& edge : j["edges"])
    {
        std::string start_id = edge.value("startNodeId", "");
        std::string end_id = edge.value("endNodeId", "");
        if (node_map.find(start_id) == node_map.end() || node_map.find(end_id) == node_map.end())
            continue;

        const NodeInfo& start_node = node_map[start_id];
        const NodeInfo& end_node = node_map[end_id];

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = start_node.x;
        p_start.y = start_node.y;
        p_start.z = 0.0;

        p_end.x = end_node.x;
        p_end.y = end_node.y;
        p_end.z = 0.0;

        std::cout << "start_node.x : " << start_node.x << " " << "start_node.y : " << start_node.y << 
            "end_node.x : " << end_node.x << " " << "end_node.y : " << end_node.y << std::endl;

        line_marker.points.push_back(p_start);
        line_marker.points.push_back(p_end);
    }

    marker_pub->publish(line_marker);
}

// MQTT 콜백 핸들러 클래스 정의 - 다수 AMR 지원
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
            marker_msg.color.a = 0.8; // 투명도
            marker_msg.color.r = 0.0;
            marker_msg.color.g = 1.0;
            marker_msg.color.b = 0.0;
            initialized_markers_.push_back(marker_msg);
        }
    }


    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::string topic = msg->get_topic();
        std::string payload = msg->to_string();

        for (size_t i = 0; i < AMR_COUNT; ++i)
        {
            // std::cout << "recv visualzation : " << i  << " " << payload <<  std::endl;

            if (topic == VISUALIZATION_TOPICS[i])
            {
                // std::cout << "[MQTT] Visualization message arrived for amr_" << i << std::endl;
                try
                {
                    auto j = json::parse(payload);
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

        if (topic == FACTS_TOPIC && !factsheet_received_)
        {
            std::cout << "[MQTT] Factsheet message arrived\n";
            try
            {
                auto j = json::parse(payload);
                auto physical_params = j.at("physicalParameters");
                double width = physical_params.at("width").get<double>();
                double length = physical_params.at("length").get<double>();
                double height = physical_params.at("heightMax").get<double>();

                auto& marker_msg = initialized_markers_[0]; // amr_0 에 대한 마커 크기 설정

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

std::string readJsonFile(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "[ERROR] Failed to open file: " << file_path << std::endl;
        return "{}";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::string createOrderPayloadForAmr(int amr_idx)
{
    // 예: amr_0_order.json, amr_1_order.json, ... 이런 형태 파일명 사용
    std::string file_path = "/home/zenix/ros2_ws/src/fms_mqtt_publish/amr_" + std::to_string(amr_idx) + "_order.json";
    return readJsonFile(file_path);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fms_mqtt_publish");
    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    auto edge_marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("fms_order_edges_marker", 10);
    
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_pubs;

    for (int i = 0; i < AMR_COUNT; ++i)
    {
        std::string marker_topic = "agv_" + std::to_string(i) + "_marker";
        marker_pubs.push_back(node->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10));
    }

    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    Callback cb(pose_pub, marker_pubs, node);
    client.set_callback(cb);

    try
    {
        std::cout << "Connecting to the MQTT server..." << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "Connected!" << std::endl;

        // AMR들에 대한 MQTT 구독
        for (int i = 0; i < AMR_COUNT; ++i)
        {
            std::string topic_filter = "vda5050/agvs/amr_" + std::to_string(i) + "/#";
            client.subscribe(topic_filter, QOS)->wait();
            std::cout << "Subscribed to topic: " << topic_filter << std::endl;
        }

        // Factsheet 요청 instantAction 메시지 발행 (amr_0만 예시)
        {
            std::cout << "Publishing factsheet request instantAction message..." << std::endl;
            auto instant_msg = mqtt::make_message("vda5050/agvs/amr_0/instantActions", FACTSHEET_REQUEST_INSTANT_ACTION);
            instant_msg->set_qos(QOS);
            client.publish(instant_msg)->wait();
            std::cout << "Factsheet request message published!" << std::endl;
        }

        for (int i = 0; i < AMR_COUNT; ++i)
        {
            std::string order_topic = "vda5050/agvs/amr_" + std::to_string(i) + "/order";

            std::string payload = createOrderPayloadForAmr(i);

            auto order_msg = mqtt::make_message(order_topic, payload);
            order_msg->set_qos(QOS);
            client.publish(order_msg)->wait();

            // RViz 경로 Marker publish
            publishOrderEdgesAsLines(payload, edge_marker_pub, node);

            // sleep(5);

            std::cout << "Published order message to topic: " << order_topic << std::endl;
        }

        // 충분히 메시지 수신 대기
        std::cout << "Waiting for state messages for 10 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));

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
