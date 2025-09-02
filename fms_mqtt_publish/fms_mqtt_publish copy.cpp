#include <iostream>
#include <string>
#include <mqtt/async_client.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define AMR_COUNT 2

using json = nlohmann::json;

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("fms_client");
const std::string ORDER_TOPIC_AMR0("vda5050/agvs/amr_0/order");
const std::string STATE_TOPIC_AMR0("vda5050/agvs/amr_0/state");
const std::string VISUALIZATION_TOPIC_AMR0("vda5050/agvs/amr_0/visualization");
const std::string ORDER_TOPIC_AMR1("vda5050/agvs/amr_1/order");
const std::string STATE_TOPIC_AMR1("vda5050/agvs/amr_1/state");
const std::string VISUALIZATION_TOPIC_AMR1("vda5050/agvs/amr_1/visualization");
const std::string FACTS_TOPIC("vda5050/agvs/amr_0/instantActions");
const int QOS = 1;


// 토픽 이름 배열
const std::string VISUALIZATION_TOPICS[AMR_COUNT] = {
    "vda5050/agvs/amr_0/visualization",
    "vda5050/agvs/amr_1/visualization"
};
const std::string MARKER_TOPICS[AMR_COUNT] = {
    "agv_0_marker",
    "agv_1_marker"
};

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

// MQTT 콜백 핸들러 클래스 정의
class Callback : public virtual mqtt::callback
{
public:
 Callback(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub,
             const std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>& marker_pubs,
             rclcpp::Node::SharedPtr node)
        : pose_pub_(pose_pub), marker_pubs_(marker_pubs), node_(node), factsheet_received_(false)
    {
        for (auto& marker_pub : marker_pubs_)
        {
            visualization_msgs::msg::Marker marker_msg;
            marker_msg.header.frame_id = "map";
            marker_msg.ns = "amr_model";
            marker_msg.id = 0;
            marker_msg.type = visualization_msgs::msg::Marker::CUBE;
            marker_msg.action = visualization_msgs::msg::Marker::ADD;
            marker_msg.color.a = 0.5; // 투명도
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

        for (size_t i = 0; i < visualization_topics_.size(); ++i)
        {
            if (topic == visualization_topics_[i])
            {
                try
                {
                    auto j = json::parse(payload);
                    auto pose_json = j.at("pose");
                    double x = pose_json.at("x").get<double>();
                    double y = pose_json.at("y").get<double>();
                    double theta = pose_json.at("theta").get<double>();

                    // PoseStamped publish
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.stamp = node_->get_clock()->now();
                    pose.header.frame_id = "map";
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0, 0, theta);
                    pose.pose.orientation = tf2::toMsg(q);
                    pose_pub_->publish(pose);

                    // Marker publish
                    auto& marker_msg = initialized_markers_[i];
                    marker_msg.header.stamp = node_->get_clock()->now();
                    marker_msg.pose = pose.pose;
                    marker_msg.ns = "amr_model_" + std::to_string(i);
                    marker_pubs_[i]->publish(marker_msg);

                    std::cout << "[ROS2] Published marker for AMR " << i << std::endl;
                }
                catch (...)
                {
                    std::cerr << "Failed to parse visualization json for AMR " << i << std::endl;
                }
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
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker marker_msg_;
    bool factsheet_received_;    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fms_mqtt_publish");

    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    // auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("agv_marker", 10);
    
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_pubs;
    for (int i = 0; i < AMR_COUNT; ++i)
    {
        std::string marker_topic = "agv_" + std::to_string(i) + "_marker";
        marker_pubs.push_back(node->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10));
    }
    
    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    Callback cb(pose_pub, marker_pub, node);
    client.set_callback(cb);

    try
    {
        std::cout << "Connecting to the MQTT server..." << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "Connected!" << std::endl;

        client.subscribe("vda5050/agvs/amr_0/#", QOS)->wait();
        client.subscribe("vda5050/agvs/amr_1/#", QOS)->wait();

        // Factsheet 요청 instantAction 메시지 발행
        {
            std::cout << "Publishing factsheet request instantAction message..." << std::endl;
            auto instant_msg = mqtt::make_message("vda5050/agvs/amr_0/instantActions", FACTSHEET_REQUEST_INSTANT_ACTION);
            instant_msg->set_qos(QOS);
            client.publish(instant_msg)->wait();
            std::cout << "Factsheet request message published!" << std::endl;
        }
        
        // sleep(1);

        // Order 메시지 발행
        std::string payload0 = R"({
              "headerId": "header_test",
              "timestamp": 1650000000,
              "orderId": "order_test_1",
              "nodes": [
                {"nodeId": "N1", "sequenceId": 0, "nodePosition": {"x":0.0, "y":0.0, "theta":0.0}},
                {"nodeId": "N2", "sequenceId": 1, "nodePosition": {"x":10.0, "y":0.0, "theta":1.57}},
                {"nodeId": "N3", "sequenceId": 2, "nodePosition": {"x":10.0, "y":10.0, "theta":3.14}},
                {"nodeId": "N4", "sequenceId": 3, "nodePosition": {"x":0.0, "y":10.0, "theta":4.71}},
                {"nodeId": "N5", "sequenceId": 4, "nodePosition": {"x":0.0, "y":0.0, "theta":0.0}}
              ],
              "edges": [
                {"edgeId": "E1", "sequenceId": 0, "startNodeId": "N1", "endNodeId": "N2", "maxSpeed": 1.0},
                {"edgeId": "E2", "sequenceId": 1, "startNodeId": "N2", "endNodeId": "N3", "maxSpeed": 4.0},
                {"edgeId": "E3", "sequenceId": 2, "startNodeId": "N3", "endNodeId": "N4", "maxSpeed": 4.0},
                {"edgeId": "E4", "sequenceId": 3, "startNodeId": "N4", "endNodeId": "N5", "maxSpeed": 1.0}
              ]
            })";

        std::string payload1 = R"({
              "headerId": "header_test",
              "timestamp": 1650000000,
              "orderId": "order_test_1",
              "nodes": [
                {"nodeId": "N1", "sequenceId": 0, "nodePosition": {"x":0.0, "y":0.0, "theta":0.0}},
                {"nodeId": "N2", "sequenceId": 1, "nodePosition": {"x":10.0, "y":0.0, "theta":1.57}},
                {"nodeId": "N3", "sequenceId": 2, "nodePosition": {"x":10.0, "y":10.0, "theta":3.14}},
                {"nodeId": "N4", "sequenceId": 3, "nodePosition": {"x":0.0, "y":10.0, "theta":4.71}},
                {"nodeId": "N5", "sequenceId": 4, "nodePosition": {"x":0.0, "y":0.0, "theta":0.0}}
              ],
              "edges": [
                {"edgeId": "E1", "sequenceId": 0, "startNodeId": "N1", "endNodeId": "N2", "maxSpeed": 1.0},
                {"edgeId": "E2", "sequenceId": 1, "startNodeId": "N2", "endNodeId": "N3", "maxSpeed": 4.0},
                {"edgeId": "E3", "sequenceId": 2, "startNodeId": "N3", "endNodeId": "N4", "maxSpeed": 4.0},
                {"edgeId": "E4", "sequenceId": 3, "startNodeId": "N4", "endNodeId": "N5", "maxSpeed": 1.0}
              ]
            })";

            // std::string payload = R"({
            //     "headerId": "header_test",
            //     "timestamp": 1650000000,
            //     "orderId": "order_test_1",
            //     "nodes": [
            //         {"nodeId": "N1", "sequenceId": 0, "nodePosition": {"x":0.0, "y":0.0}},
            //         {"nodeId": "N2", "sequenceId": 1, "nodePosition": {"x":111.0, "y":0.0}},
            //         {"nodeId": "N3", "sequenceId": 2, "nodePosition": {"x":100.0, "y":100.0}},
            //         {"nodeId": "N4", "sequenceId": 3, "nodePosition": {"x":0.0, "y":100.0}},
            //         {"nodeId": "N5", "sequenceId": 4, "nodePosition": {"x":0.0, "y":0.0}}
            //     ],
            //     "edges": [
            //         {"edgeId": "E1", "sequenceId": 0, "startNodeId": "N1", "endNodeId": "N2", "maxSpeed": 1.0},
            //         {"edgeId": "E2", "sequenceId": 1, "startNodeId": "N2", "endNodeId": "N3", "maxSpeed": 4.0,
            //         "turnCenter": {"x": 70.0, "y": 30.0}},
            //         {"edgeId": "E3", "sequenceId": 2, "startNodeId": "N3", "endNodeId": "N4", "maxSpeed": 4.0,
            //         "turnCenter": {"x": 70.0, "y": 70.0}},
            //         {"edgeId": "E4", "sequenceId": 3, "startNodeId": "N4", "endNodeId": "N5", "maxSpeed": 1.0,
            //         "turnCenter": {"x": 30.0, "y": 70.0}}
            //     ]
            //     })";

        // auto msg = mqtt::make_message(ORDER_TOPIC, payload);
        auto msg0 = mqtt::make_message(ORDER_TOPIC_AMR0, payload0);
        msg0->set_qos(QOS);
        auto msg1 = mqtt::make_message(ORDER_TOPIC_AMR1, payload1);
        msg1->set_qos(QOS);

        std::cout << "Publishing message to topic: " << ORDER_TOPIC_AMR0 << std::endl;
        client.publish(msg0)->wait_for(std::chrono::seconds(10));
        std::cout << "Publishing message to topic: " << ORDER_TOPIC_AMR1 << std::endl;
        client.publish(msg1)->wait_for(std::chrono::seconds(10));
        std::cout << "Message published!" << std::endl;

        std::cout << "Waiting for state messages for 10 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));

        // ROS 2 spin
        rclcpp::spin(node);

        // 연결 종료
        client.unsubscribe("vda5050/agvs/amr_0/#")->wait();
        client.disconnect()->wait();
        std::cout << "Disconnected." << std::endl;
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << "Error: " << exc.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}