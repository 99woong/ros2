#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <tuple>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <unistd.h>  // usleep

using json = nlohmann::json;

struct Point {
    double x, y;
};

struct Node_ {
    std::string id;
    Point pos;
};

struct Edge {
    std::string id;
    Node_ start;
    Node_ end;
};

Point normalize(const Point &p) {
    double len = std::hypot(p.x, p.y);
    return {p.x / len, p.y / len};
}

std::tuple<Point, Point, Point> computeArcPoints(
    const Edge &edge1, const Edge &edge2, double wheelbase, double max_steer_deg)
{
    double delta = max_steer_deg * M_PI / 180.0;
    double Rmin = wheelbase / std::tan(delta);

    // 교차점 (코너점)
    Point P = edge1.end.pos;

    // edge1 방향 (단위벡터)
    Point dir1 = normalize({edge1.end.pos.x - edge1.start.pos.x,
                            edge1.end.pos.y - edge1.start.pos.y});
    // edge2 방향 (단위벡터)
    Point dir2 = normalize({edge2.end.pos.x - edge2.start.pos.x,
                            edge2.end.pos.y - edge2.start.pos.y});

    // 시작점 (edge1 직선 위)
    Point S = {P.x - dir1.x * Rmin, P.y - dir1.y * Rmin};
    // 끝점 (edge2 직선 위)
    Point E = {P.x + dir2.x * Rmin, P.y + dir2.y * Rmin};

    // 각각의 법선벡터 (좌회전 기준)
    Point n1{-dir1.y, dir1.x};
    Point n2{-dir2.y, dir2.x};

    // 직선 S + λ*n1 과 E + μ*n2 의 교차점 → 중심점 C
    double A1 = n1.x, B1 = -n2.x;
    double C1 = E.x - S.x;
    double A2 = n1.y, B2 = -n2.y;
    double C2 = E.y - S.y;

    double det = A1 * B2 - A2 * B1;
    if (std::fabs(det) < 1e-9) {
        // 특수 케이스
        return {S, E, P};
    }

    double lambda = (C1 * B2 - C2 * B1) / det;
    Point C = {S.x + lambda * n1.x, S.y + lambda * n1.y};

    return {S, E, C};
}

class ArcPublisher : public rclcpp::Node {
public:
    ArcPublisher() : Node("fms_mqtt_publish") {
        pub_ = this->create_publisher<visualization_msgs::msg::Marker>("arc_marker", 10);
        loadOrderFile("/home/zenix/ros2_ws/src/fms_mqtt_publish/amr_0_order.json");
        generateArcOrder("/home/zenix/ros2_ws/src/fms_mqtt_publish/amr_0_order_arc.json");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ArcPublisher::publishMarkers, this));
    }

private:
    std::vector<Node_> nodes_;
    std::vector<Edge> edges_;
    json new_order_;

    void loadOrderFile(const std::string &filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", filename.c_str());
            return;
        }
        json j;
        file >> j;

        // 노드 로드
        for (auto &jn : j["nodes"]) {
            Node_ n;
            n.id = jn["nodeId"];
            n.pos = {jn["nodePosition"]["x"], jn["nodePosition"]["y"]};
            nodes_.push_back(n);
        }

        auto findNode = [&](const std::string &id) {
            for (auto &n : nodes_) {
                if (n.id == id) return n;
            }
            return Node_{};
        };

        // 에지 로드
        for (auto &je : j["edges"]) {
            Edge e;
            e.id = je["edgeId"];
            e.start = findNode(je["startNodeId"]);
            e.end   = findNode(je["endNodeId"]);
            edges_.push_back(e);
        }

        new_order_ = j;  // 헤더, 기존 노드/에지 복사
        new_order_["orderId"] = "order_test_arc";
        new_order_["nodes"].clear();
        new_order_["edges"].clear();
    }

    void generateArcOrder(const std::string &out_filename) {
        double wheelbase = 15.0;
        double maxSteer = 30.0;
        int node_seq = 0, edge_seq = 0;
        int arc_idx = 1;

        // 기존 노드 먼저 추가
        for (auto &n : nodes_) {
            json jn;
            jn["nodeId"] = n.id;
            jn["sequenceId"] = node_seq++;
            jn["nodePosition"] = {{"x", n.pos.x}, {"y", n.pos.y}, {"theta", 0.0}};
            new_order_["nodes"].push_back(jn);
        }

        // 새 에지/노드 구성
        for (size_t i = 0; i < edges_.size(); ++i) 
        {
            // auto &e = edges_[i];
            // // 직선 에지
            // json je;
            // je["edgeId"] = "E" + std::to_string(edge_seq+1);
            // je["sequenceId"] = edge_seq++;
            // je["startNodeId"] = e.start.id;
            // je["endNodeId"] = e.end.id;
            // je["maxSpeed"] = 2.0;  // 예시 속도
            // new_order_["edges"].push_back(je);

            // 원호 계산 (다음 에지가 존재할 때만)
            // if (i + 1 < edges_.size()) 
            if (1) 
            {
                auto [S, E, C] = computeArcPoints(edges_[i], edges_[(i+1)%edges_.size()], wheelbase, maxSteer);

                std::string ns = "NS" + std::to_string(arc_idx);
                std::string ne = "NE" + std::to_string(arc_idx);
                std::string nc = "NC" + std::to_string(arc_idx);

                edges_[i].end.id = ns;
                edges_[(i+1)%edges_.size()].start.id = ne;

                // 노드 추가
                new_order_["nodes"].push_back({{"nodeId", ns}, {"sequenceId", node_seq++},
                                               {"nodePosition", {{"x", S.x}, {"y", S.y}, {"theta", 0.0}}}});
                new_order_["nodes"].push_back({{"nodeId", ne}, {"sequenceId", node_seq++},
                                               {"nodePosition", {{"x", E.x}, {"y", E.y}, {"theta", 0.0}}}});
                new_order_["nodes"].push_back({{"nodeId", nc}, {"sequenceId", node_seq++},
                                               {"nodePosition", {{"x", C.x}, {"y", C.y}, {"theta", 0.0}}}});

                // 원호 에지 추가
                new_order_["edges"].push_back({{"edgeId", "E" + std::to_string(edge_seq+1)},
                                               {"sequenceId", edge_seq++},
                                               {"startNodeId", ns},
                                               {"endNodeId", ne},
                                               {"turnCenter", nc},
                                               {"maxSpeed", 2.0}});
                arc_idx++;
            }
        }

        for (size_t i = 0; i < edges_.size(); ++i) 
        {
            auto &e = edges_[i];
            // 직선 에지
            json je;
            je["edgeId"] = "E" + std::to_string(edge_seq+1);
            je["sequenceId"] = edge_seq++;
            je["startNodeId"] = e.start.id;
            je["endNodeId"] = e.end.id;
            je["maxSpeed"] = 2.0;  // 예시 속도
            new_order_["edges"].push_back(je);
        }

        // JSON 파일 저장
        std::ofstream out(out_filename);
        out << new_order_.dump(4);
        out.close();
        RCLCPP_INFO(this->get_logger(), "Generated new order file: %s", out_filename.c_str());
    }

    void publishMarkers() {
        int id_counter = 0;

        // 기존 직선 에지 출력
        for (auto &e : edges_) {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = this->now();
            line.ns = "edges";
            line.id = id_counter++;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.1;
            line.color.r = 0.0;
            line.color.g = 1.0;
            line.color.b = 0.0;
            line.color.a = 1.0;

            geometry_msgs::msg::Point p;
            p.x = e.start.pos.x; p.y = e.start.pos.y; p.z = 0;
            line.points.push_back(p);
            usleep(100000);
            p.x = e.end.pos.x; p.y = e.end.pos.y; p.z = 0;
            line.points.push_back(p);

            pub_->publish(line);
        }

        // 원호 출력
        double wheelbase = 15.0;
        double maxSteer = 30.0;
        for (size_t i = 0; i < edges_.size(); ++i) {
            auto [S, E, C] = computeArcPoints(edges_[i], edges_[(i+1)%edges_.size()], wheelbase, maxSteer);
            std::cout << i << " " << S.x << " " << S.y << " " << C.x << " " << C.y <<" " << E.x << " " << E.y << std::endl;

            visualization_msgs::msg::Marker arc;
            arc.header.frame_id = "map";
            arc.header.stamp = this->now();
            arc.ns = "arc";
            arc.id = id_counter++;
            arc.type = visualization_msgs::msg::Marker::LINE_STRIP;
            arc.action = visualization_msgs::msg::Marker::ADD;
            arc.scale.x = 0.1;
            arc.color.r = 1.0;
            arc.color.g = 0.0;
            arc.color.b = 0.0;
            arc.color.a = 1.0;

            double r = std::hypot(S.x - C.x, S.y - C.y);
            double theta_start = std::atan2(S.y - C.y, S.x - C.x);
            double theta_end   = std::atan2(E.y - C.y, E.x - C.x);
            if (theta_end < theta_start) theta_end += 2 * M_PI;

            int segments = 10;
            for (int k = 0; k <= segments; ++k) {
                double t = theta_start + (theta_end - theta_start) * k / segments;
                geometry_msgs::msg::Point pt;
                pt.x = C.x + r * std::cos(t);
                pt.y = C.y + r * std::sin(t);
                pt.z = 0;
                arc.points.push_back(pt);
            }
            pub_->publish(arc);
        }
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArcPublisher>());
    rclcpp::shutdown();
    return 0;
}
