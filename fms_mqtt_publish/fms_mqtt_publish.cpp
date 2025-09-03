#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <tuple>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

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
        // 특수 케이스 (fail-safe)
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
        loadOrderFile("/home/zenix/ros2_ws/src/fms_mqtt_publish/amr_0_order.json");  // JSON 파일 로드
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ArcPublisher::publishMarkers, this));
    }

private:
    std::vector<Node_> nodes_;
    std::vector<Edge> edges_;

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
            std::cout << "node : " <<  n.id << " " << n.pos.x << " " << n.pos.y << std::endl;
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
            std::cout << "edge : " <<  e.start.id << " " << e.end.id  << std::endl;
            edges_.push_back(e);
        }
    }

    void publishMarkers() {
        int id_counter = 0;

        // 직선 에지 표시
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
            p.x = e.end.pos.x; p.y = e.end.pos.y; p.z = 0;
            line.points.push_back(p);

            pub_->publish(line);
        }

        // 에지 전환부에서 원호 표시
        double wheelbase = 15.0;
        double maxSteer = 30.0;

        for (size_t i = 0; i + 1 < edges_.size(); ++i) 
        {
            auto [S, E, C] = computeArcPoints(edges_[i], edges_[i+1], wheelbase, maxSteer);

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

            int segments = 20;
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