#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <tuple>

struct Point {
    double x, y;
};

struct Edge {
    Point start;
    Point end;
};

Point normalize(const Point &p) {
    double len = std::hypot(p.x, p.y);
    return {p.x / len, p.y / len};
}

// std::tuple<Point, Point, Point> computeArcPoints(
//     const Edge &edge1, const Edge &edge2, double wheelbase, double max_steer_deg)
// {
//     double delta = max_steer_deg * M_PI / 180.0;
//     double Rmin = wheelbase / std::tan(delta);

//     // 교차점
//     Point P = edge1.end;

//     // edge1 방향
//     Point dir1 = normalize({edge1.end.x - edge1.start.x,
//                             edge1.end.y - edge1.start.y});
//     // edge2 방향
//     Point dir2 = normalize({edge2.end.x - edge2.start.x,
//                             edge2.end.y - edge2.start.y});

//     // 시작점 S
//     Point S = {P.x - dir1.x * Rmin, P.y - dir1.y * Rmin};
//     // 끝점 E
//     Point E = {P.x + dir2.x * Rmin, P.y + dir2.y * Rmin};
//     // 중심점 C (직각 교차 상황 전용)
//     Point C = {P.x - Rmin, P.y + Rmin};

//     return {S, E, C};
// }

std::tuple<Point, Point, Point> computeArcPoints(
    const Edge &edge1, const Edge &edge2, double wheelbase, double max_steer_deg)
{
    double delta = max_steer_deg * M_PI / 180.0;
    double Rmin = wheelbase / std::tan(delta);

    // 교차점 (코너점)
    Point P = edge1.end;

    // edge1 방향 (단위벡터)
    Point dir1 = normalize({edge1.end.x - edge1.start.x,
                            edge1.end.y - edge1.start.y});
    // edge2 방향 (단위벡터)
    Point dir2 = normalize({edge2.end.x - edge2.start.x,
                            edge2.end.y - edge2.start.y});

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
        // 평행 등 특수 케이스 (fail-safe: 그냥 반환)
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
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ArcPublisher::publishMarkers, this));
    }

private:
    void publishMarkers() {
        Edge edge1{{0,0}, {100,0}};
        Edge edge2{{100,0}, {100,100}};
        double wheelbase = 15.0;
        double maxSteer = 30.0;

        auto [S, E, C] = computeArcPoints(edge1, edge2, wheelbase, maxSteer);

        std::cout << "S : " << S.x << " " << S.y << std::endl;
        std::cout << "E : " << E.x << " " << E.y << std::endl;
        std::cout << "C : " << C.x << " " << C.y << std::endl;

        // 1. edge1 표시
        visualization_msgs::msg::Marker edge1_marker;
        edge1_marker.header.frame_id = "map";
        edge1_marker.header.stamp = this->now();
        edge1_marker.ns = "edges";
        edge1_marker.id = 1;
        edge1_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        edge1_marker.action = visualization_msgs::msg::Marker::ADD;
        edge1_marker.scale.x = 0.1;
        edge1_marker.color.r = 0.0;
        edge1_marker.color.g = 1.0;
        edge1_marker.color.b = 0.0;
        edge1_marker.color.a = 1.0;
        geometry_msgs::msg::Point p;
        p.x = edge1.start.x; p.y = edge1.start.y; p.z = 0;
        edge1_marker.points.push_back(p);
        p.x = edge1.end.x; p.y = edge1.end.y; p.z = 0;
        edge1_marker.points.push_back(p);

        // 2. edge2 표시
        visualization_msgs::msg::Marker edge2_marker = edge1_marker;
        edge2_marker.id = 2;
        edge2_marker.points.clear();
        p.x = edge2.start.x; p.y = edge2.start.y; p.z = 0;
        edge2_marker.points.push_back(p);
        p.x = edge2.end.x; p.y = edge2.end.y; p.z = 0;
        edge2_marker.points.push_back(p);

        // 3. 원호 표시
        visualization_msgs::msg::Marker arc_marker;
        arc_marker.header.frame_id = "map";
        arc_marker.header.stamp = this->now();
        arc_marker.ns = "arc";
        arc_marker.id = 3;
        arc_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        arc_marker.action = visualization_msgs::msg::Marker::ADD;
        arc_marker.scale.x = 0.1;
        arc_marker.color.r = 1.0;
        arc_marker.color.g = 0.0;
        arc_marker.color.b = 0.0;
        arc_marker.color.a = 1.0;

        double r = std::hypot(S.x - C.x, S.y - C.y);
        double theta_start = std::atan2(S.y - C.y, S.x - C.x);
        double theta_end   = std::atan2(E.y - C.y, E.x - C.x);
        if (theta_end < theta_start) theta_end += 2 * M_PI;

        int segments = 5;
        for (int i = 0; i <= segments; ++i) {
            double t = theta_start + (theta_end - theta_start) * i / segments;
            geometry_msgs::msg::Point pt;
            pt.x = C.x + r * std::cos(t);
            pt.y = C.y + r * std::sin(t);
            pt.z = 0;
            arc_marker.points.push_back(pt);
        }

        pub_->publish(edge1_marker);
        pub_->publish(edge2_marker);
        pub_->publish(arc_marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // auto node = rclcpp::Node::make_shared("fms_mqtt_publish");    
    rclcpp::spin(std::make_shared<ArcPublisher>());
    rclcpp::shutdown();
    return 0;
}