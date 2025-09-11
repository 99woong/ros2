#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <tuple>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <unistd.h>  // usleep

#define AMR_COUNT 2

using json = nlohmann::json;

struct Point 
{
    double x, y;
};

struct Node_ 
{
    std::string id;
    Point pos;
};

struct Edge 
{
    std::string id;
    Node_ start;
    Node_ end;
};

Point normalize(const Point &p) 
{
    double len = std::hypot(p.x, p.y);
    return {p.x / len, p.y / len};
}

std::tuple<Point, Point, Point> computeArcPoints(const Edge &edge1, const Edge &edge2, double wheelbase, double max_steer_deg)
{
    double delta = max_steer_deg * M_PI / 180.0;
    double Rmin = wheelbase / std::tan(delta);

    // 에지교차점
    Point P = edge1.end.pos;

    // edge1 방향
    Point dir1 = normalize({edge1.end.pos.x - edge1.start.pos.x,
                            edge1.end.pos.y - edge1.start.pos.y});
    // edge2 방향
    Point dir2 = normalize({edge2.end.pos.x - edge2.start.pos.x,
                            edge2.end.pos.y - edge2.start.pos.y});

    // 원호 시작점
    Point S = {P.x - dir1.x * Rmin, P.y - dir1.y * Rmin};
    // 원호 끝점
    Point E = {P.x + dir2.x * Rmin, P.y + dir2.y * Rmin};

    // 각각의 법선벡터
    Point n1{-dir1.y, dir1.x};
    Point n2{-dir2.y, dir2.x};

    // 직선 S + λ*n1 과 E + μ*n2 의 교차점->중심점 C
    double A1 = n1.x, B1 = -n2.x;
    double C1 = E.x - S.x;
    double A2 = n1.y, B2 = -n2.y;
    double C2 = E.y - S.y;

    double det = A1 * B2 - A2 * B1;
    if (std::fabs(det) < 1e-9) 
    {
        // 특수 케이스
        return {S, E, P};
    }

    double lambda = (C1 * B2 - C2 * B1) / det;
    Point C = {S.x + lambda * n1.x, S.y + lambda * n1.y};

    return {S, E, C};
}

class ArcPublisher : public rclcpp::Node 
{
public:
    ArcPublisher() : Node("map_generator") 
    {
        for(int i=0;i<2;i++)
        {
            std::string input_file_path = "/home/zenix/ros2_ws/src/map_generator/maps/amr_" + std::to_string(i) + "_order.json";            
            std::string output_file_path = "/home/zenix/ros2_ws/src/map_generator/maps/amr_" + std::to_string(i) + "_order_arc.json";            
            loadOrderFile(input_file_path);
            generateArcOrder(output_file_path);
        }
    }

private:
    std::vector<Node_> nodes_;
    std::vector<Edge> edges_;
    json new_order_= json::object();

    void loadOrderFile(const std::string &filename) 
    {
        std::ifstream file(filename);
        if (!file.is_open()) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", filename.c_str());
            return;
        }
        json j= json::object();
        file >> j;

        // 노드 로드
        for (auto &jn : j["nodes"]) 
        {
            Node_ n;
            n.id = jn["nodeId"];
            n.pos = {jn["nodePosition"]["x"], jn["nodePosition"]["y"]};
            nodes_.push_back(n);
        }

        auto findNode = [&](const std::string &id) 
        {
            for (auto &n : nodes_) {
                if (n.id == id) return n;
            }
            return Node_{};
        };

        // 에지 로드
        for (auto &je : j["edges"]) 
        {
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

    double calcEdgeDirection(const Edge& edge) 
    {
        double dx = edge.end.pos.x - edge.start.pos.x;
        double dy = edge.end.pos.y - edge.start.pos.y;
        std::cout << "calcEdge : " << edge.start.pos.x << " " << edge.start.pos.y <<std::endl;
        std::cout << "calcEdge : " << edge.end.pos.x << " " << edge.end.pos.y <<std::endl;
        std::cout << "calcEdge : " << dx << " " << dy << " " << std::atan2(dy, dx) << std::endl;
        return std::atan2(dy, dx);  // 라디안 값, -π ~ π 범위
    }

    void generateArcOrder(const std::string &out_filename) 
    {
        double wheelbase = 15.0;
        double maxSteer = 30.0;
        int node_seq = 0, edge_seq = 0;
        int arc_idx = 1;
        auto round2 = [](double x) { return std::round(x * 100.0) / 100.0; };

        // 기존 노드 먼저 추가
        for (auto &n : nodes_) 
        {
            json jn= json::object();
            jn["nodeId"] = n.id;
            jn["sequenceId"] = node_seq++;
            jn["nodePosition"] = {{"x", round2(n.pos.x)}, {"y", round2(n.pos.y)}, {"theta", round2(0.0)}};
            new_order_["nodes"].push_back(jn);
        }


        int line_seq = 0; 
        int arc_seq  = 1; 

        // 새 에지/노드 구성
        for (size_t i = 0; i < edges_.size(); ++i) 
        {
            if (1) 
            {
                auto [S, E, C] = computeArcPoints(edges_[i], edges_[(i+1)%edges_.size()], wheelbase, maxSteer);

                std::string ns = "NS" + std::to_string(arc_idx);
                std::string ne = "NE" + std::to_string(arc_idx);
                std::string nc = "NC" + std::to_string(arc_idx);

                edges_[i].end.id = ns;
                edges_[(i+1)%edges_.size()].start.id = ne;

                // 이전 직선 에지 방향 (원호 시작 노드 theta)
                size_t prev_i = (i == 0) ? edges_.size() - 1 : i - 1;
                std::cout << "pedge_start : " << edges_[prev_i].start.pos.x << " " << edges_[prev_i].start.pos.y <<std::endl;
                std::cout << "pedge_end : " << edges_[prev_i].end.pos.x << " " << edges_[prev_i].end.pos.y <<std::endl;
                double prev_theta = calcEdgeDirection(edges_[prev_i]);

                // 다음 직선 에지 방향 (원호 종료 노드 theta)
                double next_theta = calcEdgeDirection(edges_[(i+1) % edges_.size()]);

                // 노드 추가
                new_order_["nodes"].push_back({{"nodeId", ns}, {"sequenceId", node_seq++},
                                               {"nodePosition", {{"x", round2(S.x)}, {"y", round2(S.y)}, {"theta", prev_theta}}}});
                new_order_["nodes"].push_back({{"nodeId", ne}, {"sequenceId", node_seq++},
                                               {"nodePosition", {{"x", round2(E.x)}, {"y", round2(E.y)}, {"theta", next_theta}}}});
                // new_order_["nodes"].push_back({{"nodeId", nc}, {"sequenceId", node_seq++},
                //                                {"nodePosition", {{"x", C.x}, {"y", C.y}, {"theta", 0.0}}}});

                // 원호 에지 추가
                new_order_["edges"].push_back({{"edgeId", "E" + std::to_string(arc_seq)},
                                               {"sequenceId", arc_seq},
                                               {"startNodeId", ns},
                                               {"endNodeId", ne},
                                               {"turnCenter", {{"x", round2(C.x)}, {"y", round2(C.y)}}},
                                               {"maxSpeed", 2.0}});
                arc_idx++;
                arc_seq += 2;
            }
        }

        for (size_t i = 0; i < edges_.size(); ++i) 
        {
            auto &e = edges_[i];
            // 직선 에지
            json je = json::object();;
            je["edgeId"] = "E" + std::to_string(line_seq);
            je["sequenceId"] = line_seq;
            je["startNodeId"] = e.start.id;
            je["endNodeId"] = e.end.id;
            je["maxSpeed"] = 2.0;  // 예시 속도
            new_order_["edges"].push_back(je);
            line_seq += 2;
        }

        // JSON 파일 저장
        std::ofstream out(out_filename);
        out << new_order_.dump(4);
        out.close();
        RCLCPP_INFO(this->get_logger(), "Generated new order file: %s", out_filename.c_str());
    }
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArcPublisher>();
    // rclcpp::spin(std::make_shared<ArcPublisher>());
    rclcpp::shutdown();
    return 0;
}
