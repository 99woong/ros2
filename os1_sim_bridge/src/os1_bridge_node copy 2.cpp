#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>

using std::placeholders::_1;

class BridgeNode : public rclcpp::Node {
public:
    BridgeNode() : Node("isaac_to_ouster_bridge")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10,
            std::bind(&BridgeNode::callback, this, _1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/ouster_points", 10);
    }

private:
    static constexpr int N_SCAN = 32;
    static constexpr int HORIZON_SCAN = 1024;
    static constexpr float SCAN_PERIOD = 0.1f; // 10Hz

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 출력 메시지 생성
        sensor_msgs::msg::PointCloud2 out;
        out.header = msg->header;
        out.header.frame_id = "os_lidar";

        out.height = N_SCAN;
        out.width = HORIZON_SCAN;
        out.is_bigendian = false;
        out.point_step = 48;
        out.row_step = out.width * out.point_step;
        out.is_dense = false;

        // 필드 정의
        out.fields = {
            createField("x", 0, sensor_msgs::msg::PointField::FLOAT32),
            createField("y", 4, sensor_msgs::msg::PointField::FLOAT32),
            createField("z", 8, sensor_msgs::msg::PointField::FLOAT32),
            createField("intensity", 16, sensor_msgs::msg::PointField::FLOAT32),
            createField("t", 20, sensor_msgs::msg::PointField::FLOAT32),
            createField("reflectivity", 24, sensor_msgs::msg::PointField::UINT16),
            createField("ring", 26, sensor_msgs::msg::PointField::UINT16),
            createField("ambient", 28, sensor_msgs::msg::PointField::UINT16),
            createField("range", 32, sensor_msgs::msg::PointField::FLOAT32),
        };

        out.data.resize(out.row_step * out.height, 0);

        // 입력 iterator
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            float range = sqrt(x*x + y*y + z*z);
            if (range < 1e-3) continue;

            float azimuth = atan2(y, x);
            if (azimuth < 0) azimuth += 2 * M_PI;

            float elevation = atan2(z, sqrt(x*x + y*y));

            // Ouster OS1-32 vertical FOV (대략값)
            float min_elev = -16.6 * M_PI / 180.0;
            float max_elev = 16.6 * M_PI / 180.0;
            float elev_res = (max_elev - min_elev) / (N_SCAN - 1);

            int ring = (elevation - min_elev) / elev_res;
            if (ring < 0 || ring >= N_SCAN) continue;

            int col = azimuth / (2 * M_PI) * HORIZON_SCAN;
            if (col < 0 || col >= HORIZON_SCAN) continue;

            int index = ring * out.row_step + col * out.point_step;

            float t = (float)col / HORIZON_SCAN * SCAN_PERIOD;

            // 데이터 쓰기
            memcpy(&out.data[index + 0], &x, 4);
            memcpy(&out.data[index + 4], &y, 4);
            memcpy(&out.data[index + 8], &z, 4);

            float intensity = 1.0f;
            memcpy(&out.data[index + 16], &intensity, 4);

            memcpy(&out.data[index + 20], &t, 4);

            uint16_t reflectivity = 100;
            memcpy(&out.data[index + 24], &reflectivity, 2);

            uint16_t ring_u16 = ring;
            memcpy(&out.data[index + 26], &ring_u16, 2);

            uint16_t ambient = 0;
            memcpy(&out.data[index + 28], &ambient, 2);

            memcpy(&out.data[index + 32], &range, 4);
        }

        pub_->publish(out);
    }

    sensor_msgs::msg::PointField createField(
        const std::string &name, int offset, uint8_t datatype)
    {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = datatype;
        f.count = 1;
        return f;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgeNode>());
    rclcpp::shutdown();
    return 0;
}