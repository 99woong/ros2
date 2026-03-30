/**
 * os1_bridge_node.cpp
 * ====================
 * Isaac Sim /point_cloud (x,y,z only, height=1, 가변 width)를
 * 실제 Ouster OS1-32 PointCloud2 포맷으로 변환하여 발행합니다.
 *
 * 실측 OS1-32 필드 레이아웃 (point_step=48):
 * ┌────────┬──────┬──────────┬────────────────────────────┐
 * │ offset │ size │ datatype │ field                      │
 * ├────────┼──────┼──────────┼────────────────────────────┤
 * │   0    │  4   │ float32  │ x                          │
 * │   4    │  4   │ float32  │ y                          │
 * │   8    │  4   │ float32  │ z                          │
 * │  12    │  4   │ (gap)    │ -                          │
 * │  16    │  4   │ float32  │ intensity                  │
 * │  20    │  4   │ uint32   │ t  (scan 내 상대시간, ns)  │
 * │  24    │  2   │ uint16   │ reflectivity               │
 * │  26    │  2   │ uint16   │ ring (채널 번호 0~31)      │
 * │  28    │  2   │ uint16   │ ambient                    │
 * │  30    │  2   │ (gap)    │ -                          │
 * │  32    │  4   │ uint32   │ range (mm)                 │
 * │  36    │ 12   │ (pad)    │ -                          │
 * └────────┴──────┴──────────┴────────────────────────────┘
 * row_step = 1024 * 48 = 49152 bytes
 * total    = 32 * 49152 = 1,572,864 bytes ≈ 1.57MB
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>

// ── OS1-32 스펙 ──────────────────────────────────────────────────
static constexpr uint32_t N_RINGS    = 32;
static constexpr uint32_t N_COLS     = 1024;
static constexpr uint32_t POINT_STEP = 48;
static constexpr uint32_t ROW_STEP   = N_COLS * POINT_STEP;          // 49152
static constexpr uint32_t PACKET_SIZE = N_RINGS * ROW_STEP;          // 1,572,864
static constexpr double   SCAN_PERIOD_NS = 100'000'000.0;            // 10Hz = 100ms

// ── 실측 기반 필드 오프셋 ────────────────────────────────────────
static constexpr size_t OFF_X            =  0;  // float32
static constexpr size_t OFF_Y            =  4;  // float32
static constexpr size_t OFF_Z            =  8;  // float32
// offset 12: gap (4bytes)
static constexpr size_t OFF_INTENSITY    = 16;  // float32
static constexpr size_t OFF_T            = 20;  // uint32, ns
static constexpr size_t OFF_REFLECTIVITY = 24;  // uint16
static constexpr size_t OFF_RING         = 26;  // uint16
static constexpr size_t OFF_AMBIENT      = 28;  // uint16
// offset 30: gap (2bytes)
static constexpr size_t OFF_RANGE        = 32;  // uint32, mm
// offset 36~47: padding

// ── OS1-32 공식 elevation angle LUT (도, 채널 0~31) ─────────────
static constexpr std::array<double, 32> ELEV_DEG = {
     16.611,  15.703,  14.785,  13.857,
     12.919,  11.972,  11.014,  10.047,
      9.069,   8.082,   7.085,   6.078,
      5.062,   4.038,   3.005,   1.964,
      0.915,  -0.135,  -1.185,  -2.234,
     -3.284,  -4.334,  -5.384,  -6.434,
     -7.485,  -8.535,  -9.586, -10.636,
    -11.686, -12.737, -13.788, -14.838,
};

static std::array<double, 32> make_tan_lut()
{
    std::array<double, 32> t{};
    for (size_t i = 0; i < 32; ++i)
        t[i] = std::tan(ELEV_DEG[i] * M_PI / 180.0);
    return t;
}
static const auto TAN_LUT = make_tan_lut();

// ── 헬퍼 함수들 ──────────────────────────────────────────────────

// elevation 기반 ring (0~31)
inline uint16_t calc_ring(float x, float y, float z)
{
    const double d = std::sqrt((double)x*x + (double)y*y);
    if (d < 1e-6) return 0;
    const double tan_el = (double)z / d;
    int best = 0;
    double best_diff = std::abs(tan_el - TAN_LUT[0]);
    for (int i = 1; i < 32; ++i) {
        double diff = std::abs(tan_el - TAN_LUT[i]);
        if (diff < best_diff) { best_diff = diff; best = i; }
    }
    return static_cast<uint16_t>(best);
}

// azimuth 기반 column 인덱스 (0~1023)
inline uint16_t calc_col(float x, float y)
{
    double azi = std::atan2((double)y, (double)x) * 180.0 / M_PI;
    if (azi < 0.0) azi += 360.0;
    return static_cast<uint16_t>(
        static_cast<int>(azi * N_COLS / 360.0) % static_cast<int>(N_COLS));
}

// azimuth 기반 scan 내 상대시간 (uint32, ns)
// OS1-32: CCW 회전, azimuth 0°=t0, 360°=100ms
inline uint32_t calc_t_ns(float x, float y)
{
    double azi = std::atan2((double)y, (double)x) * 180.0 / M_PI;
    if (azi < 0.0) azi += 360.0;
    return static_cast<uint32_t>((azi / 360.0) * SCAN_PERIOD_NS);
}

// 3D 거리 → mm (uint32)
inline uint32_t calc_range_mm(float x, float y, float z)
{
    double d = std::sqrt((double)x*x + (double)y*y + (double)z*z);
    return static_cast<uint32_t>(std::min(d * 1000.0, 4294967295.0));
}

// ─────────────────────────────────────────────────────────────────
class OS1BridgeNode : public rclcpp::Node
{
public:
    OS1BridgeNode() : Node("os1_sim_bridge")
    {
        this->declare_parameter<std::string>("input_topic",  "/point_cloud");
        this->declare_parameter<std::string>("output_topic", "/os1_cloud_node/points");
        this->declare_parameter<std::string>("frame_id",     "os_lidar");

        input_topic_  = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        frame_id_     = this->get_parameter("frame_id").as_string();

        build_output_template();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&OS1BridgeNode::on_cloud, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, 10);

        RCLCPP_INFO(get_logger(),
            "OS1 Bridge ready: %s → %s  [%ux%u, %ubytes/pt, %.2fKB]",
            input_topic_.c_str(), output_topic_.c_str(),
            N_COLS, N_RINGS, POINT_STEP, PACKET_SIZE / 1024.0f);
    }

private:
    // 실측 포맷과 동일한 출력 템플릿 구성
    void build_output_template()
    {
        tmpl_.height       = N_RINGS;
        tmpl_.width        = N_COLS;
        tmpl_.point_step   = POINT_STEP;
        tmpl_.row_step     = ROW_STEP;
        tmpl_.is_bigendian = false;
        tmpl_.is_dense     = false;
        tmpl_.data.assign(PACKET_SIZE, 0);

        // 실측 데이터와 완전히 동일한 필드 정의
        using PF = sensor_msgs::msg::PointField;
        auto f = [](const std::string& name, uint32_t off, uint8_t dt) {
            sensor_msgs::msg::PointField field;
            field.name = name; field.offset = off;
            field.datatype = dt; field.count = 1;
            return field;
        };
        tmpl_.fields = {
            f("x",            OFF_X,            PF::FLOAT32),  // datatype=7
            f("y",            OFF_Y,            PF::FLOAT32),  // datatype=7
            f("z",            OFF_Z,            PF::FLOAT32),  // datatype=7
            f("intensity",    OFF_INTENSITY,    PF::FLOAT32),  // datatype=7, offset=16
            f("t",            OFF_T,            PF::UINT32),   // datatype=6, offset=20
            f("reflectivity", OFF_REFLECTIVITY, PF::UINT16),   // datatype=4, offset=24
            f("ring",         OFF_RING,         PF::UINT16),   // datatype=4, offset=26
            f("ambient",      OFF_AMBIENT,      PF::UINT16),   // datatype=4, offset=28
            f("range",        OFF_RANGE,        PF::UINT32),   // datatype=6, offset=32
        };
    }

    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr in)
    {
        if (in->data.empty() || in->point_step < 12) return;

        RCLCPP_INFO(get_logger(), "input points: %u",
        in->width * in->height);



        // 출력 메시지 생성
        auto out = std::make_shared<sensor_msgs::msg::PointCloud2>(tmpl_);
        out->header          = in->header;
        out->header.frame_id = frame_id_;
        std::fill(out->data.begin(), out->data.end(), 0);

        // 입력 필드 오프셋 파싱
        uint32_t ix = 0, iy = 4, iz = 8;
        for (const auto& fld : in->fields) {

               RCLCPP_INFO(get_logger(), "field: %s offset: %u",
                fld.name.c_str(), fld.offset);

            if (fld.name == "x") ix = fld.offset;
            if (fld.name == "y") iy = fld.offset;
            if (fld.name == "z") iz = fld.offset;
        }

        const uint32_t n   = in->width * in->height;
        const uint32_t stp = in->point_step;
        uint32_t written = 0, skipped = 0;

        for (uint32_t i = 0; i < n; ++i)
        {
            const uint8_t* src = in->data.data() + i * stp;
            float x, y, z;
            std::memcpy(&x, src + ix, 4);
            std::memcpy(&y, src + iy, 4);
            std::memcpy(&z, src + iz, 4);

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (x == 0.f && y == 0.f && z == 0.f) continue;

            const uint16_t ring  = calc_ring(x, y, z);
            const uint16_t col   = calc_col(x, y);
            const uint32_t t_ns  = calc_t_ns(x, y);
            const uint32_t range = calc_range_mm(x, y, z);

            // 2D 배치 인덱스: [ring][col]
            const size_t idx = (static_cast<size_t>(ring) * N_COLS + col) * POINT_STEP;

            // 셀 충돌: 기존 range 와 비교, 가까운 포인트 우선
            uint32_t prev_range = 0;
            std::memcpy(&prev_range, out->data.data() + idx + OFF_RANGE, 4);
            if (prev_range > 0 && range >= prev_range) { ++skipped; continue; }

            uint8_t* dst = out->data.data() + idx;
            std::memcpy(dst + OFF_X,            &x,     4);
            std::memcpy(dst + OFF_Y,            &y,     4);
            std::memcpy(dst + OFF_Z,            &z,     4);
            // intensity: 0 (Isaac Sim 미지원)
            std::memcpy(dst + OFF_T,            &t_ns,  4);
            std::memcpy(dst + OFF_RING,         &ring,  2);
            std::memcpy(dst + OFF_RANGE,        &range, 4);
            ++written;
        }

        pub_->publish(*out);

        if (++msg_count_ % 50 == 1) {
            RCLCPP_INFO(get_logger(),
                "[#%u] in=%u pts → %u/%u cells (%.1f%%) skip=%u",
                msg_count_, n, written, N_RINGS * N_COLS,
                written * 100.f / (N_RINGS * N_COLS), skipped);
        }
    }

    std::string input_topic_, output_topic_, frame_id_;
    sensor_msgs::msg::PointCloud2 tmpl_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
    uint32_t msg_count_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OS1BridgeNode>());
    rclcpp::shutdown();
    return 0;
}