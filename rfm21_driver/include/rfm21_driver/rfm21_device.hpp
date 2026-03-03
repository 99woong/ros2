#ifndef RFM21_DRIVER__RFM21_DEVICE_HPP_
#define RFM21_DRIVER__RFM21_DEVICE_HPP_

#include "rfm21_driver/interfaces.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

// ──────────────────────────────────────────────────────────────────────────────
//  RFM 2.1 CAN ID map  (CAN-open simplified)
//
//  Node-manager (NS → RFM)   CAN ID = 0x000
//    Reset node  : [0x81, node_id]  or  [0x81, 0x00] (broadcast)
//    Start node  : [0x01, node_id]  or  [0x01, 0x00]
//    Stop  node  : [0x02, node_id]  or  [0x02, 0x00]
//
//  Boot-up (RFM → NS)        CAN ID = 0x700 + node_id   data=[0x00]
//
//  ── RFM starts cycle (Sync-input mode, default) ──
//    Sync msg  (RFM → NS)    CAN ID = 0x180 + node_id   LEN=2
//      byte0 = start_id,  byte1 = sync_status
//    Result msg1 (RFM → NS)  CAN ID = 0x280 + node_id   LEN=8
//      [0-1] status, [2-3] RelX(mm), [4-5] RelY(mm),
//      [6] start_id, [7] time_offset(×0.1ms)
//    Result msg2 (RFM → NS)  CAN ID = 0x380 + node_id   LEN=8
//      [0-3] Abs-Y (mm, LSB first), [4-7] Abs-X (mm, LSB first)
//
//  ── NS starts cycle (Trigger mode) ──
//    Trigger (NS → RFM)      CAN ID = 0x200 + node_id   LEN=1
//      byte0 = trigger_id (walking 0-255)
//    Result msg1 (RFM → NS)  CAN ID = 0x180 + node_id   LEN=8  (same layout)
//    Result msg2 (RFM → NS)  CAN ID = 0x280 + node_id   LEN=8  (same layout)
// ──────────────────────────────────────────────────────────────────────────────

// RFM Status word bit definitions
namespace RfmStatus
{
    static constexpr uint16_t TRANSPONDER_PRESENT  = (1u << 0);
    static constexpr uint16_t CODE_VALID           = (1u << 1);
    static constexpr uint16_t POSITION_CALCULATED  = (1u << 2);
    static constexpr uint16_t REFERENCE_PRESENT    = (1u << 3);
    static constexpr uint16_t POWER_PRESENT        = (1u << 4);
    static constexpr uint16_t ERR_CRC              = (1u << 5);
    static constexpr uint16_t ERR_NO_START         = (1u << 6);
    static constexpr uint16_t ERR_STOP_NEQ_START   = (1u << 7);
    static constexpr uint16_t ERR_SMALL_SIG_X      = (1u << 8);
    static constexpr uint16_t ERR_SMALL_SIG_Y      = (1u << 9);
    static constexpr uint16_t ERR_HEIGHT_SENSOR    = (1u << 10);
    static constexpr uint16_t ERR_TRANSMITTER      = (1u << 11);
    static constexpr uint16_t HW_FAULT             = (1u << 15);
}

// Sync-status byte bit definitions (byte1 of the Sync message)
namespace SyncStatus
{
    static constexpr uint8_t VALID_SYNC_PULSE = (1u << 0);
    static constexpr uint8_t SYNC_WITH_GPS    = (1u << 1);
    static constexpr uint8_t GPS_VALID        = (1u << 2);
}

// ──────────────────────────────────────────────────────────────────────────────
class RFM21Device : public DeviceInterface
{
public:
    explicit RFM21Device(rclcpp::Logger logger);
    ~RFM21Device() override = default;

    void setup(rclcpp::Node& node) override;
    void process_can_message(const TPCANMsg& msg) override;
    void publish_data(rclcpp::Node& node) override;

    // Send the NMT Start-Node command so the RFM begins measurements.
    // The generic node calls this after CAN initialisation is done.
    void send_startup(rclcpp::Node& node) override;

    // Accessor used by the generic node to write messages back to the bus
    void set_can_driver(std::shared_ptr<CanDriverInterface> driver)
    { can_driver_ = driver; }

private:
    // ── helpers ──────────────────────────────────────────────────────────────
    void handle_bootup(const TPCANMsg& msg);
    void handle_sync_msg(const TPCANMsg& msg);          // 0x180, LEN=2
    void handle_result_msg1_sync(const TPCANMsg& msg);  // 0x280, LEN=8 (sync mode)
    void handle_result_msg2_sync(const TPCANMsg& msg);  // 0x380, LEN=8 (sync mode)
    void handle_result_msg1_trig(const TPCANMsg& msg);  // 0x180, LEN=8 (trigger mode)
    void handle_result_msg2_trig(const TPCANMsg& msg);  // 0x280, LEN=8 (trigger mode)

    void send_nmt(uint8_t command, uint8_t target_node_id);
    void send_trigger();

    void publish_measurement(rclcpp::Node& node);
    std::string build_status_string(uint16_t status) const;
    std::string build_sync_status_string(uint8_t sync_status) const;

    // ── ROS publishers ───────────────────────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr           status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           info_pub_;

    // ── parameters ───────────────────────────────────────────────────────────
    int         node_id_    = 15;   // default: front antenna (open pin)
    std::string frame_id_   = "rfm21";
    bool        trigger_mode_ = false;  // false = RFM starts (sync-input mode, default)

    // ── CAN driver (for sending messages back) ───────────────────────────────
    std::shared_ptr<CanDriverInterface> can_driver_;

    // ── logger ───────────────────────────────────────────────────────────────
    rclcpp::Logger logger_;

    // ── latest measurement data ───────────────────────────────────────────────
    // From Sync message (sync mode)
    uint8_t  last_start_id_   = 0;
    uint8_t  last_sync_status_ = 0;

    // From Result msg1
    uint16_t status_      = 0;
    int16_t  rel_x_mm_   = 0;   // relative X in mm
    int16_t  rel_y_mm_   = 0;   // relative Y in mm
    uint8_t  time_offset_ = 0;  // × 0.1 ms

    // From Result msg2
    int32_t  abs_x_mm_   = 0;   // absolute X in mm
    int32_t  abs_y_mm_   = 0;   // absolute Y in mm
    bool     has_transponder_code_ = false;

    // Update flags
    bool result1_updated_ = false;
    bool result2_updated_ = false;

    // Trigger walking counter (trigger mode only)
    uint8_t trigger_id_ = 0;
};

#endif  // RFM21_DRIVER__RFM21_DEVICE_HPP_
