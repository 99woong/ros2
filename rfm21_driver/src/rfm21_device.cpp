#include "rfm21_driver/rfm21_device.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

// ─── constructor ─────────────────────────────────────────────────────────────

RFM21Device::RFM21Device(rclcpp::Logger logger)
: logger_(logger)
{
}

// ─── setup ───────────────────────────────────────────────────────────────────

void RFM21Device::setup(rclcpp::Node& node)
{
    // ── ROS parameters ──────────────────────────────────────────────────────
    node.declare_parameter("node_id",      15);       // 15=front, 16=rear
    node.declare_parameter("frame_id",     "rfm21");
    node.declare_parameter("trigger_mode", false);    // false = sync-input (default)

    node_id_      = node.get_parameter("node_id").as_int();
    frame_id_     = node.get_parameter("frame_id").as_string();
    trigger_mode_ = node.get_parameter("trigger_mode").as_bool();

    // ── publishers ──────────────────────────────────────────────────────────
    pose_pub_   = node.create_publisher<geometry_msgs::msg::PoseStamped>("rfm21/pose",   10);
    status_pub_ = node.create_publisher<std_msgs::msg::UInt16>          ("rfm21/status", 10);
    info_pub_   = node.create_publisher<std_msgs::msg::String>          ("rfm21/info",   10);

    RCLCPP_INFO(logger_,
        "RFM21Device ready.  node_id=%d  frame_id=%s  mode=%s",
        node_id_, frame_id_.c_str(),
        trigger_mode_ ? "TRIGGER (NS starts)" : "SYNC-INPUT (RFM starts)");
}

// ─── NMT send ────────────────────────────────────────────────────────────────

void RFM21Device::send_nmt(uint8_t command, uint8_t target_node_id)
{
    if (!can_driver_) {
        RCLCPP_ERROR(logger_, "send_nmt: CAN driver not set.");
        return;
    }
    TPCANMsg msg{};
    msg.ID      = 0x000;
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    msg.LEN     = 2;
    msg.DATA[0] = command;
    msg.DATA[1] = target_node_id;

    TPCANStatus st = can_driver_->write_message(msg);
    if (st != PCAN_ERROR_OK) {
        RCLCPP_ERROR(logger_, "Failed to send NMT command 0x%02X: 0x%x", command, st);
    } else {
        RCLCPP_INFO(logger_,
            "NMT sent: cmd=0x%02X  target_node=0x%02X", command, target_node_id);
    }
}

// ─── send_startup ─────────────────────────────────────────────────────────────
// Called by the generic node once CAN is up.
// Sends "Start Node" to the RFM so it begins measurements.

void RFM21Device::send_startup(rclcpp::Node& /*node*/)
{
    RCLCPP_INFO(logger_, "Sending NMT Start-Node to RFM node_id=%d …", node_id_);
    send_nmt(0x01, static_cast<uint8_t>(node_id_));
}

// ─── send_trigger ────────────────────────────────────────────────────────────
// Only used in trigger_mode_.

void RFM21Device::send_trigger()
{
    if (!can_driver_) return;

    TPCANMsg msg{};
    msg.ID      = 0x200 + static_cast<uint32_t>(node_id_);
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    msg.LEN     = 1;
    msg.DATA[0] = trigger_id_++;   // walking 0–255

    TPCANStatus st = can_driver_->write_message(msg);
    if (st != PCAN_ERROR_OK) {
        RCLCPP_WARN(logger_, "Trigger send failed: 0x%x", st);
    }
}

// ─── process_can_message ─────────────────────────────────────────────────────

void RFM21Device::process_can_message(const TPCANMsg& msg)
{
    const uint32_t boot_id  = 0x700u + static_cast<uint32_t>(node_id_);
    const uint32_t id_180   = 0x180u + static_cast<uint32_t>(node_id_);
    const uint32_t id_280   = 0x280u + static_cast<uint32_t>(node_id_);
    const uint32_t id_380   = 0x380u + static_cast<uint32_t>(node_id_);

    // ── Boot-up ─────────────────────────────────────────────────────────────
    if (msg.ID == boot_id) {
        handle_bootup(msg);
        return;
    }

    if (!trigger_mode_) {
        // ── RFM starts cycle (sync-input mode, default) ──────────────────────
        //  0x180 + node_id  LEN=2  → Sync message
        //  0x180 + node_id  LEN=8  → Result msg1  (some firmware uses 0x180 for both)
        //  0x280 + node_id  LEN=8  → Result msg1
        //  0x380 + node_id  LEN=8  → Result msg2
        if (msg.ID == id_180 && msg.LEN == 2) {
            handle_sync_msg(msg);
        } else if (msg.ID == id_180 && msg.LEN == 8) {
            // Some RFM firmware puts result-msg1 on 0x180 when in sync mode
            handle_result_msg1_sync(msg);
        } else if (msg.ID == id_280 && msg.LEN == 8) {
            handle_result_msg1_sync(msg);
        } else if (msg.ID == id_380 && msg.LEN == 8) {
            handle_result_msg2_sync(msg);
        }
    } else {
        // ── Navigation system starts cycle (trigger mode) ────────────────────
        //  0x180 + node_id  LEN=8  → Result msg1
        //  0x280 + node_id  LEN=8  → Result msg2
        if (msg.ID == id_180 && msg.LEN == 8) {
            handle_result_msg1_trig(msg);
        } else if (msg.ID == id_280 && msg.LEN == 8) {
            handle_result_msg2_trig(msg);
        }
    }
}

// ─── handle_bootup ───────────────────────────────────────────────────────────

void RFM21Device::handle_bootup(const TPCANMsg& msg)
{
    if (msg.LEN >= 1 && msg.DATA[0] == 0x00) {
        RCLCPP_INFO(logger_,
            "[BOOT-UP] RFM node_id=%d is online.  Sending Start-Node …", node_id_);
        send_nmt(0x01, static_cast<uint8_t>(node_id_));
    }
}

// ─── handle_sync_msg  (RFM starts cycle, 0x180 LEN=2) ────────────────────────

void RFM21Device::handle_sync_msg(const TPCANMsg& msg)
{
    last_start_id_    = msg.DATA[0];
    last_sync_status_ = msg.DATA[1];

    std::string sync_str = build_sync_status_string(last_sync_status_);
    RCLCPP_DEBUG(logger_,
        "[SYNC] start_id=%u  sync_status=0x%02X (%s)",
        last_start_id_, last_sync_status_, sync_str.c_str());

    // In trigger mode we would not be here; in sync mode, optionally send trigger
    // (not needed – RFM drives the cycle itself)
}

// ─── handle_result_msg1  (sync mode, 0x280 or 0x180 LEN=8) ──────────────────
//  [0-1] status (LSB first), [2-3] RelX mm, [4-5] RelY mm,
//  [6]   start_id,           [7]   time_offset (×0.1 ms)

void RFM21Device::handle_result_msg1_sync(const TPCANMsg& msg)
{
    status_      = *reinterpret_cast<const uint16_t*>(&msg.DATA[0]);
    rel_x_mm_    = *reinterpret_cast<const int16_t*> (&msg.DATA[2]);
    rel_y_mm_    = *reinterpret_cast<const int16_t*> (&msg.DATA[4]);
    last_start_id_ = msg.DATA[6];
    time_offset_   = msg.DATA[7];

    result1_updated_ = true;

    RCLCPP_DEBUG(logger_,
        "[RESULT1] status=0x%04X  RelX=%d mm  RelY=%d mm  "
        "start_id=%u  time_offset=%.1f ms",
        status_, rel_x_mm_, rel_y_mm_,
        last_start_id_, time_offset_ * 0.1);
}

// ─── handle_result_msg2  (sync mode, 0x380 LEN=8) ────────────────────────────
//  [0-3] Abs-Y mm (LSB first), [4-7] Abs-X mm (LSB first)

void RFM21Device::handle_result_msg2_sync(const TPCANMsg& msg)
{
    // Per spec: bytes 0-3 = Abs-Y, bytes 4-7 = Abs-X  (LSB first)
    abs_y_mm_ = *reinterpret_cast<const int32_t*>(&msg.DATA[0]);
    abs_x_mm_ = *reinterpret_cast<const int32_t*>(&msg.DATA[4]);
    has_transponder_code_ = true;
    result2_updated_ = true;

    RCLCPP_DEBUG(logger_,
        "[RESULT2] AbsX=%d mm  AbsY=%d mm  (raw bytes: %02X %02X %02X %02X | %02X %02X %02X %02X)",
        abs_x_mm_, abs_y_mm_,
        msg.DATA[0], msg.DATA[1], msg.DATA[2], msg.DATA[3],
        msg.DATA[4], msg.DATA[5], msg.DATA[6], msg.DATA[7]);
}

// ─── trigger mode handlers (same payload, different CAN IDs) ─────────────────

void RFM21Device::handle_result_msg1_trig(const TPCANMsg& msg)
{
    handle_result_msg1_sync(msg);   // identical payload layout
}

void RFM21Device::handle_result_msg2_trig(const TPCANMsg& msg)
{
    handle_result_msg2_sync(msg);   // identical payload layout
}

// ─── publish_data ─────────────────────────────────────────────────────────────

void RFM21Device::publish_data(rclcpp::Node& node)
{
    // In trigger mode: fire a trigger on every timer tick
    if (trigger_mode_) {
        send_trigger();
    }

    if (result1_updated_ || result2_updated_) {
        publish_measurement(node);
        result1_updated_ = false;
        result2_updated_ = false;
    }
}

// ─── publish_measurement ─────────────────────────────────────────────────────

void RFM21Device::publish_measurement(rclcpp::Node& node)
{
    const auto now = node.now();

    // ── Status word ──────────────────────────────────────────────────────────
    {
        std_msgs::msg::UInt16 smsg;
        smsg.data = status_;
        status_pub_->publish(smsg);
    }

    // ── Human-readable info string ───────────────────────────────────────────
    {
        bool transponder_present = (status_ & RfmStatus::TRANSPONDER_PRESENT) != 0;
        bool code_valid          = (status_ & RfmStatus::CODE_VALID)          != 0;
        bool pos_calculated      = (status_ & RfmStatus::POSITION_CALCULATED) != 0;
        bool power_ok            = (status_ & RfmStatus::POWER_PRESENT)       != 0;

        std::ostringstream oss;
        oss << "=== RFM 2.1 Measurement ===\n";
        oss << "  Node ID     : " << node_id_ << "\n";
        oss << "  Start ID    : " << static_cast<int>(last_start_id_) << "\n";
        oss << "  Time offset : " << (time_offset_ * 0.1) << " ms\n";
        oss << "  Sync status : " << build_sync_status_string(last_sync_status_) << "\n";
        oss << "  --- Transponder ---\n";
        oss << "  Present     : " << (transponder_present ? "YES" : "NO") << "\n";
        oss << "  Code valid  : " << (code_valid          ? "YES" : "NO") << "\n";
        oss << "  Pos calc    : " << (pos_calculated       ? "YES" : "NO") << "\n";
        oss << "  Power OK    : " << (power_ok             ? "YES" : "NO") << "\n";

        if (transponder_present && pos_calculated) {
            oss << "  Relative X  : " << rel_x_mm_ << " mm\n";
            oss << "  Relative Y  : " << rel_y_mm_ << " mm\n";
        }
        if (has_transponder_code_ && code_valid) {
            oss << "  Absolute X  : " << abs_x_mm_ << " mm  ("
                << (abs_x_mm_ * 0.1) << " cm)\n";
            oss << "  Absolute Y  : " << abs_y_mm_ << " mm  ("
                << (abs_y_mm_ * 0.1) << " cm)\n";
        }
        oss << "  Status bits : " << build_status_string(status_);

        std_msgs::msg::String imsg;
        imsg.data = oss.str();
        info_pub_->publish(imsg);

        // Also log to console at INFO level when a transponder is seen
        if (transponder_present) {
            RCLCPP_INFO(logger_, "\n%s", oss.str().c_str());
        }
    }

    // ── PoseStamped (absolute position + relative offset as orientation) ──────
    //   position.x/y/z = absolute X/Y in metres, 0
    //   orientation encodes relative offset (RelX→x, RelY→y) via linear scale
    //   (no real quaternion meaning – purely for visualisation/inspection)
    if (has_transponder_code_) {
        geometry_msgs::msg::PoseStamped pmsg;
        pmsg.header.stamp    = now;
        pmsg.header.frame_id = frame_id_;

        pmsg.pose.position.x = abs_x_mm_ * 0.001;   // mm → m
        pmsg.pose.position.y = abs_y_mm_ * 0.001;
        pmsg.pose.position.z = 0.0;

        // Represent the relative deviation as a yaw angle (rough visualisation)
        double rel_angle = std::atan2(rel_y_mm_, rel_x_mm_);
        pmsg.pose.orientation.x = 0.0;
        pmsg.pose.orientation.y = 0.0;
        pmsg.pose.orientation.z = std::sin(rel_angle / 2.0);
        pmsg.pose.orientation.w = std::cos(rel_angle / 2.0);

        pose_pub_->publish(pmsg);
    }
}

// ─── build_status_string ─────────────────────────────────────────────────────

std::string RFM21Device::build_status_string(uint16_t status) const
{
    std::ostringstream oss;
    oss << "[";
    if (status & RfmStatus::TRANSPONDER_PRESENT)  oss << "PRESENT ";
    if (status & RfmStatus::CODE_VALID)           oss << "CODE_OK ";
    if (status & RfmStatus::POSITION_CALCULATED)  oss << "POS_OK ";
    if (status & RfmStatus::REFERENCE_PRESENT)    oss << "REF ";
    if (status & RfmStatus::POWER_PRESENT)        oss << "PWR ";
    if (status & RfmStatus::ERR_CRC)              oss << "ERR_CRC ";
    if (status & RfmStatus::ERR_NO_START)         oss << "ERR_NO_START ";
    if (status & RfmStatus::ERR_STOP_NEQ_START)   oss << "ERR_STOP ";
    if (status & RfmStatus::ERR_SMALL_SIG_X)      oss << "ERR_SIG_X ";
    if (status & RfmStatus::ERR_SMALL_SIG_Y)      oss << "ERR_SIG_Y ";
    if (status & RfmStatus::ERR_HEIGHT_SENSOR)    oss << "ERR_HEIGHT ";
    if (status & RfmStatus::ERR_TRANSMITTER)      oss << "ERR_TX ";
    if (status & RfmStatus::HW_FAULT)             oss << "HW_FAULT ";
    oss << "]";
    return oss.str();
}

// ─── build_sync_status_string ────────────────────────────────────────────────

std::string RFM21Device::build_sync_status_string(uint8_t sync_status) const
{
    uint8_t bits = sync_status & 0x07;
    switch (bits) {
        case 0x00: return "000: No valid sync pulse";
        case 0x01: return "001: Valid sync pulse (free-running, no GPS sync)";
        case 0x03: return "011: Valid sync + GPS sync (GPS signal lost, still in sync)";
        case 0x07: return "111: Valid sync + GPS sync + GPS reception OK";
        default:   return "Unknown sync state";
    }
}
