#ifndef DYNAMIXEL_ROS_CONTROL_DIAGNOSTIC_STATE_H
#define DYNAMIXEL_ROS_CONTROL_DIAGNOSTIC_STATE_H

#include <map>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace dynamixel_ros_control {

/// @brief Lightweight snapshot of diagnostic values for cheap change detection (no heap allocations).
struct DiagnosticState
{
  bool hw_ok{true};
  bool e_stop_active{false};
  bool torque_enabled{false};
  unsigned int read_errors{0};
  unsigned int write_errors{0};
  std::map<std::string, int32_t> joint_hw_errors;  ///< Per-joint hardware error status (only non-OK entries).

  bool operator==(const DiagnosticState& o) const
  {
    return hw_ok == o.hw_ok && e_stop_active == o.e_stop_active && torque_enabled == o.torque_enabled &&
           read_errors == o.read_errors && write_errors == o.write_errors && joint_hw_errors == o.joint_hw_errors;
  }
  bool operator!=(const DiagnosticState& o) const
  {
    return !(*this == o);
  }

  /// @brief Convert to a DiagnosticStatus message. Performs string allocations — only call when publishing.
  diagnostic_msgs::msg::DiagnosticStatus toMsg(const std::string& name) const;

  /// @brief Convert a hardware error status bitfield to a human-readable string.
  static std::string hardwareErrorToString(int32_t error_status);
};

}  // namespace dynamixel_ros_control
#endif
