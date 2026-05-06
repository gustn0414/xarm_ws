#pragma once

#include <hidapi/hidapi.h>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <map>

namespace xarm_1s_hardware
{

struct JointInfo {
  int servo_id;
  double min_rad;
  double max_rad;
  int servo_min = 0;
  int servo_max = 1000;
};

class XArm1SHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  hid_device * hid_dev_ = nullptr;
  uint16_t vid_ = 0x0483;
  uint16_t pid_ = 0x5750;

  std::vector<JointInfo> joint_infos_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  int radToServo(double rad, const JointInfo & info) const;
  double servoToRad(int servo_pos, const JointInfo & info) const;

  bool readAllPositions();
  bool writeAllPositions(int duration_ms = 20);

  rclcpp::Logger logger_ = rclcpp::get_logger("XArm1SHardwareInterface");
};

}  // namespace xarm_1s_hardware
