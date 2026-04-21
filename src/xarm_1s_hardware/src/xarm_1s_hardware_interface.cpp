#include "xarm_1s_hardware/xarm_1s_hardware_interface.hpp"
#include <hidapi/hidapi.h>
#include <cmath>
#include <chrono>
#include <thread>

namespace xarm_1s_hardware
{

// 서보 단위(0~1000) → 라디안
double XArm1SHardwareInterface::servoToRad(int servo_pos, const JointInfo & info) const
{
  double ratio = static_cast<double>(servo_pos - info.servo_min) /
                 static_cast<double>(info.servo_max - info.servo_min);
  return info.min_rad + ratio * (info.max_rad - info.min_rad);
}

// 라디안 → 서보 단위(0~1000)
int XArm1SHardwareInterface::radToServo(double rad, const JointInfo & info) const
{
  double clamped = std::max(info.min_rad, std::min(info.max_rad, rad));
  double ratio = (clamped - info.min_rad) / (info.max_rad - info.min_rad);
  return static_cast<int>(info.servo_min + ratio * (info.servo_max - info.servo_min));
}

hardware_interface::CallbackReturn XArm1SHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // xacro <param>에서 VID/PID 읽기 (기본값: Hiwonder xArm)
  if (info_.hardware_parameters.count("vid")) {
    vid_ = static_cast<uint16_t>(std::stoi(info_.hardware_parameters.at("vid"), nullptr, 0));
  }
  if (info_.hardware_parameters.count("pid")) {
    pid_ = static_cast<uint16_t>(std::stoi(info_.hardware_parameters.at("pid"), nullptr, 0));
  }

  // 조인트 정보 구성
  // xacro의 <command_interface name="position"><param name="min"> 에서 읽음
  // 조인트 순서: arm6(servo1), arm5(servo2), arm4(servo3), arm3(servo4), arm2(servo5), arm1(servo6)
  const std::vector<int> servo_id_map = {1, 2, 3, 4, 5, 6};  // 조인트 순서대로 서보 ID

  joint_infos_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    auto & ji = joint_infos_[i];

    ji.servo_id  = servo_id_map[i];
    ji.servo_min = 0;
    ji.servo_max = 1000;

    // <command_interface name="position"><param name="min/max"> 읽기
    ji.min_rad = -2.09439435;  // 기본값
    ji.max_rad =  2.09439435;
    for (const auto & cmd_if : joint.command_interfaces) {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION) {
        if (cmd_if.parameters.count("min")) {
          ji.min_rad = std::stod(cmd_if.parameters.at("min"));
        }
        if (cmd_if.parameters.count("max")) {
          ji.max_rad = std::stod(cmd_if.parameters.at("max"));
        }
      }
    }
    RCLCPP_INFO(logger_, "Joint[%zu] %s → servo ID %d, range [%.3f, %.3f] rad",
      i, joint.name.c_str(), ji.servo_id, ji.min_rad, ji.max_rad);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm1SHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  hid_init();
  hid_dev_ = hid_open(vid_, pid_, nullptr);
  if (!hid_dev_) {
    RCLCPP_ERROR(logger_, "HID 장치 열기 실패 (VID=0x%04X, PID=0x%04X)", vid_, pid_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  hid_set_nonblocking(hid_dev_, 0);  // blocking 모드
  RCLCPP_INFO(logger_, "Hiwonder xArm HID 연결 성공");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm1SHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  // 현재 위치 읽어서 명령값 초기화
  if (!readAllPositions()) {
    RCLCPP_WARN(logger_, "초기 위치 읽기 실패, 0으로 초기화");
  }
  hw_commands_ = hw_positions_;
  RCLCPP_INFO(logger_, "XArm1S 하드웨어 활성화 완료");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm1SHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (hid_dev_) {
    hid_close(hid_dev_);
    hid_dev_ = nullptr;
  }
  hid_exit();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
XArm1SHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
XArm1SHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

// ──────────────────────────────────────────
// read(): 서보 → ros2_control
// ──────────────────────────────────────────
hardware_interface::return_type XArm1SHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!readAllPositions()) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "서보 읽기 실패");
  }
  return hardware_interface::return_type::OK;
}

// ──────────────────────────────────────────
// write(): ros2_control → 서보
// ──────────────────────────────────────────
hardware_interface::return_type XArm1SHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!writeAllPositions(20)) {  // 20ms 이동 시간
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "서보 쓰기 실패");
  }
  return hardware_interface::return_type::OK;
}

// ──────────────────────────────────────────
// HID: 6개 서보 위치 읽기
// 프로토콜: [0x55,0x55, len=9, cmd=21, count=6, id1..id6]
// 응답:     [0x55,0x55, len, cmd, count, {id, pos_lsb, pos_msb}×6]
// ──────────────────────────────────────────
bool XArm1SHardwareInterface::readAllPositions()
{
  if (!hid_dev_) return false;

  uint8_t buf[65] = {0};
  buf[0] = 0x55; buf[1] = 0x55;
  buf[2] = static_cast<uint8_t>(5 + joint_infos_.size());  // len
  buf[3] = 21;   // CMD: ServoPositionRead
  buf[4] = static_cast<uint8_t>(joint_infos_.size());      // count
  for (size_t i = 0; i < joint_infos_.size(); ++i) {
    buf[5 + i] = static_cast<uint8_t>(joint_infos_[i].servo_id);
  }

  if (hid_write(hid_dev_, buf, 65) < 0) return false;

  uint8_t res[65] = {0};
  int n = hid_read_timeout(hid_dev_, res, 64, 500);
  if (n < 0) return false;

  // 응답 파싱: 오프셋 4=count, 5~= {id, pos_lsb, pos_msb}
  size_t count = res[4];
  for (size_t i = 0; i < count && i < joint_infos_.size(); ++i) {
    int base = 5 + static_cast<int>(i) * 3;
    int servo_pos = res[base + 1] | (res[base + 2] << 8);
    hw_positions_[i] = servoToRad(servo_pos, joint_infos_[i]);
  }
  return true;
}

// ──────────────────────────────────────────
// HID: 6개 서보 위치 일괄 명령
// 프로토콜: [0x55,0x55, len, cmd=3, count, t_lsb, t_msb, {id, pos_lsb, pos_msb}×n]
// ──────────────────────────────────────────
bool XArm1SHardwareInterface::writeAllPositions(int duration_ms)
{
  if (!hid_dev_) return false;

  int n = static_cast<int>(joint_infos_.size());
  // len = 2(time) + 1(count) + n*3 + 2(header) = 5 + n*3
  uint8_t buf[65] = {0};
  buf[0] = 0x55; buf[1] = 0x55;
  buf[2] = static_cast<uint8_t>(5 + n * 3);  // len
  buf[3] = 0x03;  // CMD: SERVO_MOVE
  buf[4] = static_cast<uint8_t>(n);
  buf[5] = static_cast<uint8_t>(duration_ms & 0xFF);
  buf[6] = static_cast<uint8_t>(duration_ms >> 8);

  for (int i = 0; i < n; ++i) {
    int servo_pos = radToServo(hw_commands_[i], joint_infos_[i]);
    int base = 7 + i * 3;
    buf[base]     = static_cast<uint8_t>(joint_infos_[i].servo_id);
    buf[base + 1] = static_cast<uint8_t>(servo_pos & 0xFF);
    buf[base + 2] = static_cast<uint8_t>(servo_pos >> 8);
  }

  return hid_write(hid_dev_, buf, 65) >= 0;
}

}  // namespace xarm_1s_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  xarm_1s_hardware::XArm1SHardwareInterface,
  hardware_interface::SystemInterface)
