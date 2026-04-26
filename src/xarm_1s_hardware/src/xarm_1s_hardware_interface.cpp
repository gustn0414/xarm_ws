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

  // 실제 측정된 서보 ID 매핑:
  // 서보 1=arm1(그리퍼), 2=arm2, 3=arm3, 4=arm4, 5=arm5, 6=arm6(베이스)
  const std::map<std::string, int> joint_servo_map = {
    {"arm1", 1}, {"arm2", 2}, {"arm3", 3},
    {"arm4", 4}, {"arm5", 5}, {"arm6", 6}
  };

  joint_infos_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    auto & ji = joint_infos_[i];

    ji.servo_min = 0;
    ji.servo_max = 1000;

    // 미믹 조인트는 command_interface가 없음 → servo_id = -1로 표시
    if (joint_servo_map.count(joint.name)) {
      ji.servo_id = joint_servo_map.at(joint.name);
    } else {
      ji.servo_id = -1;  // 미믹 조인트 (서보 없음)
    }

    // ── Jazzy: InterfaceInfo에 parameters 멤버 있음 ──────────────────────
    // for (const auto & cmd_if : joint.command_interfaces) {
    //   if (cmd_if.name == hardware_interface::HW_IF_POSITION) {
    //     if (cmd_if.parameters.count("min"))
    //       ji.min_rad = std::stod(cmd_if.parameters.at("min"));
    //     if (cmd_if.parameters.count("max"))
    //       ji.max_rad = std::stod(cmd_if.parameters.at("max"));
    //   }
    // }

    // ── Humble: InterfaceInfo에 parameters 없음 → 기본값 사용 ────────────
    ji.min_rad = -2.09439435;
    ji.max_rad =  2.09439435;
    if (joint.name == "arm1") {
      ji.min_rad = 0.0;
      ji.max_rad = 0.8464847;
    }

    if (ji.servo_id > 0) {
      RCLCPP_INFO(logger_, "Joint[%zu] %s → servo ID %d, range [%.3f, %.3f] rad",
        i, joint.name.c_str(), ji.servo_id, ji.min_rad, ji.max_rad);
    } else {
      RCLCPP_INFO(logger_, "Joint[%zu] %s → 미믹 조인트 (서보 없음)",
        i, joint.name.c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm1SHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  hid_init();

  // hidraw0을 명시적으로 열기 (Python 테스트에서 동작 확인된 경로)
  hid_dev_ = hid_open_path("/dev/hidraw0");
  if (!hid_dev_) {
    // fallback: VID/PID로 검색
    hid_dev_ = hid_open(vid_, pid_, nullptr);
  }
  if (!hid_dev_) {
    RCLCPP_ERROR(logger_, "HID 장치 열기 실패 (VID=0x%04X, PID=0x%04X)", vid_, pid_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  hid_set_nonblocking(hid_dev_, 1);  // non-blocking 모드
  RCLCPP_INFO(logger_, "Hiwonder xArm HID 연결 성공 (path: /dev/hidraw0)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm1SHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Open-loop: 초기값 0(홈 자세)으로 설정
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
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
    // velocity state interface가 요청된 조인트에 대해 더미 값 제공
    for (const auto & state_if : info_.joints[i].state_interfaces) {
      if (state_if.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(
          info_.joints[i].name,
          hardware_interface::HW_IF_VELOCITY,
          &hw_velocities_[i]);
      }
    }
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
  // Open-loop: HID read 없이 명령값을 현재 위치로 간주
  // (HID read가 제어 루프를 블로킹하는 문제 회피)
  hw_positions_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

// ──────────────────────────────────────────
// write(): ros2_control → 서보
// ──────────────────────────────────────────
hardware_interface::return_type XArm1SHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // write를 10Hz로 제한 (HID 장치 처리 속도 한계 대응)
  static int write_count = 0;
  if (++write_count >= 10) {  // 100Hz 중 10번에 1번만 쓰기 (10Hz)
    write_count = 0;
    if (!writeAllPositions(120)) {  // 10Hz 주기(100ms) + 여유 20ms
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "서보 쓰기 실패");
    }
  }
  return hardware_interface::return_type::OK;
}

// ──────────────────────────────────────────
// HID: 실제 서보(servo_id > 0)만 위치 읽기
// 미믹 조인트는 arm1 값에서 계산
// ──────────────────────────────────────────
bool XArm1SHardwareInterface::readAllPositions()
{
  if (!hid_dev_) return false;

  // 실제 서보 ID 목록 수집
  std::vector<int> real_servo_ids;
  std::vector<size_t> real_joint_indices;
  for (size_t i = 0; i < joint_infos_.size(); ++i) {
    if (joint_infos_[i].servo_id > 0) {
      real_servo_ids.push_back(joint_infos_[i].servo_id);
      real_joint_indices.push_back(i);
    }
  }

  int n_servos = static_cast<int>(real_servo_ids.size());
  uint8_t buf[65] = {0};
  buf[0] = 0x55; buf[1] = 0x55;
  buf[2] = static_cast<uint8_t>(5 + n_servos);
  buf[3] = 21;
  buf[4] = static_cast<uint8_t>(n_servos);
  for (int i = 0; i < n_servos; ++i) {
    buf[5 + i] = static_cast<uint8_t>(real_servo_ids[i]);
  }

  if (hid_write(hid_dev_, buf, 65) < 0) return false;

  uint8_t res[65] = {0};
  if (hid_read_timeout(hid_dev_, res, 64, 50) < 0) return false;  // 500ms → 50ms

  // 응답 파싱 → 실제 조인트 위치 업데이트
  int count = res[4];
  for (int i = 0; i < count && i < n_servos; ++i) {
    int base = 5 + i * 3;
    int servo_pos = res[base + 1] | (res[base + 2] << 8);
    size_t joint_idx = real_joint_indices[i];
    hw_positions_[joint_idx] = servoToRad(servo_pos, joint_infos_[joint_idx]);
  }

  // 미믹 조인트: arm1 위치 기반 계산
  // arm1_left = arm1 * (-1), arm0 = arm1 * (-1.01) + (-0.3), arm0_left = arm1 * 1.01 + 0.3
  double arm1_pos = 0.0;
  for (size_t i = 0; i < joint_infos_.size(); ++i) {
    if (info_.joints[i].name == "arm1") { arm1_pos = hw_positions_[i]; break; }
  }
  for (size_t i = 0; i < joint_infos_.size(); ++i) {
    if (joint_infos_[i].servo_id > 0) continue;
    const std::string & jname = info_.joints[i].name;
    if (jname == "arm1_left")  hw_positions_[i] = arm1_pos * (-1.0);
    else if (jname == "arm0")  hw_positions_[i] = arm1_pos * (-1.01) + (-0.3);
    else if (jname == "arm0_left") hw_positions_[i] = arm1_pos * 1.01 + 0.3;
  }
  return true;
}

// ──────────────────────────────────────────
// HID: 실제 서보(servo_id > 0)만 위치 명령
// 미믹 조인트는 건너뜀
// ──────────────────────────────────────────
bool XArm1SHardwareInterface::writeAllPositions(int duration_ms)
{
  if (!hid_dev_) return false;

  // 실제 서보만 필터링
  std::vector<std::pair<int, int>> servo_cmds;  // {servo_id, servo_pos}
  for (size_t i = 0; i < joint_infos_.size(); ++i) {
    if (joint_infos_[i].servo_id <= 0) continue;
    int servo_pos = radToServo(hw_commands_[i], joint_infos_[i]);
    servo_cmds.push_back({joint_infos_[i].servo_id, servo_pos});
  }

  int n = static_cast<int>(servo_cmds.size());
  uint8_t buf[65] = {0};
  buf[0] = 0x55; buf[1] = 0x55;
  buf[2] = static_cast<uint8_t>(5 + n * 3);
  buf[3] = 0x03;
  buf[4] = static_cast<uint8_t>(n);
  buf[5] = static_cast<uint8_t>(duration_ms & 0xFF);
  buf[6] = static_cast<uint8_t>(duration_ms >> 8);

  for (int i = 0; i < n; ++i) {
    int base = 7 + i * 3;
    buf[base]     = static_cast<uint8_t>(servo_cmds[i].first);
    buf[base + 1] = static_cast<uint8_t>(servo_cmds[i].second & 0xFF);
    buf[base + 2] = static_cast<uint8_t>(servo_cmds[i].second >> 8);
  }

  // 먼저 쌓인 응답 비우기, 그다음 명령 전송
  uint8_t drain[65];
  while (hid_read(hid_dev_, drain, 64) > 0);
  int result = hid_write(hid_dev_, buf, 65);
  return result >= 0;
}

}  // namespace xarm_1s_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  xarm_1s_hardware::XArm1SHardwareInterface,
  hardware_interface::SystemInterface)
