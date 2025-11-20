#include "frbot_hardware/frbot_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace frbot_hardware
{

hardware_interface::CallbackReturn FrbotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // joint 인터페이스 검증
  for (const hardware_interface::ComponentInfo & joint : info.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("frbot_hardware"),
        "joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("frbot_hardware"),
        "joint '%s' has '%s' command interface. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("frbot_hardware"),
        "Joint '%s' has %zu state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("frbot_hardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("frbot_hardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ROS2 Node 및 pub, sub 생성
  hardware_node_ = std::make_shared<rclcpp::Node>("frbot_hardware_node");

  // 명령용 Twist publisher (컨트롤러 → 하드웨어)
  cmd_vel_pub_ =
    hardware_node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // 상태 피드백 Twist subscriber (하드웨어 → 컨트롤러)
  state_sub_ =
    hardware_node_->create_subscription<geometry_msgs::msg::Twist>(
      "wheel_velocities",   // 실제 하드웨어 노드가 publish하는 토픽 이름
      10,
      std::bind(
        &FrbotHardware::state_callback,
        this,
        std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FrbotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FrbotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FrbotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
      hw_commands_[i] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("frbot_hardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void FrbotHardware::state_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_linear_x_  = msg->linear.x;
  latest_angular_z_ = msg->angular.z;
}

hardware_interface::return_type FrbotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // 콜백 처리
  if (hardware_node_) {
    rclcpp::spin_some(hardware_node_);
  }

  // 최신 Twist 상태를 hw_velocities_로 반영
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (hw_velocities_.size() >= 4) {
      // 4WD: Twist를 각 바퀴 속도로 변환
      // 간단히: 좌측은 linear.x - angular.z*wheel_base/2
      //        우측은 linear.x + angular.z*wheel_base/2
      double left_vel = latest_linear_x_ - latest_angular_z_ * 0.05;  // wheel_base/2 = 0.05
      double right_vel = latest_linear_x_ + latest_angular_z_ * 0.05;
      
      hw_velocities_[0] = left_vel;   // front_left
      hw_velocities_[1] = right_vel;  // front_right
      hw_velocities_[2] = left_vel;   // rear_left
      hw_velocities_[3] = right_vel;  // rear_right
    } else if (hw_velocities_.size() >= 2) {
      hw_velocities_[0] = latest_linear_x_;   // joint 0
      hw_velocities_[1] = latest_angular_z_;  // joint 1
    }
  }

  // 간단히 적분해서 position 업데이트
  for (std::size_t i = 0; i < hw_velocities_.size(); ++i) {
    hw_positions_[i] += period.seconds() * hw_velocities_[i];
  }

  // 디버깅용 로그
  std::stringstream ss;
  ss << "Reading states:";
  for (std::size_t i = 0; i < hw_velocities_.size(); ++i) {
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\tposition " << hw_positions_[i]
       << " / velocity " << hw_velocities_[i]
       << " for '" << info_.joints[i].name.c_str() << "'";
  }
  RCLCPP_INFO(
    rclcpp::get_logger("frbot_hardware"),
    "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrbotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!cmd_vel_pub_) {
    RCLCPP_WARN(
      rclcpp::get_logger("frbot_hardware"),
      "cmd_vel publisher is not initialized");
    return hardware_interface::return_type::OK;
  }

  geometry_msgs::msg::Twist cmd_msg;

  // 4WD: 동일한 측의 바퀴들은 같은 속도로 구동
  // hw_commands_[0] = front_left, hw_commands_[1] = front_right
  // hw_commands_[2] = rear_left, hw_commands_[3] = rear_right
  
  if (hw_commands_.size() >= 4) {
    // 좌측 바퀴들의 평균 속도 (앞 + 뒤) / 2
    double left_vel = (hw_commands_[0] + hw_commands_[2]) / 2.0;
    // 우측 바퀴들의 평균 속도 (앞 + 뒤) / 2
    double right_vel = (hw_commands_[1] + hw_commands_[3]) / 2.0;
    
    cmd_msg.linear.x = (left_vel + right_vel) / 2.0;   // 선속도
    cmd_msg.angular.z = (right_vel - left_vel) / 2.0;  // 각속도
  } else if (hw_commands_.size() >= 2) {
    cmd_msg.linear.x  = hw_commands_[0];
    cmd_msg.angular.z = hw_commands_[1];
  } else if (hw_commands_.size() == 1) {
    cmd_msg.linear.x  = hw_commands_[0];
    cmd_msg.angular.z = 0.0;
  } else {
    cmd_msg.linear.x  = 0.0;
    cmd_msg.angular.z = 0.0;
  }

  cmd_vel_pub_->publish(cmd_msg);

  std::stringstream ss;
  ss << "Writing Twist command:"
     << "\n\tlinear.x  = " << cmd_msg.linear.x
     << "\n\tangular.z = " << cmd_msg.angular.z;
  RCLCPP_INFO(
    rclcpp::get_logger("frbot_hardware"),
    "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

}  // namespace frbot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  frbot_hardware::FrbotHardware, hardware_interface::SystemInterface)
