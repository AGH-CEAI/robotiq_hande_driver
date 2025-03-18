#ifndef ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_
#define ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "application.hpp"

namespace robotiq_hande_driver {

namespace HWI = hardware_interface;
namespace rlccp_lc = rclcpp_lifecycle;

constexpr int LEFT_FINGER_JOINT_ID = 0;

class RobotiqHandeHardwareInterface : public HWI::SystemInterface {
   public:
    RobotiqHandeHardwareInterface();

    HWI::CallbackReturn on_init(const HWI::HardwareInfo& info) override;
    HWI::CallbackReturn on_configure(const rlccp_lc::State& previous_state) override;
    HWI::CallbackReturn on_cleanup(const rlccp_lc::State& previous_state) override;
    std::vector<HWI::StateInterface> export_state_interfaces() override;
    std::vector<HWI::CommandInterface> export_command_interfaces() override;

    HWI::CallbackReturn on_activate(const rlccp_lc::State& previous_state) override;
    HWI::CallbackReturn on_deactivate(const rlccp_lc::State& previous_state) override;

    HWI::CallbackReturn on_shutdown(const rlccp_lc::State& previous_state) override;
    HWI::CallbackReturn on_error(const rlccp_lc::State& previous_state) override;

    HWI::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    HWI::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    rclcpp::Logger get_logger() const {
        return *logger_;
    }

    rclcpp::Clock::SharedPtr get_clock() const {
        return clock_;
    }

   private:
    GripperApplication gripper_driver_;
    std::shared_ptr<rclcpp::Logger> logger_;
    rclcpp::Clock::SharedPtr clock_;

    double gripper_position_min_;
    double gripper_position_max_;
    std::string tty_port_;
    int baudrate_;
    char parity_;
    int data_bits_;
    int stop_bit_;
    int slave_id_;

    double state_position_;
    double state_velocity_;
    double cmd_position_;
    double cmd_force_;
};

}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_
