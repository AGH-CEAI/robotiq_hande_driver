#ifndef ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_
#define ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_

#include <string>
#include "hardware_interface/system_interface.hpp"

namespace robotiq_hande_driver {

class RobotiqHandeHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RobotiqHandeHardwareInterface();
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    //TODO: composition of the modbus communication
    std::string tty_port_;

};

}
#endif  // ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_
