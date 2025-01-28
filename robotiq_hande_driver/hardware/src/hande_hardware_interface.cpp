#include "robotiq_hande_driver/hande_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

const auto logger = rclcpp::get_logger("RobotiqHandeHardwareInterface");

namespace robotiq_hande_driver {

RobotiqHandeHardwareInterface::RobotiqHandeHardwareInterface() {}

hardware_interface::CallbackReturn RobotiqHandeHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    tty_port_ = info_.hardware_parameters["tty"];

    //TODO: Initialize modbus communication

    RCLCPP_INFO(logger, std::format("Initialized ModbusRTU on {} ", tty_port_));
    return hardware_interface::CallbackReturn::SUCCESS;
}


// CallbackReturn::SUCCESS or return CallbackReturn::ERROR

}
