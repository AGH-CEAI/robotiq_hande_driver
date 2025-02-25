#include "robotiq_hande_driver/hande_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robotiq_hande_driver {

RobotiqHandeHardwareInterface::RobotiqHandeHardwareInterface() {}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_init(const HWI::HardwareInfo& info) {
    if(HWI::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return HWI::CallbackReturn::ERROR;
    }
    logger_ = std::make_shared<rclcpp::Logger>(
        rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system."
                           "RobotiqHandeHardwareInterface"));

    RCLCPP_DEBUG(
        get_logger(), "grip_pos_min: %s", info_.hardware_parameters["grip_pos_min"].c_str());
    RCLCPP_DEBUG(
        get_logger(), "grip_pos_max: %s", info_.hardware_parameters["grip_pos_max"].c_str());
    RCLCPP_DEBUG(get_logger(), "tty: %s", info_.hardware_parameters["tty"].c_str());
    RCLCPP_DEBUG(get_logger(), "baudrate: %s", info_.hardware_parameters["baudrate"].c_str());
    RCLCPP_DEBUG(get_logger(), "parity: %s", info_.hardware_parameters["parity"].c_str());
    RCLCPP_DEBUG(get_logger(), "data_bits: %s", info_.hardware_parameters["data_bits"].c_str());
    RCLCPP_DEBUG(get_logger(), "stop_bit:  %s", info_.hardware_parameters["stop_bit"].c_str());
    RCLCPP_DEBUG(get_logger(), "slave_id: %s", info_.hardware_parameters["slave_id"].c_str());

    state_velocity_ = 0.0;
    cmd_force_ = 1.0;
    gripper_position_min_ = std::stod(info_.hardware_parameters["grip_pos_min"]);
    gripper_position_max_ = std::stod(info_.hardware_parameters["grip_pos_max"]);
    tty_port_ = info_.hardware_parameters["tty"];
    baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);
    parity_ = (info_.hardware_parameters["parity"].c_str())[0];
    data_bits_ = std::stoi(info_.hardware_parameters["data_bits"]);
    stop_bit_ = std::stoi(info_.hardware_parameters["stop_bit"]);
    slave_id_ = std::stoi(info_.hardware_parameters["slave_id"]);

    cmd_position_ = gripper_position_max_;
    state_position_ = gripper_position_max_;

    logger_ = std::make_shared<rclcpp::Logger>(
        rclcpp::get_logger("controller_manager.resource_manager.hardware_"
                           "component.system.RobotiqHandeHardwareInterface"));

    application_layer_.initialize(
        gripper_position_min_,
        gripper_position_max_,
        tty_port_,
        baudrate_,
        parity_,
        data_bits_,
        stop_bit_,
        slave_id_);

    RCLCPP_INFO(
        get_logger(),
        "Initialized ModbusRTU for %s, %d, %c, %d, %d",
        tty_port_.c_str(),
        baudrate_,
        parity_,
        data_bits_,
        stop_bit_);
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_configure(
    const rlccp_lc::State& /*previous_state*/) {
    int result;

    result = application_layer_.configure();

    RCLCPP_INFO(get_logger(), "Configured Hand-E Gripper");
    if(result < 0) return HWI::CallbackReturn::FAILURE;
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_cleanup(
    const rlccp_lc::State& /*previous_state*/) {
    application_layer_.cleanup();

    RCLCPP_INFO(get_logger(), "cleanup()");
    return HWI::CallbackReturn::SUCCESS;
}

std::vector<HWI::StateInterface> RobotiqHandeHardwareInterface::export_state_interfaces() {
    std::vector<HWI::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[LEFT_FINGER_JOINT_ID].name,
        hardware_interface::HW_IF_POSITION,
        &state_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[LEFT_FINGER_JOINT_ID].name,
        hardware_interface::HW_IF_VELOCITY,
        &state_velocity_));

    RCLCPP_INFO(get_logger(), "export_state_interfaces()");
    return state_interfaces;
}

std::vector<HWI::CommandInterface> RobotiqHandeHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[LEFT_FINGER_JOINT_ID].name,
        hardware_interface::HW_IF_POSITION,
        &cmd_position_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[LEFT_FINGER_JOINT_ID].name, hardware_interface::HW_IF_POSITION, &cmd_force_));

    return command_interfaces;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_activate(
    const rlccp_lc::State& /*previous_state*/) {
    application_layer_.activate();

    RCLCPP_INFO(get_logger(), "activate()");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_deactivate(
    const rlccp_lc::State& /*previous_state*/) {
    application_layer_.deactivate();

    RCLCPP_INFO(get_logger(), "deactivate()");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_shutdown(
    const rlccp_lc::State& /*previous_state*/) {
    application_layer_.shutdown();

    RCLCPP_INFO(get_logger(), "shutdown()");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_error(
    const rlccp_lc::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Handled error with FAILURE on purpose - check logs");
    return HWI::CallbackReturn::FAILURE;
}

HWI::return_type RobotiqHandeHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    application_layer_.read();
    state_position_ = application_layer_.get_position();

    RCLCPP_DEBUG(get_logger(), "read()");
    return hardware_interface::return_type::OK;
}
HWI::return_type RobotiqHandeHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    application_layer_.set_position(cmd_position_, cmd_force_);
    application_layer_.write();

    RCLCPP_DEBUG(get_logger(), "write()");

    return hardware_interface::return_type::OK;
}

}  // namespace robotiq_hande_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robotiq_hande_driver::RobotiqHandeHardwareInterface, hardware_interface::SystemInterface)
