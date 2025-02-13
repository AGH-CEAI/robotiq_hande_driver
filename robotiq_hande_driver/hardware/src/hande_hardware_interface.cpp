#include "robotiq_hande_driver/hande_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robotiq_hande_driver {

RobotiqHandeHardwareInterface::RobotiqHandeHardwareInterface() {}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_init(const HWI::HardwareInfo& info) {
    if (HWI::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return HWI::CallbackReturn::ERROR;
    }

    state_position_ = 0.025;
    state_velocity_ = 0.0;
    cmd_position_ = 0.025;
    cmd_force_ = 1.0;
    gripper_position_min_ = stod(info_.hardware_parameters["grip_pos_min"]);
    gripper_position_max_ = stod(info_.hardware_parameters["grip_pos_max"]);
    tty_port_ = info_.hardware_parameters["tty"];
    baudrate_ = stoi(info_.hardware_parameters["baudrate"]);
    parity_ = info_.hardware_parameters["parity"][0];
    data_bits_ = stoi(info_.hardware_parameters["data_bits"]);
    stop_bit_ = stoi(info_.hardware_parameters["stop_bit"]);
    slave_id_ = stoi(info_.hardware_parameters["slave_id"]);

    logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.RobotiqHandeHardwareInterface"));

    //--TODO(modbus integration): Set parameters for the modbus communication
    application_layer_.initialize(gripper_position_min_, gripper_position_max_, tty_port_, baudrate_, parity_, data_bits_, stop_bit_, slave_id_);

    RCLCPP_INFO(get_logger(), "Initialized ModbusRTU for %s", tty_port_.c_str());
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_configure(const rlccp_lc::State& /*previous_state*/){
    //--TODO(modbus integration): Initialize the ModbusRTU communication session
    application_layer_.configure();

    RCLCPP_INFO(get_logger(), "configure()");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_cleanup(const rlccp_lc::State& /*previous_state*/){
    //--TODO(modbus integration): Deinitalize the ModbusRTU session
    application_layer_.cleanup();

    RCLCPP_INFO(get_logger(), "cleanup()");
    return HWI::CallbackReturn::SUCCESS;
}

std::vector<HWI::StateInterface> RobotiqHandeHardwareInterface::export_state_interfaces() {
    std::vector<HWI::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[LEFT_FINGER_JOINT_ID].name, hardware_interface::HW_IF_POSITION, &state_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[LEFT_FINGER_JOINT_ID].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_));

    RCLCPP_INFO(get_logger(), "export_state_interfaces()");
    return state_interfaces;
}

std::vector<HWI::CommandInterface> RobotiqHandeHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[LEFT_FINGER_JOINT_ID].name, hardware_interface::HW_IF_POSITION, &cmd_position_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[LEFT_FINGER_JOINT_ID].name, hardware_interface::HW_IF_POSITION, &cmd_force_));

    RCLCPP_INFO(get_logger(), "export_command_interfaces()");
    return command_interfaces;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_activate(const rlccp_lc::State& /*previous_state*/){
    //--TODO(modbus integration): Power on the gripper (activation)
    application_layer_.activate();

    RCLCPP_INFO(get_logger(), "activate()");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_deactivate(const rlccp_lc::State& /*previous_state*/){
    //--TODO(modbus integration): Deactiavte the gripper (set the reset flag)
    application_layer_.deactivate();

    RCLCPP_INFO(get_logger(), "deactivate()");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_shutdown(const rlccp_lc::State& /*previous_state*/){
    //--TODO(modbus integration): Deactivate the gripper, deinitalize the modbus RTU session,
    application_layer_.shutdown();

    RCLCPP_INFO(get_logger(), "shutdown()");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_error(const rlccp_lc::State& /*previous_state*/){
    //--TODO(modbus integration): Placeholder for error handling
    RCLCPP_INFO(get_logger(), "Handled error with FAILURE on purpose - check logs");
    // return HWI::CallbackReturn::SUCCESS;
    return HWI::CallbackReturn::FAILURE;
}

HWI::return_type RobotiqHandeHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                     const rclcpp::Duration& /*period*/) {
    //--TODO(modbus integration): data = driver_->receive_data();
    application_layer_.read();
    state_position_ = application_layer_.get_position();

    RCLCPP_DEBUG(get_logger(), "read()");
    return hardware_interface::return_type::OK;
}
HWI::return_type RobotiqHandeHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                      const rclcpp::Duration& /*period*/) {
    //--TODO(modbus integration): driver_->send_data(cmd_position_);
    application_layer_.set_position(cmd_position_, cmd_force_);
    application_layer_.write();

    RCLCPP_DEBUG(get_logger(), "write()");
    return hardware_interface::return_type::OK;
}


} // namespace robotiq_hande_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotiq_hande_driver::RobotiqHandeHardwareInterface, hardware_interface::SystemInterface)
