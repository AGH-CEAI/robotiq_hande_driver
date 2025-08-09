#include "robotiq_hande_driver/hande_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>

namespace robotiq_hande_driver {

static constexpr auto THROTTLE_1000_MS = 1000;
static constexpr auto ACTIVATION_MAX_ITER = 20;
static constexpr auto RECONNECT_MAX_ITER = 10;
inline void wait_100ms() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

RobotiqHandeHardwareInterface::RobotiqHandeHardwareInterface() {}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_init(const HWI::HardwareInfo& info) {
    if(HWI::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return HWI::CallbackReturn::ERROR;
    }
    clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
    logger_ = std::make_shared<rclcpp::Logger>(
        rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system."
                           "RobotiqHandeHardwareInterface"));
    log_parsed_urdf_config();

    state_velocity_ = 0.0;
    cmd_force_ = 1.0;
    gripper_position_min_ = std::stod(info_.hardware_parameters["grip_pos_min"]);
    gripper_position_max_ = std::stod(info_.hardware_parameters["grip_pos_max"]);
    initalize_gripper_driver();

    cmd_position_ = gripper_position_max_;
    state_position_ = gripper_position_max_;

    return HWI::CallbackReturn::SUCCESS;
}

void RobotiqHandeHardwareInterface::log_parsed_urdf_config() {
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
    RCLCPP_DEBUG(
        get_logger(), "frequency_hz: %s", info_.hardware_parameters["frequency_hz"].c_str());
}

void RobotiqHandeHardwareInterface::initalize_gripper_driver() {
    auto frequency_hz = std::stoi(info_.hardware_parameters["frequency_hz"]);
    auto cfg = CommunicationConfig{
        info_.hardware_parameters["tty"],  // TODO change name to tty_port
        std::stoi(info_.hardware_parameters["baudrate"]),
        (info_.hardware_parameters["parity"].c_str())[0],
        std::stoi(info_.hardware_parameters["data_bits"]),
        std::stoi(info_.hardware_parameters["stop_bit"]),
        std::stoi(info_.hardware_parameters["slave_id"]),
        std::chrono::milliseconds(1000 / frequency_hz),  // th_sleep_rate
    };
    gripper_driver_.initialize(gripper_position_min_, gripper_position_max_, cfg);

    RCLCPP_INFO(
        get_logger(),
        "Initialized ModbusRTU for %s, %d, %c, %d, %d",
        cfg.tty_port.c_str(),
        cfg.baudrate,
        cfg.parity,
        cfg.data_bits,
        cfg.stop_bit);
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_configure(
    const rlccp_lc::State& /*previous_state*/) {
    int result = FAILURE_MODBUS;

    RCLCPP_INFO(get_logger(), "Connecting to ModbusRTU");

    result = gripper_driver_.configure();

    for(int iter = 0; result == FAILURE_MODBUS && iter < RECONNECT_MAX_ITER; iter++) {
        RCLCPP_DEBUG(
            get_logger(), "Reconfiguring Hand-E Gripper attempt: %d; result: %d", iter, result);

        wait_100ms();
        wait_100ms();

        gripper_driver_.cleanup();
        result = gripper_driver_.configure();
    }

    // TODO change passing result value to std::throw mechanism
    if(result == FAILURE_MODBUS) {
        RCLCPP_ERROR(get_logger(), "Failed to configure Hand-E Gripper");
        return HWI::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Connected");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_cleanup(
    const rlccp_lc::State& /*previous_state*/) {
    gripper_driver_.cleanup();

    RCLCPP_INFO(get_logger(), "Cleaned up Hand-E connection");
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
    int iter = 0;
    gripper_driver_.read();

    if(gripper_driver_.get_status().is_ready) {
        RCLCPP_INFO(get_logger(), "Hand-E already active");
        return HWI::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(get_logger(), "Hand-E activation in progress");
    gripper_driver_.activate();

    while(!gripper_driver_.get_status().is_ready) {
        RCLCPP_DEBUG_SKIPFIRST_THROTTLE(
            get_logger(),
            *get_clock(),
            THROTTLE_1000_MS,
            "Waiting for activation to be finished: %d",
            iter);
        wait_100ms();
        gripper_driver_.read();
        if(iter++ > ACTIVATION_MAX_ITER) {
            // TODO ADD COLOURS TO HIGH LEVEL LOGGING
            //  const std::string red     = "\033[31m";
            //  const std::string reset   = "\033[0m";
            RCLCPP_ERROR(get_logger(), "\033[31mFailed to activate Hand-E (Timeout)\033[0m");
            return HWI::CallbackReturn::FAILURE;
        }
    }

    RCLCPP_INFO(get_logger(), "Hand-E successfully activated");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_deactivate(
    const rlccp_lc::State& /*previous_state*/) {
    gripper_driver_.deactivate();

    RCLCPP_INFO(get_logger(), "Hand-E successfully deactivated");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_shutdown(
    const rlccp_lc::State& /*previous_state*/) {
    gripper_driver_.shutdown();

    RCLCPP_INFO(get_logger(), "Hand-E shutdown");
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_error(
    const rlccp_lc::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Handled error with FAILURE on purpose - check previous logs");
    return HWI::CallbackReturn::FAILURE;
}

HWI::return_type RobotiqHandeHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    gripper_driver_.read();
    state_position_ = gripper_driver_.get_position();

    return hardware_interface::return_type::OK;
}
HWI::return_type RobotiqHandeHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    gripper_driver_.set_position(cmd_position_, cmd_force_);
    gripper_driver_.write();
    return hardware_interface::return_type::OK;
}

rclcpp::Logger RobotiqHandeHardwareInterface::get_logger() const {
    return *logger_;
}

rclcpp::Clock::SharedPtr RobotiqHandeHardwareInterface::get_clock() const {
    return clock_;
}

}  // namespace robotiq_hande_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robotiq_hande_driver::RobotiqHandeHardwareInterface, hardware_interface::SystemInterface)
