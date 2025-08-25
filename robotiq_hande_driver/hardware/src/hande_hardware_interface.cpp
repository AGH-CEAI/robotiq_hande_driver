#include "robotiq_hande_driver/hande_hardware_interface.hpp"
#include "robotiq_hande_driver/utils.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace robotiq_hande_driver {

static constexpr auto RECONNECT_MAX_ITER = 10;

inline void wait_100ms() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

RobotiqHandeHardwareInterface::RobotiqHandeHardwareInterface() {}

RobotiqHandeHardwareInterface::~RobotiqHandeHardwareInterface() {
    if(socat_ && socat_->is_alive()) socat_->stop();
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_init(
    const HWI::HardwareComponentInterfaceParams& info) {
    if(HWI::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return HWI::CallbackReturn::ERROR;
    }
    clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
    logger_ = std::make_shared<rclcpp::Logger>(
        rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system."
                           "RobotiqHandeHardwareInterface"));
    log_parsed_urdf_config();

    th_comm_enabled_.store(false);
    gripper_position_min_ = std::stod(info_.hardware_parameters["grip_pos_min"]);
    gripper_position_max_ = std::stod(info_.hardware_parameters["grip_pos_max"]);

    auto frequency_hz = std::stoi(info_.hardware_parameters["frequency_hz"]);
    th_sleep_rate_ = std::chrono::milliseconds(1000 / frequency_hz);

    bool manage_virutal_serial = str_to_lower(info_.hardware_parameters["create_socat_tty"])
                                 == "true";
    if(manage_virutal_serial) {
        auto ip_addr = info_.hardware_parameters["socat_ip_address"];
        auto port = info_.hardware_parameters["socat_port"];
        auto tty_port = info_.hardware_parameters["tty_port"];

        RCLCPP_INFO(
            get_logger(),
            "%sCreating a virtual serial port from ip:%s port:%s with socat%s",
            color::BCYAN,
            ip_addr.c_str(),
            port.c_str(),
            color::RESET);
        socat_.emplace(SocatManager(ip_addr, std::stoi(port), tty_port));
        socat_->start();
        std::this_thread::sleep_for(WAIT_FOR_SOCAT_CONNECTION);

        RCLCPP_INFO(
            get_logger(),
            "%sVirtual serial port created at %s%s",
            color::BGREEN,
            tty_port.c_str(),
            color::RESET);
    }

    initalize_gripper_driver();

    state_position_ = gripper_position_max_;
    state_velocity_ = 0.0;
    read_position_ = state_position_;
    read_velocity_ = state_velocity_;

    cmd_force_ = 1.0;
    cmd_position_ = gripper_position_max_;
    write_position_ = cmd_position_;
    write_force_ = cmd_force_;

    return HWI::CallbackReturn::SUCCESS;
}

void RobotiqHandeHardwareInterface::log_parsed_urdf_config() {
    RCLCPP_DEBUG(
        get_logger(), "grip_pos_min: %s", info_.hardware_parameters["grip_pos_min"].c_str());
    RCLCPP_DEBUG(
        get_logger(), "grip_pos_max: %s", info_.hardware_parameters["grip_pos_max"].c_str());
    RCLCPP_DEBUG(get_logger(), "tty_port: %s", info_.hardware_parameters["tty_port"].c_str());
    RCLCPP_DEBUG(get_logger(), "baudrate: %s", info_.hardware_parameters["baudrate"].c_str());
    RCLCPP_DEBUG(get_logger(), "parity: %s", info_.hardware_parameters["parity"].c_str());
    RCLCPP_DEBUG(get_logger(), "data_bits: %s", info_.hardware_parameters["data_bits"].c_str());
    RCLCPP_DEBUG(get_logger(), "stop_bit:  %s", info_.hardware_parameters["stop_bit"].c_str());
    RCLCPP_DEBUG(get_logger(), "slave_id: %s", info_.hardware_parameters["slave_id"].c_str());
    RCLCPP_DEBUG(
        get_logger(), "frequency_hz: %s", info_.hardware_parameters["frequency_hz"].c_str());
}

void RobotiqHandeHardwareInterface::initalize_gripper_driver() {
    auto cfg = CommunicationConfig{
        info_.hardware_parameters["tty_port"],
        std::stoi(info_.hardware_parameters["baudrate"]),
        (info_.hardware_parameters["parity"].c_str())[0],
        std::stoi(info_.hardware_parameters["data_bits"]),
        std::stoi(info_.hardware_parameters["stop_bit"]),
        std::stoi(info_.hardware_parameters["slave_id"]),
    };
    gripper_driver_.initialize(gripper_position_min_, gripper_position_max_, cfg);

    RCLCPP_INFO(
        get_logger(),
        "%sInitialized ModbusRTU for %s, %d, %c, %d, %d%s",
        color::BCYAN,
        cfg.tty_port.c_str(),
        cfg.baudrate,
        cfg.parity,
        cfg.data_bits,
        cfg.stop_bit,
        color::RESET);
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_configure(
    const rlccp_lc::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "%sConnecting to ModbusRTU%s", color::BCYAN, color::RESET);

    for(int iter = 0; iter < RECONNECT_MAX_ITER; iter++) {
        RCLCPP_DEBUG(
            get_logger(),
            "%sReconfiguring Hand-E Gripper attempt: %d%s",
            color::BCYAN,
            iter,
            color::RESET);

        try {
            gripper_driver_.configure();
            RCLCPP_INFO(get_logger(), "%sConnected%s", color::BGREEN, color::RESET);
            return HWI::CallbackReturn::SUCCESS;
        } catch(const std::exception& e) {
            RCLCPP_WARN(get_logger(), "%s%s%s", color::BYELLOW, e.what(), color::RESET);
        }
        wait_100ms();
        wait_100ms();
        gripper_driver_.cleanup();
    }

    RCLCPP_ERROR(get_logger(), "%sFailed to configure Hand-E Gripper%s", color::BRED, color::RESET);
    return HWI::CallbackReturn::FAILURE;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_cleanup(
    const rlccp_lc::State& /*previous_state*/) {
    gripper_driver_.cleanup();

    RCLCPP_INFO(get_logger(), "%sCleaned up Hand-E connection%s", color::BCYAN, color::RESET);
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
        info_.joints[LEFT_FINGER_JOINT_ID].name, hardware_interface::HW_IF_EFFORT, &cmd_force_));

    return command_interfaces;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_activate(
    const rlccp_lc::State& /*previous_state*/) {
    try {
        gripper_driver_.deactivate();
        gripper_driver_.activate();

        if(!th_comm_enabled_) {
            th_comm_enabled_.store(true, std::memory_order_relaxed);
            th_comm_.emplace(&RobotiqHandeHardwareInterface::gripper_communication, this);
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(
            get_logger(),
            "%sException during Hand-E activation: %s%s",
            color::BRED,
            e.what(),
            color::RESET);
        return HWI::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "%sHand-E successfully activated%s", color::BGREEN, color::RESET);
    return HWI::CallbackReturn::SUCCESS;
}

void RobotiqHandeHardwareInterface::gripper_communication() {
    while(th_comm_enabled_) {
        try {
            gripper_driver_.read();
            {
                std::lock_guard<std::mutex> lock(mtx_read_);
                read_position_ = gripper_driver_.get_position();
            }

            {
                std::lock_guard<std::mutex> lock(mtx_write_);
                gripper_driver_.set_position(write_position_, write_force_);
            }
            gripper_driver_.write();

        } catch(const std::exception& e) {
            RCLCPP_DEBUG(
                get_logger(),
                "%sException during Hand-E background communication: %s%s",
                color::BYELLOW,
                e.what(),
                color::RESET);
        }

        std::this_thread::sleep_for(th_sleep_rate_);
    }
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_deactivate(
    const rlccp_lc::State& /*previous_state*/) {
    th_comm_enabled_.store(false, std::memory_order_relaxed);
    if(th_comm_ && th_comm_->joinable()) {
        th_comm_->join();
    }

    try {
        gripper_driver_.deactivate();
    } catch(const std::exception& e) {
        RCLCPP_ERROR(
            get_logger(),
            "%sException during Hand-E deactivation: %s%s",
            color::BRED,
            e.what(),
            color::RESET);
        return HWI::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "%sHand-E successfully deactivated%s", color::BCYAN, color::RESET);
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_shutdown(
    const rlccp_lc::State& /*previous_state*/) {
    gripper_driver_.shutdown();

    RCLCPP_INFO(get_logger(), "%sHand-E shutdown%s", color::BCYAN, color::RESET);
    return HWI::CallbackReturn::SUCCESS;
}

HWI::CallbackReturn RobotiqHandeHardwareInterface::on_error(
    const rlccp_lc::State& /*previous_state*/) {
    RCLCPP_INFO(
        get_logger(),
        "%sHandled error with FAILURE on purpose - check previous logs%s",
        color::BRED,
        color::RESET);
    return HWI::CallbackReturn::FAILURE;
}

HWI::return_type RobotiqHandeHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    std::lock_guard<std::mutex> lock(mtx_read_);

    state_position_ = read_position_;
    state_velocity_ = read_velocity_;

    return hardware_interface::return_type::OK;
}
HWI::return_type RobotiqHandeHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    std::lock_guard<std::mutex> lock(mtx_write_);

    write_position_ = cmd_position_;
    write_force_ = cmd_force_;

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
