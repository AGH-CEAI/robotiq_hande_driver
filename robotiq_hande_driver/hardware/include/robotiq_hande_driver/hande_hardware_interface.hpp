#ifndef ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_
#define ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <optional>
#include <thread>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robotiq_hande_driver/hande_gripper.hpp"
#include "robotiq_hande_driver/socat_manager.hpp"

namespace robotiq_hande_driver {

namespace HWI = hardware_interface;
namespace rlccp_lc = rclcpp_lifecycle;

constexpr int LEFT_FINGER_JOINT_ID = 0;

class RobotiqHandeHardwareInterface : public HWI::SystemInterface {
   public:
    RobotiqHandeHardwareInterface();

    ~RobotiqHandeHardwareInterface();

    HWI::CallbackReturn on_init(const HWI::HardwareComponentInterfaceParams& info) override;
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

    rclcpp::Logger get_logger() const;
    rclcpp::Clock::SharedPtr get_clock() const;

   private:
    void log_parsed_urdf_config();
    void initalize_gripper_driver();
    void gripper_communication();

    HandeGripper gripper_driver_;
    std::optional<SocatManager> socat_;

    std::shared_ptr<rclcpp::Logger> logger_;
    rclcpp::Clock::SharedPtr clock_;

    std::chrono::milliseconds th_sleep_rate_;
    std::atomic<bool> th_comm_enabled_;
    std::optional<std::thread> th_comm_;

    double gripper_position_min_;
    double gripper_position_max_;

    double state_position_;
    double state_velocity_;
    std::mutex mtx_read_;
    double read_position_;
    double read_velocity_;

    double cmd_position_;
    double cmd_force_;
    std::mutex mtx_write_;
    double write_position_;
    double write_force_;
};

}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__HANDE_HARDWARE_INTERFACE_HPP_
