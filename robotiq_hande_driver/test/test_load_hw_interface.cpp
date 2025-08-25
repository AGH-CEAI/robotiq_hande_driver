#include <gmock/gmock.h>

#include <cmath>
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#pragma GCC diagnostic pop

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

// Based on tutorial
// https://control.ros.org/rolling/doc/ros2_controllers/doc/writing_new_controller.html

// TODO(issue#23) write more HW Interface tests
// https://github.com/ros-controls/ros2_control/blob/jazzy/hardware_interface/test/mock_components/test_generic_system.cpp

class TestHWInterface : public ::testing::Test {
   protected:
    static void SetUpTestCase() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestCase() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        hw_system_gripper_1dof_ =
            R"(
            <ros2_control name="HandeGripperExample" type="system">
                <hardware>
                    <plugin>robotiq_hande_driver/RobotiqHandeHardwareInterface</plugin>
                    <param name="grip_pos_min">0.0</param>
                    <param name="grip_pos_max">0.025</param>
                    <param name="tty_port">/tmp/ttyUR</param>
                    <param name="baudrate">115200</param>
                    <param name="parity">N</param>
                    <param name="data_bits">8</param>
                    <param name="stop_bit">1</param>
                    <param name="slave_id">9</param>
                    <param name="frequency_hz">10</param>
                    <param name="create_socat_tty">false</param>
                    <param name="socat_ip_address">127.0.0.1</param>
                    <param name="socat_port">8888</param>
                </hardware>
                <joint name="joint1">
                    <command_interface name="position"/>
                    <state_interface name="position">
                        <param name="initial_value">0.025</param>
                    </state_interface>
                </joint>
            </ros2_control>
        )";
    }

    std::string hw_system_gripper_1dof_;
    rclcpp::Node node_ = rclcpp::Node("TestGenericSystem");
};

// Forward declaration
namespace hardware_interface {
class ResourceStorage;
}

class TestableResourceManager : public hardware_interface::ResourceManager {
   public:
    friend TestHWInterface;

    explicit TestableResourceManager(rclcpp::Node& node)
        : hardware_interface::ResourceManager(
              node.get_node_clock_interface(), node.get_node_logging_interface()) {}

    explicit TestableResourceManager(
        rclcpp::Node& node,
        const std::string& urdf,
        bool activate_all = false,
        unsigned int cm_update_rate = 100)
        : hardware_interface::ResourceManager(
              urdf,
              node.get_node_clock_interface(),
              node.get_node_logging_interface(),
              activate_all,
              cm_update_rate) {}
};

TEST_F(TestHWInterface, load_robotiq_hande_hardware_interface) {
    auto urdf = ros2_control_test_assets::urdf_head + hw_system_gripper_1dof_
                + ros2_control_test_assets::urdf_tail;
    ASSERT_NO_THROW(TestableResourceManager rm(node_, urdf));
}
