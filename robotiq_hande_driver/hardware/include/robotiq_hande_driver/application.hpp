#ifndef APPLICATION_HPP_
#define APPLICATION_HPP_

#include <stdint.h>

#include "protocol_logic.hpp"


namespace robotiq_hande_driver
{

constexpr auto GRIPPER_POSITION_MIN = 0.0;
constexpr auto GRIPPER_POSITION_MAX = 0.05;
constexpr auto GRIPPER_POSITION_STEP = (GRIPPER_POSITION_MAX - GRIPPER_POSITION_MIN) / 255.0;
constexpr auto GRIPPER_CURRENT_SCALE = 0.01;
constexpr auto MAX_SPEED = 255;
constexpr auto MAX_FORCE = 255;

/**
 * @brief This class contains high-level gripper commands and status.
 */
class GripperApplication {
public:

    struct Status {
        bool is_reset;
        bool is_ready;
        bool is_moving;
        bool is_stopped;
        bool is_opened;
        bool is_closed;
        bool object_detected;
    };

    struct FaultStatus {
        bool is_error;
    };

    GripperApplication();

    ~GripperApplication() {};

    /**
     * @brief Initializes driver parameters.
     *
     * @param tty_port modbus virtual port
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void initialize(std::string tty_port) {
        protocol_logic_.initialize(tty_port);
    };

    /**
     * @brief Configures driver session.
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void configure() {
        protocol_logic_.configure();
    };

    /**
     * @brief Deinitializes driver.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void cleanup() {
        protocol_logic_.cleanup();
    };

    /**
     * @brief Stops the gripper movement.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void stop() {
        protocol_logic_.stop();
    };

    /**
     * @brief Resets the gripper by deactivating and reactivating it.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void reset() {
        protocol_logic_.reset();
    };

    /**
     *  Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void auto_release() {
        protocol_logic_.auto_release();
    };

    /**
     * @brief Activates the gripper, making it ready for use.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void activate() {
        protocol_logic_.activate();
    };

    /**
     * @brief Deactivates the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void deactivate() {
        protocol_logic_.reset();
    };

        /**
     * @brief Deactivates the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void shutdown() {
        deactivate();
        cleanup();
    };

    /**
     * @brief Opens the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void open() {
        set_position(GRIPPER_POSITION_MAX);
    };

    /**
     * @brief Closes the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void close() {
        set_position(GRIPPER_POSITION_MIN);
    };

    /**
     * @brief Retrieves the gripper status.
     *
     * @param none
     * @return The current gripper status.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    Status get_status() {
        return status_;
    };

    /**
     * @brief Retrieves the gripper fault status.
     *
     * @param none
     * @return The current gripper fault status.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    FaultStatus get_fault_status() {
        return fault_status_;
    };

    /**
     * @brief Retrieves the requested position of the gripper.
     *
     * @param none
     * @return The requested gripper position in meters.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    double get_requested_position() {
        return  requested_position_;
    };

    /**
     * @brief Retrieves the actual position of the gripper.
     *
     * @param none
     * @return The actual gripper position in meters.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    double get_position() {
        return position_;
    };

    /**
     * @brief Moves the gripper to the requested position.
     *
     * @param position The target position in meters.
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void set_position(double position) {
        protocol_logic_.go_to(
            (uint8_t)((GRIPPER_POSITION_MAX - position) / GRIPPER_POSITION_STEP), MAX_SPEED, MAX_FORCE);
    };

    /**
     * @brief Retrieves the electric current drawn by the gripper.
     *
     * @param none
     * @return The electric current in amperes (range: 0–2.55 A)
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    double get_current() {
        return current_;
    };

    /**
     * @brief Closes the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void read();

    /**
     * @brief Closes the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void write() {
        protocol_logic_.refresh_registers();
    };

private:
    /**
     * Handles protocol logic for mid-level abstraction.
     */
    ProtocolLogic protocol_logic_;

    /**
     * Stores the gripper status bits.
     */
    Status status_;

    /**
     * Stores the fault status bits.
     */
    FaultStatus fault_status_;

    /**
     * Stores the requested position of the gripper in meters.
     */
    double requested_position_;

    /**
     * Stores the actual position of the gripper in meters.
     */
    double position_;

    /**
     * Stores the electric current drawn by the gripper in amperes.
     */
    double current_;

};
}   // namespace robotiq_hande_driver
#endif  // APPLICATION_HPP_
