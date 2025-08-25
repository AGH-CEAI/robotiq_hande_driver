#ifndef ROBOTIQ_HANDE_DRIVER__APPLICATION_HPP_
#define ROBOTIQ_HANDE_DRIVER__APPLICATION_HPP_

#include <stdint.h>
#include <unistd.h>

#include "protocol_logic.hpp"

namespace robotiq_hande_driver {

static constexpr auto GRIPPER_CURRENT_SCALE = 0.01;
static constexpr auto MAX_SPEED = 255;
static constexpr auto MAX_FORCE = 255;

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
     * @param gripper_position_min Minimal gripper position in meters.
     * @param gripper_position_max Maximal gripper position in meters.
     * @param tty_port Modbus virtual port.
     * @param baudrate Modbus serial baudrate.
     * @param parity Modbus serial parity.
     * @param data_bits Modbus serial data bits.
     * @param stop_bit Modbus serial stopbit.
     * @param slave_id Modbus slave id.
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void initialize(
        double gripper_position_min,
        double gripper_position_max,
        const std::string& tty_port,
        int baudrate,
        char parity,
        int data_bits,
        int stop_bit,
        int slave_id) {
        gripper_position_min_ = gripper_position_min;
        gripper_position_max_ = gripper_position_max;
        gripper_postion_step_ = (gripper_position_max_ - gripper_position_min_) / 255.0;
        protocol_logic_.initialize(tty_port, baudrate, parity, data_bits, stop_bit, slave_id);
    };

    /**
     * @brief Configures driver session.
     * @return int Connection status code.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    int configure() {
        int result;

        result = protocol_logic_.configure();

        return result;
    };

    /**
     * @brief Deinitializes driver.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void cleanup() {
        protocol_logic_.cleanup();
    };

    /**
     * @brief Stops the gripper movement.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void stop() {
        protocol_logic_.stop();
    };

    /**
     * @brief Resets the gripper by deactivating and reactivating it.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void reset() {
        protocol_logic_.reset();
    };

    /**
     *  Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void auto_release() {
        protocol_logic_.auto_release();
    };

    /**
     * @brief Activates the gripper, making it ready for use.
     *
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void activate() {
        protocol_logic_.activate();
    };

    /**
     * @brief Deactivates the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void deactivate() {
        protocol_logic_.reset();
    };

    /**
     * @brief Deactivates the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
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
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void open() {
        set_position(gripper_position_max_);
    };

    /**
     * @brief Closes the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void close() {
        set_position(gripper_position_min_);
    };

    /**
     * @brief Retrieves the gripper status.
     *
     * @param none
     * @return The current gripper status.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    Status get_status() {
        return status_;
    };

    /**
     * @brief Retrieves the gripper fault status.
     *
     * @param none
     * @return The current gripper fault status.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    FaultStatus get_fault_status() {
        return fault_status_;
    };

    /**
     * @brief Retrieves the requested position of the gripper.
     *
     * @param none
     * @return The requested gripper position in meters.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    double get_requested_position() {
        return requested_position_;
    };

    /**
     * @brief Retrieves the actual position of the gripper.
     *
     * @param none
     * @return The actual gripper position in meters.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    double get_position() {
        return position_;
    };

    /**
     * @brief Moves the gripper to the requested position.
     *
     * @param position The target position in meters.
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void set_position(double position, double force = 1.0) {
        uint8_t scaled_force = static_cast<uint8_t>(force * MAX_FORCE);
        protocol_logic_.go_to(
            (uint8_t)((gripper_position_max_ - position) / gripper_postion_step_),
            MAX_SPEED,
            scaled_force);
    };

    /**
     * @brief Retrieves the electric current drawn by the gripper.
     *
     * @param none
     * @return The electric current in amperes (range: 0–2.55 A)
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    double get_current() {
        return current_;
    };

    /**
     * @brief Reads gripper data.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void read();

    /**
     * @brief Writes gripper data.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
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

    double gripper_position_min_;
    double gripper_position_max_;
    double gripper_postion_step_;
};
}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__APPLICATION_HPP_
