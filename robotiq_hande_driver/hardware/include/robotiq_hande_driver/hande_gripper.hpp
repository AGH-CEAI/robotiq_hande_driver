#ifndef ROBOTIQ_HANDE_DRIVER__HANDE_GRIPPER_HPP_
#define ROBOTIQ_HANDE_DRIVER__HANDE_GRIPPER_HPP_

#include <stdint.h>
#include <unistd.h>

#include "robotiq_hande_driver/protocol_logic.hpp"

namespace robotiq_hande_driver {

static constexpr auto GRIPPER_CURRENT_SCALE = 0.01;
static constexpr auto MAX_SPEED = 255;
static constexpr auto MAX_FORCE = 255;

/**
 * @brief This class contains high-level gripper commands and status.
 */
class HandeGripper {
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

    HandeGripper();

    ~HandeGripper() = default;

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
     */
    void initialize(
        double gripper_position_min,
        double gripper_position_max,
        const std::string& tty_port,
        int baudrate,
        char parity,
        int data_bits,
        int stop_bit,
        int slave_id);

    /**
     * @brief Configures driver session.
     * @return int Connection status code.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    int configure();

    /**
     * @brief Deinitializes driver.
     *
     * @param none
     * @return None.
     */
    void cleanup();

    /**
     * @brief Stops the gripper movement.
     *
     * @param none
     * @return None.
     */
    void stop();

    /**
     * @brief Resets the gripper by deactivating and reactivating it.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void reset();

    /**
     *  Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void auto_release();

    /**
     * @brief Activates the gripper, making it ready for use.
     *
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void activate();

    /**
     * @brief Deactivates the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void deactivate();

    /**
     * @brief Deactivates the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void shutdown();

    /**
     * @brief Opens the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void open();

    /**
     * @brief Closes the gripper.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void close();

    /**
     * @brief Retrieves the gripper status.
     *
     * @param none
     * @return The current gripper status.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    Status get_status() const;

    /**
     * @brief Retrieves the gripper fault status.
     *
     * @param none
     * @return The current gripper fault status.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    FaultStatus get_fault_status() const;

    /**
     * @brief Retrieves the requested position of the gripper.
     *
     * @param none
     * @return The requested gripper position in meters.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    double get_requested_position() const;

    /**
     * @brief Retrieves the actual position of the gripper.
     *
     * @param none
     * @return The actual gripper position in meters.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    double get_position() const;

    /**
     * @brief Moves the gripper to the requested position.
     *
     * @param position The target position in meters.
     * @return None.
     */
    void set_position(double position, double force = 1.0);

    /**
     * @brief Retrieves the electric current drawn by the gripper.
     *
     * @param none
     * @return The electric current in amperes (range: 0–2.55 A)
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    double get_current() const;

    /**
     * @brief Reads gripper data.
     *
     * @param none
     * @return None.
     */
    void read();

    /**
     * @brief Writes gripper data.
     *
     * @param none
     * @return None.
     */
    void write();

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
#endif  // ROBOTIQ_HANDE_DRIVER__HANDE_GRIPPER_HPP_
