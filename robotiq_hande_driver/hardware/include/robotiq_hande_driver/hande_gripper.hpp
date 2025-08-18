#ifndef ROBOTIQ_HANDE_DRIVER__HANDE_GRIPPER_HPP_
#define ROBOTIQ_HANDE_DRIVER__HANDE_GRIPPER_HPP_

#include <stdint.h>

#include "robotiq_hande_driver/protocol_logic.hpp"

namespace robotiq_hande_driver {

static constexpr auto GRIPPER_CURRENT_SCALE = 0.01;
static constexpr auto MAX_SPEED = 255;
static constexpr auto MAX_FORCE = 255;

static constexpr auto EPSILON = 1e-6f;

/**
 * @brief Contains high-level gripper commands and status.
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
     * @param cfg Modbus communication cfg.
     */
    void initialize(
        double gripper_position_min, double gripper_position_max, const CommunicationConfig& cfg);

    /**
     * @brief Configures driver session.
     */
    void configure();

    /**
     * @brief Deinitializes driver.
     */
    void cleanup();

    /**
     * @brief Stops the gripper movement.
     */
    void stop();

    /**
     * @brief Resets the gripper by deactivating and reactivating it.
     */
    void reset();

    /**
     *  Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     */
    void auto_release();

    /**
     * @brief Activates the gripper, making it ready for use.
     */
    void activate();

    /**
     * @brief Deactivates the gripper.
     */
    void deactivate();

    /**
     * @brief Deactivates the gripper.
     */
    void shutdown();

    /**
     * @brief Opens the gripper.
     */
    void open();

    /**
     * @brief Closes the gripper.
     */
    void close();

    /**
     * @brief Retrieves the gripper status.
     *
     * @return The current gripper status.
     */
    Status get_status() const;

    /**
     * @brief Retrieves the gripper fault status.
     *
     * @return The current gripper fault status.
     */
    FaultStatus get_fault_status() const;

    /**
     * @brief Retrieves the requested position of the gripper.
     *
     * @return The requested gripper position in meters.
     */
    double get_requested_position() const;

    /**
     * @brief Retrieves the actual position of the gripper.
     *
     * @return The actual gripper position in meters.
     */
    double get_position() const;

    /**
     * @brief Moves the gripper to the requested position.
     *
     * @param position The target position in meters.
     */
    void set_position(double position, double force = 1.0);

    /**
     * @brief Retrieves the electric current drawn by the gripper.
     *
     * @return The electric current in amperes (range: 0–2.55 A)
     */
    double get_current() const;

    /**
     * @brief Reads gripper data.
     */
    void read();

    /**
     * @brief Writes gripper data.
     */
    void write();

   private:
    // Handles protocol logic for mid-level abstraction
    ProtocolLogic prot_;

    // Stores the gripper status bits
    Status status_;

    // Stores the fault status bits
    FaultStatus fault_status_;

    // Stores the requested position of the gripper in meters
    double requested_position_;

    // Stores the actual position of the gripper in meters
    double position_;

    // Stores the electric current drawn by the gripper in amperes
    double current_;

    double gripper_position_min_;
    double gripper_position_max_;
    double gripper_postion_step_;
};
}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__HANDE_GRIPPER_HPP_
