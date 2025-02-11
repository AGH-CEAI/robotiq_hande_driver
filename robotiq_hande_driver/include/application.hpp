#ifndef APPLICATION_HPP_
#define APPLICATION_HPP_

#include <stdint.h>

#include "protocol_logic.hpp"


namespace hande_driver
{

constexpr auto GRIPPER_POSITION_MIN = 0.0;
constexpr auto GRIPPER_POSITION_MAX = 0.05;
constexpr auto GRIPPER_POSITION_STEP = (GRIPPER_POSITION_MAX - GRIPPER_POSITION_MIN) / 255.0;
constexpr auto GRIPPER_CURRENT_SCALE = 0.01;
constexpr auto MAX_SPEED = 255;
constexpr auto MAX_FORCE = 255;

/**
 * @brief This class contains high level gripper commands and status
 */
class ApplicationLayer{
public:

    struct Status{
        bool is_reset;
        bool is_ready;
        bool is_moving;
        bool is_stopped;
        bool is_opened;
        bool is_closed;
        bool object_detected;
    };

    struct FaultStatus{
        bool is_error;
    };

    ApplicationLayer();

    ~ApplicationLayer() {};

    /**
     * @brief Stops movement of the gripper
     *
     * @param none
     * @return none
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void stop() {
        protocol_logic_.stop();
    };

    /**
     * @brief Resets the gripper: deactivate and activate again
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void reset() {
        protocol_logic_.reset();
    };

    /**
     *  Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void auto_release() {
        protocol_logic_.auto_release();
    };

    /**
     * @brief Activates the gripper, after that it can be used
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void activate() {
        protocol_logic_.activate();
    };

    /**
     * @brief Opens the gripper
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void open() {
        set_position(GRIPPER_POSITION_MAX);
    };

    /**
     * @brief Closes the gripper
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void close() {
        set_position(GRIPPER_POSITION_MIN);
    };

    /**
     * @brief Returns the gripper status
     *
     * @param none
     * @return Gripper status
     * @note see status on success, exception thrown if communicatoin issues
     */
    Status get_status() {
        return status_;
    };

    /**
     * @brief Returns the gripper fault status
     *
     * @param none
     * @return Fault Status
     * @note see status on success, exception thrown if communicatoin issues
     */
    FaultStatus get_fault_status() {
        return fault_status_;
    };

    /**
     * @brief Returns the gripper requested position
     *
     * @param none
     * @return Gripper requested position
     * @note see status on success, exception thrown if communicatoin issues
     */
    double get_requested_position() {
        return  requested_position_;
    };

    /**
     * @brief Returns the gripper position
     *
     * @param none
     * @return Gripper position in [m]
     * @note see status on success, exception thrown if communicatoin issues
     */
    double get_position() {
        return position_;
    };

    /**
     * @brief Moves the gripper to requested position
     *
     * @param position to which the gripper has to move, in [m]
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void set_position(double position) {
        protocol_logic_.go_to(
            (uint8_t)((GRIPPER_POSITION_MAX - position) / GRIPPER_POSITION_STEP), MAX_SPEED, MAX_FORCE);
    };

    /**
     * @brief Returns the gripper current
     *
     * @param none
     * @return Gripper current in [A] (range: 0-2.55A)
     * @note see status on success, exception thrown if communicatoin issues
     */
    double get_current() {
        return current_;
    };

    void read();

    void write() {
        protocol_logic_.refresh_registers();
    };

private:
    /**
     * Protocol logic, used for middle level abstraction
     */
    ProtocolLogic protocol_logic_;

    /**
     * Struct with status bits
     */
    Status status_;

    /**
     * Struct with fault status bits
     */
    FaultStatus fault_status_;

    /**
     * Requested position of the gripper, in [m]
     */
    double requested_position_;

    /**
     * POsition of the gripper, in [m]
     */
    double position_;

    /**
     * Current flowin through the gripper, in [A]
     */
    double current_;

};
}   // namespace hande_driver
#endif  // APPLICATION_HPP_
