#ifndef PROTOCOL_LOGIC_HPP_
#define PROTOCOL_LOGIC_HPP_

#include <stdint.h>

#include <communication.hpp>

/**
 * @brief This class contains protocol oriented functions and definitions
 */

namespace hande_driver
{

// Action Request
constexpr uint8_t kActionRequestByte = 0;
constexpr uint8_t kActivatePositionByte = 0;             // rACT
enum class Activate : uint8_t {
    DEACTIVATE_GRIPPER = 0u,
    ACTIVATE_GRIPPER
};

constexpr uint8_t kGoToPositionByte = 3;                 // rGTO
enum class GoTo : uint8_t {
    STOP = 0u,
    GO_TO_REQ_POS
};

constexpr uint8_t kAutomaticReleasePositionByte = 4;     // rATR
enum class AutomaticRelease : uint8_t {
    NORMAL = 0u,
    EMERGENCY_AUTO_RELEASE
};

constexpr uint8_t kAutoReleaseDirectionPositionByte = 5; // rARD
enum class AutoReleaseDirection : uint8_t  {
    CLOSING = 0u,
    OPENING
};

//Position Request
constexpr uint8_t kPositionRequestByte = 3;
// 0x00 - Open position, with 50 mm opening
// 0xFF - Closed
// Opening / count: ≈0.2 mm for 50 mm stroke

//Speed Request
constexpr uint8_t kSpeedRequestByte = 4;
// 0x00 - Minimum speed
// 0xFF - Maximum speed

//Force Request
constexpr uint8_t kForceRequestByte = 5;
// 0x00 - Minimum force
// 0xFF - Maximum force

// Gripper Status
constexpr uint8_t kStatusByte = 0;
constexpr uint8_t kActivationStatusPositionByte = 0;         // gACT
constexpr auto kActivationStatusBits = 0b1;
enum class ActivationStatus : uint8_t {
    GRIPPER_RESET = 0u,
    GRIPPER_ACTIVATION
};

constexpr uint8_t kActionStatusPositionByte = 3;               // gGTO
constexpr auto kActionStatusBits = 0b1;
enum class ActionStatus : uint8_t {
    STOPPED = 0u,
    GO_TO_POSITION_REQUEST
};

constexpr uint8_t kGripperStatusPositionByte = 4;            // gSTA
constexpr auto kGripperStatusBits = 0b11;
enum class GripperStatus : uint8_t {
    GRIPPER_IN_RESET = 0u,
    ACTIVATION_IN_PROGRESS,
    NOT_USED,
    ACTIVATION_COMPLETE
};

constexpr uint8_t kObjectDetectionStatusPositionByte = 6;    // gObj
constexpr auto kObjectDetectionStatusBits = 0b11;
enum class ObjectDetectionStatus : uint8_t {
    MOTION_NO_OBJECT = 0u,
    STOPPED_OPENING_DETECTED,
    STOPPED_CLOSING_DETECTED,
    REQ_POS_NO_OBJECT
};

// Fault Status
constexpr uint8_t kFaultStatusByte = 2;
// Position Request Echo
constexpr uint8_t kPositionRequestEchoByte = 3;
// Position (current)
constexpr uint8_t kPositionByte = 4;
// Current
constexpr uint8_t kCurrentByte = 5;

class ProtocolLogic{
public:
    ProtocolLogic();

    ~ProtocolLogic();

    /**
     *  @brief Resets the gripper
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void reset();

    /**
     *  @brief Sets the gripper
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void set();

    /**
     * @brief Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void auto_release();

    /**
     * @brief Activates the gripper, after that it can be used
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void activate();

    /**
     * @brief Moves the gripper
     *
     * @param position requested position
     * @param velocity requested position
     * @param force requested position
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void go_to(uint8_t position, uint8_t velocity, uint8_t force);

    /**
     * @brief Stops the gripper
     *
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void stop();

    /**
     * @brief Logic for reset state
     *
     * @return True if gripper is in reset state
     */
    bool is_reset();

    /**
     * @brief Logic for ready state
     *
     * @return True if gripper is ready
     */
    bool is_ready();

    /**
     * @brief Logic for moving state
     *
     * @return True if gripper is moving
     */
    bool is_moving();

    /**
     * @brief Logic for stopped state
     *
     * @return True if gripper is stopped
     */
    bool is_stopped();

    /**
     * @brief Logic for closed state
     *
     * @return True if gripper is closed
     */
    bool is_closed();

    /**
     * @brief Logic for opened state
     *
     * @return True if gripper is opened
     */
    bool is_opened();

    /**
     * @brief Logic for detecting the object
     *
     * @return True if gripper has detected the object
     */
    bool obj_detected();

    /**
     * @brief Getter of requested gripper position
     *
     * @return Requested gripper position
     */
    uint8_t get_reg_pos();

    /**
     * @brief Getter of current gripper position
     *
     * @return Current gripper position
     */
    uint8_t get_pos();

    /**
     * @brief Getter of current
     *
     * @return Current gripper
     */
    uint8_t get_current();

    /**
     * @brief Decode modbus registers and refresh appropriate data
     *
     * @return none
     */
    void refresh_registers();

private:
// Gripper
    uint8_t status_;
    ActivationStatus activation_status_;
    ActionStatus action_status_;
    GripperStatus gripper_status_;
    ObjectDetectionStatus object_detection_status_;
//Fault
    uint8_t fault_status_;

    /**
     * @brief Requested position in normalized 0-255 value
     */
    uint8_t position_request_echo_;

    /**
     * @brief Position in normalized 0-255 value
     */
    uint8_t position_;

    /**
     * @brief Current in 10 mA
     */
    uint8_t current_;

    Communication communication_;
};
}   // namespace hande_driver
#endif  // PROTOCOL_LOGIC_HPP_
