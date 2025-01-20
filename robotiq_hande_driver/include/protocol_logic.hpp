#include <stdint.h>

#include <communication.hpp>

/**
 * @brief This class contains high level gripper commands and status
 */

namespace hande_driver
{

// Action Request
constexpr uint8_t activatePositionByte = 0;             // rACT
enum Activate{
    DEACTIVATE_GRIPPER,
    ACTIVATE_GRIPPER
};

constexpr uint8_t goToPositionByte = 3;                 // rGTO
enum GoTo{
    STOP,
    GO_TO_REQ_POS
};

constexpr uint8_t automaticReleasePositionByte = 4;     // rATR
enum AutomaticRelease{
    NORMAL,
    EMERGENCY_AUTO_RELEASE
};

constexpr uint8_t autoReleaseDirectionPositionByte = 5; // rARD
enum AutoReleaseDirection{
    CLOSING,
    OPENING
};

//Position Request
// 0x00 - Open position, with 50 mm opening
// 0xFF - Closed
// Opening / count: ≈0.2 mm for 50 mm stroke

//Speed Request
// 0x00 - Minimum speed
// 0xFF - Maximum speed

//Force Request
// 0x00 - Minimum force
// 0xFF - Maximum force

// Gripper Status
constexpr uint8_t activationStatusPositionByte = 0;         // gACT
enum ActivationStatus{
    GRIPPER_RESET,
    GRIPPER_ACTIVATION
};

constexpr uint8_t goToStatusPositionByte = 3;               // gGTO
enum ActionStatus{
    STOPPED,
    GO_TO_POSITION_REQUEST
};

constexpr uint8_t gripperStatusPositionByte = 4;            // gSTA
enum GripperStatus{
    GRIPPER_IN_RESET,
    ACTIVATION_IN_PROGRESS,
    NOT_USED,
    ACTIVATION_COMPLETE
};

constexpr uint8_t objectDetectionStatusPositionByte = 6;    // gObj
enum ObjectDetectionStatus{
    MOTION_NO_OBJECT,
    STOPPED_OPENING_DETECTED,
    STOPPED_CLOSING_DETECTED,
    REQ_POS_NO_OBJECT
};

// Fault Status
// Position Request Echo
// Position (current)
// Current


class ProtocolLogic{
public:
    ProtocolLogic();

    ~ProtocolLogic();

    /**
     *  @brief Resets the gripper: deactivate and activate again
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void reset();

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
    void go_to(uint8_t position, uint8_t velocity, uint8_t force, bool arm_callback=true);

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
     * @brief Arm a callback to be triggered when gripper starts moving
     * 
     * @return none
     */
    void wait_until_moving();

    /**
     * @brief Arm a callback to be triggered when gripper stops moving
     * 
     * @return none
     */
    void wait_until_stopped();

private:
// Gripper
    ActivationStatus activation_status_;
    ActionStatus action_status_;
    GripperStatus gripper_status_;
    ObjectDetectionStatus object_detection_status_;
//Fault


    /**
     * @brief Requested position in normalized 0-225 value
     */
    uint8_t position_request;

    /**
     * @brief Position in normalized 0-225 value
     */
    uint8_t position;

    /**
     * @brief Current in 10 mA
     */
    uint8_t current;

    Communication communication_;
};
}   // namespace hande_driver