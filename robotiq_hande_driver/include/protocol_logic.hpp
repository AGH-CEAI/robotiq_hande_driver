#ifndef PROTOCOL_LOGIC_HPP_
#define PROTOCOL_LOGIC_HPP_

#include <communication.hpp>


namespace hande_driver
{

/* Register mapping done based on Hand-E documentation:
 * https://assets.robotiq.com/website-assets/support_documents/document/Hand-E_Instruction_Manual_e-Series_PDF_20190306.pdf
 */
enum class ActionRequestPositionBit : uint8_t {
    ACTIVATE = 0u,                      /* rACT */
    GO_TO = 3u,                         /* rGTO */
    AUTOMATIC_RELEASE = 4u,             /* rATR */
    AUTOMATIC_RELEASE_DIRECTION = 5u,   /* rARD */
};

enum class Activate : uint8_t {
    DEACTIVATE_GRIPPER = 0u,
    ACTIVATE_GRIPPER
};

enum class GoTo : uint8_t {
    STOP = 0u,
    GO_TO_REQ_POS
};

enum class AutomaticRelease : uint8_t {
    NORMAL = 0u,
    EMERGENCY_AUTO_RELEASE
};

enum class AutoReleaseDirection : uint8_t  {
    CLOSING = 0u,
    OPENING
};

// Gripper Status
enum class ResponseByte : uint8_t {
    STATUS = 0u,
    FAULT_STATUS = 2u,
    SPEED = 4u,
    FORCE = 5u,
};

enum class StatusPositionBit : uint8_t {
    ACTIVATION_STATUS = 0u,         /* gACT */
    ACTION_STATUS = 3u,             /* gGTO */
    GRIPPER_STATUS = 4u,            /* gSTA */
    OBJECT_DETECTION_STATUS = 6u,   /* gObj */
};

constexpr auto kActivationStatusBits = 0b1;
enum class ActivationStatus : uint8_t {
    GRIPPER_RESET = 0u,
    GRIPPER_ACTIVATION
};

constexpr auto kActionStatusBits = 0b1;
enum class ActionStatus : uint8_t {
    STOPPED = 0u,
    GO_TO_POSITION_REQUEST
};

constexpr auto kGripperStatusBits = 0b11;
enum class GripperStatus : uint8_t {
    GRIPPER_IN_RESET = 0u,
    ACTIVATION_IN_PROGRESS,
    NOT_USED,
    ACTIVATION_COMPLETE
};

constexpr auto kObjectDetectionStatusBits = 0b11;
enum class ObjectDetectionStatus : uint8_t {
    MOTION_NO_OBJECT = 0u,
    STOPPED_OPENING_DETECTED,
    STOPPED_CLOSING_DETECTED,
    REQ_POS_NO_OBJECT
};

constexpr auto kGripperPositionOpenedThreshold = 230;
constexpr auto kGripperPositionClosedThreshold = 13;

/**
 * @brief This class contains protocol oriented functions and definitions
 */
class ProtocolLogic{
public:
    ProtocolLogic();

    ~ProtocolLogic() {};

    /**
     *  @brief Resets the gripper
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void reset() {
        communication_.clear_output_bytes();
        communication_.write_action_bit((uint)ActionRequestPositionBit::ACTIVATE, (bool)Activate::DEACTIVATE_GRIPPER);
        communication_.read_write_registers();
    };

    /**
     *  @brief Sets the gripper
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void set() {
        communication_.clear_output_bytes();
        communication_.write_action_bit((uint)ActionRequestPositionBit::ACTIVATE, (bool)Activate::ACTIVATE_GRIPPER);
        communication_.read_write_registers();
    };

    /**
     * @brief Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void auto_release() {
        communication_.write_action_bit((uint)ActionRequestPositionBit::AUTOMATIC_RELEASE, (bool)AutomaticRelease::EMERGENCY_AUTO_RELEASE);
        communication_.write_action_bit((uint)ActionRequestPositionBit::AUTOMATIC_RELEASE_DIRECTION, (bool)AutoReleaseDirection::OPENING);
        communication_.read_write_registers();
    };

    /**
     * @brief Activates the gripper, after that it can be used
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void activate() {
        reset();
        set();
    };

    /**
     * @brief Moves the gripper
     *
     * @param position requested position
     * @param velocity requested position
     * @param force requested position
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void go_to(uint8_t position, uint8_t velocity, uint8_t force) {
        communication_.write_action_bit((uint)ActionRequestPositionBit::GO_TO, (bool)GoTo::GO_TO_REQ_POS);
        communication_.set_output_byte(OutputBytes::POSITION_REQUEST, position);
        communication_.set_output_byte(OutputBytes::SPEED, velocity);
        communication_.set_output_byte(OutputBytes::FORCE, force);
        communication_.read_write_registers();
    };

    /**
     * @brief Stops the gripper
     *
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void stop() {
        communication_.write_action_bit((uint)ActionRequestPositionBit::GO_TO, (bool)GoTo::STOP);
        communication_.read_write_registers();
    };

    /**
     * @brief Logic for reset state
     *
     * @return True if gripper is in reset state
     */
    bool is_reset() {
        return (gripper_status_ == GripperStatus::GRIPPER_IN_RESET &&
                activation_status_ == ActivationStatus::GRIPPER_RESET);
    };

    /**
     * @brief Logic for ready state
     *
     * @return True if gripper is ready
     */
    bool is_ready() {
        return (gripper_status_ == GripperStatus::ACTIVATION_COMPLETE &&
                activation_status_ == ActivationStatus::GRIPPER_ACTIVATION);
    };

    /**
     * @brief Logic for moving state
     *
     * @return True if gripper is moving
     */
    bool is_moving() {
    return (action_status_ == ActionStatus::GO_TO_POSITION_REQUEST &&
            object_detection_status_ == ObjectDetectionStatus::MOTION_NO_OBJECT);
    };

    /**
     * @brief Logic for stopped state
     *
     * @return True if gripper is stopped
     */
    bool is_stopped() {
        return object_detection_status_ != ObjectDetectionStatus::MOTION_NO_OBJECT;
    };

    /**
     * @brief Logic for closed state
     *
     * @return True if gripper is closed
     */
    bool is_closed() {
        return position_ >= kGripperPositionOpenedThreshold;
    };

    /**
     * @brief Logic for opened state
     *
     * @return True if gripper is opened
     */
    bool is_opened() {
        return position_ <= kGripperPositionClosedThreshold;
    };

    /**
     * @brief Logic for detecting the object
     *
     * @return True if gripper has detected the object
     */
    bool obj_detected() {
    return (object_detection_status_ == ObjectDetectionStatus::STOPPED_OPENING_DETECTED ||
            object_detection_status_ == ObjectDetectionStatus::STOPPED_CLOSING_DETECTED);
    };

    /**
     * @brief Getter of requested gripper position
     *
     * @return Requested gripper position
     */
    uint8_t get_reg_pos() {
        return position_request_echo_;
    };

    /**
     * @brief Getter of current gripper position
     *
     * @return Current gripper position
     */
    uint8_t get_pos() {
        return position_;
    };

    /**
     * @brief Getter of current
     *
     * @return Current gripper
     */
    uint8_t get_current() {
        return current_;
    };

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
