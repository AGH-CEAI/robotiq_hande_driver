#ifndef ROBOTIQ_HANDE_DRIVER__PROTOCOL_LOGIC_HPP_
#define ROBOTIQ_HANDE_DRIVER__PROTOCOL_LOGIC_HPP_

#include "robotiq_hande_driver/communication.hpp"

namespace robotiq_hande_driver {

/* Register mapping done based on Hand-E documentation:
 * https://assets.robotiq.com/website-assets/support_documents/document/Hand-E_Instruction_Manual_e-Series_PDF_20190306.pdf
 */
enum class ActionRequestPositionBit : uint8_t {
    ACTIVATE = 0u,                    /* rACT */
    GO_TO = 3u,                       /* rGTO */
    AUTOMATIC_RELEASE = 4u,           /* rATR */
    AUTOMATIC_RELEASE_DIRECTION = 5u, /* rARD */
};

enum class Activate : uint8_t { DEACTIVATE_GRIPPER = 0u, ACTIVATE_GRIPPER };

enum class GoTo : uint8_t { STOP = 0u, GO_TO_REQ_POS };

enum class AutomaticRelease : uint8_t { NORMAL = 0u, EMERGENCY_AUTO_RELEASE };

enum class AutoReleaseDirection : uint8_t { CLOSING = 0u, OPENING };

/* Gripper Status */
enum class ResponseByte : uint8_t {
    STATUS = 0u,
    FAULT_STATUS = 2u,
    SPEED = 4u,
    FORCE = 5u,
};

enum class StatusPositionBit : uint8_t {
    ACTIVATION_STATUS = 0u,       /* gACT */
    ACTION_STATUS = 3u,           /* gGTO */
    GRIPPER_STATUS = 4u,          /* gSTA */
    OBJECT_DETECTION_STATUS = 6u, /* gObj */
};

static constexpr auto ACTIVATION_STATUS_BITS = 0b1;
enum class ActivationStatus : uint8_t { GRIPPER_RESET = 0u, GRIPPER_ACTIVATION };

static constexpr auto ACTION_STATUS_BITS = 0b1;
enum class ActionStatus : uint8_t { STOPPED = 0u, GO_TO_POSITION_REQUEST };

static constexpr auto GRIPPER_STATUS_BITS = 0b11;
enum class GripperStatus : uint8_t {
    GRIPPER_IN_RESET = 0u,
    ACTIVATION_IN_PROGRESS,
    NOT_USED,
    ACTIVATION_COMPLETE
};

static constexpr auto OBJECT_DETECTION_STATUS_BITS = 0b11;
enum class ObjectDetectionStatus : uint8_t {
    MOTION_NO_OBJECT = 0u,
    STOPPED_OPENING_DETECTED,
    STOPPED_CLOSING_DETECTED,
    REQ_POS_NO_OBJECT
};

static constexpr auto GRIPPER_POSITION_OPENED_THRESHOLD = 230;
static constexpr auto GRIPPER_POSITION_CLOSED_THRESHOLD = 13;

/**
 * @brief Contains Robotiq's protocol oriented functions and definitions.
 */
class ProtocolLogic {
   public:
    ProtocolLogic();

    ~ProtocolLogic() = default;

    /**
     * @brief Initializes driver parameters.
     *
     * @param cfg Modbus communication config.
     */
    void initialize(const CommunicationConfig& cfg);

    /**
     * @brief Configures protocol layer.
     */
    void configure();

    /**
     * @brief Deinitializes protocol layer.
     */
    void cleanup();

    /**
     *  @brief Resets the gripper.
     */
    void reset();

    /**
     *  @brief Sets the gripper.
     */
    void set();

    /**
     * @brief Performs an emergency auto-release. The fingers slowly open, requiring reactivation.
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
     * @brief Moves the gripper to the requested position with specified velocity and force.
     *
     * @param position The requested position.
     * @param velocity The requested velocity.
     * @param force The requested force.
     */
    void go_to(uint8_t position, uint8_t velocity, uint8_t force);

    /**
     * @brief Stops the gripper.
     */
    void stop();

    /**
     * @brief Checks if the gripper is in reset state.
     *
     * @return True if the gripper is in reset state.
     */
    bool is_reset() const;

    /**
     * @brief Checks if the gripper is in ready state.
     *
     * @return True if the gripper is in ready state.
     */
    bool is_ready() const;

    /**
     * @brief Checks if the gripper is moving.
     *
     * @return True if gripper is moving.
     */
    bool is_moving() const;

    /**
     * @brief Checks if the gripper is stopped.
     *
     * @return True if gripper is stopped.
     */
    bool is_stopped() const;

    /**
     * @brief Checks if the gripper is closed.
     *
     * @return True if gripper is closed.
     */
    bool is_closed() const;

    /**
     * @brief Checks if the gripper is opened.
     *
     * @return True if gripper is opened.
     */
    bool is_opened() const;

    /**
     * @brief Checks if the gripper has detected an object.
     *
     * @return True if gripper has detected an object.
     */
    bool obj_detected() const;

    /**
     * @brief Retrieves the requested position of the gripper.
     *
     * @return Requested gripper position in raw value.
     */
    uint8_t get_raw_requested_pos() const;

    /**
     * @brief Retrieves the actual position of the gripper.
     *
     * @return The actual gripper position in raw value.
     */
    uint8_t get_raw_pos() const;

    /**
     * @brief Retrieves the electric current drawn by the gripper.
     *
     * @return The raw value of the electric current
     */
    uint8_t get_raw_current() const;

    /**
     * @brief Copies the the communication input registers into the working memory.
     */
    void read_input_bytes();

    /**
     * @brief Copies the modified output bytes to the communication output registers.
     */
    void write_output_bytes();

   private:
    // Methods for accessing and manipulating auxiary arrays
    uint bit_set_to(uint value, uint n, bool x) const;
    void write_action_bit(uint8_t position_bit, bool value);

    uint8_t get_input_byte(InputBytes index) const;
    void set_output_byte(OutputBytes index, uint8_t value);

    // Handles low-level communication in a separate thread
    Communication communication_;

    // Auxiary array for storing read values
    InputBuffer input_bytes_;

    // Auxiary array for preparing all registers before sending them
    OutputBuffer output_bytes_;

    // Gripper
    uint8_t raw_status_;
    ActivationStatus activation_status_;
    ActionStatus action_status_;
    GripperStatus gripper_status_;
    ObjectDetectionStatus object_detection_status_;

    // Fault
    uint8_t raw_fault_status_;

    // Stores the requested position in normalized 0–255 value
    uint8_t raw_position_request_;

    // Stores the actual position in normalized 0–255 value
    uint8_t raw_position_;

    // Stores the electric current drawn by the gripper in 10 mA value
    uint8_t raw_current_;
};
}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__PROTOCOL_LOGIC_HPP_
