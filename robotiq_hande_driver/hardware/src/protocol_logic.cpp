#include "robotiq_hande_driver/protocol_logic.hpp"

namespace robotiq_hande_driver {

ProtocolLogic::ProtocolLogic()
    : raw_status_{},
      activation_status_{ActivationStatus::GRIPPER_RESET},
      action_status_{ActionStatus::STOPPED},
      gripper_status_{GripperStatus::NOT_USED},
      object_detection_status_{ObjectDetectionStatus::REQ_POS_NO_OBJECT},
      raw_fault_status_{},
      raw_position_request_{},
      raw_position_{},
      raw_current_{} {}

void ProtocolLogic::initialize(const CommunicationConfig& cfg) {
    communication_.initialize(cfg);
}

void ProtocolLogic::configure() {
    activation_status_ = ActivationStatus::GRIPPER_RESET;
    action_status_ = ActionStatus::STOPPED;
    gripper_status_ = GripperStatus::NOT_USED;
    object_detection_status_ = ObjectDetectionStatus::REQ_POS_NO_OBJECT;
    raw_fault_status_ = 0;
    raw_position_request_ = 0;
    raw_position_ = 0;
    raw_current_ = 0;

    communication_.configure();
}

void ProtocolLogic::cleanup() {
    communication_.cleanup();
}

void ProtocolLogic::reset() {
    output_bytes_.fill(0);
    write_action_bit(
        static_cast<uint>(ActionRequestPositionBit::ACTIVATE),
        static_cast<bool>(Activate::DEACTIVATE_GRIPPER));
    write_output_bytes();
}

void ProtocolLogic::set() {
    output_bytes_.fill(0);
    write_action_bit(
        static_cast<uint>(ActionRequestPositionBit::ACTIVATE),
        static_cast<bool>(Activate::ACTIVATE_GRIPPER));
    write_output_bytes();
}

void ProtocolLogic::auto_release() {
    write_action_bit(
        static_cast<uint>(ActionRequestPositionBit::AUTOMATIC_RELEASE),
        static_cast<bool>(AutomaticRelease::EMERGENCY_AUTO_RELEASE));
    write_action_bit(
        static_cast<uint>(ActionRequestPositionBit::AUTOMATIC_RELEASE_DIRECTION),
        static_cast<bool>(AutoReleaseDirection::OPENING));
    write_output_bytes();
}

void ProtocolLogic::activate() {
    reset();
    set();
}

void ProtocolLogic::deactivate() {
    reset();
}

void ProtocolLogic::go_to(uint8_t position, uint8_t velocity, uint8_t force) {
    write_action_bit(
        static_cast<uint>(ActionRequestPositionBit::GO_TO), static_cast<bool>(GoTo::GO_TO_REQ_POS));
    set_output_byte(OutputBytes::POSITION_REQUEST, position);
    set_output_byte(OutputBytes::SPEED, velocity);
    set_output_byte(OutputBytes::FORCE, force);
    write_output_bytes();
}

void ProtocolLogic::stop() {
    write_action_bit(
        static_cast<uint>(ActionRequestPositionBit::GO_TO), static_cast<bool>(GoTo::STOP));
    write_output_bytes();
}

bool ProtocolLogic::is_reset() const {
    return (
        gripper_status_ == GripperStatus::GRIPPER_IN_RESET
        && activation_status_ == ActivationStatus::GRIPPER_RESET);
}

bool ProtocolLogic::is_ready() const {
    return (
        gripper_status_ == GripperStatus::ACTIVATION_COMPLETE
        && activation_status_ == ActivationStatus::GRIPPER_ACTIVATION);
}

bool ProtocolLogic::is_moving() const {
    return (
        action_status_ == ActionStatus::GO_TO_POSITION_REQUEST
        && object_detection_status_ == ObjectDetectionStatus::MOTION_NO_OBJECT);
}

bool ProtocolLogic::is_stopped() const {
    return object_detection_status_ != ObjectDetectionStatus::MOTION_NO_OBJECT;
}

bool ProtocolLogic::is_closed() const {
    return raw_position_ >= GRIPPER_POSITION_OPENED_THRESHOLD;
}

bool ProtocolLogic::is_opened() const {
    return raw_position_ <= GRIPPER_POSITION_CLOSED_THRESHOLD;
}

bool ProtocolLogic::obj_detected() const {
    return (
        object_detection_status_ == ObjectDetectionStatus::STOPPED_OPENING_DETECTED
        || object_detection_status_ == ObjectDetectionStatus::STOPPED_CLOSING_DETECTED);
}

uint8_t ProtocolLogic::get_raw_requested_pos() const {
    return raw_position_request_;
}

uint8_t ProtocolLogic::get_raw_pos() const {
    return raw_position_;
}

uint8_t ProtocolLogic::get_raw_current() const {
    return raw_current_;
}

void ProtocolLogic::read_input_bytes() {
    communication_.read(input_bytes_);

    raw_status_ = get_input_byte(InputBytes::GRIPPER_STATUS);

    activation_status_ =
        (ActivationStatus)((raw_status_ >> static_cast<uint>(StatusPositionBit::ACTIVATION_STATUS))
                           & ACTIVATION_STATUS_BITS);

    action_status_ =
        (ActionStatus)((raw_status_ >> static_cast<uint>(StatusPositionBit::ACTION_STATUS))
                       & ACTION_STATUS_BITS);

    gripper_status_ =
        (GripperStatus)((raw_status_ >> static_cast<uint>(StatusPositionBit::GRIPPER_STATUS))
                        & GRIPPER_STATUS_BITS);

    object_detection_status_ =
        (ObjectDetectionStatus)((raw_status_
                                 >> static_cast<uint>(StatusPositionBit::OBJECT_DETECTION_STATUS))
                                & OBJECT_DETECTION_STATUS_BITS);

    // TODO(issue#9) Read Hand-E fault status flags
    raw_fault_status_ = get_input_byte(InputBytes::FAULT_STATUS);

    raw_position_request_ = get_input_byte(InputBytes::POSITION_REQUEST_ECHO);
    raw_position_ = get_input_byte(InputBytes::POSITION);
    raw_current_ = get_input_byte(InputBytes::CURRENT);
}

void ProtocolLogic::write_output_bytes() {
    communication_.write(output_bytes_);
}

uint ProtocolLogic::bit_set_to(uint value, uint n, bool x) const {
    uint reset_n_bit = ~(1u << n);
    uint set_n_bit = static_cast<uint>(x) << n;
    return (value & reset_n_bit) | set_n_bit;
}

void ProtocolLogic::write_action_bit(uint8_t position_bit, bool value) {
    auto idx = static_cast<uint>(OutputBytes::ACTION_REQUEST);
    auto reg = output_bytes_[idx];
    auto result = bit_set_to(reg, position_bit, value);
    output_bytes_[idx] = result;
}

uint8_t ProtocolLogic::get_input_byte(InputBytes index) const {
    auto idx = static_cast<uint>(index);
    return input_bytes_[idx];
}

void ProtocolLogic::set_output_byte(OutputBytes index, uint8_t value) {
    auto idx = static_cast<uint>(index);
    output_bytes_[idx] = value;
}

}  // namespace robotiq_hande_driver
