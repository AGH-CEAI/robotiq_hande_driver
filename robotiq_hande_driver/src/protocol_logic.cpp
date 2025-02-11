#include "protocol_logic.hpp"

#include <cstdio>


namespace hande_driver  {

constexpr auto kGripperPositionOpenedThreshold = 230;
constexpr auto kGripperPositionClosedThreshold = 13;

ProtocolLogic::ProtocolLogic()
:   status_()
,   activation_status_(ActivationStatus::GRIPPER_RESET)
,   action_status_(ActionStatus::STOPPED)
,   gripper_status_(GripperStatus::NOT_USED)
,   object_detection_status_(ObjectDetectionStatus::REQ_POS_NO_OBJECT)
,   fault_status_()
,   position_request_echo_()
,   position_()
,   current_()
{
    printf("ProtocolLogic constructor\n");
}

ProtocolLogic::~ProtocolLogic(){
    printf("ProtocolLogic destructor\n");
}

void ProtocolLogic::reset(){
    communication_.clear_output_bytes();

    communication_.write_action_bit(kActivatePositionByte, (bool)Activate::DEACTIVATE_GRIPPER);

    communication_.read_write_registers();
}

void ProtocolLogic::set(){
    communication_.clear_output_bytes();

    communication_.write_action_bit(kActivatePositionByte, (bool)Activate::ACTIVATE_GRIPPER);

    communication_.read_write_registers();
}

void ProtocolLogic::auto_release(){
    communication_.write_action_bit(kAutomaticReleasePositionByte, (bool)AutomaticRelease::EMERGENCY_AUTO_RELEASE);
    communication_.write_action_bit(kAutoReleaseDirectionPositionByte, (bool)AutoReleaseDirection::OPENING);

    communication_.read_write_registers();
}

void ProtocolLogic::activate(){
    reset();
    set();
}

void ProtocolLogic::go_to(uint8_t position, uint8_t velocity, uint8_t force){
    communication_.write_action_bit(kGoToPositionByte, (bool)GoTo::GO_TO_REQ_POS);

    communication_.set_output_byte(OutputBytes::POSITION_REQUEST, position);
    communication_.set_output_byte(OutputBytes::SPEED, velocity);
    communication_.set_output_byte(OutputBytes::FORCE, force);

    communication_.read_write_registers();
}

void ProtocolLogic::stop(){
    communication_.write_action_bit(kGoToPositionByte, (bool)GoTo::STOP);

    communication_.read_write_registers();
}

bool ProtocolLogic::is_reset(){
    return (gripper_status_ == GripperStatus::GRIPPER_IN_RESET &&
            activation_status_ == ActivationStatus::GRIPPER_RESET);
}

bool ProtocolLogic::is_ready(){
    return (gripper_status_ == GripperStatus::ACTIVATION_COMPLETE &&
            activation_status_ == ActivationStatus::GRIPPER_ACTIVATION);
}

bool ProtocolLogic::is_moving(){
    return (action_status_ == ActionStatus::GO_TO_POSITION_REQUEST &&
            object_detection_status_ == ObjectDetectionStatus::MOTION_NO_OBJECT);
}

bool ProtocolLogic::is_stopped(){
    return object_detection_status_ != ObjectDetectionStatus::MOTION_NO_OBJECT;
}

bool ProtocolLogic::is_closed(){
    return position_ >= kGripperPositionOpenedThreshold;
}

bool ProtocolLogic::is_opened(){
    return position_ <= kGripperPositionClosedThreshold;
}

bool ProtocolLogic::obj_detected(){
    return (object_detection_status_ == ObjectDetectionStatus::STOPPED_OPENING_DETECTED ||
            object_detection_status_ == ObjectDetectionStatus::STOPPED_CLOSING_DETECTED);
}

uint8_t ProtocolLogic::get_reg_pos(){
    return position_request_echo_;
}

uint8_t ProtocolLogic::get_pos(){
    return position_;
}

uint8_t ProtocolLogic::get_current(){
    return current_;
}

void ProtocolLogic::refresh_registers(){
    communication_.read_write_registers();

    status_ = communication_.get_input_byte(InputBytes::GRIPPER_STATUS);

    activation_status_ = (ActivationStatus)(
        (status_>>kActivationStatusPositionByte) & kActivationStatusBits);

    action_status_ = (ActionStatus)(
        (status_>>kActionStatusPositionByte) & kActionStatusBits);

    gripper_status_ = (GripperStatus)(
        (status_>>kGripperStatusPositionByte) & kGripperStatusBits);

    object_detection_status_ = (ObjectDetectionStatus)(
        (status_>>kObjectDetectionStatusPositionByte) & kObjectDetectionStatusBits);

    fault_status_ = communication_.get_input_byte(InputBytes::FAULT_STATUS);
    //To bo specified

    position_request_echo_ = communication_.get_input_byte(InputBytes::POSITION_REQUEST_ECHO);
    position_ = communication_.get_input_byte(InputBytes::POSITION);
    current_ = communication_.get_input_byte(InputBytes::CURRENT);
}
}   // namespace hande_driver
