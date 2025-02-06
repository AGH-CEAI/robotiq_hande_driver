#include "protocol_logic.hpp"

#include <cstdio>


namespace hande_driver  {

constexpr auto kGripperPositionOpenedThreshold = 230;
constexpr auto kGripperPositionClosedThreshold = 13;

ProtocolLogic::ProtocolLogic()
:   status_(0)
,   activation_status_(GRIPPER_RESET)
,   action_status_(STOPPED)
,   gripper_status_(NOT_USED)
,   object_detection_status_(REQ_POS_NO_OBJECT)
,   fault_status_(0)
,   position_request_echo_(0)
,   position_(0)
,   current_(0)
{
    printf("ProtocolLogic constructor\n");
}

ProtocolLogic::~ProtocolLogic(){
    printf("ProtocolLogic destructor\n");
}

void ProtocolLogic::reset(){
    communication_.clear_output_bytes();

    communication_.write_action_bit(kActivatePositionByte, DEACTIVATE_GRIPPER);

    communication_.read_write_registers();
}

void ProtocolLogic::set(){
    communication_.clear_output_bytes();

    communication_.write_action_bit(kActivatePositionByte, ACTIVATE_GRIPPER);

    communication_.read_write_registers();
}

void ProtocolLogic::auto_release(){
    communication_.write_action_bit(kAutomaticReleasePositionByte, EMERGENCY_AUTO_RELEASE);
    communication_.write_action_bit(kAutoReleaseDirectionPositionByte, OPENING);

    communication_.read_write_registers();
}

void ProtocolLogic::activate(){
    reset();
    set();
}

void ProtocolLogic::go_to(uint8_t position, uint8_t velocity, uint8_t force){
    communication_.write_action_bit(kGoToPositionByte, GO_TO_REQ_POS);

    communication_.set_output_byte(OUTPUT_BYTES_POSITION_REQUEST, position);
    communication_.set_output_byte(OUTPUT_BYTES_SPEED, velocity);
    communication_.set_output_byte(OUTPUT_BYTES_FORCE, force);

    communication_.read_write_registers();
}

void ProtocolLogic::stop(){
    communication_.write_action_bit(kGoToPositionByte, STOP);

    communication_.read_write_registers();
}

bool ProtocolLogic::is_reset(){
    return (gripper_status_ == GRIPPER_IN_RESET &&
            activation_status_ == GRIPPER_RESET);
}

bool ProtocolLogic::is_ready(){
    return (gripper_status_ == ACTIVATION_COMPLETE &&
            activation_status_ == GRIPPER_ACTIVATION);
}

bool ProtocolLogic::is_moving(){
    return (action_status_ == GO_TO_POSITION_REQUEST &&
            object_detection_status_ == MOTION_NO_OBJECT);
}

bool ProtocolLogic::is_stopped(){
    return object_detection_status_ != MOTION_NO_OBJECT;
}

bool ProtocolLogic::is_closed(){
    return position_ >= kGripperPositionOpenedThreshold;
}

bool ProtocolLogic::is_opened(){
    return position_ <= kGripperPositionClosedThreshold;
}

bool ProtocolLogic::obj_detected(){
    return (object_detection_status_ == STOPPED_OPENING_DETECTED ||
            object_detection_status_ == STOPPED_CLOSING_DETECTED);
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

    status_ = communication_.get_input_byte(INPUT_BYTES_GRIPPER_STATUS);

    activation_status_ = (ActivationStatus)((status_>>kActivationStatusPositionByte) & 1u);
    action_status_ = (ActionStatus)((status_>>kActionStatusPositionByte) & 1u);
    gripper_status_ = (GripperStatus)((status_>>kGripperStatusPositionByte) & 3u);
    object_detection_status_ = (ObjectDetectionStatus)((status_>>kObjectDetectionStatusPositionByte) & 3u);

    fault_status_ = communication_.get_input_byte(INPUT_BYTES_FAULT_STATUS);
    //To bo specified

    position_request_echo_ = communication_.get_input_byte(INPUT_BYTES_POSITION_REQUEST_ECHO);
    position_ = communication_.get_input_byte(INPUT_BYTES_POSITION);
    current_ = communication_.get_input_byte(INPUT_BYTES_CURRENT);
}
}   // namespace hande_driver
