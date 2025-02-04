#include "protocol_logic.hpp"

#include <cstdio>


namespace hande_driver  {

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
    communication_.output_registers_[0] = 0x0000;
    communication_.output_registers_[1] = 0x0000;
    communication_.output_registers_[2] = 0x0000;

    communication_.output_registers_[kActionRequestByte / 2] = bit_set_to(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kActivatePositionByte,
        DEACTIVATE_GRIPPER);

    communication_.read_write_registers();
}

void ProtocolLogic::set(){
    communication_.output_registers_[0] = 0x0000;
    communication_.output_registers_[1] = 0x0000;
    communication_.output_registers_[2] = 0x0000;

    communication_.output_registers_[kActionRequestByte / 2] = bit_set_to(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kActivatePositionByte,
        ACTIVATE_GRIPPER);

    communication_.read_write_registers();
}

void ProtocolLogic::auto_release(){
    communication_.output_registers_[kActionRequestByte / 2] = bit_set_to(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kAutomaticReleasePositionByte,
        EMERGENCY_AUTO_RELEASE);

    communication_.output_registers_[kActionRequestByte / 2] = bit_set_to(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kAutomaticReleasePositionByte,
        OPENING);

    communication_.read_write_registers();
}

void ProtocolLogic::activate(){
    reset();
    set();
}

void ProtocolLogic::go_to(uint8_t position, uint8_t velocity, uint8_t force){
    communication_.output_registers_[kActionRequestByte / 2] = bit_set_to(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kGoToPositionByte,
        GO_TO_REQ_POS);

    communication_.output_registers_[1] = uint16_t(0x00 << 8 | position);
    communication_.output_registers_[2] = uint16_t(velocity << 8 | force);

    communication_.read_write_registers();
}

void ProtocolLogic::stop(){
    communication_.output_registers_[kActionRequestByte / 2] = bit_set_to(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kGoToPositionByte,
        STOP);

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
    return position_ <= 13;
}

bool ProtocolLogic::is_opened(){
    return position_ >= 230;
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

    read_register(status_, kStatusByte);

    activation_status_ = (ActivationStatus)((status_>>kActivationStatusPositionByte) & 1u);
    action_status_ = (ActionStatus)((status_>>kActionStatusPositionByte) & 1u);
    gripper_status_ = (GripperStatus)((status_>>kGripperStatusPositionByte) & 3u);
    object_detection_status_ = (ObjectDetectionStatus)((status_>>kObjectDetectionStatusPositionByte) & 3u);

    read_register(fault_status_, kFaultStatusByte);
    //To bo specified

    read_register(position_request_echo_, kPositionRequestEchoByte);
    read_register(position_, kPositionByte);
    read_register(current_, kCurrentByte);
}

void ProtocolLogic::read_register(uint8_t &reg, uint8_t byte){
    // TODO: add byte references in constructor
    reg = byte % 2 == 1 ?
        communication_.input_registers_[byte / 2] & 255u :
        communication_.input_registers_[byte / 2] >> 8u;
}

inline uint ProtocolLogic::bit_set_to(uint number, uint n, bool x) {
    //TODO: include byte/bit logic
    return (number & ~((uint)1 << n)) | ((uint)x << n);
}
}   // namespace hande_driver
