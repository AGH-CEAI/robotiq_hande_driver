#include <cstdio>
#include "protocol_logic.hpp"


namespace hande_driver
{

ProtocolLogic::ProtocolLogic()
:   status_(0)
,   activation_status_(GRIPPER_RESET)
,   action_status_(STOPPED)
,   gripper_status_(NOT_USED)
,   object_detection_status_(REQ_POS_NO_OBJECT)
,   fault_status(0)
,   position_request_echo(0)
,   position(0)
,   current(0)
{
    printf("ProtocolLogic constructor\n");
}

ProtocolLogic::~ProtocolLogic(){
    printf("ProtocolLogic destructor\n");
}

void ProtocolLogic::reset(){
    communication_.output_registers[0] = 0x0000;
    communication_.output_registers[1] = 0x0000;
    communication_.output_registers[2] = 0x0000;

    communication_.output_registers[actionRequestByte / 2] = bit_set_to(
        communication_.output_registers[actionRequestByte / 2],
        (actionRequestByte % 2 == 0 ? 8 : 0) + activatePositionByte,
        DEACTIVATE_GRIPPER);

    communication_.read_write_registers();
}

void ProtocolLogic::set(){
    communication_.output_registers[0] = 0x0000;
    communication_.output_registers[1] = 0x0000;
    communication_.output_registers[2] = 0x0000;

    communication_.output_registers[actionRequestByte / 2] = bit_set_to(
        communication_.output_registers[actionRequestByte / 2],
        (actionRequestByte % 2 == 0 ? 8 : 0) + activatePositionByte,
        ACTIVATE_GRIPPER);

    communication_.read_write_registers();
}

void ProtocolLogic::auto_release(){
    communication_.output_registers[actionRequestByte / 2] = bit_set_to(
        communication_.output_registers[actionRequestByte / 2],
        (actionRequestByte % 2 == 0 ? 8 : 0) + automaticReleasePositionByte,
        EMERGENCY_AUTO_RELEASE);

    communication_.output_registers[actionRequestByte / 2] = bit_set_to(
        communication_.output_registers[actionRequestByte / 2],
        (actionRequestByte % 2 == 0 ? 8 : 0) + automaticReleasePositionByte,
        OPENING);

    communication_.read_write_registers();
}

void ProtocolLogic::activate(){
    reset();
    set();
}

void ProtocolLogic::go_to(uint8_t position, uint8_t velocity, uint8_t force){
    communication_.output_registers[goToPositionByte / 2] = bit_set_to(
        communication_.output_registers[goToPositionByte / 2],
        (goToPositionByte % 2 == 0 ? 8 : 0) + goToPositionByte,
        GO_TO_REQ_POS);

    communication_.output_registers[1] = uint16_t(0x00 << 8 | position);
    communication_.output_registers[2] = uint16_t(velocity << 8 | force);

    communication_.read_write_registers();
    
}

void ProtocolLogic::stop(){
    communication_.output_registers[goToPositionByte / 2] = bit_set_to(
        communication_.output_registers[goToPositionByte / 2],
        (goToPositionByte % 2 == 0 ? 8 : 0) + goToPositionByte,
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
    return position <= 13;
}

bool ProtocolLogic::is_opened(){
    return position >= 230;
}

bool ProtocolLogic::obj_detected(){
    return (object_detection_status_ == STOPPED_OPENING_DETECTED ||
            object_detection_status_ == STOPPED_CLOSING_DETECTED);
}

uint8_t ProtocolLogic::get_reg_pos(){
    return position_request_echo;
}

uint8_t ProtocolLogic::get_pos(){
    return position;
}

uint8_t ProtocolLogic::get_current(){
    return current;
}

void ProtocolLogic::refresh_registers(){
    communication_.read_write_registers();

    read_register(status_, statusByte);

    activation_status_ = (ActivationStatus)((status_>>activationStatusPositionByte) & 1u);
    action_status_ = (ActionStatus)((status_>>actionStatusPositionByte) & 1u);
    gripper_status_ = (GripperStatus)((status_>>gripperStatusPositionByte) & 3u);
    object_detection_status_ = (ObjectDetectionStatus)((status_>>objectDetectionStatusPositionByte) & 3u);

    read_register(fault_status, faultStatusByte);
    //To bo specified
    
    read_register(position_request_echo, positionRequestEchoByte);
    read_register(position, positionByte);
    read_register(current, currentByte);
}

void ProtocolLogic::read_register(uint8_t &reg, uint8_t byte){
    reg = byte % 2 == 1 ? 
        communication_.input_registers[byte / 2] & 255u :
        communication_.input_registers[byte / 2] >> 8u;
}

inline uint ProtocolLogic::bit_set_to(uint number, uint n, bool x) {
    return (number & ~((uint)1 << n)) | ((uint)x << n);
}
}   // namespace hande_driver
