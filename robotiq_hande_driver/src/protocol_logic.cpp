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

void ProtocolLogic::Reset(){
    communication_.output_registers_[0] = 0x0000;
    communication_.output_registers_[1] = 0x0000;
    communication_.output_registers_[2] = 0x0000;

    communication_.output_registers_[kActionRequestByte / 2] = BitSetTo(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kActivatePositionByte,
        DEACTIVATE_GRIPPER);

    communication_.ReadWiteRegisters();
}

void ProtocolLogic::Set(){
    communication_.output_registers_[0] = 0x0000;
    communication_.output_registers_[1] = 0x0000;
    communication_.output_registers_[2] = 0x0000;

    communication_.output_registers_[kActionRequestByte / 2] = BitSetTo(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kActivatePositionByte,
        ACTIVATE_GRIPPER);

    communication_.ReadWiteRegisters();
}

void ProtocolLogic::AutoRelease(){
    communication_.output_registers_[kActionRequestByte / 2] = BitSetTo(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kAutomaticReleasePositionByte,
        EMERGENCY_AUTO_RELEASE);

    communication_.output_registers_[kActionRequestByte / 2] = BitSetTo(
        communication_.output_registers_[kActionRequestByte / 2],
        (kActionRequestByte % 2 == 0 ? 8 : 0) + kAutomaticReleasePositionByte,
        OPENING);

    communication_.ReadWiteRegisters();
}

void ProtocolLogic::Activate(){
    Reset();
    Set();
}

void ProtocolLogic::GoTo(uint8_t position, uint8_t velocity, uint8_t force){
    communication_.output_registers_[kGoToPositionByte / 2] = BitSetTo(
        communication_.output_registers_[kGoToPositionByte / 2],
        (kGoToPositionByte % 2 == 0 ? 8 : 0) + kGoToPositionByte,
        GO_TO_REQ_POS);

    communication_.output_registers_[1] = uint16_t(0x00 << 8 | position);
    communication_.output_registers_[2] = uint16_t(velocity << 8 | force);

    communication_.ReadWiteRegisters();
}

void ProtocolLogic::Stop(){
    communication_.output_registers_[kGoToPositionByte / 2] = BitSetTo(
        communication_.output_registers_[kGoToPositionByte / 2],
        (kGoToPositionByte % 2 == 0 ? 8 : 0) + kGoToPositionByte,
        STOP);

    communication_.ReadWiteRegisters();
}

bool ProtocolLogic::IsReset(){
    return (gripper_status_ == GRIPPER_IN_RESET &&
            activation_status_ == GRIPPER_RESET);
}

bool ProtocolLogic::IsReady(){
    return (gripper_status_ == ACTIVATION_COMPLETE &&
            activation_status_ == GRIPPER_ACTIVATION);
}

bool ProtocolLogic::IsMoving(){
    return (action_status_ == GO_TO_POSITION_REQUEST &&
            object_detection_status_ == MOTION_NO_OBJECT);
}

bool ProtocolLogic::IsStopped(){
    return object_detection_status_ != MOTION_NO_OBJECT;
}

bool ProtocolLogic::IsClosed(){
    return position_ <= 13;
}

bool ProtocolLogic::IsOpened(){
    return position_ >= 230;
}

bool ProtocolLogic::ObjDetected(){
    return (object_detection_status_ == STOPPED_OPENING_DETECTED ||
            object_detection_status_ == STOPPED_CLOSING_DETECTED);
}

uint8_t ProtocolLogic::GetRegPos(){
    return position_request_echo_;
}

uint8_t ProtocolLogic::GetPos(){
    return position_;
}

uint8_t ProtocolLogic::GetCurrent(){
    return current_;
}

void ProtocolLogic::RefreshRegisters(){
    communication_.ReadWiteRegisters();

    ReadRegister(status_, kStatusByte);

    activation_status_ = (ActivationStatus)((status_>>kActivationStatusPositionByte) & 1u);
    action_status_ = (ActionStatus)((status_>>kActionStatusPositionByte) & 1u);
    gripper_status_ = (GripperStatus)((status_>>kGripperStatusPositionByte) & 3u);
    object_detection_status_ = (ObjectDetectionStatus)((status_>>kObjectDetectionStatusPositionByte) & 3u);

    ReadRegister(fault_status_, kFaultStatusByte);
    //To bo specified

    ReadRegister(position_request_echo_, kPositionRequestEchoByte);
    ReadRegister(position_, kPositionByte);
    ReadRegister(current_, kCurrentByte);
}

void ProtocolLogic::ReadRegister(uint8_t &reg, uint8_t byte){
    reg = byte % 2 == 1 ?
        communication_.input_registers_[byte / 2] & 255u :
        communication_.input_registers_[byte / 2] >> 8u;
}

inline uint ProtocolLogic::BitSetTo(uint number, uint n, bool x) {
    return (number & ~((uint)1 << n)) | ((uint)x << n);
}
}   // namespace hande_driver
