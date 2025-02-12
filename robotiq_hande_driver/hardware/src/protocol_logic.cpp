#include "protocol_logic.hpp"

#include <cstdio>


namespace hande_driver  {

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

}

void ProtocolLogic::refresh_registers(){
    communication_.read_write_registers();

    status_ = communication_.get_input_byte(InputBytes::GRIPPER_STATUS);

    activation_status_ = (ActivationStatus)(
        (status_>>static_cast<uint>(StatusPositionBit::ACTIVATION_STATUS)) & ACTIVATION_STATUS_BITS);

    action_status_ = (ActionStatus)(
        (status_>>static_cast<uint>(StatusPositionBit::ACTION_STATUS)) & ACTION_STATUS_BITS);

    gripper_status_ = (GripperStatus)(
        (status_>>static_cast<uint>(StatusPositionBit::GRIPPER_STATUS)) & GRIPPER_STATUS_BITS);

    object_detection_status_ = (ObjectDetectionStatus)(
        (status_>>static_cast<uint>(StatusPositionBit::OBJECT_DETECTION_STATUS)) & OBJECT_DETECTION_STATUS_BITS);

    fault_status_ = communication_.get_input_byte(InputBytes::FAULT_STATUS);
    //To bo specified

    position_request_echo_ = communication_.get_input_byte(InputBytes::POSITION_REQUEST_ECHO);
    position_ = communication_.get_input_byte(InputBytes::POSITION);
    current_ = communication_.get_input_byte(InputBytes::CURRENT);
}
}   // namespace hande_driver
