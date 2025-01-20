#include "protocol_logic.hpp"


namespace hande_driver
{
ProtocolLogic::ProtocolLogic(){}

ProtocolLogic::~ProtocolLogic(){}

void ProtocolLogic::reset(){}

void ProtocolLogic::auto_release(){}

void ProtocolLogic::activate(){}

void ProtocolLogic::go_to(uint8_t position, uint8_t velocity, uint8_t force, bool arm_callback=true){
    std::vector<uint8_t>& data;
    size_t resp_data_len;

    //TODO: fill vector with appropriate data

    communication_.send_command(data, resp_data_len);
}

void ProtocolLogic::stop(){}

bool ProtocolLogic::is_reset(){
    return (gripper_status_ == GripperStatus.GRIPPER_IN_RESET && 
            activation_status_ == ActivationStatus.GRIPPER_RESET);
}

bool ProtocolLogic::is_ready(){
    return (gripper_status_ == GripperStatus.ACTIVATION_COMPLETE && 
            activation_status_ == ActivationStatus.GRIPPER_ACTIVATION);
}

bool ProtocolLogic::is_moving(){
    return (action_status_ == ActionStatus.GO_TO_POSITION_REQUEST &&
            object_detection_status_ == ObjectDetectionStatus.MOTION_NO_OBJECT);

            ActivationStatus activation_status_;
    ActionStatus action_status_;
    GripperStatus gripper_status_;
    ObjectDetectionStatus object_detection_status_;
}

bool ProtocolLogic::is_stopped(){
    return object_detection_status_ != ObjectDetectionStatus.MOTION_NO_OBJECT;
}

bool ProtocolLogic::is_closed(){
    return position <= 13;
}

bool ProtocolLogic::is_opened(){
    return position >= 230;
}

bool ProtocolLogic::obj_detected(){
    return (object_detection_status_ == ObjectDetectionStatus.STOPPED_OPENING_DETECTED ||
            object_detection_status_ == ObjectDetectionStatus.STOPPED_CLOSING_DETECTED);
}

uint8_t ProtocolLogic::get_reg_pos(){
    return position_request;
}

uint8_t ProtocolLogic::get_pos(){
    return position;
}

uint8_t ProtocolLogic::get_current(){
    return current;
}

void ProtocolLogic::wait_until_moving(){
    // Arm the callback
    // attach the callback to application
}

void ProtocolLogic::wait_until_stopped(){
    // Arm the callback
    // attach the callback to application
    
}


}   // namespace hande_driver