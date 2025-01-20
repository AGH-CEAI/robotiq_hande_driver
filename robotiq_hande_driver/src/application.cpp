#include "application.hpp"


namespace hande_driver
{
ApplicationLayer::ApplicationLayer(){}

ApplicationLayer::~ApplicationLayer(){}

void ApplicationLayer::stop(){
    protocol_logic.stop();
}

void ApplicationLayer::reset(){
    protocol_logic.reset();
}


void ApplicationLayer::auto_release(){
    protocol_logic.auto_release();
}


void ApplicationLayer::activate(){
    protocol_logic.activate();
}


void ApplicationLayer::open(){
    protocol_logic.go_to(255, 255, 255);
}


void ApplicationLayer::close(){
    protocol_logic.go_to(0, 255, 255);
}

Status ApplicationLayer::status(){
    return status_;
}

FaultStatus ApplicationLayer::fault_status(){
    return fault_status_;
}

uint8_t ApplicationLayer::requested_position(){
    return  requested_position_;
}

uint8_t ApplicationLayer::position(){
    return position_;
}

void ApplicationLayer::set_position(uint8_t position){
    protocol_logic.go_to(position, 255, 255);
}

uint16_t ApplicationLayer::current(){
    return current_;
}

void ApplicationLayer::update(){
    status_.isReset = protocol_logic_.is_reset();
    status_.isReady = protocol_logic_.is_ready();
    status_.isMoving = protocol_logic_.is_moving();
    status_.isStopped = protocol_logic_.is_stopped();
    status_.isOpened = protocol_logic_.is_opened();
    status_.isClosed = protocol_logic_.is_closed();
    status_.objectDetected = protocol_logic_.obj_detected();
    
    //fault_status

    requested_position_ = protocol_logic_.get_reg_pos();//TODO: add scaling
    position_ = protocol_logic.get_pos();   //TODO: add scaling
    current_ = (uint16_t)protocol_logic.get_current() * 10;
}

}   // namespace hande_driver