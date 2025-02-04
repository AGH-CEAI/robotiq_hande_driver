#include "application.hpp"

#include <cstdio>


namespace hande_driver  {

constexpr auto kGripperPositionMin = 0.0;
constexpr auto kGripperPositionMax = 0.05;
constexpr auto kGripperPositionStep = (kGripperPositionMax - kGripperPositionMin) / 255.0;
constexpr auto kGripperCurrentScale = 0.01;
constexpr auto kMaxSpeed = 255;
constexpr auto kMaxForce = 255;

ApplicationLayer::ApplicationLayer()
:   requested_position_(0)
,   position_(0)
,   current_(0)
{
    printf("ApplicationLayer constructor\n");
}

ApplicationLayer::~ApplicationLayer(){
    printf("ApplicationLayer destructor\n");
}

void ApplicationLayer::stop(){
    protocol_logic_.stop();
}

void ApplicationLayer::reset(){
    protocol_logic_.reset();
}


void ApplicationLayer::auto_release(){
    protocol_logic_.auto_release();
}


void ApplicationLayer::activate(){
    protocol_logic_.activate();
}


void ApplicationLayer::open(){
    protocol_logic_.go_to(0, kMaxSpeed, kMaxForce);
}


void ApplicationLayer::close(){
    protocol_logic_.go_to(255, kMaxSpeed, kMaxForce);
}

ApplicationLayer::Status ApplicationLayer::get_status(){
    return status_;
}

ApplicationLayer::FaultStatus ApplicationLayer::get_fault_status(){
    return fault_status_;
}

double ApplicationLayer::requested_position(){
    return  requested_position_;
}

double ApplicationLayer::position(){
    return position_;
}

void ApplicationLayer::set_position(double position){
    uint8_t raw_position = uint8_t((kGripperPositionMax - position) / kGripperPositionStep);

    protocol_logic_.go_to(raw_position, kMaxSpeed, kMaxForce);
}

double ApplicationLayer::current(){
    return current_;
}

void ApplicationLayer::read(){
    protocol_logic_.refresh_registers();

    status_.is_reset = protocol_logic_.is_reset();
    status_.is_ready = protocol_logic_.is_ready();
    status_.is_moving = protocol_logic_.is_moving();
    status_.is_stopped = protocol_logic_.is_stopped();
    status_.is_opened = protocol_logic_.is_opened();
    status_.is_closed = protocol_logic_.is_closed();
    status_.object_detected = protocol_logic_.obj_detected();

    //fault_status

    requested_position_ = (double)protocol_logic_.get_reg_pos() * kGripperPositionStep + kGripperPositionMin;
    position_ = kGripperPositionMax - (double)protocol_logic_.get_pos() * kGripperPositionStep;
    current_ = (double)protocol_logic_.get_current() * kGripperCurrentScale;
}

void ApplicationLayer::write(){
    protocol_logic_.refresh_registers();
}
}   // namespace hande_driver
