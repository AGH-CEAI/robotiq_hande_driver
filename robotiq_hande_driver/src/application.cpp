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

void ApplicationLayer::Stop(){
    protocol_logic_.Stop();
}

void ApplicationLayer::Reset(){
    protocol_logic_.Reset();
}


void ApplicationLayer::AutoRelease(){
    protocol_logic_.AutoRelease();
}


void ApplicationLayer::Activate(){
    protocol_logic_.Activate();
}


void ApplicationLayer::Open(){
    protocol_logic_.GoTo(0, kMaxSpeed, kMaxForce);
}


void ApplicationLayer::Close(){
    protocol_logic_.GoTo(255, kMaxSpeed, kMaxForce);
}

ApplicationLayer::Status ApplicationLayer::GetStatus(){
    return status_;
}

ApplicationLayer::FaultStatus ApplicationLayer::GetFaultStatus(){
    return fault_status_;
}

double ApplicationLayer::RequestedPosition(){
    return  requested_position_;
}

double ApplicationLayer::Position(){
    return position_;
}

void ApplicationLayer::SetPosition(double position){
    uint8_t raw_position = uint8_t((kGripperPositionMax - position) / kGripperPositionStep);

    protocol_logic_.GoTo(raw_position, kMaxSpeed, kMaxForce);
}

double ApplicationLayer::Current(){
    return current_;
}

void ApplicationLayer::Read(){
    protocol_logic_.RefreshRegisters();

    status_.is_reset = protocol_logic_.IsReset();
    status_.is_ready = protocol_logic_.IsReady();
    status_.is_moving = protocol_logic_.IsMoving();
    status_.is_stopped = protocol_logic_.IsStopped();
    status_.is_opened = protocol_logic_.IsOpened();
    status_.is_closed = protocol_logic_.IsClosed();
    status_.object_detected = protocol_logic_.ObjDetected();

    //fault_status

    requested_position_ = (double)protocol_logic_.GetRegPos() * kGripperPositionStep + kGripperPositionMin;
    position_ = kGripperPositionMax - (double)protocol_logic_.GetPos() * kGripperPositionStep;
    current_ = (double)protocol_logic_.GetCurrent() * kGripperCurrentScale;
}

void ApplicationLayer::Write(){
    protocol_logic_.RefreshRegisters();
}
}   // namespace hande_driver
