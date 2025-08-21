#include "robotiq_hande_driver/hande_gripper.hpp"

#include <cmath>

namespace robotiq_hande_driver {

HandeGripper::HandeGripper()
    : prot_{},
      status_{},
      fault_status_{},
      requested_position_{},
      position_{},
      current_{},
      gripper_position_min_{},
      gripper_position_max_{},
      gripper_postion_step_{} {}

void HandeGripper::initialize(
    double gripper_position_min, double gripper_position_max, const CommunicationConfig& cfg) {
    gripper_position_min_ = gripper_position_min;
    gripper_position_max_ = gripper_position_max;
    gripper_postion_step_ = (gripper_position_max_ - gripper_position_min_) / 255.0;
    prot_.initialize(cfg);
}

void HandeGripper::configure() {
    prot_.configure();
}

void HandeGripper::cleanup() {
    prot_.cleanup();
}

void HandeGripper::stop() {
    prot_.stop();
}

void HandeGripper::reset() {
    prot_.reset();
}

void HandeGripper::auto_release() {
    prot_.auto_release();
}

void HandeGripper::activate() {
    prot_.activate();
}

void HandeGripper::deactivate() {
    prot_.reset();
}

void HandeGripper::shutdown() {
    deactivate();
    cleanup();
}

void HandeGripper::open() {
    set_position(gripper_position_max_);
}

void HandeGripper::close() {
    set_position(gripper_position_min_);
}

HandeGripper::Status HandeGripper::get_status() const {
    return status_;
}

HandeGripper::FaultStatus HandeGripper::get_fault_status() const {
    return fault_status_;
}

double HandeGripper::get_requested_position() const {
    return requested_position_;
}

double HandeGripper::get_position() const {
    return position_;
}

void HandeGripper::set_position(double position, double force) {
    static double prev_position = std::numeric_limits<double>::quiet_NaN();
    static double prev_force = std::numeric_limits<double>::quiet_NaN();

    if(!std::isnan(prev_position) && std::fabs(position - prev_position) < EPSILON) return;
    if(!std::isnan(prev_force) && std::fabs(force - prev_force) < EPSILON) return;

    uint8_t scaled_force = static_cast<uint8_t>(force * MAX_FORCE);
    prot_.go_to(
        (uint8_t)((gripper_position_max_ - position) / gripper_postion_step_),
        MAX_SPEED,
        scaled_force);
}

double HandeGripper::get_current() const {
    return current_;
}

void HandeGripper::read() {
    prot_.read_input_bytes();

    status_.is_reset = prot_.is_reset();
    status_.is_ready = prot_.is_ready();
    status_.is_moving = prot_.is_moving();
    status_.is_stopped = prot_.is_stopped();
    status_.is_opened = prot_.is_opened();
    status_.is_closed = prot_.is_closed();
    status_.object_detected = prot_.obj_detected();

    // fault_status
    requested_position_ = gripper_position_max_
                          - (double)prot_.get_raw_requested_pos() * gripper_postion_step_;
    position_ = gripper_position_max_ - (double)prot_.get_raw_pos() * gripper_postion_step_;
    current_ = (double)prot_.get_raw_current() * GRIPPER_CURRENT_SCALE;
}

void HandeGripper::write() {
    prot_.write_output_bytes();
}

}  // namespace robotiq_hande_driver
