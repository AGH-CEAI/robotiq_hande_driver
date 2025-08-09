#include "robotiq_hande_driver/hande_gripper.hpp"

#include <cmath>
#include <cstdio>

namespace robotiq_hande_driver {

HandeGripper::HandeGripper()
    : protocol_logic_{},
      status_{},
      fault_status_{},
      requested_position_{},
      position_{},
      current_{},
      gripper_position_min_{},
      gripper_position_max_{},
      gripper_postion_step_{} {}

void HandeGripper::initialize(
    double gripper_position_min,
    double gripper_position_max,
    const std::string& tty_port,
    int baudrate,
    char parity,
    int data_bits,
    int stop_bit,
    int slave_id) {
    gripper_position_min_ = gripper_position_min;
    gripper_position_max_ = gripper_position_max;
    gripper_postion_step_ = (gripper_position_max_ - gripper_position_min_) / 255.0;
    protocol_logic_.initialize(tty_port, baudrate, parity, data_bits, stop_bit, slave_id);
}

int HandeGripper::configure() {
    return protocol_logic_.configure();
}

void HandeGripper::cleanup() {
    protocol_logic_.cleanup();
}

void HandeGripper::stop() {
    protocol_logic_.stop();
}

void HandeGripper::reset() {
    protocol_logic_.reset();
}

void HandeGripper::auto_release() {
    protocol_logic_.auto_release();
}

void HandeGripper::activate() {
    protocol_logic_.activate();
}

void HandeGripper::deactivate() {
    protocol_logic_.reset();
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
    protocol_logic_.go_to(
        (uint8_t)((gripper_position_max_ - position) / gripper_postion_step_),
        MAX_SPEED,
        scaled_force);
}

double HandeGripper::get_current() const {
    return current_;
}

void HandeGripper::read() {
    protocol_logic_.read_input_bytes();

    // TODO consider extracting status to one level below (protocol_logic)
    status_.is_reset = protocol_logic_.is_reset();
    status_.is_ready = protocol_logic_.is_ready();
    status_.is_moving = protocol_logic_.is_moving();
    status_.is_stopped = protocol_logic_.is_stopped();
    status_.is_opened = protocol_logic_.is_opened();
    status_.is_closed = protocol_logic_.is_closed();
    status_.object_detected = protocol_logic_.obj_detected();

    // fault_status
    requested_position_ = gripper_position_max_
                          - (double)protocol_logic_.get_reg_pos() * gripper_postion_step_;
    position_ = gripper_position_max_ - (double)protocol_logic_.get_pos() * gripper_postion_step_;
    current_ = (double)protocol_logic_.get_current() * GRIPPER_CURRENT_SCALE;
}

void HandeGripper::write() {
    // Do nothing?
}

}  // namespace robotiq_hande_driver
