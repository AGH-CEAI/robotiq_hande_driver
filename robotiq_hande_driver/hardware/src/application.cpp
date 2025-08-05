#include "robotiq_hande_driver/application.hpp"

#include <cstdio>

namespace robotiq_hande_driver {

GripperApplication::GripperApplication()
    : protocol_logic_{},
      status_{},
      fault_status_{},
      requested_position_{},
      position_{},
      current_{},
      gripper_position_min_{},
      gripper_position_max_{},
      gripper_postion_step_{} {}

void GripperApplication::initialize(
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

int GripperApplication::configure() {
    return protocol_logic_.configure();
}

void GripperApplication::cleanup() {
    protocol_logic_.cleanup();
}

void GripperApplication::stop() {
    protocol_logic_.stop();
}

void GripperApplication::reset() {
    protocol_logic_.reset();
}

void GripperApplication::auto_release() {
    protocol_logic_.auto_release();
}

void GripperApplication::activate() {
    protocol_logic_.activate();
}

void GripperApplication::deactivate() {
    protocol_logic_.reset();
}

void GripperApplication::shutdown() {
    deactivate();
    cleanup();
}

void GripperApplication::open() {
    set_position(gripper_position_max_);
}

void GripperApplication::close() {
    set_position(gripper_position_min_);
}

GripperApplication::Status GripperApplication::get_status() const {
    return status_;
}

GripperApplication::FaultStatus GripperApplication::get_fault_status() const {
    return fault_status_;
}

double GripperApplication::get_requested_position() const {
    return requested_position_;
}

double GripperApplication::get_position() const {
    return position_;
}

void GripperApplication::set_position(double position, double force) {
    uint8_t scaled_force = static_cast<uint8_t>(force * MAX_FORCE);
    protocol_logic_.go_to(
        (uint8_t)((gripper_position_max_ - position) / gripper_postion_step_),
        MAX_SPEED,
        scaled_force);
}

double GripperApplication::get_current() const {
    return current_;
}

void GripperApplication::read() {
    protocol_logic_.refresh_registers();

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

void GripperApplication::write() {
    // TODO investigate: we shouldn't read statue during the write phase
    protocol_logic_.refresh_registers();

    // Here we should take internal registers and send them to the gripper
    //  I.e. call set_position()
}

}  // namespace robotiq_hande_driver
