#include "robotiq_hande_driver/application.hpp"

#include <cstdio>

namespace robotiq_hande_driver {

GripperApplication::GripperApplication()
    : requested_position_(),
      position_(),
      current_(),
      gripper_position_min_(),
      gripper_position_max_(),
      gripper_postion_step_() {}

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
}  // namespace robotiq_hande_driver
