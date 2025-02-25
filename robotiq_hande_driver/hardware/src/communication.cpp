#include "robotiq_hande_driver/communication.hpp"

#include <cstdio>

namespace robotiq_hande_driver {

Communication::Communication() : input_bytes_{}, output_bytes_{} {}

int Communication::connect() {
    uint16_t activation_status[1] = {0x0000};
    int result;

    modbus_connect(mb_);

    result = modbus_read_registers(mb_, GRIPPER_OUTPUT_FIRST_REG, 1, activation_status);

    return result;
}
}  // namespace robotiq_hande_driver
