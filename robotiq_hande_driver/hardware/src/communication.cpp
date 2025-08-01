#include "robotiq_hande_driver/communication.hpp"

#include <cstdio>

namespace robotiq_hande_driver {

Communication::Communication()
    : input_bytes_modbus_{}, output_bytes_modbus_{}, input_bytes_{}, output_bytes_{} {}

int Communication::connect() {
    // int result;

    modbus_connect(mb_);

    // read_write_registers();

    return 0;
}
}  // namespace robotiq_hande_driver
