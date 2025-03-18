#include "robotiq_hande_driver/communication.hpp"

#include <cstdio>

namespace robotiq_hande_driver {

Communication::Communication() : input_bytes_{}, output_bytes_{} {}

int Communication::connect() {
    int result;

    modbus_connect(mb_);

    result = read_write_registers();

    return result;
}
}  // namespace robotiq_hande_driver
