#include "communication.hpp"

#include <cstdio>


namespace hande_driver  {

Communication::Communication()
:    input_bytes_{}
,    output_bytes_{}
{

}

void Communication::connect(){
    uint16_t activation_status[1] = {0x0000};
    int result;

    modbus_connect(mb_);

    result = modbus_read_registers(mb_, GRIPPER_OUTPUT_FIRST_REG, 1, activation_status);

    if (result > 0)
        printf("Connected successfully: %d\n", result);
    else
        printf("Couldn't connect: %d\n", result);
}
}   // namespace hande_driver
