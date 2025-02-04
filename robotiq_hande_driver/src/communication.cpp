#include "communication.hpp"

#include <cstdio>
#include <stdint.h>


namespace hande_driver  {

constexpr auto kDeviceName = "/tmp/ttyUR";
constexpr auto kBaudrate = 115200;
constexpr auto kParity = 'N';
constexpr auto kDataBits = 8;
constexpr auto kStopBit = 1;
constexpr auto kDebugModbus = true;
constexpr uint8_t kSlaveID = 0x09;

constexpr uint16_t kGripperOutputFirstReg = 0x07D0;
constexpr uint16_t kGripperInputFirstReg = 0x03E8;

Communication::Communication()
:    input_registers_{0, 0, 0}
,    output_registers_{0, 0, 0}
{
    printf("Communication constructor\n");
    mb_ = modbus_new_rtu(kDeviceName, kBaudrate, kParity, kDataBits, kStopBit);
    modbus_set_slave(mb_, kSlaveID);
    modbus_set_debug(mb_, kDebugModbus);
    connect();
}

Communication::~Communication(){
    printf("Communication destructor\n");
    disconnect();
    modbus_free(mb_);
}

void Communication::connect(){
    modbus_connect(mb_);

    //TODO: send read/write and verify gripper is responding
}

void Communication::disconnect(){
    modbus_close(mb_);
}

void Communication::read_write_registers(){
    // TODO: add description
    modbus_write_and_read_registers(mb_,
                                    kGripperInputFirstReg,
                                    kRegisterWordLength,
                                    output_registers_,
                                    kGripperOutputFirstReg,
                                    kRegisterWordLength,
                                    input_registers_);
}
}   // namespace hande_driver
