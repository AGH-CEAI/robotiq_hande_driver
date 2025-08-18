#include <modbus/modbus.h>
#include <stdio.h>
#include <unistd.h>

constexpr auto REGISTER_READ_LENGTH = 6;
constexpr auto DEVICE_NAME = "/tmp/ttyUR";
constexpr auto BAUDRATE = 115200;
constexpr auto PARITY = 'N';
constexpr auto DATA_BITS = 8;
constexpr auto STOP_BIT = 1;
constexpr auto DEBUG_MODBUS = true;

constexpr uint8_t SERVER_ID = 0x09;

constexpr uint16_t SERIAL_OUTPUT_FIRST_REG = 0x07D0;
constexpr uint16_t SERIAL_INPUT_FIRST_REG = 0x03E8;
constexpr uint8_t READ_WRITE_REG_LENGTH = 3;

constexpr uint16_t GRIPPER_FLAGS_DEACTIVATE = 0x0000;
constexpr uint16_t GRIPPER_FLAGS_ACTIVATE = 0x0100;
constexpr uint16_t GRIPPER_FLAGS_ACTIVATED = 0x3100;

constexpr uint16_t GRIPPER_FLAGS_MOVE = 0x0900;
constexpr uint16_t FULL_SPEED_FORCE = 0xFFFF;
constexpr uint16_t GRIPPER_OPENED = 0x0000;
constexpr uint16_t GRIPPER_CLOSED = 0x00FF;

constexpr uint16_t GRIPPER_FLAGS_CLOSED = 0xB900;
constexpr uint16_t GRIPPER_FLAGS_NO_OBJECT = 0xF900;

void sleep_100ms() {
    usleep(100 * 1000);  // ms * 1000
}

void test_connection(modbus_t* ctx) {
    uint16_t tab_reg[REGISTER_READ_LENGTH];

    // Read 5 registers from the address 0
    modbus_read_registers(ctx, SERIAL_OUTPUT_FIRST_REG, REGISTER_READ_LENGTH, tab_reg);

    printf(
        "Status: %.4X %.4X %.4X %.4X %.4X %.4X\n",
        tab_reg[0],
        tab_reg[1],
        tab_reg[2],
        tab_reg[3],
        tab_reg[4],
        tab_reg[5]);
}

void activate(modbus_t* ctx) {
    printf("Step: Activation Request (clear and set rACT)\n");
    uint16_t activation_request_register_reset[READ_WRITE_REG_LENGTH] = {
        GRIPPER_FLAGS_DEACTIVATE, 0x0000, 0x0000};
    modbus_write_registers(
        ctx, SERIAL_INPUT_FIRST_REG, READ_WRITE_REG_LENGTH, activation_request_register_reset);

    uint16_t activation_request_register_set[READ_WRITE_REG_LENGTH] = {
        GRIPPER_FLAGS_ACTIVATE, 0x0000, 0x0000};
    modbus_write_registers(
        ctx, SERIAL_INPUT_FIRST_REG, READ_WRITE_REG_LENGTH, activation_request_register_set);
}

void wait_activation_complete(modbus_t* ctx) {
    printf("Step: Read Gripper status until the activation is completed\n");
    uint16_t activation_status[READ_WRITE_REG_LENGTH] = {};

    modbus_read_registers(ctx, SERIAL_OUTPUT_FIRST_REG, READ_WRITE_REG_LENGTH, activation_status);

    while(activation_status[0] != GRIPPER_FLAGS_ACTIVATED) {
        printf(
            "Gripper not yet activated: %.4X vs. %.4X\n",
            activation_status[0],
            GRIPPER_FLAGS_ACTIVATED);
        modbus_read_registers(ctx, SERIAL_OUTPUT_FIRST_REG, 1, activation_status);
    }
}

void close_gripper(modbus_t* ctx) {
    printf("Step: Close the Gripper at full speed and full force\n");
    uint16_t close_gripper_request_register[READ_WRITE_REG_LENGTH] = {
        GRIPPER_FLAGS_MOVE, GRIPPER_CLOSED, FULL_SPEED_FORCE};
    modbus_write_registers(
        ctx, SERIAL_INPUT_FIRST_REG, READ_WRITE_REG_LENGTH, close_gripper_request_register);
}

void open_gripper(modbus_t* ctx) {
    printf("Step: Open the Gripper at full speed and full force\n");
    uint16_t open_gripper_request_register[READ_WRITE_REG_LENGTH] = {
        GRIPPER_FLAGS_MOVE, GRIPPER_OPENED, FULL_SPEED_FORCE};
    modbus_write_registers(
        ctx, SERIAL_INPUT_FIRST_REG, READ_WRITE_REG_LENGTH, open_gripper_request_register);
}

void wait_movement_complete(modbus_t* ctx) {
    printf("Step: Read Gripper status until movement is complete\n");
    uint16_t gripper_status_movement[READ_WRITE_REG_LENGTH] = {};

    modbus_read_registers(
        ctx, SERIAL_OUTPUT_FIRST_REG, READ_WRITE_REG_LENGTH, gripper_status_movement);

    while(gripper_status_movement[0] != GRIPPER_FLAGS_CLOSED
          && gripper_status_movement[0] != GRIPPER_FLAGS_NO_OBJECT) {
        printf(
            "Gripper not yet closed: %.4X vs. %.4X or %.4X\n",
            gripper_status_movement[0],
            GRIPPER_FLAGS_NO_OBJECT,
            GRIPPER_FLAGS_CLOSED);
        modbus_read_registers(
            ctx, SERIAL_OUTPUT_FIRST_REG, READ_WRITE_REG_LENGTH, gripper_status_movement);
        sleep_100ms();
    }
}

int main(void) {
    modbus_t* mb;

    mb = modbus_new_rtu(DEVICE_NAME, BAUDRATE, PARITY, DATA_BITS, STOP_BIT);
    modbus_set_slave(mb, SERVER_ID);
    modbus_set_debug(mb, DEBUG_MODBUS);
    modbus_connect(mb);
    printf("\nConnected\n");

    test_connection(mb);

    activate(mb);
    wait_activation_complete(mb);
    printf("Gripper activated, press any key to continue\n");
    getc(stdin);

    printf("BYPASS: Step: Move the robot to the pick-up location\n");

    close_gripper(mb);
    wait_movement_complete(mb);
    printf("Gripper closed, press any key to continue\n");
    getc(stdin);

    printf("BYPASS: Step: Move the robot to the release location\n");

    open_gripper(mb);
    wait_movement_complete(mb);
    printf("Gripper opened\n");

    printf("Test finished, closing connection to the Gripper\n");
    modbus_close(mb);
    modbus_free(mb);
}
