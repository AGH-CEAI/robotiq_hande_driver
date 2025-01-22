#include <stdio.h>
#include <unistd.h>
#include <modbus.h>

#define SERVER_ID 0x09;

int main(void) {
  modbus_t *mb;
  uint16_t tab_reg[32];

  mb = modbus_new_tcp("/tmp/ttyUR", 115200, 'N', 8, 1);
  modbus_set_slave(SERVER_ID);
  modbus_set_debug(TRUE);
  modbus_connect(mb);
  printf("\nConnected\n");

//   /* Read 5 registers from the address 0 */
//   modbus_read_registers(mb, 2000, 6, tab_reg);

//   printf("Status: %.2X %.2X %.2X %.2X %.2X %.2X\n" tab_reg[0], tab_reg[0], tab_reg[0], tab_reg[0], tab_reg[0], tab_reg[0]);


//   Modbus RTU example
  printf("Step 1 Activation Request (clear and set rACT)\n");
  uint16_t activation_request_register_reset = [0x00, 0x00, 0x00];
  modbus_write_registers(mb, 0x03E8, 3, activation_request_register_reset);

  uint16_t activation_request_register_set = [0x01, 0x00, 0x00];
  modbus_write_registers(mb, 0x03E8, 3, activation_request_register_set);


  printf("Step 2: Read Gripper status until the activation is completed\n");
  uint16_t activation_status = [0x0000];
  uint16_t activation_status_not_complete = [0x1100];
  uint16_t activation_status_complete = [0x3100];

  modbus_read_registers(mb, 0x07D0, 1, activation_status);

  while (activation_status[0] != activation_status_complete[0]){
    printf("Gripper not yet activated\n");
    modbus_read_registers(mb, 0x07D0, 1, activation_status);
    usleep(100 * 1000);     // ms * 1000
  }
  printf("Gripper activated\n");

  printf("BYPASS: Step 3: Move the robot to the pick-up location\n");

  printf("Step 4: Close the Gripper at full speed and full force\n");
  uint16_t close_gripper_request_register = [0x0900, 0x00FF, 0xFFFF];
  modbus_write_registers(mb, 0x03E8, 3, close_gripper_request_register);

  printf("Step 5: Read Gripper status until the grip is complete\n");
  uint16_t gripper_status_closing = [0x0000];
  uint16_t gripper_status_motion = [0x3900, 0x00FF, 0x0E0A];
  uint16_t gripper_status_closed_obj = [0xB900, 0x00FF, 0xBD00]; // gOBJ==0x02 or 0x03
  uint16_t gripper_status_requested_position_no_obj = [0xF900, 0x00FF, 0xBD00]; // gOBJ==0x02 or 0x03

  modbus_read_registers(mb, 0x07D0, 3, gripper_status_closing);

  while (gripper_status_closing[0] != gripper_status_closed_obj[0] || gripper_status_closing[0] != gripper_status_requested_position_no_obj[0]){
    printf("Gripper not yet closed\n");
    modbus_read_registers(mb, 0x07D0, 3, gripper_status_closing);
    usleep(100 * 1000);     // ms * 1000
  }
  printf("Gripper closed. Wiating 2s before continuation\n");

  sleep(2);
  
  printf("BYPASS: Step 6: Move the robot to the release location\n");

  printf("Step 7: Open the Gripper at full speed and full force\n");
  uint16_t open_gripper_request_register = [0x0900, 0x0000, 0xFFFF];
  modbus_write_registers(mb, 0x03E8, 3, open_gripper_request_register);

  printf("Step 8: Read Gripper status until opening is complete\n");
  uint16_t gripper_status_opening = [0x0000];
//   uint16_t gripper_status_motion = [0x3900, 0x00FF, 0x0E0A];
//   uint16_t gripper_status_closed_obj = [0xB900, 0x00FF, 0xBD00]; // gOBJ==0x02 or 0x03
//   uint16_t gripper_status_requested_position_no_obj = [0xF900, 0x00FF, 0xBD00]; // gOBJ==0x02 or 0x03

  modbus_read_registers(mb, 0x07D0, 3, gripper_status_opening);

  while (gripper_status_opening[0] != gripper_status_closed_obj[0] || gripper_status_opening[0] != gripper_status_requested_position_no_obj[0]){
    printf("Gripper not yet opened\n");
    modbus_read_registers(mb, 0x07D0, 3, gripper_status_opening);
    usleep(100 * 1000);     // ms * 1000
  }
  printf("Gripper opened\n");

  printf("Test finished, closing connection to the Gripper\n");

  modbus_close(mb);
  modbus_free(mb);
}