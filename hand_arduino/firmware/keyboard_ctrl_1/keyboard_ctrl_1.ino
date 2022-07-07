// ************************************************************************************** //
// Keybaord control hand open and close through Arduino Serial Monitor
// 'o' --> Open
// 'c' --> close
// '1' --> unspread to parallel pose
// '2' --> spread to triangle pose
// 'w' --> close plam by a little
// 's' --> open plam by a little
// 'a' --> spread two fingers by a little
// 'd' --> unspread two fingers by a little
// 'i' --> initialize both motors
// ************************************************************************************** //

#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

float temp_ave = 0.0;
float load_pre = 0.0;
const float pos_init_1 = 1154.0;
const float pos_init_2 = 509.0;

bool pos_ctrl = true;

float grasp_speed = 20;
float grasp_force_threshold = 90;
float grasp_force = 30;
float grasp_pos_max = 4080.0;
float spread_pos_max = 509.0; // parallel
float spread_pos_min = 200.0; 
float spread_pos_mid = 312.0; // triangle

enum grasp_state_enum
{
  grasping,
  grasped,
  fully_closed, // nothing grasped
  not_grasping
};
grasp_state_enum grasp_state = not_grasping;

enum spread_state_enum
{
  spreading,
  unspreading,
  fully_spread,
  fully_unspread,
  stay
};
spread_state_enum spread_state = stay;

// ************************ Motor Settings and Variables ******************************* //
#define DXL_SERIAL_1   Serial1
#define DXL_SERIAL_2   Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN_1 = 2; // DYNAMIXEL Shield DIR PIN for the first motor
const uint8_t DXL_DIR_PIN_2 = 6; // DYNAMIXEL Shield DIR PIN for the second motor

const uint8_t DXL_ID_1 = 11;
const uint8_t DXL_ID_2 = 1;
const float DXL_PROTOCOL_VERSION_1 = 2.0;
const float DXL_PROTOCOL_VERSION_2 = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;

Dynamixel2Arduino dxl_1(DXL_SERIAL_1, DXL_DIR_PIN_1);
Dynamixel2Arduino dxl_2(DXL_SERIAL_2, DXL_DIR_PIN_2);
// ************************************************************************************** //

// ************************ Filter Settings and Variables ******************************* //
#define WINDOW_SIZE 5
float sum = 0;
float value = 0;
int my_index = 0;
float readings[WINDOW_SIZE];
float averaged = 0;
char input;
boolean newData = false;
// ************************************************************************************** //

void enable_pos_ctrl();
void enable_voltage_ctrl();
void grasp();

void initialization();
void motor_setup();
float filter();

void setup() {
  DEBUG_SERIAL.begin(115200);

  motor_setup();

  initialization();
}

void loop() {
  readSerial();
  if(newData == true){
    switch(input){
      case 'c': // close plam
        grasp();
        DEBUG_SERIAL.println("Grasping...");
        delay(100);
        break;
      case 'o': // open plam
        enable_pos_ctrl();
        dxl_1.setGoalPosition(DXL_ID_1, pos_init_1, UNIT_RAW);
        grasp_state = not_grasping;
        newData = false;
        break;
      case '1': // parallel
        dxl_2.setGoalPosition(DXL_ID_2, pos_init_2, UNIT_RAW);
        newData = false;
        delay(100);
        break;
      case '2': // triangle
        dxl_2.setGoalPosition(DXL_ID_2, spread_pos_mid, UNIT_RAW);
        newData = false;
        delay(100);
        break;
      case 'w': // close plam by a little
        dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW)+50, UNIT_RAW);
        delay(100);
        newData = false;
        break;
      case 's': // open plam by a little
        enable_pos_ctrl();
        dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW)-50, UNIT_RAW);
        delay(100);
        newData = false;
        break;
      case 'a': // spread two fingers by a little
        dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW)-40, UNIT_RAW);
        DEBUG_SERIAL.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
        delay(100);
        newData = false;
        break;
      case 'd': // unspread two fingers by a little
        dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW)+40, UNIT_RAW);
        DEBUG_SERIAL.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
        delay(100);
        newData = false;
        break;
      case 'i': // initialize
        initialization();
        newData = false;
        break;
      default:
        newData = false;
        break;
    }
  }
}

// ************************************************************************************** //
// ********************************** Functions ***************************************** //
// ************************************************************************************** //

void readSerial() {
  if(Serial.available() > 0){
    input = Serial.read();
    newData = true;
  }
}

void enable_pos_ctrl() {
  if (!pos_ctrl)
  {
    dxl_1.torqueOff(DXL_ID_1);
    dxl_1.setOperatingMode(DXL_ID_1, OP_POSITION);
    dxl_1.torqueOn(DXL_ID_1);
    pos_ctrl = true;
  }
}

void enable_voltage_ctrl() {
  if (pos_ctrl)
  {
    dxl_1.torqueOff(DXL_ID_1);
    dxl_1.setOperatingMode(DXL_ID_1, OP_PWM);
    dxl_1.torqueOn(DXL_ID_1);
    pos_ctrl = false;
  }
}

void grasp() {
  if (dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW) >= grasp_pos_max)
  {
    enable_pos_ctrl();
    dxl_1.setGoalPosition(DXL_ID_1, grasp_pos_max, UNIT_RAW);
    grasp_state = fully_closed;
    newData = false;
    return;
  }

  load_pre = dxl_1.readControlTableItem(PRESENT_LOAD, DXL_ID_1);
  if (load_pre < grasp_force_threshold) //Constant Velocity
  {
    enable_pos_ctrl();
    dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW) + grasp_speed, UNIT_RAW);
    grasp_state = grasping;
    delay(10);
  }
  else
  {
    enable_voltage_ctrl();
    dxl_1.setGoalPWM(DXL_ID_1, grasp_force, UNIT_PERCENT);
    grasp_state = grasped;
    newData = false;
  }
}

void initialization() {
  dxl_1.setGoalPosition(DXL_ID_1, pos_init_1, UNIT_RAW);
  delay(500);
  //  motor_states[0] = 4;
  dxl_2.setGoalPosition(DXL_ID_2, pos_init_2, UNIT_RAW);
  delay(500);
  //  motor_states[1] = 2;
  for (int filter_run_time = 0; filter_run_time < 5; filter_run_time++)
  {
    temp_ave = filter();
    delay(10);
  }
}

void motor_setup() {
  // This has to match with DYNAMIXEL baudrate.
  dxl_1.begin(BAUD_RATE_MOTOR);
  dxl_2.begin(BAUD_RATE_MOTOR);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_1.setPortProtocolVersion(DXL_PROTOCOL_VERSION_1);
  dxl_2.setPortProtocolVersion(DXL_PROTOCOL_VERSION_2);

  // Get DYNAMIXEL information
  dxl_1.ping(DXL_ID_1);
  dxl_2.ping(DXL_ID_2);

  // Turn off torque when configuring items in EEPROM area
  dxl_1.torqueOff(DXL_ID_1);
  dxl_1.setOperatingMode(DXL_ID_1, OP_POSITION);
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, OP_POSITION);
  dxl_2.torqueOn(DXL_ID_2);
}

float filter() {
  sum = sum - readings[my_index];
  value = dxl_1.readControlTableItem(PRESENT_LOAD, DXL_ID_1);
  readings[my_index] = value;
  sum = sum + value;
  my_index = (my_index + 1);
  if (my_index >= WINDOW_SIZE) {
    my_index = 0;
  }
  averaged = sum / WINDOW_SIZE;
  delay(1);
  return averaged;
}
