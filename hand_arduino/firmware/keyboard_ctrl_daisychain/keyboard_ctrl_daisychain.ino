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
const float pos_init_1 = -400.0;
const float pos_init_2 = 1000.0;

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
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN for the first motor

const uint8_t DXL_ID_1 = 1;

const uint8_t DXL_ID_2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
const int32_t BAUD_RATE_MOTOR = 57600;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
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
        dxl.setGoalPosition(DXL_ID_1, pos_init_1, UNIT_RAW);
        grasp_state = not_grasping;
        newData = false;
        break;
      case '1': // parallel
        dxl.setGoalPosition(DXL_ID_2, pos_init_2, UNIT_RAW);
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_2, UNIT_RAW));
        newData = false;
        delay(100);
        break;
      case '2': // triangle
        dxl.setGoalPosition(DXL_ID_2, spread_pos_mid, UNIT_RAW);
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_2, UNIT_RAW));
        newData = false;
        delay(100);
        break;
      case 'w': // close plam by a little
        dxl.setGoalPosition(DXL_ID_1, dxl.getPresentPosition(DXL_ID_1, UNIT_RAW)+50, UNIT_RAW);
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_1, UNIT_RAW));
        delay(100);
        newData = false;
        break;
      case 's': // open plam by a little
        enable_pos_ctrl();
        dxl.setGoalPosition(DXL_ID_1, dxl.getPresentPosition(DXL_ID_1, UNIT_RAW)-50, UNIT_RAW);
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_1, UNIT_RAW));
        delay(100);
        newData = false;
        break;
      case 'a': // spread two fingers by a little
        dxl.setGoalPosition(DXL_ID_2, dxl.getPresentPosition(DXL_ID_2, UNIT_RAW)-40, UNIT_RAW);
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_2, UNIT_RAW));
        delay(100);
        newData = false;
        break;
      case 'd': // unspread two fingers by a little
        dxl.setGoalPosition(DXL_ID_2, dxl.getPresentPosition(DXL_ID_2, UNIT_RAW)+40, UNIT_RAW);
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_2, UNIT_RAW));
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
    dxl.torqueOff(DXL_ID_1);
    dxl.setOperatingMode(DXL_ID_1, OP_POSITION);
    dxl.torqueOn(DXL_ID_1);
    pos_ctrl = true;
  }
}

void enable_voltage_ctrl() {
  if (pos_ctrl)
  {
    dxl.torqueOff(DXL_ID_1);
    dxl.setOperatingMode(DXL_ID_1, OP_PWM);
    dxl.torqueOn(DXL_ID_1);
    pos_ctrl = false;
  }
}

void grasp() {
  if (dxl.getPresentPosition(DXL_ID_1, UNIT_RAW) >= grasp_pos_max)
  {
    enable_pos_ctrl();
    dxl.setGoalPosition(DXL_ID_1, grasp_pos_max, UNIT_RAW);
    grasp_state = fully_closed;
    newData = false;
    return;
  }

  load_pre = dxl.readControlTableItem(PRESENT_LOAD, DXL_ID_1);
  if (load_pre < grasp_force_threshold) //Constant Velocity
  {
    enable_pos_ctrl();
    dxl.setGoalPosition(DXL_ID_1, dxl.getPresentPosition(DXL_ID_1, UNIT_RAW) + grasp_speed, UNIT_RAW);
    grasp_state = grasping;
    delay(10);
  }
  else
  {
    enable_voltage_ctrl();
    dxl.setGoalPWM(DXL_ID_1, grasp_force, UNIT_PERCENT);
    grasp_state = grasped;
    newData = false;
  }
}

void initialization() {
  dxl.setGoalPosition(DXL_ID_1, pos_init_1, UNIT_RAW);
  delay(500);
  //  motor_states[0] = 4;
  dxl.setGoalPosition(DXL_ID_2, pos_init_2, UNIT_RAW);
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
  dxl.begin(BAUD_RATE_MOTOR);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information
  dxl.ping(DXL_ID_1);
  dxl.ping(DXL_ID_2);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_1);

  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_2, OP_POSITION);
  dxl.torqueOn(DXL_ID_2);
}

float filter() {
  sum = sum - readings[my_index];
  value = dxl.readControlTableItem(PRESENT_LOAD, DXL_ID_1);
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
