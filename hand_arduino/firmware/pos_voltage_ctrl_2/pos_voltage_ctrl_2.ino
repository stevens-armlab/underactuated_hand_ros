#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <hand/pos_ctrl_1.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <hand/grasp.h>

using namespace ControlTableItem;

ros::NodeHandle nh;

float temp_ave = 0.0;
float load_pre = 0.0;
const float pos_init_1 = 1154.0;
const float pos_init_2 = 509.0;

bool pos_ctrl = true;

float grasp_speed = 20;
float grasp_force_threshold = 90;
float grasp_force = 30;
float grasp_pos_max = 4080.0;
float spread_pos_max = 539.0; // parallel
float spread_pos_min = 262.0; // triangle

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
// ************************************************************************************** //

void hand_grasp_set_position_cb(const std_msgs::Float64& data);
void hand_grasp_start_grasp_cb(const hand::grasp& data);
void hand_grasp_stop_cb(const std_msgs::Bool& data);
void enable_pos_ctrl();
void enable_voltage_ctrl();
void grasp();

void hand_spread_set_position_cb(const std_msgs::Float64& data);
void start_spread_cb(const std_msgs::Bool& data);
void start_unspread_cb(const std_msgs::Bool& data);
void hand_spread_stop_cb(const std_msgs::Bool& data);
void spread();
void unspread();

void initialization();
void motor_setup();
float filter();

std_msgs::Int8 grasp_state_msg;
std_msgs::Int8 spread_state_msg;
std_msgs::Float64 load_msg;
std_msgs::Float64 grasp_position_msg;
std_msgs::Float64 spread_position_msg;

ros::Subscriber<std_msgs::Float64> grasp_set_pos_sub("hand/grasp/set_position", &hand_grasp_set_position_cb);
ros::Subscriber<hand::grasp> start_grasp_sub("hand/grasp/start_grasp", &hand_grasp_start_grasp_cb);
ros::Subscriber<std_msgs::Bool> stop_grasp_sub("hand/grasp/stop", &hand_grasp_stop_cb);

ros::Subscriber<std_msgs::Float64> spread_set_pos_sub("hand/spread/set_position", &hand_spread_set_position_cb);
ros::Subscriber<std_msgs::Bool> start_spread_sub("hand/spread/start_spread", &start_spread_cb);
ros::Subscriber<std_msgs::Bool> start_unspread_sub("hand/spread/start_unspread", &start_unspread_cb);
ros::Subscriber<std_msgs::Bool> stop_spread_sub("hand/spread/stop", &hand_spread_stop_cb);

ros::Publisher grasp_state_pub("hand/grasp/state", &grasp_state_msg);
ros::Publisher spread_state_pub("hand/spread/state", &spread_state_msg);
ros::Publisher grasp_load_pub("hand/grasp/load", &load_msg);
ros::Publisher grasp_position_pub("hand/grasp/position", &grasp_position_msg);
ros::Publisher spread_position_pub("hand/spread/position", &spread_position_msg);

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  motor_setup();

  nh.initNode();
  //  nh.advertise(pub_motor_states);
  nh.advertise(grasp_state_pub);
  nh.advertise(spread_state_pub);
  nh.advertise(grasp_load_pub);
  nh.advertise(grasp_position_pub);
  nh.advertise(spread_position_pub);
  
  nh.subscribe(grasp_set_pos_sub);
  nh.subscribe(start_grasp_sub);
  nh.subscribe(stop_grasp_sub);
  nh.subscribe(spread_set_pos_sub);
  nh.subscribe(start_spread_sub);
  nh.subscribe(start_unspread_sub);
  nh.subscribe(stop_spread_sub);
  
  initialization();
}

void loop() {
  if (grasp_state != not_grasping)
  {
    grasp();
  }
  else
  {
    enable_pos_ctrl();
  }

  if (spread_state != stay)
  {
    if (spread_state == spreading)
    {
      spread();
    }
    if (spread_state == unspreading)
    {
      unspread();
    }
  }

  grasp_state_msg.data = grasp_state;
  grasp_state_pub.publish(&grasp_state_msg);

  spread_state_msg.data = spread_state;
  spread_state_pub.publish(&spread_state_msg);
  
  load_msg.data = dxl_1.readControlTableItem(PRESENT_LOAD, DXL_ID_1);
  grasp_load_pub.publish(&load_msg);

  grasp_position_msg.data = dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW);
  grasp_position_pub.publish(&grasp_position_msg);

  spread_position_msg.data = dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW);
  spread_position_pub.publish(&spread_position_msg);
  
  nh.spinOnce();
  delay(10);
}

// ************************************************************************************** //
// ********************************** Functions ***************************************** //
// ************************************************************************************** //

void hand_grasp_set_position_cb(const std_msgs::Float64& data) {
  enable_pos_ctrl();
  dxl_1.setGoalPosition(DXL_ID_1, data.data, UNIT_RAW);
  grasp_state = not_grasping;
}

void hand_grasp_start_grasp_cb(const hand::grasp& data) {
  grasp_state = grasping;
  grasp_speed = data.speed;
  grasp_force_threshold = data.force_threshold;
  grasp_force = data.force;
  grasp_pos_max = data.pos_max;
}

void hand_grasp_stop_cb(const std_msgs::Bool& data) {
  if (data.data==true)
  {
    enable_pos_ctrl();
    grasp_state = not_grasping;    
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
  }
}

void hand_spread_set_position_cb(const std_msgs::Float64& data) {
  dxl_2.setGoalPosition(DXL_ID_2, data.data, UNIT_RAW);
  spread_state = stay;
}

void start_spread_cb(const std_msgs::Bool& data) {
  spread_state = spreading;
}

void spread() {
  if (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) >= spread_pos_max)
  {
    dxl_2.setGoalPosition(DXL_ID_2, spread_pos_max);
//    spread_state = fully_spread;
    return;
  }
  if (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) <= spread_pos_min)
  {
    dxl_2.setGoalPosition(DXL_ID_2, spread_pos_min);
//    spread_state = fully_unspread;
    return;
  }

//  if ( (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) < spread_pos_max) && (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) > spread_pos_min) )
//  {
    dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) + 10, UNIT_RAW);
    spread_state = spreading;
    delay(10);
//  }
}

void start_unspread_cb(const std_msgs::Bool& data) {
  spread_state = unspreading;
}

void unspread() {
  if (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) >= spread_pos_max)
  {
    dxl_2.setGoalPosition(DXL_ID_2, spread_pos_max);
//    spread_state = fully_spread;
    return;
  }
  if (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) <= spread_pos_min)
  {
    dxl_2.setGoalPosition(DXL_ID_2, spread_pos_min);
//    spread_state = fully_unspread;
    return;
  }

//  if ( (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) < spread_pos_max) && (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) > spread_pos_min) )
//  {
    dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) - 10, UNIT_RAW);
    spread_state = unspreading;
    delay(10);
//  }
}

void hand_spread_stop_cb(const std_msgs::Bool& data) {
  if (data.data == true)
  {
    spread_state = stay;
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
