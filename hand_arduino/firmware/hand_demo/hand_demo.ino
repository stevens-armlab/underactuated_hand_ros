#include <ros.h>
#include <std_msgs/Float32.h>
#include <hand_arduino/underactuatedHandSensors.h>
#include <Smoothed.h>   
#include <Dynamixel2Arduino.h>

// ************************ Motor Settings and Variables ******************************* //
#define DXL_SERIAL_1   Serial1
#define DXL_SERIAL_2   Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN_1 = 2; // DYNAMIXEL Shield DIR PIN for the first motor
const uint8_t DXL_DIR_PIN_2 = 6; // DYNAMIXEL Shield DIR PIN for the second motor

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const float DXL_PROTOCOL_VERSION_1 = 2.0;
const float DXL_PROTOCOL_VERSION_2 = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;

Dynamixel2Arduino dxl_1(DXL_SERIAL_1, DXL_DIR_PIN_1);
Dynamixel2Arduino dxl_2(DXL_SERIAL_2, DXL_DIR_PIN_2);
// ************************************************************************************** //

using namespace ControlTableItem;

float temp_ave = 0.0;
float load_pre = 0.0;
const float pos_init_1 = 1150.0;
const float pos_init_2 = 800.0;

bool pos_ctrl = true;

float grasp_pos_max = 5000.0;
float grasp_speed = 100.0;
float spread_pos_max = 200.0; 
float spread_pos_min = 450.0; // parallel
float spread_speed = 25.0;
//float spread_pos_mid = 312.0; // triangle


void enable_pos_ctrl();
void enable_voltage_ctrl();
void grasp();

void initialization();
void motor_setup();
float filter();
boolean newData = false;

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
  dxl_1.setOperatingMode(DXL_ID_1, OP_EXTENDED_POSITION);
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, OP_EXTENDED_POSITION);
  dxl_2.torqueOn(DXL_ID_2);
}

ros::NodeHandle  nh;

hand_arduino::underactuatedHandSensors pd_msg;
ros::Publisher potpub("pot_pub", &pd_msg);

int potPins[] = {14, 15, 16, 17, 18, 19};

Smoothed <int> myPot0; 
Smoothed <int> myPot1;
Smoothed <int> myPot2; 
Smoothed <int> myPot3;
Smoothed <int> myPot4;
Smoothed <int> myPot5;
Smoothed <int> myRoll;

int currentVal = 0;
int smoothFactor = 12; //default is 10
float roll = 0;

float avgVoltVal;

void setup() {
  nh.initNode();
  nh.advertise(potpub);

  //For even smoother, replace SMOOTHED_AVERAGE with SMOOTHED_EXPONENTIAL
  myPot0.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot1.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot2.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot3.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot4.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot5.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  //myRoll.begin(SMOOTHED_AVERAGE, 100); 

    // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  motor_setup(); 
  initialization();
}

void initialization() {
  dxl_1.setGoalPosition(DXL_ID_1, pos_init_1, UNIT_RAW);
  delay(500);
  dxl_2.setGoalPosition(DXL_ID_2, spread_pos_min, UNIT_RAW);
  delay(500);
}

void openAndClose() {
  while (dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW) <= grasp_pos_max) {
    dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW) + grasp_speed, UNIT_RAW);
    Serial.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW));
    readSensors();
  }
  delay(50);
  while (dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW) >= pos_init_1) {
    dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW) - grasp_speed, UNIT_RAW);
    Serial.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW));
    readSensors();
  }
  delay(500);
  dxl_2.setGoalPosition(DXL_ID_2, spread_pos_max, UNIT_RAW);
  delay(500);
  readSensors();
  dxl_2.setGoalPosition(DXL_ID_2, spread_pos_min, UNIT_RAW);
  readSensors();
  /*
  delay(50);
  while (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) >= spread_pos_max) {
    dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) - spread_speed, UNIT_RAW);
    Serial.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
    readSensors();
  }
  delay(50);
  while (dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) <= spread_pos_min) {
    dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW) + spread_speed, UNIT_RAW);
    Serial.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
    readSensors();
  }
  */
  
}

void readSensors() {
  currentVal = analogRead(potPins[0]);
  myPot0.add(currentVal);
  Serial.println(myPot0.get());

  currentVal = analogRead(potPins[1]);
  myPot1.add(currentVal);
  Serial.println(myPot1.get());

  currentVal = analogRead(potPins[2]);
  myPot2.add(currentVal);
  Serial.println(myPot2.get());

  currentVal = analogRead(potPins[3]);
  myPot3.add(currentVal);
  Serial.println(myPot3.get());
  
  currentVal = analogRead(potPins[4]);
  myPot4.add(currentVal);
  Serial.println(myPot4.get());

  currentVal = analogRead(potPins[5]);
  myPot5.add(currentVal);
  Serial.println(myPot5.get());
  
  
  //Serial.println("Roll motor position:");
  roll = dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW);
  Serial.println(roll);
  Serial.println();
  pd_msg.pot0 = myPot0.get();
  pd_msg.pot1 = myPot1.get();
  pd_msg.pot2 = myPot2.get();
  pd_msg.pot3 = myPot3.get();
  pd_msg.pot4 = myPot4.get();
  pd_msg.pot5 = myPot5.get();
  if (roll != 0) {
    pd_msg.roll = roll;
  }
  else {
  }
  
  potpub.publish(&pd_msg);
  
  nh.spinOnce();

  //delay(10);
}
  


void loop() {
  openAndClose();
  delay(50);
  //readSensors();
  
}
