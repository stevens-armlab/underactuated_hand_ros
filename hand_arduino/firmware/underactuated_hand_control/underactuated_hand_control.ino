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

const uint8_t DXL_ID_1 = 11;
const uint8_t DXL_ID_2 = 1;
const float DXL_PROTOCOL_VERSION_1 = 2.0;
const float DXL_PROTOCOL_VERSION_2 = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;

Dynamixel2Arduino dxl_1(DXL_SERIAL_1, DXL_DIR_PIN_1);
Dynamixel2Arduino dxl_2(DXL_SERIAL_2, DXL_DIR_PIN_2);
// ************************************************************************************** //

char input;
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
  dxl_1.setOperatingMode(DXL_ID_1, OP_POSITION);
  //dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, OP_POSITION);
  //dxl_2.torqueOn(DXL_ID_2);
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
}

void loop() {
  
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
  
  //myRoll.add(roll);
  
  //delay(100);
  
  
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

  delay(10);
}
