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
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, OP_POSITION);
  dxl_2.torqueOn(DXL_ID_2);
}

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  motor_setup(); 
}

void loop() {
  // put your main code here, to run repeatedly:

  // readSerial function uses Serial input to run the following switch case
  readSerial();
  if(newData == true){
    switch(input){
      case 'w':
        Serial.println("Input = w, increasing motor 1 position");
        dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_DEGREE) + 10, UNIT_DEGREE);
        Serial.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW));
        delay(100);
        newData = false;
        break;
      
      case 's':
        Serial.println("Input = s, decreasing motor 1 position");
        dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_DEGREE) - 10, UNIT_DEGREE);
        Serial.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW));
        delay(100);
        newData = false;
        break;
        
      case 'a':
        Serial.println("Input = a, increasing motor 2 position");
        dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_DEGREE) + 10, UNIT_DEGREE);
        Serial.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
        delay(100);
        newData = false;
        break;

      case 'd':
        Serial.println("Input = d, decreasing motor 2 position");
        dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_DEGREE) - 10, UNIT_DEGREE);
        Serial.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
        delay(100);
        newData = false;
        break;

      // Enter any other key to stop the motors
      default:
        newData = false;
        break;
    }
  }
}

void readSerial() {
  if(Serial.available() > 0){
    input = Serial.read();
    newData = true;
  }
}
