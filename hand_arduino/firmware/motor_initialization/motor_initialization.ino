#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

// ************************ Motor Settings and Variables ******************************* //
#define DXL_SERIAL_1   Serial1
#define DXL_SERIAL_2   Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN_1 = 6; // DYNAMIXEL Shield DIR PIN for the first motor
const uint8_t DXL_DIR_PIN_2 = 2; // DYNAMIXEL Shield DIR PIN for the second motor

const uint8_t DXL_ID_1 = 11;
const uint8_t DXL_ID_2 = 1;
const float DXL_PROTOCOL_VERSION_1 = 2.0;
const float DXL_PROTOCOL_VERSION_2 = 6.0;
const int32_t BAUD_RATE_MOTOR = 1000000;

Dynamixel2Arduino dxl_1(Serial1, 2);
Dynamixel2Arduino dxl_2(Serial2, 6);
// ************************************************************************************** //

char input;
boolean newData = false;

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  motor_setup();
}

void loop() {
  readSerial();
  if(newData == true){
    switch(input){
      case 'i':
        dxl_1.setGoalPosition(DXL_ID_1, 2700, UNIT_RAW);
        dxl_2.setGoalPosition(DXL_ID_2, 220, UNIT_RAW);
        delay(1000);
        DEBUG_SERIAL.print("M1 = ");
        DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW));
        DEBUG_SERIAL.print("M2 = "  );
        DEBUG_SERIAL.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
        newData = false;
        break;

      case 'w':
        DEBUG_SERIAL.println("Input = w, increasing motor 1 position");
        dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_DEGREE) + 10, UNIT_DEGREE);
        DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW));
        delay(100);
        newData = false;
        break;

      case 's':
        DEBUG_SERIAL.println("Input = s, decreasing motor 1 position");
        dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_DEGREE) - 10, UNIT_DEGREE);
        DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW));
        delay(100);
        newData = false;
        break;
        
      case 'a':
        DEBUG_SERIAL.println("Input = a, increasing motor 2 position");
        dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_DEGREE) + 10, UNIT_DEGREE);
        DEBUG_SERIAL.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
        delay(100);
        newData = false;
        break;

      case 'd':
        DEBUG_SERIAL.println("Input = d, decreasing motor 2 position");
        dxl_2.setGoalPosition(DXL_ID_2, dxl_2.getPresentPosition(DXL_ID_2, UNIT_DEGREE) - 10, UNIT_DEGREE);
        DEBUG_SERIAL.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
        delay(100);
        newData = false;
        break;

      case 'r':
        DEBUG_SERIAL.print("Motor 1 position = ");
        DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW));
        DEBUG_SERIAL.print("Motor 2 position = ");
        DEBUG_SERIAL.println(dxl_2.getPresentPosition(DXL_ID_2, UNIT_RAW));
        delay(100);
        newData = false;
        break;
    }
  }
}

// ************************************************************************************** //
// ********************************** Functions ***************************************** //
// ************************************************************************************** //

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

  if(dxl_2.ping(DXL_ID_2) == true) {
    DEBUG_SERIAL.print("ping succeeded!");
  }
  else {
    DEBUG_SERIAL.println("ping failed!");
  }

  // Turn off torque when configuring items in EEPROM area
  dxl_1.torqueOff(DXL_ID_1);
  dxl_1.setOperatingMode(DXL_ID_1, OP_EXTENDED_POSITION);
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, OP_EXTENDED_POSITION);
  dxl_2.torqueOn(DXL_ID_2);
}

void readSerial() {
  if(Serial.available() > 0){
    input = Serial.read();
    newData = true;
  }
}
