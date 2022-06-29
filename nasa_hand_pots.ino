#include <ros.h>
#include <std_msgs/Float32.h>
#include <nasa_hand_msgs/nasaHandPots.h>

ros::NodeHandle  nh;

nasa_hand_msgs::nasaHandPots pd_msg;
ros::Publisher potpub("pot_pub", &pd_msg);

int potPins[] = {14, 15, 16, 17, 18, 19};
float myVoltages[6];

float avgVoltVal;

void setup() {
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(potpub);
  
}

void loop() {
  //Serial.begin(500000);
  for (int i = 0; i < 6; i++) {
    float voltVal = 0; //voltage value coming from the pot
    float netVoltVal = 0; //sum of the 75 inputs, to be averaged

    for(int j = 0; j<75; j++){
      voltVal = analogRead(potPins[i]);
      netVoltVal = netVoltVal + voltVal;
      }

      avgVoltVal = netVoltVal / 75;
      myVoltages[i] = avgVoltVal;
    
      //Serial.print("Pot ");
      //Serial.print(i + 1);
      //Serial.print(": ");
      //Serial.println(myVoltages[i]);
  }
    //Serial.println();
  

  pd_msg.pot1 = myVoltages[0];
  pd_msg.pot2 = myVoltages[1];
  pd_msg.pot3 = myVoltages[2];
  pd_msg.pot4 = myVoltages[3];
  pd_msg.pot5 = myVoltages[4];
  pd_msg.pot6 = myVoltages[5];
  //pd_msg.pot7 = myVoltages[6];
  //pd_msg.pot8 = myVoltages[7];
  
  potpub.publish(&pd_msg);
  
  nh.spinOnce();

  delay(10);
}
