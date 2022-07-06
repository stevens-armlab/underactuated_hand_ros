#include <ros.h>
#include <std_msgs/Float32.h>
#include <underactuated_hand/underactuatedHandSensors.h>
#include <Smoothed.h>   

ros::NodeHandle  nh;

underactuated_hand::underactuatedHandSensors pd_msg;
ros::Publisher potpub("pot_pub", &pd_msg);

int potPins[] = {14, 15, 16, 17, 18, 19};

Smoothed <int> myPot0; 
Smoothed <int> myPot1;
Smoothed <int> myPot2; 
Smoothed <int> myPot3;
Smoothed <int> myPot4;
Smoothed <int> myPot5;

int currentVal = 0;
int smoothFactor = 12; //default is 10

float avgVoltVal;

void setup() {
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(potpub);

//For even smoother, replace SMOOTHED_AVERAGE with SMOOTHED_EXPONENTIAL


  myPot0.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot1.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot2.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot3.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot4.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
  myPot5.begin(SMOOTHED_EXPONENTIAL, smoothFactor); 
 
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
  Serial.println();
  
  pd_msg.pot0 = myPot0.get();
  pd_msg.pot1 = myPot1.get();
  pd_msg.pot2 = myPot2.get();
  pd_msg.pot3 = myPot3.get();
  pd_msg.pot4 = myPot4.get();
  pd_msg.pot5 = myPot5.get();
 
  potpub.publish(&pd_msg);
  
  nh.spinOnce();

  delay(10);
}
