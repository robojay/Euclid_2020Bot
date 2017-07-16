/******************************************************************************
  Copyright (c) 2016, Intel Corporation
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors
  may be used to endorse or promote products derived from this software without
  specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


const uint8_t LeftAPin = 10;
const uint8_t LeftBPin = 9;
const uint8_t RightAPin = 6;
const uint8_t RightBPin = 5;


ros::NodeHandle  nh;

//Published Message

//Arduino periodicaly tries to connect to ROS, When a handshake is recieved it stops.
bool isReady = false;
void setIsReady(bool status){
  isReady = status;
  digitalWrite(13, status == false ? 0 : 1);
}    

bool gotMessage = false;

void setLeft( const std_msgs::Int16& msg) {
  static int16_t lastLeft = 0;
  int16_t leftSpeed;
  
  gotMessage = true;
  
  leftSpeed = msg.data;

  if (leftSpeed != lastLeft) {
    lastLeft = leftSpeed;
    if (leftSpeed > 0) {
      leftSpeed = min(leftSpeed, 255);
      analogWrite(LeftAPin, leftSpeed);
      analogWrite(LeftBPin, 0);
    }
    else {
      leftSpeed = min(-leftSpeed, 255);
      analogWrite(LeftAPin, 0);
      analogWrite(LeftBPin, leftSpeed);
    }
  }
}

void setRight( const std_msgs::Int16& msg) {
  static int16_t lastRight = 0;
  int16_t rightSpeed;

  gotMessage = true;
  
  rightSpeed = msg.data;

  if (rightSpeed != lastRight) {
    lastRight = rightSpeed;  
    if (rightSpeed > 0) {
      rightSpeed = min(rightSpeed, 255);
      analogWrite(RightAPin, rightSpeed);
      analogWrite(RightBPin, 0);
    }
    else {
      rightSpeed = min(-rightSpeed, 255);
      analogWrite(RightAPin, 0);
      analogWrite(RightBPin, rightSpeed);
    }
  }
}


void stopMotors() {
  analogWrite(LeftAPin, 0);     
  analogWrite(LeftBPin, 0);     
  analogWrite(RightAPin, 0);     
  analogWrite(RightBPin, 0);
}

//Declaration of the publisher
std_msgs::Bool arduinoRegisterMessage;
ros::Publisher mArduinoStatusPub("/bot2020/ready", &arduinoRegisterMessage);

//subscribers declaration for the gain settings
ros::Subscriber<std_msgs::Int16> sub("/bot2020/left_motor", setLeft );
ros::Subscriber<std_msgs::Int16> sub1("/bot2020/right_motor", setRight );


void setup() {
  analogWrite(LeftAPin, 0);     
  analogWrite(LeftBPin, 0);     
  analogWrite(RightAPin, 0);     
  analogWrite(RightBPin, 0);

  nh.initNode(); //Initialize the node (needed by ROS)
  setIsReady(false); //Initialize ready status to false

  //Advertise the topic
  nh.advertise(mArduinoStatusPub);

  //Subscribing to the different topics, defined above
  nh.subscribe(sub);
  nh.subscribe(sub1);


  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  setIsReady(true);

}

void registerArduino(bool amReady){
    arduinoRegisterMessage.data = amReady;
    mArduinoStatusPub.publish(&arduinoRegisterMessage);
}

void loop() {
  static unsigned long readyTimer = 0;
  static unsigned long messageTimer = 0;
  unsigned long now = millis();
    
  //Register the arduino in ROS and wait for the system to be ready.
  if (now >= readyTimer) {
    registerArduino(isReady);
    readyTimer = now + 1000;
  }

  if (now > messageTimer) {
    if (gotMessage) {
      gotMessage = false;
    }
    else {
      stopMotors();
    }
    messageTimer = now + 5000;
  }

  nh.spinOnce();  //Allows ROS to run and to send/receive new messages.

}

