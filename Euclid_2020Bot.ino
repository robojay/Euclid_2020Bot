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
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ?( x) : (y))


const uint8_t LeftAPin = 10;
const uint8_t LeftBPin = 9;
const uint8_t RightAPin = 6;
const uint8_t RightBPin = 5;


ros::NodeHandle  nh;

//Published Message

//default gains
float forwardMultiply = 1;
float backwardMultiply = 1;
float rotationMultiply = 1;
float leftGain = 1;
float rightGain = 1;


//Arduino periodicaly tries to connect to ROS, When a handshake is recieved it stops.
bool isReady = false;
void setIsReady(bool status){
  isReady = status;
  digitalWrite(13, status == false ? 0 : 1);
}    

//Callback function for the cmd_vel topic messges.
void messageCb( const geometry_msgs::Twist& msg) {
  static int lastLeft = 0;
  static int lastRight = 0;
  
  float leftSpeed;
  float rightSpeed;

  const float deadZone = 0.05;

  //  if ( msg.linear.x  <= 0.001 && msg.linear.x >= -0.001) {
  //    rightSpeed = MIN(msg.angular.z * 255 * rotationMultiply, 255);
  //    leftSpeed = MIN(msg.angular.z * -255 * rotationMultiply, 255);
  //  }
  //  else {
  //    rightSpeed = MIN(MAX(-255, msg.linear.x * 255.0 + msg.angular.z * 60.0), 255);
  //    leftSpeed = MIN(MAX(-255, msg.linear.x * 255.0 - msg.angular.z * 60.0), 255);
  //  }

  if ( (msg.linear.x > deadZone) || (msg.linear.x < -deadZone) ) {
    leftSpeed = 2.0 * msg.linear.x;
    rightSpeed = 2.0 * msg.linear.x; 
  }
  else {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  if ( (msg.angular.z > (deadZone/2.0)) || (msg.angular.z < -(deadZone/2.0)) ) {
    leftSpeed += msg.angular.z;
    rightSpeed -= msg.angular.z; 
  }
  
  leftSpeed *= 127.5;
  rightSpeed *= 127.5;

  if ((int)(leftSpeed) != lastLeft) {
    if (leftSpeed > 0.0) {
      analogWrite(LeftAPin, (int)leftSpeed);
      analogWrite(LeftBPin, 0);
    }
    else {
      analogWrite(LeftAPin, 0);
      analogWrite(LeftBPin, -(int)leftSpeed);
    }
    lastLeft = (int)(leftSpeed);
  }

  if ((int)(rightSpeed) != lastRight) {
    if (rightSpeed > 0.0) {
      analogWrite(RightAPin, (int)rightSpeed);
      analogWrite(RightBPin, 0);
    }
    else {
      analogWrite(RightAPin, 0);
      analogWrite(RightBPin, -(int)rightSpeed);
    }
    lastRight = (int)(rightSpeed);
  }
}

//callback functions for the forward/backward/rotaion/right/left gains.

void setForwardMul( const std_msgs::Float32& msg) {
  //When a message is recieved set connected = true
  setIsReady(true);
  forwardMultiply = msg.data;
}

void setBackwardMul( const std_msgs::Float32& msg) {
  backwardMultiply = msg.data;
}

void setRotationMul( const std_msgs::Float32& msg) {
  rotationMultiply = msg.data;
}

void setRightGain( const std_msgs::Float32& msg) {
  rightGain = msg.data;
}
void setLeftGain( const std_msgs::Float32& msg) {
  leftGain = msg.data;
}


//Declaration of the publisher
std_msgs::Bool arduinoRegisterMessage;
ros::Publisher mArduinoStatusPub("/arduino_ready", &arduinoRegisterMessage);

//declaration of the command velocity topic subscriber
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel_mux/input/teleop", messageCb );

//subscribers declaration for the gain settings
ros::Subscriber<std_msgs::Float32> sub1("/set_forward_multiply", setForwardMul );
ros::Subscriber<std_msgs::Float32> sub2("/set_backward_multiply", setBackwardMul );
ros::Subscriber<std_msgs::Float32> sub3("/set_rotation_multiply", setRotationMul );

ros::Subscriber<std_msgs::Float32> sub4("/set_right_gain", setRightGain );
ros::Subscriber<std_msgs::Float32> sub5("/set_left_gain", setLeftGain );


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
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);
}

void registerArduino(){
    arduinoRegisterMessage.data = false;
    mArduinoStatusPub.publish(&arduinoRegisterMessage);
    delay(50);
}

void loop() {
  //Register the arduino in ROS and wait for the system to be ready.
  if (isReady == false) {
    registerArduino();
  }
  nh.spinOnce();  //Allows ROS to run and to send/receive new messages.

}

