// Jonas Nebl, 06-12-2022

#include <Servo.h>
#include <AccelStepper.h>

const int mainButtonPin = 3;
const int demoButtonPin = 2;

const int S0_pin = 5;
const int S1_pin = 4;
const int S2_pin = 7;
const int S3_pin = 6;
const int Sout_pin = 8;

int state = 0;
int color = 0;
long starttime = 0;
float red_bias = 0;
float green_bias = 0;

const int theta0_pos0 = 160;
const int theta0_pos1 = 0;
const int theta0_pickup = (theta0_pos0 - theta0_pos1)/2;
const int theta1_lifted = 42;
const int theta1_lifted_2inch = 60;
const int theta1_table = 22;
const int updowntime = 150;
const int grippertime = 140;

const int theta_gripper_open = 110;
const int theta_gripper_closed = 80;

const int motorInterfaceType = 1;
const int stepPin = 10;
const int dirPin = 9;
const int pinGripper = 11;
const int pinJoint1 = 13;
const int pinJoint2 = 12;

AccelStepper stepper_0 = AccelStepper(motorInterfaceType, stepPin, dirPin);
Servo servo_1;
Servo servo_2;
Servo servo_gripper;

void moveServos(int theta1)
{
  servo_1.write(theta1);
  servo_2.write(theta1 + 85);
}

int getColor()
{
  // check for red
  digitalWrite(S2_pin, LOW);
  digitalWrite(S3_pin, LOW);
  float red = pulseIn(Sout_pin, LOW, 100000) - red_bias;
  
  // check for green
  digitalWrite(S2_pin, HIGH);
  digitalWrite(S3_pin, HIGH);
  float green = pulseIn(Sout_pin, LOW, 100000) - green_bias;
  
  if (red < green - 10)
  {
    Serial.println("Detected color: red");
    return 0;
  }
  else
  {
    Serial.println("Detected color: green");
    return 1;
  }
}

int getGreenRaw()
{
  // check for green
  digitalWrite(S2_pin, HIGH);
  digitalWrite(S3_pin, HIGH);
  return pulseIn(Sout_pin, LOW, 100000);
}

int getRedRaw()
{
  // check for red
  digitalWrite(S2_pin, LOW);
  digitalWrite(S3_pin, LOW);
  return pulseIn(Sout_pin, LOW, 100000);  
}

void setup() 
{
  Serial.begin(115200);
  pinMode(mainButtonPin, INPUT);
  pinMode(S0_pin, OUTPUT);
  pinMode(S1_pin, OUTPUT);
  pinMode(S2_pin, OUTPUT);
  pinMode(S3_pin, OUTPUT);
  pinMode(Sout_pin, INPUT);

  // Setting frequency-scaling to 20%
  digitalWrite(S0_pin, HIGH);
  digitalWrite(S1_pin, LOW);
  stepper_0.setMaxSpeed(600.0);
  stepper_0.setAcceleration(2500.0);
  // stepper must be manually moved to theta0_pickup position before startup 
  stepper_0.setCurrentPosition(theta0_pickup);
 
  moveServos(theta1_lifted);
  servo_1.attach(pinJoint1);
  servo_2.attach(pinJoint2);
  
  servo_gripper.write(theta_gripper_open);
  servo_gripper.attach(pinGripper);

  // Color sensor calibration
  delay(500);
  int i_max = 100;
  for(int i = 0; i<= i_max; i++)
  {
    red_bias += ((float)getRedRaw()) / ((float)i_max);
    green_bias += ((float)getGreenRaw()) / ((float)i_max);
    delay(5);
  }
  Serial.print("Color sensor calibration completed. red_bias = ");
  Serial.print(red_bias);
  Serial.print(" green_bias = ");
  Serial.println(green_bias);
}

void loop() 
{
  switch(state)
  {
    case 0: // Waiting_at_initial
      // entry
      Serial.println(state);
      // do nothing
      // exit
      while(true)
      {
        if(digitalRead(mainButtonPin))
        {
          state = 1;
          starttime = millis();
          break;
        }
        if(digitalRead(demoButtonPin))
        {
          state = 8;
          starttime = millis();
          break;
        }
      }
      break;
    case 1: // Determining_color
      // entry
      Serial.println(state);
      // do
      color = getColor();
      // exit
      state = 2;
      break;
    case 2: // Picking up
      // entry
      Serial.println(state);
      // do
      moveServos(theta1_table);
      delay(updowntime);
      servo_gripper.write(theta_gripper_closed);
      delay(grippertime);
      moveServos(theta1_lifted);
      delay(updowntime);
      // exit
      state = color + 3;
      break;
    case 3: // Transport_to_pos0
      // entry
      Serial.println(state);
      // do
      stepper_0.runToNewPosition(theta0_pos0);
      // exit
      state = 5;
      break;
    case 4: // Transport_to_pos1
      // entry
      Serial.println(state);
      // do 
      stepper_0.runToNewPosition(theta0_pos1);
      // exit
      state = 5;
      break;
    case 5: // Dropping_off
      // entry
      Serial.println(state);
      // do
      moveServos(theta1_table);
      delay(updowntime);
      servo_gripper.write(theta_gripper_open);
      delay(grippertime);
      moveServos(theta1_lifted);
      delay(updowntime);
      // exit
      Serial.println(millis()-starttime);
      state = 6;
      break;
    case 6: // Waiting_after_Dropoff
      // entry
      Serial.println(state);
      // do nothing
      // exit
      while(!digitalRead(mainButtonPin)){}
      state = 7;
      break;
    case 7: // Moving_back_to_pickup
      // entry
      Serial.println(state);
      // do 
      stepper_0.runToNewPosition(theta0_pickup);
      // exit
      // button must not be pressed to prevent instant start in state 0
      while(digitalRead(mainButtonPin)){}
      state = 0;
      break;
    case 8: //2 inch lift demonstration
      // entry
      Serial.println(state);
      // do
      moveServos(theta1_table);
      delay(1000);
      servo_gripper.write(theta_gripper_closed);
      delay(grippertime);
      moveServos(theta1_lifted_2inch);
      delay(5000);
      moveServos(theta1_table);
      delay(1000);
      servo_gripper.write(theta_gripper_open);
      delay(grippertime);
      moveServos(theta1_lifted);
      delay(1000);
      // exit
      state = 0;
      break;
    default:
      // entry
      Serial.println("default");
      // exit
      break;
  }
}