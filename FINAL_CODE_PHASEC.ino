/* PHASE C FINAL CODE */
/* FOR EN1/DIR1: REFERRING TO RIGHT MOTOR */
/* FOR EN2/DIR2: REFERRING TO LEFT MOTOR */

#include <PID_v1.h>
#include "Shield2AMotor.h"
#include <Arduino.h>
#include <Servo.h>

Servo servo;
int pos = 0;

//Define Variables we'll be connecting to
double Setpoint1, Setpoint2, Input1, Input2, Output1, Output2;

//Front Sensor Variables
double error_F;
double desired_distance_F;
double measured_distance_F;
double turn_rate_F;
double forward_rate_F;

//Right Sensor Variables
double error_R;
double desired_distance_R;
double measured_distance_R;
double turn_rate_R;
double forward_rate_R;

//Left Sensor Variables
double error_L;
double desired_distance_L;
double measured_distance_L;
double turn_rate_L;
double forward_rate_L;

//Define motor speed values
int left_motor = 0;
int right_motor = 0;

//Specify the links and initial tuning parameters
double Kp=10, Ki=5, Kd=0.5;  // PID Gains for Sensors
double Kp1=2, Ki1=0.5, Kd1=0.01; // PID Gains for Motor 1
double Kp2=2, Ki2=0.5, Kd2=0.01; // PID Gains for Motor 2
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

void setup()
{

  Serial.begin(9600);

  servo.attach(2);

  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  analogWrite(EN1, 0);    
  digitalWrite(DIR1, LOW); //low is forwards, high is backwards
  analogWrite(EN2, 0);
  digitalWrite(DIR2, LOW);
  
  //initialize the variables we're linked to
  Input1 = analogRead(EN1);
  Input2 = analogRead(EN2);
  Setpoint1 = 100;
  Setpoint2 = 100;

  //turn the PID on
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);

}

void loop()
{
    SetMotorPower(255, 255); // Sets motor speed for left motor and right motor, respectively
    FrontSensor();           // calls function to activate
    Center();                // calls function to activate
    StopServo();             // calls function to activate

    int A = analogRead(A0); // Assigns 'A' to analog pin A0 for Front IR sensor
    int B = analogRead(A5); // Assigns 'B' to analog pin A5 for Right IR sensor 
    int C = analogRead(A4); // Assigns 'C' to analog pin A4 for Left IR sensor 

    Serial.print("Front Sensor: ");
    Serial.println(A);

    Serial.print("Right Sensor: ");
    Serial.println(B);

    Serial.print("Left Sensor: ");
    Serial.println(C);

    if(620 > B && 671 > A) // Tells system to turn Left while reading front and left sensor values
    {
      FrontSensor();
      delay(1000);
      Stop();
      delay(1000);
      TurnLeft();       // Go to class TurnLeft() in this case
      delay(3000);
    }

    if(610 > C && 671 > A) // Tells system to turn Right while reading front and right sensor values
    {
      FrontSensor();
      delay(1000);
      Stop();
      delay(1000);
      TurnRight();        // Go to class TurnRight() in this case
      delay(3000);
    }

    if (A > 670) // Tells the system to turn right or left in a "dead-end" situation
    {
      if(B < 500)       // If the right sensor does not detect a wall...
      {
        FrontSensor();
        Stop();
        delay(1000);
        TurnLeft();     // Go to class TurnLeft() in this case
        delay(1000);
      }
      else if (C < 500) // If the left sensor does not detect a wall...
      {
        FrontSensor();
        Stop();
        delay(1000);
        TurnRight();    // Go to class TurnRight() in this case
        delay(1000);
      }
    }

    if(600 < A < 650 && 581 < B < 661 && 559 < C < 660) // Tells system to reverse while checking if all sensor values are detecting walls at high values
    {
      FrontSensor();
      Stop();
      delay(1000);
      Reverse();      // Go to class Reverse() in this case
      delay(1000);
    }

// The following lines of code below for each distance dictates when the robot moves at 
// certain instances in the map for Phase C (map path provided in report)

    //Distance 1:
    if(A > 200 && B < 581)
    {
        FrontSensor();
        Stop();
        delay(1000);
        TurnLeft();
        delay(3000);
    }
    //Distance 2:
    if(C > 378 && B < 125)
    {
        Stop();
        delay(1000);
        TurnRight();
        delay(3000);
    }

    //Point to pick up ore:
    if(A > 600 && B > 126) {
      Stop();
      delay(2000);
      SetMotorPower(255, 255);
      MoveServo();
      delay(5000);
      StopServo();
      delay(2000);
      Reverse();
      delay(7000);
      TurnLeft();
      delay(2000);
    }
    //Distance 3:
    if(C > 400 && B < 205)
    {
      if(A < 216) // 24 cm
      {
        Stop();
        delay(1000);
        TurnRight();
      }
    }
    //Distance 4:
    if(C > 585 && A < 630)
    {
        Stop();
        delay(1000);
        TurnLeft();
    }
    //Distance 5
    if(C > 610 && A < 442)
    {
        Stop();
        delay(1000);
        TurnLeft();
    }

    //Distance 6
    if (A > 671 && B > 375) {
      Stop();
      delay(1000);
      TurnRight();
      delay(1000);
    }
    
    //Distance 7
    if (A > 207 && B > 601 && C > 180) {
      Stop();
      delay(1000);
      TurnLeft();
      delay(1000);
    }

    // Endpoint to drop off rocks
    if(A > 640 && B > 600 && C > 550)
    {
      Stop();
      delay(1000);
      MoveServo();
      delay(5000);
      StopServo();
    }

    delay(500);
}

void StopServo() {
  pos = 0;
  servo.write(pos);
}

void MoveServo()
{   
  pos = 30;
  servo.write(pos);
}

void FrontSensor()
{
  desired_distance_F = 6.0;
  measured_distance_F = -0.0883*(analogRead(A0)) + 64.542;
  error_F = measured_distance_F - desired_distance_F;
  if(error_F < 1.0 && error_F > 0.0)
  {
    Stop();
  }
  else if (error_F < 0.0)
  {
    left_motor = 50;
    right_motor = 50;
    digitalWrite(DIR1, HIGH); // backwards
    digitalWrite(DIR2, HIGH); // backwards

    SetMotorPower(left_motor, right_motor);
  }
  
}

void Center() {
  // Left Sensor
  desired_distance_L = 14.0;
  measured_distance_L = -0.0883*(analogRead(A4)) + 64.542; // Converts sensor output to approx. distance in cm
  Serial.print("measured_distance_L: ");
  Serial.println(measured_distance_L);
  if(measured_distance_L > desired_distance_L)
  {
    error_L = measured_distance_L - desired_distance_L;

    turn_rate_L = Kp*error_L; //

    forward_rate_L = 30;
    
    left_motor = forward_rate_L + turn_rate_L; 
    right_motor = forward_rate_L - turn_rate_L;

    SetMotorPower(left_motor, right_motor);
  } else
  {
    left_motor = forward_rate_L - turn_rate_L; 
    right_motor = forward_rate_L + turn_rate_L;

    SetMotorPower(left_motor, right_motor);
  }
  
}

void Stop()
{
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
}

void TurnRight()
{
  analogWrite(EN1, 0);    // tells the right motor to speed up and reverse
  digitalWrite(DIR1, LOW); 
  analogWrite(EN2, 255);    
  digitalWrite(DIR2, LOW);
}

void TurnLeft()
{
  analogWrite(EN1, 255);    // tells the right motor to a set speed and move forward
  digitalWrite(DIR1, LOW);
  analogWrite(EN2, 0);   
  digitalWrite(DIR2, LOW);
}

void Reverse()
{
  analogWrite(EN1, 127);    // tells the right motor to a set speed and reverse
  digitalWrite(DIR1, HIGH);
  analogWrite(EN2, 120);    // tells the left motor to a set speed and reverse, causing the robot to reverse as a unit
  digitalWrite(DIR2, HIGH);
}

void SetMotorPower(int left_motor, int right_motor) // Function to set motor speed and direction
{
   //handle direction
  if(left_motor > 0)
  {
    digitalWrite(DIR2, LOW); //forwards
  }
  else
  {
    digitalWrite(DIR2, HIGH); //backwards
  } 
  
  if(right_motor > 0)
  {
    digitalWrite(DIR1, LOW); //forwards
  }
  else
  {
    digitalWrite(DIR1, HIGH); //backwards
  } 

//handle power settings here
  if(left_motor < 0)
    {left_motor = left_motor*-1;}
  if(right_motor < 0)
    {right_motor = right_motor*-1;}
  
  if(left_motor > 255)
  {
    left_motor = 150;
  }
  
  if(right_motor > 255)
  {
    right_motor = 200;
  }

  analogWrite(EN2, left_motor);
  analogWrite(EN1, right_motor);  
}
