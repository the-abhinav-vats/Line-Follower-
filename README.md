# Line-Follower-
This repository helps to design the Line Follower using PID control with Arduino Nano.

# Components Required:
1. Arduino Nano Microcontroller
2. 2 x N20 Gear Motors
3. TB6612FNG Motor Driver
4. 12V to 5V buck convertor
5. 1 x Castor wheel
6. 8 x IR reciever and Transmitter
7. Pin Headers
8. 220 ohm Resistor
9. 10k Resistor
10. 12v Battery( You can use three 3.7v batteries with a battery holder)
11. PCB Board/ Zero PCB/ Purf Board
12. Wheels
13. Connecting Wires
    
# Tools Required:
1. Screwdriver Set
2. Soldering Iron
3. Glue Gun

# Builduing IR Sensor Array:
1. Put 8 recievers and transmitter on zero pcb board.
2. connect the -ve of transmitter to GND and +ve of reciever to GND via 10K resistor.
3. connect +ve of transmitter to 5V via 220 ohm and -ve of Receiver directly to 5V.  (refer to image given)

# Connections of Arduino Nano:

vcc -----> 5v
GND -----> GND


A0 - IR1
A1 - IR2
A2 - IR3
A3 - IR4
A4 - IR5
A5 - IR6
A6 - IR7
A7 - IR8

# Motor Driver:

PWMA - D11
PWMB - D3
AIN1 - D9
AIN2 - D10
BIN1 - D6
BIN2 - D5
STBY - D8

Vcc - 5v
VM - 12v
GND - GND (all 3 GND need to be shorted)

# SEE REFRENCE SCHEMATIC FOR CONNECTION:
REF1 - For connection of IR sensor to get analog output
REF2 - Design of IR sensor Array
REF3 - Complete Connection Diagram.



# Code:
// ===== 8 SENSOR PID LINE FOLLOWER =====
// Arduino Nano + TB6612FNG + 8 Analog IR Sensors
//ABHINAV VATS
//www.linkedin.com/in/abhinavvats06

// -------- IR Sensors --------
int sensorPins[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
int sensorValue[8];

#define THRESHOLD 600

// -------- TB6612FNG Motor Driver --------

// Motor A
const int PWMA = 11;
const int AIN1 = 9;
const int AIN2 = 10;

// Motor B
const int PWMB = 3;
const int BIN1 = 6;
const int BIN2 = 5;

// Standby
const int STBY = 8;


// -------- PID CONSTANTS --------
float Kp = 25;
float Ki = 0;
float Kd = 15;

// -------- SPEED --------
int baseSpeed = 160;
int maxSpeed = 235;


// -------- PID VARIABLES --------
int error = 0;
int lastError = 0;
float integral = 0;

int position = 0;
int setPoint = 3500;   // middle of 8 sensors


void setup()
{
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);

  pinMode(PWMB,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);

  pinMode(STBY,OUTPUT);
  digitalWrite(STBY,HIGH);

  Serial.begin(9600);
}


void loop()
{
  long weightedSum = 0;
  int activeSensors = 0;

  // -------- READ SENSORS --------
  for(int i=0;i<8;i++)
  {
    sensorValue[i] = analogRead(sensorPins[i]);

    if(sensorValue[i] < THRESHOLD) // line detected
    {
      weightedSum += (long)i * 1000;
      activeSensors++;
    }
  }

  // -------- CALCULATE POSITION --------
  if(activeSensors > 0)
  {
    position = weightedSum / activeSensors;
  }

  // -------- ERROR --------
  error = position - setPoint;

  // -------- PID --------
  integral += error;
  int derivative = error - lastError;

  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  lastError = error;

  // -------- MOTOR SPEED --------
  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed,0,maxSpeed);
  rightSpeed = constrain(rightSpeed,0,maxSpeed);

  moveForward(leftSpeed,rightSpeed);
}


// -------- MOTOR FUNCTION --------
void moveForward(int leftSpeed,int rightSpeed)
{

  analogWrite(PWMA,leftSpeed);
  analogWrite(PWMB,rightSpeed);

  // Motor A
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);

  // Motor B
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
}
