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
