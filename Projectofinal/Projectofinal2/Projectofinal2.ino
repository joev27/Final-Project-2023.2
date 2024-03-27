#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <vector>

#define SERVO_FORWARD_STEP_ANGLE 1
#define SERVO_BACKWARD_STEP_ANGLE -1

struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;
};

std::vector<ServoPins> servoPins =
{
  { Servo(), 13 , "Base", 180},
  { Servo(), 12 , "Shoulder", 180},
  { Servo(), 4 , "Elbow", 180},
  { Servo(), 2 , "Gripper", 90},
};

bool gripperSwitch = false ;

// Motor configuration
int enableMotor1 = 22; 
int motor1Pin1 = 16;
int motor1Pin2 = 17;

int enableMotor2 = 23;
int motor2Pin1 = 18;
int motor2Pin2 = 19;

int enableMotor3 = 32; 
int motor3Pin1 = 33;
int motor3Pin2 = 25;

int enableMotor4 = 14;
int motor4Pin1 = 26;
int motor4Pin2 = 27;

#define MAX_MOTOR_SPEED 100  // Rango de 0 a 255, 255 es la velocidad máxima.

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;

void writeServoValues(int servoIndex, int servoDirection)
{
  int servoPosition = servoPins[servoIndex].servo.read();

  if (servoIndex == 3) 
  { // Control del gripper
    if (gripperSwitch) 
    {
      servoPosition = 180; // Posición cuando el gripper está cerrado
    } 
    else 
    {
      servoPosition = 90; // Posición cuando el gripper está abierto
    }
  } else 
  {
    if (servoDirection == SERVO_FORWARD_STEP_ANGLE) 
    {
      servoPosition += SERVO_FORWARD_STEP_ANGLE;
    } else if (servoDirection == SERVO_BACKWARD_STEP_ANGLE) 
    {
      servoPosition += SERVO_BACKWARD_STEP_ANGLE;
    }
  }

  if (servoPosition > 180) {
    servoPosition = 180;
  } else if (servoPosition < 0) {
    servoPosition = 0;
  }

  servoPins[servoIndex].servo.write(servoPosition);
}

void notify()
{
  bool rx = PS4.Triangle();  // Base       =>  Left stick - eje x
  bool ry = PS4.Cross();  // Shoulder   =>  Left stick  - eje y
  bool ly = PS4.Circle();  // Elbow      =>  Right stick  - eje y
  bool lx = PS4.Square();  // Gripper    =>  Right stick - eje x

  if (rx == 1) {
    writeServoValues(0, SERVO_BACKWARD_STEP_ANGLE);
  } else if (rx == 0) {
    writeServoValues(0, SERVO_FORWARD_STEP_ANGLE);
  }

  if (ry == 1) {
    writeServoValues(1, SERVO_BACKWARD_STEP_ANGLE);
  } else if (ry == 0) {
    writeServoValues(1, SERVO_FORWARD_STEP_ANGLE);
  }

  if (PS4.Share()) {
    gripperSwitch = !gripperSwitch;  // Alternar agarre cerrado / abierto
    writeServoValues(3, 0); // Control del gripper
  }
  // Control de los motores
  if (PS4.Up())
  {
    rotateMotors(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (PS4.Down())
  {
    rotateMotors(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (PS4.Left())
  {
    rotateMotors(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (PS4.Right())
  {
    rotateMotors(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
 
  else 
  {
    stopMotors();
  }

}

void onConnect()
{
  Serial.println("Conectado.");
}

void onDisConnect()
{
  Serial.println("Desconectado.");
}

void rotateMotors(int speed1, int speed2, int speed3, int speed4)
{
  analogWrite(enableMotor1, abs(speed1));
  digitalWrite(motor1Pin1, speed1 > 0 ? HIGH : LOW);
  digitalWrite(motor1Pin2, speed1 < 0 ? HIGH : LOW);

  analogWrite(enableMotor2, abs(speed2));
  digitalWrite(motor2Pin1, speed2 > 0 ? HIGH : LOW);
  digitalWrite(motor2Pin2, speed2 < 0 ? HIGH : LOW);

  analogWrite(enableMotor3, abs(speed3));
  digitalWrite(motor3Pin1, speed3 > 0 ? HIGH : LOW);
  digitalWrite(motor3Pin2, speed3 < 0 ? HIGH : LOW);

  analogWrite(enableMotor4, abs(speed4));
  digitalWrite(motor4Pin1, speed4 > 0 ? HIGH : LOW);
  digitalWrite(motor4Pin2, speed4 < 0 ? HIGH : LOW);
}

void stopMotors()
{
  rotateMotors(0, 0, 0, 0);
}

void setUpPinModes()
{
  for (int i = 0; i < servoPins.size(); i++)
  {
    servoPins[i].servo.attach(servoPins[i].servoPin);
    servoPins[i].servo.write(servoPins[i].initialPosition);
  }

  pinMode(enableMotor1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(enableMotor2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(enableMotor3, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);

  pinMode(enableMotor4, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);

  // Configurar PWM para la velocidad de los motores
  ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableMotor1, PWMSpeedChannel);
  ledcAttachPin(enableMotor2, PWMSpeedChannel);
  ledcAttachPin(enableMotor3, PWMSpeedChannel);
  ledcAttachPin(enableMotor4, PWMSpeedChannel);
  ledcWrite(PWMSpeedChannel, MAX_MOTOR_SPEED);

  rotateMotors(0, 0, 0, 0);
}

void setup()
{
  setUpPinModes();
  Serial.begin(115200);
  PS4.begin();  
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  Serial.println("Listo.");
}

void loop()
{
}