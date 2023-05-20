#include <SoftwareSerial.h> //INCLUSÃO DE BIBLIOTECA

const int pinoRX = 13; //PINO DIGITAL 2 (RX)
const int pinoTX = 12; //PINO DIGITAL 3 (TX)
//const int pinoLed = 12; //PINO DIGITAL UTILIZADO PELO LED
int dadoBluetooth = 0; //VARIÁVEL QUE ARMAZENA O VALOR ENVIADO PELO BLUETOOTH
 
SoftwareSerial bluetooth(pinoRX, pinoTX); //PINOS QUE EMULAM A SERIAL, ONDE
//O PINO 2 É O RX E O PINO 3 É O TX
 
#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 10
int MOTOR_SPEED = 255;

// //Right motor
// int enableRightMotor=6;
// int rightMotorPin1=8;
// int rightMotorPin2=7;

// //Left motor
// int enableLeftMotor=5;
// int leftMotorPin1=4;
// int leftMotorPin2=3;


//Right motor
int enableRightMotor=5;
int rightMotorPin1=4;
int rightMotorPin2=3;

//Left motor
int enableLeftMotor=6;
int leftMotorPin1=8;
int leftMotorPin2=7;


int lastRightIRSensorValue = 1;
int lastLeftIRSensorValue = 1;


int lineMode = 0;
int bluetoothMode = 1;
int currentMode = bluetoothMode;

void setup()
{
  //The problem with TT gear motors is that, at very low pwm value it does not even rotate.
  //If we increase the PWM value then it rotates faster and our robot is not controlled in that speed and goes out of line.
  //For that we need to increase the frequency of analogWrite.
  //Below line is important to change the frequency of PWM signal on pin D5 and D6
  //Because of this, motor runs in controlled manner (lower speed) at high PWM value.
  //This sets frequecny as 7812.5 hz.
  TCCR0B = TCCR0B & B11111000 | B00000010 ;
  TCCR0B = TCCR0B & B11111000 | B00000101; // for PWM frequency of 61.04 Hz
  
  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  rotateMotor(0,0);   
  Serial.begin(9600);
  bluetooth.begin(9600); //INICIALIZA A SERIAL DO BLUETOOTH
  bluetooth.print("$"); //IMPRIME O CARACTERE
  bluetooth.print("$"); //IMPRIME O CARACTERE
  bluetooth.print("$"); //IMPRIME O CARACTERE
  delay(100); //INTERVALO DE 100 MILISSEGUNDOS
}


void loop()
{
  if(bluetooth.available()){ //SE O BLUETOOTH ESTIVER HABILITADO, FAZ
       dadoBluetooth = bluetooth.read(); //VARIÁVEL RECEBE O VALOR ENVIADO PELO BLUETOOTH
       if(dadoBluetooth != 83){
         Serial.println( char(dadoBluetooth));
         if(char(dadoBluetooth) == 'W'){
           currentMode = bluetoothMode;
           MOTOR_SPEED = 255;
         }
         if(char(dadoBluetooth) == 'w'){
           currentMode = lineMode;
           MOTOR_SPEED = 150;
         }
         if(char(dadoBluetooth) == 'F'){
           rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
         }
         if(char(dadoBluetooth) == 'I'){
           rotateMotor(0, MOTOR_SPEED);
         }
         if(char(dadoBluetooth) == 'G'){
           rotateMotor(MOTOR_SPEED, 0);
         }
         if(char(dadoBluetooth) == 'L'){
           rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
         }
         if(char(dadoBluetooth) == 'R'){
           rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
         }
         if(char(dadoBluetooth) == 'J'){
           rotateMotor(0, -MOTOR_SPEED);
         }
         if(char(dadoBluetooth) == 'H'){
           rotateMotor(-MOTOR_SPEED, 0);
         }
         if(char(dadoBluetooth) == 'B'){
           rotateMotor(-MOTOR_SPEED, -MOTOR_SPEED);
         }
       }
       else {
         rotateMotor(0, 0);
       }
     
  }

  if (currentMode == lineMode){
    int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
    int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

    Serial.print("L:");
    Serial.print(leftIRSensorValue);
    Serial.print("R");
    Serial.println(rightIRSensorValue);
//rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    //If none of the sensors detects black line, then go straight
    if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
    {
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
      if((rightIRSensorValue != lastRightIRSensorValue) || (leftIRSensorValue != lastLeftIRSensorValue)){
        lastRightIRSensorValue = rightIRSensorValue;
        lastLeftIRSensorValue = leftIRSensorValue;
      }
    }
    //If right sensor detects black line, then turn right
    else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
    {
      
      
        rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
      if((rightIRSensorValue != lastRightIRSensorValue) || (leftIRSensorValue != lastLeftIRSensorValue)){
        lastRightIRSensorValue = rightIRSensorValue;
        lastLeftIRSensorValue = leftIRSensorValue;
      }
    }
    //If left sensor detects black line, then turn left  
    else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
    {
      
      
        rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
        if((rightIRSensorValue != lastRightIRSensorValue) || (leftIRSensorValue != lastLeftIRSensorValue)){
        lastRightIRSensorValue = rightIRSensorValue;
        lastLeftIRSensorValue = leftIRSensorValue;
      }
    } 
    //If both the sensors detect black line, then stop 
    else 
    {
      rotateMotor(0, 0);
      if((rightIRSensorValue != lastRightIRSensorValue) || (leftIRSensorValue != lastLeftIRSensorValue)){
        lastRightIRSensorValue = rightIRSensorValue;
        lastLeftIRSensorValue = leftIRSensorValue;
      }
    }

    //Serial.println(leftIRSensorValue);
    //Serial.println(rightIRSensorValue);
  }
  else{

  }
  
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{

  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
   analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed)); 
}