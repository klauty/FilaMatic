#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>

#define POT_TEMP A7
#define POT_VEL A6

#define BTN_TEMP A0
#define BTN_MOTOR A1

#define HEATER 10

#define THERMISTORNOMINAL 100000      
#define TEMPERATURENOMINAL 25   
#define BCOEFFICIENT 3799
#define SERIESRESISTOR 100000    

#define SENSOR_PIN             A3
#define REFERENCE_RESISTANCE   100000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950

#define MOTOR_A1 6
#define MOTOR_A2 7
#define MOTOR_DIR 8
#define MOTOR_STEP 9
#define MOTOR_ENABLE 11

#define MAX_TEMP 500

Thermistor* thermistor;
LiquidCrystal_I2C lcd(0x27,16,2);
double targetTemp = 0;
double temp = 0;
int motorVel = -700;
int potTemp = 0;
unsigned long deltaTela = 0;
unsigned long deltaPID = 0;

bool toggleMotor = false;
bool toggleHeater = false;

double Setpoint, Input, Output;
double Kp=1, Ki=0.01, Kd=0.25;
PID myPID(&temp, &Output, &targetTemp, Kp, Ki, Kd, DIRECT);

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP, MOTOR_DIR);

void setup()
{
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  thermistor = new NTC_Thermistor(
    SENSOR_PIN,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    B_VALUE
  );

  lcd.init();
  lcd.setBacklight(1);
  stepper.setMaxSpeed(800);

  pinMode(BTN_MOTOR,INPUT_PULLUP);
  pinMode(BTN_TEMP,INPUT_PULLUP);

  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_STEP,OUTPUT);
  
  //liga o motor -> active low
  pinMode(MOTOR_ENABLE,OUTPUT);
  digitalWrite(MOTOR_ENABLE,HIGH);
  pinMode(HEATER,OUTPUT);

}

void updateScreenFull(){
  if(millis() - deltaTela > 300){
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(toggleHeater?"1:":"0:");
    lcd.print(targetTemp,1);
    lcd.print("/");
    lcd.print(temp,1);
    lcd.print("  ");
    lcd.setCursor(0,1);                                                                                                                                
    lcd.print("M:");
    lcd.print(toggleMotor?"1:":"0:");
    lcd.print(motorVel);
    lcd.print(" step/s ");
    lcd.print("  ");
    deltaTela = millis();

  }
}

void updateScreen(){
  if(millis() - deltaTela > 1000){
    lcd.setCursor(0,0);
    lcd.print(targetTemp,0);
    lcd.print("/");
    lcd.print(temp,0);
    deltaTela = millis();

  }
}

void loop(){
  
  temp =  thermistor->readCelsius();
  temp = temp < 0 ? MAX_TEMP:temp;

  targetTemp = map(analogRead(POT_TEMP),0,1024,60,MAX_TEMP);
  motorVel = map(analogRead(POT_VEL),0,1024,16,1000);
  
  stepper.setSpeed(-motorVel);
 
  //verificando botao de ativacao do motor
  if(!digitalRead(BTN_MOTOR) && !toggleMotor){
    toggleMotor = true;
    digitalWrite(MOTOR_ENABLE,LOW);
    lcd.clear();
    delay(500);
  }

  if(!digitalRead(BTN_MOTOR) && toggleMotor){
    toggleMotor = false;
    digitalWrite(MOTOR_ENABLE,HIGH);
    delay(500);
  }

  //verificando botao de ativacao do aquecedor
  if(!digitalRead(BTN_TEMP) && !toggleHeater){
    toggleHeater = true;
    delay(500);
  }

  if(!digitalRead(BTN_TEMP) && toggleHeater){
    toggleHeater = false;
    delay(500);
  }

  if(toggleHeater){
    myPID.Compute();
    analogWrite(HEATER,Output);
  }else{
    analogWrite(HEATER,0);
  }

  if(toggleMotor){
    // ligar motor de passo
    stepper.runSpeed();
    updateScreen();
  }else{
    updateScreenFull();
  }

}