#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#include <PID_v1.h>
#include <AccelStepper.h>

#define POT_TEMP A7
#define POT_VEL A6

#define BTN_TEMP A0
#define BTN_MOTOR A1

#define LED_TEMP 10
#define LED_MOTOR 11

#define HEATER 10
#define THERMISTOR A3

#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3799
// the value of the 'other' resistor
#define SERIESRESISTOR 100000    

#include <Thermistor.h>
#include <NTC_Thermistor.h>

#define SENSOR_PIN             A3
#define REFERENCE_RESISTANCE   100000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950

Thermistor* thermistor;

#define MOTOR_A1 6
#define MOTOR_A2 7
#define MOTOR_DIR 8
#define MOTOR_STEP 9
#define MOTOR_ENABLE 11

#define MAX_TEMP 500

LiquidCrystal_I2C lcd(0x27,16,2);
float temperature = 0;
double targetTemp = 0;
double temp = 0;
int pwmVal = 0;
int potMotor = 0;
int motorVel = 0;
int potTemp = 0;
unsigned long deltaTela = 0;
unsigned long deltaPID = 0;

bool toggleMotor = false;
bool toggleHeater = false;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//double Kp=1, Ki=0.05, Kd=0.25;
double Kp=1, Ki=0.01, Kd=0.25;
PID myPID(&temp, &Output, &targetTemp, Kp, Ki, Kd, DIRECT);

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP, MOTOR_DIR); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

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
  Serial.begin(9600);
  //stepper.setEnablePin(MOTOR_ENABLE);
  //stepper.enableOutputs();
  stepper.setMaxSpeed(800);
  stepper.setSpeed(5);

  pinMode(BTN_MOTOR,INPUT_PULLUP);
  pinMode(BTN_TEMP,INPUT_PULLUP);

  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_STEP,OUTPUT);
  pinMode(MOTOR_ENABLE,OUTPUT);

  //liga o motor -> active low
  digitalWrite(MOTOR_ENABLE,LOW);

  pinMode(HEATER,OUTPUT);
  pinMode(LED_MOTOR,OUTPUT);
  pinMode(LED_TEMP,OUTPUT); 
}

int analogAverage(int pin, int reads){
  int acc = 0;
  for(int c =0; c < reads;c++){
    acc += analogRead(pin);
  }
  return acc/reads;
}

void updateScreen(){
  if(millis() - deltaTela > 500){
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(toggleHeater?"1:":"0:");
    lcd.print(int(targetTemp));
    lcd.print("/");
    lcd.print(int(temp));
    lcd.print("  ");
    lcd.setCursor(0,1);                                                                                                                                
    lcd.print("M:");
    lcd.print(toggleMotor?"1:":"0:");
    lcd.print(motorVel);
    lcd.print(" step/s ");
    lcd.print("  ");
   // lcd.print(Output);
    deltaTela = millis();

  }
}

void loop(){
  // int temp_acc = 0;
  // for(int c = 0; c<5 ;c++){
  //   temp =  thermistor->readCelsius();
  //   if(temp < 0){
  //     c--;
  //   }else{
  //     temp_acc += temp;
  //   }
  // }
  //temp = temp_acc/5;
  //temp = temp < 0 ? MAX_TEMP:temp;

  temp =  thermistor->readCelsius();
  targetTemp = map(analogAverage(POT_TEMP,1),0,1024,60,MAX_TEMP);
  motorVel = map(analogAverage(POT_VEL,1),0,1024,16,800);
  updateScreen();
  stepper.setSpeed(-motorVel);
 
  //verificando botao de ativacao do motor
  if(!digitalRead(BTN_MOTOR) && !toggleMotor){
    toggleMotor = true;
    digitalWrite(MOTOR_ENABLE,LOW);
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
    //stepper.runToNewPosition(0);
    //stepper.runToNewPosition(500);
    //stepper.runToNewPosition(100);
    //stepper.runToNewPosition(120);
  }else{
    //stepper.disableOutputs();
  }
}