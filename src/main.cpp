#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "PID_utils.h"
#include <PID_v1.h>

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
#define MOTOR_DIR 9
#define MOTOR_STEP 8

#define MAX_TEMP 500

LiquidCrystal_I2C lcd(0x27,16,2);
float temperature = 0;
double targetTemp = 0;
double temp = 0;
int pwmVal = 0;
int potMotor = 0;
int potTemp = 0;
unsigned long deltaTela = 0;
unsigned long deltaPID = 0;

bool toggleMotor = false;
bool toggleHeater = false;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//double Kp=1, Ki=0.05, Kd=0.25;
double Kp=2, Ki=5, Kd=1;
PID myPID(&temp, &Output, &targetTemp, Kp, Ki, Kd, DIRECT);

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
  pinMode(BTN_MOTOR,INPUT_PULLUP);
  pinMode(BTN_TEMP,INPUT_PULLUP);

  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_STEP,OUTPUT);

  pinMode(HEATER,OUTPUT);
  pinMode(LED_MOTOR,OUTPUT);
  pinMode(LED_TEMP,OUTPUT); 
}

int analogAverage(int pin,int reads){
  int acc = 0;
  for(int c =0; c< reads;c++){
    acc += analogRead(pin);
  }
  return acc/reads;
}

int getTemp(double read){
  double temperature;
  temperature = (SERIESRESISTOR / (1023 / read -1)) / THERMISTORNOMINAL;     // (R/Ro)
  Serial.println(1023 / read );
  temperature = log(temperature);                  // ln(R/Ro)
  temperature /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                         // convert absolute temp to C
  return temperature < 0 ? MAX_TEMP:temperature;
}

void updateScreen(){
  if(millis() - deltaTela > 200){
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(toggleHeater?"1:":"0:");
    lcd.print(int(targetTemp));
    lcd.print("/");
    lcd.print(int(temp));
    lcd.print("  ");
    lcd.setCursor(0,1);                                                                                                                                
    // lcd.print("M:");
    // lcd.print(toggleMotor?"1:":"0:");
    // lcd.print(map(analogAverage(POT_VEL,20),0,1024,50,270));
    // lcd.print(" RPM");
    // lcd.print("  ");
    lcd.print(Output);
    lcd.print(":");
    lcd.print(PID_d);
    lcd.print("  ");
    deltaTela = millis();

  }
}

void loop()
{
  temp =  thermistor->readCelsius();//getTemp(analogAverage(THERMISTOR,1));
  targetTemp = map(analogAverage(POT_TEMP,1),0,1024,60,MAX_TEMP);
  updateScreen();
 
  //verificando botao de ativacao do motor
  if(!digitalRead(BTN_MOTOR) && !toggleMotor){
    toggleMotor = true;
    delay(500);
  }
  if(!digitalRead(BTN_MOTOR) && toggleMotor){
    toggleMotor = false;
    delay(500);
  }

  //verificando botao de ativacao do aquecedor
  if(!digitalRead(BTN_TEMP) && !toggleHeater){
    toggleHeater = true;
    //analogWrite(LED_TEMP,255);
    delay(500);
  }
  if(!digitalRead(BTN_TEMP) && toggleHeater){
    toggleHeater = false;
    //analogWrite(LED_TEMP,0);
    delay(500);
  }

  if(toggleHeater){
    //implementar a logica do PID aqui
    if(millis() - deltaPID > 500){
      //pwmVal = getHeaterPower(temp,targetTemp);
      //analogWrite(HEATER,pwmVal);
      deltaPID = millis();
    }
    //analogWrite(HEATER,map(analogRead(POT_TEMP),0,1024,0,255));
    myPID.Compute();
    analogWrite(HEATER, Output);
  }else{
    analogWrite(HEATER,0);
    pwmVal = 0;
  }

  if(toggleMotor){
    // ligar motor de passo
  }else{
    //desligar motor
  }
}