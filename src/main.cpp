#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define POT_TEMP A7
#define POT_VEL A6

#define BTN_TEMP A0
#define BTN_MOTOR A1

#define LED_TEMP 10
#define LED_MOTOR 11

#define HEATER 10
#define THERMISTOR A3

#define MOTOR_A1 6
#define MOTOR_A2 7
#define MOTOR_DIR 9
#define MOTOR_STEP 8


#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3799
// the value of the 'other' resistor
#define SERIESRESISTOR 100000    


LiquidCrystal_I2C lcd(0x27,16,2);
float temperature = 0;
int targetTemp = 0;
int temp = 0;
int pwmVal = 0;
int potMotor = 0;
int potTemp = 0;
unsigned long deltaTela = 0;

bool toggleMotor = false;
bool toggleHeater = false;


void setup()
{
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
    //delay(10);
  }
  return acc/reads;
}

int getTemp(float read){
  float temperature;
  temperature = (SERIESRESISTOR / (1023 / read -1)) / THERMISTORNOMINAL;     // (R/Ro)
  Serial.println(1023 / read );
  temperature = log(temperature);                  // ln(R/Ro)
  temperature /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                         // convert absolute temp to C
  return temperature-5;
}

void updateScreen(){
  if(millis() - deltaTela > 150){
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(toggleHeater?"1:":"0:");
    lcd.print(targetTemp);
    lcd.print("/");
    lcd.print(temp);
    lcd.print("  ");
    lcd.setCursor(0,1);                                                                                                                                
    // lcd.print("M:");
    // lcd.print(toggleMotor?"1:":"0:");
    // lcd.print(map(analogAverage(POT_VEL,20),0,1024,50,270));
    // lcd.print(" RPM");
    // lcd.print("  ");
    lcd.print(pwmVal);
    lcd.print("  ");
    targetTemp = map(analogAverage(POT_TEMP,20),0,1024,60,260);
    deltaTela = millis();

    //Serial.println();
  }
}

void loop()
{
  temp = getTemp(analogAverage(THERMISTOR,20));
  updateScreen();
 
  //verificando botao de ativacao do motor
  if(!digitalRead(BTN_MOTOR) && !toggleMotor){
    toggleMotor = true;
    //analogWrite(LED_MOTOR,100);
    delay(500);
  }
  if(!digitalRead(BTN_MOTOR) && toggleMotor){
    toggleMotor = false;
    //analogWrite(LED_MOTOR,0);
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
     if(temp < targetTemp){
      pwmVal += 1;
      pwmVal = pwmVal>100?100:pwmVal;
     }else{
      pwmVal -=1;
      pwmVal = pwmVal<0?0:pwmVal;
    }
    analogWrite(HEATER,pwmVal);
    //analogWrite(HEATER,map(analogRead(POT_TEMP),0,1024,0,255));
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