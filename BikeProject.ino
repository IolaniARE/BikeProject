#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_7segment L_volts_disp = Adafruit_7segment();
Adafruit_7segment L_watts_disp = Adafruit_7segment();
Adafruit_7segment L_whr_disp = Adafruit_7segment();
Adafruit_7segment R_volts_disp = Adafruit_7segment();
Adafruit_7segment R_watts_disp = Adafruit_7segment();
Adafruit_7segment R_whr_disp = Adafruit_7segment();
// #include <avr/pgmspace.h>
// analog inputs
#define L_voltage_pin 0                
#define L_current_pin 1
#define R_voltage_pin 2                
#define R_current_pin 3



// digital outputs to relays,  active LOW
#define L_relay_1 2  // Left Relay Board K1
#define L_relay_2 3  // Left Relay Board K2
#define L_relay_3 4  // Left Relay Board K3
#define L_relay_4 5  // Left Relay Board K4
#define R_relay_1 6  // Right Relay Board K1
#define R_relay_2 7  // Right Relay Board K2
#define R_relay_3 8  // Right Relay Board K3
#define R_relay_4 9  // Right Relay Board K4
#define L_gen_relay 10  // Left Relay Board K8
#define R_gen_relay 11  // Right Relay Board K8
#define L_winner 0  // Left Relay Board K5
#define R_winner 1  // Right Relay Board K5

#define aref_voltage 5000           // ADC reference 5V, ~5.04 for usb power, 4.99 for external power
#define L_volt_ratio 99             // 10.1:1 divider
#define R_volt_ratio 98            //  10.2:1 divider
#define sensitivity 66              // ACS712-30 current sensor  66mV/A
#define ECHO_TO_SERIAL 1


boolean game_start = false;
boolean game_over = false;
int blinkState = LOW;
unsigned long currentMillis;
long time_start = 0;
long timer = 0;
long time = 0;
long deltatime = 0;
long previousMillis = 0;
long interval = 200;           
float L_mAh, R_mAh;
float L_Wh = 0;
float R_Wh = 0;
float L_voltage, R_voltage;
float L_gen_volts, R_gen_volts;
float L_current, R_current;
float L_gen_amps, R_gen_amps;
float L_power, R_power;
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x20 for a 16 chars and 2 line display

uint16_t counter = 0;

//**************************************setup****************************************

void setup() {

    L_volts_disp.begin(0x070);  // green
    L_watts_disp.begin(0x071);  // red
    L_whr_disp.begin(0x072);    // yellow
    R_volts_disp.begin(0x073);  // green
    R_watts_disp.begin(0x074);  // red
    R_whr_disp.begin(0x075);    // yellow
    
 //   LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 4x20 display
    lcd.init();                        // initialize the lcd
    lcd.backlight();
    
//    Serial.begin(9600);  do not initialize if digital pins 0 or 1 are being used as outputs
    
    // initialize relay driver outputs active low
    pinMode(L_relay_1, OUTPUT);
    digitalWrite(L_relay_1, HIGH);
    pinMode(L_relay_2, OUTPUT);
    digitalWrite(L_relay_2, HIGH);
    pinMode(L_relay_3, OUTPUT);
    digitalWrite(L_relay_3, HIGH);
    pinMode(L_relay_4, OUTPUT);
    digitalWrite(L_relay_4, HIGH);
    pinMode(L_gen_relay, OUTPUT);
    digitalWrite(L_gen_relay, HIGH);
    pinMode(L_winner, OUTPUT);
    digitalWrite(L_winner, HIGH);
    
    pinMode(R_relay_1, OUTPUT);
    digitalWrite(R_relay_1, HIGH);
    pinMode(R_relay_2, OUTPUT);
    digitalWrite(R_relay_2, HIGH);
    pinMode(R_relay_3, OUTPUT);
    digitalWrite(R_relay_3, HIGH);
    pinMode(R_relay_4, OUTPUT);
    digitalWrite(R_relay_4, HIGH);
    pinMode(R_gen_relay, OUTPUT);
    digitalWrite(R_gen_relay, HIGH);
    pinMode(R_winner, OUTPUT);
    digitalWrite(R_winner, HIGH);
    
    lcd.print("PowerMeter v2.0"); 
    lcd.setCursor(0,1);
    lcd.print("(c)2014 IOLANI");
    delay(200);
    lcd.clear();
    lcd.setCursor(7,0);
    lcd.print("V");
    lcd.setCursor(7,1);
    lcd.print("A");
    lcd.setCursor(7,2);
    lcd.print("W");
    lcd.setCursor(7,3);
    lcd.print("Wh");
    
    L_mAh = 0;
    R_mAh = 0;
    L_Wh = 0;
    R_Wh = 0;
    
//    load_test();  // Test Code - cycle all relays one at a time for 1 second, test displays
    
}
//**************************************************end setup***********************************************


//**************************************************loop****************************************************

void loop() {
    while (!game_start) {
      get_voltage();
      get_current();
      get_power();
      currentMillis = millis();
      if(currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;
        display_R_VP();
        display_L_VP();
        disp_R_Wh();
        disp_L_Wh();
        display_LCD();
        }
        time_start = millis();
      while((R_gen_volts >= 13.0  && R_gen_volts <= 29.0) && (L_gen_volts >= 13.0  && L_gen_volts <= 29.0)  && !game_start) {
        get_voltage();
        get_current();
        get_power();
        currentMillis = millis();
        if(currentMillis - previousMillis > interval) {
          previousMillis = currentMillis;
          display_R_VP();
          display_L_VP();
          disp_R_Wh();
          disp_L_Wh();
          display_LCD();
        }
        timer = millis()-time_start;
        if(timer > 7000) {
          game_start = true;
        }
      }
    }   
      
// committed to game start here

while(game_start) {
  get_voltage();
  get_current();
  get_power();
  
  if(R_gen_volts <= 29.0) { 
    digitalWrite(R_gen_relay, LOW);  // turn on main relay (active low)
  }
  else {
    digitalWrite(R_gen_relay, HIGH);  // turn off main relay (active low)
  }
  
  if(L_gen_volts <= 29.0) { 
    digitalWrite(L_gen_relay, LOW);  // turn on main relay (active low)
  }
  else {
    digitalWrite(L_gen_relay, HIGH);  // turn off main relay (active low)
  }

//  display_L_VP();  // display volts, current, power on left side 7-segment displays
  deltatime = millis() - time;
  time = millis();
  currentMillis = millis();  
  // count Ah/Wh after game has started    
  R_mAh = milliamphourscalc(deltatime, R_gen_amps) + R_mAh;
  R_Wh = watthourscalc(deltatime, R_power) + R_Wh; 
  L_mAh = milliamphourscalc(deltatime, L_gen_amps) + L_mAh;
  L_Wh = watthourscalc(deltatime, L_power) + L_Wh; 
  
  if(currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;
        display_R_VP();
        display_L_VP();
        disp_R_Wh();  // display Whr
        disp_L_Wh();  // display Whr
        display_LCD();
        }
  digitalWrite(R_relay_1, LOW);  //turn-on right level 1 load
  digitalWrite(L_relay_1, LOW);  //turn-on left level 1 load
    
    if(R_Wh >= 0.25) {
      digitalWrite(R_relay_2, LOW);
    }
    if(L_Wh >= 0.25) {
      digitalWrite(L_relay_2, LOW);
    }
    
    
    if(R_Wh >= 0.75) {
      digitalWrite(R_relay_3, LOW);
    }
    if(L_Wh >= 0.75) {
      digitalWrite(L_relay_3, LOW);
    }
    
    
    if(R_Wh >= 1.50) {
      digitalWrite(R_relay_4, LOW);
    }
    if(L_Wh >= 1.50) {
      digitalWrite(L_relay_4, LOW);
    }
    
    if((R_Wh >= 2.00) && (L_Wh <= 2.00)) {  //right player wins
      game_over = true;
      digitalWrite(R_winner, LOW);
      digitalWrite(R_relay_4, HIGH);
      digitalWrite(L_relay_4, HIGH);
      delay(2000);
      digitalWrite(R_relay_3, HIGH);
      digitalWrite(L_relay_3, HIGH);
      delay(2000);
      digitalWrite(R_relay_2, HIGH);
      digitalWrite(L_relay_2, HIGH);
      delay(2000);
      digitalWrite(R_relay_1, HIGH);
      digitalWrite(L_relay_1, HIGH);
    // dispense candy (drive one of the unused relays with low voltage (3V) DC/DC converter
      digitalWrite(R_winner, LOW);
      while(1) {
        get_voltage();
        get_current();
        get_power();
        currentMillis = millis();
        if(currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;
        display_R_VP();
        display_LCD();
        }
      }
    }
    
    
    if((L_Wh >= 2.00) && (R_Wh <= 2.00)) {  //right player wins
      game_over = true;
      digitalWrite(L_winner, LOW);
      digitalWrite(R_relay_4, HIGH);
      digitalWrite(L_relay_4, HIGH);
      delay(2000);
      digitalWrite(R_relay_3, HIGH);
      digitalWrite(L_relay_3, HIGH);
      delay(2000);
      digitalWrite(R_relay_2, HIGH);
      digitalWrite(L_relay_2, HIGH);
      delay(2000);
      digitalWrite(R_relay_1, HIGH);
      digitalWrite(L_relay_1, HIGH);
    // dispense candy (drive one of the unused relays with low voltage (3V) DC/DC converter
      digitalWrite(L_winner, HIGH);
      while(1) {
        get_voltage();
        get_current();
        get_power();
        currentMillis = millis();
        if(currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;
        display_R_VP();
        display_LCD();
        }
      }
    }  
  }  
}
    

//*****************************************************************end loop**************************************************

// functions are ordered by sequential use in void loop()

//*******************************************************************get_voltage************************************************
// averages every 10 readings from a specified analog input pin
void get_voltage() {        
    L_voltage = avgByTen(L_voltage_pin);  //raw analog input data
    R_voltage = avgByTen(R_voltage_pin);  // raw analog input data
    L_gen_volts = L_voltsCalc(L_voltage);  // scaled generator input voltage (10.1 : 1 divider)
    R_gen_volts = R_voltsCalc(R_voltage);  // scaled generator input voltage (10.2 : 1 divider)
}
//*****************************************************************end get_voltage************************************************

//*******************************************************************get_current************************************************
// averages every 10 readings from a specified analog input pin
void get_current() {        
    L_current = avgByTen(L_current_pin);
    R_current = avgByTen(R_current_pin);
    L_gen_amps = currentCalc(L_current);
    R_gen_amps = currentCalc(R_current); 
}
//*****************************************************************end get_current************************************************

//*******************************************************************get_power************************************************
void get_power() {        
    L_power = powercalc(L_gen_volts, L_gen_amps);
    R_power = powercalc(R_gen_volts, R_gen_amps); 
}
//*****************************************************************end get_power************************************************

void display_L_VP() {
  
 L_volts_disp.printFloat(L_gen_volts, 2, 10);
 L_volts_disp.writeDigitRaw(2, 0x02);  // print colon for decimal point
 L_volts_disp.writeDisplay();
 
 L_watts_disp.print(int(L_power));
 L_watts_disp.writeDisplay();

}

void display_R_VP() {
 R_volts_disp.printFloat(R_gen_volts, 2, 10);
 R_volts_disp.writeDigitRaw(2, 0x02);  // print colon for decimal point
 R_volts_disp.writeDisplay();
 
 R_watts_disp.print(int(R_power));
 R_watts_disp.writeDisplay();
 
}



//*******************************************************************averaging************************************************
// averages every 10 readings from a specified analog input pin
float avgByTen(uint8_t pin) {        
    int reading = 0;
    for (int i=0; i<10; i++){
        analogRead(pin); //arduino alalog in pin
        reading += analogRead(pin);
    }
    return reading / 10;
}
//*****************************************************************end averaging************************************************


//********************************************************************  L_generator voltage calculation************************************************
// computes scaled voltage based on 10.1:1 divider
float L_voltsCalc (float voltageReading) {     
  return ((voltageReading * aref_voltage / 1024)/ L_volt_ratio); //pack voltage in mV ~10.1:1 voltage divider
}
//*******************************************************************end L_generator voltage calculation*******************************************

//********************************************************************  R_generator voltage calculation************************************************
// computes scaled voltage based on 10.2:1 divider
float R_voltsCalc (float voltageReading) {     
  return ((voltageReading * aref_voltage / 1024)/ R_volt_ratio); //pack voltage in mV ~10.2:1 voltage divider
}
//*******************************************************************end R_generator voltage calculation*******************************************

//*********************************************************************current calculation*****************************************
//calculates current
float currentCalc(float currentReading) {     
 return ((currentReading * aref_voltage / 1024) - 2495) / 66;  // current in Amps (ACS712-30A, 66mV/A) 
}
//**********************************************************************end current calculation*************************************


//*****************************************************************power**********************************************************
//calculates power
float powercalc(float battery, float current) {
  return ((battery * current));
}
//*****************************************************************end power********************************************************

//****************************************************************amphours**********************************************************
//calculates milliamphours
float milliamphourscalc(long deltatime, float current){ 
  return ((deltatime * current *1000)/3600000);
}
//***************************************************************end amphours*******************************************************


//***************************************************************watthours**********************************************************
//calculates watthours
float watthourscalc(long deltatime, float power){
  return ((deltatime * power)/3600000);
}
//***************************************************************end watthours******************************************************


//***************************************************************display L_Wh**********************************************************
void disp_L_Wh() {
  if (L_Wh>=0.00 && L_Wh<=0.09) {
    L_whr_disp.printFloat(L_Wh, 2, 10);  // void Adafruit_7segment::printFloat(double n, uint8_t fracDigits, uint8_t base) 
    L_whr_disp.writeDisplay();
    L_whr_disp.writeDigitNum(1, 0);  // leading zero in ones position
    L_whr_disp.writeDisplay();
    L_whr_disp.writeDigitNum(3, 0);  //leading zero in tenths position
    L_whr_disp.writeDisplay();
    L_whr_disp.writeDigitRaw(2, 0x02);   // center colon - cover top dot for decimal point
    L_whr_disp.writeDisplay();
    
 
    }
    
  if (L_Wh>=0.10 && L_Wh<=0.99) {
    L_whr_disp.printFloat(L_Wh, 2, 10);
    L_whr_disp.writeDisplay();
    L_whr_disp.writeDigitNum(1, 0);  // leading zero in ones position
    L_whr_disp.writeDisplay();
    L_whr_disp.writeDigitRaw(2, 0x02);  // center colon - cover top dot for decimal point
    L_whr_disp.writeDisplay();
    }
    
  if (L_Wh>0.99) {
    L_whr_disp.printFloat(L_Wh, 2, 10);
    L_whr_disp.writeDisplay();
    L_whr_disp.writeDigitRaw(2, 0x02);
    L_whr_disp.writeDisplay();
    }
}

//***************************************************************end display L_Wh**********************************************************


//***************************************************************display R_Wh**********************************************************
void disp_R_Wh() {
  if (R_Wh>=0.00 && R_Wh<=0.09) {
    R_whr_disp.printFloat(R_Wh, 2, 10);  // void Adafruit_7segment::printFloat(double n, uint8_t fracDigits, uint8_t base) 
    R_whr_disp.writeDisplay();
    R_whr_disp.writeDigitNum(1, 0);  // leading zero in ones position
    R_whr_disp.writeDisplay();
    R_whr_disp.writeDigitNum(3, 0);  //leading zero in tenths position
    R_whr_disp.writeDisplay();
    R_whr_disp.writeDigitRaw(2, 0x02);   // center colon - cover top dot for decimal point
    R_whr_disp.writeDisplay();
    
 
    }
    
  if (R_Wh>=0.10 && R_Wh<=0.99) {
    R_whr_disp.printFloat(R_Wh, 2, 10);
    R_whr_disp.writeDisplay();
    R_whr_disp.writeDigitNum(1, 0);  // leading zero in ones position
    R_whr_disp.writeDisplay();
    R_whr_disp.writeDigitRaw(2, 0x02);  // center colon - cover top dot for decimal point
    R_whr_disp.writeDisplay();
    }
    
  if (R_Wh>0.99) {
    R_whr_disp.printFloat(R_Wh, 2, 10);
    R_whr_disp.writeDisplay();
    R_whr_disp.writeDigitRaw(2, 0x02);
    R_whr_disp.writeDisplay();
    }
}

//***************************************************************end display R_Wh**********************************************************



//****************************************************************display_LCD**********************************************************

void display_LCD() {
    lcd.backlight(); 
    lcd.setCursor(0,0);
    lcd.print(L_gen_volts,1);
    lcd.setCursor(0,1);
    lcd.print(L_gen_amps,1);
    lcd.setCursor(0,2);
    lcd.print(int (L_power));
    lcd.setCursor(0,3);
    lcd.print(L_Wh);
    
    lcd.setCursor(11,0);
    lcd.print(R_gen_volts,1);
    lcd.setCursor(11,1);
    lcd.print(R_gen_amps,1);
    lcd.setCursor(11,2);
    lcd.print(int (R_power));
    lcd.setCursor(11,3);
    lcd.print(R_Wh);
    
    
}
//***************************************************************end display_LCD*******************************************************

//*******************************************************load_test*********************************************************
void load_test() {
// Test Code - cycle all relays one at a time for 1 second, test displays
  delay(2000);  
  digitalWrite(L_gen_relay, LOW);  // turn on main relay (active low)
  get_voltage();
  get_current();
  get_power();
  display_L_VP();
  currentMillis = millis();
  deltatime = millis() - time;
  time = millis();
  // count Ah/Wh after game has started    
  L_mAh = milliamphourscalc(deltatime, L_gen_amps) + L_mAh;
  L_Wh = watthourscalc(deltatime, L_power) + L_Wh; 
  display_L_VP();  // display volts, current, power on left side 7-segment displays
  digitalWrite(L_relay_1, LOW);  //turn-on level 1 load
  delay(1000);
  digitalWrite(L_relay_1, HIGH);
  delay(250);
  digitalWrite(L_relay_2, LOW);
  delay(1000);
  digitalWrite(L_relay_2, HIGH);
  delay(250);
  digitalWrite(L_relay_3, LOW);
  delay(1000);
  digitalWrite(L_relay_3, HIGH);
  delay(250);
  digitalWrite(L_relay_4, LOW);
  delay(1000);
  digitalWrite(L_relay_4, HIGH);
  delay(250);
  digitalWrite(L_winner, LOW);
  delay(1000);
  digitalWrite(L_winner, HIGH);
  delay(250);
  
  
    get_voltage();
    get_current();
    get_power();
    display_L_VP();
    display_R_VP();    
}
//*******************************************************end load_test*********************************************************








//*******************************************************display trip time*********************************************************
// prints the time to the lcd screen
// format: 00:00:00 (hours:minutes:seconds)
void print_time() {
    int sec = (millis())/1000;
    int second = sec%60;
    int minut = (sec/60)%60;
    int hour = sec/3600;
    lcd.setCursor(12, 0);
    if(hour > 0) {
        lcd.print(hour);
        lcd.print(":");
    }
    if(minut < 10)
    lcd.print("0");
    lcd.print(minut);
    lcd.print(":");
    if(second < 10)
    lcd.print("0");
    lcd.print(second); // print the number of seconds since reset:
}
//******************************************************end display trip time*******************************************************



