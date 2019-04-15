#include <OneWire.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <stdio.h>

#define SSRPin 3
OneWire  ds(11);  // on pin 11 (a 4.7K resistor is necessary)

float celsius, fahrenheit;
double Setpoint, Input, Output;
double aggKp=100, aggKi=3, aggKd=5;
//double consKp=60, consKi=0.1, consKd=4;
int mode, duty;
boolean active = true;
PID myPID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

unsigned long Timer = 0;     //for backlight
const long Length = 10000;   //10 seconds

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

const int pin_BL = 10; // arduino pin wired to LCD backlight circuit

#define SafeBLon(pin) pinMode(pin, INPUT)
#define SafeBLoff(pin) pinMode(pin, OUTPUT)

void setup()
{
  Serial.begin(9600);
  analogWrite(SSRPin, 0);
  Setpoint = 25;
  myPID.SetMode(AUTOMATIC);
  lcd.begin(16, 2);     // set up the LCD's number of columns and rows:
  mode = 0;             //for changing inputs
}

void loop()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  ds.reset_search();
  if ( !ds.search(addr)) {
    lcd.clear();
    lcd.println(" DS18 not found ");
    ds.reset_search();
    delay(250);
    return;
  }
  
//  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
//    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      lcd.clear();
      lcd.println("CRC is not valid!");
      return;
  }
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //lcd.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 0);        // start conversion, without parasite power on at the end
  
  delay(100);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

 // Serial.print("  Data = ");
 // Serial.print(present, HEX);
 // Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
 //   Serial.print(data[i], HEX);
 //   Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  
  celsius = (float)raw / 16.0;

  duty = Output * 100;
  duty = duty / 255;
  
  lcd.setCursor(0,0);
  lcd.print(Setpoint,1);
  lcd.print("c");
  lcd.print("  now:");
  lcd.print(celsius,1);
  lcd.println("c");
  lcd.setCursor(0,1);
  char buffer [15];
  sprintf(buffer, "HEATER %3d / 100", duty);
  lcd.print(buffer); 


  Serial.print(Setpoint);
  Serial.print("c");
  Serial.print(" -> ");
  Serial.print(celsius);
  Serial.println("c");
  Serial.print("Duty = ");
  Serial.print(duty);
  Serial.println(" %  ");


 
  
  Input = celsius;

// PID SECTION

  //double gap = (Setpoint-Input); //distance away from setpoint
  //if (gap > 0){
  //  if (gap < 0.5)
   // {  //we're close to setpoint, use conservative tuning parameters
   //   myPID.SetTunings(consKp, consKi, consKd);
   // }
   // else
   // {
   //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
   // }
 // }

  myPID.Compute();
  analogWrite(SSRPin, Output);

// KEYPAD SECTION

lcd_key = read_LCD_buttons();  // read the buttons

switch (lcd_key)               // depending on which button was pushed, we perform an action
 {
   case btnRIGHT:
     {
     //lcd.println("RIGHT ");
     active = 1;
     break;
     }
   case btnLEFT:
     {
     //lcd.print("LEFT   ");
     active = 1;
     break;
     }
   case btnUP:
     {
     //lcd.print("UP    ");
     Setpoint = Setpoint + 0.5;
     active = 1;
     break;
     }
   case btnDOWN:
     {
     //lcd.print("DOWN  ");
     Setpoint = Setpoint - 0.5;
     active = 1;
     break;
     }
   case btnSELECT:
     {
     //lcd.print("SELECT");
     active = 1;
     break;
     }
     case btnNONE:
     {
     //lcd.print("NONE  ");
     active = 0;
     break;
     }
 }

}

// read the buttons
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor 
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE;
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 250)  return btnUP; 
 if (adc_key_in < 450)  return btnDOWN; 
 if (adc_key_in < 650)  return btnLEFT; 
 if (adc_key_in < 850)  return btnSELECT;
}


