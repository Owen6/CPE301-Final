// CPE 301 - Semester Project - December, 2021
// Team: Owen Boxx, Cody Jackson, Estephan Salcedo

//=======DOCUMENTATION========//
//                            //
// BUTTON         = PIN 2     //
// SERVOdata      = PIN 3     //
//                            //
// LEDS>                      //
//  BLUE          = PIN 10    //
//  RED           = PIN 11    //
//  GREEN         = PIN 12    //
//  YELLOW        = PIN 13    //
// <                          //
//                            //
// JOYSTICK Ydata = A1        //
//                            //
// WATERdata      = A5        //
//                            //
// FAN            = A7        //
//                            //
// TEMP/HUMdata   = 22        //
//                            //
//============================//

#include "DHT.h"
#include <LiquidCrystal.h>
#include <Servo.h>
#include <TimeLib.h>

#define DHTPIN 22     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11 
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

volatile unsigned char *pinB =     (unsigned char *) 0x23;
volatile unsigned char *portDDRB = (unsigned char *) 0x24;
volatile unsigned char *portB =    (unsigned char *) 0x25;
volatile unsigned char *pinH =     (unsigned char *) 0x100;
volatile unsigned char *portDDRH = (unsigned char *) 0x101;
volatile unsigned char *portH =    (unsigned char *) 0x102;
volatile unsigned char *pinE =     (unsigned char *) 0x2C;
volatile unsigned char *portDDRE = (unsigned char *) 0x2D;
volatile unsigned char *portE =    (unsigned char *) 0x2E;

Servo myservo;
LiquidCrystal lcd(8,9,4,5,6,7);

//starting state is off
int state = 0;

int stateButton;
int previous = LOW;
long debounce = 200;
long time = 0;
int resval = 0;
int waterThresh = 200;
float tempThresh = 70;
int respin = A5;
uint8_t a_ref = DEFAULT;
DHT dht(22, DHT11);
int VRy = A1; // Joystick Y pin
int vent_angle = 0; // Current Angle of the vent
int yPosition = 0;
int mapY = 0; // Joystick Y pos
int fan = A7; // Fan pin

void dWrite(uint8_t pin, uint8_t val);
void aRef(uint8_t mode);
int aRead(uint8_t pin);
int aWrite(uint8_t pin, int val);
void displayLCD(float h, float t, int s);
void leds(int s);
void buttonPress();

void setup(){

  Serial.begin(9600);
  dht.begin(); 
  lcd.begin(16, 2);       // set up the LCD's number of columns and rows:
   
  
  myservo.attach(3); // attach the signal pin of servo to pin3 of arduino
/*
  // set PB7-PB4 to output           //Register B
  *portDDRB |= 0xF0;
  // Initialize PB7-4 to low
  *portB &= 0x0F;

  //set PH6-5 to output             //Register H
  *portDDRH |= 0x60;
  //set PH4 to input 
  *portDDRH &= 0xEF;
  // Initialize PH6-5 to low
  *portH &= 0x9F;
  //sets PH5 to high
  *portH |= 0x20;
   // sets pullup resistor for PH4
  *portH |= 0x10;

   // set PE4 to output            //Register E
  *portDDRE |= 0x10;
  // set PE5 to output
  *portDDRE |= 0x20;
  // Initialize PE5 to low
  *portE &= 0xDF;  // 0b1101 1111
  //initialize water sensor power to PE4
  *portE |= 0x10;  // 0b0001 0000
  //set PF0 to input
  //*portVRy &=  
  */
  pinMode(VRy, INPUT); //REPLACE THIS
  pinMode(fan, OUTPUT);
  
}

//Modes(state): off-0 idle-1 running-2 error-3

void loop(){
  delay(2000);
  time_t t = now();
  resval = aRead(respin);
  leds(state);

  buttonPress();

  float temp = dht.readTemperature(true);
  float hum = dht.readHumidity();
  
  
//--------------------vent-----------------
  //JOYSTICK | VENT
  yPosition = aRead(VRy); //Check Joystick pos
  mapY = map(yPosition, 0, 1023, -512, 512); // math???
  mapY = (mapY * -1); // reads negative for some reason
  if(mapY >= 100 && vent_angle < 90){ //open the vent
    Serial.print("Opening Vent (");
    Serial.print(vent_angle);
    Serial.print(")");
    Serial.print('\n');
    vent_angle = vent_angle + 15;
  }
  else if(mapY <= -100 && vent_angle > 0){ //close the vent
    Serial.print("Closing Vent (");
    Serial.print(vent_angle);
    Serial.print(")");
    Serial.print('\n');
    vent_angle = vent_angle - 15;
  }
  else{
    Serial.print("Vent Angle [");
    Serial.print(vent_angle);
    Serial.print("]");
    Serial.print('\n');
    if(vent_angle == 90){
      Serial.print("Vent is fully open");
    }
    else if(vent_angle == 0){
      Serial.print("Vent is fully closed");
    }
    else{
      Serial.print("Vent is slightly open");
    }
    Serial.print('\n');
  }
  myservo.write(vent_angle); //command to rotate the servo to the specified angle

//---------------------state----------------

  switch(state){
    case 0:
      //Off
      break;
    case 1:

      displayLCD(hum,temp,state);

      if(resval < waterThresh){
        //Sets state to error if water level is too low
        state = 3;
      }else if(temp > tempThresh){
        //Sets state to running if temperature is too hot
        state = 2;
      }
      break;
    case 2:
      displayLCD(hum,temp,state);

      //Need to set fan to running(turn off in if statement)
      dWrite(fan, HIGH); //DO THIS WITHOUT DWRITE
      Serial.write("Fan turned on at :");
      Serial.write(day());
      Serial.write("d");
      Serial.write(hour());
      Serial.write("h");
      Serial.write(minute());
      Serial.write("m");
      Serial.write(second());
      Serial.write("s Into Runtime.");
      Serial.write('\n');
      //output to file

      if(resval < waterThresh){
        //Sets state to error if water level is too low
        state = 3;
      }else if(temp < tempThresh){
        dWrite(fan, LOW); //DO THIS WITHOUT DWRITE 
        Serial.write("Fan turned off at :");
        Serial.write(day(t));
        Serial.write("d");
        Serial.write(hour(t));
        Serial.write("h");
        Serial.write(minute(t));
        Serial.write("m");
        Serial.write(second(t));
        Serial.write("s Into Runtime.");
        Serial.write('\n');
        //Sets state to idle once water has been cooled to lowest threshhold
        state = 1;
      }
      break;
    case 3:
      displayLCD(hum,temp,state);
      
      if(resval > waterThresh){
        //Sets state to idle once water level is above threshhold
        state = 1;
      }
      break;
    default :
      state = 1;
      break;
  }

}

//Display function for the lcd.(h = humidity, t = temperature, s = state)
void displayLCD(float h, float t, int s){

  lcd.clear();

  if(s == 3){
    lcd.setCursor(0,0);
    lcd.print("Error:     ");
    lcd.setCursor(0,1);
    lcd.print("Not enough water");
    return;
  }

  if(s == 1){
    lcd.setCursor(0,0);
    lcd.print("Cooler is Off");
  }

  
  //set cursor position to start of first line on the LCD
  lcd.setCursor(0,0);
  //text to print
  
  delay(100);

  lcd.setCursor(0,0);
  lcd.print("HUMID: ");
  lcd.print('\t' + h);
  lcd.setCursor(0,1);
  lcd.print("TEMP: ");
  lcd.print('\t' + t);


}

//set leds based on state of system
void leds(int s){
  if(s == 0){
    //off = yellow
    //digitalWrite(10, LOW);
    //digitalWrite(11, LOW);
    //digitalWrite(12, LOW);
    //digitalWrite(13, HIGH);
    *portB |= 0x80;
    *portB &= 0x8F;

  }else if(s == 1){
    //idle = green
    //digitalWrite(10, LOW);
    //digitalWrite(11, LOW);
    //digitalWrite(12, HIGH);
    //digitalWrite(13, LOW);
    *portB |= 0x40;
    *portB &= 0x4F;
  }else if(s == 2){
    //running = blue
    //digitalWrite(10, HIGH);
    //digitalWrite(11, LOW);
    //digitalWrite(12, LOW);
    //digitalWrite(13, LOW);

    *portB |= 0X10;
    *portB &= 0x1F;
  }else if(s == 3){
    //error = red
    //digitalWrite(10, HIGH);
    //digitalWrite(11, LOW);
    //digitalWrite(12, LOW);
    //digitalWrite(13, LOW);

    *portB |= 0x20;
    *portB &= 0x2F;
  }else{
    //not a valid state
    return;
  }
}



void aRef(uint8_t mode) //helper for aRead & aWrite
{
  a_ref = mode;
}

int aRead(uint8_t pin){

  uint8_t low, high;

#if defined(__AVR_ATmega2560__)
  if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#else
  if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(ADMUX)
  ADMUX = (a_ref << 6) | (pin & 0x07);
#endif

#if defined(ADCSRA) && defined(ADCL)
  // start the conversion
  sbi(ADCSRA, ADSC);

  // ADSC is cleared when the conversion finishes
  while (bit_is_set(ADCSRA, ADSC));

  low  = ADCL;
  high = ADCH;
#else
  // we dont have an ADC, return 0
  low = 0;
  high= 0;
#endif

  // combine the two bytes
  return (high << 8) | low;
}


int aWrite(uint8_t pin, int val){

  uint8_t bit = digitalPinToBitMask(pin);

  pin |= 0xF0; //set pin to OUTPUT (Questionable)
  if(val == 0){
    pin &= ~bit; //set pin to LOW
  }
  else if(val == 255){
    pin |= bit; //set pin to HIGH
  }
  else
  {
    switch(digitalPinToTimer(pin))
    {
      // XXX fix needed for atmega8
      #if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
      case TIMER0A:
        // connect pwm to pin on timer 0
        sbi(TCCR0, COM00);
        OCR0 = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR0A) && defined(COM0A1)
      case TIMER0A:
        // connect pwm to pin on timer 0, channel A
        sbi(TCCR0A, COM0A1);
        OCR0A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR0A) && defined(COM0B1)
      case TIMER0B:
        // connect pwm to pin on timer 0, channel B
        sbi(TCCR0A, COM0B1);
        OCR0B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR1A) && defined(COM1A1)
      case TIMER1A:
        // connect pwm to pin on timer 1, channel A
        sbi(TCCR1A, COM1A1);
        OCR1A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR1A) && defined(COM1B1)
      case TIMER1B:
        // connect pwm to pin on timer 1, channel B
        sbi(TCCR1A, COM1B1);
        OCR1B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR2) && defined(COM21)
      case TIMER2:
        // connect pwm to pin on timer 2
        sbi(TCCR2, COM21);
        OCR2 = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR2A) && defined(COM2A1)
      case TIMER2A:
        // connect pwm to pin on timer 2, channel A
        sbi(TCCR2A, COM2A1);
        OCR2A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR2A) && defined(COM2B1)
      case TIMER2B:
        // connect pwm to pin on timer 2, channel B
        sbi(TCCR2A, COM2B1);
        OCR2B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR3A) && defined(COM3A1)
      case TIMER3A:
        // connect pwm to pin on timer 3, channel A
        sbi(TCCR3A, COM3A1);
        OCR3A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR3A) && defined(COM3B1)
      case TIMER3B:
        // connect pwm to pin on timer 3, channel B
        sbi(TCCR3A, COM3B1);
        OCR3B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR3A) && defined(COM3C1)
      case TIMER3C:
        // connect pwm to pin on timer 3, channel C
        sbi(TCCR3A, COM3C1);
        OCR3C = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR4A)
      case TIMER4A:
        //connect pwm to pin on timer 4, channel A
        sbi(TCCR4A, COM4A1);
        #if defined(COM4A0)   // only used on 32U4
        cbi(TCCR4A, COM4A0);
        #endif
        OCR4A = val;  // set pwm duty
        break;
      #endif
      
      #if defined(TCCR4A) && defined(COM4B1)
      case TIMER4B:
        // connect pwm to pin on timer 4, channel B
        sbi(TCCR4A, COM4B1);
        OCR4B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR4A) && defined(COM4C1)
      case TIMER4C:
        // connect pwm to pin on timer 4, channel C
        sbi(TCCR4A, COM4C1);
        OCR4C = val; // set pwm duty
        break;
      #endif
        
      #if defined(TCCR4C) && defined(COM4D1)
      case TIMER4D:       
        // connect pwm to pin on timer 4, channel D
        sbi(TCCR4C, COM4D1);
        #if defined(COM4D0)   // only used on 32U4
        cbi(TCCR4C, COM4D0);
        #endif
        OCR4D = val;  // set pwm duty
        break;
      #endif

              
      #if defined(TCCR5A) && defined(COM5A1)
      case TIMER5A:
        // connect pwm to pin on timer 5, channel A
        sbi(TCCR5A, COM5A1);
        OCR5A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR5A) && defined(COM5B1)
      case TIMER5B:
        // connect pwm to pin on timer 5, channel B
        sbi(TCCR5A, COM5B1);
        OCR5B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR5A) && defined(COM5C1)
      case TIMER5C:
        // connect pwm to pin on timer 5, channel C
        sbi(TCCR5A, COM5C1);
        OCR5C = val; // set pwm duty
        break;
      #endif

      case NOT_ON_TIMER:
      default:
        if (val < 128) {
          pin &= ~bit; //set pin to LOW
        } else {
          pin |= bit; //set pin to HIGH
        }
    }
  }
}

void buttonPress(){
  stateButton = digitalRead(2);
  if(stateButton == HIGH && previous == LOW && millis() - time > debounce){
    if(state == 0){
      state = 1;
    }else{
      state = 0;
    }
    time = millis();
  }
  previous == stateButton;
}

void dWrite(uint8_t pin, uint8_t val){
  uint8_t timer = digitalPinToTimer(pin);
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out;

  if(port == NOT_A_PIN) return;

  if(timer != NOT_ON_TIMER) turnOffPWM(timer);

  out = portOutputRegister(port);

  uint8_t oldSREG = SREG;
  cli();

  if(val == LOW){
    *out &= ~bit;
  }else{
    *out |= bit;
  }
  SREG = oldSREG;

}
