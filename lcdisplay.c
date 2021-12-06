#include "LiquidCrystal.h"

// initialize the library by providing the nuber of pins to it
LiquidCrystal lcd(8,9,4,5,6,7);

void setup() {
lcd.begin(16,2);

// set cursor position to start of first line on the LCD
lcd.setCursor(0,0);
//text to print
lcd.print("HUMID: ");
lcd.print("TEMP: ");
delay(100);

int humid=0;
int temp=25;
lcd.setCursor(0,1);
lcd.print("       "); //spacing
lcd.print(humid);
while(humid<=100) //counting timer
{
  humid=humid+1;
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print(humid);
  lcd.print("       "); //spacing
  lcd.print(temp);
}

}
void loop()
{
  lcd.clear();
}