int resval = 0;
int respin = A5;

void setup(){
  Serial.begin(9600);
}

void loop(){
  resval = analogRead(respin);
  if(resval <= 100){
    Serial.println("Empty");
  }else if(resval > 100 && resval <= 300){
    Serial.println("Low");
  }else if(resval > 300 && resval <= 330){
    Serial.println("Med");
  }else if(resval > 330){
    Serial.println("High");
  }
  delay(1000);
}