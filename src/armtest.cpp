#include <Arduino.h>
#include <Servo.h>

//First axis D6
Servo axis1;
Servo axis2;
//Secont axis D5

void setup() {
  axis1.attach(6);
  axis2.attach(5);

}

void loop() {
  for (int i = 0; i <= 180;i++) {
    axis1.write(i);
    delay(5);
  }
  for (int i = 0; i <= 180; i++) {
    axis2.write(i);
    delay(5);
  }
  delay(1000);
  for (int i = 180; i >= 0; i--) {
    axis1.write(i);
    delay(5);
  }
  for (int i = 180; i >= 0; i--) {
    axis2.write(i);
    delay(5);
  }
  delay(1000);


}