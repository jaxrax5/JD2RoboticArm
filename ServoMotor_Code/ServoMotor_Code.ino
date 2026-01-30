#include <Servo.h> 

Servo myServo1; // create the Servo object to manipulate later. 

Servo myServo2; // 2nd motor. 
 
int pos1 = 0; // set the 1st angle position for motor 
int pos2 = 0; // set the 2nd angle position for motor. 

void setup() {
  myServo1.attach(9); // attaches 1st Servo to position 9. 
  myServo2.attach(10); // Attach 2nd Servo to position 10. 
}

void loop() {
  for (pos1 = 0; pos1 <= 90; pos1++) {
    myServo1.write(pos1); // write to Servo the corrseponding angle. 
  }
  
  for (pos1 = 90; pos1 >= 0; pos1--) {
    myServo1.write(pos1); // write to servo to go backwards. 
  }

  for (pos2 = 0; pos2 <= 180; pos2++) {
    myServo2.write(pos2); // write to Servo the corrseponding angle. 
  }
  
  for (pos2 = 180; pos2 >= 0; pos2--) {
    myServo2.write(pos1); // write to servo to go backwards. 
  }


}
