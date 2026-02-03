#include <SD.h>
#include <Servo.h>
#include <SPI.h>

File myFile;
Servo axis1;
Servo axis2;


int pos1 = 75;
int pos2 = 120;

void moveServos(int move_a1, int move_a2, int stepDelay) {
  while (pos1 != move_a1 || pos2 != move_a2) {

    if (pos1 < move_a1) pos1++;
    else if (pos1 > move_a1) pos1--;

    if (pos2 < move_a2) pos2++;
    else if (pos2 > move_a2) pos2--;

    axis1.write(pos1);
    axis2.write(pos2);

    delay(stepDelay);
  }

}


void setup() {
  /*
  Serial.begin(9600);
  while(!Serial) {
    ;
  }
  Serial.println("Initilizing SD");
  */

  if (!SD.begin(10)) {
    //Serial.println("initialization failed!");
    while (1);
  }
  //Serial.println("initialization done.");

  axis1.attach(6);
  axis2.attach(5);

  delay(500);


}

void loop() {
  myFile = SD.open("moves.txt");

  while(myFile.available()) {
    String line = myFile.readStringUntil('\n');
    //Serial.println(line);
    line.trim();
    //if (line.length() == 0) continue; //what does this continue keyword mean?
    int comma = line.indexOf(','); // is this how we seperate axis commands
    //if (comma < 0) continue; //no idea

    int target1 = line.substring(0, comma).toInt();
    int target2 = line.substring(comma + 1).toInt();

    //Serial.println(target1);
    //Serial.println(target2);
    //axis1.write(target1);

    moveServos(target1, target2, 100);
  }

  myFile.close();
  while(1);

}

