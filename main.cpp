#include <SD.h>
#include <Servo.h>
#include <SPI.h>

File myFile;
Servo axis1;
Servo axis2;

int pos1 = 90;
int pos2 = 90;

char token[3][10];

void move_servos(int move_a1, int move_a2, int stepDelay) {
  move_a1 = constrain(move_a1, 0, 180);
  move_a2 = constrain(move_a2, 0, 180);

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

void tokenize(String line) {
  int i = 0;
  int len = line.length();
  for (int token_count = 0; token_count < 3; token_count++) {
    int j = 0;
    while (i < len && line[i] != ' ') {
      if (j < 9) token[token_count][j++] = line[i];
      i++;
    }
    token[token_count][j] = '\0';
    i++; // skip space
  }
}

float get_x() {
  int i = 1;
  String s = "";
  while (token[1][i] != '\0') s += token[1][i++];
  return s.toFloat();
}

float get_y() {
  int i = 1;
  String s = "";
  while (token[2][i] != '\0') s += token[2][i++];
  return s.toFloat();
}

void send_coordinates(float x, float y) {
  float r = sqrt(x * x + y * y);
  float theta = atan2(y, x);

  if (r > 986.0) {
    Serial.println("ERR: out of range");
    return;
  }

  float angle1 = degrees(acos(r / 986.0) + theta);
  float angle2 = degrees((PI / 2.0 - (radians(angle1) - theta)) * 2.0);
  angle1 = round(angle1);
  angle2 = round(angle2);

  Serial.print("A ");
  Serial.print(angle1);
  Serial.print(' ');
  Serial.println(angle2);

  move_servos((int)angle1, (int)angle2, 15);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  if (!SD.begin(10)) {
    Serial.println("ERR: SD init failed");
    while (1);
  }

  axis1.attach(6);
  axis2.attach(9);
  axis1.write(90);
  axis2.write(90);
  delay(500);
  

  // --- Do everything here, ONCE ---
  myFile = SD.open("moves.txt", FILE_READ);
  if (!myFile) {
    Serial.println("ERR: could not open moves.txt");
    while (1);
  }

  while (myFile.available()) {
    String line = myFile.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) continue; // skip blank lines

    tokenize(line);
    float x = get_x();
    float y = get_y();

    Serial.print("C ");
    Serial.print(x);
    Serial.print(" ");
    Serial.println(y);

    send_coordinates(x, y);
  }

  myFile.close();
  Serial.println("DONE");
}

void loop() {
  // intentionally empty - everything runs once in setup()
}