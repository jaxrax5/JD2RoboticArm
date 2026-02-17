/*
 * SCARA Robot — Direct G-code Parser
 *
 * Reads "moves.txt" from SD card, parses G-code commands,
 * runs inverse kinematics, and drives two servo motors.
 *
 * Supported commands:
 *   G0  - Rapid positioning
 *   G1  - Linear move (feed rate)
 *   G2  - Clockwise arc
 *   G3  - Counter-clockwise arc
 *   G4  - Dwell / pause
 *   G20 - Units: inches
 *   G21 - Units: millimeters
 *   G28 - Home
 *   G90 - Absolute positioning
 *   G91 - Incremental positioning
 *   G92 - Set position offset
 *   M2  - Program end
 *   M6  - Tool change (acknowledged, not implemented)
 *
 * ============================================================================
 * PIN CONNECTIONS  (Arduino Nano)
 * ============================================================================
 *   SD card:  MOSI->D11  MISO->D12  SCK->D13  CS->D10
 *   Servo 1 (shoulder) signal -> D6
 *   Servo 2 (elbow)    signal -> D5
 *   Servo power -> external 5-6 V  (NOT Arduino 5 V pin!)
 * ============================================================================
 */

#include <SPI.h>
#include <SD.h>
#include <Servo.h>

// ============================================================================
// CONFIGURATION  —  edit to match your robot
// ============================================================================

const int   SD_CS_PIN     = 10;
const int   SERVO1_PIN    = 6;      // shoulder
const int   SERVO2_PIN    = 5;      // elbow
const char* MOVES_FILE    = "moves.txt";

const float ARM_L1        = 6.0;   // inches — shoulder to elbow
const float ARM_L2        = 6.0;   // inches — elbow to pen

const float HOME_X        = 6.0;   // home position (inches)
const float HOME_Y        = 6.0;

const int   SERVO1_HOME   = 90;    // degrees
const int   SERVO2_HOME   = 90;

const int   STEP_DELAY    = 15;    // ms between 1-degree servo steps
const float MM_PER_INCH   = 25.4;
const int   ARC_STEPS_PER_INCH = 8; // arc resolution

// ============================================================================
// GLOBALS
// ============================================================================

File  movesFile;
Servo axis1;
Servo axis2;

int   servoPos1 = SERVO1_HOME;
int   servoPos2 = SERVO2_HOME;

float currentX = HOME_X;
float currentY = HOME_Y;

bool  absoluteMode = true;
bool  inchMode     = true;
float offsetX      = 0.0;   // G92 coordinate offset
float offsetY      = 0.0;

// ============================================================================
// SERVO — smooth movement
// ============================================================================

void moveServos(int t1, int t2) {
  t1 = constrain(t1, 0, 180);
  t2 = constrain(t2, 0, 180);
  while (servoPos1 != t1 || servoPos2 != t2) {
    if      (servoPos1 < t1) servoPos1++;
    else if (servoPos1 > t1) servoPos1--;
    if      (servoPos2 < t2) servoPos2++;
    else if (servoPos2 > t2) servoPos2--;
    axis1.write(servoPos1);
    axis2.write(servoPos2);
    delay(STEP_DELAY);
  }
}

// ============================================================================
// INVERSE KINEMATICS
// ============================================================================

/*
 * Convert Cartesian (x, y) inches → servo angles.
 * Returns false if position is unreachable.
 *
 * Standard SCARA:
 *   cos(θ2) = (x²+y²−L1²−L2²) / (2·L1·L2)
 *   θ1 = atan2(y,x) − atan2(k2,k1)
 *   k1 = L1+L2·cos(θ2),  k2 = L2·sin(θ2)
 */
bool calcAngles(float x, float y, int &a1, int &a2) {
  float r    = sqrt(x*x + y*y);
  float rMax = ARM_L1 + ARM_L2;
  float rMin = abs(ARM_L1 - ARM_L2);

  if (r > rMax || r < rMin) {
    Serial.print("  SKIP out-of-reach (");
    Serial.print(x,2); Serial.print(",");
    Serial.print(y,2); Serial.print(") r=");
    Serial.println(r,2);
    return false;
  }

  float cosT2 = (x*x + y*y - ARM_L1*ARM_L1 - ARM_L2*ARM_L2)
                / (2.0 * ARM_L1 * ARM_L2);
  cosT2 = constrain(cosT2, -1.0, 1.0);

  float theta2 = acos(cosT2);
  float k1     = ARM_L1 + ARM_L2 * cos(theta2);
  float k2     = ARM_L2 * sin(theta2);
  float theta1 = atan2(y, x) - atan2(k2, k1);

  a1 = constrain((int)round(degrees(theta1)), 0, 180);
  a2 = constrain((int)round(degrees(theta2)), 0, 180);
  return true;
}

bool moveTo(float x, float y) {
  int a1, a2;
  if (!calcAngles(x, y, a1, a2)) return false;
  moveServos(a1, a2);
  currentX = x;
  currentY = y;
  return true;
}

// ============================================================================
// PARSING HELPERS
// ============================================================================

/*
 * Extract the float value following a parameter letter.
 * e.g. getParam("G1 X6.5 Y4 F2", 'X', found) → 6.5, found=true
 * Requires the letter to be preceded by a space (or be at position 0).
 */
float getParam(const String &line, char letter, bool &found) {
  found = false;
  int idx = line.indexOf(letter);
  if (idx == -1) return 0.0;
  if (idx > 0 && line.charAt(idx - 1) != ' ') return 0.0;

  String num = "";
  for (int i = idx + 1; i < (int)line.length(); i++) {
    char c = line.charAt(i);
    if (c == ' ' || c == ';') break;
    if (c == '-' || c == '.' || (c >= '0' && c <= '9')) num += c;
  }
  if (num.length()) { found = true; return num.toFloat(); }
  return 0.0;
}

/*
 * Return true when the token `cmd` (e.g. "G2") appears as a whole word.
 */
bool hasCmd(const String &line, const char *cmd) {
  String s  = cmd;
  int    idx = line.indexOf(s);
  if (idx == -1) return false;
  if (idx > 0 && line.charAt(idx - 1) != ' ') return false;
  int end = idx + s.length();
  if (end < (int)line.length()) {
    char nx = line.charAt(end);
    if (nx != ' ' && nx != ';' && nx != '\r' && nx != '\n') return false;
  }
  return true;
}

// ============================================================================
// COMMAND HANDLERS
// ============================================================================

// ---- G0 / G1 ---------------------------------------------------------------
void handleMove(const String &line, bool rapid) {
  bool hX, hY, hF;
  float px = getParam(line, 'X', hX);
  float py = getParam(line, 'Y', hY);
  float pf = getParam(line, 'F', hF);

  float tx = currentX, ty = currentY;
  if (absoluteMode) {
    if (hX) tx = px - offsetX;
    if (hY) ty = py - offsetY;
  } else {
    if (hX) tx += (inchMode ? px : px / MM_PER_INCH);
    if (hY) ty += (inchMode ? py : py / MM_PER_INCH);
  }
  if (!inchMode && absoluteMode) {
    if (hX) tx = px / MM_PER_INCH - offsetX;
    if (hY) ty = py / MM_PER_INCH - offsetY;
  }

  Serial.print(rapid ? "  G0 -> (" : "  G1 -> (");
  Serial.print(tx,2); Serial.print(", "); Serial.print(ty,2); Serial.println(")");
  moveTo(tx, ty);
}

// ---- G2 / G3 — arc ---------------------------------------------------------
void handleArc(const String &line, bool clockwise) {
  bool hX, hY, hI, hJ;
  float ex = getParam(line, 'X', hX);
  float ey = getParam(line, 'Y', hY);
  float oi = getParam(line, 'I', hI);   // I = X offset to centre
  float oj = getParam(line, 'J', hJ);   // J = Y offset to centre

  // End position
  float tx = absoluteMode ? ex - offsetX : currentX + ex;
  float ty = absoluteMode ? ey - offsetY : currentY + ey;

  // Arc centre (absolute)
  float cx = currentX + oi;
  float cy = currentY + oj;

  // Radius and angles
  float radius     = sqrt(oi*oi + oj*oj);
  float startAngle = atan2(currentY - cy, currentX - cx);
  float endAngle   = atan2(ty        - cy, tx        - cx);

  // Sweep angle
  float sweep;
  if (clockwise) {
    sweep = endAngle - startAngle;
    if (sweep > 0) sweep -= TWO_PI;
  } else {
    sweep = endAngle - startAngle;
    if (sweep < 0) sweep += TWO_PI;
  }

  // Number of steps
  float arcLen  = abs(radius * sweep);
  int   nSteps  = max(4, (int)(arcLen * ARC_STEPS_PER_INCH));

  Serial.print(clockwise ? "  G2 arc " : "  G3 arc ");
  Serial.print(nSteps); Serial.println(" steps");

  for (int i = 1; i <= nSteps; i++) {
    float t     = (float)i / nSteps;
    float angle = startAngle + t * sweep;
    float wx    = cx + radius * cos(angle);
    float wy    = cy + radius * sin(angle);
    moveTo(wx, wy);
  }
}

// ---- G4 — dwell ------------------------------------------------------------
void handleDwell(const String &line) {
  bool hP;
  float secs = getParam(line, 'P', hP);
  if (hP && secs > 0) {
    Serial.print("  G4 dwell "); Serial.print(secs,2); Serial.println(" s");
    delay((unsigned long)(secs * 1000));
  }
}

// ---- G28 — home ------------------------------------------------------------
void handleHome() {
  Serial.println("  G28 home");
  int a1, a2;
  if (calcAngles(HOME_X, HOME_Y, a1, a2)) {
    moveServos(a1, a2);
    currentX = HOME_X;
    currentY = HOME_Y;
  } else {
    // Fallback: use preset servo angles
    moveServos(SERVO1_HOME, SERVO2_HOME);
    currentX = HOME_X;
    currentY = HOME_Y;
  }
}

// ---- G92 — set position offset ---------------------------------------------
void handleSetPos(const String &line) {
  bool hX, hY;
  float px = getParam(line, 'X', hX);
  float py = getParam(line, 'Y', hY);
  if (hX) offsetX = currentX - px;
  if (hY) offsetY = currentY - py;
  if (!hX && !hY) { offsetX = 0; offsetY = 0; }
  Serial.print("  G92 offset ("); Serial.print(offsetX,2);
  Serial.print(", "); Serial.print(offsetY,2); Serial.println(")");
}

// ============================================================================
// MAIN LINE PROCESSOR
// ============================================================================

/*
 * Parse one line of G-code and act on it.
 * Returns false when M2 is encountered (end of program).
 */
bool processLine(String line) {
  line.trim();
  line.toUpperCase();

  if (line.length() == 0 || line.charAt(0) == ';') return true;

  // Strip inline comment
  int ci = line.indexOf(';');
  if (ci != -1) { line = line.substring(0, ci); line.trim(); }
  if (line.length() == 0) return true;

  Serial.print("< "); Serial.println(line);

  // Movement
  if      (hasCmd(line, "G0"))  handleMove(line, true);
  else if (hasCmd(line, "G1"))  handleMove(line, false);
  else if (hasCmd(line, "G2"))  handleArc(line, true);
  else if (hasCmd(line, "G3"))  handleArc(line, false);
  // Timing
  else if (hasCmd(line, "G4"))  handleDwell(line);
  // Units
  else if (hasCmd(line, "G20")) { inchMode = true;  Serial.println("  G20 inches"); }
  else if (hasCmd(line, "G21")) { inchMode = false; Serial.println("  G21 mm"); }
  // Positioning mode
  else if (hasCmd(line, "G90")) { absoluteMode = true;  Serial.println("  G90 absolute"); }
  else if (hasCmd(line, "G91")) { absoluteMode = false; Serial.println("  G91 incremental"); }
  // Coordinate system
  else if (hasCmd(line, "G28")) handleHome();
  else if (hasCmd(line, "G92")) handleSetPos(line);
  // Machine
  else if (hasCmd(line, "M6"))  Serial.println("  M6 tool change (ignored)");
  else if (hasCmd(line, "M2"))  { Serial.println("  M2 end"); return false; }
  else { Serial.print("  ? "); Serial.println(line); }

  return true;   // keep reading
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  SCARA Direct G-code Parser");
  Serial.println("  Reads moves.txt -> servo angles");
  Serial.println("========================================");
  Serial.print("  Arm: L1="); Serial.print(ARM_L1);
  Serial.print("\"  L2="); Serial.print(ARM_L2);
  Serial.print("\"  MaxReach="); Serial.print(ARM_L1+ARM_L2);
  Serial.println("\"");
  Serial.println("========================================");
  Serial.println();

  // SD card
  Serial.print("SD card... ");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("FAILED — check MOSI->D11 MISO->D12 SCK->D13 CS->D10");
    while (1);
  }
  Serial.println("OK");

  if (!SD.exists(MOVES_FILE)) {
    Serial.print("ERROR: '"); Serial.print(MOVES_FILE);
    Serial.println("' not found. Create with Python GUI then copy to SD card.");
    while (1);
  }
  Serial.println("Found moves.txt");

  // Servos
  axis1.attach(SERVO1_PIN);
  axis2.attach(SERVO2_PIN);
  axis1.write(SERVO1_HOME);
  axis2.write(SERVO2_HOME);
  delay(800);
  Serial.print("Servos at ("); Serial.print(SERVO1_HOME);
  Serial.print(", "); Serial.print(SERVO2_HOME); Serial.println(")");

  Serial.println();
  Serial.println("Starting execution...");
  Serial.println("----------------------------------------");
  delay(500);
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  movesFile = SD.open(MOVES_FILE);
  if (!movesFile) {
    Serial.println("ERROR: cannot open moves.txt");
    while (1);
  }

  int  lines   = 0;
  bool running = true;

  while (movesFile.available() && running) {
    String line = movesFile.readStringUntil('\n');
    lines++;
    running = processLine(line);
  }

  movesFile.close();

  Serial.println("----------------------------------------");
  Serial.println("Done.");
  Serial.print(lines); Serial.println(" lines processed.");
  Serial.println("Reset Arduino to run again.");

  // Blink to signal completion
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(200);
    digitalWrite(LED_BUILTIN, LOW);  delay(200);
  }
  while (1) delay(1000);
}
