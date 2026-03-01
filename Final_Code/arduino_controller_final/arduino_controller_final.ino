/*
 * SCARA Robot — Direct G-code Parser (FIXED)
 *
 * Reads "moves.txt" from SD card, parses G-code commands,
 * runs inverse kinematics, and drives two servo motors.
 *
 * FIX: Uses forward kinematics at startup to calculate actual position
 *      from servo angles. This fixes the "jitter but no movement" bug.
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
 *   M6  - Tool change
 *
 * PIN CONNECTIONS:
 *   SD card:  MOSI->D11  MISO->D12  SCK->D13  CS->D10
 *   Servo 1 (shoulder) -> D6
 *   Servo 2 (elbow) -> D5
 *   External power for servos (5-6V, NOT Arduino 5V pin)
 */

#include <SPI.h>
#include <SD.h>
#include <Servo.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

const int   SD_CS_PIN     = 10;
const int   SERVO1_PIN    = 6;
const int   SERVO2_PIN    = 5;
const char* MOVES_FILE    = "moves.txt";

const float ARM_L1        = 8.0;   // inches
const float ARM_L2        = 8.0;   // inches

// Home position servo angles (change these to match YOUR robot's home)
const int   SERVO1_HOME   = 45;    // degrees - shoulder at home
const int   SERVO2_HOME   = 90;    // degrees - elbow at home

const int   STEP_DELAY    = 15;    // ms between steps
const float MM_PER_INCH   = 25.4;
const int   ARC_STEPS_PER_INCH = 8;

// ============================================================================
// GLOBALS
// ============================================================================

File  movesFile;
Servo axis1;
Servo axis2;

int   servoPos1;
int   servoPos2;

float currentX;   // Will be calculated from actual servo positions at startup
float currentY;

bool  absoluteMode = true;
bool  inchMode     = true;
float offsetX      = 0.0;
float offsetY      = 0.0;

// ============================================================================
// FORWARD KINEMATICS
// ============================================================================

/*
 * Calculate Cartesian position from servo angles.
 * This tells us where the arm actually is.
 *
 * x = L1·cos(θ1) + L2·cos(θ1+θ2)
 * y = L1·sin(θ1) + L2·sin(θ1+θ2)
 */
void forwardKinematics(int angle1, int angle2, float &x, float &y) {
  float t1 = radians(angle1);
  float t2 = radians(angle2);
  
  x = ARM_L1 * cos(t1) + ARM_L2 * cos(t1 + t2);
  y = ARM_L1 * sin(t1) + ARM_L2 * sin(t1 + t2);
}

// ============================================================================
// INVERSE KINEMATICS
// ============================================================================

bool calcAngles(float x, float y, int &a1, int &a2) {
  float r    = sqrt(x*x + y*y);
  float rMax = ARM_L1 + ARM_L2;
  float rMin = abs(ARM_L1 - ARM_L2);

  if (r > rMax || r < rMin) {
    Serial.print("  SKIP out-of-reach (");
    Serial.print(x,2); Serial.print(", ");
    Serial.print(y,2); Serial.print(") r=");
    Serial.print(r,2);
    Serial.print(" [min="); Serial.print(rMin,2);
    Serial.print(" max="); Serial.print(rMax,2);
    Serial.println("]");
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

// ============================================================================
// SERVO CONTROL
// ============================================================================

void moveServos(int t1, int t2) {
  t1 = constrain(t1, 0, 180);
  t2 = constrain(t2, 0, 180);
  
  Serial.print("  Moving servos from (");
  Serial.print(servoPos1); Serial.print("°, ");
  Serial.print(servoPos2); Serial.print("°) to (");
  Serial.print(t1); Serial.print("°, ");
  Serial.print(t2); Serial.println("°)");
  
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

bool moveTo(float x, float y) {
  int a1, a2;
  if (!calcAngles(x, y, a1, a2)) return false;
  moveServos(a1, a2);
  
  // Update position
  currentX = x;
  currentY = y;
  
  Serial.print("  Position now: (");
  Serial.print(currentX, 2); Serial.print("\", ");
  Serial.print(currentY, 2); Serial.println("\")");
  
  return true;
}

// ============================================================================
// PARSING HELPERS
// ============================================================================

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
  if (num.length()) { 
    found = true;
    return num.toFloat();
  }
  return 0.0;
}

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

  if (!hX && !hY) {
    Serial.println("  (no X or Y - skipping)");
    return;
  }

  float tx = currentX, ty = currentY;
  
  if (absoluteMode) {
    // Absolute positioning
    if (hX) tx = px - offsetX;
    if (hY) ty = py - offsetY;
    
    // Unit conversion for absolute mode
    if (!inchMode) {
      tx /= MM_PER_INCH;
      ty /= MM_PER_INCH;
    }
  } else {
    // Incremental positioning
    float dx = hX ? px : 0;
    float dy = hY ? py : 0;
    
    // Unit conversion for incremental mode
    if (!inchMode) {
      dx /= MM_PER_INCH;
      dy /= MM_PER_INCH;
    }
    
    tx = currentX + dx;
    ty = currentY + dy;
  }

  Serial.print(rapid ? "  G0 (rapid) -> (" : "  G1 (feed) -> (");
  Serial.print(tx, 2); Serial.print("\", ");
  Serial.print(ty, 2); Serial.println("\")");
  
  moveTo(tx, ty);
}

// ---- G2 / G3 ---------------------------------------------------------------
void handleArc(const String &line, bool clockwise) {
  bool hX, hY, hI, hJ;
  float ex = getParam(line, 'X', hX);
  float ey = getParam(line, 'Y', hY);
  float oi = getParam(line, 'I', hI);
  float oj = getParam(line, 'J', hJ);

  if (!hI && !hJ) {
    Serial.println("  ERROR: G2/G3 needs I and/or J");
    return;
  }

  // Calculate end position
  float tx = absoluteMode ? ex - offsetX : currentX + ex;
  float ty = absoluteMode ? ey - offsetY : currentY + ey;

  // Arc centre (absolute)
  float cx = currentX + oi;
  float cy = currentY + oj;

  float radius     = sqrt(oi*oi + oj*oj);
  float startAngle = atan2(currentY - cy, currentX - cx);
  float endAngle   = atan2(ty        - cy, tx        - cx);

  // Calculate sweep
  float sweep;
  if (clockwise) {
    sweep = endAngle - startAngle;
    if (sweep > 0) sweep -= TWO_PI;
  } else {
    sweep = endAngle - startAngle;
    if (sweep < 0) sweep += TWO_PI;
  }

  float arcLen = abs(radius * sweep);
  int   nSteps = max(4, (int)(arcLen * ARC_STEPS_PER_INCH));

  Serial.print(clockwise ? "  G2 (CW arc) " : "  G3 (CCW arc) ");
  Serial.print(nSteps); Serial.print(" steps, radius ");
  Serial.print(radius, 2); Serial.println("\"");

  for (int i = 1; i <= nSteps; i++) {
    float t     = (float)i / nSteps;
    float angle = startAngle + t * sweep;
    float wx    = cx + radius * cos(angle);
    float wy    = cy + radius * sin(angle);
    
    if (!moveTo(wx, wy)) {
      Serial.print("  Arc aborted at step ");
      Serial.println(i);
      return;
    }
  }
}

// ---- G4 ----------------------------------------------------------------
void handleDwell(const String &line) {
  bool hP;
  float secs = getParam(line, 'P', hP);
  if (hP && secs > 0) {
    Serial.print("  G4 pausing ");
    Serial.print(secs, 2);
    Serial.println(" seconds...");
    delay((unsigned long)(secs * 1000));
    Serial.println("  G4 done");
  }
}

// ---- G28 ---------------------------------------------------------------
void handleHome() {
  Serial.println("  G28 homing...");
  
  // Move servos to home angles
  moveServos(SERVO1_HOME, SERVO2_HOME);
  
  // Calculate where we actually are now
  forwardKinematics(SERVO1_HOME, SERVO2_HOME, currentX, currentY);
  
  Serial.print("  Home position: (");
  Serial.print(currentX, 2); Serial.print("\", ");
  Serial.print(currentY, 2); Serial.print("\") at servo angles (");
  Serial.print(SERVO1_HOME); Serial.print("°, ");
  Serial.print(SERVO2_HOME); Serial.println("°)");
}

// ---- G92 ---------------------------------------------------------------
void handleSetPos(const String &line) {
  bool hX, hY;
  float px = getParam(line, 'X', hX);
  float py = getParam(line, 'Y', hY);
  
  if (!hX && !hY) {
    // No parameters = clear offset
    offsetX = 0;
    offsetY = 0;
    Serial.println("  G92 offset cleared");
  } else {
    if (hX) offsetX = currentX - px;
    if (hY) offsetY = currentY - py;
    Serial.print("  G92 position set to (");
    Serial.print(hX ? px : currentX + offsetX, 2); Serial.print(", ");
    Serial.print(hY ? py : currentY + offsetY, 2); Serial.print(")  offset (");
    Serial.print(offsetX, 2); Serial.print(", ");
    Serial.print(offsetY, 2); Serial.println(")");
  }
}

// ============================================================================
// LINE PROCESSOR
// ============================================================================

bool processLine(String line) {
  line.trim();
  line.toUpperCase();

  if (line.length() == 0 || line.charAt(0) == ';') return true;

  // Remove inline comments
  int ci = line.indexOf(';');
  if (ci != -1) {
    line = line.substring(0, ci);
    line.trim();
  }
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
  else if (hasCmd(line, "G20")) {
    inchMode = true;
    Serial.println("  G20 units = inches");
  }
  else if (hasCmd(line, "G21")) {
    inchMode = false;
    Serial.println("  G21 units = millimeters");
  }
  // Coordinate system
  else if (hasCmd(line, "G28")) handleHome();
  else if (hasCmd(line, "G90")) {
    absoluteMode = true;
    Serial.println("  G90 absolute positioning");
  }
  else if (hasCmd(line, "G91")) {
    absoluteMode = false;
    Serial.println("  G91 incremental positioning");
  }
  else if (hasCmd(line, "G92")) handleSetPos(line);
  // Machine
  else if (hasCmd(line, "M6"))  Serial.println("  M6 tool change (ignored)");
  else if (hasCmd(line, "M2"))  {
    Serial.println("  M2 end program");
    return false;
  }
  else {
    Serial.print("  ? unknown: ");
    Serial.println(line);
  }

  return true;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  SCARA G-code Parser (FIXED)");
  Serial.println("========================================");
  Serial.print("  Arm: L1="); Serial.print(ARM_L1);
  Serial.print("\"  L2="); Serial.print(ARM_L2);
  Serial.print("\"  Reach="); Serial.print(ARM_L1+ARM_L2);
  Serial.println("\"");
  Serial.println("========================================");

  // SD card
  Serial.print("SD card... ");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("FAILED");
    Serial.println("Check: MOSI->D11 MISO->D12 SCK->D13 CS->D10");
    while (1);
  }
  Serial.println("OK");

  if (!SD.exists(MOVES_FILE)) {
    Serial.print("ERROR: "); Serial.print(MOVES_FILE);
    Serial.println(" not found");
    while (1);
  }
  Serial.println("Found moves.txt");
  Serial.println();

  // Servos - initialize to home position
  axis1.attach(SERVO1_PIN);
  axis2.attach(SERVO2_PIN);
  
  Serial.println("Initializing servos to home position...");
  axis1.write(SERVO1_HOME);
  axis2.write(SERVO2_HOME);
  servoPos1 = SERVO1_HOME;
  servoPos2 = SERVO2_HOME;
  delay(1000);
  
  // Calculate actual starting position using forward kinematics
  forwardKinematics(SERVO1_HOME, SERVO2_HOME, currentX, currentY);
  
  Serial.print("Starting position: (");
  Serial.print(currentX, 2); Serial.print("\", ");
  Serial.print(currentY, 2); Serial.print("\") at (");
  Serial.print(servoPos1); Serial.print("°, ");
  Serial.print(servoPos2); Serial.println("°)");
  Serial.println();

  Serial.println("========================================");
  Serial.println("  Starting G-code execution");
  Serial.println("========================================");
  Serial.println();
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

  Serial.println();
  Serial.println("========================================");
  Serial.print("Done. "); Serial.print(lines);
  Serial.println(" lines processed.");
  Serial.println("Reset Arduino to run again.");
  Serial.println("========================================");

  // Blink LED
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  while (1) delay(1000);
}
