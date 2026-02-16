/*
 * SCARA Robot Controller - Direct G-code Parsing
 * 
 * This version reads G-code commands DIRECTLY from SD card
 * and performs inverse kinematics calculations on the Arduino.
 * 
 * Hardware: Arduino Nano + SD Card Module + 2 Servos
 * 
 * ============================================================================
 * PIN CONNECTIONS
 * ============================================================================
 * SD Card Module:
 *   MOSI → D11
 *   MISO → D12
 *   SCK  → D13
 *   CS   → D10
 *   VCC  → 5V
 *   GND  → GND
 * 
 * Servos:
 *   Servo 1 Signal → D6
 *   Servo 2 Signal → D5
 *   Power → External 5-6V (NOT Arduino!)
 * 
 * ============================================================================
 * G-CODE FILE FORMAT
 * ============================================================================
 * Create a file named "program.gcode" on SD card with G-code commands:
 * 
 * G90 G20        ; Absolute mode, inches
 * G0 X4 Y4       ; Rapid move to (4,4)
 * G1 X6 Y4 F2    ; Linear move to (6,4) at 2 in/sec
 * G1 X6 Y6       ; Move to (6,6)
 * G28            ; Return home
 * M2             ; End program
 * 
 * Supported Commands:
 *   G0  - Rapid positioning
 *   G1  - Linear move with feed rate
 *   G2  - Clockwise arc
 *   G3  - Counter-clockwise arc
 *   G4  - Dwell/pause
 *   G20 - Inches
 *   G21 - Millimeters
 *   G28 - Home
 *   G90 - Absolute positioning
 *   G91 - Incremental positioning
 *   G92 - Set position
 *   M2  - Program end
 *   M6  - Tool change (ignored)
 * 
 * Author: Robot Team
 * Version: 3.0 - Direct G-code Parser
 */

#include <SPI.h>
#include <SD.h>
#include <Servo.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Pin Configuration
const int SD_CS_PIN = 10;
const int SERVO1_PIN = 6;
const int SERVO2_PIN = 5;

// File Configuration
const char* GCODE_FILE = "program.gcode";

// Robot Arm Configuration (MUST MATCH config.py!)
const float ARM_L1 = 6.0;        // Length of first arm segment (inches)
const float ARM_L2 = 6.0;        // Length of second arm segment (inches)

// Home Position
const float HOME_X = 6.0;        // Home X position (inches)
const float HOME_Y = 6.0;        // Home Y position (inches)
const int SERVO1_HOME = 75;      // Servo 1 home angle
const int SERVO2_HOME = 120;     // Servo 2 home angle

// Motion Parameters
const int STEP_DELAY = 15;       // ms between servo steps
const float DEFAULT_FEED_RATE = 2.0;  // inches/sec
const float MAX_FEED_RATE = 10.0;

// Servo Limits
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

// Arc Interpolation
const int ARC_SEGMENTS_PER_INCH = 8;

// Units Conversion
const float MM_PER_INCH = 25.4;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Hardware
File gcodeFile;
Servo axis1;
Servo axis2;

// Current State
int currentPos1 = SERVO1_HOME;
int currentPos2 = SERVO2_HOME;
float currentX = HOME_X;
float currentY = HOME_Y;

// G-code Parser State
bool absoluteMode = true;         // G90/G91
bool inchMode = true;             // G20/G21
float feedRate = DEFAULT_FEED_RATE;
float offsetX = 0.0;              // G92 offset
float offsetY = 0.0;

// Statistics
int linesProcessed = 0;
int movesExecuted = 0;
int errorsEncountered = 0;

// ============================================================================
// INVERSE KINEMATICS
// ============================================================================

/**
 * Calculate servo angles from Cartesian coordinates
 * Returns true if position is reachable, false otherwise
 */
bool inverseKinematics(float x, float y, int &angle1, int &angle2) {
  // Calculate distance from origin
  float r = sqrt(x*x + y*y);
  
  // Check reachability
  float maxReach = ARM_L1 + ARM_L2;
  float minReach = abs(ARM_L1 - ARM_L2);
  
  if (r > maxReach || r < minReach) {
    Serial.print("ERROR: Position (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(") out of reach. Distance: ");
    Serial.println(r);
    return false;
  }
  
  // Law of cosines for elbow angle
  float cos_theta2 = (x*x + y*y - ARM_L1*ARM_L1 - ARM_L2*ARM_L2) / (2.0 * ARM_L1 * ARM_L2);
  
  // Clamp to valid range
  cos_theta2 = constrain(cos_theta2, -1.0, 1.0);
  
  float theta2 = acos(cos_theta2);
  
  // Calculate shoulder angle
  float k1 = ARM_L1 + ARM_L2 * cos(theta2);
  float k2 = ARM_L2 * sin(theta2);
  float theta1 = atan2(y, x) - atan2(k2, k1);
  
  // Convert to degrees
  angle1 = (int)round(theta1 * 57.2957795);  // radians to degrees
  angle2 = (int)round(theta2 * 57.2957795);
  
  // Constrain to servo limits
  angle1 = constrain(angle1, SERVO_MIN, SERVO_MAX);
  angle2 = constrain(angle2, SERVO_MIN, SERVO_MAX);
  
  return true;
}

// ============================================================================
// SERVO CONTROL
// ============================================================================

/**
 * Move servos smoothly to target angles
 */
void moveServos(int target1, int target2, int stepDelay) {
  target1 = constrain(target1, SERVO_MIN, SERVO_MAX);
  target2 = constrain(target2, SERVO_MIN, SERVO_MAX);
  
  while (currentPos1 != target1 || currentPos2 != target2) {
    if (currentPos1 < target1) currentPos1++;
    else if (currentPos1 > target1) currentPos1--;
    
    if (currentPos2 < target2) currentPos2++;
    else if (currentPos2 > target2) currentPos2--;
    
    axis1.write(currentPos1);
    axis2.write(currentPos2);
    delay(stepDelay);
  }
}

/**
 * Move to Cartesian position
 */
bool moveTo(float x, float y) {
  int angle1, angle2;
  
  if (inverseKinematics(x, y, angle1, angle2)) {
    moveServos(angle1, angle2, STEP_DELAY);
    currentX = x;
    currentY = y;
    movesExecuted++;
    return true;
  }
  
  errorsEncountered++;
  return false;
}

/**
 * Move to home position
 */
void goHome() {
  Serial.println("Moving to home position...");
  moveServos(SERVO1_HOME, SERVO2_HOME, STEP_DELAY);
  currentX = HOME_X;
  currentY = HOME_Y;
  Serial.println("Home reached");
}

// ============================================================================
// G-CODE PARSING UTILITIES
// ============================================================================

/**
 * Extract numeric value after a letter parameter
 * Example: extractParam("G1 X10 Y20", 'X') returns 10.0
 */
float extractParam(String line, char param, bool &found) {
  found = false;
  int idx = line.indexOf(param);
  
  if (idx == -1) return 0.0;
  
  // Make sure it's a parameter (preceded by space or at start)
  if (idx > 0 && line.charAt(idx-1) != ' ') return 0.0;
  
  // Extract number after parameter letter
  String numStr = "";
  for (int i = idx + 1; i < line.length(); i++) {
    char c = line.charAt(i);
    if (c == ' ' || c == ';') break;
    if (c == '-' || c == '.' || (c >= '0' && c <= '9')) {
      numStr += c;
    }
  }
  
  if (numStr.length() > 0) {
    found = true;
    return numStr.toFloat();
  }
  
  return 0.0;
}

/**
 * Check if line contains a specific command
 */
bool hasCommand(String line, String cmd) {
  int idx = line.indexOf(cmd);
  if (idx == -1) return false;
  
  // Verify it's at word boundary
  if (idx > 0 && line.charAt(idx-1) != ' ') return false;
  if (idx + cmd.length() < line.length()) {
    char next = line.charAt(idx + cmd.length());
    if (next != ' ' && next != ';' && next != '\n' && next != '\r') return false;
  }
  
  return true;
}

// ============================================================================
// LINEAR MOVE (G0, G1)
// ============================================================================

void executeLinearMove(String line, bool rapid) {
  bool hasX, hasY, hasF;
  float x = extractParam(line, 'X', hasX);
  float y = extractParam(line, 'Y', hasY);
  float f = extractParam(line, 'F', hasF);
  
  // Update feed rate if specified
  if (hasF) {
    feedRate = constrain(f, 0.1, MAX_FEED_RATE);
  }
  
  // Calculate target position
  float targetX = currentX;
  float targetY = currentY;
  
  if (absoluteMode) {
    // Absolute positioning
    if (hasX) targetX = x;
    if (hasY) targetY = y;
  } else {
    // Relative positioning
    if (hasX) targetX += x;
    if (hasY) targetY += y;
  }
  
  // Apply offset (G92)
  targetX -= offsetX;
  targetY -= offsetY;
  
  // Convert units if needed
  if (!inchMode) {
    targetX /= MM_PER_INCH;
    targetY /= MM_PER_INCH;
  }
  
  // Execute move
  if (moveTo(targetX, targetY)) {
    Serial.print(rapid ? "G0 " : "G1 ");
    Serial.print("(");
    Serial.print(targetX, 2);
    Serial.print(", ");
    Serial.print(targetY, 2);
    Serial.println(")");
  }
}

// ============================================================================
// ARC MOVE (G2, G3)
// ============================================================================

void executeArcMove(String line, bool clockwise) {
  bool hasX, hasY, hasI, hasJ;
  float endX = extractParam(line, 'X', hasX);
  float endY = extractParam(line, 'Y', hasY);
  float offsetI = extractParam(line, 'I', hasI);
  float offsetJ = extractParam(line, 'J', hasJ);
  
  if (!hasI && !hasJ) {
    Serial.println("ERROR: Arc missing I or J parameter");
    errorsEncountered++;
    return;
  }
  
  // Calculate target position
  float targetX = absoluteMode ? endX : currentX + endX;
  float targetY = absoluteMode ? endY : currentY + endY;
  
  // Convert units
  if (!inchMode) {
    targetX /= MM_PER_INCH;
    targetY /= MM_PER_INCH;
    offsetI /= MM_PER_INCH;
    offsetJ /= MM_PER_INCH;
  }
  
  // Calculate arc center
  float centerX = currentX + offsetI;
  float centerY = currentY + offsetJ;
  
  // Calculate radius
  float radius = sqrt(offsetI*offsetI + offsetJ*offsetJ);
  
  // Calculate angles
  float startAngle = atan2(currentY - centerY, currentX - centerX);
  float endAngle = atan2(targetY - centerY, targetX - centerX);
  
  // Calculate sweep angle
  float sweep;
  if (clockwise) {
    sweep = endAngle - startAngle;
    if (sweep > 0) sweep -= 2.0 * PI;
  } else {
    sweep = endAngle - startAngle;
    if (sweep < 0) sweep += 2.0 * PI;
  }
  
  // Calculate number of segments
  float arcLength = abs(radius * sweep);
  int numSegments = max(4, (int)(arcLength * ARC_SEGMENTS_PER_INCH));
  
  Serial.print(clockwise ? "G2 " : "G3 ");
  Serial.print("Arc (");
  Serial.print(numSegments);
  Serial.println(" segments)");
  
  // Generate waypoints along arc
  for (int i = 1; i <= numSegments; i++) {
    float t = (float)i / (float)numSegments;
    float angle = startAngle + t * sweep;
    
    float x = centerX + radius * cos(angle);
    float y = centerY + radius * sin(angle);
    
    if (!moveTo(x, y)) {
      Serial.println("ERROR: Arc waypoint unreachable");
      return;
    }
  }
}

// ============================================================================
// OTHER G-CODE COMMANDS
// ============================================================================

void executeDwell(String line) {
  bool hasP;
  float duration = extractParam(line, 'P', hasP);
  
  if (hasP) {
    Serial.print("G4 P");
    Serial.print(duration);
    Serial.println(" (Dwell)");
    delay((int)(duration * 1000));
  }
}

void executeHome(String line) {
  Serial.println("G28 (Home)");
  goHome();
}

void executeSetPosition(String line) {
  bool hasX, hasY;
  float x = extractParam(line, 'X', hasX);
  float y = extractParam(line, 'Y', hasY);
  
  if (hasX || hasY) {
    // Set coordinate offset
    if (hasX) offsetX = currentX - x;
    if (hasY) offsetY = currentY - y;
    
    Serial.print("G92 X");
    Serial.print(x);
    Serial.print(" Y");
    Serial.print(y);
    Serial.println(" (Set offset)");
  } else {
    // Clear offset
    offsetX = 0.0;
    offsetY = 0.0;
    Serial.println("G92 (Clear offset)");
  }
}

// ============================================================================
// MAIN G-CODE PROCESSOR
// ============================================================================

void processGcodeLine(String line) {
  // Remove whitespace and convert to uppercase
  line.trim();
  line.toUpperCase();
  
  // Skip empty lines and comments
  if (line.length() == 0) return;
  if (line.startsWith(";") || line.startsWith("(")) return;
  
  // Remove inline comments
  int commentIdx = line.indexOf(';');
  if (commentIdx != -1) {
    line = line.substring(0, commentIdx);
    line.trim();
  }
  
  linesProcessed++;
  
  // Process commands
  if (hasCommand(line, "G0")) {
    executeLinearMove(line, true);
  }
  else if (hasCommand(line, "G1")) {
    executeLinearMove(line, false);
  }
  else if (hasCommand(line, "G2")) {
    executeArcMove(line, true);
  }
  else if (hasCommand(line, "G3")) {
    executeArcMove(line, false);
  }
  else if (hasCommand(line, "G4")) {
    executeDwell(line);
  }
  else if (hasCommand(line, "G20")) {
    inchMode = true;
    Serial.println("G20 (Inches)");
  }
  else if (hasCommand(line, "G21")) {
    inchMode = false;
    Serial.println("G21 (Millimeters)");
  }
  else if (hasCommand(line, "G28")) {
    executeHome(line);
  }
  else if (hasCommand(line, "G90")) {
    absoluteMode = true;
    Serial.println("G90 (Absolute)");
  }
  else if (hasCommand(line, "G91")) {
    absoluteMode = false;
    Serial.println("G91 (Incremental)");
  }
  else if (hasCommand(line, "G92")) {
    executeSetPosition(line);
  }
  else if (hasCommand(line, "M2")) {
    Serial.println("M2 (Program End)");
    // Will exit loop in main
  }
  else if (hasCommand(line, "M6")) {
    Serial.println("M6 (Tool Change - Ignored)");
  }
  else {
    Serial.print("UNKNOWN: ");
    Serial.println(line);
  }
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  delay(500);
  
  // Header
  Serial.println();
  Serial.println("========================================");
  Serial.println("  SCARA Direct G-code Parser v3.0");
  Serial.println("  Arduino Nano + SD Card");
  Serial.println("========================================");
  Serial.println();
  
  // Robot Configuration
  Serial.println("Robot Configuration:");
  Serial.print("  Arm L1: ");
  Serial.print(ARM_L1);
  Serial.println(" inches");
  Serial.print("  Arm L2: ");
  Serial.print(ARM_L2);
  Serial.println(" inches");
  Serial.print("  Max Reach: ");
  Serial.print(ARM_L1 + ARM_L2);
  Serial.println(" inches");
  Serial.println();
  
  // SD Card Init
  Serial.print("Initializing SD card... ");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("FAILED!");
    Serial.println("Check wiring and SD card");
    while (1);
  }
  Serial.println("OK");
  
  // Check for G-code file
  Serial.print("Looking for ");
  Serial.print(GCODE_FILE);
  Serial.print("... ");
  
  if (!SD.exists(GCODE_FILE)) {
    Serial.println("NOT FOUND!");
    Serial.println();
    Serial.println("Please create program.gcode on SD card");
    Serial.println("Example:");
    Serial.println("  G90 G20");
    Serial.println("  G0 X4 Y4");
    Serial.println("  G1 X6 Y4 F2");
    Serial.println("  G28");
    Serial.println("  M2");
    while (1);
  }
  Serial.println("FOUND");
  Serial.println();
  
  // Servo Init
  Serial.println("Initializing servos...");
  axis1.attach(SERVO1_PIN);
  axis2.attach(SERVO2_PIN);
  goHome();
  Serial.println();
  
  Serial.println("========================================");
  Serial.println("  READY - Starting Execution");
  Serial.println("========================================");
  Serial.println();
  delay(1000);
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  // Open G-code file
  gcodeFile = SD.open(GCODE_FILE);
  
  if (!gcodeFile) {
    Serial.println("ERROR: Cannot open G-code file");
    while (1);
  }
  
  Serial.println("Executing G-code program...");
  Serial.println("----------------------------------------");
  
  // Reset statistics
  linesProcessed = 0;
  movesExecuted = 0;
  errorsEncountered = 0;
  
  // Process each line
  while (gcodeFile.available()) {
    String line = gcodeFile.readStringUntil('\n');
    processGcodeLine(line);
    
    // Check for program end
    if (hasCommand(line, "M2")) {
      break;
    }
  }
  
  gcodeFile.close();
  
  // Report results
  Serial.println("----------------------------------------");
  Serial.println();
  Serial.println("========================================");
  Serial.println("  EXECUTION COMPLETE");
  Serial.println("========================================");
  Serial.print("Lines processed: ");
  Serial.println(linesProcessed);
  Serial.print("Moves executed: ");
  Serial.println(movesExecuted);
  Serial.print("Errors: ");
  Serial.println(errorsEncountered);
  Serial.println();
  
  // Return home
  goHome();
  
  Serial.println("Program finished. Reset to run again.");
  
  // Blink LED
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  
  // Stop
  while (1) {
    delay(1000);
  }
}

// ============================================================================
// END OF PROGRAM
// ============================================================================
