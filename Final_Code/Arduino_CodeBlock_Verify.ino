/*
 * SCARA Robot — Serial Monitor G-code Verifier
 *
 * Type G-code commands into Serial Monitor.
 * Arduino parses them and prints:
 *   - Mode confirmations  (G90, G91, G20, G21)
 *   - Motion output       (target X/Y + calculated servo angles)
 *   - Arc info            (centre, radius, step count)
 *   - Dwell confirmation  (pause duration)
 *   - Home confirmation   (G28 angles)
 *   - Error messages      (out of reach, bad syntax)
 *   - Tool / end messages (M6, M2)
 *
 * NO SD card or servos needed — runs on bare Arduino Nano.
 *
 * Open Serial Monitor at 9600 baud, type a command, press Enter.
 *
 * ============================================================================
 * QUICK TEST SCRIPT  (paste these one at a time or all at once)
 * ============================================================================
 *   G90          → [MODE] Absolute positioning
 *   G20          → [MODE] Units: inches
 *   G0 X4 Y4     → [G0]  Target (4.00, 4.00) | Angles: θ1=__° θ2=__°
 *   G1 X6 Y4 F2  → [G1]  Target (6.00, 4.00) | Feed: 2.00 in/s | Angles: ...
 *   G1 X15 Y15   → [ERROR] Out of reach ...
 *   G91          → [MODE] Incremental positioning
 *   G1 X1 Y0     → [G1]  Delta (1.00, 0.00) | Target (...) | Angles: ...
 *   G90          → [MODE] Absolute positioning
 *   G2 X6 Y4 I-1 J0 → [G2]  CW arc | Centre (...) | Radius ... | N steps
 *   G3 X4 Y4 I0 J-1 → [G3]  CCW arc ...
 *   G4 P1.5      → [G4]  Dwell 1.50 s
 *   G28          → [G28] Home | Angles: θ1=__° θ2=__°
 *   G92 X0 Y0    → [G92] Offset set ...
 *   G21          → [MODE] Units: millimeters
 *   G1 X100 Y100 → [G1]  (converts mm→in) ...
 *   M6           → [M6]  Tool change acknowledged
 *   M2           → [M2]  Program end
 *   HELLO        → [ERROR] Unknown command: HELLO
 * ============================================================================
 */

#include <math.h>

// ============================================================================
// CONFIGURATION  —  must match your physical robot
// ============================================================================

const float ARM_L1      = 6.0;   // inches
const float ARM_L2      = 6.0;   // inches
const float HOME_X      = 6.0;
const float HOME_Y      = 6.0;
const float MM_PER_INCH = 25.4;
const int   ARC_STEPS   = 8;     // arc segments per inch

// ============================================================================
// PARSER STATE
// ============================================================================

float currentX     = HOME_X;
float currentY     = HOME_Y;
bool  absoluteMode = true;
bool  inchMode     = true;
float offsetX      = 0.0;
float offsetY      = 0.0;
float feedRate     = 2.0;

// ============================================================================
// INVERSE KINEMATICS
// ============================================================================

bool calcAngles(float x, float y, float &a1, float &a2) {
  float r    = sqrt(x*x + y*y);
  float rMax = ARM_L1 + ARM_L2;
  float rMin = abs(ARM_L1 - ARM_L2);

  if (r > rMax) {
    Serial.print("[ERROR] Out of reach: distance ");
    Serial.print(r, 3);
    Serial.print("\" exceeds max reach ");
    Serial.print(rMax, 2);
    Serial.println("\"");
    return false;
  }
  if (r < rMin) {
    Serial.print("[ERROR] Too close: distance ");
    Serial.print(r, 3);
    Serial.print("\" below min reach ");
    Serial.print(rMin, 2);
    Serial.println("\"");
    return false;
  }

  float cosT2 = (x*x + y*y - ARM_L1*ARM_L1 - ARM_L2*ARM_L2)
                / (2.0 * ARM_L1 * ARM_L2);
  cosT2 = constrain(cosT2, -1.0, 1.0);

  float t2 = acos(cosT2);
  float k1 = ARM_L1 + ARM_L2 * cos(t2);
  float k2 = ARM_L2 * sin(t2);
  float t1 = atan2(y, x) - atan2(k2, k1);

  a1 = degrees(t1);
  a2 = degrees(t2);
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
  if (num.length()) { found = true; return num.toFloat(); }
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
// COMMAND HANDLERS  (print verification output, no hardware calls)
// ============================================================================

// ---- G0 / G1 ---------------------------------------------------------------
void handleMove(const String &line, bool rapid) {
  bool hX, hY, hF;
  float px = getParam(line, 'X', hX);
  float py = getParam(line, 'Y', hY);
  float pf = getParam(line, 'F', hF);

  if (hF) feedRate = pf;

  if (!hX && !hY) {
    Serial.println("[ERROR] G0/G1 requires at least one of X or Y");
    return;
  }

  // Resolve target in inches
  float tx = currentX, ty = currentY;

  if (absoluteMode) {
    float rx = hX ? px : currentX + offsetX;
    float ry = hY ? py : currentY + offsetY;
    if (!inchMode) { rx /= MM_PER_INCH; ry /= MM_PER_INCH; }
    tx = rx - offsetX;
    ty = ry - offsetY;
  } else {
    float dx = hX ? px : 0;
    float dy = hY ? py : 0;
    if (!inchMode) { dx /= MM_PER_INCH; dy /= MM_PER_INCH; }
    tx = currentX + dx;
    ty = currentY + dy;
  }

  // Print header
  Serial.print(rapid ? "[G0]  Rapid  " : "[G1]  Feed   ");

  if (!absoluteMode) {
    Serial.print("Delta (");
    Serial.print(hX ? px : 0, 3); Serial.print(", ");
    Serial.print(hY ? py : 0, 3);
    Serial.print(inchMode ? "\" )" : " mm)");
    Serial.print("  →  ");
  }

  Serial.print("Target (");
  Serial.print(tx, 3); Serial.print("\", ");
  Serial.print(ty, 3); Serial.print("\")");

  if (!rapid && hF) {
    Serial.print("  Feed: ");
    Serial.print(feedRate, 2);
    Serial.print(inchMode ? " in/s" : " mm/s");
  }

  // Inverse kinematics
  float a1, a2;
  if (calcAngles(tx, ty, a1, a2)) {
    Serial.print("  |  θ1=");
    Serial.print(a1, 1);
    Serial.print("°  θ2=");
    Serial.print(a2, 1);
    Serial.println("°");
    currentX = tx;
    currentY = ty;
  } else {
    Serial.println();   // error already printed inside calcAngles
  }
}

// ---- G2 / G3 ---------------------------------------------------------------
void handleArc(const String &line, bool clockwise) {
  bool hX, hY, hI, hJ;
  float ex = getParam(line, 'X', hX);
  float ey = getParam(line, 'Y', hY);
  float oi = getParam(line, 'I', hI);
  float oj = getParam(line, 'J', hJ);

  if (!hI && !hJ) {
    Serial.println("[ERROR] G2/G3 requires I and/or J (arc centre offset)");
    return;
  }

  float tx = absoluteMode ? ex - offsetX : currentX + ex;
  float ty = absoluteMode ? ey - offsetY : currentY + ey;

  if (!inchMode) { tx /= MM_PER_INCH; ty /= MM_PER_INCH; }

  float cx = currentX + oi;
  float cy = currentY + oj;

  float radius     = sqrt(oi*oi + oj*oj);
  float startAngle = atan2(currentY - cy, currentX - cx);
  float endAngle   = atan2(ty        - cy, tx        - cx);

  float sweep;
  if (clockwise) {
    sweep = endAngle - startAngle;
    if (sweep > 0) sweep -= TWO_PI;
  } else {
    sweep = endAngle - startAngle;
    if (sweep < 0) sweep += TWO_PI;
  }

  float arcLen = abs(radius * sweep);
  int   nSteps = max(4, (int)(arcLen * ARC_STEPS));

  Serial.print(clockwise ? "[G2]  CW arc   " : "[G3]  CCW arc  ");
  Serial.print("Centre (");
  Serial.print(cx, 3); Serial.print("\", ");
  Serial.print(cy, 3); Serial.print("\")  Radius ");
  Serial.print(radius, 3); Serial.print("\"  Arc ");
  Serial.print(arcLen, 3); Serial.print("\"  Steps: ");
  Serial.println(nSteps);

  // Verify every waypoint is reachable
  int errors = 0;
  for (int i = 1; i <= nSteps; i++) {
    float t     = (float)i / nSteps;
    float angle = startAngle + t * sweep;
    float wx    = cx + radius * cos(angle);
    float wy    = cy + radius * sin(angle);
    float a1, a2;
    if (!calcAngles(wx, wy, a1, a2)) errors++;
  }

  if (errors == 0) {
    Serial.print("  All "); Serial.print(nSteps);
    Serial.println(" waypoints reachable ✓");
    currentX = tx;
    currentY = ty;
  } else {
    Serial.print("  [ERROR] ");
    Serial.print(errors);
    Serial.println(" waypoints out of reach");
  }
}

// ---- G4 — dwell ------------------------------------------------------------
void handleDwell(const String &line) {
  bool hP;
  float secs = getParam(line, 'P', hP);
  if (!hP || secs <= 0) {
    Serial.println("[ERROR] G4 requires P > 0  (e.g. G4 P1.5)");
    return;
  }
  Serial.print("[G4]  Dwell ");
  Serial.print(secs, 2);
  Serial.print(" s  (");
  Serial.print((int)(secs * 1000));
  Serial.println(" ms)");
}

// ---- G28 — home ------------------------------------------------------------
void handleHome() {
  float a1, a2;
  Serial.print("[G28] Home  Target (");
  Serial.print(HOME_X, 2); Serial.print("\", ");
  Serial.print(HOME_Y, 2); Serial.print("\")");

  if (calcAngles(HOME_X, HOME_Y, a1, a2)) {
    Serial.print("  |  θ1=");
    Serial.print(a1, 1);
    Serial.print("°  θ2=");
    Serial.print(a2, 1);
    Serial.println("°");
    currentX = HOME_X;
    currentY = HOME_Y;
  } else {
    Serial.println();
  }
}

// ---- G92 — set position offset ---------------------------------------------
void handleSetPos(const String &line) {
  bool hX, hY;
  float px = getParam(line, 'X', hX);
  float py = getParam(line, 'Y', hY);

  if (!hX && !hY) {
    offsetX = 0; offsetY = 0;
    Serial.println("[G92] Offset cleared  (0.00, 0.00)");
    return;
  }
  if (hX) offsetX = currentX - px;
  if (hY) offsetY = currentY - py;
  Serial.print("[G92] Position set to (");
  Serial.print(hX ? px : currentX + offsetX, 3); Serial.print("\", ");
  Serial.print(hY ? py : currentY + offsetY, 3); Serial.print("\")  ");
  Serial.print("Offset: (");
  Serial.print(offsetX, 3); Serial.print(", ");
  Serial.print(offsetY, 3); Serial.println(")");
}

// ============================================================================
// MAIN LINE DISPATCHER
// ============================================================================

void processLine(String line) {
  line.trim();
  line.toUpperCase();

  if (line.length() == 0 || line.charAt(0) == ';') return;

  int ci = line.indexOf(';');
  if (ci != -1) { line = line.substring(0, ci); line.trim(); }
  if (line.length() == 0) return;

  // Echo input
  Serial.println();
  Serial.print(">> "); Serial.println(line);

  // Dispatch
  if      (hasCmd(line, "G0"))  handleMove(line, true);
  else if (hasCmd(line, "G1"))  handleMove(line, false);
  else if (hasCmd(line, "G2"))  handleArc(line, true);
  else if (hasCmd(line, "G3"))  handleArc(line, false);
  else if (hasCmd(line, "G4"))  handleDwell(line);
  else if (hasCmd(line, "G20")) {
    inchMode = true;
    Serial.println("[MODE] Units: inches");
  }
  else if (hasCmd(line, "G21")) {
    inchMode = false;
    Serial.println("[MODE] Units: millimeters");
  }
  else if (hasCmd(line, "G28")) handleHome();
  else if (hasCmd(line, "G90")) {
    absoluteMode = true;
    Serial.println("[MODE] Absolute positioning");
  }
  else if (hasCmd(line, "G91")) {
    absoluteMode = false;
    Serial.println("[MODE] Incremental positioning");
  }
  else if (hasCmd(line, "G92")) handleSetPos(line);
  else if (hasCmd(line, "M6"))  Serial.println("[M6]  Tool change acknowledged");
  else if (hasCmd(line, "M2"))  {
    Serial.println("[M2]  Program end");
    Serial.println("      (Reset Arduino or keep typing to continue)");
  }
  else {
    Serial.print("[ERROR] Unknown command: ");
    Serial.println(line);
  }

  // Print state after every command
  Serial.print("      State: ");
  Serial.print(absoluteMode ? "ABS" : "INC");
  Serial.print("  ");
  Serial.print(inchMode ? "IN" : "MM");
  Serial.print("  Pos (");
  Serial.print(currentX, 2); Serial.print("\", ");
  Serial.print(currentY, 2); Serial.println("\")");
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(9600);
  while (!Serial);   // wait for Serial Monitor to open

  Serial.println("========================================");
  Serial.println("  SCARA G-code Verifier");
  Serial.println("  Type commands into Serial Monitor");
  Serial.println("========================================");
  Serial.print("  Arm: L1="); Serial.print(ARM_L1);
  Serial.print("\"  L2="); Serial.print(ARM_L2);
  Serial.print("\"  MaxReach="); Serial.print(ARM_L1+ARM_L2);
  Serial.println("\"");
  Serial.print("  Home: ("); Serial.print(HOME_X);
  Serial.print(", "); Serial.print(HOME_Y); Serial.println(")");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Commands: G0 G1 G2 G3 G4 G20 G21 G28");
  Serial.println("          G90 G91 G92 M2 M6");
  Serial.println();
  Serial.println("Output format:");
  Serial.println("  [MODE]  confirmation messages");
  Serial.println("  [G0/G1] target + servo angles");
  Serial.println("  [G2/G3] arc info + reachability");
  Serial.println("  [G4]    dwell duration");
  Serial.println("  [G28]   home angles");
  Serial.println("  [ERROR] out-of-reach or bad syntax");
  Serial.println();
  Serial.println("Type a command and press Enter:");
  Serial.println("----------------------------------------");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    processLine(line);
  }
}
