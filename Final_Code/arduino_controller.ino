/*
 * SCARA Robot Controller - Arduino SD Card Reader
 * 
 * This program reads servo angle pairs from an SD card file and 
 * controls a 2-axis SCARA robotic arm.
 * 
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, etc.)
 * - SD card module connected via SPI
 * - Two servo motors (connected to pins 5 and 6)
 * - SD card with "moves.txt" file containing angle pairs
 * 
 * File Format (moves.txt):
 * Each line contains two comma-separated integers: "angle1,angle2"
 * Example:
 *   75,120
 *   80,115
 *   85,110
 * 
 * Servo Connections:
 * - Servo 1 (axis1): Pin 6 (shoulder joint)
 * - Servo 2 (axis2): Pin 5 (elbow joint)
 * - SD Card CS: Pin 10
 * 
 * Author: Robot Team
 * Version: 2.0
 */

#include <SD.h>
#include <Servo.h>
#include <SPI.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// SD Card Configuration
const int SD_CS_PIN = 10;           // Chip select pin for SD card
const char* MOVES_FILE = "moves.txt"; // File containing servo angles

// Servo Pins
const int SERVO1_PIN = 6;           // Shoulder servo pin
const int SERVO2_PIN = 5;           // Elbow servo pin

// Motion Parameters
const int STEP_DELAY = 15;          // Delay between servo steps (milliseconds)
                                    // Lower = faster but jerkier
                                    // Higher = slower but smoother

// Home Position (starting position)
const int HOME_POS1 = 75;           // Servo 1 home angle
const int HOME_POS2 = 120;          // Servo 2 home angle

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

File movesFile;                     // SD card file handle
Servo axis1;                        // Servo 1 object (shoulder)
Servo axis2;                        // Servo 2 object (elbow)

int currentPos1 = HOME_POS1;        // Current servo 1 position
int currentPos2 = HOME_POS2;        // Current servo 2 position

// ============================================================================
// FUNCTIONS
// ============================================================================

/**
 * Move servos smoothly from current position to target position
 * 
 * @param target1 - Target angle for servo 1 (0-180)
 * @param target2 - Target angle for servo 2 (0-180)
 * @param stepDelay - Delay between each 1-degree step (milliseconds)
 * 
 * How it works:
 * - Increments/decrements each servo by 1 degree per step
 * - Continues until both servos reach their target positions
 * - Provides smooth motion instead of instant jumps
 */
void moveServos(int target1, int target2, int stepDelay) {
  // Continue moving until both servos reach target
  while (currentPos1 != target1 || currentPos2 != target2) {
    
    // Move servo 1 one step closer to target
    if (currentPos1 < target1) {
      currentPos1++;                // Increment if below target
    } else if (currentPos1 > target1) {
      currentPos1--;                // Decrement if above target
    }
    
    // Move servo 2 one step closer to target
    if (currentPos2 < target2) {
      currentPos2++;                // Increment if below target
    } else if (currentPos2 > target2) {
      currentPos2--;                // Decrement if above target
    }
    
    // Write new positions to servos
    axis1.write(currentPos1);
    axis2.write(currentPos2);
    
    // Wait before next step (controls motion speed)
    delay(stepDelay);
  }
}

/**
 * Move to home position
 * Used at startup and after program completion
 */
void goHome() {
  moveServos(HOME_POS1, HOME_POS2, STEP_DELAY);
}

/**
 * Parse a line from the file and extract two angle values
 * 
 * @param line - String containing "angle1,angle2"
 * @param angle1 - Output: First angle value
 * @param angle2 - Output: Second angle value
 * @return true if parsing successful, false if error
 */
bool parseLine(String line, int &angle1, int &angle2) {
  line.trim();  // Remove leading/trailing whitespace
  
  // Skip empty lines
  if (line.length() == 0) {
    return false;
  }
  
  // Find comma separator
  int commaIndex = line.indexOf(',');
  
  // Check if comma exists
  if (commaIndex < 0) {
    Serial.println("Error: No comma found in line");
    return false;
  }
  
  // Extract angle values
  String angle1Str = line.substring(0, commaIndex);
  String angle2Str = line.substring(commaIndex + 1);
  
  // Convert to integers
  angle1 = angle1Str.toInt();
  angle2 = angle2Str.toInt();
  
  // Validate angle ranges (servos work 0-180 degrees)
  if (angle1 < 0 || angle1 > 180 || angle2 < 0 || angle2 > 180) {
    Serial.println("Error: Angle out of range (0-180)");
    return false;
  }
  
  return true;
}

/**
 * Blink LED to indicate error
 * 
 * @param count - Number of blinks
 * @param delayMs - Delay between blinks
 */
void errorBlink(int count, int delayMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayMs);
  }
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // Initialize serial communication for debugging (optional)
  Serial.begin(9600);
  
  // Wait for serial port to connect (for Arduino Leonardo, etc.)
  // Comment out these lines if not using Serial Monitor
  /*
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  */
  
  Serial.println("===========================================");
  Serial.println("SCARA Robot Controller - SD Card Mode");
  Serial.println("===========================================");
  
  // Setup LED pin for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize SD card
  Serial.print("Initializing SD card...");
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(" FAILED!");
    Serial.println("Check:");
    Serial.println("- SD card is inserted");
    Serial.println("- Wiring connections (CS pin 10)");
    Serial.println("- Card is formatted (FAT16 or FAT32)");
    
    // Blink LED rapidly to indicate SD card error
    while (1) {
      errorBlink(5, 100);
      delay(1000);
    }
  }
  
  Serial.println(" OK");
  
  // Check if moves.txt file exists
  if (!SD.exists(MOVES_FILE)) {
    Serial.print("Error: File '");
    Serial.print(MOVES_FILE);
    Serial.println("' not found on SD card!");
    
    // Blink LED slowly to indicate file not found
    while (1) {
      errorBlink(3, 200);
      delay(1000);
    }
  }
  
  // Attach servos to pins
  Serial.println("Attaching servos...");
  axis1.attach(SERVO1_PIN);
  axis2.attach(SERVO2_PIN);
  
  // Move to home position
  Serial.println("Moving to home position...");
  goHome();
  
  delay(500);  // Wait half second before starting
  
  Serial.println("Ready to execute moves!");
  Serial.println("===========================================");
  Serial.println();
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  // Open the moves file
  movesFile = SD.open(MOVES_FILE);
  
  if (!movesFile) {
    Serial.println("Error: Cannot open moves file");
    errorBlink(5, 200);
    while (1);  // Stop execution
  }
  
  Serial.println("Executing moves from SD card...");
  int lineNumber = 0;
  int movesExecuted = 0;
  
  // Read and execute each line
  while (movesFile.available()) {
    String line = movesFile.readStringUntil('\n');
    lineNumber++;
    
    int target1, target2;
    
    // Parse the line
    if (parseLine(line, target1, target2)) {
      // Print move information
      Serial.print("Move ");
      Serial.print(movesExecuted + 1);
      Serial.print(": ");
      Serial.print(target1);
      Serial.print(", ");
      Serial.println(target2);
      
      // Execute the move
      moveServos(target1, target2, STEP_DELAY);
      
      movesExecuted++;
    } else {
      // Skip invalid lines (empty or malformed)
      if (line.length() > 0) {
        Serial.print("Skipping line ");
        Serial.print(lineNumber);
        Serial.print(": ");
        Serial.println(line);
      }
    }
  }
  
  // Close the file
  movesFile.close();
  
  // Report completion
  Serial.println();
  Serial.println("===========================================");
  Serial.print("Execution complete! ");
  Serial.print(movesExecuted);
  Serial.println(" moves executed.");
  Serial.println("===========================================");
  
  // Return to home position
  Serial.println("Returning to home position...");
  goHome();
  
  // Blink LED to indicate completion
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  
  Serial.println("Program finished. Reset Arduino to run again.");
  
  // Stop here (don't loop again)
  while (1) {
    delay(1000);
  }
}

// ============================================================================
// END OF PROGRAM
// ============================================================================
