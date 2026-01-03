// Made by Bart van der Haagen 2 januari 2026 
// Project Goliath_7 Stepper motor driver - SEPARATED ARM/RAIL VERSION
// TMC 5160 connected with SPI  
// made for: STM32F411CUE6 

#include <Arduino.h>
#include <TMC5160.h>
#include <SPI.h>
#include <HardwareSerial.h>

// Serial over UART - UART1 on PA9(TX), PA10(RX)
HardwareSerial SerialUART(USART1);  // connection USB to computer

// Serial over UART - UART1 on PA2(TX), PA3(RX)
HardwareSerial SerialUART2(USART2);

// STM32F411CEU6 SPI pins - SPI1
#define MOSI_PIN PA7
#define MISO_PIN PA6  
#define SCK_PIN  PA5

// MOTOR PINS - All 7 motors
const uint8_t SPI_CS1 = PB0;   const uint8_t DRV_ENN1 = PB1;   // M1 - NEMA17 1.2A
const uint8_t SPI_CS2 = PB10;  const uint8_t DRV_ENN2 = PB12;  // M2 - NEMA23 2.5A
const uint8_t SPI_CS3 = PB13;  const uint8_t DRV_ENN3 = PB14;  // M3 - NEMA23 2.0A
const uint8_t SPI_CS4 = PB15;  const uint8_t DRV_ENN4 = PB2;   // M4 - NEMA17 1.2A
const uint8_t SPI_CS5 = PC14;   const uint8_t DRV_ENN5 = PB4;   // M5 - NEMA17 1.2A
const uint8_t SPI_CS6 = PB5;   const uint8_t DRV_ENN6 = PB6;   // M6 - NEMA17 1.2A
const uint8_t SPI_CS7 = PB7;   const uint8_t DRV_ENN7 = PB8;   // M7 - NEMA23 2.0A (Slider/RAIL)

// LIMIT SWITCH PINS - Input pullup (LOW when switch pressed)
const uint8_t LIMIT_M1 = PC15;  // M1 - dir - to reach limit
const uint8_t LIMIT_M2 = PA8;   // M2 - dir - to reach limit  
const uint8_t LIMIT_M3 = PA4;   // M3 - dir + to reach limit
const uint8_t LIMIT_M4 = PB3;   // M4 - dir - to reach limit
const uint8_t LIMIT_M5 = PA15;  // M5 - dir - to reach limit
const uint8_t LIMIT_M6 = PA1;   // M6 - dir - to reach limit
const uint8_t LIMIT_M7 = PA0;   // M7 - dir - to reach limit (RAIL)

// Motor objects
TMC5160_SPI motor1 = TMC5160_SPI(SPI_CS1);
TMC5160_SPI motor2 = TMC5160_SPI(SPI_CS2);
TMC5160_SPI motor3 = TMC5160_SPI(SPI_CS3);
TMC5160_SPI motor4 = TMC5160_SPI(SPI_CS4);
TMC5160_SPI motor5 = TMC5160_SPI(SPI_CS5);
TMC5160_SPI motor6 = TMC5160_SPI(SPI_CS6);
TMC5160_SPI motor7 = TMC5160_SPI(SPI_CS7);

// Steps per radian conversion (using full rotation steps)
const float STEPS_PER_RAD[7] = {
  24000.0f / (2 * PI),   // M1: steps/rad    
  54400.0f / (2 * PI),   // M2: steps/rad
  20000.0f / (2 * PI),   // M3: steps/rad
  16000.0f / (2 * PI),   // M4: steps/rad
  6000.0f / (2 * PI),    // M5: steps/rad
  6000.0f / (2 * PI),    // M6: steps/rad
  120000.0f / (2 * PI)   // M7: steps/rad (RAIL)
};

// Motor configurations - SEPARATED ARM & RAIL
struct MotorConfig {
  const char* name;
  TMC5160_SPI* motor;
  uint8_t csPin;
  uint8_t ennPin;
  uint8_t limitPin;
  uint8_t globalScaler;
  uint8_t irun;
  uint8_t ihold;
  const char* type;
  float maxSpeed;        // Radians per second
  float homingSpeed;     // Radians per second used for homing
  float acceleration;    // Radians per second²
  int homingDirection;   // +1 or -1 for homing direction
  float homingBackoff;   // Steps to back off after homing
};

// ARM MOTORS (M1-M6)
MotorConfig armMotors[6] = {
  {"M1", &motor1, SPI_CS1, DRV_ENN1, LIMIT_M1, 200, 25, 10, "NEMA17 1.8A", 
   1000.0f/STEPS_PER_RAD[0], 400.0f/STEPS_PER_RAD[0], 1000.0f/STEPS_PER_RAD[0], 1, 11525.0f},
  
  {"M2", &motor2, SPI_CS2, DRV_ENN2, LIMIT_M2, 240, 31, 15, "NEMA23 2.5A", 
   1000.0f/STEPS_PER_RAD[1], 400.0f/STEPS_PER_RAD[1], 1000.0f/STEPS_PER_RAD[1], 1, 13600.0f},
  
  {"M3", &motor3, SPI_CS3, DRV_ENN3, LIMIT_M3, 240, 25, 12, "NEMA23 2.0A", 
   1000.0f/STEPS_PER_RAD[2], 400.0f/STEPS_PER_RAD[2], 1000.0f/STEPS_PER_RAD[2], 1, 8400.0f},
  
  {"M4", &motor4, SPI_CS4, DRV_ENN4, LIMIT_M4, 200, 20, 10, "NEMA17 1.2A", 
   1000.0f/STEPS_PER_RAD[3], 400.0f/STEPS_PER_RAD[3], 1000.0f/STEPS_PER_RAD[3], 1, 6650.0f},
  
  {"M5", &motor5, SPI_CS5, DRV_ENN5, LIMIT_M5, 200, 25, 10, "NEMA17 1.2A", 
   1000.0f/STEPS_PER_RAD[4], 400.0f/STEPS_PER_RAD[4], 1000.0f/STEPS_PER_RAD[4], -1, 1870.0f},
  
  {"M6", &motor6, SPI_CS6, DRV_ENN6, LIMIT_M6, 200, 20, 10, "NEMA17 1.2A", 
   1000.0f/STEPS_PER_RAD[5], 400.0f/STEPS_PER_RAD[5], 1000.0f/STEPS_PER_RAD[5], -1, 3270.0f}
};

// RAIL MOTOR (M7/Slider)
MotorConfig railMotor = {
  "R1", &motor7, SPI_CS7, DRV_ENN7, LIMIT_M7, 240, 31, 10, "NEMA23 2.0A", 
  1000.0f/STEPS_PER_RAD[6], 400.0f/STEPS_PER_RAD[6], 1000.0f/STEPS_PER_RAD[6], 1, 6400.0f
};

const int NUM_ARM_MOTORS = 6;

// System States
enum SystemState {
  STATE_READY,
  STATE_MOVING,
  STATE_HOMING,
  STATE_UNCALIBRATED
};

// Global variables
SystemState systemState = STATE_UNCALIBRATED;
unsigned long lastFeedbackTime = 0;
const unsigned long FEEDBACK_INTERVAL = 10; // 10ms feedback

// Motion control variables - SEPARATED
struct ArmMotionProfile {
  float startPositions[6];  // In radians
  float targetPositions[6]; // In radians
  float distances[6];       // In radians
  float maxDistance;        // In radians
  bool moving;
  unsigned long moveStartTime;
  unsigned long estimatedMoveTime;
};

struct RailMotionProfile {
  float startPosition;
  float targetPosition;
  float distance;
  bool moving;
  unsigned long moveStartTime;
};

ArmMotionProfile currentArmMove;
RailMotionProfile currentRailMove;
bool combinedMove = false; // Flag for M command

// Function prototypes
void initializeSystem();
void setArmMove(float targets[6]);            // Arm only (6 joints)
void setRailMove(float target);               // Rail only (1 position)
void setCombinedMove(float targets[7]);       // Combined arm+rail (backward compatibility)
void updateArmMotion();
void updateRailMotion();
void sendFeedback();
bool allArmMotorsReachedTarget();
bool railMotorReachedTarget();
float calculateRequiredTime(float distance, float maxSpeed, float acceleration);
void homeRobot();
float radiansToSteps(int motorIndex, float radians);
float stepsToRadians(int motorIndex, int32_t steps);

// Helper functions
void parseArmCommand(String command, float targets[6]);
void parseCombinedCommand(String command, float targets[7]);

// Custom math functions
float customMax(float a, float b) {
  return (a > b) ? a : b;
}

float customAbs(float x) {
  return (x < 0) ? -x : x;
}

float customSqrt(float x) {
  if (x <= 0) return 0;
  float guess = x;
  for (int i = 0; i < 10; i++) {
    guess = 0.5f * (guess + x / guess);
  }
  return guess;
}

// Conversion functions
float radiansToSteps(int motorIndex, float radians) {
  return radians * STEPS_PER_RAD[motorIndex];
}

float stepsToRadians(int motorIndex, int32_t steps) {
  return steps / STEPS_PER_RAD[motorIndex];
}

void setup() {
  SerialUART.begin(115200);
  SerialUART2.begin(9600);
  delay(1000);
  
  initializeSystem();
  
  SerialUART.println("==========================================");
  SerialUART.println("7-Axis Coordinated Motion Controller v2.0");
  SerialUART.println("ARM & RAIL SEPARATED VERSION");
  SerialUART.println("==========================================");
  SerialUART.println("Commands:");
  SerialUART.println("  A 1.57,0.78,0.0,0.0,0.0,0.0  - Arm only (6 joints)");
  SerialUART.println("  R 0.5                        - Rail only");
  SerialUART.println("  M 1.57,0.78,0.0,0.0,0.0,0.0,0.5 - Combined (legacy)");
  SerialUART.println("  HOME                         - Homing sequence");
  SerialUART.println("==========================================");
  SerialUART.println();
}

void loop() {
  // Check for new move commands
  if (SerialUART.available()) {
    String command = SerialUART.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("A ")) {
      // Arm only command: A 1.57,0.78,-0.52,0.0,0.0,0.0
      float targets[6];
      parseArmCommand(command, targets);
      setArmMove(targets);
      combinedMove = false;
      SerialUART.print("Arm move started to: ");
      for (int i = 0; i < 6; i++) {
        SerialUART.print(targets[i], 3);
        if (i < 5) SerialUART.print(", ");
      }
      SerialUART.println();
    }
    else if (command.startsWith("R ")) {
      // Rail only command: R 0.5
      float railPos = command.substring(2).toFloat();
      setRailMove(railPos);
      combinedMove = false;
      SerialUART.print("Rail move started to: ");
      SerialUART.println(railPos, 3);
    }
    else if (command.startsWith("M ")) {
      // Combined command (backward compatibility): M 1.57,0.78,-0.52,0.0,0.0,0.0,0.5
      float targets[7];
      parseCombinedCommand(command, targets);
      setCombinedMove(targets);
      combinedMove = true;
      SerialUART.print("Combined move started (legacy mode)");
      SerialUART.println();
    }
    else if (command.startsWith("HOME")) {
      homeRobot();
    }
    else if (command.length() > 0) {
      SerialUART.println("Unknown command. Use: A, R, M, or HOME");
    }
  }

  // Update motion control
  if (currentArmMove.moving) {
    updateArmMotion();
  }
  
  if (currentRailMove.moving) {
    updateRailMotion();
  }
  
  // Send feedback every 10ms
  if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL) {
    sendFeedback();
    lastFeedbackTime = millis();
  }
  
  delay(1);
}

// Command parsing functions
void parseArmCommand(String command, float targets[6]) {
  int startIndex = 2; // Skip "A "
  
  for (int i = 0; i < 6; i++) {
    int commaIndex = command.indexOf(',', startIndex);
    if (commaIndex == -1) commaIndex = command.length();
    
    String numStr = command.substring(startIndex, commaIndex);
    targets[i] = numStr.toFloat(); // This is now in radians
    startIndex = commaIndex + 1;
  }
}

void parseCombinedCommand(String command, float targets[7]) {
  int startIndex = 2; // Skip "M "
  
  for (int i = 0; i < 7; i++) {
    int commaIndex = command.indexOf(',', startIndex);
    if (commaIndex == -1) commaIndex = command.length();
    
    String numStr = command.substring(startIndex, commaIndex);
    targets[i] = numStr.toFloat(); // This is now in radians
    startIndex = commaIndex + 1;
  }
}

// Motion control functions
void setArmMove(float targets[6]) {
  // Store current positions as start positions (in radians)
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    currentArmMove.startPositions[i] = stepsToRadians(i, armMotors[i].motor->getCurrentPosition());
    currentArmMove.targetPositions[i] = targets[i];
    currentArmMove.distances[i] = customAbs(targets[i] - currentArmMove.startPositions[i]);
  }
  
  // Find the motor with the longest travel distance
  currentArmMove.maxDistance = 0;
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    if (currentArmMove.distances[i] > currentArmMove.maxDistance) {
      currentArmMove.maxDistance = currentArmMove.distances[i];
    }
  }
  
  // Calculate individual speeds to ensure simultaneous arrival
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    if (currentArmMove.distances[i] > 0) {
      // Scale speed based on distance (longer distance = faster speed)
      float speedRatio = currentArmMove.distances[i] / currentArmMove.maxDistance;
      float individualSpeed = armMotors[i].maxSpeed * speedRatio;
      
      // Ensure minimum speed for very short moves
      individualSpeed = customMax(individualSpeed, armMotors[i].maxSpeed * 0.1f);
      
      // Convert radians to steps for motor control
      armMotors[i].motor->setMaxSpeed(radiansToSteps(i, individualSpeed));
    }
    
    // Set target position - convert radians to steps
    armMotors[i].motor->setTargetPosition((int32_t)radiansToSteps(i, targets[i]));
  }
  
  // Calculate estimated move time (based on longest move)
  currentArmMove.estimatedMoveTime = (unsigned long)calculateRequiredTime(
    currentArmMove.maxDistance, 
    armMotors[0].maxSpeed,
    armMotors[0].acceleration
  );
  
  currentArmMove.moveStartTime = millis();
  currentArmMove.moving = true;
  systemState = STATE_MOVING;
}

void setRailMove(float target) {
  // Simple independent rail movement
  currentRailMove.startPosition = stepsToRadians(6, railMotor.motor->getCurrentPosition());
  currentRailMove.targetPosition = target;
  currentRailMove.distance = customAbs(target - currentRailMove.startPosition);
  
  // Set rail target position
  int32_t railSteps = (int32_t)radiansToSteps(6, target);
  railMotor.motor->setTargetPosition(railSteps);
  
  currentRailMove.moveStartTime = millis();
  currentRailMove.moving = true;
  systemState = STATE_MOVING;
}

void setCombinedMove(float targets[7]) {
  // For backward compatibility - separate arm and rail
  float armTargets[6];
  for (int i = 0; i < 6; i++) {
    armTargets[i] = targets[i];
  }
  
  setArmMove(armTargets);
  setRailMove(targets[6]);
  combinedMove = true;
}

void updateArmMotion() {
  if (allArmMotorsReachedTarget()) {
    currentArmMove.moving = false;
    
    // Reset to default speeds - convert radians to steps
    for (int i = 0; i < NUM_ARM_MOTORS; i++) {
      armMotors[i].motor->setMaxSpeed(radiansToSteps(i, armMotors[i].maxSpeed));
    }
    
    // If this was part of a combined move, check if rail is also done
    if (!combinedMove && !currentRailMove.moving) {
      systemState = STATE_READY;
    }
  }
}

void updateRailMotion() {
  if (railMotorReachedTarget()) {
    currentRailMove.moving = false;
    
    // Reset rail to default speed
    railMotor.motor->setMaxSpeed(radiansToSteps(6, railMotor.maxSpeed));
    
    // If this was part of a combined move, check if arm is also done
    if (!combinedMove && !currentArmMove.moving) {
      systemState = STATE_READY;
    }
  }
}

// Motor status checking
bool allArmMotorsReachedTarget() {
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    if (!armMotors[i].motor->isTargetPositionReached()) {
      return false;
    }
  }
  return true;
}

bool railMotorReachedTarget() {
  return railMotor.motor->isTargetPositionReached();
}

void sendFeedback() {
  if (systemState == STATE_HOMING) {
    // No feedback during homing
    return;
  }
  
  // Send compact position feedback in radians
  // First 6 arm joints
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    SerialUART.print("A"); 
    SerialUART.print(i+1); 
    SerialUART.print(":");
    
    float position_rad = stepsToRadians(i, armMotors[i].motor->getCurrentPosition());
    SerialUART.print(position_rad, 3);
    
    SerialUART.print(" ");
  }
  
  // Then rail
  SerialUART.print("R:");
  float rail_position_rad = stepsToRadians(6, railMotor.motor->getCurrentPosition());
  SerialUART.print(rail_position_rad, 3);
  
  // State indicator
  SerialUART.print(" S:");
  if (currentArmMove.moving || currentRailMove.moving) {
    SerialUART.print("MOVING");
  } else {
    SerialUART.print("READY");
  }
  
  SerialUART.println();
}

float calculateRequiredTime(float distance, float maxSpeed, float acceleration) {
  // Calculate time for trapezoidal velocity profile (all in radians)
  float accelerationDistance = (maxSpeed * maxSpeed) / (2.0f * acceleration);
  
  if (distance <= 2.0f * accelerationDistance) {
    // Triangular profile
    return 2.0f * customSqrt(distance / acceleration) * 1000.0f; // Convert to milliseconds
  } else {
    // Trapezoidal profile
    float constantSpeedDistance = distance - 2.0f * accelerationDistance;
    float constantSpeedTime = constantSpeedDistance / maxSpeed;
    float accelerationTime = maxSpeed / acceleration;
    return (2.0f * accelerationTime + constantSpeedTime) * 1000.0f; // Convert to milliseconds
  }
}

void initializeSystem() {
  SerialUART.println("Initializing 7-Axis Motion System...");
  SerialUART.println("ARM & RAIL SEPARATED CONFIGURATION");
  SerialUART.println();
  
  // Initialize enable pins for all motors
  // Arm motors first
  SerialUART.println("Setting up ARM motor enable pins...");
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    pinMode(armMotors[i].ennPin, OUTPUT);
    digitalWrite(armMotors[i].ennPin, LOW);
    SerialUART.print("  ");
    SerialUART.print(armMotors[i].name);
    SerialUART.println(" ENABLED");
  }
  
  // Rail motor
  SerialUART.println("Setting up RAIL motor enable pin...");
  pinMode(railMotor.ennPin, OUTPUT);
  digitalWrite(railMotor.ennPin, LOW);
  SerialUART.print("  ");
  SerialUART.print(railMotor.name);
  SerialUART.println(" ENABLED");
  
  // Initialize CS pins
  SerialUART.println("Setting up Chip Select pins...");
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    pinMode(armMotors[i].csPin, OUTPUT);
    digitalWrite(armMotors[i].csPin, HIGH);
  }
  pinMode(railMotor.csPin, OUTPUT);
  digitalWrite(railMotor.csPin, HIGH);
  SerialUART.println("  All CS pins HIGH");
  
  // Initialize limit switches
  SerialUART.println("Setting up limit switches (INPUT_PULLUP)...");
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    pinMode(armMotors[i].limitPin, INPUT_PULLUP);
  }
  pinMode(railMotor.limitPin, INPUT_PULLUP);
  SerialUART.println("  All limit switches ready");
  
  // Initialize SPI
  SerialUART.println("Initializing SPI...");
  SPI.setMOSI(MOSI_PIN);
  SPI.setMISO(MISO_PIN);
  SPI.setSCLK(SCK_PIN);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SerialUART.println("  SPI initialized (MODE3, MSBFIRST, DIV16)");
  
  // Initialize motors
  TMC5160::PowerStageParameters powerParams;
  
  // Initialize arm motors
  SerialUART.println();
  SerialUART.println("Initializing ARM motors...");
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    SerialUART.print("  ");
    SerialUART.print(armMotors[i].name);
    SerialUART.print(" (");
    SerialUART.print(armMotors[i].type);
    SerialUART.print(")... ");
    
    TMC5160::MotorParameters motorParams;
    motorParams.globalScaler = armMotors[i].globalScaler;
    motorParams.irun = armMotors[i].irun;
    motorParams.ihold = armMotors[i].ihold;
    
    bool success = armMotors[i].motor->begin(powerParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
    
    if (success) {
      armMotors[i].motor->setRampMode(TMC5160::POSITIONING_MODE);
      armMotors[i].motor->setMaxSpeed(radiansToSteps(i, armMotors[i].maxSpeed));
      armMotors[i].motor->setAcceleration(radiansToSteps(i, armMotors[i].acceleration));
      armMotors[i].motor->setCurrentPosition(0);
      SerialUART.println("OK");
    } else {
      SerialUART.println("FAILED");
    }
    delay(50);
  }
  
  // Initialize rail motor
  SerialUART.println();
  SerialUART.print("Initializing RAIL motor ");
  SerialUART.print(railMotor.name);
  SerialUART.print(" (");
  SerialUART.print(railMotor.type);
  SerialUART.print(")... ");
  
  TMC5160::MotorParameters railParams;
  railParams.globalScaler = railMotor.globalScaler;
  railParams.irun = railMotor.irun;
  railParams.ihold = railMotor.ihold;
  
  bool railSuccess = railMotor.motor->begin(powerParams, railParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  
  if (railSuccess) {
    railMotor.motor->setRampMode(TMC5160::POSITIONING_MODE);
    railMotor.motor->setMaxSpeed(radiansToSteps(6, railMotor.maxSpeed));
    railMotor.motor->setAcceleration(radiansToSteps(6, railMotor.acceleration));
    railMotor.motor->setCurrentPosition(0);
    SerialUART.println("OK");
  } else {
    SerialUART.println("FAILED");
  }
  
  // Initialize motion profiles
  currentArmMove.moving = false;
  currentRailMove.moving = false;
  combinedMove = false;
  
  for (int i = 0; i < 6; i++) {
    currentArmMove.startPositions[i] = 0;
    currentArmMove.targetPositions[i] = 0;
  }
  
  currentRailMove.startPosition = 0;
  currentRailMove.targetPosition = 0;
  
  SerialUART.println();
  SerialUART.println("==========================================");
  SerialUART.println("System Initialized Successfully!");
  SerialUART.println("==========================================");
  SerialUART.println();
}

// Home Robot function - UPDATED for separated motors
void homeRobot() {
  systemState = STATE_HOMING;
  SerialUART.println("=== ROBUST HOMING STARTED ===");
  SerialUART.println("(ARM & RAIL SEPARATED VERSION)");
  
  // GROUP 1: Arm Motors 2, 3 & 5 (now indices 1, 2, 4)
  SerialUART.println("Homing GROUP 1: M2, M3, M5 (Arm motors)");
  
  // Set positioning mode
  armMotors[1].motor->setRampMode(TMC5160::POSITIONING_MODE);
  armMotors[2].motor->setRampMode(TMC5160::POSITIONING_MODE);
  armMotors[4].motor->setRampMode(TMC5160::POSITIONING_MODE);
  
  // Set homing speed (in steps/sec)
  armMotors[1].motor->setMaxSpeed(400);
  armMotors[2].motor->setMaxSpeed(250);
  armMotors[4].motor->setMaxSpeed(250);
  
  // Move toward limits (using steps directly)
  long target2 = armMotors[1].motor->getCurrentPosition() + (100000 * armMotors[1].homingDirection);
  long target3 = armMotors[2].motor->getCurrentPosition() + (100000 * armMotors[2].homingDirection);
  long target5 = armMotors[4].motor->getCurrentPosition() + (100000 * armMotors[4].homingDirection);
  
  armMotors[1].motor->setTargetPosition(target2);
  armMotors[2].motor->setTargetPosition(target3);
  armMotors[4].motor->setTargetPosition(target5);
  
  // Homing state for GROUP 1
  bool m2Hit = false, m3Hit = false, m5Hit = false;
  bool m2FirstLow = false, m3FirstLow = false, m5FirstLow = false;
  
  unsigned long startTime = millis();
  
  SerialUART.println("Moving toward limits...");
  
  while ((!m2Hit || !m3Hit || !m5Hit) && (millis() - startTime < 30000000)) {
    
    // Check M2 - Two consecutive LOW readings required
    if (!m2Hit) {
      if (digitalRead(armMotors[1].limitPin) == LOW) {
        if (m2FirstLow) {
          // Second consecutive LOW - CONFIRMED HIT
          armMotors[1].motor->setTargetPosition(armMotors[1].motor->getCurrentPosition());
          m2Hit = true;
          SerialUART.println("M2: Limit CONFIRMED (2 consecutive LOW)");
        } else {
          // First LOW reading - mark it but don't stop yet
          m2FirstLow = true;
        }
      } else {
        // Switch is HIGH - reset first low flag
        m2FirstLow = false;
      }
    }
    
    // Check M3 - Two consecutive LOW readings required
    if (!m3Hit) {
      if (digitalRead(armMotors[2].limitPin) == LOW) {
        if (m3FirstLow) {
          // Second consecutive LOW - CONFIRMED HIT
          armMotors[2].motor->setTargetPosition(armMotors[2].motor->getCurrentPosition());
          m3Hit = true;
          SerialUART.println("M3: Limit CONFIRMED (2 consecutive LOW)");
        } else {
          // First LOW reading - mark it but don't stop yet
          m3FirstLow = true;
        }
      } else {
        // Switch is HIGH - reset first low flag
        m3FirstLow = false;
      }
    }
    
    // Check M5 - Two consecutive LOW readings required
    if (!m5Hit) {
      if (digitalRead(armMotors[4].limitPin) == LOW) {
        if (m5FirstLow) {
          // Second consecutive LOW - CONFIRMED HIT
          armMotors[4].motor->setTargetPosition(armMotors[4].motor->getCurrentPosition());
          m5Hit = true;
          SerialUART.println("M5: Limit CONFIRMED (2 consecutive LOW)");
        } else {
          // First LOW reading - mark it but don't stop yet
          m5FirstLow = true;
        }
      } else {
        // Switch is HIGH - reset first low flag
        m5FirstLow = false;
      }
    }
    
    delay(5); // Small delay between checks
  }
  
  // Handle results for GROUP 1
  if (m2Hit && m3Hit && m5Hit) {
    SerialUART.println("Group 1 limits confirmed! Setting zero and backing off...");
    
    // Small delay to settle
    delay(50);
    
    // Set zero at limit position
    armMotors[1].motor->setCurrentPosition(0);
    armMotors[2].motor->setCurrentPosition(0);
    armMotors[4].motor->setCurrentPosition(0);
    
    // Back off from limits using STEP VALUES from motor config
    armMotors[1].motor->setMaxSpeed(1800);
    armMotors[2].motor->setMaxSpeed(1000);
    armMotors[4].motor->setMaxSpeed(600);
    
    // Use step backoff values from motor configuration
    long back2 = -armMotors[1].homingBackoff * armMotors[1].homingDirection;
    long back3 = -armMotors[2].homingBackoff * armMotors[2].homingDirection;
    long back5 = -armMotors[4].homingBackoff * armMotors[4].homingDirection;
    
    SerialUART.print("Backoff steps - M2:");
    SerialUART.print(armMotors[1].homingBackoff);
    SerialUART.print(" M3:");
    SerialUART.print(armMotors[2].homingBackoff);
    SerialUART.print(" M5:");
    SerialUART.println(armMotors[4].homingBackoff);
    
    armMotors[1].motor->setTargetPosition(back2);
    armMotors[2].motor->setTargetPosition(back3);
    armMotors[4].motor->setTargetPosition(back5);
    
    // Wait for backoff to complete
    unsigned long backoffStart = millis();
    while ((!armMotors[1].motor->isTargetPositionReached() || 
           !armMotors[2].motor->isTargetPositionReached() || 
           !armMotors[4].motor->isTargetPositionReached()) && 
           (millis() - backoffStart < 1000000)) {
      delay(1);
    }
    
    // Final zero position after backoff
    armMotors[1].motor->setTargetPosition(0);
    armMotors[2].motor->setTargetPosition(0);
    armMotors[4].motor->setTargetPosition(0);
    armMotors[1].motor->setCurrentPosition(0);
    armMotors[2].motor->setCurrentPosition(0);
    armMotors[4].motor->setCurrentPosition(0);
    
    SerialUART.println("Group 1 homing SUCCESSFUL");
  } else {
    SerialUART.println("Group 1 homing FAILED - timeout or incomplete");
    // Stop all motors in group
    armMotors[1].motor->setTargetPosition(armMotors[1].motor->getCurrentPosition());
    armMotors[2].motor->setTargetPosition(armMotors[2].motor->getCurrentPosition());
    armMotors[4].motor->setTargetPosition(armMotors[4].motor->getCurrentPosition());
    systemState = STATE_UNCALIBRATED;
    return;
  }
  
  // GROUP 2: Arm Motors 1, 4, 6 & RAIL Motor 7
  SerialUART.println("Homing GROUP 2: M1, M4, M6 (Arm) + R1 (Rail)");
  
  // Set positioning mode
  armMotors[0].motor->setRampMode(TMC5160::POSITIONING_MODE);
  armMotors[3].motor->setRampMode(TMC5160::POSITIONING_MODE);
  armMotors[5].motor->setRampMode(TMC5160::POSITIONING_MODE);
  railMotor.motor->setRampMode(TMC5160::POSITIONING_MODE);
  
  // Set homing speed (in steps/sec)
  armMotors[0].motor->setMaxSpeed(300);
  armMotors[3].motor->setMaxSpeed(250);
  armMotors[5].motor->setMaxSpeed(150);
  railMotor.motor->setMaxSpeed(500);
  
  // Move toward limits (using steps directly)
  long target1 = armMotors[0].motor->getCurrentPosition() + (100000 * armMotors[0].homingDirection);
  long target4 = armMotors[3].motor->getCurrentPosition() + (100000 * armMotors[3].homingDirection);
  long target6 = armMotors[5].motor->getCurrentPosition() + (100000 * armMotors[5].homingDirection);
  long target7 = railMotor.motor->getCurrentPosition() + (100000 * railMotor.homingDirection);
  
  armMotors[0].motor->setTargetPosition(target1);
  armMotors[3].motor->setTargetPosition(target4);
  armMotors[5].motor->setTargetPosition(target6);
  railMotor.motor->setTargetPosition(target7);
  
  // Homing state for GROUP 2
  bool m1Hit = false, m4Hit = false, m6Hit = false, r1Hit = false;
  bool m1FirstLow = false, m4FirstLow = false, m6FirstLow = false, r1FirstLow = false;
  
  startTime = millis();
  
  SerialUART.println("Moving toward limits...");
  
  while ((!m1Hit || !m4Hit || !m6Hit || !r1Hit) && (millis() - startTime < 3000000)) {
    
    // Same two-consecutive-LOW logic for all motors in group 2
    if (!m1Hit) {
      if (digitalRead(armMotors[0].limitPin) == LOW) {
        if (m1FirstLow) {
          armMotors[0].motor->setTargetPosition(armMotors[0].motor->getCurrentPosition());
          m1Hit = true;
          SerialUART.println("M1: Limit CONFIRMED (2 consecutive LOW)");
        } else {
          m1FirstLow = true;
        }
      } else {
        m1FirstLow = false;
      }
    }
    
    if (!m4Hit) {
      if (digitalRead(armMotors[3].limitPin) == LOW) {
        if (m4FirstLow) {
          armMotors[3].motor->setTargetPosition(armMotors[3].motor->getCurrentPosition());
          m4Hit = true;
          SerialUART.println("M4: Limit CONFIRMED (2 consecutive LOW)");
        } else {
          m4FirstLow = true;
        }
      } else {
        m4FirstLow = false;
      }
    }
    
    if (!m6Hit) {
      if (digitalRead(armMotors[5].limitPin) == LOW) {
        if (m6FirstLow) {
          armMotors[5].motor->setTargetPosition(armMotors[5].motor->getCurrentPosition());
          m6Hit = true;
          SerialUART.println("M6: Limit CONFIRMED (2 consecutive LOW)");
        } else {
          m6FirstLow = true;
        }
      } else {
        m6FirstLow = false;
      }
    }
    
    if (!r1Hit) {
      if (digitalRead(railMotor.limitPin) == LOW) {
        if (r1FirstLow) {
          railMotor.motor->setTargetPosition(railMotor.motor->getCurrentPosition());
          r1Hit = true;
          SerialUART.println("R1 (Rail): Limit CONFIRMED (2 consecutive LOW)");
        } else {
          r1FirstLow = true;
        }
      } else {
        r1FirstLow = false;
      }
    }
    
    delay(5);
  }
  
  // Handle results for GROUP 2
  if (m1Hit && m4Hit && m6Hit && r1Hit) {
    SerialUART.println("Group 2 limits confirmed! Setting zero and backing off...");
    
    delay(50);
    
    // Set zero at limit position
    armMotors[0].motor->setCurrentPosition(0);
    armMotors[3].motor->setCurrentPosition(0);
    armMotors[5].motor->setCurrentPosition(0);
    railMotor.motor->setCurrentPosition(0);
    
    // Back off from limits using STEP VALUES from motor config
    armMotors[0].motor->setMaxSpeed(1000);
    armMotors[3].motor->setMaxSpeed(1000);
    armMotors[5].motor->setMaxSpeed(1000);
    railMotor.motor->setMaxSpeed(1000);
    
    // Use step backoff values from motor configuration
    long back1 = -armMotors[0].homingBackoff * armMotors[0].homingDirection;
    long back4 = -armMotors[3].homingBackoff * armMotors[3].homingDirection;
    long back6 = -armMotors[5].homingBackoff * armMotors[5].homingDirection;
    long back7 = -railMotor.homingBackoff * railMotor.homingDirection;
    
    SerialUART.print("Backoff steps - M1:");
    SerialUART.print(armMotors[0].homingBackoff);
    SerialUART.print(" M4:");
    SerialUART.print(armMotors[3].homingBackoff);
    SerialUART.print(" M6:");
    SerialUART.print(armMotors[5].homingBackoff);
    SerialUART.print(" R1:");
    SerialUART.println(railMotor.homingBackoff);
    
    armMotors[0].motor->setTargetPosition(back1);
    armMotors[3].motor->setTargetPosition(back4);
    armMotors[5].motor->setTargetPosition(back6);
    railMotor.motor->setTargetPosition(back7);
    
    // Wait for backoff to complete
    unsigned long backoffStart = millis();
    while ((!armMotors[0].motor->isTargetPositionReached() || 
           !armMotors[3].motor->isTargetPositionReached() || 
           !armMotors[5].motor->isTargetPositionReached() || 
           !railMotor.motor->isTargetPositionReached()) && 
           (millis() - backoffStart < 1000000)) {
      delay(1);
    }
    
    // Final zero position after backoff
    armMotors[0].motor->setTargetPosition(0);
    armMotors[3].motor->setTargetPosition(0);
    armMotors[5].motor->setTargetPosition(0);
    railMotor.motor->setTargetPosition(0);
    armMotors[0].motor->setCurrentPosition(0);
    armMotors[3].motor->setCurrentPosition(0);
    armMotors[5].motor->setCurrentPosition(0);
    railMotor.motor->setCurrentPosition(0);
    
    SerialUART.println("Group 2 homing SUCCESSFUL");
    systemState = STATE_READY;
  } else {
    SerialUART.println("Group 2 homing FAILED - timeout or incomplete");
    // Stop all motors in group
    armMotors[0].motor->setTargetPosition(armMotors[0].motor->getCurrentPosition());
    armMotors[3].motor->setTargetPosition(armMotors[3].motor->getCurrentPosition());
    armMotors[5].motor->setTargetPosition(armMotors[5].motor->getCurrentPosition());
    railMotor.motor->setTargetPosition(railMotor.motor->getCurrentPosition());
    systemState = STATE_UNCALIBRATED;
  }
  
  // Reset to normal operating speeds (using radians conversion for normal operation)
  for (int i = 0; i < NUM_ARM_MOTORS; i++) {
    armMotors[i].motor->setMaxSpeed(radiansToSteps(i, armMotors[i].maxSpeed));
  }
  railMotor.motor->setMaxSpeed(radiansToSteps(6, railMotor.maxSpeed));
  
  SerialUART.println("=== HOMING COMPLETE ===");
  SerialUART.println("System is READY for commands");
}
