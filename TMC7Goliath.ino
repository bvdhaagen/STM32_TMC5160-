/* TMC5160 - 7-Axis Coordinated Motion Controller */
#include <Arduino.h>
#include <TMC5160.h>
#include <SPI.h>
#include <HardwareSerial.h>

// Serial over UART - UART1 on PA9(TX), PA10(RX)
#define SERIAL_TX PA9
#define SERIAL_RX PA10
HardwareSerial SerialUART(USART1);

// STM32F411CEU6 SPI pins - SPI1
#define MOSI_PIN PA7
#define MISO_PIN PA6  
#define SCK_PIN  PA5

// MOTOR PINS - All 7 motors
const uint8_t SPI_CS1 = PB0;   const uint8_t DRV_ENN1 = PB1;   // M1 - NEMA17 1.2A
const uint8_t SPI_CS2 = PB10;  const uint8_t DRV_ENN2 = PB12;  // M2 - NEMA23 2.5A
const uint8_t SPI_CS3 = PB13;  const uint8_t DRV_ENN3 = PB14;  // M3 - NEMA23 2.0A
const uint8_t SPI_CS4 = PB15;  const uint8_t DRV_ENN4 = PB2;   // M4 - NEMA17 1.2A
const uint8_t SPI_CS5 = PB3;   const uint8_t DRV_ENN5 = PB4;   // M5 - NEMA17 1.2A
const uint8_t SPI_CS6 = PB5;   const uint8_t DRV_ENN6 = PB6;   // M6 - NEMA17 1.2A
const uint8_t SPI_CS7 = PB7;   const uint8_t DRV_ENN7 = PB8;   // M7 - NEMA23 2.0A (Slider)

// LIMIT SWITCH PINS - Input pullup (LOW when switch pressed)
const uint8_t LIMIT_M1 = PA15;  // M1 - dir - to reach limit
const uint8_t LIMIT_M2 = PA8;   // M2 - dir - to reach limit  
const uint8_t LIMIT_M3 = PA4;   // M3 - dir + to reach limit
const uint8_t LIMIT_M4 = PA3;   // M4 - dir - to reach limit
const uint8_t LIMIT_M5 = PA2;   // M5 - dir - to reach limit
const uint8_t LIMIT_M6 = PA1;   // M6 - dir - to reach limit
const uint8_t LIMIT_M7 = PA0;   // M7 - dir - to reach limit

// Motor objects
TMC5160_SPI motor1 = TMC5160_SPI(SPI_CS1);
TMC5160_SPI motor2 = TMC5160_SPI(SPI_CS2);
TMC5160_SPI motor3 = TMC5160_SPI(SPI_CS3);
TMC5160_SPI motor4 = TMC5160_SPI(SPI_CS4);
TMC5160_SPI motor5 = TMC5160_SPI(SPI_CS5);
TMC5160_SPI motor6 = TMC5160_SPI(SPI_CS6);
TMC5160_SPI motor7 = TMC5160_SPI(SPI_CS7);

// Motor configurations
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
  float maxSpeed;    // Steps per second
  float homingSpeed; //used for homing
  float acceleration; // Steps per second²
  int homingDirection;   // +1 or -1 for homing direction
  int stepsBack;         // Steps to back off after homing
};

MotorConfig motors[] = {                   //Iglobal iru iho   type         max    sp     acc  dir  backoff
  {"M1", &motor1, SPI_CS1, DRV_ENN1, LIMIT_M1, 200, 25, 10, "NEMA17 1.8A", 1000.0, 400, 1600.0, 1, 11450}, // base motor
  {"M2", &motor2, SPI_CS2, DRV_ENN2, LIMIT_M2, 240, 31, 15, "NEMA23 2.5A", 1600.0, 400, 1600.0, 1, 13600}, // shoulder motor
  {"M3", &motor3, SPI_CS3, DRV_ENN3, LIMIT_M3, 240, 25, 12, "NEMA23 2.0A", 1000.0, 400, 1600.0, 1, 8400}, // elbow motor
  {"M4", &motor4, SPI_CS4, DRV_ENN4, LIMIT_M4, 200, 25, 10, "NEMA17 1.2A", 1000.0, 400, 1600.0, 1, 6850}, // arm motor
  {"M5", &motor5, SPI_CS5, DRV_ENN5, LIMIT_M5, 200, 25, 10, "NEMA17 1.2A", 1000.0, 400, 1600.0, -1, 1870}, // wrist motor 
  {"M6", &motor6, SPI_CS6, DRV_ENN6, LIMIT_M6, 200, 20, 10, "NEMA17 1.2A", 1000.0, 400, 1600.0, -1, 3270}, // hand motor
  {"M7", &motor7, SPI_CS7, DRV_ENN7, LIMIT_M7, 240, 31, 12, "NEMA23 2.0A", 1000.0, 400, 1600.0, -1, 12800}, // linear slidar motor
};

const int NUM_MOTORS = 7;

// Steps per radian conversion (using full rotation steps)
const float STEPS_PER_RAD[NUM_MOTORS] = {
  26000.0f / (2 * PI),   //  steps/rad    
  44000.0f / (2 * PI),  //   steps/rad
  20000.0f / (2 * PI),   // J3 = 11140.85 steps/rad
  24000.0f / (2 * PI),   // J4 = 3978.87 steps/rad
  5000.0f / (2 * PI),   // J5 = 2069.01 steps/rad
  6000.0f / (2 * PI),    // J6 = 3023.94 steps/rad
  240000.0f / (2 * PI)    // J7 = 3023.94 steps/rad
};

// System States
enum SystemState {
  STATE_READY,
  STATE_MOVING,
  STATE_HOMING,
  STATE_UNCALIBRATED
};

// Global variables
SystemState systemState = STATE_UNCALIBRATED; // Start in unknown
unsigned long lastFeedbackTime = 0;
const unsigned long FEEDBACK_INTERVAL = 10; // 10ms feedback
bool motorsHoming[7] = {false};
bool checkLimitSwitch(int motorIndex);
bool allMotorsReachedTarget();
bool allHomingMotorsFinished();
void stopHomingMotor(int motorIndex);
bool motorsBackingOff[7] = {false};
//unsigned long homingStartTime = 0;
volatile bool limitSwitchTriggered[7] = {false};
volatile int triggeredMotorIndex = -1;

// Custom max function to avoid type issues
float customMax(float a, float b) {
  return (a > b) ? a : b;
}

// Custom abs function for floats
float customAbs(float x) {
  return (x < 0) ? -x : x;
}

// Custom sqrt function approximation
float customSqrt(float x) {
  if (x <= 0) return 0;
  float guess = x;
  for (int i = 0; i < 10; i++) {
    guess = 0.5 * (guess + x / guess);
  }
  return guess;
}

// Motion control variables
struct MotionProfile {
  float startPositions[7];
  float targetPositions[7];
  float distances[7];
  float maxDistance;
  bool moving;
  unsigned long moveStartTime;
  unsigned long estimatedMoveTime;
};

MotionProfile currentMove;
//unsigned long lastFeedbackTime = 0;
//const unsigned long FEEDBACK_INTERVAL = 100; // 100ms feedback

// Function prototypes
void initializeSystem();
void setCoordinatedMove(float targets[7]);
void updateMotion();
void sendFeedback();
bool allMotorsReachedTarget();
float calculateRequiredTime(float distance, float maxSpeed, float acceleration);

void limitSwitchISR() {
  // Find which motor triggered the interrupt
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (digitalRead(motors[i].limitPin) == LOW) {
      limitSwitchTriggered[i] = true;
      triggeredMotorIndex = i;
      break;
    }
  }
}

void setup() {
  SerialUART.begin(115200);
  delay(1000);
  
  initializeSystem();
  
  SerialUART.println("7-Axis Coordinated Motion Controller Ready!");

  SerialUART.println();
}


void loop() {

  while (systemState == STATE_HOMING) {
    SerialUART.println("Initialized Stage : Homing");

      break;
  }

  // Check for new move commands
  if (SerialUART.available()) {
    String command = SerialUART.readStringUntil('\n');
    command.trim();
    
   if (command.startsWith("M ")) {
    // Parse positions: M 1000,2000,1500,800,1200,900,3000
    float targets[7];
    int startIndex = 2; // After "M " (2 characters)
    
    for (int i = 0; i < 7; i++) {
        int commaIndex = command.indexOf(',', startIndex);
        if (commaIndex == -1) commaIndex = command.length();
        
        String numStr = command.substring(startIndex, commaIndex);
        targets[i] = numStr.toFloat();
        startIndex = commaIndex + 1;
        }
      
      setCoordinatedMove(targets);
    }
    else if (command.startsWith("HOME")) {
      homeRobot();
     }
  }
  

  // Update motion control
  if (currentMove.moving) {
    updateMotion();
  }
  
  // Send feedback every 10ms
  if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL) {
    sendFeedback();
    lastFeedbackTime = millis();
  }
  
  delay(1); // Small delay for stability
}

////////////////////////////END LOOP///////////////////////

void homeRobot() {
  // GROUP 1: Motors 2, 3 & 5 (shoulder, elbow, wrist) - Simultaneous with interrupts
  SerialUART.println("\n=== HOMING MOTORS 2, 3 & 5 SIMULTANEOUSLY ===");
  systemState = STATE_HOMING;
  
  bool motor2Homed = false;
  bool motor3Homed = false;
  bool motor5Homed = false;
  
  SerialUART.println("Step 1: Starting homing for M2, M3 and M5");
  
  // Reset interrupt flags
  limitSwitchTriggered[1] = false;
  limitSwitchTriggered[2] = false;
  limitSwitchTriggered[4] = false;
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(motors[1].limitPin), limitSwitchISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(motors[2].limitPin), limitSwitchISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(motors[4].limitPin), limitSwitchISR, FALLING);
  
  // Start all motors
  motors[1].motor->setMaxSpeed(motors[1].homingSpeed);
  motors[2].motor->setMaxSpeed(motors[2].homingSpeed);
  motors[4].motor->setMaxSpeed(motors[4].homingSpeed);
  
  long target2 = motors[1].motor->getCurrentPosition() + (1000000 * motors[1].homingDirection);
  long target3 = motors[2].motor->getCurrentPosition() + (1000000 * motors[2].homingDirection);
  long target5 = motors[4].motor->getCurrentPosition() + (1000000 * motors[4].homingDirection);
  
  motors[1].motor->setTargetPosition(target2);
  motors[2].motor->setTargetPosition(target3);
  motors[4].motor->setTargetPosition(target5);
  
  SerialUART.println("Step 2: Waiting for all limit switches...");
  
  unsigned long startTime1 = millis();
  bool m2Hit = false, m3Hit = false, m5Hit = false;
  long m2StopPos = 0, m3StopPos = 0, m5StopPos = 0;
  
  while ((!m2Hit || !m3Hit || !m5Hit) && (millis() - startTime1 < 60000)) {
    if (!m2Hit && limitSwitchTriggered[1]) {
      SerialUART.println("M2 limit hit!");
      m2StopPos = motors[1].motor->getCurrentPosition();
      motors[1].motor->setTargetPosition(m2StopPos);
      m2Hit = true;
    }
    
    if (!m3Hit && limitSwitchTriggered[2]) {
      SerialUART.println("M3 limit hit!");
      m3StopPos = motors[2].motor->getCurrentPosition();
      motors[2].motor->setTargetPosition(m3StopPos);
      m3Hit = true;
    }
    
    if (!m5Hit && limitSwitchTriggered[4]) {
      SerialUART.println("M5 limit hit!");
      m5StopPos = motors[4].motor->getCurrentPosition();
      motors[4].motor->setTargetPosition(m5StopPos);
      m5Hit = true;
    }
    delay(10);
  }
  
  // Detach interrupts
  detachInterrupt(digitalPinToInterrupt(motors[1].limitPin));
  detachInterrupt(digitalPinToInterrupt(motors[2].limitPin));
  detachInterrupt(digitalPinToInterrupt(motors[4].limitPin));
  
  if (!m2Hit || !m3Hit || !m5Hit) {
    SerialUART.println("Homing failed - timeout");
    systemState = STATE_UNCALIBRATED;
    return;
  }
  
  SerialUART.println("Step 3: All limits reached, backing off...");
  delay(50);
  
  // CRITICAL: Switch back to positioning mode BEFORE setting position
  motors[1].motor->setRampMode(TMC5160::POSITIONING_MODE);
  motors[2].motor->setRampMode(TMC5160::POSITIONING_MODE);
  motors[4].motor->setRampMode(TMC5160::POSITIONING_MODE);

  motors[1].motor->setCurrentPosition(0);
  motors[2].motor->setCurrentPosition(0);
  motors[4].motor->setCurrentPosition(0);

  // Back off all motors
  motors[1].motor->setMaxSpeed(motors[1].maxSpeed);
  motors[2].motor->setMaxSpeed(motors[2].maxSpeed);
  motors[4].motor->setMaxSpeed(motors[4].maxSpeed);
  
  long back2 = (motors[1].stepsBack * (-motors[1].homingDirection));
  long back3 = (motors[2].stepsBack * (-motors[2].homingDirection));
  long back5 = (motors[4].stepsBack * (-motors[4].homingDirection));
  
  motors[1].motor->setTargetPosition(back2);
  motors[2].motor->setTargetPosition(back3);
  motors[4].motor->setTargetPosition(back5);
  
  // Wait for back-off to complete
  while (!motors[1].motor->isTargetPositionReached() || 
         !motors[2].motor->isTargetPositionReached() || 
         !motors[4].motor->isTargetPositionReached()) {
    delay(10);
  }
  
  motors[1].motor->setTargetPosition(0);
  motors[2].motor->setTargetPosition(0);
  motors[4].motor->setTargetPosition(0);
  motors[1].motor->setCurrentPosition(0);
  motors[2].motor->setCurrentPosition(0);
  motors[4].motor->setCurrentPosition(0);
  
  SerialUART.println("Step 4: Motors 2, 3 & 5 homed successfully!");

  // GROUP 2: Motors 1, 4, 6 & 7 (base, arm, hand, slider) - Simultaneous with polling
  SerialUART.println("\n=== HOMING MOTORS 1, 4, 6 & 7 SIMULTANEOUSLY ===");
  systemState = STATE_HOMING;
  
  bool motor1Homed = false;
  bool motor4Homed = false;
  bool motor6Homed = false;
  bool motor7Homed = false;
  
  SerialUART.println("Step 1: Starting homing for M1, M4, M6 and M7");
  
  // Start all motors
  motors[0].motor->setMaxSpeed(motors[0].homingSpeed);
  motors[3].motor->setMaxSpeed(motors[3].homingSpeed);
  motors[5].motor->setMaxSpeed(motors[5].homingSpeed);
  motors[6].motor->setMaxSpeed(motors[6].homingSpeed);
  
  long target1 = motors[0].motor->getCurrentPosition() + (1000000 * motors[0].homingDirection);
  long target4 = motors[3].motor->getCurrentPosition() + (1000000 * motors[3].homingDirection);
  long target6 = motors[5].motor->getCurrentPosition() + (1000000 * motors[5].homingDirection);
  long target7 = motors[6].motor->getCurrentPosition() + (1000000 * motors[6].homingDirection);
  
  motors[0].motor->setTargetPosition(target1);
  motors[3].motor->setTargetPosition(target4);
  motors[5].motor->setTargetPosition(target6);
  motors[6].motor->setTargetPosition(target7);
  
  SerialUART.println("Step 2: Waiting for all limit switches...");
  
  unsigned long startTime = millis();
  bool m1Hit = false, m4Hit = false, m6Hit = false, m7Hit = false;
  long m1StopPos = 0, m4StopPos = 0, m6StopPos = 0, m7StopPos = 0;
  
  while ((!m1Hit || !m4Hit || !m6Hit || !m7Hit) && (millis() - startTime < 240000)) {
    // Simple polling - check limit switches every loop
    if (!m1Hit && digitalRead(motors[0].limitPin) == LOW) {
      SerialUART.println("M1 limit hit!");
      m1StopPos = motors[0].motor->getCurrentPosition();
      motors[0].motor->setTargetPosition(m1StopPos);
      m1Hit = true;
    }
    
    if (!m4Hit && digitalRead(motors[3].limitPin) == LOW) {
      SerialUART.println("M4 limit hit!");
      m4StopPos = motors[3].motor->getCurrentPosition();
      motors[3].motor->setTargetPosition(m4StopPos);
      m4Hit = true;
    }
    
    if (!m6Hit && digitalRead(motors[5].limitPin) == LOW) {
      SerialUART.println("M6 limit hit!");
      m6StopPos = motors[5].motor->getCurrentPosition();
      motors[5].motor->setTargetPosition(m6StopPos);
      m6Hit = true;
    }
    
    if (!m7Hit && digitalRead(motors[6].limitPin) == LOW) {
      SerialUART.println("M7 limit hit!");
      m7StopPos = motors[6].motor->getCurrentPosition();
      motors[6].motor->setTargetPosition(m7StopPos);
      m7Hit = true;
    }
    
    // Optional: Print progress every 2 seconds
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
      lastPrint = millis();
      SerialUART.print("Progress - M1:");
      SerialUART.print(motors[0].motor->getCurrentPosition());
      SerialUART.print(" M4:");
      SerialUART.print(motors[3].motor->getCurrentPosition());
      SerialUART.print(" M6:");
      SerialUART.print(motors[5].motor->getCurrentPosition());
      SerialUART.print(" M7:");
      SerialUART.println(motors[6].motor->getCurrentPosition());
    }
    
    delay(10);
  }
  
  if (!m1Hit || !m4Hit || !m6Hit || !m7Hit) {
    SerialUART.println("Homing failed - timeout");
    motors[0].motor->setTargetPosition(motors[0].motor->getCurrentPosition());
    motors[3].motor->setTargetPosition(motors[3].motor->getCurrentPosition());
    motors[5].motor->setTargetPosition(motors[5].motor->getCurrentPosition());
    motors[6].motor->setTargetPosition(motors[6].motor->getCurrentPosition());
    systemState = STATE_UNCALIBRATED;
    return;
  }
  
  SerialUART.println("Step 3: All limits reached, backing off...");
  delay(50);

  // CRITICAL: Switch back to positioning mode BEFORE setting position
  motors[0].motor->setRampMode(TMC5160::POSITIONING_MODE);
  motors[3].motor->setRampMode(TMC5160::POSITIONING_MODE);
  motors[5].motor->setRampMode(TMC5160::POSITIONING_MODE);
  motors[6].motor->setRampMode(TMC5160::POSITIONING_MODE);

  motors[0].motor->setTargetPosition(0);
  motors[3].motor->setTargetPosition(0);
  motors[5].motor->setTargetPosition(0);
  motors[6].motor->setTargetPosition(0);
  motors[0].motor->setCurrentPosition(0);
  motors[3].motor->setCurrentPosition(0);
  motors[5].motor->setCurrentPosition(0);
  motors[6].motor->setCurrentPosition(0);

  // Back off all motors
  motors[0].motor->setMaxSpeed(motors[0].maxSpeed);
  motors[3].motor->setMaxSpeed(motors[3].maxSpeed);
  motors[5].motor->setMaxSpeed(motors[5].maxSpeed);
  motors[6].motor->setMaxSpeed(motors[6].maxSpeed);
  
  long back1 = (motors[0].stepsBack * (-motors[0].homingDirection));
  long back4 = (motors[3].stepsBack * (-motors[3].homingDirection));
  long back6 = (motors[5].stepsBack * (-motors[5].homingDirection));
  long back7 = (motors[6].stepsBack * (-motors[6].homingDirection));
  
  motors[0].motor->setTargetPosition(back1);
  motors[3].motor->setTargetPosition(back4);
  motors[5].motor->setTargetPosition(back6);
  motors[6].motor->setTargetPosition(back7);
  
  // Wait for back-off to complete
  while (!motors[0].motor->isTargetPositionReached() || 
         !motors[3].motor->isTargetPositionReached() || 
         !motors[5].motor->isTargetPositionReached() || 
         !motors[6].motor->isTargetPositionReached()) {
    delay(10);
  }  
  
  // Final zero position
  motors[0].motor->setTargetPosition(0);
  motors[3].motor->setTargetPosition(0);
  motors[5].motor->setTargetPosition(0);
  motors[6].motor->setTargetPosition(0);
  motors[0].motor->setCurrentPosition(0);
  motors[3].motor->setCurrentPosition(0);
  motors[5].motor->setCurrentPosition(0);
  motors[6].motor->setCurrentPosition(0);
  
  SerialUART.println("Step 4: Motors 1, 4, 6 & 7 homed successfully!");

  systemState = STATE_READY;
}

////////////////////////////////////////////////////////////

void initializeSystem() {
  SerialUART.println("Initializing 7-Axis Motion System...");
  
  // Initialize enable pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].ennPin, OUTPUT);
    digitalWrite(motors[i].ennPin, LOW);
  }
  
  // Initialize CS pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].csPin, OUTPUT);
    digitalWrite(motors[i].csPin, HIGH);
  }
  
  // Initialize limit switches
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].limitPin, INPUT_PULLUP);
  }

  // Initialize SPI
  SPI.setMOSI(MOSI_PIN);
  SPI.setMISO(MISO_PIN);
  SPI.setSCLK(SCK_PIN);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  
  // Initialize motors
  TMC5160::PowerStageParameters powerParams;
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    SerialUART.print("Initializing ");
    SerialUART.print(motors[i].name);
    SerialUART.print("... ");
    
    TMC5160::MotorParameters motorParams;
    motorParams.globalScaler = motors[i].globalScaler;
    motorParams.irun = motors[i].irun;
    motorParams.ihold = motors[i].ihold;
    
    bool success = motors[i].motor->begin(powerParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
    
    if (success) {
      motors[i].motor->setRampMode(TMC5160::POSITIONING_MODE);
      motors[i].motor->setMaxSpeed(motors[i].maxSpeed);
      motors[i].motor->setAcceleration(motors[i].acceleration);
      motors[i].motor->setCurrentPosition(0);
      SerialUART.println("OK");
    } else {
      SerialUART.println("FAILED");
    }
    delay(50);
  }
  
  // Initialize motion profile
  currentMove.moving = false;
  for (int i = 0; i < 7; i++) {
    currentMove.startPositions[i] = 0;
    currentMove.targetPositions[i] = 0;
  }
  
  SerialUART.println("System Initialized!");

}

void setCoordinatedMove(float targets[7]) {
  // Store current positions as start positions
  for (int i = 0; i < NUM_MOTORS; i++) {
    currentMove.startPositions[i] = motors[i].motor->getCurrentPosition();
    currentMove.targetPositions[i] = targets[i];
    currentMove.distances[i] = customAbs(targets[i] - currentMove.startPositions[i]);
  }
  
  // Find the motor with the longest travel distance
  currentMove.maxDistance = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (currentMove.distances[i] > currentMove.maxDistance) {
      currentMove.maxDistance = currentMove.distances[i];
    }
  }
  
  // Calculate individual speeds to ensure simultaneous arrival
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (currentMove.distances[i] > 0) {
      // Scale speed based on distance (longer distance = faster speed)
      float speedRatio = currentMove.distances[i] / currentMove.maxDistance;
      float individualSpeed = motors[i].maxSpeed * speedRatio;
      
      // Ensure minimum speed for very short moves
      individualSpeed = customMax(individualSpeed, motors[i].maxSpeed * 0.1f);
      
      motors[i].motor->setMaxSpeed(individualSpeed);
    }
    
    // Set target position
    motors[i].motor->setTargetPosition((int32_t)targets[i]);
  }
  
  // Calculate estimated move time (based on longest move)
  currentMove.estimatedMoveTime = (unsigned long)calculateRequiredTime(
    currentMove.maxDistance, 
    motors[0].maxSpeed, // Use first motor's max speed as reference
    motors[0].acceleration
  );
  
  currentMove.moveStartTime = millis();
  currentMove.moving = true;
  
  //SerialUART.println("Move Started!");
  //SerialUART.print("Estimated time: ");
  //SerialUART.print(currentMove.estimatedMoveTime / 1000.0, 2);
  //SerialUART.println(" seconds");
}

void updateMotion() {
  if (allMotorsReachedTarget()) {
    currentMove.moving = false;
    
    
    // Reset to default speeds
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].motor->setMaxSpeed(motors[i].maxSpeed);
    }
  }
}

void sendFeedback() {
  if (systemState == STATE_HOMING){
     SerialUART.print("...Please wait for calibration");
  }
  else{
  // Send compact position feedback
for (int i = 0; i < NUM_MOTORS; i++) {
        SerialUART.print("J"); SerialUART.print(i+1); 
        SerialUART.print(":");
    
    float position = motors[i].motor->getCurrentPosition();
    if (isnan(position)) {
        SerialUART.print("0.0");
    }
    if (i == 6) {  // Changed to 6 since arrays are 0-indexed and you want the 7th motor
        float position_rad = -position / STEPS_PER_RAD[i];
        SerialUART.print(position_rad, 3);
    } else {
        // Convert steps to radians using your STEPS_PER_RAD array
        float position_rad = position / STEPS_PER_RAD[i];
        SerialUART.print(position_rad, 3);
    }
   
    // Add a space or comma between motors if needed
    if (i < NUM_MOTORS - 1) {
        SerialUART.print(" ");
    }
}
SerialUART.println();

  // Add move status
  // SerialUART.print(currentMove.moving ? ",MOVING" : ",READY");
  // SerialUART.println();
}
}

bool allMotorsReachedTarget() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].motor->isTargetPositionReached()) {
      return false;
    }
  }
  return true;
}

float calculateRequiredTime(float distance, float maxSpeed, float acceleration) {
  // Calculate time for trapezoidal velocity profile
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

