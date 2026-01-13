#include <Arduino.h>

// Motor Pins - Connected to L298N
#define MOTOR_A_EN 13   // Enable pin for Motor A (Left)
#define MOTOR_A_IN1 14  // Input 1 for Motor A
#define MOTOR_A_IN2 12  // Input 2 for Motor A
#define MOTOR_B_EN 25   // Enable pin for Motor B (Right)
#define MOTOR_B_IN3 26  // Input 3 for Motor B
#define MOTOR_B_IN4 27  // Input 4 for Motor B

// LEDC Configuration
#define LEDC_CHANNEL_A 0
#define LEDC_CHANNEL_B 1
#define LEDC_FREQUENCY 5000
#define LEDC_RESOLUTION 8

// IR Sensor Pins
#define LEFT_IR_SENSOR 32
#define MIDDLE_IR_SENSOR 33
#define RIGHT_IR_SENSOR 34

// Speed settings
#define BASE_SPEED 210
#define TURN_SPEED 200
#define SHARP_TURN_SPEED 175
#define SLOW_SPEED 190
#define SEARCH_SPEED 190

enum RobotState { FOLLOWING, SEARCH_LEFT, SEARCH_RIGHT, LOST, SHARP_TURN_LEFT, SHARP_TURN_RIGHT };
RobotState currentState = FOLLOWING;
unsigned long searchStartTime = 0;
unsigned long turnStartTime = 0;
const unsigned long SEARCH_TIME = 800;
const unsigned long SHARP_TURN_TIME = 1200; // Maximum time for sharp turn execution

// Sharp turn detection
bool wasOnLine = false;
unsigned long offLineTime = 0;
int consecutiveSharpTurns = 0;

void setup() {
  // Serial.begin(115200);
  
  // Initialize pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(MIDDLE_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  
  // Setup LEDC
  ledcSetup(LEDC_CHANNEL_A, LEDC_FREQUENCY, LEDC_RESOLUTION);
  ledcSetup(LEDC_CHANNEL_B, LEDC_FREQUENCY, LEDC_RESOLUTION);
  ledcAttachPin(MOTOR_A_EN, LEDC_CHANNEL_A);
  ledcAttachPin(MOTOR_B_EN, LEDC_CHANNEL_B);
  // Serial.println("Enhanced Line Following Robot with Sharp Turn Handling Started!");
  delay(2000);
}

void readSensors(bool &left, bool &middle, bool &right) {
  left = digitalRead(LEFT_IR_SENSOR);
  middle = digitalRead(MIDDLE_IR_SENSOR);
  right = digitalRead(RIGHT_IR_SENSOR);
}

void setMotorSpeed(int pin, int speed) {
  speed = constrain(speed, 0, 255);
  ledcWrite(pin, speed);
}

void setMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  digitalWrite(MOTOR_A_IN1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_A_IN2, leftSpeed >= 0 ? LOW : HIGH);
  leftSpeed *= 1;
  setMotorSpeed(MOTOR_A_EN, abs(leftSpeed));
  
  // Right motor
  digitalWrite(MOTOR_B_IN3, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_B_IN4, rightSpeed >= 0 ? LOW : HIGH);
  rightSpeed *= 0.94;
  setMotorSpeed(MOTOR_B_EN, abs(rightSpeed));
}

void stopMotors() {
  setMotors(0, 0);
}

bool detectSharpTurn(bool left, bool middle, bool right) {
  bool sharpTurn = false;
  
  // Detect potential sharp turn conditions
  if (wasOnLine && !left && !middle && !right) {
    // Just lost the line - could be entering a sharp turn
    offLineTime = millis();
    sharpTurn = true;
  }
  
  // Update wasOnLine for next iteration
  wasOnLine = (left || middle || right);
  
  return sharpTurn;
}

void handleSharpTurnLeftState(bool left, bool middle, bool right) {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - turnStartTime;
  
  // Execute sharp left turn maneuver
  setMotors(-SHARP_TURN_SPEED, SHARP_TURN_SPEED);
  // Serial.println("Executing sharp left turn maneuver");
  
  if (left || middle || right) {
    currentState = FOLLOWING;
    // Serial.println("Sharp left turn completed - line found!");
    return;
  }
  
  // Check if turn time is complete without finding line
  if (elapsedTime >= SHARP_TURN_TIME) {
    // Serial.println("Sharp left turn timeout - switching to search");
    currentState = SEARCH_LEFT;
    searchStartTime = currentTime;
  }
}

void handleSharpTurnRightState(bool left, bool middle, bool right) {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - turnStartTime;
  
  // Execute sharp right turn maneuver
  setMotors(SHARP_TURN_SPEED, -SHARP_TURN_SPEED);
  // Serial.println("Executing sharp right turn maneuver");
  
  
  if (left || middle || right) {
    currentState = FOLLOWING;
    // Serial.println("Sharp right turn completed - line found!");
    return;
  }
  
  // Check if turn time is complete without finding line
  if (elapsedTime >= SHARP_TURN_TIME) {
    // Serial.println("Sharp right turn timeout - switching to search");
    currentState = SEARCH_RIGHT;
    searchStartTime = currentTime;
  }
}

void handleFollowingState(bool left, bool middle, bool right) {
  static unsigned long lostLineTime = 0;
  static bool lineWasLost = false;
  static bool lastLeft = false, lastMiddle = false, lastRight = false;
  
  // Enhanced sharp turn detection
  bool sharpTurnDetected = detectSharpTurn(left, middle, right);
  
  if (sharpTurnDetected && consecutiveSharpTurns < 3) {
    // Use last sensor reading to determine turn direction
    if (lastLeft && !lastRight) {
      currentState = SHARP_TURN_LEFT;
      // Serial.println("Sharp LEFT turn detected - executing turn maneuver");
    } else if (lastRight && !lastLeft) {
      currentState = SHARP_TURN_RIGHT;
      // Serial.println("Sharp RIGHT turn detected - executing turn maneuver");
    } else {
      // If no clear direction from last sensors, use middle sensor history
      if (lastMiddle) {
        // Was going straight, choose direction based on context
        currentState = SEARCH_RIGHT; // Default to right search
      }
    }
    
    if (currentState == SHARP_TURN_LEFT || currentState == SHARP_TURN_RIGHT) {
      turnStartTime = millis();
      consecutiveSharpTurns++;
    }
    
    // Save current sensor state before returning
    lastLeft = left; lastMiddle = middle; lastRight = right;
    return;
  }
  
  // Save current sensor state for next iteration
  lastLeft = left; lastMiddle = middle; lastRight = right;
  
  if (middle && !left && !right) {
    // Straight
    setMotors(BASE_SPEED, BASE_SPEED);
    lineWasLost = false;
    consecutiveSharpTurns = 0;
    // Serial.println("Moving forward");
  }
  else if (left && !middle && !right) {
    // Sharp left turn - more aggressive
    setMotors(-TURN_SPEED, TURN_SPEED);
    lineWasLost = false;
    consecutiveSharpTurns = 0;
    // Serial.println("Sharp left turn");
  }
  else if (right && !middle && !left) {
    // Sharp right turn - more aggressive
    setMotors(TURN_SPEED, -TURN_SPEED);
    lineWasLost = false;
    consecutiveSharpTurns = 0;
    // Serial.println("Sharp right turn");
  }
  else if (left && middle && !right) {
    // Slight left - faster correction
    setMotors(SLOW_SPEED, BASE_SPEED + 20);
    lineWasLost = false;
    consecutiveSharpTurns = 0;
    // Serial.println("Slight left adjustment");
  }
  else if (right && middle && !left) {
    // Slight right - faster correction
    setMotors(BASE_SPEED + 20, SLOW_SPEED);
    lineWasLost = false;
    consecutiveSharpTurns = 0;
    // Serial.println("Slight right adjustment");
  }
  else if (!left && !middle && !right) {
    // Line lost - use debouncing
    if (!lineWasLost) {
      lostLineTime = millis();
      lineWasLost = true;
    }
    
    // Shorter debounce time for sharp turns (50ms vs 100ms)
    if (millis() - lostLineTime > 50) {
      // For sharp turns, immediately go to aggressive search
      currentState = SEARCH_RIGHT; // Default to right search
      searchStartTime = millis();
      // Serial.println("Quick transition to search for sharp turns");
    } else {
      // Continue with last known command briefly
      setMotors(SLOW_SPEED, SLOW_SPEED);
    }
  }
  else {
    // All sensors on line or other combinations
    setMotors(BASE_SPEED, BASE_SPEED);
    lineWasLost = false;
    consecutiveSharpTurns = 0;
    // Serial.println("All sensors on line - moving forward");
  }
}

void handleSearchLeftState(bool left, bool middle, bool right) {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - searchStartTime;
  
  if (elapsedTime < 200) {
    setMotors(-SEARCH_SPEED, SEARCH_SPEED); // Sharp left
  } else if (elapsedTime < 500) {
    setMotors(-SEARCH_SPEED * 0.8, SEARCH_SPEED * 0.6); // Moderate left
  } else {
    setMotors(-SEARCH_SPEED * 0.6, SEARCH_SPEED * 0.4); // Wide left with forward
  }
  
  if (left || middle || right) {
    currentState = FOLLOWING;
    // Serial.println("Line found during left search! Resuming following.");
    return;
  }
  
  if (elapsedTime >= SEARCH_TIME) {
    currentState = LOST;
    // Serial.println("Left search timeout - Entering LOST state");
  }
}

void handleSearchRightState(bool left, bool middle, bool right) {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - searchStartTime;
  
  
  if (elapsedTime < 200) {
    setMotors(SEARCH_SPEED, -SEARCH_SPEED); // Sharp right
  } else if (elapsedTime < 500) {
    setMotors(SEARCH_SPEED * 0.6, -SEARCH_SPEED * 0.8); // Moderate right
  } else {
    setMotors(SEARCH_SPEED * 0.4, -SEARCH_SPEED * 0.6); // Wide right with forward
  }
  
  if (left || middle || right) {
    currentState = FOLLOWING;
    // Serial.println("Line found during right search! Resuming following.");
    return;
  }
  
  // Check if search time is complete
  if (elapsedTime >= SEARCH_TIME) {
    currentState = LOST;
    // Serial.println("Right search timeout - Entering LOST state");
  }
}

void handleLostState(bool left, bool middle, bool right) {
  static unsigned long lostStartTime = 0;
  static int recoveryPhase = 0;
  
  // Initialize lost state
  if (currentState != LOST) {
    lostStartTime = millis();
    recoveryPhase = 0;
    // Serial.println("Entering LOST state - starting aggressive recovery");
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lostStartTime;
  
  if (left || middle || right) {
    currentState = FOLLOWING;
    // Serial.println("Line found during recovery! Returning to FOLLOWING");
    return;
  }
  
  // Aggressive recovery pattern for sharp turn scenarios
  switch (recoveryPhase) {
    case 0:
      // Phase 1: Sharp 180 turn
      setMotors(-SHARP_TURN_SPEED, SHARP_TURN_SPEED);
      if (elapsedTime > 800) {
        recoveryPhase = 1;
        lostStartTime = currentTime;
      }
      break;
      
    case 1:
      // Phase 2: Forward with sweeping
      if (elapsedTime < 600) {
        setMotors(SEARCH_SPEED * 0.8, SEARCH_SPEED * 0.3);
      } else if (elapsedTime < 1200) {
        setMotors(SEARCH_SPEED * 0.3, SEARCH_SPEED * 0.8);
      } else {
        recoveryPhase = 2;
        lostStartTime = currentTime;
      }
      break;
      
    case 2:
      // Phase 3: Full 360 scan
      setMotors(-TURN_SPEED, TURN_SPEED);
      if (elapsedTime > 2000) {
        // Restart the entire process
        recoveryPhase = 0;
        lostStartTime = currentTime;
        currentState = SEARCH_RIGHT;
        searchStartTime = currentTime;
      }
      break;
  }
}

void followLine() {
  bool left, middle, right;
  readSensors(left, middle, right);
  
  // Debug output
  /*
  Serial.print("State: ");
  switch(currentState) {
    case FOLLOWING: Serial.print("FOLLOWING"); break;
    case SEARCH_LEFT: Serial.print("SEARCH_LEFT"); break;
    case SEARCH_RIGHT: Serial.print("SEARCH_RIGHT"); break;
    case LOST: Serial.print("LOST"); break;
    case SHARP_TURN_LEFT: Serial.print("SHARP_TURN_LEFT"); break;
    case SHARP_TURN_RIGHT: Serial.print("SHARP_TURN_RIGHT"); break;
  }
  Serial.print(" | Sensors: L=");
  Serial.print(left);
  Serial.print(" M=");
  Serial.print(middle);
  Serial.print(" R=");
  Serial.println(right);
  */
  
  // State machine
  switch(currentState) {
    case FOLLOWING:
      handleFollowingState(left, middle, right);
      break;
    case SEARCH_LEFT:
      handleSearchLeftState(left, middle, right);
      break;
    case SEARCH_RIGHT:
      handleSearchRightState(left, middle, right);
      break;
    case LOST:
      handleLostState(left, middle, right);
      break;
    case SHARP_TURN_LEFT:
      handleSharpTurnLeftState(left, middle, right);
      break;
    case SHARP_TURN_RIGHT:
      handleSharpTurnRightState(left, middle, right);
      break;
  }
}

void loop() {
  followLine();
  delay(20); // Faster loop for better sharp turn response
}