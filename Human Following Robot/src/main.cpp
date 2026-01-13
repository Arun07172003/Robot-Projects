#include <Arduino.h>

#define DEBUG 0

#if DEBUG
#define DEBUG_PRINT(x)
DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
DEBUG_PRINTln(x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// Pin definitions
#define TRIG_PIN 5     // Ultrasonic trigger
#define ECHO_PIN 18    // Ultrasonic echo
#define SERVO_PIN 2    // Servo signal
#define IN1 26         // Left motor forward
#define IN2 27         // Left motor backward
#define ENA 25         // Left motor PWM
#define IN3 14         // Right motor forward
#define IN4 12         // Right motor backward
#define ENB 13         // Right motor PWM
#define GREEN_LED 22   // Forward
#define YELLOW_LED 21  // Turning
#define RED_LED 19     // Stop/too close

#define MIN_SAFE_DISTANCE 25     // cm, stop if closer
#define MAX_FOLLOW_DISTANCE 100  // cm, max distance
#define SCAN_RANGE 30            // ±30° for three sides
#define MIN_SCAN_ANGLE 30        // Min scan angle (assumed right side)
#define MAX_SCAN_ANGLE 150       // Max scan angle (assumed left side)
#define PIVOT_DURATION 300       // ms for initial pivot
#define ADJUST_DURATION 100      // ms for small adjustments
#define CHECK_INTERVAL 100       // ms for distance checks during forward

// Servo PWM settings
#define SERVO_PWM_FREQ 50        // 50 Hz
#define SERVO_PWM_RESOLUTION 10  // 10-bit
#define SERVO_MIN_DUTY 26        // ~500 µs (~0°)
#define SERVO_MAX_DUTY 128       // ~2500 µs (~180°)
#define SERVO_MOVE_DELAY 50      // Reduced delay for faster response

// Motor PWM settings
#define MOTOR_PWM_FREQ 5000            // 5 kHz
#define MOTOR_PWM_RESOLUTION 8         // 8-bit
#define MIN_SPEED 170                  // Min speed (≥170)
#define MAX_SPEED 200                  // Max speed
#define PIVOT_SPEED 180                // Speed for pivot turns
#define LEFT_MOTOR_CALIBRATION 1.0     // Left motor calibration
#define RIGHT_MOTOR_CALIBRATION 0.965  // Right motor calibration

// Tracking variables
int target_angle = 90;      // Start at center
bool is_following = false;  // Track if pursuing human
int last_side = 1;          // -1: full scan, 0: left (high angles), 2: right (low angles), 1: center

// Scan angles (assuming low angles = right, high = left)
const int full_angles[] = { 30, 45, 60, 75, 90, 105, 120, 135, 150 };
const int left_biased[] = { 90, 105, 120, 135, 150 };  // Focus left (high angles)
const int center_biased[] = { 75, 90, 105 };
const int right_biased[] = { 30, 45, 60, 75, 90 };  // Focus right (low angles)

long current_dist = 0;

// Map angle to servo PWM duty cycle
uint32_t angleToDuty(int angle) {
  return map(angle, 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
}

// Fast servo movement
void moveServo(int target_angle) {
  target_angle = constrain(target_angle, MIN_SCAN_ANGLE, MAX_SCAN_ANGLE);
  ledcWrite(SERVO_PIN, angleToDuty(target_angle));
  delay(SERVO_MOVE_DELAY);
}

long getDistance() {
  long sum = 0;
  int valid = 0;
  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 8000);  
    if (duration > 0) {
      long dist = duration * 0.034 / 2;
      if (dist > 0 && dist < 500) { 
        sum += dist;
        valid++;
      }
    }
    delay(60);  
  }
  return valid > 0 ? sum / valid : -1;
}

// Calculate motor speed based on distance
int calculateMotorSpeed(long distance) {
  if (distance <= MIN_SAFE_DISTANCE || distance > MAX_FOLLOW_DISTANCE) {
    return 0;
  }
  return map(distance, MIN_SAFE_DISTANCE, MAX_FOLLOW_DISTANCE, MIN_SPEED, MAX_SPEED);
}
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, 0);
  DEBUG_PRINTLN("Motors stopped");
}


// Motor control functions
void moveForward(int speed) {
  if (speed == 0) {
    stopMotors();
    return;
  }

  // int left_speed = constrain(speed * LEFT_MOTOR_CALIBRATION, MIN_SPEED, MAX_SPEED);
  // int right_speed = constrain(speed * RIGHT_MOTOR_CALIBRATION, MIN_SPEED, MAX_SPEED);
  int left_speed = speed * LEFT_MOTOR_CALIBRATION;
  int right_speed = speed * RIGHT_MOTOR_CALIBRATION;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, left_speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, right_speed);
  DEBUG_PRINTLN("Moving forward at speed: " + String(speed));
}

void pivotLeft() {
  // Pivot left: left backward, right forward for tighter turn
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(ENA, PIVOT_SPEED * LEFT_MOTOR_CALIBRATION);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, PIVOT_SPEED * RIGHT_MOTOR_CALIBRATION);
  DEBUG_PRINTLN("Pivoting left");
}

void pivotRight() {
  // Pivot right: left forward, right backward for tighter turn
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, PIVOT_SPEED * LEFT_MOTOR_CALIBRATION);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENB, PIVOT_SPEED * RIGHT_MOTOR_CALIBRATION);
  DEBUG_PRINTLN("Pivoting right");
}


// LED control
void setLEDs(bool green, bool yellow, bool red) {
  digitalWrite(GREEN_LED, green ? HIGH : LOW);
  digitalWrite(YELLOW_LED, yellow ? HIGH : LOW);
  digitalWrite(RED_LED, red ? HIGH : LOW);
}

// Find best angle to follow with better detection
int findBestAngle(int min_angle, int max_angle, int step) {
  long min_distance = MAX_FOLLOW_DISTANCE + 1;
  int best_angle = 90;
  bool found = false;

  for (int angle = min_angle; angle <= max_angle; angle += step) {
    moveServo(angle);
    long distance = getDistance();
    DEBUG_PRINT("Rescan angle: " + String(angle) + " dist: " + String(distance));

    if (distance > MIN_SAFE_DISTANCE && distance < MAX_FOLLOW_DISTANCE) {
      if (distance < min_distance) {
        min_distance = distance;
        best_angle = angle;
        found = true;
      }
    }
  }

  return found ? best_angle : -1;
}

void setup() {
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // Setup PWM
  ledcSetup(0, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 0);
  ledcSetup(1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(ENA, 1);
  ledcSetup(2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(ENB, 2);

  Serial.begin(115200);
  DEBUG_PRINTLN("Starting optimized human-following robot...");

  moveServo(90);
  setLEDs(false, false, false);
  stopMotors();
  delay(1000);
}

void loop() {
  if (!is_following) {
    // Determine scan angles based on last_side
    int scan_angles[9];
    int num_angles;
    if (last_side == -1) {
      num_angles = sizeof(full_angles) / sizeof(full_angles[0]);
      memcpy(scan_angles, full_angles, sizeof(full_angles));
      DEBUG_PRINTLN("Full scan");
    } else if (last_side == 0) {  // Left (high angles)
      num_angles = sizeof(left_biased) / sizeof(left_biased[0]);
      memcpy(scan_angles, left_biased, sizeof(left_biased));
      DEBUG_PRINTLN("Left-biased scan");
    } else if (last_side == 2) {  // Right (low angles)
      num_angles = sizeof(right_biased) / sizeof(right_biased[0]);
      memcpy(scan_angles, right_biased, sizeof(right_biased));
      DEBUG_PRINTLN("Right-biased scan");
    } else {  // Center
      num_angles = sizeof(center_biased) / sizeof(center_biased[0]);
      memcpy(scan_angles, center_biased, sizeof(center_biased));
      DEBUG_PRINTLN("Center-biased scan");
    }

    long distances[9];
    bool too_close = false;
    for (int i = 0; i < num_angles; i++) {
      moveServo(scan_angles[i]);
      distances[i] = getDistance();
      DEBUG_PRINT("Angle: " + String(scan_angles[i]) + "° Distance: " + (distances[i] == -1 ? "Invalid" : String(distances[i]) + " cm"));
      if (distances[i] != -1 && distances[i] <= MIN_SAFE_DISTANCE) {
        too_close = true;
      }
    }

    if (too_close) {
      setLEDs(false, false, true);
      stopMotors();
      is_following = false;
      last_side = -1;
      DEBUG_PRINTLN("Object too close on one side, stopping");
    } else {
      // Find closest valid target
      long min_dist = MAX_FOLLOW_DISTANCE + 1;
      int min_index = -1;
      for (int i = 0; i < num_angles; i++) {
        if (distances[i] > MIN_SAFE_DISTANCE && distances[i] < MAX_FOLLOW_DISTANCE) {
          if (distances[i] < min_dist) {
            min_dist = distances[i];
            min_index = i;
          }
        }
      }

      if (min_index == -1) {
        // No target found
        setLEDs(false, false, true);
        stopMotors();
        if (last_side != -1) {
          last_side = -1;  // Fall back to full scan next
          DEBUG_PRINTLN("No target in biased scan, switching to full next");
        } else {
          DEBUG_PRINTLN("No target detected");
        }
      } else {
        // Target found
        int detected_angle = scan_angles[min_index];
        is_following = true;

        // Determine side (low angles = right (2), high = left (0))
        if (detected_angle < 75) last_side = 2;        // Right (low)
        else if (detected_angle > 105) last_side = 0;  // Left (high)
        else last_side = 1;                            // Center
        DEBUG_PRINTLN("Detected side: " + String(last_side));

        // Handle based on side
        if (last_side == 1) {  // Center
          setLEDs(true, false, false);
          moveServo(90);
          int speed = calculateMotorSpeed(min_dist);
          moveForward(speed);
          DEBUG_PRINTLN("Center: Moving forward");
        } else if (last_side == 0) {  // Left (high angles)
          setLEDs(false, true, false);
          pivotLeft();
          delay(PIVOT_DURATION);
          stopMotors();

          // Rescan biased to left (high angles)
          int best_angle = findBestAngle(90, 150, 10);
          if (best_angle != -1) {
            moveServo(best_angle);
            long dist = getDistance();
            if (dist > MIN_SAFE_DISTANCE && dist < MAX_FOLLOW_DISTANCE) {
              // Update last_side based on best_angle
              last_side = (best_angle < 75) ? 2 : (best_angle > 105 ? 0 : 1);
              if (best_angle >= 90 && best_angle <= 105) {
                int speed = calculateMotorSpeed(dist);
                moveForward(speed);
                setLEDs(true, false, false);
              } else if (best_angle > 105 && best_angle <= 150) {
                setLEDs(false, true, false);
                pivotLeft();
                delay(ADJUST_DURATION);
                stopMotors();
                last_side = 2;
              }
            } else {
              is_following = false;
              last_side = -1;
            }
          } else {
            is_following = false;
            last_side = -1;
          }
        } else if (last_side == 2) {  // Right (low angles)
          setLEDs(false, true, false);
          pivotRight();
          delay(PIVOT_DURATION);
          stopMotors();

          // Rescan biased to right (low angles)
          int best_angle = findBestAngle(30, 90, 10);
          if (best_angle != -1) {
            moveServo(best_angle);
            long dist = getDistance();
            if (dist > MIN_SAFE_DISTANCE && dist < MAX_FOLLOW_DISTANCE) {
              // Update last_side based on best_angle
              last_side = (best_angle < 75) ? 2 : (best_angle > 105 ? 0 : 1);
              if (best_angle >= 75 && best_angle <= 90) {
                int speed = calculateMotorSpeed(dist);
                moveForward(speed);
                setLEDs(true, false, false);
              } else if (best_angle >= 30 && best_angle < 75) {
                setLEDs(false, true, false);
                pivotRight();
                delay(ADJUST_DURATION);
                stopMotors();
                last_side = 0;
              }
            } else {
              is_following = false;
              last_side = -1;
            }
          } else {
            is_following = false;
            last_side = -1;
          }
        }
      }
    }
  } else {
    // Following mode - quick center check
    if (current_dist > 30 || current_dist == 0) {
      int best_angle = findBestAngle(75, 105, 15);
      moveServo(best_angle);
      current_dist = getDistance();
    } else {
      moveServo(90);
      current_dist = getDistance();
    }
    DEBUG_PRINT("Following, center distance: " + String(current_dist));

    if (current_dist <= MIN_SAFE_DISTANCE || current_dist > MAX_FOLLOW_DISTANCE || current_dist == -1) {
      setLEDs(false, false, true);
      stopMotors();
      is_following = false;
      last_side = -1;
      current_dist = 0;
      DEBUG_PRINTLN("Target lost or too close");
    } else {
      setLEDs(true, false, false);
      int speed = calculateMotorSpeed(current_dist);
      moveForward(speed);
    }
  }

  //  delay(50);
}