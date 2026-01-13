#include <Arduino.h>

// Template and authentication (REPLACE THESE WITH YOUR VALUES)
#define BLYNK_TEMPLATE_ID "TMPL3YRdHre8L"
#define BLYNK_TEMPLATE_NAME "Wifi Controlled Car"
#define BLYNK_AUTH_TOKEN "UrrINDRPFWETF0crd89cHfJS56nXMa-V"
#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Motor pins
#define ENA 25  // right motor enable26
#define IN1 26  // right motor27 14
#define IN2 27
#define ENB 13  // left motor enable 25 12 13
#define IN3 14
#define IN4 12

#define red_led 19
#define blue_led 21
#define green_led 22

#define TRIG_PIN 5   // Ultrasonic trigger
#define ECHO_PIN 18  // Ultrasonic echo
#define SERVO_PIN 2  // Servo signal

bool isRedOn = false;
bool isBlueOn = false;
bool isGreenOn = false;

unsigned long red_prev = 0;
unsigned long blue_prev = 0;
unsigned long green_prev = 0;

#define SERVO_PWM_FREQ 50        // 50 Hz
#define SERVO_PWM_RESOLUTION 10  // 10-bit

// WiFi credentials
char ssid[] = "vivo1904";
char pass[] = "arunachalam07";

// PWM channels
#define CH_A 0
#define CH_B 1
#define FREQ 5000
#define RES 8  // 8-bit resolution (0-255)
float left_motor_calibration = 0.965;
float right_motor_calibration = 1.0;
// Motor speed (default full)
int motorSpeed = 200;

long dist = 0;
const int MIN_SAFE_DISTANCE = 20;

const int CENTER_ANGLE = 90;
const int LEFT_ANGLE = 120;
const int RIGHT_ANGLE = 60;

int angle = CENTER_ANGLE;
unsigned long present = 0;
unsigned long last_string_written = 0;

bool moving_foward = false;

#define SERVO_MIN_DUTY 26    // ~500 µs (~0°)
#define SERVO_MAX_DUTY 128   // ~2500 µs (~180°)
#define SERVO_MOVE_DELAY 50  // Reduced delay for faster response

long getDistance() {
  long sum = 0;
  int valid = 0;
  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 10000);  // Increased timeout for ~5m range
    long dist = duration * 0.034 / 2;
    sum += dist;
    valid++;
    delay(60);  // Critical: 60ms delay between measurements to prevent echo interference
  }
  return valid > 0 ? sum / valid : -1;
}

void blinkLeds() {
  digitalWrite(red_led, HIGH);
  delay(100);
  digitalWrite(blue_led, HIGH);
  delay(100);
  digitalWrite(green_led, HIGH);
  delay(100);
  digitalWrite(green_led, LOW);
  delay(100);
  digitalWrite(blue_led, LOW);
  delay(100);
  digitalWrite(red_led, LOW);
}
uint32_t angleToDuty(int angle) {
  return map(angle, 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
}

// Fast servo movement
void moveServo(int target_angle) {
  ledcWrite(SERVO_PIN, angleToDuty(target_angle));
  delay(SERVO_MOVE_DELAY);
}

bool objectDetection(bool left, bool right) {
  if (left) {
    int scan_angles[] = { 105, 120, 135, 150 };
    for (int i = 0; i < 4; i++) {
      moveServo(scan_angles[i]);
      int distance = getDistance();
      if (distance <= MIN_SAFE_DISTANCE && dist != 0) {
        return true;
      }
    }
    return false;
  } else{
    int scan_angles[] = { 30, 45, 60, 75 };
    for (int i = 0; i < 4; i++) {
      moveServo(scan_angles[i]);
      int distance = getDistance();
      if (distance <= MIN_SAFE_DISTANCE && dist != 0) {
        return true;
      }
    }
    return false;
  }
}

void setDirectionForwardBackward(bool leftForward, bool rightForward) {
  // Left motor
  digitalWrite(IN1, leftForward ? HIGH : LOW);
  digitalWrite(IN2, leftForward ? LOW : HIGH);
  ledcWrite(ENA, motorSpeed * right_motor_calibration);

  // Right motor
  digitalWrite(IN3, rightForward ? HIGH : LOW);
  digitalWrite(IN4, rightForward ? LOW : HIGH);
  ledcWrite(ENB, motorSpeed * left_motor_calibration);
}

void setDirectionLeftRight(bool left, bool right) {
  if (left) {
    // Stop left motor
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, 0);
    // Move right motor forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, motorSpeed * right_motor_calibration);
  } else if (right) {
    // Stop right motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
    // Move left motor forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, motorSpeed * left_motor_calibration);
  }
}



// Forward button
BLYNK_WRITE(V0) {
  if (param.asInt()) {
    moveServo(CENTER_ANGLE);
    dist = getDistance();
    if (dist > MIN_SAFE_DISTANCE || dist == 0) {
      digitalWrite(green_led, HIGH);
      isGreenOn = true;
      green_prev = millis();
      setDirectionForwardBackward(true, true);  // Both forward
      moving_foward = true;
    } else {
      Blynk.virtualWrite(V11, "Obstacle detected in front!");
      last_string_written = present;
      blinkLeds();
    }
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
  }
}

// Backward button
BLYNK_WRITE(V1) {
  if (param.asInt()) {
    digitalWrite(red_led, HIGH);
    isRedOn = true;
    red_prev = millis();
    setDirectionForwardBackward(false, false);  // Both backward
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
  }
}

// Left button
BLYNK_WRITE(V2) {
  if (param.asInt()) {
    if (!objectDetection(true, false)) {
      digitalWrite(blue_led, HIGH);
      isBlueOn = true;
      blue_prev = millis();
      setDirectionLeftRight(true, false);
      moveServo(CENTER_ANGLE);
      delay(200);
    } else {
      Blynk.virtualWrite(V11, "Obstacle detected in left!");
      last_string_written = present;
      blinkLeds();
    }
  } else {
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
  }
}

// Right button
BLYNK_WRITE(V3) {
  if (param.asInt()) {
    if (!objectDetection(false, true)) {
      digitalWrite(blue_led, HIGH);
      isBlueOn = true;
      blue_prev = millis();
      setDirectionLeftRight(false, true);
      moveServo(CENTER_ANGLE);
      delay(200);
    } else {
      Blynk.virtualWrite(V11, "Obstacle detected in right!");
      last_string_written = present;
      blinkLeds();
    }
  } else {
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
  }
}

// Speed control slider
BLYNK_WRITE(V4) {
  motorSpeed = param.asInt();  // Read slider value (0–255)
  Serial.print("Speed set to: ");
  Serial.println(motorSpeed);
  Blynk.virtualWrite(V9, motorSpeed);
}

BLYNK_WRITE(V5) {
  left_motor_calibration = param.asFloat();
  Blynk.virtualWrite(V7, left_motor_calibration);
}

BLYNK_WRITE(V6) {
  right_motor_calibration = param.asFloat();
  Blynk.virtualWrite(V8, right_motor_calibration);
}


void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PWM setup
  ledcSetup(CH_A, FREQ, RES);
  ledcAttachPin(ENA, CH_A);
  ledcSetup(CH_B, FREQ, RES);
  ledcAttachPin(ENB, CH_B);
  ledcSetup(2, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  pinMode(red_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  pinMode(green_led, OUTPUT);

  // Configure ADC
  analogReadResolution(12);        // Set to 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db);  // Set attenuation to 0-3.3V range
  analogSetWidth(12);              // Set 12-bit resolution

  moveServo(CENTER_ANGLE);
}

void loop() {
  present = millis();
  if (moving_foward) {
    dist = getDistance();
    if (dist <= MIN_SAFE_DISTANCE && dist != 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      ledcWrite(ENA, 0);
      ledcWrite(ENB, 0);
      moving_foward = false;
    }
  }
  if (present - red_prev > 1500 && isRedOn) {
    digitalWrite(red_led, LOW);
    isRedOn = false;
  }
  if (present - blue_prev > 1500 && isBlueOn) {
    digitalWrite(blue_led, LOW);
    isBlueOn = false;
  }
  if (present - green_prev > 1500 && isGreenOn) {
    digitalWrite(green_led, LOW);
    isGreenOn = false;
  }
  if (present - last_string_written >= 5000) {
    Blynk.virtualWrite(V11, "");
    last_string_written = present;
  }
  Blynk.run();
}