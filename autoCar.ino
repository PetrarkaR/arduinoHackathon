#include <IRremote.h>  // Include the legacy IR library
#include <AFMotor.h>   // Include the AFMotor library for Motor Shield v1.0.1
#include <math.h>

AF_DCMotor MOTOR_LEVO_GORE(4);
AF_DCMotor MOTOR_DESNO_GORE(3);
AF_DCMotor MOTOR_LEVO_DOLE(1);
AF_DCMotor MOTOR_DESNO_DOLE(2);

#define irReceiverPin 14
#define trigPin 17
#define echoPin 16
#define pinA 18
#define pinB 19

const int userRPMMin = 0;            // Minimalni unos korisnika
const int userRPMMax = 1000;         // Maksimalni unos korisnika
const int realRPMMin = 213;          // Minimalni realni RPM
const int realRPMMax = 348;          // Maksimalni realni RPM
const int minPWM = 130;              // Minimalni PWM za pokretanje motora
const int maxPWM = 255;              // Maksimalni PWM
float desiredRPM = 0;                // Željeni RPM od korisnika
float scaledRPM = 0;                 // Skalirani RPM (realni opseg)
float actualRPM = 0;                 // Izmereni RPM
int motorSpeed = 0;                  // Brzina motora (0-255)
volatile long counts = 0;            // Brojač impulsa
const int pulsesPerRevolution = 20;  // Broj impulsa po rotaciji



volatile float distance = 0;
volatile unsigned long hexCode;
volatile bool followFlag = false;
volatile int samples = 0;
volatile float totalDistance = 0;
volatile float averageDistance = 0;
volatile float totalError = 0;
volatile float averageError = 0;
volatile bool key_flagF = false;
volatile bool key_flagR = false;
volatile bool key_flagB = false;
volatile bool key_flagL = false;
volatile bool counter_flag = false;
volatile bool rpm_flag = false;
volatile long pulses = 0;
volatile int lastState = LOW;
volatile float rotations = 0;
volatile int speed = 0;
volatile int speedRPM = 0;
volatile int maxRPM = 0;
volatile int pulseCount = 0;
volatile int encoder_enabler = false;
volatile bool collision_flag = false;
volatile float estimate = 0;
volatile bool test_flag = false;

IRrecv irrecv(irReceiverPin);
decode_results results;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();  // Start the IR receiver
  Serial.println("IR Receiver is ready.");


  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  MOTOR_LEVO_GORE.setSpeed(255);
  MOTOR_DESNO_GORE.setSpeed(255);
  MOTOR_LEVO_DOLE.setSpeed(255);
  MOTOR_DESNO_DOLE.setSpeed(255);

  motorStop();
  pinMode(pinB, INPUT_PULLUP);

  speed = map(speedRPM, 0, maxRPM, 100, 255);
  //eksperimentisi ovde
}
//jedan tocak je malo naduven
void loop() {
  static unsigned long lastDistanceCheck = 0;
  static unsigned long lastIRCheck = 0;
  static unsigned long lastEncoderCheck = 0;
  static unsigned long lastRPMPrint = 0;
  static unsigned long lastCollisionCheck = 0;
  unsigned long lastPrintTime = 0;
  static unsigned long lastRPMtime = 0;
  // Handle encoder calculations every 200ms
  /* if (millis() - lastEncoderCheck > 100) {
    lastEncoderCheck = millis();

    // Calculate rotations
    rotations = getRotations();

    // Print rotations every second
    if (millis() - lastRPMPrint > 1000) {
      lastRPMPrint = millis();
      Serial.print("Rotations: ");
      Serial.println(rotations);
    }
  }*/
  if (millis() - lastRPMtime >= 200 && rpm_flag == true) {
    lastRPMtime = millis();
    readDesiredRPM();
    calculateRPM();
    adjustMotorSpeed();
  }
  if (millis() - lastPrintTime >= 1) {
    lastPrintTime = millis();
    handleRotations();  // Ažuriranje vremena poslednjeg ispisa
  }
  // Check IR commands periodically
  if (millis() - lastIRCheck > 20) {  // Check IR every 50ms
    lastIRCheck = millis();
    handleIRCommands();
  }
  if (millis() - lastCollisionCheck > 10) {  // Check IR every 50ms
    lastCollisionCheck = millis();

    handleCollision();
  }


  // Perform follow mode operations every 100ms
  if (followFlag && millis() - lastDistanceCheck > 100) {
    lastDistanceCheck = millis();
    distance = checkDistance();
    followObstacle();
    calculateAverages();
  }

  // Print averages if follow mode is active
  if (followFlag) {
    Serial.print("Average Distance: ");
    Serial.print(averageDistance);
    Serial.print(" cm, Average Error: ");
    Serial.println(averageError);
  }
}
void handleCollision() {
  distance = checkDistance();
  if (distance > 0 && distance < 6.25 && followFlag == false) {
    motorStop();
    collision_flag = true;
    key_flagF = false;
  } else {
    collision_flag = false;
  }
}
// Handle IR commands
void handleIRCommands() {
  if (irrecv.decode(&results)) {
    hexCode = results.value;
    Serial.print("Received HEX Code: ");
    Serial.println(hexCode, HEX);
    irrecv.resume();
    switch (hexCode) {
      case 0xFF18E7:  // Forward
        key_flagF = !key_flagF;
        key_flagB = false;
        key_flagL = false;
        key_flagR = false;
        if (key_flagF == true && collision_flag == false) {
          motorForward();
        } else
          motorStop();
        break;
      case 0xFF4AB5:  // Backward
        key_flagB = !key_flagB;
        key_flagF = false;
        key_flagL = false;
        key_flagR = false;
        if (key_flagB == true) {
          motorBack();
        } else
          motorStop();
        break;
      case 0xFF10EF:  // Left
        key_flagL = !key_flagL;
        key_flagF = false;
        key_flagB = false;
        key_flagR = false;
        if (key_flagL == true) {
          motorLeft();
        } else
          motorStop();
        break;
      case 0xFF5AA5:  // Right
        key_flagR = !key_flagR;
        key_flagF = false;
        key_flagB = false;
        key_flagL = false;
        if (key_flagR == true) {
          motorRight();
        } else
          motorStop();
        break;
      case 0xFF38C7:  // Stop
        motorStop();
        key_flagF = false;
        key_flagB = false;
        key_flagL = false;
        key_flagR = false;
        break;
      case 0xFFC23D:  // Toggle follow mode (button "6")
        followFlag = !followFlag;
        key_flagF = false;
        key_flagB = false;
        key_flagL = false;
        key_flagR = false;
        if (followFlag) {
          Serial.println("Follow Mode Activated");
          resetTrackingData();
        } else {
          Serial.println("Follow Mode Deactivated");
          motorStop();
        }
        break;
      case 0xFFA25D:  // taster 1
        counter_flag = !counter_flag;
        if (counter_flag) {
          pulses = 0;
        } else {
          double distance_passed = double((pulses / 20)) * 0.204;

          Serial.println(pulses);
          Serial.println(distance_passed * 1.2 + 0.22);
        }
        break;
      case 0xFF629D:  // taster 2
        rpm_flag = !rpm_flag;
        if (rpm_flag) {
          key_flagF = false;
          key_flagB = false;
          key_flagL = false;
          key_flagR = false;
        } else {
          motorStop();
        }
      default:
        Serial.println("Unknown Command");
        break;
    }
  }
}

void readDesiredRPM() {

  desiredRPM = 300;                                                             // Čita željeni RPM od korisnika
  desiredRPM = constrain(desiredRPM, userRPMMin, userRPMMax);                   // Ograničava unos
  scaledRPM = map(desiredRPM, userRPMMin, userRPMMax, realRPMMin, realRPMMax);  // Skalira na realni RPM opseg
  Serial.print("[INFO] Željeni RPM korisnika ažuriran na: ");
  Serial.print(desiredRPM);
  Serial.print(" ; Skalirani realni RPM: ");
  Serial.println(scaledRPM);
}
void calculateRPM() {
  unsigned long startTime = millis();
  while (millis() - startTime < 200) {  // Merenje tokom 200 ms
    int currentState = digitalRead(pinB);
    if (currentState != lastState) {
      counts++;  // Povećava brojač impulsa
    }
    lastState = currentState;
  }
  actualRPM = (counts * 60 * 1000) / (pulsesPerRevolution * 200);  // RPM formula za 200 ms
  counts = 0;                                                      // Reset brojača impulsa nakon merenja
}

void adjustMotorSpeed() {
  // Skaliranje realnog RPM-a na PWM
  motorSpeed = map(scaledRPM, realRPMMin, realRPMMax, minPWM, maxPWM);

  // Ograničavanje na opseg PWM vrednosti
  motorSpeed = constrain(motorSpeed, minPWM, maxPWM);

  // Postavljanje brzine motora
  MOTOR_LEVO_GORE.setSpeed(motorSpeed);
  MOTOR_DESNO_GORE.setSpeed(motorSpeed);
  MOTOR_LEVO_DOLE.setSpeed(motorSpeed);
  MOTOR_DESNO_DOLE.setSpeed(motorSpeed);

  // Pokretanje motora
  MOTOR_LEVO_GORE.run(BACKWARD);
  MOTOR_DESNO_GORE.run(FORWARD);
  MOTOR_LEVO_DOLE.run(BACKWARD);
  MOTOR_DESNO_DOLE.run(FORWARD);
}


//zid




void handleRotations() {

  int currentState = digitalRead(pinB);
  if (currentState != lastState) {  // Detekcija bilo koje promene stanja (rastući ili opadajući rub)
    pulses++;                       // Povećava brojač impulsa
  }
  lastState = currentState;
}

// Check distance using ultrasonic sensor
float checkDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);  // Timeout after 10ms
  return duration * 0.0343 / 2;                            // Convert to cm
}

// Follow obstacle while maintaining 15cm distance
void followObstacle() {
  distance = checkDistance();
  float error = distance - 15;  // Calculate error (desired distance = 15cm)
  totalDistance += distance;
  totalError += abs(error);
  samples++;
  Serial.println(distance);
  // Adjust motors based on distance
  if (error > 3) {  // Too far, move forward
    motorForward();
  } else if (error < -5) {  // Too close, move backward
    motorBack();
  } else {  // Within range, stop
    motorStop();
  }
}


// Calculate averages for distance and error
void calculateAverages() {
  if (samples > 0) {
    averageDistance = totalDistance / samples;
    averageError = totalError / samples;
  }
}

// Reset tracking data for follow mode
void resetTrackingData() {
  totalDistance = 0;
  totalError = 0;
  samples = 0;
  averageDistance = 0;
  averageError = 0;
}

// Motor control functions
void motorStop() {
  encoder_enabler = false;
  MOTOR_LEVO_GORE.run(RELEASE);
  MOTOR_DESNO_GORE.run(RELEASE);
  MOTOR_LEVO_DOLE.run(RELEASE);
  MOTOR_DESNO_DOLE.run(RELEASE);
}

void motorBack() {
  MOTOR_LEVO_GORE.setSpeed(220);
  MOTOR_DESNO_GORE.setSpeed(220);
  MOTOR_LEVO_DOLE.setSpeed(220);
  MOTOR_DESNO_DOLE.setSpeed(220);

  MOTOR_LEVO_GORE.run(FORWARD);
  MOTOR_DESNO_GORE.run(BACKWARD);
  MOTOR_LEVO_DOLE.run(FORWARD);
  MOTOR_DESNO_DOLE.run(BACKWARD);
}

void motorForward() {
  encoder_enabler = true;
  MOTOR_LEVO_GORE.setSpeed(255);
  MOTOR_DESNO_GORE.setSpeed(255);
  MOTOR_LEVO_DOLE.setSpeed(255);
  MOTOR_DESNO_DOLE.setSpeed(255);

  MOTOR_LEVO_GORE.run(BACKWARD);
  MOTOR_DESNO_GORE.run(FORWARD);
  MOTOR_LEVO_DOLE.run(BACKWARD);
  MOTOR_DESNO_DOLE.run(FORWARD);
}

void motorRight() {
  MOTOR_LEVO_GORE.run(BACKWARD);
  MOTOR_DESNO_GORE.run(BACKWARD);
  MOTOR_LEVO_DOLE.run(BACKWARD);
  MOTOR_DESNO_DOLE.run(BACKWARD);
}

void motorLeft() {
  MOTOR_LEVO_GORE.run(FORWARD);
  MOTOR_DESNO_GORE.run(FORWARD);
  MOTOR_LEVO_DOLE.run(FORWARD);
  MOTOR_DESNO_DOLE.run(FORWARD);
}
