#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Audio.h>
#include <Encoder.h>
#include <math.h>

// --- 1. PHYSICAL DIMENSIONS ---
const float L1 = 61.0; 
const float L2 = 59.0; 
const float START_X = -8;   
const float START_Y = 88.0; 
const float HIP_CPD  = 11.644; 
const float KNEE_CPD = 11.644; 

float hipStartAngle, kneeStartAngle;

// --- 2. BALANCING SETTINGS ---
bool balanceEnabled = false;   
bool debugLevel = false;
float targetX = 3.0;          
float targetY = 106.0;        

// WHEEL GAINS (Pitch)
float kpWheel = -40.0;         
float kdWheel = 4.0;          
float wheelMax = 500.0;
float pitchDeadband = 10.0; 

// LEG GAINS (Roll)
float kpRoll  = 0.3; 
float smoothing = 0.2;        
float rollDeadband = 3.0;     
float rollLimit = 6.0;        

float filteredPitch = 0.0;
float filteredRoll  = 90.0;
float lastPitch = 0.0;

const float PITCH_OFFSET = 0;  
const float ROLL_OFFSET  = 90; 

// --- 3. LUCKY ACTION STATE ---
enum LuckyState { IDLE, THINKING, DANCING };
LuckyState currentState = IDLE;
unsigned long actionTimer = 0;

// --- 4. PIN DEFINITIONS ---
const int H1_DIR = 28; const int H1_PWM = 29;
const int H2_DIR = 24; const int H2_PWM = 25;
const int K1_DIR = 10; const int K1_PWM = 12;
const int K2_DIR = 8;  const int K2_PWM = 9;
const int W1_DIR = 2;  const int W1_PWM = 3;
const int W2_DIR = 4;  const int W2_PWM = 6;
const int PIN_MIC = 22;

// --- 5. AUDIO ---
AudioSynthWaveformSine sine1;          
AudioOutputI2S         i2s1;           
AudioConnection        p1(sine1, 0, i2s1, 0);
AudioConnection        p2(sine1, 0, i2s1, 1);

// --- 6. PID STRUCTURE ---
struct PIDController {
  float Kp = 5.0, Ki = 0.0, Kd = 3.0; 
  long target = 0;             
  float currentSetpoint = 0;   
  float maxVel = 15;          
  long lastError = 0;
  float integral = 0;
  int outMax = 200; 

  float calculate(long current) {
    if (currentSetpoint < target) {
      currentSetpoint += maxVel;
      if (currentSetpoint > target) currentSetpoint = target;
    } else if (currentSetpoint > target) {
      currentSetpoint -= maxVel;
      if (currentSetpoint > target) currentSetpoint = target;
    }
    long error = (long)currentSetpoint - current;
    integral = constrain(integral + error, -100, 100);
    float derivative = error - lastError;
    lastError = error;
    return constrain((Kp * error) + (Ki * integral) + (Kd * derivative), -outMax, outMax);
  }
};

PIDController pidH1, pidH2, pidK1, pidK2;
int w1_speed = 0; int w2_speed = 0; 

Encoder encH1(39, 40); Encoder encH2(38, 37); 
Encoder encK1(36, 35); Encoder encK2(33, 34); 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
unsigned long lastPIDTime = 0;

void solveIK(float x, float y, float &hAng, float &kAng) {
  float cosTheta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
  cosTheta2 = constrain(cosTheta2, -1.0, 1.0); 
  float theta2 = acos(cosTheta2); 
  float theta1 = atan2(x, y) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
  hAng = theta1 * 180.0 / PI;
  kAng = theta2 * 180.0 / PI;
}

void setup() {
  Serial.begin(115200); Serial1.begin(921600); 
  AudioMemory(12); sine1.amplitude(0);
  int pins[] = {H1_DIR, H1_PWM, H2_DIR, H2_PWM, K1_DIR, K1_PWM, K2_DIR, K2_PWM, W1_DIR, W1_PWM, W2_DIR, W2_PWM};
  for(int p : pins) pinMode(p, OUTPUT);
  bno.begin();
  analogWriteFrequency(H1_PWM, 20000); 

  solveIK(START_X, START_Y, hipStartAngle, kneeStartAngle);
  float initH, initK;
  solveIK(targetX, targetY, initH, initK);
  pidH1.target = pidH2.target = (long)((initH - hipStartAngle) * HIP_CPD);
  pidK1.target = pidK2.target = (long)((initK - kneeStartAngle) * KNEE_CPD);
}

void loop() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }

  if (millis() - lastPIDTime >= 10) {
    lastPIDTime = millis();

    sensors_event_t event;
    bno.getEvent(&event);
    filteredPitch = (smoothing * event.orientation.y) + ((1.0 - smoothing) * filteredPitch);
    filteredRoll  = (smoothing * event.orientation.z) + ((1.0 - smoothing) * filteredRoll);

    float pErr = filteredPitch - PITCH_OFFSET;
    float rErr = filteredRoll - ROLL_OFFSET;
    float pitchRate = filteredPitch - lastPitch;
    lastPitch = filteredPitch;

    // Default modifiers for Actions
    float autoW1 = 0; 
    float autoW2 = 0;

    // --- 1. ACTION TIMEOUT CHECK ---
    if (currentState != IDLE && millis() > actionTimer) {
        currentState = IDLE;
        sine1.amplitude(0);
        w1_speed = 0;  // Explicitly turn off wheel power
        w2_speed = 0;
    }

    // --- 2. LUCKY ACTIONS CALCULATION ---
    if (currentState == THINKING) {
      autoW1 = 80; // Circular spin (one wheel only)
      if (millis() % 60 < 10) sine1.frequency(random(300, 1200));
      sine1.amplitude(0.2);
    } 
    else if (currentState == DANCING) {
      // Differential oscillation (Back and forth)
      float danceTurn = sin(millis() * 0.008) * 100.0; 
      autoW1 = danceTurn;
      autoW2 = -danceTurn;
    }

    // --- 3. POWER MERGING & EXECUTION ---
    if (balanceEnabled) {
      // Calculate Balancing Base
      float balancePower = 0;
      if (abs(pErr) > pitchDeadband) {
          balancePower = (pErr * kpWheel) + (pitchRate * kdWheel);
      }
      // Add balance to action movement
      w1_speed = constrain((int)(balancePower + autoW1), -wheelMax, wheelMax);
      w2_speed = constrain((int)(balancePower + autoW2), -wheelMax, wheelMax);

      // Kinematic Roll Logic
      float rollEffect = 0;
      if (abs(rErr) > rollDeadband) {
          rollEffect = constrain(rErr * kpRoll, -rollLimit, rollLimit);
      }
      float hL, kL, hR, kR;
      solveIK(targetX, targetY + rollEffect, hL, kL);  
      solveIK(targetX, targetY - rollEffect, hR, kR); 
      pidH1.target = (long)((hL - hipStartAngle) * HIP_CPD);
      pidK1.target = (long)((kL - kneeStartAngle) * KNEE_CPD);
      pidH2.target = (long)((hR - hipStartAngle) * HIP_CPD);
      pidK2.target = (long)((kR - kneeStartAngle) * KNEE_CPD);
    } 
    else if (currentState != IDLE) {
      // Balancing is off, but an action is running: apply auto values directly
      w1_speed = (int)autoW1;
      w2_speed = (int)autoW2;
    }

    // Apply Calculated Power to All Motors
    applyMotorPower(H1_DIR, H1_PWM, -pidH1.calculate(encH1.read())); 
    applyMotorPower(H2_DIR, H2_PWM, pidH2.calculate(encH2.read()));
    applyMotorPower(K1_DIR, K1_PWM, pidK1.calculate(encK1.read()));
    applyMotorPower(K2_DIR, K2_PWM, -pidK2.calculate(encK2.read())); 
    
    applyMotorPower(W1_DIR, W1_PWM, -w1_speed);                     
    applyMotorPower(W2_DIR, W2_PWM, w2_speed);
  }
}

void applyMotorPower(int dPin, int pPin, float val) {
  digitalWrite(dPin, val >= 0 ? HIGH : LOW);
  analogWrite(pPin, abs((int)val));
}

void processCommand(String cmd) {
  cmd.toUpperCase();
  
  // A. MANUAL WHEEL CONTROL (Parser fix)
  if (cmd.startsWith("W ") || cmd.startsWith("WHEEL:")) {
    int val = 0;
    // Extract the numeric value from the end of the string
    int lastSpace = cmd.lastIndexOf(" ");
    int lastColon = cmd.lastIndexOf(":");
    int index = max(lastSpace, lastColon);
    val = cmd.substring(index + 1).toInt();

    if (cmd.indexOf("W1") != -1) w1_speed = val;
    else if (cmd.indexOf("W2") != -1) w2_speed = val;
  }
  // B. ACTION COMMANDS
  else if (cmd == "LUCKY:THINK") { currentState = THINKING; actionTimer = millis() + 5000; }
  else if (cmd == "LUCKY:DANCE") { currentState = DANCING; actionTimer = millis() + 5000; }
  
  // C. CORE CONTROLS
  else if (cmd == "BAL:ON") balanceEnabled = true;
  else if (cmd == "BAL:OFF") { balanceEnabled = false; w1_speed = 0; w2_speed = 0; currentState = IDLE; }
  else if (cmd == "DEBUG:ON") debugLevel = true;
  else if (cmd == "DEBUG:OFF") debugLevel = false;
  
  // D. DIAGNOSTICS & HARDWARE
  else if (cmd == "GET_IMU") { sensors_event_t e; bno.getEvent(&e); Serial1.printf("IMU: P:%.2f R:%.2f\n", e.orientation.y, e.orientation.z); }
  else if (cmd == "GET_ENC") { Serial1.printf("ENC: %ld %ld %ld %ld\n", encH1.read(), encH2.read(), encK1.read(), encK2.read()); }
  else if (cmd == "ZERO:ALL") {
    encH1.write(0); encH2.write(0); encK1.write(0); encK2.write(0);
    pidH1.target = pidH2.target = pidK1.target = pidK2.target = 0;
    pidH1.currentSetpoint = pidH2.currentSetpoint = pidK1.currentSetpoint = pidK2.currentSetpoint = 0;
  }
  else if (cmd == "TEST_SPK") { sine1.frequency(440); sine1.amplitude(0.5); delay(200); sine1.amplitude(0); }
}