#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Audio.h>
#include <Encoder.h>
#include <math.h>

// --- 1. PHYSICAL DIMENSIONS & CALIBRATION ---
const float L1 = 61.0; 
const float L2 = 59.0; 

const float START_X = -8;   
const float START_Y = 88.0; 

const float HIP_CPD  = 11.644; 
const float KNEE_CPD = 11.644; 

float hipStartAngle, kneeStartAngle;

// --- 2. PIN DEFINITIONS ---
const int PIN_MIC = 22;
const int H1_DIR = 28; const int H1_PWM = 29;
const int H2_DIR = 24; const int H2_PWM = 25;
const int K1_DIR = 10; const int K1_PWM = 12;
const int K2_DIR = 8;  const int K2_PWM = 9;
const int W1_DIR = 2;  const int W1_PWM = 3;
const int W2_DIR = 4;  const int W2_PWM = 6;

// --- 3. AUDIO OBJECTS ---
AudioSynthWaveformSine sine1;          
AudioOutputI2S         i2s1;           
AudioConnection        p1(sine1, 0, i2s1, 0);
AudioConnection        p2(sine1, 0, i2s1, 1);

// --- 4. PID STRUCTURE WITH MOTION PROFILING ---
struct PIDController {
  float Kp = 5.0, Ki = 0.0, Kd = 3.0; 
  long target = 0;             // The ultimate destination
  float currentSetpoint = 0;   // The moving point the PID actually follows
  float maxVel = 15;          // Max counts to move per 10ms loop
  
  long lastError = 0;
  float integral = 0;
  int outMax = 200; 

  float calculate(long current) {
    // --- SLEW RATE LOGIC (MOTION PROFILE) ---
    if (currentSetpoint < target) {
      currentSetpoint += maxVel;
      if (currentSetpoint > target) currentSetpoint = target;
    } else if (currentSetpoint > target) {
      currentSetpoint -= maxVel;
      if (currentSetpoint < target) currentSetpoint = target;
    }

    // PID calculation based on the moving setpoint
    long error = (long)currentSetpoint - current;
    integral = constrain(integral + error, -100, 100);
    float derivative = error - lastError;
    lastError = error;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    return constrain(output, -outMax, outMax);
  }
};

PIDController pidH1, pidH2, pidK1, pidK2;
int w1_speed = 0; int w2_speed = 0; 

// --- 5. ENCODERS & SENSORS ---
Encoder encH1(39, 40); 
Encoder encH2(38, 37); 
Encoder encK1(36, 35); 
Encoder encK2(33, 34); 

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
unsigned long lastPIDTime = 0;

// --- 6. IK SOLVER ---
void solveIK(float x, float y, float &hAng, float &kAng) {
  float cosTheta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
  cosTheta2 = constrain(cosTheta2, -1.0, 1.0); 
  float theta2 = acos(cosTheta2); 
  float theta1 = atan2(x, y) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
  hAng = theta1 * 180.0 / PI;
  kAng = theta2 * 180.0 / PI;
}

// --- 7. SETUP ---
void setup() {
  Serial.begin(115200);  
  Serial1.begin(921600); 

  AudioMemory(12);
  sine1.amplitude(0);
  
  int pins[] = {H1_DIR, H1_PWM, H2_DIR, H2_PWM, K1_DIR, K1_PWM, K2_DIR, K2_PWM, W1_DIR, W1_PWM, W2_DIR, W2_PWM};
  for(int p : pins) pinMode(p, OUTPUT);

  if(!bno.begin()) Serial.println("IMU OFFLINE");
  analogWriteFrequency(H1_PWM, 20000); 

  solveIK(START_X, START_Y, hipStartAngle, kneeStartAngle);
  
  float initHipAng, initKneeAng;
  solveIK(0, 110, initHipAng, initKneeAng);

  // Set initial targets for standing up
  pidH1.target = pidH2.target = (long)((hipStartAngle - initHipAng) * HIP_CPD);
  pidK1.target = pidK2.target = (long)((initKneeAng - kneeStartAngle) * KNEE_CPD);

  // Initialize Setpoints to 0 so the robot ramps from power-on position
  pidH1.currentSetpoint = pidH2.currentSetpoint = 0;
  pidK1.currentSetpoint = pidK2.currentSetpoint = 0;

  Serial.println("Robot Ready. Standing up with Motion Profile...");
}

// --- 8. LOOP ---
void loop() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }

  if (millis() - lastPIDTime >= 10) {
    lastPIDTime = millis();
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

// --- 9. COMMAND PROCESSOR ---
void processCommand(String cmd) {
  // --- ADJUST VELOCITY (New Command) ---
  // Usage: VEL:H1:10 (Set H1 speed to 10 counts/10ms)
  // Usage: VEL:ALL:5 (Set all joints to 5 counts/10ms)
  if (cmd.startsWith("VEL:")) {
    int firstColon = cmd.indexOf(':', 4);
    String joint = cmd.substring(4, firstColon);
    float val = cmd.substring(firstColon + 1).toFloat();

    if (joint == "H1") pidH1.maxVel = val;
    else if (joint == "H2") pidH2.maxVel = val;
    else if (joint == "K1") pidK1.maxVel = val;
    else if (joint == "K2") pidK2.maxVel = val;
    else if (joint == "ALL") {
      pidH1.maxVel = pidH2.maxVel = pidK1.maxVel = pidK2.maxVel = val;
    }
    Serial1.printf("CONFIRM: %s MaxVel set to %.2f\n", joint.c_str(), val);
  }

  // --- COORDINATE MOVE (IK) ---
  else if (cmd.startsWith("POS:")) {
    int firstColon = cmd.indexOf(':', 4);
    float tx = cmd.substring(4, firstColon).toFloat();
    float ty = cmd.substring(firstColon + 1).toFloat();
    float hA, kA;
    solveIK(tx, ty, hA, kA);
    pidH1.target = pidH2.target = (long)((hipStartAngle - hA) * HIP_CPD);
    pidK1.target = pidK2.target = (long)((kA - kneeStartAngle) * KNEE_CPD);
    Serial1.printf("CONFIRM: Profiling to X%.1f Y%.1f\n", tx, ty);
  }

  // --- GAIN TUNING ---
  else if (cmd.startsWith("GAIN:")) {
    int firstColon = cmd.indexOf(':', 5);
    int secondColon = cmd.indexOf(':', firstColon + 1);
    String joint = cmd.substring(5, firstColon);
    char pType = cmd.charAt(secondColon - 1);
    float val = cmd.substring(secondColon + 1).toFloat();
    PIDController* targetPID;
    if (joint == "H1") targetPID = &pidH1; else if (joint == "H2") targetPID = &pidH2;
    else if (joint == "K1") targetPID = &pidK1; else targetPID = &pidK2;
    if (pType == 'P') targetPID->Kp = val; else if (pType == 'I') targetPID->Ki = val; else if (pType == 'D') targetPID->Kd = val;
    Serial1.printf("CONFIRM: %s %c set to %.2f\n", joint.c_str(), pType, val);
  }

  // --- ZEROING ---
  else if (cmd == "ZERO:ALL") {
    encH1.write(0); encH2.write(0); encK1.write(0); encK2.write(0);
    pidH1.target = pidH2.target = pidK1.target = pidK2.target = 0;
    pidH1.currentSetpoint = pidH2.currentSetpoint = pidK1.currentSetpoint = pidK2.currentSetpoint = 0;
    solveIK(START_X, START_Y, hipStartAngle, kneeStartAngle);
    Serial1.println("CONFIRM: All Zeroed");
  }

  // --- DIAGNOSTICS & WHEELS ---
  else if (cmd == "GET_IMU") {
    sensors_event_t event; bno.getEvent(&event);
    Serial1.printf("IMU: PITCH:%.2f ROLL:%.2f\n", event.orientation.y, event.orientation.z);
  }
  else if (cmd == "GET_ENC") {
    Serial1.printf("ENC: H1:%ld H2:%ld K1:%ld K2:%ld\n", encH1.read(), encH2.read(), encK1.read(), encK2.read());
  }
  else if (cmd == "GET_MIC") {
    Serial1.printf("MIC: %d\n", analogRead(PIN_MIC));
  }
  else if (cmd == "TEST_SPK") {
    sine1.frequency(440); sine1.amplitude(0.5); delay(300); sine1.amplitude(0);
  }
  else if (cmd.startsWith("WHEEL:")) {
    int spd = cmd.substring(9).toInt();
    if (cmd.substring(6,8) == "W1") w1_speed = spd; else w2_speed = spd;
  }
  else if (cmd == "TEST_H1") pulseMotor("H1", H1_DIR, H1_PWM, true);
  else if (cmd == "TEST_H2") pulseMotor("H2", H2_DIR, H2_PWM, false);
  else if (cmd == "TEST_K1") pulseMotor("K1", K1_DIR, K1_PWM, false);
  else if (cmd == "TEST_K2") pulseMotor("K2", K2_DIR, K2_PWM, true);
}

void pulseMotor(String name, int dPin, int pPin, bool inverted) {
  digitalWrite(dPin, inverted ? LOW : HIGH); analogWrite(pPin, 100); delay(200); analogWrite(pPin, 0);
}