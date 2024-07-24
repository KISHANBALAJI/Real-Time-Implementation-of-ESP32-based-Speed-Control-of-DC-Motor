#include "PID_v1.h"
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define BLYNK_TEMPLATE_ID "TMPL37PmnVLpp"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "ko1knnau27I9CFWEdEcVSDADW-3b8PzW"


// const double setupPin = 5;    // setup speed
const int enablePin = 2;      // PWM generating pin
const int enablePin2 = 4;
// const int speedPin = 4;       // speed sensor pin
// const int encoderPinA = 23;  // Encoder channel A pin
// const int encoderPinB = 5;  // Encoder channel B pin
#define encoderPinA 23
#define encoderPinB 5

// ADC parameter setting
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

// PID parameters
double Kp,Ki,Kd;
double setupspeed = 0;
// double speed_mapped = 0;
double current_speed = 0;
double actual_speed = 0;
// double error = 0;
// double previousError = 0;
double output = 0;
volatile int pulseCount = 0;  // Number of pulses
unsigned long lastTime = 0;   // Time of the last pulse
// unsigned long previousTime = 0;
// double integral = 0;
// double derivative = 0;

char auth[] = "ko1knnau27I9CFWEdEcVSDADW-3b8PzW";

// WiFi credentials
// char ssid[] = "V2037";
// char pass[] = "kishanmeruga";
// char ssid[] = "Chandu";
// char pass[] = "1234567890";
char ssid[]="navaneeth";
char pass[]="navaneeth";



BLYNK_WRITE(V7) {// Virtual pin for forward rotation

  int value = param.asInt();
  
  if (value == 1) {
    digitalWrite(enablePin, HIGH);
    digitalWrite(enablePin2, LOW);

  }
  else {
    digitalWrite(enablePin, LOW);
    digitalWrite(enablePin2, LOW);
  }
}

BLYNK_WRITE(V4) {
  Kp = param.asDouble();
  Serial.print("kp value is: ");
  Serial.println(Kp);
}

BLYNK_WRITE(V5) {
  Ki = param.asDouble();
  Serial.print("ki value is: ");
  Serial.println(Ki);
}

BLYNK_WRITE(V6) {
  Kd = param.asDouble();
  Serial.print("kd value is: ");
  Serial.println(Kd);
}

BLYNK_WRITE(V3) {
  setupspeed = param.asDouble();
  Serial.print("setupspeed value is: ");
  Serial.println(setupspeed);
  // int dutyCycle = map(setupspeed,0,776.086956522,255,0);
  // ledcWrite(ledChannel, dutyCycle);
  // Blynk.virtualWrite(enablePin,dutyCycle);
}

BlynkTimer timer;
PID myPID(&actual_speed, &output, &setupspeed, Kp, Ki, Kd, DIRECT);


void setup() {

  myPID.SetMode(AUTOMATIC);
  // myPID.SetOutputLimits(0,255);

  // Initialize serial communication
  Serial.begin(115200);

  // Set pin modes
  // pinMode(speedPin, INPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  // pinMode(enablePin, INPUT);
  // pinMode(enablePin2, INPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleInterrupt, RISING);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(enablePin, ledChannel);

  Blynk.begin(auth, ssid, pass);
}

void loop() {
  actual_speed = calculateSpeed();
  int input = actual_speed;

  
  //PID SECTION

  //Adjusted PID values
  myPID.SetTunings(Kp, Ki, Kd);
  
  //PID calculation
  myPID.Compute();
  // int dutyCycle = map(setupspeed,0,776.086956522,255,0);
  // ledcWrite(ledChannel, dutyCycle);
  int manpulated = map(output,0,255,255,0);
  ledcWrite(ledChannel, manpulated);
  
  Serial.print("actual_speed: ");
  Serial.print(actual_speed);
  Serial.print("Current Speed: ");
  Serial.print(current_speed);
  Serial.print(" RPM\t");
  Serial.print("PID Output: ");
  Serial.print(output);
  Serial.print("error");
  Serial.println(setupspeed-actual_speed);
  Blynk.virtualWrite(V0, actual_speed);
  Blynk.virtualWrite(V1, actual_speed);

  delay(100);  // Delay between iterations
}

void handleInterrupt() {
  // Increment pulse count on each rising edge of encoder channel A
  // if (digitalRead(encoderPinA) == HIGH) {
  pulseCount++;
  // }
  // int A=digitalRead(encoderPinA);
  // if (A>0) {
  //   pulseCount++;
  // } 
  // else {
  //   pulseCount--;
  // }
}

double calculateSpeed(){
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;

  if (elapsedTime >= 100)
  {
    double pulsesPerRevolution = 2500.0;  // Encoder resolution (example: 2500 pulses per revolution)
    current_speed = (float)((pulseCount * 600) /(pulsesPerRevolution) ); // Calculate speed in RPM
    // intercept: 10.255499800613649
    // slope: 0.5121243149526155
    actual_speed = 0.5121243149526155*(current_speed)+10.255499800613649;

    pulseCount = 0;
    lastTime = currentTime;

    return actual_speed;
    Blynk.virtualWrite(V0, actual_speed);
    Blynk.virtualWrite(V1, actual_speed);
  }
}