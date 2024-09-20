// Lib
#include <Arduino.h>

// Globals & Constants
#define BAT_VOLTAGE 7.8
#define KP 0.05
#define REDUCER 4
#define PWM_MAX 200

#define COUNTS_PER_REV 3200
#define COUNTS_PER_RAD 509
#define METERS_PER_REV 0.4826
#define WHEEL_WIDTH 0.2032

// Control system vars
int numOne = 0;
int numTwo = 0;

int target1 = 0;
int target2 = 0;

double loop_delay = 1500;
long lastTime = 0;

bool first = true;

long lastMeas1 = 0;
long lastMeas2 = 0;

long lastA1 = 0;
long lastB1 = 0;

long lastA2 = 0;
long lastB2 = 0;

long rightPos = 0;
long leftPos = 0;

long prevLooprightPos = 0;
long prevLoopleftPos = 0;

// Localization vars

const double TARGET_RAD_PER_SEC = 0.5;
unsigned long desired_Ts_ms = 25;
unsigned long last_time_ms;
int lastA1 = 0;
long lastB1 = 0;
long rightPos = 0;
long leftPos = 0; // Position Count
int lastA2 = 0;
long lastB2 = 0;
long lastMeas1 = 0;
long lastMeas2 = 0;

// Global Variables for position
double x = 0;
double y = 0;
double phi = 0;
long lastPrint = 0;
long previousCountA = 0, previousCountB = 0;

void encoderUpdateRight() {  // Covered Motor interrupt callback
  if ((micros() - 300) > lastMeas1) {
    int thisA1 = digitalRead(3);
    int thisB1 = digitalRead(6);

    if ((thisA1 == thisB1)) {
      rightPos += 2;  // Forward
    } else {
      rightPos -= 2;  // Backwards
    }
    lastA1 = thisA1;
    lastB1 = thisB1;
  }
  lastMeas1 = micros();
}

void encoderUpdateLeft() {  // Uncovered Motor interrupt callback
  if ((micros() - 300) > lastMeas2) {
    int thisA2 = digitalRead(2);
    int thisB2 = digitalRead(5);

    if ((thisA2 == thisB2)) {
      leftPos -= 2;  // Backwards
    } else {
      leftPos += 2;  // Forward
    }
    lastA2 = thisA2;
    lastB2 = thisB2;
  }
  lastMeas2 = micros();
}

// Functions
void position(double dl, double dr){
    x += cos(phi) * ((dl + dr) / 2);
    y += sin(phi) * ((dl + dr) / 2);

    phi += (1 / WHEEL_WIDTH) * (dl - dr);

    if (lastPrint + 250 < millis()){
        // Serial.println(String(millis()) + "," + String(x, 4) + "," + String(y, 4) + ", " + String(phi, 4));
        lastPrint = millis();
    }
}

// Setup
void setup(){
    // Pin setup

    // Encoder Setup
    pinMode(2, INPUT); // Left Encoder A
    pinMode(3, INPUT); // Right Encoder A
    pinMode(5, INPUT); // Left Encoder B
    pinMode(6, INPUT); // Right Encoder B

    // Motor Setup
    pinMode(4, OUTPUT);        // Enable pin
    digitalWrite(4, HIGH);     // Set Enable High
    pinMode(7, OUTPUT);        // Motor 1 Direction
    pinMode(8, OUTPUT);        // Motor 2 Direction
    pinMode(9, OUTPUT);        // Motor 1 Speed Output
    pinMode(10, OUTPUT);       // Motor 2 Speed Output
    pinMode(12, INPUT_PULLUP); // Fault pin
    // A0 is Motor Current 1 Sense
    // A1 is Motor Current 2 Sense
    
    // Interrupts
    attachInterrupt(digitalPinToInterrupt(3), encoderUpdateRight, CHANGE);  // Interrupt for covered motor
    attachInterrupt(digitalPinToInterrupt(2), encoderUpdateLeft, CHANGE);  // Interrupt for uncovered motor
    
    // Serial
    Serial.begin(115200);
    Serial.println("Program Start!");
    
    // Set Start Time
    lastTime = micros();
}

// Loop
void loop(){
    // put your main code here, to run repeatedly:
    while (loop_delay + lastTime > micros()){
        if (Serial.available()){ // wait for data available
            char in = Serial.read();
            if (first && isDigit(in))
            {
                numOne = in - 48;
                Serial.println("NUM ONE IS: " + String(numOne) + ". Target1 is: " + String(target1));
                first = false;
            }
            else if (isDigit(in))
            {
                numTwo = in - 48;
                first = true;
            }
        }
    }

    lastTime = micros();
    long looprightPos = rightPos; // Position of motor 1
    long loopleftPos = leftPos; // Position of motor 2

    target1 = numOne * 1600;
    target2 = numTwo * 1600;

    int error1 = target1 - rightPos;
    int error2 = target2 - leftPos;

    double voltage1 = min((error1 * KP) / REDUCER, BAT_VOLTAGE);
    double voltage2 = min((error2 * KP) / REDUCER, BAT_VOLTAGE);

    voltage1 = max(-BAT_VOLTAGE, voltage1);
    voltage2 = max(-BAT_VOLTAGE, voltage2);

    if (voltage1 < 0){ // MOTOR 1 On pololu
        digitalWrite(7, HIGH);
    }
    else{
        digitalWrite(7, LOW);
    }
    if (voltage2 < 0){
        digitalWrite(8, LOW);
    }
    else{
        digitalWrite(8, HIGH);
    }

    // Calculate PWM values based
    double pwm1 = min((abs((double)voltage1 / BAT_VOLTAGE) * 255), PWM_MAX);
    double pwm2 = min((abs((double)voltage2 / BAT_VOLTAGE) * 255), PWM_MAX);

    // Serial.println(String(target1) + "," + String(error1) + "," + String(voltage1));

    // Write to pins
    analogWrite(9, pwm1);
    analogWrite(10, pwm2);

    // Find Count difference
    long diffLeftCount = long(myEnc2()) - previousCountA;
    long diffRightCount = long(myEnc1()) - previousCountB;

    previousCountA = myEnc2();
    previousCountB = myEnc1();

    double diffLeftMeter = ((double)diffLeftCount / COUNTS_PER_REV) * METERS_PER_REV;
    double diffRightMeter = ((double)diffRightCount / COUNTS_PER_REV) * METERS_PER_REV;

    position(diffLeftMeter, diffRightMeter);

    // Record last position
    prevLooprightPos = looprightPos;
    prevLoopleftPos = loopleftPos;
}