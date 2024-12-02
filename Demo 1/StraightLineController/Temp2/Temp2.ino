#define BAT_VOLTAGE 8.5  // 7.8
#define KP 0.003          // 0.05. 0.005
#define KI 0.1           // 0.05
#define KDrift 3.5  // 3.5
#define KDriftReducer 20// 16
#define I_MAX 0.7         // 0.5
#define REDUCER 4         // 4
#define PWM_MAX 60       // 100
#define WHEEL_CIRC 0.47  // 0.47 very close, 0.469
#define COUNTS_PER_REV 3200

double targetPosition = 0;  // Position in meters

// General vars
int numOne = 0;
int numTwo = 0;

int target1 = 0;
int target2 = 0;

double loop_delay = 1500;
long lastTime = 0;

bool first = true;

// Control system vars
long lastMeas1 = 0;
long lastMeas2 = 0;

long lastA1 = 0;
long lastB1 = 0;

long lastA2 = 0;
long lastB2 = 0;

long i1 = 0;
long i2 = 0;

long prevLoopi1 = 0;
long prevLoopi2 = 0;

double integral1 = 0;
double integral2 = 0;

void encoderUpdate1() {  // Covered Motor interrupt callback
  if ((micros() - 300) > lastMeas1) {
    int thisA1 = digitalRead(3);
    int thisB1 = digitalRead(6);

    if ((thisA1 == thisB1)) {
      i1 += 2;  // Forward
    } else {
      i1 -= 2;  // Backwards
    }
    lastA1 = thisA1;
    lastB1 = thisB1;
  }
  lastMeas1 = micros();  // Debounce for encoder
}

void encoderUpdate2() {  // Uncovered Motor interrupt callback
  if ((micros() - 300) > lastMeas2) {
    int thisA2 = digitalRead(2);
    int thisB2 = digitalRead(5);

    if ((thisA2 == thisB2)) {
      i2 -= 2;  // Backwards
    } else {
      i2 += 2;  // Forward
    }
    lastA2 = thisA2;
    lastB2 = thisB2;
  }
  lastMeas2 = micros();  // Debounce for encoder
}

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  pinMode(4, OUTPUT);                                                 // Enable pin
  pinMode(7, OUTPUT);                                                 // Motor 1 Direction
  pinMode(8, OUTPUT);                                                 // Motor 2 Direction
  pinMode(9, OUTPUT);                                                 // Motor 1 Speed Output
  pinMode(10, OUTPUT);                                                // Motor 2 Speed Output
  attachInterrupt(digitalPinToInterrupt(3), encoderUpdate1, CHANGE);  // Interrupt for covered motor
  attachInterrupt(digitalPinToInterrupt(2), encoderUpdate2, CHANGE);  // Interrupt for uncovered motor
  Serial.begin(115200);
  digitalWrite(4, HIGH);
  //delay(3500);
  Serial.println("Ready!");  // Start MATLAB Reading
  //delay(1500);
  lastTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (loop_delay + lastTime > micros()) {
    if (Serial.available()) {  //wait for data available
      String in = Serial.readString();
      Serial.println("Input " + in);
      targetPosition = in.toDouble();
      targetPosition *= 0.3048;
      Serial.println("Going to " + String(targetPosition, 4));
    }
  }

  lastTime = micros();  // Log loop start time for delay system
  long loopi1 = i1;     // Position of motor 1
  long loopi2 = i2;     // Position of motor 2

  target1 = (targetPosition / WHEEL_CIRC) * COUNTS_PER_REV;  // Set target position
  target2 = (targetPosition / WHEEL_CIRC) * COUNTS_PER_REV;

  int error1 = target1 - i1;  // Calculate error
  int error2 = target2 - i2;

  double drift = loopi1 - loopi2;

  double comp1 = 0;
  double comp2 = 0;

  if (drift > 0) {
    comp2 = abs(drift);
    comp1 = 0;
  } else if (drift < 0) {
    comp1 = abs(drift);
    comp2 = 0;
  }

  if (comp1 < KDriftReducer) {
    comp1 = 0;
  } else if (comp2 < KDriftReducer) {
    comp2 = 0;
  }

  Serial.println(String(comp1) + "," + String(comp2) + "," + String(drift));

  integral1 += (error1 * (loop_delay / 1000));                                                  // Calculate integral error factor
  integral1 = constrain(integral1, -I_MAX * (loop_delay / 1000), I_MAX * (loop_delay / 1000));  // Prevent windup
  integral2 += (error2 * (loop_delay / 1000));
  integral2 = constrain(integral2, -I_MAX * (loop_delay / 1000), I_MAX * (loop_delay / 1000));

  double voltage1 = min((error1 * KP) + (KI * integral1) - (KDrift * comp2), BAT_VOLTAGE);  // Limit control voltage to battery voltage
  double voltage2 = min((error2 * KP) + (KI * integral2) - (KDrift * comp1), BAT_VOLTAGE);

  // double voltage1 = min((error1 * KP) + (KI * integral1) - (KDrift * comp2), BAT_VOLTAGE);  // Limit control voltage to battery voltage
  // double voltage2 = min((error2 * KP) + (KI * integral2) - (KDrift * comp1), BAT_VOLTAGE);

  // double voltage1 = min((error1 * KP)/REDUCER, BAT_VOLTAGE);
  // double voltage2 = min((error2 * KP)/REDUCER, BAT_VOLTAGE);

  voltage1 = max(-BAT_VOLTAGE, voltage1);
  voltage2 = max(-BAT_VOLTAGE, voltage2);

  if (voltage1 < 0) {  // MOTOR 1 On pololu. Direction control
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }
  if (voltage2 < 0) {
    digitalWrite(8, LOW);
  } else {
    digitalWrite(8, HIGH);
  }

  // Calculate PWM values based
  double pwm1 = min((abs((double)voltage1 / BAT_VOLTAGE) * 255), PWM_MAX);  // Calculate PWM based on voltage input
  double pwm2 = min((abs((double)voltage2 / BAT_VOLTAGE) * 255), PWM_MAX);

  // Serial.println(String(target1) + "," + String(error1) + "," + String(voltage1));

  // Write to pins
  analogWrite(9, pwm1);
  analogWrite(10, pwm2);

  // Record last position
  prevLoopi1 = loopi1;
  prevLoopi2 = loopi2;
}




/*
#define BAT_VOLTAGE 7.8  // 7.8
#define KP 0.005          // 0.05
#define KI 0.05
#define KDrift 15  // 10
#define KDriftReducer 20// 16
#define I_MAX 0.5
#define REDUCER 4         // 4
#define PWM_MAX 100       // 200
#define WHEEL_CIRC 0.47  // 0.48 very close, 0.469
#define COUNTS_PER_REV 3200

double targetPosition = 0;  // Position in meters

// General vars
int numOne = 0;
int numTwo = 0;

int target1 = 0;
int target2 = 0;

double loop_delay = 1500;
long lastTime = 0;

bool first = true;

// Control system vars
long lastMeas1 = 0;
long lastMeas2 = 0;

long lastA1 = 0;
long lastB1 = 0;

long lastA2 = 0;
long lastB2 = 0;

long i1 = 0;
long i2 = 0;

long prevLoopi1 = 0;
long prevLoopi2 = 0;

double integral1 = 0;
double integral2 = 0;

void encoderUpdate1() {  // Covered Motor interrupt callback
  if ((micros() - 300) > lastMeas1) {
    int thisA1 = digitalRead(3);
    int thisB1 = digitalRead(6);

    if ((thisA1 == thisB1)) {
      i1 += 2;  // Forward
    } else {
      i1 -= 2;  // Backwards
    }
    lastA1 = thisA1;
    lastB1 = thisB1;
  }
  lastMeas1 = micros();  // Debounce for encoder
}

void encoderUpdate2() {  // Uncovered Motor interrupt callback
  if ((micros() - 300) > lastMeas2) {
    int thisA2 = digitalRead(2);
    int thisB2 = digitalRead(5);

    if ((thisA2 == thisB2)) {
      i2 -= 2;  // Backwards
    } else {
      i2 += 2;  // Forward
    }
    lastA2 = thisA2;
    lastB2 = thisB2;
  }
  lastMeas2 = micros();  // Debounce for encoder
}

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  pinMode(4, OUTPUT);                                                 // Enable pin
  pinMode(7, OUTPUT);                                                 // Motor 1 Direction
  pinMode(8, OUTPUT);                                                 // Motor 2 Direction
  pinMode(9, OUTPUT);                                                 // Motor 1 Speed Output
  pinMode(10, OUTPUT);                                                // Motor 2 Speed Output
  attachInterrupt(digitalPinToInterrupt(3), encoderUpdate1, CHANGE);  // Interrupt for covered motor
  attachInterrupt(digitalPinToInterrupt(2), encoderUpdate2, CHANGE);  // Interrupt for uncovered motor
  Serial.begin(115200);
  digitalWrite(4, HIGH);
  //delay(3500);
  Serial.println("Ready!");  // Start MATLAB Reading
  //delay(1500);
  lastTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (loop_delay + lastTime > micros()) {
    if (Serial.available()) {  //wait for data available
      String in = Serial.readString();
      Serial.println("Input " + in);
      targetPosition = in.toDouble();
      targetPosition *= 0.3048;
      Serial.println("Going to " + String(targetPosition, 4));
    }
  }

  lastTime = micros();  // Log loop start time for delay system
  long loopi1 = i1;     // Position of motor 1
  long loopi2 = i2;     // Position of motor 2

  target1 = (targetPosition / WHEEL_CIRC) * COUNTS_PER_REV;  // Set target position
  target2 = (targetPosition / WHEEL_CIRC) * COUNTS_PER_REV;

  int error1 = target1 - i1;  // Calculate error
  int error2 = target2 - i2;

  double drift = loopi1 - loopi2;

  double comp1 = 0;
  double comp2 = 0;

  if (drift > 0) {
    comp2 = abs(drift);
    comp1 = 0;
  } else if (drift < 0) {
    comp1 = abs(drift);
    comp2 = 0;
  }

  if (comp1 < KDriftReducer) {
    comp1 = 0;
  } else if (comp2 < KDriftReducer) {
    comp2 = 0;
  }

  Serial.println(String(comp1) + "," + String(comp2) + "," + String(drift));

  integral1 += (error1 * (loop_delay / 1000));                                                  // Calculate integral error factor
  integral1 = constrain(integral1, -I_MAX * (loop_delay / 1000), I_MAX * (loop_delay / 1000));  // Prevent windup
  integral2 += (error2 * (loop_delay / 1000));
  integral2 = constrain(integral2, -I_MAX * (loop_delay / 1000), I_MAX * (loop_delay / 1000));

  double voltage1 = min((error1 * KP) + (KI * integral1) - (KDrift * comp2), BAT_VOLTAGE);  // Limit control voltage to battery voltage
  double voltage2 = min((error2 * KP) + (KI * integral2) - (KDrift * comp1), BAT_VOLTAGE);

  // double voltage1 = min((error1 * KP) + (KI * integral1) - (KDrift * comp2), BAT_VOLTAGE);  // Limit control voltage to battery voltage
  // double voltage2 = min((error2 * KP) + (KI * integral2) - (KDrift * comp1), BAT_VOLTAGE);

  // double voltage1 = min((error1 * KP)/REDUCER, BAT_VOLTAGE);
  // double voltage2 = min((error2 * KP)/REDUCER, BAT_VOLTAGE);

  voltage1 = max(-BAT_VOLTAGE, voltage1);
  voltage2 = max(-BAT_VOLTAGE, voltage2);

  if (voltage1 < 0) {  // MOTOR 1 On pololu. Direction control
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }
  if (voltage2 < 0) {
    digitalWrite(8, LOW);
  } else {
    digitalWrite(8, HIGH);
  }

  // Calculate PWM values based
  double pwm1 = min((abs((double)voltage1 / BAT_VOLTAGE) * 255), PWM_MAX);  // Calculate PWM based on voltage input
  double pwm2 = min((abs((double)voltage2 / BAT_VOLTAGE) * 255), PWM_MAX);

  // Serial.println(String(target1) + "," + String(error1) + "," + String(voltage1));

  // Write to pins
  analogWrite(9, pwm1);
  analogWrite(10, pwm2);

  // Record last position
  prevLoopi1 = loopi1;
  prevLoopi2 = loopi2;
}
*/




/*
#define BAT_VOLTAGE 7.8  // 7.8
#define KP 0.02          // 0.05
#define KI 0.05
#define I_MAX 0.5
#define REDUCER 4         // 4
#define PWM_MAX 200       // 200
#define WHEEL_CIRC 0.479  // 0.48 very close
#define COUNTS_PER_REV 3200

double targetPosition = 0.9144;  // Position in meters

// General vars
int numOne = 0;
int numTwo = 0;

int target1 = 0;
int target2 = 0;

double loop_delay = 1500;
long lastTime = 0;

bool first = true;

// Control system vars
long lastMeas1 = 0;
long lastMeas2 = 0;

long lastA1 = 0;
long lastB1 = 0;

long lastA2 = 0;
long lastB2 = 0;

long i1 = 0;
long i2 = 0;

long prevLoopi1 = 0;
long prevLoopi2 = 0;

double integral1 = 0;
double integral2 = 0;

void encoderUpdate1() {  // Covered Motor interrupt callback
  if ((micros() - 300) > lastMeas1) {
    int thisA1 = digitalRead(3);
    int thisB1 = digitalRead(6);

    if ((thisA1 == thisB1)) {
      i1 += 2;  // Forward
    } else {
      i1 -= 2;  // Backwards
    }
    lastA1 = thisA1;
    lastB1 = thisB1;
  }
  lastMeas1 = micros();  // Debounce for encoder
}

void encoderUpdate2() {  // Uncovered Motor interrupt callback
  if ((micros() - 300) > lastMeas2) {
    int thisA2 = digitalRead(2);
    int thisB2 = digitalRead(5);

    if ((thisA2 == thisB2)) {
      i2 -= 2;  // Backwards
    } else {
      i2 += 2;  // Forward
    }
    lastA2 = thisA2;
    lastB2 = thisB2;
  }
  lastMeas2 = micros();  // Debounce for encoder
}

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  pinMode(4, OUTPUT);                                                 // Enable pin
  pinMode(7, OUTPUT);                                                 // Motor 1 Direction
  pinMode(8, OUTPUT);                                                 // Motor 2 Direction
  pinMode(9, OUTPUT);                                                 // Motor 1 Speed Output
  pinMode(10, OUTPUT);                                                // Motor 2 Speed Output
  attachInterrupt(digitalPinToInterrupt(3), encoderUpdate1, CHANGE);  // Interrupt for covered motor
  attachInterrupt(digitalPinToInterrupt(2), encoderUpdate2, CHANGE);  // Interrupt for uncovered motor
  Serial.begin(115200);
  digitalWrite(4, HIGH);
  //delay(3500);
  Serial.println("Ready!");  // Start MATLAB Reading
  //delay(1500);
  lastTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (loop_delay + lastTime > micros()) {
    if (Serial.available()) {  //wait for data available
      String in = Serial.readString();
      Serial.println("Input " + in);
      targetPosition = in.toDouble();
      targetPosition *= 0.3048;
      Serial.println("Going to " + String(targetPosition, 4));
    }
  }

  lastTime = micros();  // Log loop start time for delay system
  long loopi1 = i1;     // Position of motor 1
  long loopi2 = i2;     // Position of motor 2

  target1 = (targetPosition / WHEEL_CIRC) * COUNTS_PER_REV;  // Set target position
  target2 = (targetPosition / WHEEL_CIRC) * COUNTS_PER_REV;

  int error1 = target1 - i1;  // Calculate error
  int error2 = target2 - i2;

  integral1 += error1 * (loop_delay / 1000);                                                    // Calculate integral error factor
  integral1 = constrain(integral1, -I_MAX * (loop_delay / 1000), I_MAX * (loop_delay / 1000));  // Prevent windup
  integral2 += error2 * (loop_delay / 1000);
  integral2 = constrain(integral2, -I_MAX * (loop_delay / 1000), I_MAX * (loop_delay / 1000));

  double voltage1 = min((error1 * KP) + (KI * integral1), BAT_VOLTAGE);  // Limit control voltage to battery voltage
  double voltage2 = min((error2 * KP) + (KI * integral2), BAT_VOLTAGE);

  // double voltage1 = min((error1 * KP)/REDUCER, BAT_VOLTAGE);
  // double voltage2 = min((error2 * KP)/REDUCER, BAT_VOLTAGE);

  voltage1 = max(-BAT_VOLTAGE, voltage1);
  voltage2 = max(-BAT_VOLTAGE, voltage2);

  if (voltage1 < 0) {  // MOTOR 1 On pololu. Direction control
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }
  if (voltage2 < 0) {
    digitalWrite(8, LOW);
  } else {
    digitalWrite(8, HIGH);
  }

  // Calculate PWM values based
  double pwm1 = min((abs((double)voltage1 / BAT_VOLTAGE) * 255), PWM_MAX);  // Calculate PWM based on voltage input
  double pwm2 = min((abs((double)voltage2 / BAT_VOLTAGE) * 255), PWM_MAX);

  // Serial.println(String(target1) + "," + String(error1) + "," + String(voltage1));

  // Write to pins
  analogWrite(9, pwm1);
  analogWrite(10, pwm2);

  // Record last position
  prevLoopi1 = loopi1;
  prevLoopi2 = loopi2;
}
*/
