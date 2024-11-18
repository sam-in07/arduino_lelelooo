// Motor Pins
#define rmf 7  // Right Motor Forward
#define rmb 4  // Right Motor Backward
#define lmf 3  // Left Motor Forward
#define lmb 2  // Left Motor Backward
#define rms 6  // Right Motor Speed (PWM)
#define lms 5  // Left Motor Speed (PWM)

// Button and LED Pins
#define button1 8
#define button2 9
#define button3 10
#define button4 11
#define led12 12
#define led 13

// Sensors
int sensor[6];
int threshold = 512;

// PID Variables
float kp = 50, kd = 1200, ki = 5;
float current_error, previous_error, integral;
int PID_value;
float c;

// Speed Settings
int base_speed = 200;
int max_speed = 255;
int turn_speed = 180;

// Turn Control
char t;

// Button States
bool button1_state, button2_state, button3_state, button4_state;

// Function Declarations
void Line_Follow();
void sensor_reading();
void motor(int left_speed, int right_speed);
void right();
void left();
void U_turn();
void button_status();

void setup() {
  // Set motor pins as output
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);

  // Set button pins as input
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);

  // Set LED pins as output
  pinMode(led12, OUTPUT);
  pinMode(led, OUTPUT);

  // Initialize Serial Monitor
  Serial.begin(9600);
}

void loop() {
  button_status();

  // Trigger actions based on button states
  if (!button1_state) {
    digitalWrite(led12, HIGH);  // Indicate active state
    Line_Follow();
  } else if (!button2_state) {
    motor(0, 0);  // Pause
    digitalWrite(led12, LOW);
  } else if (!button3_state) {
    // Perform reset or calibration
    Serial.println("Resetting robot...");
    motor(0, 0);
    integral = 0;
    previous_error = 0;
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
  } else if (!button4_state) {
    // Additional functionality (e.g., change PID mode or speed)
    base_speed += 50;
    if (base_speed > max_speed) base_speed = 100;  // Reset speed if exceeding limit
    Serial.println("Increased base speed!");
    delay(300);  // Debounce delay
  }
}

void button_status() {
  button1_state = digitalRead(button1);
  button2_state = digitalRead(button2);
  button3_state = digitalRead(button3);
  button4_state = digitalRead(button4);
}

void sensor_reading() {
  float weighted_sum = 0;
  float total = 0;

  for (int i = 0; i < 6; i++) {
    if (i < 4) {
      sensor[i] = analogRead(i);  // A0 to A3
    } else {
      sensor[i] = analogRead(i + 2);  // A4, A5
    }

    sensor[i] = (sensor[i] > threshold) ? 1 : 0;
    weighted_sum += sensor[i] * (i + 1);
    total += sensor[i];
  }

  c = (total > 0) ? (weighted_sum / total) : 3.5;
}

void Line_Follow() {
  while (1) {
    sensor_reading();

    current_error = 3.5 - c;
    integral += current_error;
    PID_value = kp * current_error + kd * (current_error - previous_error) + ki * integral;
    previous_error = current_error;

    int left_motor = base_speed + PID_value;
    int right_motor = base_speed - PID_value;

    left_motor = constrain(left_motor, 0, max_speed);
    right_motor = constrain(right_motor, 0, max_speed);

    motor(left_motor, right_motor);

    if (sensor[0] == 1 && sensor[5] == 0) t = 'r';
    if (sensor[0] == 0 && sensor[5] == 1) t = 'l';

    if (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] == 0) {
      if (t == 'r') right();
      else if (t == 'l') left();
      else U_turn();
    }

    if (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] == 6) {
      motor(0, 0);
      delay(500);
    }
  }
}

void motor(int left_speed, int right_speed) {
  if (left_speed >= 0) {
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  } else {
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
    left_speed = -left_speed;
  }

  if (right_speed >= 0) {
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
  } else {
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
    right_speed = -right_speed;
  }

  analogWrite(lms, constrain(left_speed, 0, max_speed));
  analogWrite(rms, constrain(right_speed, 0, max_speed));
}

void right() {
  motor(turn_speed, -turn_speed);
  while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
  motor(base_speed, base_speed);
}

void left() {
  motor(-turn_speed, turn_speed);
  while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
  motor(base_speed, base_speed);
}

void U_turn() {
  motor(-turn_speed, turn_speed);
  delay(600);
  while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
  motor(base_speed, base_speed);
}
