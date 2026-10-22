const int A_pinF = 6, A_pinR = 5;    // motor a = right wheel = phi_1
const int B_pinF = 9, B_pinR = 10;   // motor a = left wheel = phi_2

const float R_WHEEL = 0.03275;       // wheel radius r [m]
const float B_HALF  = 0.06475;       // half wheel separation b [m]

// (2.2): PWM calibration --> phi_dot = K * PWM + C [rad/s]
const float K = 0.03698;
const float C = 0.4617;

// path commands
const float SIDE_LENGTH  = 1.0;       // [m]
const int   PWM_STRAIGHT = 160;
const int   PWM_TURN     = 160;
const float TRIM_RATIO   = 0.84;      // left (B) = right (A) * ratio, drives straight
const int   TRIM_TURN    = 5;         // left wheel reduction during turns

const float STRAIGHT_SCALE = .64;     // adjust based on how far robot goes
const float TURN_SCALE     = .9;      // adjust based on how much robot turns

const unsigned long PAUSE_MS = 300;   // settle time between moves
unsigned long tStraight, tTurn;       // computed durations [ms]

float wheelSpeed(int pwm) {           // PWM -> wheel angular velocity [rad/s]
  return K * pwm + C;
}

void setup() {
  pinMode(A_pinF, OUTPUT); pinMode(A_pinR, OUTPUT);
  pinMode(B_pinF, OUTPUT); pinMode(B_pinR, OUTPUT);
  stopMove();
  Serial.begin(9600);

  // straight: v = r * phi_dot,  t = d / v
  float v = R_WHEEL * wheelSpeed(PWM_STRAIGHT);
  tStraight = (unsigned long)(1000.0 * STRAIGHT_SCALE * SIDE_LENGTH / v);

  // turn on the spot: omega = r * phi_dot / b,  t = (pi/2) / omega
  float omega = R_WHEEL * wheelSpeed(PWM_TURN) / B_HALF;
  tTurn = (unsigned long)(1000.0 * TURN_SCALE * (PI / 2.0) / omega);

  Serial.print("Straight time (ms): "); Serial.println(tStraight);
  Serial.print("Turn time (ms): ");     Serial.println(tTurn);

  delay(5000);                   
}

void loop() {
  for (int i = 0; i < 4; i++) {
    nudge();                          // castor in line before every straight
    straight();    delay(tStraight);
    stopMove();    delay(PAUSE_MS);
    turn90Right(); delay(tTurn);
    stopMove();    delay(PAUSE_MS);
  }
  delay(1000);
}


const int NUDGE_PWM = 100;
const unsigned long NUDGE_MS = 300;

void nudge() {                         // slow forward so the castor trails straight
  analogWrite(A_pinF, NUDGE_PWM);
  analogWrite(A_pinR, 0);
  analogWrite(B_pinF, (int)(NUDGE_PWM * TRIM_RATIO));
  analogWrite(B_pinR, 0);
  delay(NUDGE_MS);
  stopMove();
  delay(100);
}

void straight() {
  for (int p = 80; p <= PWM_STRAIGHT; p += 10) {   // ramp 80 -> 160
    analogWrite(A_pinF, p);
    analogWrite(A_pinR, 0);
    analogWrite(B_pinF, (int)(p * TRIM_RATIO));
    analogWrite(B_pinR, 0);
    delay(45);                                       // ~0.4 s total
  }
}

void turn90Right() {                 // right wheel backward, left wheel forward
  analogWrite(A_pinF, 0);
  analogWrite(A_pinR, PWM_TURN);
  analogWrite(B_pinF, PWM_TURN - TRIM_TURN);
  analogWrite(B_pinR, 0);
}

void stopMove() {
  analogWrite(A_pinF, 0); analogWrite(A_pinR, 0);
  analogWrite(B_pinF, 0); analogWrite(B_pinR, 0);
}