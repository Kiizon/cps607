// cps607 a3: wall following, wall on the right at 15 cm
// FOLLOW_WALL: sensors -> d, theta -> pi controller -> wheels
// wall ahead -> TURN_LEFT -> LEFT_EXIT -> FOLLOW_WALL
// wall ends -> CORNER_ADVANCE -> TURN_RIGHT -> FOLLOW_WALL
// finish tape (any state) -> FINISH_PASS -> FINISHED

#define MODE 4
// 0 sensors only, 1 straight, 2 follow, 3 + left turns, 4 full course

const int R_FWD = 6, R_REV = 5;   // right wheel
const int L_FWD = 9, L_REV = 10;  // left wheel
enum { BACK, FRONT, AHEAD };      // ping order
const int TRIG[3] = {7, 3, 12};
const int ECHO[3] = {8, 4, 13};
const int LINE_PIN = 2;
const int LINE_ON_TAPE = 1;

const float L_MM = 103;                 // gap between the two side sensors
const float NO_ECHO = 4000;
const float WALL_LOST_MM = 400;         // side reading past this = no wall
const unsigned long PING_GAP_MS = 25;   // one sensor per 25 ms so echoes never overlap
const unsigned long ECHO_TIMEOUT_US = 15000;   // ~2.5 m
const float TURN_DONE_DEG = 25;

// tunables, set over serial as letter + number (p0.5), g prints all
float target_mm = 150;         // w
float Kp_L = 0.4;              // p  pwm per mm of distance error
float Kp_H = 2.5;              // h  pwm per degree of heading error
float Ki = 0.0;                // i
float max_approach_deg = 15;   // a  steepest angle the distance term may ask for
int base = 120;                // s  forward pwm
int max_steer = 30;            // x
int left_trim = 7;             // m  taken off the left wheel to drive straight
int front_stop_mm = 170;       // f  wall ahead closer than this -> TURN_LEFT
int turn_pwm = 150;            // u
int turn_min_ms = 250;         // n
int turn_max_ms = 700;         // t
int ahead_clear_mm = 350;      // e  TURN_LEFT can't end until ahead is clear past this
int overturn_ms = 0;           // o  extra spin after parallel (0 = off)
int left_done_deg = 5;         // d  TURN_LEFT ends within this many degrees of parallel
int exit_straight_ms = 150;    // b  LEFT_EXIT straight time
int lost_cycles = 3;           // l  readings without the front side wall -> CORNER_ADVANCE
int advance_ms = 0;            // v
int arc_left = 200;            // k  TURN_RIGHT outer wheel
int arc_right = 70;            // y  TURN_RIGHT inner wheel, stalls below ~70
int arc_min_ms = 350;          // j
int arc_max_ms = 2500;         // r
int finish_pass_ms = 400;      // z
int use_median = 1;            // q  median of last 3 readings (0 = raw)

float raw[3] = {NO_ECHO, NO_ECHO, NO_ECHO};   // newest reading
float hist[3][3];                             // last 3 readings per sensor
float dist[3] = {NO_ECHO, NO_ECHO, NO_ECHO};  // filtered
int next_sensor = BACK;
unsigned long last_ping_ms = 0;
bool sides_fresh = false;
bool ahead_fresh = false;

enum EchoPhase { IDLE, WAIT_RISE, WAIT_FALL };
EchoPhase echo_phase = IDLE;
unsigned long trig_us = 0, rise_us = 0;

float theta_deg = 0;   // + = nose away from the wall
float d_mm = 0;

float integral_L = 0;
float steer_L = 0, steer_H = 0, steer_I = 0, steer_out = 0;
unsigned long last_ctrl_ms = 0;

enum State { FOLLOW_WALL, TURN_LEFT, LEFT_EXIT, CORNER_ADVANCE, TURN_RIGHT, FINISH_PASS, FINISHED };
State state = FOLLOW_WALL;
unsigned long state_start_ms = 0;
unsigned long overturn_start_ms = 0;
int ahead_close_count = 0;
int front_lost_count = 0;
unsigned long last_print_ms = 0;

char cmd[16];
int cmd_len = 0;


void setup() {
  Serial.begin(115200);
  for (int s = 0; s < 3; s++) {
    pinMode(TRIG[s], OUTPUT);
    pinMode(ECHO[s], INPUT);
    for (int k = 0; k < 3; k++) hist[s][k] = NO_ECHO;
  }
  pinMode(R_FWD, OUTPUT); pinMode(R_REV, OUTPUT);
  pinMode(L_FWD, OUTPUT); pinMode(L_REV, OUTPUT);
  pinMode(LINE_PIN, INPUT);
  stopMotors();

  Serial.print("mode "); Serial.println(MODE);
  Serial.println("tune: w p h i a s x m f u n t e o d b l v k y j r z q, g prints all");
  if (MODE >= 1) delay(3000);   // time to put the robot down
  setState(FOLLOW_WALL);
}

void loop() {
  handleSerial();
  updateSensors();

  if (MODE >= 2 && state != FINISH_PASS && state != FINISHED && onFinishLine()) setState(FINISH_PASS);

  if (sides_fresh) {
    sides_fresh = false;
    if (sidesSeeWall(dist)) estimate(dist, theta_deg, d_mm);

    if (MODE >= 2 && state == FOLLOW_WALL) {
      // outside corner: front side sensor passes the wall end while the back still sees it
      bool front_lost = dist[FRONT] >= WALL_LOST_MM && dist[BACK] < WALL_LOST_MM;
      front_lost_count = front_lost ? front_lost_count + 1 : 0;
      int need_lost = use_median ? max(lost_cycles - 1, 1) : lost_cycles;   // median already needs 2 of 3

      if (MODE == 4 && front_lost_count >= need_lost) {
        setState(CORNER_ADVANCE);
      } else if (sidesSeeWall(dist)) {
        float s = wallController();
        drive(base + s, base - s);   // s > 0 turns right, toward the wall
      } else {
        driveStraight();
      }
    }
  }

  if (ahead_fresh) {
    ahead_fresh = false;
    if (MODE >= 3 && (state == FOLLOW_WALL || state == LEFT_EXIT)) {
      ahead_close_count = (dist[AHEAD] < front_stop_mm) ? ahead_close_count + 1 : 0;
      if (ahead_close_count >= (use_median ? 1 : 2)) setState(TURN_LEFT);
    }
  }

  unsigned long elapsed = millis() - state_start_ms;   // after any state change above
  switch (state) {
    case TURN_LEFT:
      spinLeft(turn_pwm);
      if (overturn_start_ms == 0 && leftTurnDone(elapsed)) overturn_start_ms = millis();
      if (overturn_start_ms != 0 && millis() - overturn_start_ms >= (unsigned long)max(overturn_ms, 0)) {
        stopMotors();
        setState(LEFT_EXIT);
      }
      break;
    case LEFT_EXIT:
      driveStraight();
      if (elapsed >= (unsigned long)exit_straight_ms) setState(FOLLOW_WALL);
      break;
    case CORNER_ADVANCE:
      driveStraight();
      if (elapsed >= (unsigned long)advance_ms) setState(TURN_RIGHT);
      break;
    case TURN_RIGHT:
      drive(arc_left, arc_right);
      // wall back early = a missed echo, not a corner
      if (elapsed < (unsigned long)arc_min_ms && sidesSeeWall(raw)) setState(FOLLOW_WALL);
      else if (rightTurnDone(elapsed)) setState(FOLLOW_WALL);
      break;
    case FINISH_PASS:
      driveStraight();
      if (elapsed >= (unsigned long)finish_pass_ms) setState(FINISHED);
      break;
    case FINISHED:
      stopMotors();
      break;
    default:
      break;
  }

  if (MODE == 1) drive(base, base);

  // print only between pings so serial never delays timing an echo
  if (echo_phase == IDLE && millis() - last_print_ms >= 200) {
    last_print_ms = millis();
    printStatus();
  }
}


// non-blocking ultrasonic: each call fires a ping or checks the echo pin, never waits
void updateSensors() {
  unsigned long now = micros();
  int s = next_sensor;

  if (echo_phase == WAIT_RISE) {
    if (digitalRead(ECHO[s])) { rise_us = now; echo_phase = WAIT_FALL; }
    else if (now - trig_us > ECHO_TIMEOUT_US) finishReading(NO_ECHO);
    return;
  }
  if (echo_phase == WAIT_FALL) {
    if (!digitalRead(ECHO[s])) finishReading((now - rise_us) * 0.343 / 2);   // 0.343 mm/us, there and back
    else if (now - rise_us > ECHO_TIMEOUT_US) finishReading(NO_ECHO);
    return;
  }

  if (millis() - last_ping_ms < PING_GAP_MS) return;
  last_ping_ms = millis();
  digitalWrite(TRIG[s], LOW);  delayMicroseconds(2);   // 12 us trigger, the only wait
  digitalWrite(TRIG[s], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG[s], LOW);
  trig_us = micros();
  echo_phase = WAIT_RISE;
}

void finishReading(float mm) {
  int s = next_sensor;
  echo_phase = IDLE;
  raw[s] = mm;
  hist[s][2] = hist[s][1]; hist[s][1] = hist[s][0]; hist[s][0] = mm;
  dist[s] = use_median ? median3(hist[s][0], hist[s][1], hist[s][2]) : mm;
  if (s == FRONT) sides_fresh = true;   // back was read just before
  if (s == AHEAD) ahead_fresh = true;
  next_sensor = (s + 1) % 3;
}

// one bad reading (spike, dropout, no echo) is thrown out
float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

bool sidesSeeWall(const float *v) {
  return v[BACK] < WALL_LOST_MM && v[FRONT] < WALL_LOST_MM;
}

void estimate(const float *v, float &theta, float &d) {
  theta = degrees(atan2(v[FRONT] - v[BACK], L_MM));
  d = (v[FRONT] + v[BACK]) / 2.0 * cos(radians(theta));
}


// steering pwm, + = turn right (toward the wall)
float wallController() {
  unsigned long now = millis();
  float dt = (now - last_ctrl_ms) / 1000.0;
  last_ctrl_ms = now;

  float e_L = d_mm - target_mm;   // + = too far
  float e_H = theta_deg;          // + = nose away

  // cap so the distance term alone never asks for more than max_approach_deg
  float cap = (Kp_H > 0) ? Kp_H * max_approach_deg : max_steer;
  steer_L = constrain(Kp_L * e_L, -cap, cap);
  steer_H = Kp_H * e_H;   // damping

  // anti-windup: only integrate near the target, cap at 20 pwm
  if (abs(e_L) < 50) integral_L += e_L * dt;
  if (Ki > 0) integral_L = constrain(integral_L, -20 / Ki, 20 / Ki);
  steer_I = Ki * integral_L;

  steer_out = constrain(steer_L + steer_H + steer_I, -max_steer, max_steer);
  return steer_out;
}


// turn ends use raw readings, the median lags while spinning
// theta is negative mid-spin (nose into the new wall) and rises toward 0
bool leftTurnDone(unsigned long elapsed) {
  if (elapsed >= (unsigned long)turn_max_ms) return true;
  if (elapsed < (unsigned long)turn_min_ms) return false;
  if (!sidesSeeWall(raw)) return false;
  float th, d;
  estimate(raw, th, d);
  return th > -left_done_deg && raw[AHEAD] > ahead_clear_mm;
}

bool rightTurnDone(unsigned long elapsed) {
  if (elapsed >= (unsigned long)arc_max_ms) return true;
  if (elapsed < (unsigned long)arc_min_ms) return false;
  if (!sidesSeeWall(raw)) return false;
  float th, d;
  estimate(raw, th, d);
  return abs(th) < TURN_DONE_DEG;
}

// on tape for 30 ms straight, so one bad read can't end the run
unsigned long line_seen_since = 0;
bool onFinishLine() {
  if (digitalRead(LINE_PIN) != LINE_ON_TAPE) { line_seen_since = 0; return false; }
  if (line_seen_since == 0) line_seen_since = millis();
  return millis() - line_seen_since >= 30;
}

void setState(State s) {
  state = s;
  state_start_ms = millis();
  ahead_close_count = 0;
  front_lost_count = 0;
  overturn_start_ms = 0;
  if (s == FOLLOW_WALL) last_ctrl_ms = millis();   // no huge dt after a turn
  Serial.print("-> ");
  Serial.println(stateName(s));
}

const char* stateName(State s) {
  switch (s) {
    case FOLLOW_WALL:    return "FOLLOW_WALL";
    case TURN_LEFT:      return "TURN_LEFT";
    case LEFT_EXIT:      return "LEFT_EXIT";
    case CORNER_ADVANCE: return "CORNER_ADVANCE";
    case TURN_RIGHT:     return "TURN_RIGHT";
    case FINISH_PASS:    return "FINISH_PASS";
    case FINISHED:       return "FINISHED";
  }
  return "?";
}


void drive(int left, int right) {
  analogWrite(L_FWD, constrain(left - left_trim, 0, 255)); analogWrite(L_REV, 0);
  analogWrite(R_FWD, constrain(right, 0, 255));            analogWrite(R_REV, 0);
}

void driveStraight() {   // keeps the integral's motor correction
  drive(base + steer_I, base - steer_I);
}

void spinLeft(int pwm) {
  analogWrite(R_FWD, pwm); analogWrite(R_REV, 0);
  analogWrite(L_FWD, 0);   analogWrite(L_REV, pwm);
}

void stopMotors() {
  analogWrite(R_FWD, 0); analogWrite(R_REV, 0);
  analogWrite(L_FWD, 0); analogWrite(L_REV, 0);
}


// name:value pairs for the serial plotter
void printStatus() {
  Serial.print("state:"); Serial.print((int)state);
  Serial.print(" ahead:"); Serial.print(dist[AHEAD], 0);
  Serial.print(" front:"); Serial.print(dist[FRONT], 0);
  Serial.print(" back:"); Serial.print(dist[BACK], 0);
  Serial.print(" d:"); Serial.print(d_mm, 0);
  Serial.print(" theta:"); Serial.print(theta_deg, 1);
  Serial.print(" sL:"); Serial.print(steer_L, 1);
  Serial.print(" sH:"); Serial.print(steer_H, 1);
  Serial.print(" sI:"); Serial.print(steer_I, 1);
  Serial.print(" steer:"); Serial.print(steer_out, 1);
  Serial.print(" line:"); Serial.println(digitalRead(LINE_PIN));
}

void printSettings() {
  Serial.print("w="); Serial.print(target_mm);  Serial.print(" p="); Serial.print(Kp_L);
  Serial.print(" h="); Serial.print(Kp_H);      Serial.print(" i="); Serial.print(Ki);
  Serial.print(" a="); Serial.print(max_approach_deg);
  Serial.print(" s="); Serial.print(base);      Serial.print(" x="); Serial.print(max_steer);
  Serial.print(" m="); Serial.print(left_trim); Serial.print(" f="); Serial.print(front_stop_mm);
  Serial.print(" u="); Serial.print(turn_pwm);  Serial.print(" n="); Serial.print(turn_min_ms);
  Serial.print(" t="); Serial.print(turn_max_ms); Serial.print(" e="); Serial.print(ahead_clear_mm);
  Serial.print(" o="); Serial.print(overturn_ms); Serial.print(" d="); Serial.print(left_done_deg);
  Serial.print(" b="); Serial.print(exit_straight_ms);
  Serial.print(" l="); Serial.print(lost_cycles); Serial.print(" v="); Serial.print(advance_ms);
  Serial.print(" k="); Serial.print(arc_left); Serial.print(" y="); Serial.print(arc_right);
  Serial.print(" j="); Serial.print(arc_min_ms); Serial.print(" r="); Serial.print(arc_max_ms);
  Serial.print(" z="); Serial.print(finish_pass_ms); Serial.print(" q="); Serial.println(use_median);
}

// non-blocking: collects characters until newline, then applies the command
void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') {
      if (cmd_len < (int)sizeof(cmd) - 1) cmd[cmd_len++] = ch;
      continue;
    }
    if (cmd_len == 0) continue;
    cmd[cmd_len] = '\0';
    cmd_len = 0;
    applyCommand(cmd[0], atof(cmd + 1));
  }
}

void applyCommand(char c, float v) {
  switch (c) {
    case 'g': printSettings(); return;
    case 'w': target_mm = v; break;
    case 'p': Kp_L = v; break;
    case 'h': Kp_H = v; break;
    case 'i': Ki = v; break;
    case 'a': max_approach_deg = v; break;
    case 's': base = (int)v; break;
    case 'x': max_steer = (int)v; break;
    case 'm': left_trim = (int)v; break;
    case 'f': front_stop_mm = (int)v; break;
    case 'u': turn_pwm = (int)v; break;
    case 'n': turn_min_ms = (int)v; break;
    case 't': turn_max_ms = (int)v; break;
    case 'e': ahead_clear_mm = (int)v; break;
    case 'o': overturn_ms = max((int)v, 0); break;
    case 'd': left_done_deg = (int)v; break;
    case 'b': exit_straight_ms = (int)v; break;
    case 'l': lost_cycles = (int)v; break;
    case 'v': advance_ms = (int)v; break;
    case 'k': arc_left = (int)v; break;
    case 'y': arc_right = (int)v; break;
    case 'j': arc_min_ms = (int)v; break;
    case 'r': arc_max_ms = (int)v; break;
    case 'z': finish_pass_ms = (int)v; break;
    case 'q': use_median = (int)v; break;
    default: Serial.println("unknown, use w p h i a s x m f u n t e o d b l v k y j r z q g"); return;
  }
  integral_L = 0;   // restart the integral after any change
  printSettings();
}
