// CPS607 A3 - wall following, wall on the RIGHT at 15 cm.
// One sketch, built up in MODEs. Set MODE, upload, tune until the step is "ready", then go up one MODE.
// The checklist for each step is in STEPS.md next to this file.
//
// How it fits together:
//   updateSensors() -> updateWallEstimate() -> wallController() -> drive()           (FOLLOW_WALL)
//   wall ahead          -> TURN_LEFT  (spin until the new wall is beside us)  -> FOLLOW_WALL
//   front side loses wall -> CORNER_ADVANCE (short straight) -> TURN_RIGHT (curve until new wall seen) -> FOLLOW_WALL

#define MODE 4
// 0 = SENSORS : motors off. Print readings, d and theta.
// 1 = STRAIGHT: drive at base speed, no steering. Tune left_trim.
// 2 = FOLLOW  : wall following only, never turns. Tune the gains.
// 3 = LEFT    : + left turn when a wall is ahead (inside corners and the steep diagonal).
// 4 = FULL    : + right turn when the wall ends (outside corner). The whole course.

// ---------- PINS ----------
const int R_FWD = 6, R_REV = 5;   // right wheel (motor A)
const int L_FWD = 9, L_REV = 10;  // left wheel (motor B)
enum { BACK, FRONT, AHEAD };      // sensor index. Pinged in this order.
const int TRIG[3] = {7, 3, 12};
const int ECHO[3] = {8, 4, 13};
const int LINE_PIN = 2;           // line sensor under the robot, pointing at the floor
const int LINE_ON_TAPE = 1;       // what it reads on the black finish tape (check "line:" in MODE 0)

// ---------- FIXED NUMBERS ----------
const float L_MM = 103;               // distance between the two side sensors (mm)
const float NO_ECHO = 4000;           // what readSensor() returns when nothing echoes (mm)
const float WALL_LOST_MM = 400;       // a side reading beyond this = no wall there
const unsigned long PING_GAP_MS = 25; // one sensor every 25 ms, so they never hear each other's echo
const float TURN_DONE_DEG = 25;       // a turn may end once the wall beside us is within this angle

// ---------- TUNABLES (change live over Serial: letter + number, e.g. "p0.5". "g" prints them all) ----------
float target_mm = 150;         // w  desired distance to the wall
float Kp_L = 0.4;              // p  steering PWM per mm of distance error
float Kp_H = 2.5;              // h  steering PWM per degree of heading error
float Ki = 0.0;                // i  integral gain. Leave 0 until P works.
float max_approach_deg = 15;   // a  distance error may never ask for a steeper angle than this
int base = 120;               // s  forward PWM
int max_steer = 30;            // x  keep base - max_steer above the PWM where a wheel stalls
int left_trim = 7;             // m  PWM taken off the left wheel so the robot drives straight
int front_stop_mm = 170;       // f  wall ahead closer than this (2 readings in a row) -> turn left
int turn_pwm = 150;            // u  spin power (both turns)
int turn_min_ms = 250;         // n  left turn: always spin at least this long
int turn_max_ms = 700;         // t  left turn: never spin longer than this. Set n = t for a purely timed turn.
int ahead_clear_mm = 350;      // e  left turn only ends once nothing is closer than this ahead
int overturn_ms = 100;         // o  left turn: keep spinning this long AFTER the new wall is beside us (0 = off)
int exit_straight_ms = 150;    // b  then drive straight this long, holding the overturned angle, before following
int lost_cycles = 3;           // l  front side sensor must lose the wall this many readings in a row
int advance_ms = 0;          // v  outside corner: drive straight this long before curving (wheels reach the corner)
int arc_left = 200;            // k  outside corner curve: LEFT (outer) wheel PWM. Higher = tighter curve
int arc_right = 70;            // y  outside corner curve: RIGHT (inner) wheel PWM. Keep at or above ~70 so it does not stall
int arc_min_ms = 350;          // j  curve at least this long (so the old wall / corner tip can't end it)
int arc_max_ms = 2500;         // r  give up curving after this, back to following
int finish_pass_ms = 400;      // z  after the finish tape, keep driving this long so the whole robot crosses, then stop

// ---------- STATE ----------
float dist[3] = {NO_ECHO, NO_ECHO, NO_ECHO};   // latest reading per sensor (mm)
int next_sensor = BACK;
unsigned long last_ping_ms = 0;
bool sides_fresh = false;   // back + front both have a new reading
bool ahead_fresh = false;

float theta_deg = 0;   // heading to the wall. + = nose pointing AWAY from the wall
float d_mm = 0;        // perpendicular distance to the wall

float integral_L = 0;  // integral of the distance error (mm * s)
float steer_L = 0, steer_H = 0, steer_I = 0, steer_out = 0;   // parts of the steering command
unsigned long last_ctrl_ms = 0;

enum State { FOLLOW_WALL, TURN_LEFT, LEFT_EXIT, CORNER_ADVANCE, TURN_RIGHT, FINISH_PASS, FINISHED };
State state = FOLLOW_WALL;
unsigned long state_start_ms = 0;
unsigned long overturn_start_ms = 0;   // 0 = left turn hasn't reached the new wall yet
int ahead_close_count = 0;
int front_lost_count = 0;
unsigned long last_print_ms = 0;


void setup() {
  Serial.begin(115200);   // fast, so printing never stalls the control loop. Set the monitor to 115200.
  Serial.setTimeout(50);
  for (int s = 0; s < 3; s++) {
    pinMode(TRIG[s], OUTPUT);
    pinMode(ECHO[s], INPUT);
  }
  pinMode(R_FWD, OUTPUT); pinMode(R_REV, OUTPUT);
  pinMode(L_FWD, OUTPUT); pinMode(L_REV, OUTPUT);
  pinMode(LINE_PIN, INPUT);
  stopMotors();

  Serial.print("MODE "); Serial.println(MODE);
  Serial.println("Tune: w p h i a s x m f u n t e o b l v k y j r z, g = print all (e.g. p0.5)");
  if (MODE >= 1) delay(3000);   // time to put the robot down
  setState(FOLLOW_WALL);
}

void loop() {
  handleSerial();
  updateSensors();

  // ---- Finish line: checked every loop in every state, so it reacts within a few ms ----
  if (MODE >= 2 && state != FINISH_PASS && state != FINISHED && onFinishLine()) setState(FINISH_PASS);

  // ---- New side readings (~every 75 ms) ----
  if (sides_fresh) {
    sides_fresh = false;
    if (sidesSeeWall()) updateWallEstimate();

    if (MODE >= 2 && state == FOLLOW_WALL) {
      // Outside corner: the FRONT side sensor passes the end of the wall while the BACK one still sees it
      bool front_lost = dist[FRONT] >= WALL_LOST_MM && dist[BACK] < WALL_LOST_MM;
      front_lost_count = front_lost ? front_lost_count + 1 : 0;

      if (MODE == 4 && front_lost_count >= lost_cycles) {
        setState(CORNER_ADVANCE);
      } else if (sidesSeeWall()) {
        float s = wallController();
        drive(base + s, base - s);   // s > 0: left wheel faster -> turn RIGHT (toward the wall)
      } else {
        driveStraight();             // a side sensor can't see the wall: keep going straight
      }
    }
  }

  // ---- New ahead reading: wall in front for 2 readings in a row -> turn left ----
  if (ahead_fresh) {
    ahead_fresh = false;
    if (MODE >= 3 && (state == FOLLOW_WALL || state == LEFT_EXIT)) {
      ahead_close_count = (dist[AHEAD] < front_stop_mm) ? ahead_close_count + 1 : 0;
      if (ahead_close_count >= 2) setState(TURN_LEFT);
    }
  }

  // ---- States that run on time ----
  // Measured HERE, after any state change above. (It used to be measured at the top of loop(), so a turn
  // that had just started saw the old state's time and could end instantly, delaying turns by a reading.)
  unsigned long elapsed = millis() - state_start_ms;
  switch (state) {
    case TURN_LEFT:
      spinLeft(turn_pwm);
      // Once the new wall is beside us, keep spinning overturn_ms MORE on purpose, so the robot leaves
      // the corner angled away from the wall, already pointing along the slant that comes right after it.
      if (overturn_start_ms == 0 && leftTurnDone(elapsed)) overturn_start_ms = millis();
      if (overturn_start_ms != 0 && millis() - overturn_start_ms >= (unsigned long)overturn_ms) {
        stopMotors();
        setState(LEFT_EXIT);
      }
      break;
    case LEFT_EXIT:
      // Hold the overturned angle for a moment. Otherwise the controller sees "angled away from the
      // wall" and steers straight back to parallel, undoing the overturn.
      driveStraight();
      if (elapsed >= (unsigned long)exit_straight_ms) setState(FOLLOW_WALL);
      break;
    case CORNER_ADVANCE:
      driveStraight();
      if (elapsed >= (unsigned long)advance_ms) setState(TURN_RIGHT);
      break;
    case TURN_RIGHT:
      drive(arc_left, arc_right);   // forward curve to the right, around the corner
      // Wall back beside us EARLY in the curve -> it was only a gap or seam in the wall, not its end.
      // Stop curving and follow it. (At a real corner the front sensor sees nothing early in the curve.)
      if (elapsed < (unsigned long)arc_min_ms && sidesSeeWall()) setState(FOLLOW_WALL);
      else if (rightTurnDone(elapsed)) setState(FOLLOW_WALL);
      break;
    case FINISH_PASS:
      driveStraight();              // keep going so the whole robot crosses the line
      if (elapsed >= (unsigned long)finish_pass_ms) setState(FINISHED);
      break;
    case FINISHED:
      stopMotors();                 // done for good. Press reset to run again.
      break;
    default:
      break;
  }

  if (MODE == 1) drive(base, base);

  if (millis() - last_print_ms >= 200) {
    last_print_ms = millis();
    printStatus();
  }
}


// ---------- SENSORS ----------

// Non-blocking: returns at once unless PING_GAP_MS has passed, then pings ONE sensor.
void updateSensors() {
  if (millis() - last_ping_ms < PING_GAP_MS) return;
  last_ping_ms = millis();

  dist[next_sensor] = readSensor(next_sensor);
  if (next_sensor == FRONT) sides_fresh = true;   // BACK was read just before, so both are new
  if (next_sensor == AHEAD) ahead_fresh = true;
  next_sensor = (next_sensor + 1) % 3;
}

// Distance in mm, or NO_ECHO. pulseIn waits at most 15 ms (about 2.5 m).
float readSensor(int s) {
  digitalWrite(TRIG[s], LOW);  delayMicroseconds(2);
  digitalWrite(TRIG[s], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG[s], LOW);
  unsigned long us = pulseIn(ECHO[s], HIGH, 15000);
  if (us == 0) return NO_ECHO;
  return us * 0.343 / 2;   // sound: 0.343 mm per microsecond, there and back
}

bool sidesSeeWall() {
  return dist[BACK] < WALL_LOST_MM && dist[FRONT] < WALL_LOST_MM;
}

// Figure 2 of the assignment
void updateWallEstimate() {
  float dfr = dist[FRONT], dbr = dist[BACK];
  theta_deg = degrees(atan2(dfr - dbr, L_MM));        // front further than back -> nose points away -> +
  d_mm = (dfr + dbr) / 2.0 * cos(radians(theta_deg)); // slanted reading -> perpendicular distance
}


// ---------- CONTROLLER ----------

// Returns steering PWM. + = turn right (toward the wall).
float wallController() {
  unsigned long now = millis();
  float dt = (now - last_ctrl_ms) / 1000.0;
  last_ctrl_ms = now;

  float e_L = d_mm - target_mm;   // + = too far from the wall
  float e_H = theta_deg;          // + = nose pointing away from the wall (desired heading is 0)

  // Distance part, capped: a big distance error can only ask for a max_approach_deg approach.
  // At that angle steer_H cancels it, so the robot drives in at a fixed angle instead of turning
  // so far that the side sensors lose the wall.
  // (With Kp_H = 0 there is nothing to cancel against, so no cap: max_steer still limits it.)
  float cap = (Kp_H > 0) ? Kp_H * max_approach_deg : max_steer;
  steer_L = constrain(Kp_L * e_L, -cap, cap);

  // Heading part: the damping. It turns the robot parallel before it overshoots the target line.
  steer_H = Kp_H * e_H;

  // Integral part: removes the small leftover offset caused by unequal motors.
  // Only builds when close to the target (so big swings can't wind it up), and is capped at 20 PWM.
  if (abs(e_L) < 50) integral_L += e_L * dt;
  if (Ki > 0) integral_L = constrain(integral_L, -20 / Ki, 20 / Ki);
  steer_I = Ki * integral_L;

  steer_out = constrain(steer_L + steer_H + steer_I, -max_steer, max_steer);
  return steer_out;
}


// ---------- STATE MACHINE ----------

// Left turn ends when: spun at least turn_min_ms AND both side sensors see a wall that is nearly
// parallel AND nothing is close ahead. Or at turn_max_ms, whatever happens.
// Ending on sensors (not a fixed time) lets the same turn handle the ~100 deg corner and the ~40 deg diagonal.
bool leftTurnDone(unsigned long elapsed) {
  if (elapsed >= (unsigned long)turn_max_ms) return true;
  if (elapsed < (unsigned long)turn_min_ms) return false;
  return sidesSeeWall() && abs(theta_deg) < TURN_DONE_DEG && dist[AHEAD] > ahead_clear_mm;
}

// True once the line sensor has seen the tape for 30 ms in a row, so one bad read can't end the run
unsigned long line_seen_since = 0;   // 0 = not on tape right now
bool onFinishLine() {
  if (digitalRead(LINE_PIN) != LINE_ON_TAPE) { line_seen_since = 0; return false; }
  if (line_seen_since == 0) line_seen_since = millis();
  return millis() - line_seen_since >= 30;
}

// Right (outside corner) curve ends when: curved at least arc_min_ms AND both side sensors see the
// new wall nearly parallel. Or at arc_max_ms, whatever happens.
bool rightTurnDone(unsigned long elapsed) {
  if (elapsed >= (unsigned long)arc_max_ms) return true;
  if (elapsed < (unsigned long)arc_min_ms) return false;
  return sidesSeeWall() && abs(theta_deg) < TURN_DONE_DEG;
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


// ---------- MOTORS ----------

void drive(int left, int right) {   // both wheels forward
  analogWrite(L_FWD, constrain(left - left_trim, 0, 255)); analogWrite(L_REV, 0);
  analogWrite(R_FWD, constrain(right, 0, 255));            analogWrite(R_REV, 0);
}

// Straight, but keeping the integral's correction for unequal motors
void driveStraight() {
  drive(base + steer_I, base - steer_I);
}

void spinLeft(int pwm) {            // right wheel forward, left wheel backward
  analogWrite(R_FWD, pwm); analogWrite(R_REV, 0);
  analogWrite(L_FWD, 0);   analogWrite(L_REV, pwm);
}

void spinRight(int pwm) {           // left wheel forward, right wheel backward
  analogWrite(L_FWD, pwm); analogWrite(L_REV, 0);
  analogWrite(R_FWD, 0);   analogWrite(R_REV, pwm);
}

void stopMotors() {
  analogWrite(R_FWD, 0); analogWrite(R_REV, 0);
  analogWrite(L_FWD, 0); analogWrite(L_REV, 0);
}


// ---------- PRINTING AND TUNING ----------

// "name:value" pairs, so Tools > Serial Plotter can graph them. state: 0 follow, 1 left, 2 advance, 3 right
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

// Prints every tunable in the same form as the code, so you can paste the tuned values back in
void printSettings() {
  Serial.print("w="); Serial.print(target_mm);  Serial.print(" p="); Serial.print(Kp_L);
  Serial.print(" h="); Serial.print(Kp_H);      Serial.print(" i="); Serial.print(Ki);
  Serial.print(" a="); Serial.print(max_approach_deg);
  Serial.print(" s="); Serial.print(base);      Serial.print(" x="); Serial.print(max_steer);
  Serial.print(" m="); Serial.print(left_trim); Serial.print(" f="); Serial.print(front_stop_mm);
  Serial.print(" u="); Serial.print(turn_pwm);  Serial.print(" n="); Serial.print(turn_min_ms);
  Serial.print(" t="); Serial.print(turn_max_ms); Serial.print(" e="); Serial.print(ahead_clear_mm);
  Serial.print(" l="); Serial.print(lost_cycles); Serial.print(" v="); Serial.print(advance_ms);
  Serial.print(" k="); Serial.print(arc_left); Serial.print(" y="); Serial.print(arc_right); Serial.print(" j="); Serial.print(arc_min_ms);
  Serial.print(" r="); Serial.print(arc_max_ms); Serial.print(" z="); Serial.print(finish_pass_ms);
  Serial.print(" o="); Serial.print(overturn_ms); Serial.print(" b="); Serial.println(exit_straight_ms);
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;
  if (c == 'g') { while (Serial.available()) Serial.read(); printSettings(); return; }
  float v = Serial.parseFloat();
  while (Serial.available()) Serial.read();

  switch (c) {
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
    case 'l': lost_cycles = (int)v; break;
    case 'v': advance_ms = (int)v; break;
    case 'k': arc_left = (int)v; break;
    case 'y': arc_right = (int)v; break;
    case 'j': arc_min_ms = (int)v; break;
    case 'r': arc_max_ms = (int)v; break;
    case 'z': finish_pass_ms = (int)v; break;
    case 'o': overturn_ms = (int)v; break;
    case 'b': exit_straight_ms = (int)v; break;
    default: Serial.println("Unknown. Use w p h i a s x m f u n t e o b l v k y j r z g"); return;
  }
  integral_L = 0;   // restart the integral after any change
  printSettings();
}
