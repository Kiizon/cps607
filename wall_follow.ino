// cps607 a4: the kidnapped robot, built on the a3 wall follower
//
// follow: the a3 controller keeps a wall on the right and turns left at every corner, so the
// robot goes counter-clockwise around the arena. with no wall in sight it drives straight
// until it reaches one.
//
// localize: at every stop the robot faces a wall and reads its colour with the two forward IR
// sensors (white reflects IR, black doesn't). counter-clockwise the walls go W B B W, so the
// colours of two neighbouring walls name them: (W,B) bottom then right, (B,B) right then top,
// (B,W) top then left, (W,W) left then bottom.
// after the first stop it only has one colour, so it turns right to face the clockwise
// neighbour, creeps in if that wall is too far to read, and reads it too. then it turns
// left 180 so the first wall is on its right, and follows it.
//
// navigate: once localized, each left turn moves one wall on. BR and TR are corners, so the
// robot stops there. L is half way down the left wall, reached by driving l_time_ms after
// the top-left turn.
//
// FOLLOW_WALL -> black wall close on the ahead sensor, or IR sees white -> CORNER (read colour)
// CORNER      -> first colour -> TURN_RIGHT -> CHECK_RIGHT (creep if needed, read) -> TURN_LEFT 180
// CORNER      -> at BR or TR, not visited yet -> AT_WAYPOINT -> TURN_LEFT
// CORNER      -> anywhere else                -> TURN_LEFT -> LEFT_EXIT -> FOLLOW_WALL
// FOLLOW_WALL -> l_time_ms after the top-left turn -> AT_WAYPOINT -> FOLLOW_WALL
// AT_WAYPOINT -> 3rd waypoint done -> DONE

#define MODE 4
// 0 sensors only, 1 straight, 2 follow, 3 + corners and localizing, 4 full run with waypoints


// PINS: same as A3, plus the two forward IR sensors
const int R_FWD = 6, R_REV = 5;     // right wheel
const int L_FWD = 9, L_REV = 10;    // left wheel
enum { BACK, FRONT, AHEAD };        // ping order
const int TRIG[3] = {7, 3, 12};
const int ECHO[3] = {8, 4, 13};
const int IR_RIGHT = 0, IR_LEFT = 1;   // forward IR sensors (DO pins), free on the R4 Minima since USB serial doesn't use D0/D1
const int IR_WHITE = LOW;              // most IR modules pull DO low when they see reflected IR (white), flip if not


// MAP: wall colours, the corners the robot stops at, and the waypoints
// origin bottom-left, x right, y up, heading counter-clockwise from +x (0 = facing the right wall)
const float ARENA_MM = 1500;
enum Colour { WHITE, BLACK };
enum { W_BOTTOM, W_RIGHT, W_TOP, W_LEFT };   // counter-clockwise order
const Colour WALL_COLOUR[4] = {WHITE, BLACK, BLACK, WHITE};
const char* const WALL_NAME[4] = {"bottom", "right", "top", "left"};

// indexed by the wall the robot faces when it stops at a corner (on the 250 mm ring)
const char* const CORNER_NAME[4] = {"bottom-left", "bottom-right", "top-right", "top-left"};
const float CORNER_X[4] = {250, 1250, 1250, 250};
const float CORNER_Y[4] = {250, 250, 1250, 1250};
const float FACING_DEG[4] = {-90, 0, 90, 180};

enum { WP_BR, WP_TR, WP_L };
const char* const WP_NAME[3] = {"BR", "TR", "L"};
const int NUM_WP = 3;


// CONSTANTS: fixed limits, same as A3
const float L_MM = 103;                 // gap between the two side sensors
const float NO_ECHO = 4000;
const float WALL_LOST_MM = 400;         // side reading past this = no wall
const float IR_GATE_MM = 400;           // only trust the IR when the ahead sensor also sees a wall this close
const unsigned long PING_GAP_MS = 25;   // one sensor per 25 ms so echoes never overlap
const unsigned long ECHO_TIMEOUT_US = 15000;   // ~2.5 m
const unsigned long SETTLE_MS = 300;    // stopped this long after a turn so the ahead median is fresh
const unsigned long CHECK_TIMEOUT_MS = 8000;   // CHECK_RIGHT gives up creeping after this
const int SURE_READS = 15;              // of 20 IR reads, this many must agree for a colour to count


// TUNABLES: set over serial as letter + number (p0.5), g prints all
float target_mm = 190;         // w  side sensors to the wall. ~190 + the sensor offset puts the centre on the 250 mm waypoint ring
float Kp_L = 0.4;              // p  pwm per mm of distance error
float Kp_H = 2.5;              // h  pwm per degree of heading error
float Ki = 0.0;                // i
float max_approach_deg = 15;   // a  steepest angle the distance term may ask for
int base = 120;                // s  forward pwm
int max_steer = 30;            // x
int left_trim = 7;             // m  taken off the left wheel to drive straight
int front_stop_mm = 170;       // f  ahead sensor stop for black walls. keep it under the IR range so white walls stop on the IR
int turn_pwm = 150;            // u
int turn_min_ms = 250;         // n
int turn_max_ms = 700;         // t
int ahead_clear_mm = 350;      // e  TURN_LEFT can't end until ahead is clear past this
int overturn_ms = 0;           // o  extra spin after parallel (0 = off)
int left_done_deg = 5;         // d  TURN_LEFT ends within this many degrees of parallel
int exit_straight_ms = 150;    // b  LEFT_EXIT straight time
int right_turn_ms = 450;       // r  time for a 90 deg right turn at turn_pwm, tune
int hold_ms = 3200;            // z  stop at each waypoint (3 s required + margin)
int l_time_ms = 2500;          // l  drive time from the top-left turn to waypoint L (500 mm), tune
float mm_per_s = 200;          // y  forward speed at `base`, for the pose estimate, tune
float side_off_mm = 60;        // c  robot centre to the side sensor faces, measure
int kick_ms = 40;              // k  full-power burst when a stopped motor starts, 0 = off
int use_median = 1;            // q  median of last 3 readings (0 = raw)


// RUNTIME STATE: sensor readings, controller, localization and FSM variables
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
unsigned long ir_white_since = 0;

float theta_deg = 0;   // + = nose away from the wall
float d_mm = 0;

float integral_L = 0;
float steer_L = 0, steer_H = 0, steer_I = 0, steer_out = 0;
unsigned long last_ctrl_ms = 0;
bool motors_stopped = true;   // the next start gets a kick

bool localized = false;
int ahead_wall = -1;          // wall faced at the last corner; while following it's the wall on the right
int last_colour = -1;         // colour seen at the previous corner, -1 = none yet
bool stopped_by_ir = false;
bool colour_sure = false;     // both IR sensors agreed on the last colour read
bool turn_around = false;     // the next TURN_LEFT is 180 deg, after a right check
bool crept = false;           // CHECK_RIGHT had to drive in to read the wall
unsigned long settle_ms = 0;  // when CHECK_RIGHT stopped at the wall, 0 = not yet
bool visited[NUM_WP] = {false, false, false};
int wps_visited = 0;
int current_wp = -1;
unsigned long l_timer_ms = 0; // when the top-left turn finished, 0 = not heading for L
float pose_x = 0, pose_y = 0, pose_th = 0;   // mm, mm, deg
unsigned long last_pose_ms = 0;

enum State { FOLLOW_WALL, CORNER, TURN_RIGHT, CHECK_RIGHT, AT_WAYPOINT, TURN_LEFT, LEFT_EXIT, DONE };
State state = FOLLOW_WALL;
unsigned long state_start_ms = 0;
unsigned long overturn_start_ms = 0;
int ahead_close_count = 0;
unsigned long last_print_ms = 0;

char cmd[16];
int cmd_len = 0;


// SETUP: pins, filters and a 3 s pause to put the robot down
void setup() {
  Serial.begin(115200);
  for (int s = 0; s < 3; s++) {
    pinMode(TRIG[s], OUTPUT);
    pinMode(ECHO[s], INPUT);
    for (int k = 0; k < 3; k++) hist[s][k] = NO_ECHO;
  }
  pinMode(R_FWD, OUTPUT); pinMode(R_REV, OUTPUT);
  pinMode(L_FWD, OUTPUT); pinMode(L_REV, OUTPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_LEFT, INPUT);
  stopMotors();

  const char* const MODE_NAME[5] = {"sensors only, motors off", "drive straight", "follow the wall",
    "follow, turn at corners and localize", "full run: localize, then visit BR, TR, L"};
  Serial.print("MODE "); Serial.print(MODE); Serial.print(": "); Serial.println(MODE_NAME[MODE]);
  Serial.println("tune: w p h i a s x m f u n t e o d b r z l y c k q, g prints all");
  if (MODE >= 1) {
    Serial.println("starting in 3 s, put the robot down");
    delay(3000);
  }
  setState(FOLLOW_WALL);
}


// MAIN LOOP: read sensors, react to new readings, then run the current FSM state
void loop() {
  handleSerial();
  updateSensors();
  updateIR();

  if (MODE == 0) { printStatusEvery500ms(); return; }
  if (MODE == 1) { drive(base, base); printStatusEvery500ms(); return; }

  if (sides_fresh) {
    sides_fresh = false;
    if (sidesSeeWall(dist)) estimate(dist, theta_deg, d_mm);
    updatePose();

    if (state == FOLLOW_WALL) {
      if (sidesSeeWall(dist)) {
        float s = wallController();
        drive(base + s, base - s);   // s > 0 turns right, toward the wall
      } else {
        driveStraight();             // no wall yet: keep going until one is ahead
      }
    }
  }

  if (ahead_fresh) {
    ahead_fresh = false;
    ahead_close_count = (dist[AHEAD] < front_stop_mm) ? ahead_close_count + 1 : 0;
  }

  // corner: a black wall only shows on the ahead sensor, a white one also lights the IR sensors
  if (MODE >= 3 && (state == FOLLOW_WALL || state == LEFT_EXIT)) {
    bool black_close = ahead_close_count >= (use_median ? 1 : 2);   // median already needs 2 of 3
    bool white_seen = irSeesWhite() && dist[AHEAD] < IR_GATE_MM;
    if (black_close || white_seen) {
      stopped_by_ir = white_seen;
      setState(CORNER);
    }
  }

  // waypoint L: a set time after the top-left turn
  if (MODE == 4 && state == FOLLOW_WALL && l_timer_ms != 0 && millis() - l_timer_ms >= (unsigned long)l_time_ms) {
    l_timer_ms = 0;
    arriveAt(WP_L);
  }

  unsigned long elapsed = millis() - state_start_ms;   // after any state change above
  switch (state) {
    case CORNER:
      stopMotors();
      if (elapsed >= 250) handleCorner(readColour());   // let it stop rocking first
      break;

    case TURN_RIGHT:
      spinRight(turn_pwm);
      if (elapsed >= (unsigned long)right_turn_ms) setState(CHECK_RIGHT);
      break;

    case CHECK_RIGHT:
      if (settle_ms != 0) {
        stopMotors();
        if (millis() - settle_ms >= 250) handleRightCheck(readColour());   // let it stop rocking first
      } else if (elapsed < SETTLE_MS) {
        stopMotors();   // wait for fresh ahead readings in the new direction
      } else if (dist[AHEAD] < front_stop_mm || (irSeesWhite() && dist[AHEAD] < IR_GATE_MM) || elapsed > CHECK_TIMEOUT_MS) {
        stopMotors();   // close enough to read the colour
        settle_ms = millis();
        event("CHECK_RIGHT");
        if (elapsed > CHECK_TIMEOUT_MS) Serial.println("no wall after 8 s, reading anyway");
        else if (crept) Serial.println("crept in, close enough to read");
        else { Serial.print("wall already "); printMm(dist[AHEAD]); Serial.println(" away, no need to creep"); }
      } else {
        if (!crept) { event("CHECK_RIGHT"); Serial.print("wall "); printMm(dist[AHEAD]); Serial.println(" away, too far to read, creeping in"); }
        crept = true;
        driveStraight();
      }
      break;

    case AT_WAYPOINT:
      stopMotors();
      if (elapsed >= (unsigned long)hold_ms) leaveWaypoint();
      break;

    case TURN_LEFT:
      spinLeft(turn_pwm);
      if (overturn_start_ms == 0 && leftTurnDone(elapsed)) overturn_start_ms = millis();
      if (overturn_start_ms != 0 && millis() - overturn_start_ms >= (unsigned long)max(overturn_ms, 0)) {
        stopMotors();
        afterLeftTurn();
        setState(LEFT_EXIT);
      }
      break;

    case LEFT_EXIT:
      driveStraight();
      if (elapsed >= (unsigned long)exit_straight_ms) setState(FOLLOW_WALL);
      break;

    case DONE:
      stopMotors();
      break;

    case FOLLOW_WALL:   // driven by the sensor updates above
      break;
  }

  printStatusEvery500ms();
}


// LOCALIZATION LOGIC: name the walls from the colours of two neighbouring walls
void handleCorner(Colour c) {
  event("CORNER"); Serial.print("wall ahead is "); Serial.print(c == WHITE ? "WHITE" : "BLACK");
  if (stopped_by_ir) Serial.println(", stopped by the IR sensors");
  else { Serial.print(", stopped by the ahead sensor at "); printMm(dist[AHEAD]); Serial.println(); }

  if (!colour_sure) {
    // one sensor on white, one on black: probably facing a corner at an angle
    event("CORNER"); Serial.println("the IR sensors disagree, not using this colour");
    if (localized) ahead_wall = (ahead_wall + 1) % 4;   // still one wall on, just unchecked
    else { setState(TURN_LEFT); return; }
  } else if (localized) {
    ahead_wall = (ahead_wall + 1) % 4;   // one left turn = one wall on, counter-clockwise
    if (WALL_COLOUR[ahead_wall] != c && last_colour >= 0) {
      event("MISMATCH"); Serial.print("expected the "); Serial.print(WALL_NAME[ahead_wall]);
      Serial.println(" wall but the colour is wrong, trusting the last two colours instead");
      ahead_wall = wallFromColours((Colour)last_colour, c);
    }
  } else if (last_colour >= 0) {
    ahead_wall = wallFromColours((Colour)last_colour, c);
    localized = true;
    event("LOCALIZED"); Serial.print("at the "); Serial.print(CORNER_NAME[ahead_wall]);
    Serial.print(" corner, facing the "); Serial.print(WALL_NAME[ahead_wall]); Serial.println(" wall");
  } else {
    // one colour isn't enough: read the clockwise neighbour too
    last_colour = c;
    event("CORNER"); Serial.println("one colour isn't enough, turning right to read the next wall");
    setState(TURN_RIGHT);
    return;
  }
  if (colour_sure) last_colour = c;

  setPoseAtCorner();
  stopAtCornerWaypoint();
}

// after the right turn: this wall comes just before the first one, counter-clockwise
void handleRightCheck(Colour c) {
  event("CHECK_RIGHT"); Serial.print("this wall is "); Serial.println(c == WHITE ? "WHITE" : "BLACK");
  turn_around = true;   // turn left 180 so the first wall ends up on the right

  if (!colour_sure) {
    event("CHECK_RIGHT"); Serial.println("the IR sensors disagree, will localize at the next corner instead");
    setState(TURN_LEFT);
    return;
  }

  // either way we're now at the corner where this wall meets the first one
  ahead_wall = wallFromColours(c, (Colour)last_colour);   // the first wall
  localized = true;
  event("LOCALIZED"); Serial.print("at the "); Serial.print(CORNER_NAME[ahead_wall]);
  Serial.print(" corner, the first wall was the "); Serial.print(WALL_NAME[ahead_wall]); Serial.println(" wall");

  setPoseAtCorner();
  pose_th = FACING_DEG[(ahead_wall + 3) % 4];   // actually facing the wall we just read
  stopAtCornerWaypoint();
}

// BR and TR are ring corners: stop there if we haven't yet, otherwise turn onto the next wall
void stopAtCornerWaypoint() {
  int wp = (ahead_wall == W_RIGHT) ? WP_BR : (ahead_wall == W_TOP) ? WP_TR : -1;
  if (MODE == 4 && wp >= 0 && !visited[wp]) arriveAt(wp);
  else setState(TURN_LEFT);
}

// each (previous wall, wall ahead) colour pair happens exactly once around W B B W
int wallFromColours(Colour before, Colour ahead) {
  for (int i = 0; i < 4; i++)
    if (WALL_COLOUR[(i + 3) % 4] == before && WALL_COLOUR[i] == ahead) return i;
  return 0;   // unreachable, all four pairs exist
}

// after turning at a corner, the wall we faced is now on the right
void afterLeftTurn() {
  bool was_around = turn_around;
  turn_around = false;
  if (!localized) return;
  pose_th += was_around ? 180 : 90;
  if (pose_th > 180) pose_th -= 360;
  event("TURN_LEFT"); Serial.print("done, now on the "); Serial.print(WALL_NAME[ahead_wall]); Serial.println(" wall");
  if (MODE == 4 && ahead_wall == W_LEFT && !visited[WP_L]) {
    l_timer_ms = millis();
    event("WAYPOINT"); Serial.print("L is next, driving for "); Serial.print(l_time_ms); Serial.println(" ms");
  }
}


// WAYPOINTS: stop for 3 s, then carry on around
void arriveAt(int wp) {
  current_wp = wp;
  event("WAYPOINT"); Serial.print("reached "); Serial.print(WP_NAME[wp]);
  printPose(); Serial.println();
  setState(AT_WAYPOINT);
}

void leaveWaypoint() {
  visited[current_wp] = true;
  wps_visited++;
  event("WAYPOINT"); Serial.print(WP_NAME[current_wp]); Serial.print(" done, ");
  Serial.print(wps_visited); Serial.print(" of "); Serial.print(NUM_WP); Serial.println(" visited");
  if (wps_visited >= NUM_WP) setState(DONE);
  else if (current_wp == WP_L) setState(FOLLOW_WALL);   // mid-wall: keep following
  else setState(TURN_LEFT);                             // corner: turn onto the next wall
}


// POSE ESTIMATE: exact at each corner, then dead reckoning plus the wall on the right
void setPoseAtCorner() {
  pose_x = CORNER_X[ahead_wall];
  pose_y = CORNER_Y[ahead_wall];
  pose_th = FACING_DEG[ahead_wall];
  last_pose_ms = millis();
}

void updatePose() {
  unsigned long now = millis();
  float dt = (now - last_pose_ms) / 1000.0;
  last_pose_ms = now;
  if (!localized) return;

  // along the wall: speed x time
  if (state == FOLLOW_WALL || state == LEFT_EXIT) {
    pose_x += mm_per_s * dt * cos(radians(pose_th));
    pose_y += mm_per_s * dt * sin(radians(pose_th));
  }

  // across the wall and heading: the side sensors measure them directly
  if (state == FOLLOW_WALL && sidesSeeWall(dist)) {
    float c = d_mm + side_off_mm;   // robot centre to the wall on the right
    switch (ahead_wall) {
      case W_BOTTOM: pose_y = c; break;
      case W_RIGHT:  pose_x = ARENA_MM - c; break;
      case W_TOP:    pose_y = ARENA_MM - c; break;
      case W_LEFT:   pose_x = c; break;
    }
    pose_th = FACING_DEG[ahead_wall] + 90 + theta_deg;   // wall on the right: heading is 90 left of facing it
    if (pose_th > 180) pose_th -= 360;
  }
}


// WALL FOLLOWING CONTROL: A3 PI controller on distance + heading, and the left-turn end check
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
// a 180 turn gets twice the time. half way round the ahead sensor sees a wall, so it can't end there
bool leftTurnDone(unsigned long elapsed) {
  unsigned long scale = turn_around ? 2 : 1;
  if (elapsed >= scale * turn_max_ms) return true;
  if (elapsed < scale * turn_min_ms) return false;
  if (!sidesSeeWall(raw)) return false;
  float th, d;
  estimate(raw, th, d);
  return th > -left_done_deg && raw[AHEAD] > ahead_clear_mm;
}


// ULTRASONIC SENSORS: non-blocking pings, median filter, wall angle and distance
// each call fires a ping or checks the echo pin, never waits
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


// IR SENSORS: white walls reflect IR back to the forward sensors, black walls don't
// either sensor on white for 20 ms straight, so one bad read can't stop the robot
void updateIR() {
  bool white = digitalRead(IR_RIGHT) == IR_WHITE || digitalRead(IR_LEFT) == IR_WHITE;
  if (!white) ir_white_since = 0;
  else if (ir_white_since == 0) ir_white_since = millis();
}

bool irSeesWhite() {
  return ir_white_since != 0 && millis() - ir_white_since >= 20;
}

// stopped at a wall: 20 reads from each sensor, majority of all 40 decides.
// colour_sure only if both sensors clearly say the same thing
Colour readColour() {
  int right_white = 0, left_white = 0;
  for (int i = 0; i < 20; i++) {
    if (digitalRead(IR_RIGHT) == IR_WHITE) right_white++;
    if (digitalRead(IR_LEFT) == IR_WHITE) left_white++;
    delay(2);
  }
  bool both_white = right_white >= SURE_READS && left_white >= SURE_READS;
  bool both_black = right_white <= 20 - SURE_READS && left_white <= 20 - SURE_READS;
  colour_sure = both_white || both_black;
  event("IR"); Serial.print("white reads: right "); Serial.print(right_white);
  Serial.print("/20, left "); Serial.print(left_white);
  Serial.println(colour_sure ? "/20, sensors agree" : "/20, sensors DISAGREE");
  return (right_white + left_white > 20) ? WHITE : BLACK;
}


// MOTORS: wheel commands, left_trim keeps straight driving straight, a kick gets them going from a stop
// every motor command ends up here, as a pwm for each pin of the two wheels
void setWheels(int lf, int lr, int rf, int rr) {
  lf = constrain(lf, 0, 255); lr = constrain(lr, 0, 255);
  rf = constrain(rf, 0, 255); rr = constrain(rr, 0, 255);
  bool moving = lf || lr || rf || rr;
  if (moving && motors_stopped && kick_ms > 0) {
    // a stopped motor needs more push to start than to keep turning: full power for a moment
    analogWrite(L_FWD, lf ? 255 : 0); analogWrite(L_REV, lr ? 255 : 0);
    analogWrite(R_FWD, rf ? 255 : 0); analogWrite(R_REV, rr ? 255 : 0);
    delay(kick_ms);
  }
  analogWrite(L_FWD, lf); analogWrite(L_REV, lr);
  analogWrite(R_FWD, rf); analogWrite(R_REV, rr);
  motors_stopped = !moving;
}

void drive(int left, int right) {
  setWheels(left - left_trim, 0, right, 0);
}

void driveStraight() {   // keeps the integral's motor correction
  drive(base + steer_I, base - steer_I);
}

void spinLeft(int pwm) {
  setWheels(0, pwm, pwm, 0);
}

void spinRight(int pwm) {
  setWheels(pwm, 0, 0, pwm);
}

void stopMotors() {
  setWheels(0, 0, 0, 0);
}


// STATE CHANGES: every transition goes through setState so timers and counters reset
void setState(State s) {
  state = s;
  state_start_ms = millis();
  ahead_close_count = 0;
  overturn_start_ms = 0;
  settle_ms = 0;
  crept = false;
  if (s == FOLLOW_WALL) last_ctrl_ms = millis();   // no huge dt after a turn
  stamp();
  Serial.print("-> "); Serial.print(stateName(s));
  Serial.print(": "); Serial.println(stateAction(s));
}

const char* stateName(State s) {
  switch (s) {
    case FOLLOW_WALL: return "FOLLOW_WALL";
    case CORNER:      return "CORNER";
    case TURN_RIGHT:  return "TURN_RIGHT";
    case CHECK_RIGHT: return "CHECK_RIGHT";
    case AT_WAYPOINT: return "AT_WAYPOINT";
    case TURN_LEFT:   return "TURN_LEFT";
    case LEFT_EXIT:   return "LEFT_EXIT";
    case DONE:        return "DONE";
  }
  return "?";
}

// what the robot is doing in each state, for the serial monitor
const char* stateAction(State s) {
  switch (s) {
    case FOLLOW_WALL: return "following the wall on the right (straight ahead if there isn't one)";
    case CORNER:      return "stopped at a wall, reading its colour";
    case TURN_RIGHT:  return "turning right to face the next wall clockwise";
    case CHECK_RIGHT: return "checking the distance, creeping in only if too far to read";
    case AT_WAYPOINT: return "stopped on a waypoint for 3 s";
    case TURN_LEFT:   return turn_around ? "turning left 180 so the first wall is on the right"
                                         : "turning left so the wall ahead is on the right";
    case LEFT_EXIT:   return "pulling away from the corner";
    case DONE:        return "finished, motors off";
  }
  return "?";
}


// SERIAL OUTPUT: plain-English log. a line on every state change and event, plus a status line every 0.5 s
//   12.4s  -> FOLLOW_WALL: following the wall on the right (straight ahead if there isn't one)
//   12.9s  FOLLOW_WALL | right wall 192 mm (want 190), angle +2 | ahead 830 mm | IR black | steering left 4 | on the bottom wall, x 1080 y 251 heading 0
//   15.1s  CORNER: wall ahead is BLACK, stopped by the ahead sensor at 168 mm

// time since power-on, so you can see how long each step took
void stamp() {
  float s = millis() / 1000.0;
  if (s < 100) Serial.print(' ');
  if (s < 10) Serial.print(' ');
  Serial.print(s, 1); Serial.print("s  ");
}

// start an event line, e.g. "  5.2s  CORNER: "
void event(const char* tag) {
  stamp(); Serial.print(tag); Serial.print(": ");
}

void printMm(float mm) {
  if (mm >= NO_ECHO) { Serial.print("no echo"); return; }
  Serial.print(mm, 0); Serial.print(" mm");
}

void printPose() {
  Serial.print(", x "); Serial.print(pose_x, 0);
  Serial.print(" y "); Serial.print(pose_y, 0);
  Serial.print(" heading "); Serial.print(pose_th, 0);
}

// print only between pings so serial never delays timing an echo
void printStatusEvery500ms() {
  if (echo_phase != IDLE || millis() - last_print_ms < 500) return;
  last_print_ms = millis();
  printStatus();
}

void printStatus() {
  stamp();
  if (MODE == 0) {   // sensor check: every sensor in words
    Serial.print("SENSORS | ahead "); printMm(dist[AHEAD]);
    Serial.print(" | right side: front "); printMm(dist[FRONT]);
    Serial.print(", back "); printMm(dist[BACK]);
    Serial.print(" | IR right "); Serial.print(digitalRead(IR_RIGHT) == IR_WHITE ? "white" : "black");
    Serial.print(", left "); Serial.println(digitalRead(IR_LEFT) == IR_WHITE ? "white" : "black");
    return;
  }

  Serial.print(stateName(state));
  if (state == DONE) { Serial.println(); return; }

  Serial.print(" | right wall ");
  if (sidesSeeWall(dist)) {
    printMm(d_mm);
    Serial.print(" (want "); Serial.print(target_mm, 0);
    Serial.print("), angle "); if (theta_deg >= 0) Serial.print('+'); Serial.print(theta_deg, 0);
  } else {
    Serial.print("not seen");
  }
  Serial.print(" | ahead "); printMm(dist[AHEAD]);
  Serial.print(" | IR "); Serial.print(irSeesWhite() ? "white" : "black");

  if (state == FOLLOW_WALL && sidesSeeWall(dist)) {
    Serial.print(" | steering ");
    if (abs(steer_out) < 1) Serial.print("straight");
    else { Serial.print(steer_out > 0 ? "right " : "left "); Serial.print(abs(steer_out), 0); }
  }
  if (l_timer_ms != 0) {
    Serial.print(" | L in "); Serial.print(max(0L, (long)l_time_ms - (long)(millis() - l_timer_ms)) / 1000.0, 1); Serial.print(" s");
  }
  if (state == AT_WAYPOINT) {
    Serial.print(" | at "); Serial.print(WP_NAME[current_wp]); Serial.print(", leaving in ");
    Serial.print(max(0L, (long)hold_ms - (long)(millis() - state_start_ms)) / 1000.0, 1); Serial.print(" s");
  }

  if (localized) {
    Serial.print(" | on the "); Serial.print(WALL_NAME[ahead_wall]); Serial.print(" wall"); printPose();
    Serial.print(" | "); Serial.print(wps_visited); Serial.print("/3 waypoints");
  } else {
    Serial.print(last_colour < 0 ? " | not localized, no colours yet" : " | not localized, 1 colour read");
  }
  Serial.println();
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
  Serial.print(" b="); Serial.print(exit_straight_ms); Serial.print(" r="); Serial.print(right_turn_ms);
  Serial.print(" z="); Serial.print(hold_ms);   Serial.print(" l="); Serial.print(l_time_ms);
  Serial.print(" y="); Serial.print(mm_per_s);  Serial.print(" c="); Serial.print(side_off_mm);
  Serial.print(" k="); Serial.print(kick_ms);   Serial.print(" q="); Serial.println(use_median);
}


// SERIAL TUNING: change a tunable live by sending its letter + a number
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
    case 'r': right_turn_ms = (int)v; break;
    case 'z': hold_ms = (int)v; break;
    case 'l': l_time_ms = (int)v; break;
    case 'y': mm_per_s = v; break;
    case 'c': side_off_mm = v; break;
    case 'k': kick_ms = max((int)v, 0); break;
    case 'q': use_median = (int)v; break;
    default: Serial.println("unknown, use w p h i a s x m f u n t e o d b r z l y c k q g"); return;
  }
  integral_L = 0;   // restart the integral after any change
  printSettings();
}
