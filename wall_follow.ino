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
// after the first stop it only has one colour. if a wall is close and parallel on its right
// it is in a corner, so it turns right to face that clockwise
// neighbour, creeps in if that wall is too far to read, and reads it too. then it turns
// left 180 so the first wall is on its right, and follows it.
//
// navigate: once localized, each left turn moves one wall on. BR and TR are corners, so the
// robot stops there. L is half way down the left wall, reached by driving drive_to_L_ms after
// the top-left turn.
//
// FOLLOW_WALL -> black wall close on the ahead sensor, or IR sees white -> CORNER (read colour)
// CORNER      -> first colour, wall close on the right -> TURN_RIGHT -> CHECK_RIGHT (creep if needed, read) -> TURN_LEFT 180
// CORNER      -> first colour, no wall on the right -> TURN_LEFT, localize at the next corner
// CORNER      -> at BR or TR, not visited yet -> AT_WAYPOINT -> TURN_LEFT
// CORNER      -> anywhere else                -> TURN_LEFT -> LEFT_EXIT -> FOLLOW_WALL
// FOLLOW_WALL -> drive_to_L_ms after the top-left turn -> AT_WAYPOINT -> FOLLOW_WALL
// AT_WAYPOINT -> 3rd waypoint done -> DONE

#define MODE 4
// 0 sensors only, 1 straight, 2 follow, 3 + corners and localizing, 4 full run with waypoints


// PINS: same as A3, plus the two forward IR sensors
const int RIGHT_FWD_PIN = 6, RIGHT_REV_PIN = 5;     // right wheel
const int LEFT_FWD_PIN = 9, LEFT_REV_PIN = 10;    // left wheel
enum { SIDE_BACK, SIDE_FRONT, AHEAD };        // ping order
const int TRIG_PIN[3] = {7, 3, 12};
const int ECHO_PIN[3] = {8, 4, 13};
const int IR_RIGHT_PIN = 0, IR_LEFT_PIN = 1;   // forward IR sensors (DO pins), free on the R4 Minima since USB serial doesn't use D0/D1
const int IR_LEVEL_WHITE = LOW;              // most IR modules pull DO low when they see reflected IR (white), flip if not


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
const int NUM_WAYPOINTS = 3;


// CONSTANTS: fixed limits, same as A3
const float SIDE_SENSOR_GAP_MM = 103;        // gap between the two side sensors
const float NO_ECHO_MM = 4000;
const float SIDE_WALL_MAX_MM = 400;          // side reading past this = no wall
const float IR_TRUST_MM = 400;               // only trust the IR when the ahead sensor also sees a wall this close
const unsigned long PING_GAP_MS = 25;        // one sensor per 25 ms so echoes never overlap
const unsigned long ECHO_TIMEOUT_US = 15000;   // ~2.5 m
const unsigned long CHECK_SETTLE_MS = 300;   // stopped this long after a turn so the ahead median is fresh
const unsigned long CREEP_TIMEOUT_MS = 8000;   // CHECK_RIGHT gives up creeping after this
const int IR_AGREE_READS = 15;               // of 20 IR reads, this many must agree for a colour to count


// TUNABLES: set over serial as letter + number (p0.5), g prints all
float wall_target_mm = 190;                  // w  side sensors to the wall. ~190 + the sensor offset puts the centre on the 250 mm waypoint ring
float kp_distance = 0.4;                     // p  pwm per mm of distance error
float kp_heading = 2.5;                      // h  pwm per degree of heading error
float ki_distance = 0.0;                     // i  pwm per mm*s of built-up distance error (0 = off)
float max_approach_deg = 15;                 // a  steepest angle the distance term may ask for
int forward_pwm = 120;                       // s  forward pwm
int max_steer_pwm = 30;                      // x  most steering pwm the controller can add or take off
int left_trim_pwm = 7;                       // m  taken off the left wheel to drive straight
int black_wall_stop_mm = 230;                // f  ahead sensor stop for black walls. keep it under the IR range so white walls stop on the IR
int turn_pwm = 150;                          // u  spin pwm for left and right turns
int left_turn_min_ms = 250;                  // n  a left turn always spins at least this long
int left_turn_max_ms = 700;                  // t  a left turn gives up and ends after this
int ahead_clear_mm = 350;                    // e  TURN_LEFT can't end until ahead is clear past this
int left_turn_done_deg = 5;                  // d  TURN_LEFT ends within this many degrees of parallel
int left_exit_ms = 150;                      // b  LEFT_EXIT straight time
int right_turn_ms = 600;                     // r  time for a 90 deg right turn at turn_pwm, tune
float corner_right_wall_mm = 250;            // j  right wall closer than this at the first stop = in a corner
float corner_right_wall_deg = 10;            // v  and the robot is within this many deg of parallel to it
int waypoint_hold_ms = 5000;                 // z  stop at each waypoint (3 s required + margin)
int drive_to_L_ms = 2700;                    // l  drive time from the top-left turn to waypoint L (500 mm), tune
float forward_mm_per_s = 200;                // y  forward speed at `forward_pwm`, for the pose estimate, tune
float centre_to_side_sensor_mm = 60;         // c  robot centre to the side sensor faces, measure
int kick_start_ms = 40;                      // k  full-power burst when a stopped motor starts, 0 = off


// RUNTIME STATE: sensor readings, controller, localization and FSM variables
float raw_mm[3] = {NO_ECHO_MM, NO_ECHO_MM, NO_ECHO_MM};   // newest reading
float history_mm[3][3];                      // last 3 readings per sensor
float filtered_mm[3] = {NO_ECHO_MM, NO_ECHO_MM, NO_ECHO_MM};  // median of the last 3, used for control
int next_sensor = SIDE_BACK;
unsigned long last_ping_ms = 0;
bool new_side_reading = false;
bool new_ahead_reading = false;

enum EchoPhase { IDLE, WAIT_RISE, WAIT_FALL };
EchoPhase echo_phase = IDLE;
unsigned long trigger_us = 0, echo_start_us = 0;
unsigned long ir_white_since_ms = 0;

float wall_angle_deg = 0;                    // + = nose away from the wall
float wall_dist_mm = 0;                      // side sensors to the wall, at a right angle

float distance_integral = 0;
float steer_integral = 0, steer_pwm = 0;
unsigned long last_control_ms = 0;
bool motors_stopped = true;                  // the next start gets a kick

bool localized = false;
int current_wall = -1;                       // wall faced at the last corner; while following it's the wall on the right
int previous_colour = -1;                    // colour seen at the previous corner, -1 = none yet
bool stopped_by_ir = false;                  // last corner stop came from the IR (white), not the ahead sensor
bool ir_sensors_agree = false;               // both IR sensors agreed on the last colour read
bool turning_180 = false;                    // the next TURN_LEFT is 180 deg, after a right check
bool crept_in = false;                       // CHECK_RIGHT had to drive in to read the wall
unsigned long reached_wall_ms = 0;           // when CHECK_RIGHT stopped at the wall, 0 = not yet
bool waypoint_visited[NUM_WAYPOINTS] = {false, false, false};
int waypoints_done = 0;                       // how many of BR, TR, L are finished
int current_waypoint = -1;
unsigned long l_drive_start_ms = 0;          // when the top-left turn finished, 0 = not heading for L
float pose_x_mm = 0, pose_y_mm = 0, pose_heading_deg = 0;   // where the robot thinks it is
unsigned long last_pose_ms = 0;

enum State { FOLLOW_WALL, CORNER, TURN_RIGHT, CHECK_RIGHT, AT_WAYPOINT, TURN_LEFT, LEFT_EXIT, DONE };
State state = FOLLOW_WALL;
unsigned long state_start_ms = 0;
int ahead_close_readings = 0;                // ahead readings under black_wall_stop_mm in a row
unsigned long last_print_ms = 0;

char serial_cmd[16];
int serial_cmd_len = 0;


// SETUP: pins, filters and a 3 s pause to put the robot down
void setup() {
  Serial.begin(115200);
  for (int s = 0; s < 3; s++) {
    pinMode(TRIG_PIN[s], OUTPUT);
    pinMode(ECHO_PIN[s], INPUT);
    for (int k = 0; k < 3; k++) history_mm[s][k] = NO_ECHO_MM;
  }
  pinMode(RIGHT_FWD_PIN, OUTPUT); pinMode(RIGHT_REV_PIN, OUTPUT);
  pinMode(LEFT_FWD_PIN, OUTPUT); pinMode(LEFT_REV_PIN, OUTPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(IR_LEFT_PIN, INPUT);
  stopMotors();

  const char* const MODE_NAME[5] = {"sensors only, motors off", "drive straight", "follow the wall",
    "follow, turn at corners and localize", "full run: localize, then visit BR, TR, L"};
  Serial.print("MODE "); Serial.print(MODE); Serial.print(": "); Serial.println(MODE_NAME[MODE]);
  Serial.println("tune: w p h i a s x m f u n t e d b r j v z l y c k, g prints all");
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

  if (MODE == 0) { printStatusEveryHalfSecond(); return; }
  if (MODE == 1) { drive(forward_pwm, forward_pwm); printStatusEveryHalfSecond(); return; }

  if (new_side_reading) {
    new_side_reading = false;
    if (sidesSeeWall(filtered_mm)) wallAngleAndDistance(filtered_mm, wall_angle_deg, wall_dist_mm);
    updatePose();

    if (state == FOLLOW_WALL) {
      if (sidesSeeWall(filtered_mm)) {
        float steer = wallController();
        drive(forward_pwm + steer, forward_pwm - steer);   // steer > 0 turns right, toward the wall
      } else {
        driveStraight();             // no wall yet: keep going until one is ahead
      }
    }
  }

  if (new_ahead_reading) {
    new_ahead_reading = false;
    ahead_close_readings = (filtered_mm[AHEAD] < black_wall_stop_mm) ? ahead_close_readings + 1 : 0;
  }

  // corner: a black wall only shows on the ahead sensor, a white one also lights the IR sensors
  if (MODE >= 3 && (state == FOLLOW_WALL || state == LEFT_EXIT)) {
    bool black_close = ahead_close_readings > 0;   // a fresh median reading since this state started
    bool white_seen = irSeesWhite() && filtered_mm[AHEAD] < IR_TRUST_MM;
    if (black_close || white_seen) {
      stopped_by_ir = white_seen;
      setState(CORNER);
    }
  }

  // waypoint L: a set time after the top-left turn
  if (MODE == 4 && state == FOLLOW_WALL && l_drive_start_ms != 0 && millis() - l_drive_start_ms >= (unsigned long)drive_to_L_ms) {
    l_drive_start_ms = 0;
    arriveAt(WP_L);
  }

  unsigned long elapsed = millis() - state_start_ms;   // after any state change above
  switch (state) {
    case CORNER:
      stopMotors();
      if (elapsed >= 250) handleCornerColour(readColour());   // let it stop rocking first
      break;

    case TURN_RIGHT:
      spinRight(turn_pwm);
      if (elapsed >= (unsigned long)right_turn_ms) setState(CHECK_RIGHT);
      break;

    case CHECK_RIGHT:
      if (reached_wall_ms != 0) {
        stopMotors();
        if (millis() - reached_wall_ms >= 250) handleRightCheck(readColour());   // let it stop rocking first
      } else if (elapsed < CHECK_SETTLE_MS) {
        stopMotors();   // wait for fresh ahead readings in the new direction
      } else if (filtered_mm[AHEAD] < black_wall_stop_mm || (irSeesWhite() && filtered_mm[AHEAD] < IR_TRUST_MM) || elapsed > CREEP_TIMEOUT_MS) {
        stopMotors();   // close enough to read the colour
        reached_wall_ms = millis();
        event("CHECK_RIGHT");
        if (elapsed > CREEP_TIMEOUT_MS) Serial.println("no wall after 8 s, reading anyway");
        else if (crept_in) Serial.println("crept in, close enough to read");
        else { Serial.print("wall already "); printMm(filtered_mm[AHEAD]); Serial.println(" away, no need to creep"); }
      } else {
        if (!crept_in) { event("CHECK_RIGHT"); Serial.print("wall "); printMm(filtered_mm[AHEAD]); Serial.println(" away, too far to read, creeping in"); }
        crept_in = true;
        driveStraight();
      }
      break;

    case AT_WAYPOINT:
      stopMotors();
      if (elapsed >= (unsigned long)waypoint_hold_ms) leaveWaypoint();
      break;

    case TURN_LEFT:
      spinLeft(turn_pwm);
      if (leftTurnDone(elapsed)) {
        stopMotors();
        finishLeftTurn();
        setState(LEFT_EXIT);
      }
      break;

    case LEFT_EXIT:
      driveStraight();
      if (elapsed >= (unsigned long)left_exit_ms) setState(FOLLOW_WALL);
      break;

    case DONE:
      stopMotors();
      break;

    case FOLLOW_WALL:   // driven by the sensor updates above
      break;
  }

  printStatusEveryHalfSecond();
}


// LOCALIZATION LOGIC: name the walls from the colours of two neighbouring walls
void handleCornerColour(Colour c) {
  event("CORNER"); Serial.print("wall ahead is "); Serial.print(c == WHITE ? "WHITE" : "BLACK");
  if (stopped_by_ir) Serial.println(", stopped by the IR sensors");
  else { Serial.print(", stopped by the ahead sensor at "); printMm(filtered_mm[AHEAD]); Serial.println(); }

  if (!ir_sensors_agree) {
    // one sensor on white, one on black: probably facing a corner at an angle
    event("CORNER"); Serial.println("the IR sensors disagree, not using this colour");
    if (localized) current_wall = (current_wall + 1) % 4;   // still one wall on, just unchecked
    else { setState(TURN_LEFT); return; }
  } else if (localized) {
    current_wall = (current_wall + 1) % 4;   // one left turn = one wall on, counter-clockwise
    if (WALL_COLOUR[current_wall] != c && previous_colour >= 0) {
      event("MISMATCH"); Serial.print("expected the "); Serial.print(WALL_NAME[current_wall]);
      Serial.println(" wall but the colour is wrong, trusting the last two colours instead");
      current_wall = wallFromColours((Colour)previous_colour, c);
    }
  } else if (previous_colour >= 0) {
    current_wall = wallFromColours((Colour)previous_colour, c);
    localized = true;
    event("LOCALIZED"); Serial.print("at the "); Serial.print(CORNER_NAME[current_wall]);
    Serial.print(" corner, facing the "); Serial.print(WALL_NAME[current_wall]); Serial.println(" wall");
  } else {
    // one colour isn't enough. a close, parallel wall on the right means we're in a corner,
    // so read that wall too. otherwise we're mid-wall: turn left and localize at the next corner
    previous_colour = c;
    if (inCornerWithRightWall()) {
      event("CORNER"); Serial.println("one colour isn't enough, wall close on the right, turning right to read it");
      setState(TURN_RIGHT);
    } else {
      event("CORNER"); Serial.println("one colour isn't enough, no wall close on the right, will read the next corner");
      setState(TURN_LEFT);
    }
    return;
  }
  if (ir_sensors_agree) previous_colour = c;

  setPoseAtCorner();
  stopIfCornerWaypoint();
}

// in a corner: both right sensors see a wall, it's close, and we're roughly parallel to it
bool inCornerWithRightWall() {
  float angle_deg = 0, dist_mm = NO_ECHO_MM;
  if (sidesSeeWall(filtered_mm)) wallAngleAndDistance(filtered_mm, angle_deg, dist_mm);
  event("CORNER"); Serial.print("right wall "); printMm(dist_mm);
  Serial.print(", angle "); Serial.print(angle_deg, 0); Serial.println(" deg");
  return dist_mm < corner_right_wall_mm && fabs(angle_deg) < corner_right_wall_deg;
}

// after the right turn: this wall comes just before the first one, counter-clockwise
void handleRightCheck(Colour c) {
  event("CHECK_RIGHT"); Serial.print("this wall is "); Serial.println(c == WHITE ? "WHITE" : "BLACK");
  turning_180 = true;   // turn left 180 so the first wall ends up on the right

  if (!ir_sensors_agree) {
    event("CHECK_RIGHT"); Serial.println("the IR sensors disagree, will localize at the next corner instead");
    setState(TURN_LEFT);
    return;
  }

  // either way we're now at the corner where this wall meets the first one
  current_wall = wallFromColours(c, (Colour)previous_colour);   // the first wall
  localized = true;
  event("LOCALIZED"); Serial.print("at the "); Serial.print(CORNER_NAME[current_wall]);
  Serial.print(" corner, the first wall was the "); Serial.print(WALL_NAME[current_wall]); Serial.println(" wall");

  setPoseAtCorner();
  pose_heading_deg = FACING_DEG[(current_wall + 3) % 4];   // actually facing the wall we just read
  stopIfCornerWaypoint();
}

// BR and TR are ring corners: stop there if we haven't yet, otherwise turn onto the next wall
void stopIfCornerWaypoint() {
  int wp = (current_wall == W_RIGHT) ? WP_BR : (current_wall == W_TOP) ? WP_TR : -1;
  if (MODE == 4 && wp >= 0 && !waypoint_visited[wp]) arriveAt(wp);
  else setState(TURN_LEFT);
}

// each (previous wall, wall ahead) colour pair happens exactly once around W B B W
int wallFromColours(Colour before, Colour ahead) {
  for (int i = 0; i < 4; i++)
    if (WALL_COLOUR[(i + 3) % 4] == before && WALL_COLOUR[i] == ahead) return i;
  return 0;   // unreachable, all four pairs exist
}

// after turning at a corner, the wall we faced is now on the right
void finishLeftTurn() {
  bool was_around = turning_180;
  turning_180 = false;
  if (!localized) return;
  pose_heading_deg += was_around ? 180 : 90;
  if (pose_heading_deg > 180) pose_heading_deg -= 360;
  event("TURN_LEFT"); Serial.print("done, now on the "); Serial.print(WALL_NAME[current_wall]); Serial.println(" wall");
  if (MODE == 4 && current_wall == W_LEFT && !waypoint_visited[WP_L]) {
    l_drive_start_ms = millis();
    event("WAYPOINT"); Serial.print("L is next, driving for "); Serial.print(drive_to_L_ms); Serial.println(" ms");
  }
}


// WAYPOINTS: stop for 3 s, then carry on around
void arriveAt(int wp) {
  current_waypoint = wp;
  event("WAYPOINT"); Serial.print("reached "); Serial.print(WP_NAME[wp]);
  printPose(); Serial.println();
  setState(AT_WAYPOINT);
}

void leaveWaypoint() {
  waypoint_visited[current_waypoint] = true;
  waypoints_done++;
  event("WAYPOINT"); Serial.print(WP_NAME[current_waypoint]); Serial.print(" done, ");
  Serial.print(waypoints_done); Serial.print(" of "); Serial.print(NUM_WAYPOINTS); Serial.println(" visited");
  if (waypoints_done >= NUM_WAYPOINTS) setState(DONE);
  else if (current_waypoint == WP_L) setState(FOLLOW_WALL);   // mid-wall: keep following
  else setState(TURN_LEFT);                             // corner: turn onto the next wall
}


// POSE ESTIMATE: exact at each corner, then dead reckoning plus the wall on the right
void setPoseAtCorner() {
  pose_x_mm = CORNER_X[current_wall];
  pose_y_mm = CORNER_Y[current_wall];
  pose_heading_deg = FACING_DEG[current_wall];
  last_pose_ms = millis();
}

void updatePose() {
  unsigned long now = millis();
  float dt = (now - last_pose_ms) / 1000.0;
  last_pose_ms = now;
  if (!localized) return;

  // along the wall: speed x time
  if (state == FOLLOW_WALL || state == LEFT_EXIT) {
    pose_x_mm += forward_mm_per_s * dt * cos(radians(pose_heading_deg));
    pose_y_mm += forward_mm_per_s * dt * sin(radians(pose_heading_deg));
  }

  // across the wall and heading: the side sensors measure them directly
  if (state == FOLLOW_WALL && sidesSeeWall(filtered_mm)) {
    float centre_to_wall_mm = wall_dist_mm + centre_to_side_sensor_mm;   // robot centre to the wall on the right
    switch (current_wall) {
      case W_BOTTOM: pose_y_mm = centre_to_wall_mm; break;
      case W_RIGHT:  pose_x_mm = ARENA_MM - centre_to_wall_mm; break;
      case W_TOP:    pose_y_mm = ARENA_MM - centre_to_wall_mm; break;
      case W_LEFT:   pose_x_mm = centre_to_wall_mm; break;
    }
    pose_heading_deg = FACING_DEG[current_wall] + 90 + wall_angle_deg;   // wall on the right: heading is 90 left of facing it
    if (pose_heading_deg > 180) pose_heading_deg -= 360;
  }
}


// WALL FOLLOWING CONTROL: A3 PI controller on distance + heading, and the left-turn end check
// steering pwm, + = turn right (toward the wall)
float wallController() {
  unsigned long now = millis();
  float dt = (now - last_control_ms) / 1000.0;
  last_control_ms = now;

  float distance_error = wall_dist_mm - wall_target_mm;   // + = too far
  float heading_error = wall_angle_deg;          // + = nose away

  // cap so the distance term alone never asks for more than max_approach_deg
  float cap = (kp_heading > 0) ? kp_heading * max_approach_deg : max_steer_pwm;
  float steer_distance = constrain(kp_distance * distance_error, -cap, cap);
  float steer_heading = kp_heading * heading_error;   // damping

  // anti-windup: only integrate near the target, cap at 20 pwm
  if (abs(distance_error) < 50) distance_integral += distance_error * dt;
  if (ki_distance > 0) distance_integral = constrain(distance_integral, -20 / ki_distance, 20 / ki_distance);
  steer_integral = ki_distance * distance_integral;

  steer_pwm = constrain(steer_distance + steer_heading + steer_integral, -max_steer_pwm, max_steer_pwm);
  return steer_pwm;
}

// turn ends use raw readings, the median lags while spinning
// theta is negative mid-spin (nose into the new wall) and rises toward 0
// a 180 turn gets twice the time. half way round the ahead sensor sees a wall, so it can't end there
bool leftTurnDone(unsigned long elapsed) {
  unsigned long scale = turning_180 ? 2 : 1;
  if (elapsed >= scale * left_turn_max_ms) return true;
  if (elapsed < scale * left_turn_min_ms) return false;
  if (!sidesSeeWall(raw_mm)) return false;
  float angle_deg, dist_mm;
  wallAngleAndDistance(raw_mm, angle_deg, dist_mm);
  return angle_deg > -left_turn_done_deg && raw_mm[AHEAD] > ahead_clear_mm;
}


// ULTRASONIC SENSORS: non-blocking pings, median filter, wall angle and distance
// each call fires a ping or checks the echo pin, never waits
void updateSensors() {
  unsigned long now = micros();
  int s = next_sensor;

  if (echo_phase == WAIT_RISE) {
    if (digitalRead(ECHO_PIN[s])) { echo_start_us = now; echo_phase = WAIT_FALL; }
    else if (now - trigger_us > ECHO_TIMEOUT_US) finishReading(NO_ECHO_MM);
    return;
  }
  if (echo_phase == WAIT_FALL) {
    if (!digitalRead(ECHO_PIN[s])) finishReading((now - echo_start_us) * 0.343 / 2);   // 0.343 mm/us, there and back
    else if (now - echo_start_us > ECHO_TIMEOUT_US) finishReading(NO_ECHO_MM);
    return;
  }

  if (millis() - last_ping_ms < PING_GAP_MS) return;
  last_ping_ms = millis();
  digitalWrite(TRIG_PIN[s], LOW);  delayMicroseconds(2);   // 12 us trigger, the only wait
  digitalWrite(TRIG_PIN[s], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN[s], LOW);
  trigger_us = micros();
  echo_phase = WAIT_RISE;
}

void finishReading(float mm) {
  int s = next_sensor;
  echo_phase = IDLE;
  raw_mm[s] = mm;
  history_mm[s][2] = history_mm[s][1]; history_mm[s][1] = history_mm[s][0]; history_mm[s][0] = mm;
  filtered_mm[s] = median3(history_mm[s][0], history_mm[s][1], history_mm[s][2]);
  if (s == SIDE_FRONT) new_side_reading = true;   // back was read just before
  if (s == AHEAD) new_ahead_reading = true;
  next_sensor = (s + 1) % 3;
}

// one bad reading (spike, dropout, no echo) is thrown out
float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

bool sidesSeeWall(const float *v) {
  return v[SIDE_BACK] < SIDE_WALL_MAX_MM && v[SIDE_FRONT] < SIDE_WALL_MAX_MM;
}

void wallAngleAndDistance(const float *v, float &angle_deg, float &dist_mm) {
  angle_deg = degrees(atan2(v[SIDE_FRONT] - v[SIDE_BACK], SIDE_SENSOR_GAP_MM));
  dist_mm = (v[SIDE_FRONT] + v[SIDE_BACK]) / 2.0 * cos(radians(angle_deg));
}


// IR SENSORS: white walls reflect IR back to the forward sensors, black walls don't
// either sensor on white for 20 ms straight, so one bad read can't stop the robot
void updateIR() {
  bool white = digitalRead(IR_RIGHT_PIN) == IR_LEVEL_WHITE || digitalRead(IR_LEFT_PIN) == IR_LEVEL_WHITE;
  if (!white) ir_white_since_ms = 0;
  else if (ir_white_since_ms == 0) ir_white_since_ms = millis();
}

bool irSeesWhite() {
  return ir_white_since_ms != 0 && millis() - ir_white_since_ms >= 20;
}

// stopped at a wall: 20 reads from each sensor, majority of all 40 decides.
// ir_sensors_agree only if both sensors clearly say the same thing
Colour readColour() {
  int right_white = 0, left_white = 0;
  for (int i = 0; i < 20; i++) {
    if (digitalRead(IR_RIGHT_PIN) == IR_LEVEL_WHITE) right_white++;
    if (digitalRead(IR_LEFT_PIN) == IR_LEVEL_WHITE) left_white++;
    delay(2);
  }
  bool both_white = right_white >= IR_AGREE_READS && left_white >= IR_AGREE_READS;
  bool both_black = right_white <= 20 - IR_AGREE_READS && left_white <= 20 - IR_AGREE_READS;
  ir_sensors_agree = both_white || both_black;
  event("IR"); Serial.print("white reads: right "); Serial.print(right_white);
  Serial.print("/20, left "); Serial.print(left_white);
  Serial.println(ir_sensors_agree ? "/20, sensors agree" : "/20, sensors DISAGREE");
  return (right_white + left_white > 20) ? WHITE : BLACK;
}


// MOTORS: wheel commands, left_trim_pwm keeps straight driving straight, a kick gets them going from a stop
// every motor command ends up here, as a pwm for each pin of the two wheels
void setWheels(int lf, int lr, int rf, int rr) {
  lf = constrain(lf, 0, 255); lr = constrain(lr, 0, 255);
  rf = constrain(rf, 0, 255); rr = constrain(rr, 0, 255);
  bool moving = lf || lr || rf || rr;
  if (moving && motors_stopped && kick_start_ms > 0) {
    // a stopped motor needs more push to start than to keep turning: full power for a moment
    analogWrite(LEFT_FWD_PIN, lf ? 255 : 0); analogWrite(LEFT_REV_PIN, lr ? 255 : 0);
    analogWrite(RIGHT_FWD_PIN, rf ? 255 : 0); analogWrite(RIGHT_REV_PIN, rr ? 255 : 0);
    delay(kick_start_ms);
  }
  analogWrite(LEFT_FWD_PIN, lf); analogWrite(LEFT_REV_PIN, lr);
  analogWrite(RIGHT_FWD_PIN, rf); analogWrite(RIGHT_REV_PIN, rr);
  motors_stopped = !moving;
}

void drive(int left, int right) {
  setWheels(left - left_trim_pwm, 0, right, 0);
}

void driveStraight() {   // keeps the integral's motor correction
  drive(forward_pwm + steer_integral, forward_pwm - steer_integral);
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
  ahead_close_readings = 0;
  reached_wall_ms = 0;
  crept_in = false;
  if (s == FOLLOW_WALL) last_control_ms = millis();   // no huge dt after a turn
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
    case TURN_LEFT:   return turning_180 ? "turning left 180 so the first wall is on the right"
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
  if (mm >= NO_ECHO_MM) { Serial.print("no echo"); return; }
  Serial.print(mm, 0); Serial.print(" mm");
}

void printPose() {
  Serial.print(", x "); Serial.print(pose_x_mm, 0);
  Serial.print(" y "); Serial.print(pose_y_mm, 0);
  Serial.print(" heading "); Serial.print(pose_heading_deg, 0);
}

// print only between pings so serial never delays timing an echo
void printStatusEveryHalfSecond() {
  if (echo_phase != IDLE || millis() - last_print_ms < 500) return;
  last_print_ms = millis();
  printStatus();
}

void printStatus() {
  stamp();
  if (MODE == 0) {   // sensor check: every sensor in words
    Serial.print("SENSORS | ahead "); printMm(filtered_mm[AHEAD]);
    Serial.print(" | right side: front "); printMm(filtered_mm[SIDE_FRONT]);
    Serial.print(", back "); printMm(filtered_mm[SIDE_BACK]);
    Serial.print(" | IR right "); Serial.print(digitalRead(IR_RIGHT_PIN) == IR_LEVEL_WHITE ? "white" : "black");
    Serial.print(", left "); Serial.println(digitalRead(IR_LEFT_PIN) == IR_LEVEL_WHITE ? "white" : "black");
    return;
  }

  Serial.print(stateName(state));
  if (state == DONE) { Serial.println(); return; }

  Serial.print(" | right wall ");
  if (sidesSeeWall(filtered_mm)) {
    printMm(wall_dist_mm);
    Serial.print(" (want "); Serial.print(wall_target_mm, 0);
    Serial.print("), angle "); if (wall_angle_deg >= 0) Serial.print('+'); Serial.print(wall_angle_deg, 0);
  } else {
    Serial.print("not seen");
  }
  Serial.print(" | ahead "); printMm(filtered_mm[AHEAD]);
  Serial.print(" | IR "); Serial.print(irSeesWhite() ? "white" : "black");

  if (state == FOLLOW_WALL && sidesSeeWall(filtered_mm)) {
    Serial.print(" | steering ");
    if (abs(steer_pwm) < 1) Serial.print("straight");
    else { Serial.print(steer_pwm > 0 ? "right " : "left "); Serial.print(abs(steer_pwm), 0); }
  }
  if (l_drive_start_ms != 0) {
    Serial.print(" | L in "); Serial.print(max(0L, (long)drive_to_L_ms - (long)(millis() - l_drive_start_ms)) / 1000.0, 1); Serial.print(" s");
  }
  if (state == AT_WAYPOINT) {
    Serial.print(" | at "); Serial.print(WP_NAME[current_waypoint]); Serial.print(", leaving in ");
    Serial.print(max(0L, (long)waypoint_hold_ms - (long)(millis() - state_start_ms)) / 1000.0, 1); Serial.print(" s");
  }

  if (localized) {
    Serial.print(" | on the "); Serial.print(WALL_NAME[current_wall]); Serial.print(" wall"); printPose();
    Serial.print(" | "); Serial.print(waypoints_done); Serial.print("/3 waypoints");
  } else {
    Serial.print(previous_colour < 0 ? " | not localized, no colours yet" : " | not localized, 1 colour read");
  }
  Serial.println();
}

void printSettings() {
  Serial.print("w="); Serial.print(wall_target_mm);  Serial.print(" p="); Serial.print(kp_distance);
  Serial.print(" h="); Serial.print(kp_heading);      Serial.print(" i="); Serial.print(ki_distance);
  Serial.print(" a="); Serial.print(max_approach_deg);
  Serial.print(" s="); Serial.print(forward_pwm);      Serial.print(" x="); Serial.print(max_steer_pwm);
  Serial.print(" m="); Serial.print(left_trim_pwm); Serial.print(" f="); Serial.print(black_wall_stop_mm);
  Serial.print(" u="); Serial.print(turn_pwm);  Serial.print(" n="); Serial.print(left_turn_min_ms);
  Serial.print(" t="); Serial.print(left_turn_max_ms); Serial.print(" e="); Serial.print(ahead_clear_mm);
  Serial.print(" d="); Serial.print(left_turn_done_deg);
  Serial.print(" b="); Serial.print(left_exit_ms); Serial.print(" r="); Serial.print(right_turn_ms);
  Serial.print(" j="); Serial.print(corner_right_wall_mm, 0); Serial.print(" v="); Serial.print(corner_right_wall_deg, 0);
  Serial.print(" z="); Serial.print(waypoint_hold_ms);   Serial.print(" l="); Serial.print(drive_to_L_ms);
  Serial.print(" y="); Serial.print(forward_mm_per_s);  Serial.print(" c="); Serial.print(centre_to_side_sensor_mm);
  Serial.print(" k="); Serial.print(kick_start_ms);
  Serial.println();
}


// SERIAL TUNING: change a tunable live by sending its letter + a number
// non-blocking: collects characters until newline, then applies the command
void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') {
      if (serial_cmd_len < (int)sizeof(serial_cmd) - 1) serial_cmd[serial_cmd_len++] = ch;
      continue;
    }
    if (serial_cmd_len == 0) continue;
    serial_cmd[serial_cmd_len] = '\0';
    serial_cmd_len = 0;
    applyCommand(serial_cmd[0], atof(serial_cmd + 1));
  }
}

void applyCommand(char c, float v) {
  switch (c) {
    case 'g': printSettings(); return;
    case 'w': wall_target_mm = v; break;
    case 'p': kp_distance = v; break;
    case 'h': kp_heading = v; break;
    case 'i': ki_distance = v; break;
    case 'a': max_approach_deg = v; break;
    case 's': forward_pwm = (int)v; break;
    case 'x': max_steer_pwm = (int)v; break;
    case 'm': left_trim_pwm = (int)v; break;
    case 'f': black_wall_stop_mm = (int)v; break;
    case 'u': turn_pwm = (int)v; break;
    case 'n': left_turn_min_ms = (int)v; break;
    case 't': left_turn_max_ms = (int)v; break;
    case 'e': ahead_clear_mm = (int)v; break;
    case 'd': left_turn_done_deg = (int)v; break;
    case 'b': left_exit_ms = (int)v; break;
    case 'r': right_turn_ms = (int)v; break;
    case 'j': corner_right_wall_mm = v; break;
    case 'v': corner_right_wall_deg = v; break;
    case 'z': waypoint_hold_ms = (int)v; break;
    case 'l': drive_to_L_ms = (int)v; break;
    case 'y': forward_mm_per_s = v; break;
    case 'c': centre_to_side_sensor_mm = v; break;
    case 'k': kick_start_ms = max((int)v, 0); break;
    default: Serial.println("unknown, use w p h i a s x m f u n t e d b r j v z l y c k g"); return;
  }
  distance_integral = 0;   // restart the integral after any change
  printSettings();
}
