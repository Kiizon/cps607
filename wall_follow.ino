// cps607 a4: the kidnapped robot
//
// localize: spin to face the nearest wall, drive to it and read its colour, turn left and
// follow it (wall on the right) to the corner, then read the colour of the wall ahead.
// counter-clockwise the walls go W B B W, so that colour pair names the wall we're on.
//
// navigate: keep following walls counter-clockwise 250 mm out. all 3 waypoints lie on that
// path, so the robot stops at each one as it passes it.
//
// pose: while following, the side sensors give heading and the distance to the wall, the
// front sensor gives the distance along it. dead reckoning fills the gaps.
//
// SCAN -> SEEK_WALL -> APPROACH_READ -> BACK_OFF -> TURN_LEFT -> LEFT_EXIT -> FOLLOW_WALL
// FOLLOW_WALL, wall ahead, not localized -> APPROACH_READ -> BACK_OFF -> localized
// FOLLOW_WALL, wall ahead, localized     -> TURN_LEFT
// FOLLOW_WALL, at a waypoint             -> AT_WAYPOINT -> FOLLOW_WALL / TURN_LEFT / DONE
// FOLLOW_WALL, side wall gone too long   -> SCAN (relocalize)

#define MODE 4
// 0 sensors only, 1 straight, 2 follow the 250 mm ring forever, 3 localize then stop, 4 full run


// PINS: same as A3, plus the line sensor and the wall colour sensor
const int R_FWD = 6, R_REV = 5;     // right wheel
const int L_FWD = 9, L_REV = 10;    // left wheel
enum { BACK, FRONT, AHEAD };        // ping order
const int TRIG[3] = {7, 3, 12};
const int ECHO[3] = {8, 4, 13};
const int LINE_PIN = 2;             // downward line sensor, sees the black star on the floor
const int LINE_ON_TAPE = 1;
const int COLOUR_PIN = A0;          // forward IR reflectance sensor, reads the wall colour
const bool WHITE_READS_HIGH = false;   // most TCRT5000 boards read low on white, flip if not
const int LED_PIN = -1;             // optional waypoint LED, -1 = none (13 is an echo pin)


// MAP: the arena, its wall colours and the waypoints
// origin bottom-left, x right, y up, heading counter-clockwise from +x
const float ARENA_MM = 1500;
enum Colour { WHITE, BLACK };
enum { W_BOTTOM, W_RIGHT, W_TOP, W_LEFT };   // counter-clockwise order
const Colour WALL_COLOUR[4] = {WHITE, BLACK, BLACK, WHITE};
const float WALL_NORMAL_DEG[4] = {-90, 0, 90, 180};   // direction from inside toward the wall
const char* const WALL_NAME[4] = {"bottom", "right", "top", "left"};

// wall = the wall on the robot's right when it reaches the waypoint
struct Waypoint { const char* name; float x, y; int wall; bool visited; };
Waypoint wps[] = {
  {"BR", 1250, 250,  W_BOTTOM, false},
  {"TR", 1250, 1250, W_RIGHT,  false},
  {"L",  250,  750,  W_LEFT,   false},
};
const int NUM_WP = sizeof(wps) / sizeof(wps[0]);


// CONSTANTS: fixed limits and timeouts
const float L_MM = 103;                       // gap between the two side sensors
const float NO_ECHO = 4000;
const float WALL_LOST_MM = 450;               // side reading past this = no wall
const float AHEAD_TRUST_MM = 1200;            // forward readings past this are too noisy for the pose
const unsigned long PING_GAP_MS = 25;         // one sensor per 25 ms so echoes never overlap
const unsigned long ECHO_TIMEOUT_US = 15000;  // ~2.5 m
const float SEEK_STOP_MM = 300;               // SEEK_WALL stops here and squares up
const float SCAN_MATCH_MM = 30;               // "back on the shortest reading" tolerance
const unsigned long SEEK_TIMEOUT_MS = 8000;
const unsigned long READ_TIMEOUT_MS = 5000;
const unsigned long BACKOFF_TIMEOUT_MS = 4000;
const unsigned long LOST_MS = 2500;           // side wall gone this long while localized -> relocalize
const int SQ_MAX_STEPS = 8;
const float SQ_GAIN_MM = 4;                   // a square-up step must shorten the range this much
const float POSE_GAIN = 0.5;                  // share of each measurement that goes into the pose


// TUNABLES: set over serial as letter + number (p0.5), g prints all
float ring_mm = 250;           // w  robot centre to the wall while following
float Kp_L = 0.4;              // p  pwm per mm of distance error
float Kp_H = 2.5;              // h  pwm per degree of heading error
float Ki = 0.0;                // i
float max_approach_deg = 15;   // a  steepest angle the distance term may ask for
int base = 120;                // s  forward pwm
int max_steer = 30;            // x
int left_trim = 7;             // m  taken off the left wheel to drive straight
int turn_pwm = 150;            // u
int turn_min_ms = 250;         // n
int turn_max_ms = 700;         // t
int ahead_clear_mm = 350;      // e  TURN_LEFT can't end until ahead is clear past this
int left_done_deg = 5;         // d  TURN_LEFT ends within this many degrees of parallel
int exit_straight_ms = 150;    // b  LEFT_EXIT straight time
int slow_pwm = 90;             // v  approach, back-off and slowest follow speed, keep above stall
int read_mm = 50;              // r  forward range where the wall colour is read
int colour_thr = 512;          // c  analog value halfway between black and white at read_mm
int scan_steps = 26;           // l  scan steps, should add up to a bit more than one full turn
int scan_step_ms = 60;         // j  spin time for one scan step (~15 deg)
int sq_step_ms = 40;           // k  spin time for one square-up step (~5 deg)
int star_window_mm = 50;       // f  look for the floor star this far either side of the expected spot, 0 = off
int hold_ms = 3200;            // z  stop at each waypoint (3 s required + margin)
int slow_zone_mm = 150;        // o  start slowing this far before a stop
float mm_per_s = 200;          // y  forward speed at `base`, for dead reckoning
int use_median = 1;            // q  median of last 3 readings (0 = raw)
float front_off_mm = 60;       // F  robot centre to the forward sensor face, measure
float side_off_mm = 60;        // S  robot centre to the side sensor faces, measure
float line_off_mm = 0;         // L  how far the line sensor sits ahead of the robot centre, measure


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
unsigned long line_since_ms = 0;

float theta_deg = 0;   // + = nose away from the wall
float d_mm = 0;        // side sensor face to the wall

float integral_L = 0;
float steer_L = 0, steer_H = 0, steer_I = 0, steer_out = 0;
unsigned long last_ctrl_ms = 0;
int fwd_pwm = 0;       // forward pwm actually commanded, for dead reckoning

bool localized = false;
int wall = -1;         // wall on our right, valid once localized
Colour colours[2];     // first wall read, then the wall ahead at the corner
int reads_done = 0;
int wps_visited = 0;
int current_wp = -1;
int window_wp = -1;    // waypoint whose star window we're in, -1 = none
float pose_x = 0, pose_y = 0, pose_th = 0;   // mm, mm, deg
unsigned long last_pose_ms = 0;
unsigned long last_wall_seen_ms = 0;

enum State { SCAN, SEEK_WALL, APPROACH_READ, BACK_OFF, TURN_LEFT, LEFT_EXIT, FOLLOW_WALL, AT_WAYPOINT, DONE };
State state = SCAN;
unsigned long state_start_ms = 0;
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
  pinMode(LINE_PIN, INPUT);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  stopMotors();

  Serial.print("mode "); Serial.println(MODE);
  Serial.println("tune: w p h i a s x m u n t e d b v r c l j k f z o y q F S L, g prints all");
  if (MODE >= 1) delay(3000);   // time to put the robot down
  setState(SCAN);
}


// MAIN LOOP: read sensors, react to new readings, then run the current FSM state
void loop() {
  handleSerial();
  updateSensors();
  updateLine();

  if (MODE == 0) { printEvery200ms(); return; }
  if (MODE == 1) { driveStraightAt(base); printEvery200ms(); return; }

  if (sides_fresh) {
    sides_fresh = false;
    if (sidesSeeWall(dist)) {
      estimate(dist, theta_deg, d_mm);
      last_wall_seen_ms = millis();
    }
    updatePose();

    if (state == FOLLOW_WALL) {
      int v = followSpeed();
      if (sidesSeeWall(dist)) {
        float s = wallController();
        drive(v + s, v - s);   // s > 0 turns right, toward the wall
      } else {
        driveStraightAt(v);
      }
      if (localized && millis() - last_wall_seen_ms > LOST_MS) relocalize();
    }
  }

  if (ahead_fresh) {
    ahead_fresh = false;
    if (state == FOLLOW_WALL || state == LEFT_EXIT) checkAhead();
  }

  // checked every loop, the star is small and passes under the sensor quickly
  if (state == FOLLOW_WALL && window_wp >= 0 && onStar()) arriveAtWaypoint(window_wp, true);

  unsigned long elapsed = millis() - state_start_ms;   // after any state change above
  switch (state) {
    case SCAN:
      faceNearestWall();
      setState(SEEK_WALL);
      break;

    case SEEK_WALL:
      driveStraightAt(base);
      if (dist[AHEAD] <= SEEK_STOP_MM) {
        stopMotors();
        squareUp();
        setState(MODE >= 3 ? APPROACH_READ : BACK_OFF);
      } else if (elapsed > SEEK_TIMEOUT_MS) {
        // likely aimed into a corner where the echo glances off, turn a bit and retry
        spinRight(turn_pwm); delay(250); stopMotors();
        setState(SEEK_WALL);
      }
      break;

    case APPROACH_READ:
      // creep in so the IR sensor is close enough to see the wall colour
      driveStraightAt(slow_pwm);
      if (dist[AHEAD] <= read_mm || elapsed > READ_TIMEOUT_MS) {
        stopMotors();
        delay(250);   // let it stop rocking
        colours[reads_done++] = readColour();
        setState(BACK_OFF);
      }
      break;

    case BACK_OFF:
      // reverse until we're ring_mm from the wall, back on the waypoint path
      driveReverseAt(slow_pwm);
      if ((dist[AHEAD] >= cornerStopMm() && dist[AHEAD] < NO_ECHO) || elapsed > BACKOFF_TIMEOUT_MS) {
        stopMotors();
        afterBackOff();
      }
      break;

    case TURN_LEFT:
      spinLeft(turn_pwm);
      if (leftTurnDone(elapsed)) {
        stopMotors();
        if (localized) wall = (wall + 1) % 4;   // every left turn moves us one wall counter-clockwise
        setState(LEFT_EXIT);
      }
      break;

    case LEFT_EXIT:
      fwd_pwm = base;
      driveStraightAt(base);
      if (elapsed >= (unsigned long)exit_straight_ms) setState(FOLLOW_WALL);
      break;

    case FOLLOW_WALL:   // driven by the sensor updates above
      break;

    case AT_WAYPOINT:
      stopMotors();
      if (elapsed >= (unsigned long)hold_ms) {
        wps[current_wp].visited = true;
        wps_visited++;
        if (LED_PIN >= 0) digitalWrite(LED_PIN, LOW);
        if (wps_visited >= NUM_WP) setState(DONE);
        else if (dist[AHEAD] <= cornerStopMm() + star_window_mm + 40) setState(TURN_LEFT);   // corner waypoint
        else setState(FOLLOW_WALL);                                                        // mid-wall waypoint
      }
      break;

    case DONE:
      stopMotors();
      break;
  }

  printEvery200ms();
}


// LOCALIZATION LOGIC: find the nearest wall, then name the wall from two colour readings
// the shortest range to a flat wall is straight at it, so: one full turn remembering the
// shortest reading, then keep turning until we're back on it. no step angles needed.
void faceNearestWall() {
  echo_phase = IDLE;   // drop any half-finished non-blocking ping
  stopMotors();
  delay(150);
  float best = NO_ECHO;
  for (int i = 0; i < scan_steps; i++) {
    best = min(best, aheadMedian5());
    spinStep(1, scan_step_ms);
  }
  float r = aheadMedian5();
  for (int i = 0; i < scan_steps && best < NO_ECHO && r > best + SCAN_MATCH_MM; i++) {
    spinStep(1, scan_step_ms);
    r = aheadMedian5();
  }
  resyncAhead(r);
}

// turn until the forward range stops getting shorter, i.e. facing the wall square
void squareUp() {
  echo_phase = IDLE;
  delay(150);
  float best = aheadMedian5();
  int dir = 1;   // try right first
  for (int pass = 0; pass < 2; pass++, dir = -dir) {
    int gained = 0;
    while (gained < SQ_MAX_STEPS) {
      spinStep(dir, sq_step_ms);
      float r = aheadMedian5();
      if (r < best - SQ_GAIN_MM) { best = r; gained++; }
      else { spinStep(-dir, sq_step_ms); break; }   // got worse: undo and stop
    }
    if (gained > 0) break;   // found the minimum this way, no need to try the other
  }
  resyncAhead(best);
}

void afterBackOff() {
  // first read (or MODE 2): turn so this wall is on our right, then follow it
  if (MODE < 3 || reads_done < 2) {
    setState(TURN_LEFT);
    return;
  }

  // second read: colours[0] is the wall on our right, colours[1] the next wall ccw, ahead
  wall = wallFromColours(colours[0], colours[1]);
  localized = true;
  reads_done = 0;
  fixPoseAtCorner();
  Serial.print("LOCALIZED: wall on right = "); Serial.print(WALL_NAME[wall]);
  Serial.print(", facing "); Serial.println(WALL_NAME[(wall + 1) % 4]);
  printPose();

  if (MODE == 3) { setState(DONE); return; }

  // we're on the ring corner at the end of this wall, which may itself be a waypoint
  int wp = waypointOnWall(wall);
  if (wp >= 0 && waypointStopMm(wp) <= cornerStopMm() + 1) arriveAtWaypoint(wp, false);
  else setState(TURN_LEFT);
}

// each (wall, next wall) colour pair happens exactly once around W B B W
int wallFromColours(Colour on_right, Colour ahead) {
  for (int i = 0; i < 4; i++)
    if (WALL_COLOUR[i] == on_right && WALL_COLOUR[(i + 1) % 4] == ahead) return i;
  return 0;   // unreachable, all four pairs exist
}

void relocalize() {
  Serial.println("LOST: side wall gone, relocalizing");
  localized = false;
  reads_done = 0;
  setState(SCAN);   // visited waypoints stay visited
}


// NAVIGATION LOGIC: where to stop next (waypoint or corner) and how fast to approach it
float distToWall(int w, float x, float y) {
  switch (w) {
    case W_BOTTOM: return y;
    case W_RIGHT:  return ARENA_MM - x;
    case W_TOP:    return ARENA_MM - y;
    default:       return x;
  }
}

// forward reading when the robot centre is on the waypoint (the wall ahead is the next one ccw)
float waypointStopMm(int wp) {
  return distToWall((wps[wp].wall + 1) % 4, wps[wp].x, wps[wp].y) - front_off_mm;
}

// turning here keeps the robot on the ring
float cornerStopMm() {
  return ring_mm - front_off_mm;
}

int waypointOnWall(int w) {
  for (int i = 0; i < NUM_WP; i++)
    if (wps[i].wall == w && !wps[i].visited) return i;
  return -1;
}

int upcomingWaypoint() {
  return localized ? waypointOnWall(wall) : -1;
}

void checkAhead() {
  int wp = upcomingWaypoint();
  if (wp >= 0) {
    // inside the window, black on the floor is the star; past it, stop on distance
    float gap = dist[AHEAD] - waypointStopMm(wp);
    window_wp = (gap <= star_window_mm) ? wp : -1;
    if (gap <= -star_window_mm) arriveAtWaypoint(wp, false);
    return;
  }

  window_wp = -1;
  ahead_close_count = (dist[AHEAD] <= cornerStopMm()) ? ahead_close_count + 1 : 0;
  if (ahead_close_count < (use_median ? 1 : 2)) return;   // median already needs 2 of 3
  setState(MODE >= 3 && !localized ? APPROACH_READ : TURN_LEFT);
}

// slow down before the next stop so we don't overshoot it, and cross the star slowly
int followSpeed() {
  int wp = upcomingWaypoint();
  float stop = (wp >= 0) ? waypointStopMm(wp) + star_window_mm : cornerStopMm();
  float gap = dist[AHEAD] - stop;
  if (gap >= slow_zone_mm) fwd_pwm = base;
  else fwd_pwm = slow_pwm + (base - slow_pwm) * max(gap, 0.0f) / slow_zone_mm;
  return fwd_pwm;
}

void arriveAtWaypoint(int wp, bool on_star) {
  current_wp = wp;
  if (on_star) fixPoseAtStar(wp);
  Serial.print("WAYPOINT "); Serial.print(wps[wp].name);
  Serial.println(on_star ? " (saw star)" : " (by distance)");
  printPose();
  if (LED_PIN >= 0) digitalWrite(LED_PIN, HIGH);
  setState(AT_WAYPOINT);
}


// POSE ESTIMATION: dead reckoning between readings, corrected by the walls and the stars
// move one coordinate toward what the distance to wall w says it is
void correctAxis(int w, float to_wall, float gain) {
  switch (w) {
    case W_BOTTOM: pose_y += gain * (to_wall - pose_y); break;
    case W_RIGHT:  pose_x += gain * ((ARENA_MM - to_wall) - pose_x); break;
    case W_TOP:    pose_y += gain * ((ARENA_MM - to_wall) - pose_y); break;
    case W_LEFT:   pose_x += gain * (to_wall - pose_x); break;
  }
}

float wrapDeg(float a) {
  while (a > 180) a -= 360;
  while (a <= -180) a += 360;
  return a;
}

// just localized: on the ring at the end of `wall`, facing the next wall
void fixPoseAtCorner() {
  int ahead_wall = (wall + 1) % 4;
  pose_th = WALL_NORMAL_DEG[ahead_wall];
  correctAxis(wall, ring_mm, 1.0);
  correctAxis(ahead_wall, dist[AHEAD] + front_off_mm, 1.0);
  last_pose_ms = millis();
}

// the line sensor is over the star, so the centre is line_off_mm behind it
void fixPoseAtStar(int wp) {
  pose_x = wps[wp].x - line_off_mm * cos(radians(pose_th));
  pose_y = wps[wp].y - line_off_mm * sin(radians(pose_th));
}

// predict from the commanded speed, then correct with whatever the walls say
void updatePose() {
  unsigned long now = millis();
  float dt = (now - last_pose_ms) / 1000.0;
  last_pose_ms = now;
  if (!localized) return;

  if (state == FOLLOW_WALL || state == LEFT_EXIT) {
    float v = mm_per_s * fwd_pwm / base;
    pose_x += v * dt * cos(radians(pose_th));
    pose_y += v * dt * sin(radians(pose_th));
  }

  if (state == FOLLOW_WALL && sidesSeeWall(dist)) {
    // wall on the right: heading is 90 deg left of the wall's normal, plus our angle to it
    pose_th = wrapDeg(WALL_NORMAL_DEG[wall] + 90 + theta_deg);
    correctAxis(wall, d_mm + side_off_mm, POSE_GAIN);
    if (dist[AHEAD] < AHEAD_TRUST_MM) {
      float along = dist[AHEAD] * cos(radians(theta_deg)) + front_off_mm;
      correctAxis((wall + 1) % 4, along, POSE_GAIN);
    }
  }
}


// WALL FOLLOWING CONTROL: A3 PI controller on distance + heading, and the left-turn end check
// steering pwm, + = turn right (toward the wall)
float wallController() {
  unsigned long now = millis();
  float dt = (now - last_ctrl_ms) / 1000.0;
  last_ctrl_ms = now;

  float target = ring_mm - side_off_mm;   // sensor-to-wall distance that puts the centre on the ring
  float e_L = d_mm - target;              // + = too far
  float e_H = theta_deg;                  // + = nose away

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

// uses raw readings, the median lags while spinning
// theta is negative mid-spin (nose into the new wall) and rises toward 0
bool leftTurnDone(unsigned long elapsed) {
  if (elapsed >= (unsigned long)turn_max_ms) return true;
  if (elapsed < (unsigned long)turn_min_ms) return false;
  if (!sidesSeeWall(raw)) return false;
  float th, d;
  estimate(raw, th, d);
  return th > -left_done_deg && raw[AHEAD] > ahead_clear_mm;
}


// ULTRASONIC SENSORS: non-blocking pings for driving, blocking pings for the scan, median filter
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

// angle and distance to the wall from the two side sensors
void estimate(const float *v, float &theta, float &d) {
  theta = degrees(atan2(v[FRONT] - v[BACK], L_MM));
  d = (v[FRONT] + v[BACK]) / 2.0 * cos(radians(theta));
}

// blocking ping, only used while stopped (scan and square-up)
float pingBlocking(int s) {
  digitalWrite(TRIG[s], LOW);  delayMicroseconds(2);
  digitalWrite(TRIG[s], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG[s], LOW);
  unsigned long t = pulseIn(ECHO[s], HIGH, ECHO_TIMEOUT_US);
  return t == 0 ? NO_ECHO : t * 0.343 / 2;
}

float aheadMedian5() {
  float r[5];
  for (int i = 0; i < 5; i++) { r[i] = pingBlocking(AHEAD); delay(30); }
  for (int a = 1; a < 5; a++) {   // insertion sort
    float key = r[a];
    int b = a - 1;
    while (b >= 0 && r[b] > key) { r[b + 1] = r[b]; b--; }
    r[b + 1] = key;
  }
  return r[2];
}

// after blocking reads, restart the forward filter so old values don't linger
void resyncAhead(float mm) {
  for (int k = 0; k < 3; k++) hist[AHEAD][k] = mm;
  raw[AHEAD] = dist[AHEAD] = mm;
  echo_phase = IDLE;
  last_ping_ms = millis();
}


// FLOOR AND WALL COLOUR SENSORS: line sensor for the stars, IR sensor for black vs white walls
// on black for 20 ms straight, so one bad read can't stop the robot
void updateLine() {
  if (digitalRead(LINE_PIN) != LINE_ON_TAPE) line_since_ms = 0;
  else if (line_since_ms == 0) line_since_ms = millis();
}

bool onStar() {
  return line_since_ms != 0 && millis() - line_since_ms >= 20;
}

// average of 20 reads, white reflects IR and black doesn't
Colour readColour() {
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(COLOUR_PIN); delay(2); }
  int v = sum / 20;
  bool white = WHITE_READS_HIGH ? (v > colour_thr) : (v < colour_thr);
  Serial.print("COLOUR: raw "); Serial.print(v);
  Serial.println(white ? " -> white" : " -> black");
  return white ? WHITE : BLACK;
}


// MOTORS: wheel commands, left_trim keeps straight driving straight
void drive(int left, int right) {
  analogWrite(L_FWD, constrain(left - left_trim, 0, 255)); analogWrite(L_REV, 0);
  analogWrite(R_FWD, constrain(right, 0, 255));            analogWrite(R_REV, 0);
}

void driveStraightAt(int pwm) {   // keeps the integral's motor correction
  drive(pwm + steer_I, pwm - steer_I);
}

void driveReverseAt(int pwm) {
  analogWrite(L_FWD, 0); analogWrite(L_REV, constrain(pwm - left_trim, 0, 255));
  analogWrite(R_FWD, 0); analogWrite(R_REV, constrain(pwm, 0, 255));
}

void spinLeft(int pwm) {
  analogWrite(R_FWD, pwm); analogWrite(R_REV, 0);
  analogWrite(L_FWD, 0);   analogWrite(L_REV, pwm);
}

void spinRight(int pwm) {
  analogWrite(R_FWD, 0);   analogWrite(R_REV, pwm);
  analogWrite(L_FWD, pwm); analogWrite(L_REV, 0);
}

// short timed spin then a pause, used by the scan and square-up
void spinStep(int dir, int ms) {
  if (dir > 0) spinRight(turn_pwm); else spinLeft(turn_pwm);
  delay(ms);
  stopMotors();
  delay(120);
}

void stopMotors() {
  analogWrite(R_FWD, 0); analogWrite(R_REV, 0);
  analogWrite(L_FWD, 0); analogWrite(L_REV, 0);
}


// STATE CHANGES: every transition goes through setState so timers and counters reset
void setState(State s) {
  state = s;
  state_start_ms = millis();
  ahead_close_count = 0;
  window_wp = -1;
  if (s == FOLLOW_WALL) {
    last_ctrl_ms = millis();       // no huge dt after a turn
    last_wall_seen_ms = millis();  // give it time to find the wall again
  }
  Serial.print("-> ");
  Serial.println(stateName(s));
}

const char* stateName(State s) {
  switch (s) {
    case SCAN:          return "SCAN";
    case SEEK_WALL:     return "SEEK_WALL";
    case APPROACH_READ: return "APPROACH_READ";
    case BACK_OFF:      return "BACK_OFF";
    case TURN_LEFT:     return "TURN_LEFT";
    case LEFT_EXIT:     return "LEFT_EXIT";
    case FOLLOW_WALL:   return "FOLLOW_WALL";
    case AT_WAYPOINT:   return "AT_WAYPOINT";
    case DONE:          return "DONE";
  }
  return "?";
}


// SERIAL OUTPUT: status line for the serial plotter, pose and settings
// print only between pings so serial never delays timing an echo
void printEvery200ms() {
  if (echo_phase != IDLE || millis() - last_print_ms < 200) return;
  last_print_ms = millis();
  printStatus();
}

void printStatus() {
  Serial.print("state:"); Serial.print((int)state);
  Serial.print(" ahead:"); Serial.print(dist[AHEAD], 0);
  Serial.print(" front:"); Serial.print(dist[FRONT], 0);
  Serial.print(" back:"); Serial.print(dist[BACK], 0);
  Serial.print(" d:"); Serial.print(d_mm, 0);
  Serial.print(" theta:"); Serial.print(theta_deg, 1);
  Serial.print(" steer:"); Serial.print(steer_out, 1);
  Serial.print(" line:"); Serial.print(digitalRead(LINE_PIN));
  if (MODE == 0) { Serial.print(" colour:"); Serial.print(analogRead(COLOUR_PIN)); }
  if (localized) {
    Serial.print(" wall:"); Serial.print(wall);
    Serial.print(" x:"); Serial.print(pose_x, 0);
    Serial.print(" y:"); Serial.print(pose_y, 0);
    Serial.print(" th:"); Serial.print(pose_th, 0);
  }
  Serial.println();
}

void printPose() {
  Serial.print("POSE x="); Serial.print(pose_x, 0);
  Serial.print(" y="); Serial.print(pose_y, 0);
  Serial.print(" th="); Serial.println(pose_th, 0);
}

void printSettings() {
  Serial.print("w="); Serial.print(ring_mm);     Serial.print(" p="); Serial.print(Kp_L);
  Serial.print(" h="); Serial.print(Kp_H);       Serial.print(" i="); Serial.print(Ki);
  Serial.print(" a="); Serial.print(max_approach_deg);
  Serial.print(" s="); Serial.print(base);       Serial.print(" x="); Serial.print(max_steer);
  Serial.print(" m="); Serial.print(left_trim);  Serial.print(" u="); Serial.print(turn_pwm);
  Serial.print(" n="); Serial.print(turn_min_ms); Serial.print(" t="); Serial.print(turn_max_ms);
  Serial.print(" e="); Serial.print(ahead_clear_mm); Serial.print(" d="); Serial.print(left_done_deg);
  Serial.print(" b="); Serial.print(exit_straight_ms); Serial.print(" v="); Serial.print(slow_pwm);
  Serial.print(" r="); Serial.print(read_mm);    Serial.print(" c="); Serial.print(colour_thr);
  Serial.print(" l="); Serial.print(scan_steps); Serial.print(" j="); Serial.print(scan_step_ms);
  Serial.print(" k="); Serial.print(sq_step_ms); Serial.print(" f="); Serial.print(star_window_mm);
  Serial.print(" z="); Serial.print(hold_ms);    Serial.print(" o="); Serial.print(slow_zone_mm);
  Serial.print(" y="); Serial.print(mm_per_s);   Serial.print(" q="); Serial.print(use_median);
  Serial.print(" F="); Serial.print(front_off_mm); Serial.print(" S="); Serial.print(side_off_mm);
  Serial.print(" L="); Serial.println(line_off_mm);
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
    case 'w': ring_mm = v; break;
    case 'p': Kp_L = v; break;
    case 'h': Kp_H = v; break;
    case 'i': Ki = v; break;
    case 'a': max_approach_deg = v; break;
    case 's': base = (int)v; break;
    case 'x': max_steer = (int)v; break;
    case 'm': left_trim = (int)v; break;
    case 'u': turn_pwm = (int)v; break;
    case 'n': turn_min_ms = (int)v; break;
    case 't': turn_max_ms = (int)v; break;
    case 'e': ahead_clear_mm = (int)v; break;
    case 'd': left_done_deg = (int)v; break;
    case 'b': exit_straight_ms = (int)v; break;
    case 'v': slow_pwm = (int)v; break;
    case 'r': read_mm = (int)v; break;
    case 'c': colour_thr = (int)v; break;
    case 'l': scan_steps = (int)v; break;
    case 'j': scan_step_ms = (int)v; break;
    case 'k': sq_step_ms = (int)v; break;
    case 'f': star_window_mm = max((int)v, 0); break;
    case 'z': hold_ms = (int)v; break;
    case 'o': slow_zone_mm = max((int)v, 1); break;
    case 'y': mm_per_s = v; break;
    case 'q': use_median = (int)v; break;
    case 'F': front_off_mm = v; break;
    case 'S': side_off_mm = v; break;
    case 'L': line_off_mm = v; break;
    default: Serial.println("unknown, use w p h i a s x m u n t e d b v r c l j k f z o y q F S L g"); return;
  }
  integral_L = 0;   // restart the integral after any change
  printSettings();
}
