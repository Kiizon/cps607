// pins
const int A_pinF = 6;     // motor A = RIGHT wheel
const int A_pinR = 5;
const int B_pinF = 9;     // motor B = LEFT wheel
const int B_pinR = 10;

const int linePin = 2;
const int frontTrigPin = 13;
const int frontEchoPin = 12;
const int irPinR = 7;
const int irPinL = 8;

// settings
const bool PLOT = false;       // true = Serial Plotter graph, false = text messages

const int LINE_ON_TAPE = 1;    // what the line sensor reads on the black tape

// Turning (search)
const int scan_speed = 140;    // above your minimum PWM
const int slow_scan_speed = 120; // after a black box: slower so we don't spin past the white one (keep above minimum PWM)
const long turn_time = 75;     // ms per ~10 deg (used to size a full circle and the random turn)
const int FULL_TURN_STEPS = 36;// ~10 deg steps in a full circle

// Detection
const float detect_range = 0.40; // m, turn until something is this close or closer

// Approach
const int approach_speed = 110;
const float stop_dist = 0.10;  // m
const long max_approach_time = 4000; // ms, give up if the box is never reached

// Color check
const long color_wait = 500;   // ms stopped so all 5 IR readings are fresh
const long nudge_time = 40;    // ms small turn to re-centre on the box
const int MAX_NUDGES = 3;
const int BLACK_LIMIT = 3;     // black boxes in a row before switching turn direction

// Push
const int push_speed = 160;
const int push_slow = 120;     // slower wheel when steering (above minimum PWM)
const long backoff_time = 800; // ms reversing after the box is out

// Escape (line seen when not pushing)
const long escape_time = 1500;  // ms reversing

// Reposition (nothing found in a full turn each way): turn randomly, drive, search again
const int RANDOM_TURN_MIN = 3;        // steps (~10 deg each)
const int RANDOM_TURN_MAX = 18;
const long RANDOM_DRIVE_MIN = 500;    // ms
const long RANDOM_DRIVE_MAX = 1500;
const float random_drive_stop = 0.15; // m, stop driving if something is this close ahead

// sensor variables
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 60;

float front_distance = 4.0; // Ultrasonic: raw reading. A moving average lagged while turning; no echo returns 4.0 m instead of 0 to avoid false detections

const int ir_window_size = 5;
int ir_window_R[ir_window_size];
int ir_window_L[ir_window_size];
int ir_window_index = 0;
int ir_sum_R = 0;
int ir_sum_L = 0;

// state machine
enum State {
  SEARCH_TURN,     // turning until something is within detect_range
  APPROACH,        // driving straight at it
  CHECK_COLOR,     // stopped, reading IR
  NUDGE,           // small turn to centre the box
  PUSH,            // pushing white box to the line
  BACKOFF,         // reversing after the box is out
  RETURN_BACK,     // reversing away from a black box
  ESCAPE,          // reversing away from the line
  DONE,
  RANDOM_TURN,     // nothing found: turning a random amount
  RANDOM_DRIVE     // nothing found: driving forward a random time, then searching again
};

State state = SEARCH_TURN;
unsigned long state_start = 0;

bool ignore_until_clear = false; // true = still looking at the rejected black box
bool slow_search = false;      // true after a black box, until a white one is found
int black_in_row = 0;          // black boxes rejected back to back
int search_dir = 1;            // 1 = searching right, -1 = searching left
bool tried_other_way = false;  // true = a full circle in one direction found nothing, already flipped

int random_dir = 1;            // 1 = right, -1 = left (RANDOM_TURN)
long random_turn_ms = 0;       // how long RANDOM_TURN lasts
long random_drive_ms = 0;      // how long RANDOM_DRIVE lasts

unsigned long drive_time = 0;  // how long we drove forward (to reverse the same amount)
int nudges = 0;
int nudge_dir = 0;             // 1 = right, -1 = left

void setup() {
  Serial.begin(9600);

  pinMode(A_pinF, OUTPUT);
  pinMode(A_pinR, OUTPUT);
  pinMode(B_pinF, OUTPUT);
  pinMode(B_pinR, OUTPUT);
  pinMode(linePin, INPUT);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(irPinR, INPUT);
  pinMode(irPinL, INPUT);

  for (int i = 0; i < ir_window_size; i++) { ir_window_R[i] = 0; ir_window_L[i] = 0; }

  randomSeed(analogRead(A0));    // floating pin = different random moves each run

  stopMove();
  delay(5000); // wait 5 s before starting for safety

  currentMillis = millis();
  setState(SEARCH_TURN);
}


void loop() {
  currentMillis = millis();
  unsigned long elapsed = currentMillis - state_start;

  // ---------- SENSORS every 60 ms ----------
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    updateSensors();
    if (PLOT) plotData();
  }

  // ---------- SAFETY: line seen when not pushing ----------
  bool line_matters = (state == SEARCH_TURN ||
                       state == APPROACH || state == CHECK_COLOR || state == NUDGE ||
                       state == RANDOM_TURN || state == RANDOM_DRIVE);
  if (line_matters && onBlackLine()) {
    stopMove();
    say("Black line! Escaping.");
    setState(ESCAPE);
    return;
  }

  // ---------- STATE MACHINE ----------
  switch (state) {

    case SEARCH_TURN: {
      int ts = slow_search ? slow_scan_speed : scan_speed;
      if (search_dir == 1) turnRight(ts);
      else turnLeft(ts);

      bool close = front_distance <= detect_range;

      if (ignore_until_clear) {
        // Still facing the rejected black box: keep turning until it's behind us
        if (!close) {
          ignore_until_clear = false;
          say("Passed the black box, searching again.");
          state_start = currentMillis;   // restart the full-circle timer
        }
      }
      else if (close) {
        stopMove();
        tried_other_way = false;
        if (!PLOT) {
          Serial.print("Found something at ");
          Serial.print(front_distance * 100);
          Serial.println(" cm. Driving at it.");
        }
        setState(APPROACH);
      }
      else if (elapsed >= (unsigned long)FULL_TURN_STEPS * turn_time) {
        if (tried_other_way) {
          stopMove();
          startRandomMove();
        } else {
          // Full circle found nothing: reverse the search direction and keep going
          search_dir = -search_dir;
          tried_other_way = true;
          say(search_dir == 1 ? "Nothing found. Searching right." : "Nothing found. Searching left.");
          state_start = currentMillis;
        }
      }
      break;
    }

    case APPROACH:
      moveForward(approach_speed);
      drive_time = elapsed;

      if (front_distance <= stop_dist) {
        stopMove();
        nudges = 0;
        say("At the box. Checking colour.");
        setState(CHECK_COLOR);
      }
      else if (drive_time > max_approach_time) {
        say("Never reached the box. Going back.");
        setState(RETURN_BACK);
      }
      break;

    case CHECK_COLOR:
      stopMove();
      if (elapsed < color_wait) break;

      if (ir_sum_R == ir_window_size && ir_sum_L == ir_window_size) {
        black_in_row = 0;
        slow_search = false;
        say("WHITE! Pushing.");
        setState(PUSH);
      }
      else if (ir_sum_R == 0 && ir_sum_L == 0) {
        say("BLACK. Rejecting.");
        rejectBox();
      }
      else if (nudges < MAX_NUDGES) {
        // Only one side sees it -> box is off-centre, turn toward that side
        nudge_dir = (ir_sum_R > ir_sum_L) ? 1 : -1;
        nudges++;
        say("Unclear, nudging.");
        setState(NUDGE);
      }
      else {
        say("Still unclear. Treating as black.");
        rejectBox();
      }
      break;

    case NUDGE:
      if (nudge_dir == 1) turnRight(scan_speed);
      else turnLeft(scan_speed);
      if (elapsed >= nudge_time) {
        stopMove();
        setState(CHECK_COLOR);
      }
      break;

    case PUSH: {
      bool L = ir_sum_L >= 3;
      bool R = ir_sum_R >= 3;

      if (L && R)  moveForward(push_speed);
      else if (L)  steer(push_speed, push_slow);   // box slid left -> curve left
      else if (R)  steer(push_slow, push_speed);   // box slid right -> curve right
      else         moveForward(push_speed);

      if (onBlackLine()) {
        stopMove();
        say("Line reached. Box is out!");
        setState(BACKOFF);
      }
      else if (!L && !R && front_distance > 0.15) {
        say("Lost the box. Escaping and searching.");
        setState(ESCAPE);
      }
      break;
    }

    case BACKOFF:
      moveBackward(push_speed);
      if (elapsed >= backoff_time) {
        stopMove();
        setState(DONE);
      }
      break;

    case RETURN_BACK:
      moveBackward(approach_speed);
      if (elapsed >= drive_time) {
        stopMove();
        ignore_until_clear = true;   // skip this box while we keep turning
        setState(SEARCH_TURN);
      }
      break;

    case ESCAPE:
      moveBackward(scan_speed);
      if (elapsed >= escape_time) {
        stopMove();
        ignore_until_clear = false;
        setState(SEARCH_TURN);
      }
      break;

    case DONE:
      stopMove();
      break;

    case RANDOM_TURN:
      if (random_dir == 1) turnRight(scan_speed);
      else turnLeft(scan_speed);
      if (elapsed >= random_turn_ms) {
        stopMove();
        setState(RANDOM_DRIVE);
      }
      break;

    case RANDOM_DRIVE: {
      moveForward(approach_speed);
      // after 300 ms the reading is fresh; don't ram anything ahead
      bool blocked = (elapsed > 300 && front_distance < random_drive_stop);
      if (elapsed >= random_drive_ms || blocked) {
        stopMove();
        // run again from the top
        ignore_until_clear = false;
        search_dir = 1;
        tried_other_way = false;
        say("Repositioned. Searching again.");
        setState(SEARCH_TURN);
      }
      break;
    }
  }
}

// Black box: reverse away; after BLACK_LIMIT in a row, search the other way
void rejectBox() {
  black_in_row++;
  slow_search = true;
  if (black_in_row >= BLACK_LIMIT) {
    black_in_row = 0;
    search_dir = -search_dir;
    tried_other_way = false;
    say(search_dir == 1 ? "3 black in a row. Turning right." : "3 black in a row. Turning left.");
  }
  setState(RETURN_BACK);
}

// Pick a random turn (direction and amount) and drive time, then start turning
void startRandomMove() {
  random_dir = random(2) ? 1 : -1;
  long steps = random(RANDOM_TURN_MIN, RANDOM_TURN_MAX + 1);
  random_turn_ms = steps * turn_time;
  random_drive_ms = random(RANDOM_DRIVE_MIN, RANDOM_DRIVE_MAX + 1);
  if (!PLOT) {
    Serial.print("Nothing found. Random turn ");
    Serial.print(random_dir == 1 ? "right " : "left ");
    Serial.print(steps);
    Serial.print(" steps, then drive ");
    Serial.print(random_drive_ms);
    Serial.println(" ms.");
  }
  setState(RANDOM_TURN);
}

// STATE HELPERS
void setState(State s) {
  state = s;
  state_start = currentMillis;
  if (!PLOT) {
    Serial.print("-> ");
    Serial.println(stateName(s));
  }
}

const char* stateName(State s) {
  switch (s) {
    case SEARCH_TURN:    return "SEARCH_TURN";
    case APPROACH:       return "APPROACH";
    case CHECK_COLOR:    return "CHECK_COLOR";
    case NUDGE:          return "NUDGE";
    case PUSH:           return "PUSH";
    case BACKOFF:        return "BACKOFF";
    case RETURN_BACK:    return "RETURN_BACK";
    case ESCAPE:         return "ESCAPE";
    case DONE:           return "DONE";
    case RANDOM_TURN:    return "RANDOM_TURN";
    case RANDOM_DRIVE:   return "RANDOM_DRIVE";
  }
  return "?";
}

void say(const char* msg) {
  if (!PLOT) Serial.println(msg);
}


// SENSORS
void updateSensors() {
  // Ultrasonic: raw reading, no filtering
  front_distance = readFrontSensorData();

  // IR: votes over last 5 (LOW = sees white)
  ir_window_R[ir_window_index] = !digitalRead(irPinR);
  ir_window_L[ir_window_index] = !digitalRead(irPinL);
  ir_window_index = (ir_window_index + 1) % ir_window_size;

  ir_sum_R = 0;
  ir_sum_L = 0;
  for (int i = 0; i < ir_window_size; i++) {
    ir_sum_R += ir_window_R[i];
    ir_sum_L += ir_window_L[i];
  }
}

float readFrontSensorData() {
  digitalWrite(frontTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(frontTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrigPin, LOW);

  float time = pulseIn(frontEchoPin, HIGH, 30000) * 1e-6;
  if (time == 0) return 4.0;     // no echo = nothing there
  return time * 343 * 0.5;       // meters
}

bool onBlackLine() {
  return digitalRead(linePin) == LINE_ON_TAPE;
}

void plotData() {
  float cm = front_distance * 100;
  if (cm > 100) cm = 100;
  Serial.print("distance_cm:");
  Serial.print(cm);
  Serial.print(",range_limit_cm:");
  Serial.print(detect_range * 100);
  Serial.print(",ir_R:");
  Serial.print(ir_sum_R * 4);      // 0-20
  Serial.print(",ir_L:");
  Serial.print(ir_sum_L * 4);
  Serial.print(",state:");
  Serial.println((int)state * 5);
}

// MOTORS
void moveForward(int s) {
  analogWrite(A_pinF, s);
  analogWrite(A_pinR, 0);
  analogWrite(B_pinF, s - 5);
  analogWrite(B_pinR, 0);
}

void moveBackward(int s) {
  analogWrite(A_pinF, 0);
  analogWrite(A_pinR, s);
  analogWrite(B_pinF, 0);
  analogWrite(B_pinR, s - 5);
}

void turnLeft(int s) {
  analogWrite(A_pinF, s);
  analogWrite(A_pinR, 0);
  analogWrite(B_pinF, 0);
  analogWrite(B_pinR, s);
}

void turnRight(int s) {
  analogWrite(A_pinF, 0);
  analogWrite(A_pinR, s);
  analogWrite(B_pinF, s);
  analogWrite(B_pinR, 0);
}

// Gentle curve: right wheel (A) speed, left wheel (B) speed
void steer(int speedRight, int speedLeft) {
  analogWrite(A_pinF, speedRight);
  analogWrite(A_pinR, 0);
  analogWrite(B_pinF, speedLeft);
  analogWrite(B_pinR, 0);
}

void stopMove() {
  analogWrite(A_pinF, 0);
  analogWrite(A_pinR, 0);
  analogWrite(B_pinF, 0);
  analogWrite(B_pinR, 0);
}
