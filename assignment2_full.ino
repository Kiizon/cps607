// FULL ROBOT: turn -> detect -> go back to closest -> approach -> color -> push/reject

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
const long turn_time = 75;     // ms per ~10 deg step turning RIGHT (calibrated)
const long turn_back_time = 75;// ms per ~10 deg step turning LEFT (calibrate separately if needed)
const long settle_time = 400;  // ms stopped before saving a reading
const long back_pause = 200;   // ms stopped between steps when turning back
const int FULL_TURN_STEPS = 36;// steps in a full circle

// Detection
const float max_range = 0.55;  // m, closer than this = object
const float rise_margin = 0.10;// m, distance rises this much above the minimum = we passed it
const int HIST_SIZE = 10;      // how many recent step readings we remember

// Approach
const int approach_speed = 110;
const float stop_dist = 0.10;  // m, MEASURE THIS: ultrasonic reading when IR sensors are ~4 cm from the box
const long max_approach_time = 4000; // ms, give up if the box is never reached

// Color check
const long color_wait = 500;   // ms stopped so all 5 IR readings are fresh
const long nudge_time = 40;    // ms small turn to re-centre on the box
const int MAX_NUDGES = 3;

// Push
const int push_speed = 160;
const int push_slow = 120;     // slower wheel when steering (above minimum PWM)
const long backoff_time = 800; // ms reversing after the box is out

// Escape (line seen when not pushing)
const long escape_time = 500;  // ms reversing

// sensor variables
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 60;

const int front_window_size = 5;
float front_window[front_window_size];
int front_window_index = 0;
float front_filtered_distance = 4.0;   // m

const int ir_window_size = 5;
int ir_window_R[ir_window_size];
int ir_window_L[ir_window_size];
int ir_window_index = 0;
int ir_sum_R = 0;
int ir_sum_L = 0;

// history of readings
float hist[HIST_SIZE];
int hist_index = 0;            // where the next reading goes
int hist_count = 0;            // how many readings are stored

// state machine
enum State {
  SEARCH_TURN,     // turning one step right
  SEARCH_MEASURE,  // stopped, saving a reading, deciding
  TURN_BACK,       // turning left back to the closest reading
  APPROACH,        // driving to the box
  CHECK_COLOR,     // stopped, reading IR
  NUDGE,           // small turn to centre the box
  PUSH,            // pushing white box to the line
  BACKOFF,         // reversing after the box is out
  RETURN_BACK,     // reversing away from a black box
  ESCAPE,          // reversing away from the line
  DONE,
  GIVE_UP
};

State state = SEARCH_MEASURE;
unsigned long state_start = 0;

bool in_object = false;        // currently passing an object
float obj_min = 100;           // closest distance of the object being passed
bool ignore_until_clear = false; // true = still looking at the rejected black box
int steps_without_object = 0;

int steps_left = 0;            // steps still to turn back
bool turn_phase = true;        // TURN_BACK: true = turning, false = pausing

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

  for (int i = 0; i < front_window_size; i++) front_window[i] = 4.0;
  for (int i = 0; i < ir_window_size; i++) { ir_window_R[i] = 0; ir_window_L[i] = 0; }

  stopMove();
  delay(5000); // wait 5 s before starting for safety

  currentMillis = millis();
  setState(SEARCH_MEASURE);
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
  bool line_matters = (state == SEARCH_TURN || state == SEARCH_MEASURE || state == TURN_BACK ||
                       state == APPROACH || state == CHECK_COLOR || state == NUDGE);
  if (line_matters && onBlackLine()) {
    stopMove();
    say("Black line! Escaping.");
    setState(ESCAPE);
    return;
  }

  // ---------- STATE MACHINE ----------
  switch (state) {

    case SEARCH_TURN:
      turnRight(scan_speed);
      if (elapsed >= turn_time) {
        stopMove();
        setState(SEARCH_MEASURE);
      }
      break;

    case SEARCH_MEASURE: {
      stopMove();
      if (elapsed < settle_time) break;

      float d = front_filtered_distance;
      addHistory(d);
      steps_without_object++;
      bool close = d < max_range;

      if (!PLOT) {
        Serial.print("step: ");
        if (d >= 4.0) Serial.println("nothing");
        else { Serial.print(d * 100); Serial.println(" cm"); }
      }

      if (ignore_until_clear) {
        // Still facing the rejected black box: wait until it's behind us
        if (!close) {
          ignore_until_clear = false;
          say("Passed the black box, searching again.");
        }
        setState(SEARCH_TURN);
      }
      else if (in_object && (!close || d > obj_min + rise_margin)) {
        // Distance went back up -> we passed the object
        float min_d;
        int ago = stepsAgoOfMin(min_d);
        in_object = false;
        obj_min = 100;

        if (!PLOT) {
          Serial.print("Object passed. Closest: ");
          Serial.print(min_d * 100);
          Serial.print(" cm, ");
          Serial.print(ago);
          Serial.println(" steps ago. Turning back.");
        }

        if (ago == 0) {
          setState(APPROACH);
        } else {
          steps_left = ago;
          turn_phase = true;
          setState(TURN_BACK);
        }
      }
      else {
        if (close) {
          in_object = true;
          if (d < obj_min) obj_min = d;
          steps_without_object = 0;
        }
        if (steps_without_object > FULL_TURN_STEPS) {
          setState(GIVE_UP);
        } else {
          setState(SEARCH_TURN);
        }
      }
      break;
    }

    case TURN_BACK:
      if (turn_phase) {
        turnLeft(scan_speed);
        if (elapsed >= turn_back_time) {
          stopMove();
          steps_left--;
          turn_phase = false;
          state_start = currentMillis;    // restart timer for the pause
        }
      } else {
        stopMove();
        if (elapsed >= back_pause) {
          if (steps_left <= 0) {
            setState(APPROACH);
          } else {
            turn_phase = true;
            state_start = currentMillis;  // restart timer for the next turn
          }
        }
      }
      break;

    case APPROACH:
      // First 300 ms: stay still so the distance filter refreshes
      if (elapsed < 300) { stopMove(); break; }

      moveForward(approach_speed);
      drive_time = elapsed - 300;

      if (front_filtered_distance <= stop_dist) {
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
        say("WHITE! Pushing.");
        setState(PUSH);
      }
      else if (ir_sum_R == 0 && ir_sum_L == 0) {
        say("BLACK. Rejecting.");
        setState(RETURN_BACK);
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
        setState(RETURN_BACK);
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
      else if (!L && !R && front_filtered_distance > 0.15) {
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
        clearHistory();
        in_object = false;
        obj_min = 100;
        ignore_until_clear = true;   // skip this box while we keep turning
        steps_without_object = 0;
        setState(SEARCH_MEASURE);
      }
      break;

    case ESCAPE:
      moveBackward(scan_speed);
      if (elapsed >= escape_time) {
        stopMove();
        clearHistory();
        in_object = false;
        obj_min = 100;
        ignore_until_clear = false;
        steps_without_object = 0;
        setState(SEARCH_MEASURE);
      }
      break;

    case DONE:
      stopMove();
      break;

    case GIVE_UP:
      stopMove();
      break;
  }
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
    case SEARCH_MEASURE: return "SEARCH_MEASURE";
    case TURN_BACK:      return "TURN_BACK";
    case APPROACH:       return "APPROACH";
    case CHECK_COLOR:    return "CHECK_COLOR";
    case NUDGE:          return "NUDGE";
    case PUSH:           return "PUSH";
    case BACKOFF:        return "BACKOFF";
    case RETURN_BACK:    return "RETURN_BACK";
    case ESCAPE:         return "ESCAPE";
    case DONE:           return "DONE";
    case GIVE_UP:        return "GIVE_UP (nothing found in a full turn)";
  }
  return "?";
}

void say(const char* msg) {
  if (!PLOT) Serial.println(msg);
}


// HISTORY: last 10 step readings
void addHistory(float d) {
  hist[hist_index] = d;
  hist_index = (hist_index + 1) % HIST_SIZE;
  if (hist_count < HIST_SIZE) hist_count++;
}

void clearHistory() {
  hist_index = 0;
  hist_count = 0;
}

// Finds the smallest reading in the history.
// Returns how many steps ago it was (0 = the reading we just took).
int stepsAgoOfMin(float &min_d) {
  min_d = 100;
  int ago = 0;
  for (int a = 0; a < hist_count; a++) {
    int idx = (hist_index - 1 - a + HIST_SIZE) % HIST_SIZE;
    if (hist[idx] < min_d) {
      min_d = hist[idx];
      ago = a;
    }
  }
  return ago;
}

// SENSORS
void updateSensors() {
  // Ultrasonic: median of last 5
  front_window[front_window_index] = readFrontSensorData();
  front_window_index = (front_window_index + 1) % front_window_size;

  float sorted[front_window_size];
  for (int i = 0; i < front_window_size; i++) sorted[i] = front_window[i];
  for (int a = 1; a < front_window_size; a++) {
    float key = sorted[a];
    int b = a - 1;
    while (b >= 0 && sorted[b] > key) { sorted[b + 1] = sorted[b]; b--; }
    sorted[b + 1] = key;
  }
  front_filtered_distance = sorted[front_window_size / 2];

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
  float cm = front_filtered_distance * 100;
  if (cm > 100) cm = 100;
  Serial.print("distance_cm:");
  Serial.print(cm);
  Serial.print(",range_limit_cm:");
  Serial.print(max_range * 100);
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
  analogWrite(B_pinF, s - 15);
  analogWrite(B_pinR, 0);
}

void moveBackward(int s) {
  analogWrite(A_pinF, 0);
  analogWrite(A_pinR, s);
  analogWrite(B_pinF, 0);
  analogWrite(B_pinR, s);
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
