/*
 * robot.c - finite state machine for training (mapping the maze) and solving it.
 *
 *   IDLE ──TRAIN/SOLVE──> LEAVE_START ──line found──> FOLLOW
 *   FOLLOW ──branch seen──> NODE ──L/R──> TURN ──────> FOLLOW
 *                             ├──S─────────────────> FOLLOW
 *                             ├──B──> UTURN ───────> FOLLOW
 *                             ├──circle──> EXIT_CIRCLE ──> UTURN   (training: back out of start/end)
 *                             └──done / end reached──> FINISHED ──> IDLE
 *   FOLLOW ──line ended (small error)──> UTURN       (dead end)
 *   FOLLOW ──line lost (big error)─────> RECOVER ──> FOLLOW
 *   TURN / UTURN failed ───────────────> RECOVER
 *   any state ──STOP──> IDLE
 *
 * TRAIN: at every node the map in maze.c decides where to go (depth-first
 *        exploration) until every branch is explored, then computes the shortest path.
 * SOLVE: replays the shortest path, one move per node, and stops at the end circle.
 * RETURN: after training, drives back to the start circle the same way (one planned
 *        move per node), turns round inside it and waits for SOLVE.
 * FORGET (command): erases the saved path and map, e.g. when the maze changes.
 * The start is whichever circle the car is in when TRAIN is sent.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "util.h"
#include "motors.h"
#include "sensors.h"
#include "maze.h"
#include "ble_link.h"
#include "robot.h"

typedef enum {
  ST_IDLE,
  ST_LEAVE_START,
  ST_FOLLOW,
  ST_NODE,
  ST_TURN,
  ST_UTURN,
  ST_EXIT_CIRCLE,
  ST_RECOVER,
  ST_FINISHED,
} robot_state_t;

static const char *STATE_NAMES[] = {
  "IDLE", "LEAVE_START", "FOLLOW", "NODE", "TURN", "UTURN", "EXIT_CIRCLE", "RECOVER", "FINISHED"
};

typedef enum { MODE_TRAIN, MODE_SOLVE, MODE_RETURN } run_mode_t;
static const char *MODE_NAMES[] = { "TRAIN", "SOLVE", "RETURN" };

// ---------- state ----------

static robot_state_t state = ST_IDLE;
static bool stateFirst = true;         // true on the first pass through a new state
static run_mode_t mode = MODE_TRAIN;

static bool readyForNode = false;
static bool turnCompleted = false;
static uint32_t lastNodeTime = 0;
static uint32_t lastTurnTime = 0;
static uint32_t lineLostTime = 0;
static uint32_t runStartTime = 0;

static int lastTurnDirection = 1;
static float lastError = 0;

// distance since the last node = time spent actually driving along the line
static uint32_t segMs = 0;
static uint32_t lastDriveTick = 0;

static uint8_t nodeFirstReading = 0;   // reading that triggered NODE
static int turnDir = 1;                // TURN / UTURN: 1 right, -1 left
static int turnSkip = 0;               // UTURN: lines to pass before stopping
static bool uturnFromDeadEnd = false;  // UTURN: was it a dead end (or a missed corner)?

static int solveStep = 0;              // solving / returning: index into the planned moves
static bool lastLeft = false, lastRight = false;   // branches seen at the last node
static bool finishedByUturn = false;   // training ended right after a dead-end U-turn
static int extraMoves = 0;             // moves made off-plan while solving

#ifndef SOLVE_EXTRA_MOVES
#define SOLVE_EXTRA_MOVES 2            // off-plan moves allowed in SOLVE before stopping (none in RETURN)
#endif
static uint32_t avgTurn90 = TURN90_GUESS_MS;

// PID
static float integral = 0;
static float prevError = 0;
static float iTerm = 0, dTerm = 0;
static uint32_t lastPidTime = 0;

// RECOVER
static bool driftLoss = false;
static bool waitingRetry = false;
static uint32_t searchStart = 0, centerStart = 0, failedAt = 0;
static int searchTries = 0;

// EXIT_CIRCLE
static bool backOnLine = false;
static uint32_t phaseStart = 0;

// ---------- helpers ----------

static void arrivedHome(void);

static void enter(robot_state_t next) {

  if (next == state)
    return;

  state = next;
  stateFirst = true;

  // NODE/TURN/FOLLOW happen at every intersection and have their own messages
  if (next != ST_FOLLOW && next != ST_NODE && next != ST_TURN)
    link_send("STATE %s", STATE_NAMES[next]);
}

static void pidReset(void) {
  integral = 0;
  prevError = 0;
  iTerm = 0;
  dTerm = 0;
  lastPidTime = millis();
}

// called while driving along the line: adds the time to the current segment
static void countDriving(void) {
  uint32_t now = millis();
  if (now - lastDriveTick < 20)        // ignore gaps (stops, searches)
    segMs += now - lastDriveTick;
  lastDriveTick = now;
}

static void afterManeuver(void) {
  readyForNode = false;
  lastNodeTime = millis();
  lineLostTime = 0;
  lastError = 0;
  segMs = 0;
  lastDriveTick = 0;
  pidReset();
}

// spin until the center sensor sits on a line, passing `skip` lines first
// (phase 0 turn90, extended so a U-turn at an intersection doesn't stop on a side branch)
static bool spinToLine(int dir, int skip, uint32_t maxTime, uint32_t *elapsed) {

  bool onLine = readLine() & CENTER_MASK;   // starting on the old line?
  bool leftOldLine = !onLine;
  uint32_t whiteSince = 0;
  int lines = 0;

  stopCar();
  delay_ms(60);

  // small push to start the spin
  spin(dir, KICK_SPEED);
  delay_ms(KICK_TIME);

  spin(dir, TURN_SPEED);

  uint32_t start = millis();
  uint32_t stableTime = 0;

  while (millis() - start < maxTime) {

    if (link_stop_requested()) {
      stopCar();
      return false;
    }

    bool center = readLine() & CENTER_MASK;
    uint32_t now = millis();

    if (!center) {
      if (onLine || whiteSince == 0)
        whiteSince = now;
      onLine = false;
      leftOldLine = true;
      stableTime = 0;
      continue;
    }

    // center on black: a new line if we were off a line long enough,
    // otherwise just a flicker on the same line, so restart the stable timer (phase 0 behaviour)
    if (!onLine && leftOldLine && now - whiteSince >= LINE_GAP_MS) {
      lines++;
      stableTime = now;
    } else if (stableTime == 0) {
      stableTime = now;
    }
    onLine = true;

    if (lines == skip + 1 && stableTime != 0 &&
        now - start > MIN_TURN_TIME &&
        now - stableTime > TURN_STABLE_TIME) {

      *elapsed = now - start;
      brakeCar();
      delay_ms(70);

      lastTurnTime = millis();
      turnCompleted = true;
      return true;
    }
  }

  stopCar();
  return false;
}

static bool moveAvailable(char m, bool left, bool straight, bool right) {
  return (m == 'L' && left) || (m == 'S' && straight) || (m == 'R' && right) || m == 'B';
}

// right-hand rule, only used when solving goes off-plan
static char handRule(bool left, bool straight, bool right) {
  if (right)    return 'R';
  if (straight) return 'S';
  if (left)     return 'L';
  return 'B';
}

static void startRun(run_mode_t m) {

  mode = m;
  solveStep = 0;
  searchTries = 0;
  turnCompleted = false;
  lastTurnDirection = 1;
  finishedByUturn = false;
  extraMoves = 0;

  if (m == MODE_TRAIN) {
    maze_reset();
    link_send("RUN TRAIN");
  } else {
    link_send("RUN SOLVE %d %s", maze_best_len(), maze_best());
  }

  delay_ms(START_DELAY);

  afterManeuver();
  runStartTime = millis();
  enter(ST_LEAVE_START);
}

static void sendStatus(void) {
  link_send("STATUS %s %s path=%s",
            STATE_NAMES[state],
            MODE_NAMES[mode],
            maze_best_len() > 0 ? maze_best() : "-");
}

static void handleCommands(void) {

  robot_cmd_t cmd;

  while (link_get_command(&cmd)) {

    switch (cmd) {

      case CMD_TRAIN:
        if (state != ST_IDLE) link_send("ERR busy, send STOP first");
        else startRun(MODE_TRAIN);
        break;

      case CMD_SOLVE:
        if (state != ST_IDLE) link_send("ERR busy, send STOP first");
        else if (maze_best_len() == 0) link_send("ERR no path yet, run TRAIN first");
        else startRun(MODE_SOLVE);
        break;

      case CMD_PATH:
        link_send("PATH RAW %s", maze_raw());
        link_send("PATH BEST %s", maze_best());
        if (state == ST_IDLE)
          maze_send_map();
        break;

      case CMD_STATUS:
        sendStatus();
        break;

      case CMD_FORGET:
        if (state != ST_IDLE) link_send("ERR busy, send STOP first");
        else {
          maze_forget();
          link_send("FORGOTTEN: saved path and map erased, send TRAIN");
        }
        break;
    }
  }
}

// ---------- line following (PID) ----------

static void followLine(uint8_t s) {

  float error;
  if (!lineError(s, &error))
    return;

  lastError = error;

  if (error < -0.3f)
    lastTurnDirection = -1;
  else if (error > 0.3f)
    lastTurnDirection = 1;

  // I and D at a fixed rate, P every pass (same response as phase 0 when KI = KD = 0)
  uint32_t now = millis();
  if (now - lastPidTime >= PID_PERIOD_MS) {

    float dt = (now - lastPidTime) / 1000.0f;
    if (dt > 0.1f) dt = PID_PERIOD_MS / 1000.0f;   // after a pause, don't spike

    integral += error * dt;
    if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
    if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

    iTerm = KI * integral;
    dTerm = KD * (error - prevError) / dt;

    prevError = error;
    lastPidTime = now;
  }

  int correction = (int)(KP * error + iTerm + dTerm);

  int base = NORMAL_SPEED;
  int maxSpeed = FOLLOW_MAX_SPEED;
  if (mode == MODE_SOLVE) {
    base += SOLVE_SPEED_BOOST;
    maxSpeed += SOLVE_SPEED_BOOST;
  }

  drive(constrain(base + correction, 0, maxSpeed),
        constrain(base - correction, 0, maxSpeed));

  countDriving();
}

// ---------- states ----------

static void stIdle(void) {
  if (stateFirst) {
    stateFirst = false;
    stopCar();
  }
}

// drive straight out of the start circle until a normal line is under the car
static void stLeaveStart(void) {

  uint8_t s = readLine();

  // out of the circle and some line under the car (doesn't have to be centered,
  // the line follower will center it)
  if (s != 0 && s != ALL_BLACK && !leftBranch(s) && !rightBranch(s)) {
    readyForNode = normalLine(s);
    lastNodeTime = millis();
    pidReset();
    enter(ST_FOLLOW);
    return;
  }

  drive(SCAN_SPEED, SCAN_SPEED);
  countDriving();

  if (millis() - runStartTime > START_TIMEOUT) {
    stopCar();
    link_send("ERR could not find the line leaving the start");
    enter(ST_IDLE);
  }
}

static void stFollow(void) {

  uint8_t s = readLine();

  // no black under the car
  if (s == 0) {

    stopCar();

    // ignore it right after a turn
    if (turnCompleted && millis() - lastTurnTime < TURN_PROTECTION_TIME) {
      lineLostTime = 0;
      return;
    }

    if (lineLostTime == 0)
      lineLostTime = millis();

    if (millis() - lineLostTime >= LOST_TIME) {

      lineLostTime = 0;
      readyForNode = false;

      if (fabsf(lastError) >= DRIFT_ERR) {
        enter(ST_RECOVER);             // slid off the side of the line
      } else {
        turnDir = lastTurnDirection;   // line ended in front of us
        turnSkip = 0;
        uturnFromDeadEnd = true;
        enter(ST_UTURN);
      }
    }
    return;
  }

  lineLostTime = 0;

  // normal line seen, so we are ready for the next intersection
  if (normalLine(s))
    readyForNode = true;

  if (readyForNode &&
      millis() - lastNodeTime > COOLDOWN_TIME &&
      (leftBranch(s) || rightBranch(s))) {
    nodeFirstReading = s;
    enter(ST_NODE);
    return;
  }

  followLine(s);
}

// the list of moves being replayed: the shortest path (SOLVE) or the way home (RETURN)
static const char *plan(void) { return mode == MODE_RETURN ? maze_home() : maze_best(); }
static int planLen(void)      { return mode == MODE_RETURN ? maze_home_len() : maze_best_len(); }
static const char *planTag(void) { return mode == MODE_RETURN ? "HOME" : "STEP"; }

// solving / returning: pick the next planned move at this node
static char solveMove(bool left, bool straight, bool right) {

  int total = planLen();

  if (solveStep >= total) {
    // the plan is finished but we're not in the circle: the map doesn't match the track.
    // Don't wander around the maze: allow a couple of moves while solving, then stop.
    if (mode == MODE_RETURN || ++extraMoves > SOLVE_EXTRA_MOVES)
      return 'X';
    char m = handRule(left, straight, right);
    link_send("WARN path used up before the end, taking %c", m);
    return m;
  }

  char planned = plan()[solveStep];

  if (moveAvailable(planned, left, straight, right)) {
    solveStep++;
    link_send("%s %d/%d %c", planTag(), solveStep, total, planned);
    return planned;
  }

  if (straight) {
    // probably a false intersection, don't use up a step
    link_send("WARN unexpected node L%d S%d R%d, going straight", left, straight, right);
    return 'S';
  }

  if (mode == MODE_RETURN || ++extraMoves > SOLVE_EXTRA_MOVES)
    return 'X';
  char m = handRule(left, straight, right);
  solveStep++;
  link_send("WARN planned %c not possible, taking %c", planned, m);
  return m;
}

// look at the intersection, then let the map (training) or the best path (solving) decide
static void stNode(void) {

  stateFirst = false;

  // read again to make sure it is real
  uint8_t first = nodeFirstReading;
  delay_ms(NODE_CONFIRM_TIME);
  uint8_t again = readLine();

  bool left = leftBranch(first) && leftBranch(again);
  bool right = rightBranch(first) && rightBranch(again);

  if (!left && !right) {
    enter(ST_FOLLOW);                  // noise
    return;
  }

  // move forward slowly and collect every branch
  drive(SCAN_SPEED, SCAN_SPEED);
  uint32_t scanStart = millis();

  while (millis() - scanStart < NODE_TIME) {

    if (link_stop_requested())
      return;

    uint8_t r = readLine();
    if (leftBranch(r)) left = true;
    if (rightBranch(r)) right = true;
  }

  // move a bit more so the wheels are in the middle of the intersection
  delay_ms(PIVOT_CREEP_TIME);
  uint8_t after = readLine();

  // still all black: either a 20 cm circle or we haven't cleared the cross line yet
  bool circle = false;
  if (after == ALL_BLACK) {
    delay_ms(END_CONFIRM_TIME);
    after = readLine();
    circle = (after == ALL_BLACK);
  }

  brakeCar();

  // the sensors are past the cross line now: black in the middle = the line goes on
  bool straight = !circle && (after & MIDDLE_MASK) != 0;
  char move;
  lastLeft = left;
  lastRight = right;

  if (mode == MODE_SOLVE || mode == MODE_RETURN) {

    if (circle) {
      if (mode == MODE_SOLVE) enter(ST_FINISHED);
      else arrivedHome();
      return;
    }
    move = solveMove(left, straight, right);

    if (move == 'X') {
      stopCar();
      if (mode == MODE_RETURN)
        link_send("ERR lost on the way back to the start: put the car in the start circle by hand, then SOLVE");
      else
        link_send("ERR the saved path does not match the track: stopping. Check the car is in the training start circle, or FORGET and TRAIN again");
      enter(ST_IDLE);
      return;
    }

  } else {

    uint8_t rel = 0;
    if (left)     rel |= 1 << REL_L;
    if (straight) rel |= 1 << REL_S;
    if (right)    rel |= 1 << REL_R;

    maze_arrive(rel, circle, segMs);

    // safety net: never explore forever (e.g. going round a loop the map didn't recognise)
    if (maze_goal_found() && millis() - runStartTime > TRAIN_MAX_TIME) {
      link_send("WARN training time limit reached, using the map built so far");
      move = 'D';
    } else {
      move = maze_next_move();
    }

    if (move == 'D') {
      enter(ST_FINISHED);
      return;
    }

    if (circle) {                      // start or end circle: back out, then turn around
      afterManeuver();
      enter(ST_EXIT_CIRCLE);
      return;
    }
  }

  afterManeuver();

  if (move == 'S') {
    enter(ST_FOLLOW);

  } else if (move == 'B') {
    // spin the way with no branch in the way, or pass one branch
    if (!right)     { turnDir = 1;  turnSkip = 0; }
    else if (!left) { turnDir = -1; turnSkip = 0; }
    else            { turnDir = 1;  turnSkip = 1; }
    uturnFromDeadEnd = false;
    enter(ST_UTURN);

  } else {
    turnDir = (move == 'R') ? 1 : -1;
    enter(ST_TURN);
  }
}

static void stTurn(void) {

  stateFirst = false;
  uint32_t elapsed = 0;

  if (spinToLine(turnDir, 0, MAX_TURN_TIME, &elapsed)) {
    // learn how long a 90 degree spin takes (used to tell corners from dead ends)
    avgTurn90 = (avgTurn90 * 3 + elapsed) / 4;
    afterManeuver();
    enter(ST_FOLLOW);
  } else if (!link_stop_requested()) {
    link_send("WARN turn failed");
    lastError = 0;
    enter(ST_RECOVER);
  }
}

static void stUturn(void) {

  stateFirst = false;
  uint32_t seg = segMs;                // distance to this dead end
  uint32_t elapsed = 0;

  if (!spinToLine(turnDir, turnSkip, UTURN_MAX_TIME, &elapsed)) {
    if (!link_stop_requested()) {
      link_send("WARN u-turn failed");
      lastError = 0;
      enter(ST_RECOVER);
    }
    return;
  }

  afterManeuver();

  if (!uturnFromDeadEnd) {
    enter(ST_FOLLOW);
    return;
  }

  // a short spin means we overshot a corner and just turned into it
  bool corner = elapsed < avgTurn90 * CORNER_RATIO;
  char move = corner ? (turnDir > 0 ? 'R' : 'L') : 'B';

  if (mode == MODE_TRAIN) {

    if (corner) {
      maze_arrive(1 << (turnDir > 0 ? REL_R : REL_L), false, seg);
      maze_force_move(move);
    } else {
      maze_arrive(0, false, seg);      // dead end: only way out is back
      if (maze_next_move() == 'D') {   // that was the last unexplored branch
        finishedByUturn = true;        // the car already faces back out
        enter(ST_FINISHED);
        return;
      }
    }

  } else if (corner && solveStep < planLen() && plan()[solveStep] == move) {
    // solving: a missed corner that matches the plan still counts as that step
    solveStep++;
    link_send("%s %d/%d %c", planTag(), solveStep, planLen(), move);
  } else {
    link_send("WARN unexpected %s while solving", corner ? "corner" : "dead end");
  }

  enter(ST_FOLLOW);
}

// back out of a start/end circle onto its line, then turn around on that line
static void stExitCircle(void) {

  if (stateFirst) {
    stateFirst = false;
    backOnLine = false;
    phaseStart = millis();
    drive(-SCAN_SPEED, -SCAN_SPEED);
  }

  if (link_stop_requested())
    return;

  if (!backOnLine) {
    if (normalLine(readLine())) {
      backOnLine = true;
      phaseStart = millis();
    } else if (millis() - phaseStart > CIRCLE_REVERSE_TIMEOUT) {
      stopCar();
      link_send("WARN could not back out of the circle");
      enter(ST_RECOVER);
    }
    return;
  }

  if (millis() - phaseStart >= CIRCLE_REVERSE_EXTRA) {
    brakeCar();
    turnDir = 1;
    turnSkip = 0;
    uturnFromDeadEnd = false;
    enter(ST_UTURN);
  }
}

// phase 0 recoverLine(): sweep if we drifted, full spin otherwise
static void stRecover(void) {

  if (stateFirst) {
    stateFirst = false;
    searchStart = millis();
    centerStart = 0;
    waitingRetry = false;
    readyForNode = false;
    driftLoss = fabsf(lastError) >= DRIFT_ERR;
    link_send(driftLoss ? "LOST drifted off -> small sweeps" : "LOST -> full spin");
    stopCar();
  }

  // wait after a failed search then try again
  if (waitingRetry) {
    stopCar();
    if (readLine() != 0) {
      lastNodeTime = millis();
      enter(ST_FOLLOW);
    } else if (millis() - failedAt > RETRY_AFTER) {
      waitingRetry = false;
      searchStart = millis();
    }
    return;
  }

  bool center = readLine() & CENTER_MASK;

  if (center) {

    if (centerStart == 0) {
      brakeCar();
      centerStart = millis();
    }
    stopCar();

    if (millis() - centerStart >= ALIGN_TIME) {
      // keep segMs: the distance to the next node continues from here
      searchTries = 0;
      readyForNode = false;
      lastNodeTime = millis();
      lineLostTime = 0;
      pidReset();
      link_send("FOUND line");
      enter(ST_FOLLOW);
    }
    return;
  }

  centerStart = 0;

  int dir = lastTurnDirection;

  // if we only drifted, sweep one way then the other
  if (driftLoss) {
    uint32_t t = millis() - searchStart;
    if (t >= 300 && t < 900)
      dir = -lastTurnDirection;
  }

  spin(dir, SEARCH_SPEED);

  // give up after max time, retry later
  if (millis() - searchStart >= MAX_SEARCH_TIME) {

    stopCar();
    searchTries++;

    if (searchTries >= MAX_SEARCH_TRIES) {
      searchTries = 0;
      link_send("ERR line lost, stopping");
      enter(ST_IDLE);
      return;
    }

    link_send("WARN search failed, retry %d", searchTries);
    waitingRetry = true;
    failedAt = millis();
  }
}

// we drove into the start circle at the end of RETURN: turn round on the spot so the
// car faces out of the circle again, ready for SOLVE (a timed 180, using the learned 90 degree time)
static void arrivedHome(void) {
  brakeCar();
  delay_ms(100);
  spin(1, TURN_SPEED);
  delay_ms(2 * avgTurn90);
  brakeCar();
  stopCar();
  mode = MODE_SOLVE;
  link_send("BACK AT START: ready, send SOLVE");
  enter(ST_IDLE);
}

// after training: drive back to the start circle (first move from maze_plan_home)
static void startReturn(char first) {

  mode = MODE_RETURN;
  solveStep = 0;
  extraMoves = 0;
  afterManeuver();

  if (first == 'H') {
    arrivedHome();
    return;
  }
  if (first == 0) {
    link_send("WARN no route back to the start, put the car in the start circle by hand");
    enter(ST_IDLE);
    return;
  }

  link_send("RETURN to start: first %c, then %d moves %s", first, maze_home_len(), maze_home());

  if (first == 'C') {
    enter(ST_EXIT_CIRCLE);             // back out of the end circle, turn round, follow
  } else if (first == 'S') {
    enter(ST_FOLLOW);
  } else if (first == 'B') {
    if (!lastRight)     { turnDir = 1;  turnSkip = 0; }
    else if (!lastLeft) { turnDir = -1; turnSkip = 0; }
    else                { turnDir = 1;  turnSkip = 1; }
    uturnFromDeadEnd = false;
    enter(ST_UTURN);
  } else {
    turnDir = (first == 'R') ? 1 : -1;
    enter(ST_TURN);
  }
}

static void stFinished(void) {

  stopCar();
  uint32_t runTime = millis() - runStartTime;

  if (mode == MODE_TRAIN) {

    bool ok = false;

    if (!maze_goal_found()) {
      link_send("ERR explored everything but never found the end circle");
    } else if (!maze_compute_best()) {
      link_send("ERR no route from start to end in the map");
    } else {
      bool saved = maze_save();
      link_send("PATH RAW %s", maze_raw());
      link_send("PATH BEST %s", maze_best());
      link_send("TRAIN_DONE time=%lu ms best=%d moves%s",
                (unsigned long)runTime, maze_best_len(),
                saved ? "" : " (not saved to flash)");
      link_send(saved ? "MAZE COORDINATES STORED" : "WARN maze not saved to flash");
      ok = true;
    }
    maze_send_map();

    if (ok) {
      startReturn(maze_plan_home(finishedByUturn));
      return;
    }

  } else {
    link_send("SOLVE_DONE time=%lu ms steps=%d/%d",
              (unsigned long)runTime, solveStep, maze_best_len());
    link_send("MAZE SOLVED: reached the end circle (%d/%d planned moves)", solveStep, maze_best_len());
  }

  enter(ST_IDLE);
}

// ---------- wiring tests (TEST_MODE in config.h) ----------

static void sensorTest(void) {
  while (1) {
    uint8_t s = readLine();
    printf("L ");
    for (int i = 0; i < 5; i++)
      printf("%d ", (s >> i) & 1);
    printf("R\n");
    delay_ms(200);
  }
}

static void motorTest(void) {
  while (1) {
    printf("LEFT wheels forward\n");
    drive(130, 0);
    delay_ms(2000);
    stopCar();
    delay_ms(1000);

    printf("RIGHT wheels forward\n");
    drive(0, 130);
    delay_ms(2000);
    stopCar();
    delay_ms(1000);
  }
}

// ---------- task ----------

void robot_task(void *arg) {

  motors_init();
  sensors_init();

  if (TEST_MODE == 1) sensorTest();
  if (TEST_MODE == 2) motorTest();

  if (maze_load())
    link_send("READY stored path %s", maze_best());
  else
    link_send("READY no stored path");

  while (1) {

    if (link_stop_requested()) {
      link_clear_stop();
      stopCar();
      enter(ST_IDLE);
      link_send("STOPPED");
    }

    handleCommands();

    switch (state) {
      case ST_IDLE:        stIdle();       break;
      case ST_LEAVE_START: stLeaveStart(); break;
      case ST_FOLLOW:      stFollow();     break;
      case ST_NODE:        stNode();       break;
      case ST_TURN:        stTurn();       break;
      case ST_UTURN:       stUturn();      break;
      case ST_EXIT_CIRCLE: stExitCircle(); break;
      case ST_RECOVER:     stRecover();    break;
      case ST_FINISHED:    stFinished();   break;
    }

    vTaskDelay(1);   // 1 ms, lets the idle task run
  }
}
