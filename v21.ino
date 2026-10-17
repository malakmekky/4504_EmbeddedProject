/*
Embedded project
Malak Mekky
Malak Taha
Jannah Hella
*/

// h bridge bits
#define ENA 5
#define IN1 22
#define IN2 23
#define IN3 24
#define IN4 25
#define ENB 6

//sensor pinout
const byte sensors[5] = {8,9,10,11,12};   

// speeds
const int normalSpeed = 120;
const int turnSpeed = 110;        // was 90: spinning on the spot needs more torque (weaker battery stalled it)
const int scanSpeed = 90;
const int kickSpeed = 150;        // short burst at the start of a spin to break static friction
const unsigned long kickTime = 40;

// recovery speed
const int searchSpeed = 100;      // was 80, same reason as turnSpeed
const unsigned long lostTime = 120;
const unsigned long maxSearchTime = 6000;
const unsigned long alignTime = 30;
const unsigned long turnProtectionTime = 350;
const unsigned long retryAfter = 1500;   // after a failed search, wait this long then try again

// turn 
const unsigned long nodeTime = 130;
const unsigned long minTurnTime = 180;
const unsigned long maxTurnTime = 2200;
const unsigned long cooldownTime = 300;

// intersection
const unsigned long nodeConfirmTime = 15;

// line following
const float kp = 15.0;

// state
bool readyForNode = false;
bool isSearching = false;
bool searchFailed = false;
bool turnCompleted = false;

unsigned long lastNodeTime = 0;
unsigned long lastTurnTime = 0;
unsigned long lineLostTime = 0;
unsigned long searchStartTime = 0;
unsigned long centerStartTime = 0;
unsigned long failedAt = 0;

int lastTurnDirection = 1;

// motor control
void drive(int left, int right) {

  digitalWrite(IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left >= 0 ? LOW : HIGH);

  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right < 0 ? HIGH : LOW);

  analogWrite(ENA, constrain(abs(left),0,255));
  analogWrite(ENB, constrain(abs(right),0,255));
}

void stopCar() {
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

// sensor reading
byte readLine() {

  byte sensorValues = 0;

  for(int i=0; i<5; i++) {
    if(digitalRead(sensors[i]) == LOW)
      sensorValues |= (1 << i);
  }

  return sensorValues;
}

bool leftBranch(byte sensorValues) {
  return (sensorValues & 0b00011) == 0b00011;
}

bool rightBranch(byte sensorValues) {
  return (sensorValues & 0b11000) == 0b11000;
}

// ORIGINAL 90 DEGREE TURN
bool turn90(int direction) {

  stopCar();
  delay(60);

  // short burst so the car actually starts rotating, then the normal turn speed
  drive(direction * kickSpeed,
       -direction * kickSpeed);
  delay(kickTime);

  drive(direction * turnSpeed,
       -direction * turnSpeed);

  unsigned long turnStart = millis();
  bool leftOldLine = false;
  unsigned long stableTime = 0;

  while(millis() - turnStart < maxTurnTime) {

    byte sensorValues = readLine();
    bool centerDetected = sensorValues & 0b00100;

    if(!centerDetected) {
      leftOldLine = true;
      stableTime = 0;
    }

    if(leftOldLine &&
       millis() - turnStart > minTurnTime &&
       centerDetected) {

      if(stableTime == 0)
        stableTime = millis();

      if(millis() - stableTime > 30) {

        stopCar();
        delay(70);

        lastTurnTime = millis();
        turnCompleted = true;

        return true;
      }
    }
    else if(!centerDetected) {
      stableTime = 0;
    }
  }

  stopCar();
  return false;
}

// full circle search to fix
void recoverLine() {

  if(!isSearching) {

    isSearching = true;
    Serial.println("line lost -> searching");
    searchStartTime = millis();
    centerStartTime = 0;
    readyForNode = false;

    stopCar();
  }

  byte sensorValues = readLine();

  bool centerDetected = (sensorValues & 0b00100) != 0;

  // centering
  if(centerDetected) {

    if(centerStartTime == 0)
      centerStartTime = millis();
    stopCar();

    if(millis() - centerStartTime >= alignTime) {

      isSearching = false;
      centerStartTime = 0;
      lineLostTime = 0;
      lastNodeTime = millis();
      Serial.println("line found");

      return;
    }

  } else {

    centerStartTime = 0;
    drive(
      lastTurnDirection * searchSpeed,
      -lastTurnDirection * searchSpeed
    );
  }
  if(millis() - searchStartTime >= maxSearchTime) {

    stopCar();
    isSearching = false;
    searchFailed = true;
    failedAt = millis();
    Serial.println("search failed -> will retry");
  }
}

void setup() {

  Serial.begin(115200);

  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  for(int i=0; i<5; i++)
    pinMode(sensors[i],INPUT);

  stopCar();

  delay(3000);

  lastNodeTime = millis();
}

void loop() {

  // a failed search is not final: resume as soon as black is seen, or retry after a pause
  if(searchFailed) {
    stopCar();
    if(readLine() != 0 || millis() - failedAt > retryAfter) {
      searchFailed = false;
      lineLostTime = 0;
      readyForNode = false;
      lastNodeTime = millis();
      Serial.println("resuming");
    }
    return;
  }

  byte sensorValues = readLine();

  if(isSearching) {
    recoverLine();
    return;
  }

  // confirm if lost
  if(sensorValues == 0) {

    stopCar();

    // protect against false recovery
    if(turnCompleted &&
       millis() - lastTurnTime <
       turnProtectionTime) {

      lineLostTime = 0;
      return;
    }

    if(lineLostTime == 0)
      lineLostTime = millis();

    if(millis() - lineLostTime >= lostTime) {
      recoverLine();
    }

    return;
  }

  //detected
  lineLostTime = 0;

  // intersection detection
  if(sensorValues == 0b00100 ||
     sensorValues == 0b00110 ||
     sensorValues == 0b01100 ||
     sensorValues == 0b01110) {

    readyForNode = true;
  }

  if(readyForNode &&
     millis() - lastNodeTime > cooldownTime &&
     (leftBranch(sensorValues) || rightBranch(sensorValues))) {

    // intersection confirmation
    bool leftDetected = leftBranch(sensorValues);
    bool rightDetected = rightBranch(sensorValues);

    delay(nodeConfirmTime);

    byte newReading = readLine();

    bool turnLeft = leftDetected && leftBranch(newReading);
    bool turnRight = rightDetected && rightBranch(newReading);

    if(turnLeft || turnRight) {

      drive(scanSpeed,scanSpeed);

      unsigned long scanStart = millis();

      while(millis() - scanStart < nodeTime) {

        byte currentReading = readLine();

        if(leftBranch(currentReading)) turnLeft = true;
        if(rightBranch(currentReading)) turnRight = true;
      }

      stopCar();

      int direction = 0;

      // prioritise right
      if(turnRight)
        direction = 1;
      else if(turnLeft)
        direction = -1;

      Serial.print("node L=");
      Serial.print(turnLeft);
      Serial.print(" R=");
      Serial.print(turnRight);
      Serial.print(" -> ");
      Serial.println(direction == 1 ? "RIGHT" : direction == -1 ? "LEFT" : "NONE");

      if(direction != 0) {

        lastTurnDirection = direction;

        if(!turn90(direction)) {
          // a failed turn is no longer fatal: carry on and let the lost-line search handle it
          Serial.println("turn failed -> recovering");
          stopCar();
        }
      }

      readyForNode = false;
      lastNodeTime = millis();
      return;
    }
  }

  // follow straight line 
  int totalPosition = 0;
  int activeSensors = 0;

  for(int i=0; i<5; i++) {

    if(sensorValues & (1 << i)) {
      totalPosition += i - 2;
      activeSensors++;
    }
  }

  if(activeSensors > 0) {

    float error = (float)totalPosition / activeSensors;

    if(error < -0.3)
      lastTurnDirection = -1;

    else if(error > 0.3)
      lastTurnDirection = 1;

    int correction = (int)(kp * error);

    int leftSpeed = normalSpeed + correction;
    int rightSpeed = normalSpeed - correction;

    drive(
      constrain(leftSpeed,0,100),
      constrain(rightSpeed,0,100)
    );
  }
}
