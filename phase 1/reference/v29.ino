/*
Embedded project
Malak Mekky
Malak Taha
Jannah Hella
*/

// motor driver pins
#define ENA 5
#define IN1 22
#define IN2 23
#define IN3 24
#define IN4 25
#define ENB 6

// sensor pins (leftmost to rightmost)
const byte sensors[5] = {8,9,10,11,12};

// speeds
const int normalSpeed = 120;
const int turnSpeed = 110;
const int scanSpeed = 90;
const int kickSpeed = 150;
const unsigned long kickTime = 40;
const int followMaxSpeed = 160;
const int searchSpeed = 100;

// spin settings
const float spinTrimLeft = 1.00;
const float spinTrimRight = 1.00;
const unsigned long pivotCreepTime = 50;
const unsigned long brakeTime = 30;

// lost line settings
const float driftErr = 0.9;
const unsigned long lostTime = 120;
const unsigned long maxSearchTime = 6000;
const unsigned long alignTime = 30;
const unsigned long turnProtectionTime = 350;
const unsigned long retryAfter = 1500;

// turn settings
const unsigned long nodeTime = 130;
const unsigned long minTurnTime = 180;
const unsigned long maxTurnTime = 2200;
const unsigned long cooldownTime = 300;
const unsigned long nodeConfirmTime = 15;

// line following
const float kp = 15.0;

// state
bool readyForNode = false;
bool isSearching = false;
bool searchFailed = false;
bool turnCompleted = false;
bool driftLoss = false;

unsigned long lastNodeTime = 0;
unsigned long lastTurnTime = 0;
unsigned long lineLostTime = 0;
unsigned long searchStartTime = 0;
unsigned long centerStartTime = 0;
unsigned long failedAt = 0;

int lastTurnDirection = 1;
float lastError = 0;

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

// stops the motors fast so the car doesn't keep rolling
void brakeCar() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  delay(brakeTime);
  stopCar();
}

// spin on the spot (1 = right, -1 = left)
void spin(int dir, int speed) {
  drive(dir * (int)(speed * spinTrimLeft),
       -dir * (int)(speed * spinTrimRight));
}

// sensor reading (black = 1)
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

// 90 degree turn
bool turn90(int direction) {

  stopCar();
  delay(60);

  // small push to start the spin
  spin(direction, kickSpeed);
  delay(kickTime);

  spin(direction, turnSpeed);

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

        brakeCar();
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

// search for the line when it is lost
void recoverLine() {

  if(!isSearching) {

    isSearching = true;
    Serial.println("line lost -> searching");
    searchStartTime = millis();
    centerStartTime = 0;
    readyForNode = false;

    // big error before losing it = drifted off, otherwise the line ended
    driftLoss = fabs(lastError) >= driftErr;
    Serial.println(driftLoss ? "drifted off -> small sweeps" : "line ended -> full spin");

    stopCar();
  }

  byte sensorValues = readLine();

  bool centerDetected = (sensorValues & 0b00100) != 0;

  // center sensor found the line
  if(centerDetected) {

    if(centerStartTime == 0) {
      brakeCar();
      centerStartTime = millis();
    }
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

    int dir = lastTurnDirection;

    // if we only drifted, sweep one way then the other
    if(driftLoss) {
      unsigned long t = millis() - searchStartTime;
      if(t >= 300 && t < 900)
        dir = -lastTurnDirection;
    }

    spin(dir, searchSpeed);
  }

  // give up after max time, retry later
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

  // wait after a failed search then try again
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

  // no black under the car
  if(sensorValues == 0) {

    stopCar();

    // ignore it right after a turn
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

  lineLostTime = 0;

  // normal line seen, so we are ready for the next intersection
  if(sensorValues == 0b00100 ||
     sensorValues == 0b00110 ||
     sensorValues == 0b01100 ||
     sensorValues == 0b01110) {

    readyForNode = true;
  }

  // intersection
  if(readyForNode &&
     millis() - lastNodeTime > cooldownTime &&
     (leftBranch(sensorValues) || rightBranch(sensorValues))) {

    bool leftDetected = leftBranch(sensorValues);
    bool rightDetected = rightBranch(sensorValues);

    // read again to make sure it is real
    delay(nodeConfirmTime);

    byte newReading = readLine();

    bool turnLeft = leftDetected && leftBranch(newReading);
    bool turnRight = rightDetected && rightBranch(newReading);

    if(turnLeft || turnRight) {

      // move forward slowly and check for branches
      drive(scanSpeed,scanSpeed);

      unsigned long scanStart = millis();

      while(millis() - scanStart < nodeTime) {

        byte currentReading = readLine();

        if(leftBranch(currentReading)) turnLeft = true;
        if(rightBranch(currentReading)) turnRight = true;
      }

      // move a bit more so the wheels are in the middle of the intersection
      delay(pivotCreepTime);
      brakeCar();

      int direction = 0;

      // right has priority
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
          Serial.println("turn failed -> recovering");
          stopCar();
        }
      }

      readyForNode = false;
      lastNodeTime = millis();
      lastError = 0;
      return;
    }
  }

  // follow the line
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
    lastError = error;

    if(error < -0.3)
      lastTurnDirection = -1;

    else if(error > 0.3)
      lastTurnDirection = 1;

    int correction = (int)(kp * error);

    int leftSpeed = normalSpeed + correction;
    int rightSpeed = normalSpeed - correction;

    drive(
      constrain(leftSpeed,0,followMaxSpeed),
      constrain(rightSpeed,0,followMaxSpeed)
    );
  }
}
