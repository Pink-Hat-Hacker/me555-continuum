const int IN1 = 8;
const int IN2 = 9;

const int PULL_MS    = 100000;   // run time per key press, pull
const int RELEASE_MS = 1500;   // run time per key press, release

// Motor state
enum Dir { STOPPED, PULLING, RELEASING };
Dir motorDir = STOPPED;
unsigned long moveStart = 0;
unsigned long moveDuration = 0;

// Arrow key escape-sequence parser (ESC [ A = up, ESC [ B = down)
int escState = 0;

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  motorDir = STOPPED;
}

void startPull() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  motorDir = PULLING;
  moveStart = millis();
  moveDuration = PULL_MS;
  Serial.println("Pull");
}

void startRelease() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  motorDir = RELEASING;
  moveStart = millis();
  moveDuration = RELEASE_MS;
  Serial.println("Release");
}

void handleKey(char c) {
  if (escState == 0) {
    if (c == 27)                    { escState = 1; return; }   // ESC
    if (c == 'd' || c == 'D')       startPull();
    else if (c == 'u' || c == 'U')  startRelease();
    else if (c == 's' || c == 'S' || c == ' ') { motorStop(); Serial.println("Stop"); }
  } else if (escState == 1) {
    escState = (c == '[') ? 2 : 0;
  } else if (escState == 2) {
    if (c == 'B')      startPull();      // down arrow
    else if (c == 'A') startRelease();   // up arrow
    escState = 0;
  }
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorStop();
  Serial.begin(9600);
  Serial.println("Down arrow / d = pull, Up arrow / u = release, s = stop");
}

void loop() {
  while (Serial.available() > 0) {
    handleKey(Serial.read());
  }

  // Stop automatically when the move time is up
  if (motorDir != STOPPED && millis() - moveStart >= moveDuration) {
    motorStop();
    Serial.println("Done");
  }
}