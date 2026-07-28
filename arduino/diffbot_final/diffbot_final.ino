/*
  diffbot_final.ino

  4WD car — L298N motor driver, no PWM (ENA/ENB jumpered to 5V on the
  board itself), controlled over USB serial from a PC/ROS2.

  WIRING:
    L298N IN1 -> Arduino D7   (left side direction A)
    L298N IN2 -> Arduino D8   (left side direction B)
    L298N IN3 -> Arduino D9   (right side direction A)
    L298N IN4 -> Arduino D10  (right side direction B)
    L298N ENA -> L298N 5V pin (jumper wire, board-to-board, NOT to Arduino)
    L298N ENB -> L298N 5V pin (jumper wire, board-to-board, NOT to Arduino)
    L298N GND -> Arduino GND  (common ground, required)
    L298N 12V -> battery pack + (2x 18650 in series)
    L298N GND (power terminal) -> battery pack -

  SERIAL PROTOCOL:
    Send:  L:<val>,R:<val>\n
      val > 0  -> that side spins forward
      val < 0  -> that side spins reverse
      val = 0  -> that side stops
      (magnitude is ignored, no PWM on this hardware)

    Examples:
      L:100,R:100\n    -> drive straight forward
      L:-100,R:-100\n  -> drive straight backward
      L:-100,R:100\n   -> spin left in place
      L:100,R:-100\n   -> spin right in place
      L:0,R:0\n        -> stop

    Receive: OK:<left>,<right>\n   (ack after every accepted command)

  SAFETY:
    If no valid command arrives for CMD_TIMEOUT_MS, motors auto-stop.
    This prevents a runaway car if USB/serial disconnects.
*/

#include <Arduino.h>

// ---------------- pin configuration ----------------
const uint8_t IN1 = 7;   // left  motors, direction pin A
const uint8_t IN2 = 8;   // left  motors, direction pin B
const uint8_t IN3 = 9;   // right motors, direction pin A
const uint8_t IN4 = 10;  // right motors, direction pin B

// ---------------- safety ----------------
const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdTime = 0;

// ---------------- state ----------------
int leftCmd  = 0;   // >0 forward, <0 reverse, 0 stop
int rightCmd = 0;

String rxBuffer;

// ---------------- motor control ----------------
void driveSide(uint8_t pinA, uint8_t pinB, int cmd) {
  if (cmd > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (cmd < 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void applyMotors() {
  driveSide(IN1, IN2, leftCmd);
  driveSide(IN3, IN4, rightCmd);
}

void stopMotors() {
  leftCmd = 0;
  rightCmd = 0;
  applyMotors();
}

// ---------------- command parsing ----------------
// Expects a line like: L:100,R:-100
bool parseLine(const String &line, int &l, int &r) {
  int lPos = line.indexOf("L:");
  int rPos = line.indexOf("R:");
  if (lPos == -1 || rPos == -1) return false;

  int commaPos = line.indexOf(',', lPos);
  if (commaPos == -1 || commaPos > rPos) return false;

  String lStr = line.substring(lPos + 2, commaPos);
  String rStr = line.substring(rPos + 2);
  lStr.trim();
  rStr.trim();

  if (lStr.length() == 0 || rStr.length() == 0) return false;

  l = lStr.toInt();
  r = rStr.toInt();
  return true;
}

// ---------------- setup / loop ----------------
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  Serial.begin(115200);
  rxBuffer.reserve(32);
  lastCmdTime = millis();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      int l, r;
      if (parseLine(rxBuffer, l, r)) {
        leftCmd  = l;
        rightCmd = r;
        lastCmdTime = millis();
        applyMotors();

        Serial.print("OK:");
        Serial.print(leftCmd);
        Serial.print(",");
        Serial.println(rightCmd);
      }
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
    }
  }

  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    if (leftCmd != 0 || rightCmd != 0) {
      stopMotors();
    }
  }
}
