// EncoderTest.cpp - standalone AS5600 diagnostic for prototype_1_BA
// The motor driver is NOT initialized, so the motor is never driven.
// Turn the motor slowly by hand and watch the serial output (115200 baud).

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

static const uint8_t  AS5600_ADDR = 0x36;
static const uint32_t I2C_CLOCK   = 400000;   // same as main firmware; try 100000 if errors appear

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

uint32_t i2cErrors = 0;

// Read 'len' bytes starting at register 'reg'. Returns false on any I2C error.
bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { i2cErrors++; return false; }
  if (Wire.requestFrom(AS5600_ADDR, len) != len) { i2cErrors++; return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== AS5600 ENCODER TEST ===");

  Wire.begin();
  sensor.init(&Wire);          // calls Wire.begin() internally
  Wire.setClock(I2C_CLOCK);    // set clock afterwards so it is not reset

  Wire.beginTransmission(AS5600_ADDR);
  uint8_t err = Wire.endTransmission();
  Serial.print("AS5600 at 0x36: ");
  Serial.println(err == 0 ? "found" : "NOT FOUND");
  Serial.print("I2C clock [Hz]: ");
  Serial.println(I2C_CLOCK);

  Serial.println("Turn the motor slowly by hand.");
  Serial.println("angle[deg]  raw   MD ML MH  AGC  magnitude  span2s[deg]  i2cErr  notes");
}

void loop() {
  // SimpleFOC angle, sampled as fast as possible
  sensor.update();
  float angleDeg = sensor.getAngle() * RAD_TO_DEG;

  // Movement span over a 2 s window (shows whether hand rotation is detected)
  static float    winMin = 1e9f, winMax = -1e9f, span = 0.0f;
  static uint32_t winStart = millis();
  winMin = fminf(winMin, angleDeg);
  winMax = fmaxf(winMax, angleDeg);
  if (millis() - winStart >= 2000) {
    span = winMax - winMin;
    winMin = 1e9f; winMax = -1e9f;
    winStart = millis();
  }

  // Print 10x per second
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  uint8_t  b[2];
  int      raw = -1, agc = -1, mag = -1;
  int      md = -1, ml = -1, mh = -1;

  if (readRegs(0x0C, b, 2)) raw = ((b[0] & 0x0F) << 8) | b[1];      // RAW ANGLE (12 bit)
  if (readRegs(0x0B, b, 1)) {                                       // STATUS
    md = (b[0] >> 5) & 1;  // magnet detected
    ml = (b[0] >> 4) & 1;  // magnet too weak
    mh = (b[0] >> 3) & 1;  // magnet too strong
  }
  if (readRegs(0x1A, b, 1)) agc = b[0];                             // AGC
  if (readRegs(0x1B, b, 2)) mag = ((b[0] & 0x0F) << 8) | b[1];      // MAGNITUDE

  Serial.print(angleDeg, 2);  Serial.print("\t");
  Serial.print(raw);          Serial.print("\t");
  Serial.print(md);           Serial.print("  ");
  Serial.print(ml);           Serial.print("  ");
  Serial.print(mh);           Serial.print("   ");
  Serial.print(agc);          Serial.print("\t");
  Serial.print(mag);          Serial.print("\t");
  Serial.print(span, 1);      Serial.print("\t");
  Serial.print(i2cErrors);    Serial.print("\t");

  if (raw < 0)        Serial.print("I2C READ FAILED ");
  if (md == 0)        Serial.print("NO MAGNET ");
  if (ml == 1)        Serial.print("MAGNET TOO WEAK/FAR ");
  if (mh == 1)        Serial.print("MAGNET TOO STRONG/CLOSE ");
  Serial.println();
}