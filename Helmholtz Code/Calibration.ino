#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <Adafruit_LSM9DS1.h>

// ================= PIN DEFINITIONS =================
constexpr int N_AXES = 3;
constexpr int RPWM_PINS[N_AXES] = {3, 9, 11};
constexpr int LPWM_PINS[N_AXES] = {5, 6, 10};
constexpr int R_EN_PINS[N_AXES] = {7, 4, 12};
constexpr int L_EN_PINS[N_AXES] = {8, 2, 13};

// ================= SENSORS =================
Adafruit_INA260 ina[N_AXES];
const uint8_t INA_ADDR[N_AXES] = {0x40, 0x41, 0x44};

Adafruit_LSM9DS1 lsm;

// ================= H-BRIDGE =================
void enableBridges() {
  for (int axis = 0; axis < N_AXES; ++axis) {
    pinMode(R_EN_PINS[axis], OUTPUT);
    pinMode(L_EN_PINS[axis], OUTPUT);
    digitalWrite(R_EN_PINS[axis], HIGH);
    digitalWrite(L_EN_PINS[axis], HIGH);
    analogWrite(RPWM_PINS[axis], 0);
    analogWrite(LPWM_PINS[axis], 0);
  }
}

void zeroAll() {
  for (int axis = 0; axis < N_AXES; ++axis) {
    analogWrite(RPWM_PINS[axis], 0);
    analogWrite(LPWM_PINS[axis], 0);
  }
}

void writeAxis(int axis, int pwm_signed) {
  pwm_signed = constrain(pwm_signed, -255, 255);
  int pwm = abs(pwm_signed);
  digitalWrite(R_EN_PINS[axis], HIGH);
  digitalWrite(L_EN_PINS[axis], HIGH);
  if (pwm_signed >= 0) {
    analogWrite(RPWM_PINS[axis], pwm);
    analogWrite(LPWM_PINS[axis], 0);
  } else {
    analogWrite(RPWM_PINS[axis], 0);
    analogWrite(LPWM_PINS[axis], pwm);
  }
}

// ================= SENSOR READING =================
float readCurrent(int axis) {
  return ina[axis].readCurrent() * 1e-3f;
}

// Average N magnetometer readings to reduce noise
void readMagAvg(float &bx, float &by, float &bz, int n = 10) {
  bx = by = bz = 0.0f;
  for (int i = 0; i < n; i++) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    bx += mag.magnetic.x;
    by += mag.magnetic.y;
    bz += mag.magnetic.z;
    delay(20);  // LSM9DS1 mag ODR ~80Hz so 20ms between reads is safe
  }
  bx /= n; by /= n; bz /= n;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  enableBridges();
  zeroAll();

  // Init INA260s
  for (int axis = 0; axis < N_AXES; ++axis) {
    if (!ina[axis].begin(INA_ADDR[axis], &Wire)) {
      Serial.print(F("INA260 not found at 0x"));
      Serial.println(INA_ADDR[axis], HEX);
      while (1) {}
    }
    ina[axis].setAveragingCount(INA260_COUNT_16);
    ina[axis].setVoltageConversionTime(INA260_TIME_1_1_ms);
    ina[axis].setCurrentConversionTime(INA260_TIME_1_1_ms);
  }

  // Init LSM9DS1 (Reverted to standard call to compile properly)
  if (!lsm.begin()) {
    Serial.println(F("LSM9DS1 not found — check wiring or I2C addresses"));
    while (1) {}
  }
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  Serial.println(F("READY"));
}

// ================= LOOP =================
char buf[32];
int  buf_pos = 0;

void processCommand(const char* cmd) {
  if (strncmp(cmd, "zero", 4) == 0) {
    zeroAll();
    Serial.println(F("ok"));

  } else if (strncmp(cmd, "read", 4) == 0) {
    float I0 = readCurrent(0);
    float I1 = readCurrent(1);
    float I2 = readCurrent(2);
    float bx, by, bz;
    readMagAvg(bx, by, bz, 10);

    Serial.print(F("data,"));
    Serial.print(I0, 4); Serial.print(F(","));
    Serial.print(I1, 4); Serial.print(F(","));
    Serial.print(I2, 4); Serial.print(F(",")); 
    Serial.print(bx, 4); Serial.print(F(","));
    Serial.print(by, 4); Serial.print(F(","));
    Serial.println(bz, 4);

  } else if (strncmp(cmd, "pwm,", 4) == 0) {
    int axis = 0, value = 0;
    if (sscanf(cmd + 4, "%d,%d", &axis, &value) == 2) {
      zeroAll();  
      delay(100);
      if (axis >= 0 && axis < N_AXES) {
        writeAxis(axis, value);
      }
      Serial.println(F("ok"));
    } else {
      Serial.println(F("err"));
    }
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[buf_pos] = '\0';
      if (buf_pos > 0) processCommand(buf);
      buf_pos = 0;
    } else if (buf_pos < (int)sizeof(buf) - 1) {
      buf[buf_pos++] = c;
    } else {
      buf_pos = 0;
    }
  }
}