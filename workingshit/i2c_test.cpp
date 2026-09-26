#include <Wire.h>

#define MOTOR_SPEED 255

#define PCA9548A_ADDR  0x70  // Default address of PCA9548A
#define AS5600_ADDR    0x36  // Fixed address of AS5600 encoder
#define TOTAL_CHANNELS 8     // Channels 0 to 7

#define SDA_PIN 4
#define SCL_PIN 15

int wheel[4] = {7,5,0,1};
const int pwm[4] = { 23, 21, 18, 17 }; //5,17,16,22
const int dir[4] = { 22, 19, 5, 16 }; //15,21,4,23

void selectI2CChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Read 12-bit raw angle from the active channel's AS5600 sensor
uint16_t readAS5600Angle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // RAW ANGLE register high byte
  if (Wire.endTransmission(true) != 0) {
    return 0xFFFF; // Communication error or sensor disconnected
  }

  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) == 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();
    return ((highByte & 0x0F) << 8) | lowByte;
  }

  return 0xFFFF; // Timeout or read failure
}

void setMotor(int speed, bool direction, int M_PWM, int M_DIR) {
  //make pin compatible
  if (speed == 0) {
    analogWrite(M_PWM, 0);
    return;
  }
  digitalWrite(M_DIR, direction ? HIGH : LOW);
  analogWrite(M_PWM, speed);
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
  }

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.setTimeOut(5);
}

void loop() {
  /*
  setMotor(MOTOR_SPEED, true, pwm[1], dir[1]);
  Serial.println(readAS5600Angle());
  delay(5000);

  setMotor(MOTOR_SPEED, false, pwm[1], dir[1]);
  Serial.println(readAS5600Angle());
  delay(5000);
  */
  for (int i=0 ; i<4 ; i++){
     selectI2CChannel(wheel[i]);
     Serial.print(readAS5600Angle());
     Serial.print(", ");
  }
  Serial.println();
}
