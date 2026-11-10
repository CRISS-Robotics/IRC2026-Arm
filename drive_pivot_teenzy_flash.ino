#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rmw_microros/rmw_microros.h>
// Do not call Serial.print: micro-ROS owns the USB serial port.
// Do not use LED_BUILTIN: pin 13 is M1_PWM.
// I2C = Wire2: SDA2 = pin 25, SCL2 = pin 24. Never use 24/25 elsewhere.

// <---------------- DRIVE PINS ---------------->
#define M1_PWM 13
#define M1_DIR 41
#define M2_PWM 15
#define M2_DIR 14
#define M3_PWM 5
#define M3_DIR 4
#define M4_PWM 3
#define M4_DIR 2
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)
const float PWM_GAIN = 255.0;

// <---------------- ACTUATOR (disabled) ---------------->
// #define ACTUATOR_PWM 23
// #define ACTUATOR_DIR 22

// <---------------- PIVOT PINS & CONSTANTS ---------------->
#define MOTOR_SPEED   255
#define PCA9548A_ADDR 0x70   // A0 = A1 = A2 = GND
#define AS5600_ADDR   0x36

int wheel_ch[4] = {0, 1, 2, 3};
const int TOLERANCE = 5;

// Limit switches are unused in the logic. Moved off 14/15, which the drive uses.
const int limitSwitchPin[4] = { 20, 21, 22, 23 };   // SET TO YOUR REAL PINS
const int pwm[4]            = { 12, 10, 9, 7 };
const int dir[4]            = { 33, 11, 8, 6 };

int ZERO_DEG_OFFSET[4] = { 2744, 3321, 3911, 214 };
int target_enc[4] = { ZERO_DEG_OFFSET[0], ZERO_DEG_OFFSET[1], ZERO_DEG_OFFSET[2], ZERO_DEG_OFFSET[3] };

const bool dirclockhigh[4] = { false, true, false, false };
float diag[4] = { 0, 0, 0, 0 };   // published per wheel: angle, or a negative error code

// <---------------- MDD10A CLASS (DRIVE) ---------------->
class MDD10A {
  public:
    MDD10A(int pwm_pin, int dir_pin) : _pwm_pin(pwm_pin), _dir_pin(dir_pin) {
      pinMode(_pwm_pin, OUTPUT);
      pinMode(_dir_pin, OUTPUT);
    }
    void run(int pwr) {
      int d = (pwr >= 0) ? HIGH : LOW;
      int u = min(abs(pwr), 255);
      if (abs(pwr) < 15) u = 0;
      analogWrite(_pwm_pin, u);
      digitalWrite(_dir_pin, d);
    }
    void stop() {
      analogWrite(_pwm_pin, 0);
      digitalWrite(_dir_pin, LOW);
    }
  private:
    int _pwm_pin, _dir_pin;
};

MDD10A motor1(M1_PWM, M1_DIR);
MDD10A motor2(M2_PWM, M2_DIR);
MDD10A motor3(M3_PWM, M3_DIR);
MDD10A motor4(M4_PWM, M4_DIR);

// <---------------- MICRO-ROS VARIABLES ---------------->
rcl_subscription_t sub_wheels;
rcl_subscription_t sub_angles;
rcl_publisher_t pub_encoders;

std_msgs__msg__Float32MultiArray msg_wheels;
float msg_wheels_data[5];
std_msgs__msg__Float32MultiArray msg_angles;
float msg_angles_data[4];
std_msgs__msg__Float32MultiArray pub_enc_msg;
float pub_enc_data[4];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// <---------------- STATE ---------------->
float target_v1 = 0, target_v2 = 0, target_v3 = 0, target_v4 = 0;
float actuator_cmd = 0;
unsigned long last_cmd_time = 0;

void stopAllMotors() {
  motor1.stop(); motor2.stop(); motor3.stop(); motor4.stop();
  for (int i = 0; i < 4; i++) analogWrite(pwm[i], 0);
}

// Setup failed: motors off, then wait forever
void error_loop() {
  stopAllMotors();
  while (true) delay(100);
}
#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

// <---------------- I2C HELPERS ---------------->
uint8_t selectI2CChannel(uint8_t channel) {          // returns I2C error code, 0 = OK
  Wire2.beginTransmission(PCA9548A_ADDR);
  Wire2.write(1 << channel);
  return Wire2.endTransmission();
}

uint16_t readAS5600Angle() {                         // 0xFFFF on failure
  Wire2.beginTransmission(AS5600_ADDR);
  Wire2.write(0x0C);
  if (Wire2.endTransmission(true) != 0) return 0xFFFF;
  if (Wire2.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return 0xFFFF;
  uint8_t high = Wire2.read();
  uint8_t low  = Wire2.read();
  return ((high & 0x0F) << 8) | low;
}

void restartI2C() {
  Wire2.end();
  Wire2.begin();
  Wire2.setClock(100000);
}

// <---------------- MOTOR HELPER & CALLBACKS ---------------->
void setMotor(int speed, bool direction, int M_PWM, int M_DIR) {
  if (speed == 0) {
    analogWrite(M_PWM, 0);
    return;
  }
  digitalWrite(M_DIR, direction ? HIGH : LOW);
  analogWrite(M_PWM, speed);
}

void wheels_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * m = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (m->data.size >= 5) {
    target_v1 = m->data.data[0];
    target_v2 = m->data.data[1];
    target_v3 = m->data.data[2];
    target_v4 = m->data.data[3];
    actuator_cmd = m->data.data[4];
    last_cmd_time = millis();
  }
}

void angles_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * m = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (m->data.size >= 4) {
    for (int i = 0; i < 4; i++) {
      target_enc[i] = ZERO_DEG_OFFSET[i] + (int)((m->data.data[i] / 360.0f) * 4096.0f);
      target_enc[i] = target_enc[i] % 4096;
      if (target_enc[i] < 0) target_enc[i] += 4096;
    }
  }
}
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void setup() {
  set_microros_serial_transports(Serial);
  state = WAITING_AGENT;
  // Initialize motors, AS5600, etc. here
}

// Wrap your current setup code into a boolean function
bool create_entities() {
  allocator = rcl_get_default_allocator();
  
  // init options, support, node, publishers, subscribers, executor
  rclc_support_init(&support, 0, NULL, &allocator);
  // ... create everything ...
  
  return true; // Return false if any rclc creation step fails
}

// Function to clean up the old session
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Destroy everything in reverse order
  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// <---------------- SETUP ---------------->
void setup() {
  // Hardware first, everything forced off
  for (int i = 0; i < 4; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
    analogWriteFrequency(pwm[i], 20000);
    pinMode(limitSwitchPin[i], INPUT_PULLDOWN);
  }
  stopAllMotors();

  Wire2.begin();
  Wire2.setClock(100000);

  set_microros_transports();

  // Wait until the agent answers (motors stay off meanwhile)
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    delay(200);
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "rover_drive_pivot_node", "", &support));

  msg_wheels.data.capacity = 5;
  msg_wheels.data.data = msg_wheels_data;
  msg_wheels.data.size = 0;

  msg_angles.data.capacity = 4;
  msg_angles.data.data = msg_angles_data;
  msg_angles.data.size = 0;

  pub_enc_msg.data.capacity = 4;
  pub_enc_msg.data.data = pub_enc_data;
  pub_enc_msg.data.size = 4;

  RCCHECK(rclc_subscription_init_default(
    &sub_wheels, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/wheel_speeds"));
  RCCHECK(rclc_subscription_init_default(
    &sub_angles, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "target_angles"));
  RCCHECK(rclc_publisher_init_default(
    &pub_encoders, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "pivot_encoders"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_wheels, &msg_wheels, &wheels_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_angles, &msg_angles, &angles_callback, ON_NEW_DATA));
}

// <---------------- LOOP ---------------->
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // 1. Drive (stops 200 ms after the last command)
  if (millis() - last_cmd_time > 200) {
    target_v1 = target_v2 = target_v3 = target_v4 = 0.0;
    actuator_cmd = 0.0;
    motor1.stop(); motor2.stop(); motor3.stop(); motor4.stop();
  } else {
    motor1.run((int)(target_v1 * PWM_GAIN));
    motor2.run(-(int)(target_v2 * PWM_GAIN));
    motor3.run(-(int)(target_v3 * PWM_GAIN));
    motor4.run(-(int)(target_v4 * PWM_GAIN));
  }

  // 2. Actuator logic stays disabled (see the commented block in your original)

  // 3. Pivot closed loop
  bool anyMuxError = false;

  for (int i = 0; i < 4; i++) {
    uint8_t muxErr = selectI2CChannel(wheel_ch[i]);
    uint16_t raw = 0xFFFF;

    if (muxErr != 0) {
      anyMuxError = true;
      diag[i] = -100.0f - muxErr;          // mux did not accept the channel
    } else {
      raw = readAS5600Angle();
      diag[i] = (raw == 0xFFFF) ? -200.0f : (float)raw;   // -200: mux OK, sensor failed
    }

    if (raw == 0xFFFF) {                   // never drive on bad data
      setMotor(0, false, pwm[i], dir[i]);
      continue;
    }

    long error = target_enc[i] - (long)raw;
    if (error > 2048) error -= 4096;
    else if (error < -2048) error += 4096;

    bool motor_dir = (error > 0) ? !dirclockhigh[i] : dirclockhigh[i];

    if (abs(error) <= TOLERANCE) setMotor(0, false, pwm[i], dir[i]);
    else setMotor(MOTOR_SPEED, motor_dir, pwm[i], dir[i]);
  }

  if (anyMuxError) restartI2C();

  // 4. Publish at 20 Hz
  static uint32_t last_pub = 0;
  if (millis() - last_pub >= 50) {
    last_pub = millis();
    for (int i = 0; i < 4; i++) pub_enc_data[i] = diag[i];
    RCSOFTCHECK(rcl_publish(&pub_encoders, &pub_enc_msg, NULL));
  }
}



