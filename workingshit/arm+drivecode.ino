#include <math.h>

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h> 

// <---------------- HARDWARE PINS (DRIVE) ---------------->
#define M1_PWM 13
#define M1_DIR 21

#define M2_PWM 16
#define M2_DIR 4

#define M3_PWM 2
#define M3_DIR 15

#define M4_PWM 0
#define M4_DIR 12

// <---------------- HARDWARE PINS (ACTUATOR) ---------------->
#define ACTUATOR_PWM 23
#define ACTUATOR_DIR 22

// <---------------- HARDWARE PINS (ARM) ---------------->
#define WRIST1_PWM 18
#define WRIST1_DIR 19
#define WRIST1_ENCA 37
#define WRIST1_ENCB 35

#define WRIST2_PWM 17
#define WRIST2_DIR 5
#define WRIST2_ENCA 41
#define WRIST2_ENCB 40

#define GRIPPY_PWM 14
#define GRIPPY_DIR 27
#define GRIPPY_ENCA 44
#define GRIPPY_ENCB 45

#define LA1_PWM 33
#define LA1_DIR 32
#define LA1_POTEN 39

#define LA2_PWM 26
#define LA2_DIR 25
#define LA2_POTEN 36

// <---------------- CONSTANTS ---------------->
const float PWM_GAIN = 255.0; 

// <---------------- UNIFIED MDD10A CLASS ---------------->
class MDD10A {
  public:
    MDD10A(int pwm, int dir); 
    MDD10A(int pwm, int dir, int enca, int encb); 
    ~MDD10A();

    void run(int pwr);
    void stop();
    void attachEncInterrupt();
    int getEncoderCount() const;

  private:
    int _pwm_pin, _dir_pin, _enca, _encb;
    volatile int _encoderCount; 
    static MDD10A* _instance; 
};

MDD10A* MDD10A::_instance = nullptr;

MDD10A::MDD10A(int pwm, int dir) 
  : _pwm_pin(pwm), _dir_pin(dir), _enca(0), _encb(0), _encoderCount(0) {
  pinMode(_pwm_pin, OUTPUT); pinMode(_dir_pin, OUTPUT);
}

MDD10A::MDD10A(int pwm, int dir, int enca, int encb)
  : _pwm_pin(pwm), _dir_pin(dir), _enca(enca), _encb(encb), _encoderCount(0) {
  pinMode(_pwm_pin, OUTPUT); pinMode(_dir_pin, OUTPUT);
  pinMode(_enca, INPUT_PULLUP); pinMode(_encb, INPUT_PULLUP);
  if (enca != 0 && encb != 0) _instance = this;
}

MDD10A::~MDD10A() { stop(); }

void MDD10A::run(int pwr) {
  int dir = (pwr >= 0) ? HIGH : LOW; 
  int u = min(abs(pwr), 255);
  if (abs(pwr) < 15) u = 0; 
  analogWrite(_pwm_pin, u);
  digitalWrite(_dir_pin, dir);
}

void MDD10A::stop() {
  analogWrite(_pwm_pin, 0); 
  digitalWrite(_dir_pin, LOW); 
}

int MDD10A::getEncoderCount() const { return _encoderCount; }

// <---------------- MOTORS INSTANTIATION ---------------->
MDD10A motor1(M1_PWM, M1_DIR);
MDD10A motor2(M2_PWM, M2_DIR);
MDD10A motor3(M3_PWM, M3_DIR);
MDD10A motor4(M4_PWM, M4_DIR);

MDD10A wrist1_motor(WRIST1_PWM, WRIST1_DIR, WRIST1_ENCA, WRIST1_ENCB);
MDD10A wrist2_motor(WRIST2_PWM, WRIST2_DIR, WRIST2_ENCA, WRIST2_ENCB);
MDD10A grippy_motor(GRIPPY_PWM, GRIPPY_DIR, GRIPPY_ENCA, GRIPPY_ENCB);
MDD10A LA1(LA1_PWM, LA1_DIR, 0, 0);
MDD10A LA2(LA2_PWM, LA2_DIR, 0, 0);

// <-------------- MICRO ROS VARIABLES --------------->
rcl_publisher_t publisher2;
rcl_subscription_t subscriber2;
rcl_subscription_t sub; 
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void error_loop() { while (1) { delay(100); } }

// <---------------- STATE VARIABLES -------->
float target_v1 = 0.0, target_v2 = 0.0, target_v3 = 0.0, target_v4 = 0.0;
float actuator_cmd = 0.0; // 0=Stop, 1=Push, 2=Pull
unsigned long last_cmd_time = 0; 

int arm_posi[3] = { 0, 0, 0 };
int LA1_enc, LA2_enc;

std_msgs__msg__Float32MultiArray msg_sub; 
std_msgs__msg__Int32MultiArray arm_encoder_msg;
std_msgs__msg__Int32MultiArray arm_pwm_msg;

volatile uint32_t last_message_time_arm = 0;
volatile int arm_pwm_values[5] = { 0 };

// < -------------- CALLBACKS ------------- >
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    arm_encoder_msg.data.data[0] = LA1_enc;
    arm_encoder_msg.data.data[1] = LA2_enc;
    arm_encoder_msg.data.data[2] = arm_posi[0];
    arm_encoder_msg.data.data[3] = arm_posi[1];
    arm_encoder_msg.data.data[4] = arm_posi[2];
    RCSOFTCHECK(rcl_publish(&publisher2, &arm_encoder_msg, NULL));
  }
}

void subscription_callback_arm(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  for (int i = 0; i < 5; i++) {
    arm_pwm_values[i] = msg->data.data[i];
  }
  last_message_time_arm = millis();
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * received_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (received_msg->data.size >= 5) { // INCREASED TO 5
    target_v1 = received_msg->data.data[0];
    target_v2 = received_msg->data.data[1];
    target_v3 = received_msg->data.data[2];
    target_v4 = received_msg->data.data[3];
    actuator_cmd = received_msg->data.data[4]; // GRAB ACTUATOR COMMAND
    last_cmd_time = millis();
  }
}

// < ----------- SETUP -------------------->
void setup() {
  set_microros_transports();
  Serial.begin(115200);

  // Setup Actuator Pins
  pinMode(ACTUATOR_PWM, OUTPUT);
  pinMode(ACTUATOR_DIR, OUTPUT);
  digitalWrite(ACTUATOR_PWM, LOW);
  digitalWrite(ACTUATOR_DIR, LOW);

  delay(2000);
  allocator = rcl_get_default_allocator();
  
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "multi_motor_node", "", &support));

  // Memory Allocation
  msg_sub.data.capacity = 5; // INCREASED TO 5
  msg_sub.data.data = (float *)malloc(5 * sizeof(float)); // INCREASED TO 5
  msg_sub.data.size = 5; // INCREASED TO 5

  arm_encoder_msg.data.capacity = 5;
  arm_encoder_msg.data.size = 5;
  arm_encoder_msg.data.data = (int32_t *)malloc(5 * sizeof(int32_t));

  arm_pwm_msg.data.capacity = 5;
  arm_pwm_msg.data.size = 5;
  arm_pwm_msg.data.data = (int32_t *)malloc(5 * sizeof(int32_t));

  // Initialize ROS Publishers/Subscribers
  RCCHECK(rclc_subscription_init_default(&sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/wheel_speeds"));
  RCCHECK(rclc_publisher_init_default(&publisher2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "arm_encoder_data"));  
  RCCHECK(rclc_subscription_init_default(&subscriber2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "arm_pwm_commands"));  

  // Timer setup
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Executor setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &arm_pwm_msg, &subscription_callback_arm, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub, &msg_sub, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  uint32_t current_time = millis();

  // --- DRIVE TIMEOUT ---
  if (current_time - last_cmd_time > 200) {
    target_v1 = target_v2 = target_v3 = target_v4 = 0.0;
    actuator_cmd = 0.0; // STOP ACTUATOR ON TIMEOUT
    motor1.run(0); motor2.run(0); motor3.run(0); motor4.run(0);
  } else {
    motor1.run((int)(target_v1 * PWM_GAIN));
    motor2.run(-    (int)(target_v2 * PWM_GAIN));
    motor3.run(-(int)(target_v3 * PWM_GAIN)); 
    motor4.run((int)(target_v4 * PWM_GAIN));
  }

  // --- ACTUATOR HARDWARE LOGIC ---
  if (actuator_cmd == 1.0) {
    digitalWrite(ACTUATOR_PWM, HIGH);
    digitalWrite(ACTUATOR_DIR, HIGH);
  } else if (actuator_cmd == 2.0) {
    digitalWrite(ACTUATOR_PWM, HIGH);
    digitalWrite(ACTUATOR_DIR, LOW);
  } else {
    digitalWrite(ACTUATOR_PWM, LOW);
    digitalWrite(ACTUATOR_DIR, LOW);
  }

  // --- ARM TIMEOUT ---
  if (current_time - last_message_time_arm > 1000) {
    for (int i = 0; i < 5; i++) {
      arm_pwm_values[i] = 0;
    }
  }

  // --- ARM SENSORS & MOTORS ---
  LA1_enc = analogRead(LA1_POTEN);
  LA2_enc = analogRead(LA2_POTEN);

  LA1.run(arm_pwm_values[0]);
  LA2.run(arm_pwm_values[1]);
  wrist1_motor.run(arm_pwm_values[2]);
  wrist2_motor.run(arm_pwm_values[3]);
  grippy_motor.run(arm_pwm_values[4]);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
