// #include <Arduino.h>
// #include <micro_ros_platformio.h>

// #include <Cytron.h>
// #include <Encoder.h>

// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <rcl/error_handling.h>
// #include <rmw_microros/rmw_microros.h>

// #include <std_msgs/msg/int32.h>
// #include <irc_interfaces/msg/ps4.h>
// #include <geometry_msgs/msg/twist.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// int Right_front_motor_pin=10;
// int Right_front_dir_pin=11;

// int Left_front_motor_pin=2;
// int Left_front_dir_pin=3; 

// int Right_middle_motor_pin=22;
// int Right_middle_dir_pin=23;

// int Left_middle_motor_pin=5;
// int Left_middle_dir_pin=4;

// int Right_back_motor_pin=36;
// int Right_back_dir_pin=37;

// int Left_back_motor_pin=7;
// int Left_back_dir_pin=6;

// Cytron R_F( Right_front_motor_pin, Right_front_dir_pin, 0);
// Cytron L_F( Left_front_motor_pin, Left_front_dir_pin, true);

// Cytron R_M( Right_middle_motor_pin, Right_middle_dir_pin, 0);
// Cytron L_M( Left_middle_motor_pin, Left_middle_dir_pin, false);

// Cytron R_B( Right_back_motor_pin, Right_back_dir_pin, 0);
// Cytron L_B( Left_back_motor_pin, Left_back_dir_pin, 0);

// rcl_publisher_t publisher;
// rcl_subscription_t subscription;

// irc_interfaces__msg__Ps4 ps4_msg;
// geometry_msgs__msg__Twist twist_msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;
// bool micro_ros_init_successful;

// long last_nav_cmd_time = 0;

// enum states {
//   WAITING_AGENT,
//   AGENT_AVAILABLE,
//   AGENT_CONNECTED,
//   AGENT_DISCONNECTED
// } state;

// float_t linear_x = 0.0;
// float_t angular_z = 0.0;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// #define EXECUTE_EVERY_N_MS(MS, X)  do { \
//   static volatile int64_t init = -1; \
//   if (init == -1) { init = uxr_millis();} \
//   if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
// } while (0)\

// // Error handle loop
// void error_loop() {
//   while(1) {
//     delay(100);
//   }
// }

// void subscription_callback(const void * msgin)
// {
//   if (msgin == NULL) {
//     return;
//   }

//   const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;

//   twist_msg = *msg; 
  
//   linear_x = twist_msg.linear.x;
//   angular_z = twist_msg.angular.z;
//   digitalWrite(13, 1); // Indicate message received

//   last_nav_cmd_time = uxr_millis();

// }

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   (void)last_call_time;
//   if (timer != NULL) {
//     float_t right_speed = linear_x + angular_z;
//     float_t left_speed = linear_x - angular_z;
    
//     R_F.rotate(right_speed);
//     R_M.rotate(right_speed);
//     R_B.rotate(right_speed);

//     L_F.rotate(left_speed);
//     L_M.rotate(left_speed);
//     L_B.rotate(left_speed);

//   }
// }

// bool create_entities()
// {
//   allocator = rcl_get_default_allocator();

//   // create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, "micro_ros_chassis", "", &support));

//   // create publisher
//   RCCHECK(rclc_publisher_init_default(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(irc_interfaces, msg, Ps4),
//     "micro_ros_platformio_node_publisher"));

//   RCCHECK(rclc_subscription_init_default(
//     &subscription,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
//     "/cmd_vel"));

//   // create timer,
//   const unsigned int timer_timeout = 0.05;
//   RCCHECK(rclc_timer_init_default(
//     &timer,
//     &support,
//     RCL_MS_TO_NS(timer_timeout),
//     timer_callback));
        
//   // create executor
//   executor = rclc_executor_get_zero_initialized_executor();
//   RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));
//   RCCHECK(rclc_executor_add_subscription(
//     &executor,
//     &subscription,
//     &twist_msg,
//     &subscription_callback,
//     ON_NEW_DATA));

//   return true;
// }

// void destroy_entities()
// {
//   rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
//   (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

//   rcl_subscription_fini(&subscription, &node);
//   rcl_publisher_fini(&publisher, &node);
//   rcl_timer_fini(&timer);
//   rclc_executor_fini(&executor);
//   rcl_node_fini(&node);
//   rclc_support_fini(&support);
// }


// void setup() {
//   // Configure serial transport
//   delay(2000);
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);
//   delay(2000);

//   state = WAITING_AGENT;
    
//   digitalWrite(13, HIGH); // Indicate micro-ROS is running
// }

// void loop() {
//   // delay(100);
//   switch (state) {
//     case WAITING_AGENT:
//       EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
//       break;
//     case AGENT_AVAILABLE:
//       state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
//       if (state == WAITING_AGENT) {
//         destroy_entities();
//       };
//       break;
//     case AGENT_CONNECTED:
//       EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
//       if (state == AGENT_CONNECTED) {
//         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(25));
//       }
//       break;
//     case AGENT_DISCONNECTED:
//       destroy_entities();
//       state = WAITING_AGENT;
//       break;
//     default:
//       break;
//   }

//   if(state == WAITING_AGENT || state == AGENT_AVAILABLE || state == AGENT_DISCONNECTED || (millis() - last_nav_cmd_time) > 1000) {
//     linear_x = 0;
//     angular_z = 0;
//   }

//   // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
// }





#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Wire.h>
#include <Cytron.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <irc_interfaces/msg/ps4.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif


rcl_subscription_t subscription_rover;
rcl_subscription_t subscription_arm;
rcl_publisher_t ps_data_check;

irc_interfaces__msg__Ps4 ps4_msg_arm;
irc_interfaces__msg__Ps4 ps4_msg_rover;
std_msgs__msg__UInt8MultiArray ps_data;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define ESP_ADDR 8         // ESP32 Rover address
#define ARM_TEENSY_ADDR 9  // Arm Teensy I2C address
#define PWM_MAX 100
#define DEADZONE 25
// #define size 7

bool pwmFlag = false;
long st = 0;

// LEFT MOTORS
#define LF_PWM 2
#define LF_DIR 3
#define LM_PWM 5
#define LM_DIR 4
#define LR_PWM 7
#define LR_DIR 6

// RIGHT MOTORS
#define RF_PWM 10
#define RF_DIR 11
#define RM_PWM 22
#define RM_DIR 23
#define RR_PWM 36
#define RR_DIR 37

// Motor Objects
Cytron motorLF(LF_PWM, LF_DIR, 1, PWM_MAX);
Cytron motorLM(LM_PWM, LM_DIR, 0, PWM_MAX);
Cytron motorLR(LR_PWM, LR_DIR, 0, PWM_MAX);

Cytron motorRF(RF_PWM, RF_DIR, 0, PWM_MAX);
Cytron motorRM(RM_PWM, RM_DIR, 0, PWM_MAX);
Cytron motorRR(RR_PWM, RR_DIR, 0, PWM_MAX);

uint8_t ps_data_buffer[4];  // BUTTONS, NAV_X, NAV_Y, RESERVED

uint8_t rx_data[8];
uint8_t nav_data[3];  // BUTTONS,NAV_X, NAV_Y
uint8_t arm_data[5];  // Remaining 5 bytes of arm

int8_t x = 0, y = 0;
int8_t prev_x, prev_y = 0;
uint8_t buttons;

long last_nav_cmd_time = 0;

int8_t buttons_rover, buttons_arm;

bool up_arm, right_arm, down_arm, left_arm;
bool cross_arm, circle_arm, triangle_arm, square_arm;

bool up, right, down, left;
bool cross, circle, triangle, square;

double L_joystick_x_arm;
double L_joystick_y_arm;
double R_joystick_y_arm;

double L_joystick_x_rover;
double L_joystick_y_rover;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback_arm(const void * msgin)
{
  if (msgin == NULL) {
    return;
  }
  
  const irc_interfaces__msg__Ps4 * msg = (const irc_interfaces__msg__Ps4 *) msgin;
  
  // for (size_t i = 0; i < 6; i++) {
    //   joint_pwm.data.data[i] = msg->data.data[i];
    // }
    
    L_joystick_x_arm = uint8_t(msg->ps4_data_analog[0]);
    L_joystick_y_arm = uint8_t(msg->ps4_data_analog[1]);
    R_joystick_y_arm = uint8_t(msg->ps4_data_analog[4]);

    up_arm = msg->ps4_data_buttons[13];
    right_arm = msg->ps4_data_buttons[16];
    down_arm = msg->ps4_data_buttons[14];
    left_arm = msg->ps4_data_buttons[15];
    
    triangle_arm = msg->ps4_data_buttons[2];
    circle_arm = msg->ps4_data_buttons[1];  
    cross_arm = msg->ps4_data_buttons[0];
    square_arm = msg->ps4_data_buttons[3];
    

    buttons_arm = up_arm << 0 | right_arm << 1 | down_arm << 2 | left_arm << 3 | triangle_arm << 4 | circle_arm << 5 | cross_arm << 6 | square_arm << 7;
    
    // motorRoll.rotate(50);
    // digitalWrite(13, HIGH);

    // last_arm_cmd_time = millis();
}

void subscription_callback_rover(const void * msgin)
{
  if (msgin == NULL) {
    return;
  }
  
  const irc_interfaces__msg__Ps4 * msg = (const irc_interfaces__msg__Ps4 *) msgin;
  
  // for (size_t i = 0; i < 6; i++) {
    //   joint_pwm.data.data[i] = msg->data.data[i];
    // }
    
    L_joystick_x_rover = uint8_t(msg->ps4_data_analog[0]);
    L_joystick_y_rover = uint8_t(msg->ps4_data_analog[1]);
    // R_joystick_y = double(msg->ps4_data_analog[4]);
    
    up_arm = msg->ps4_data_buttons[13];
    right_arm = msg->ps4_data_buttons[16];
    down_arm = msg->ps4_data_buttons[14];
    left_arm = msg->ps4_data_buttons[15];
    
    triangle_arm = msg->ps4_data_buttons[2];
    circle_arm = msg->ps4_data_buttons[1];  
    cross_arm = msg->ps4_data_buttons[0];
    square_arm = msg->ps4_data_buttons[3];
    
    buttons_rover = up_arm << 0 | right_arm << 1 | down_arm << 2 | left_arm << 3 | triangle_arm << 4 | circle_arm << 5 | cross_arm << 6 | square_arm << 7;
    
    ps_data.data.data[0] = buttons_rover;
    ps_data.data.data[1] = L_joystick_x_rover;
    ps_data.data.data[2] = L_joystick_y_rover;

    RCSOFTCHECK(rcl_publish(&ps_data_check, &ps_data, NULL));

    // motorRoll.rotate(50);
    // digitalWrite(13, HIGH);

    last_nav_cmd_time = millis();
}


//Functions
void stopMotors() {  //stops all motors
  motorLF.rotate(0);
  motorLM.rotate(0);
  motorLR.rotate(0);
  motorRF.rotate(0);
  motorRM.rotate(0);
  motorRR.rotate(0);
}

void setLeftMotors(int speed) {  //lett motor speed
  motorLF.rotate(speed);
  motorLM.rotate(speed);
  motorLR.rotate(speed);
}

void setRightMotors(int speed) {  //right motor speed
  motorRF.rotate(speed);
  motorRM.rotate(speed);
  motorRR.rotate(speed);
}

bool create_entities()
{
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_rover", "", &support));

  RCCHECK(rclc_subscription_init(
    &subscription_rover,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(irc_interfaces, msg, Ps4),
    "/ps4_data_rover",
    &qos_profile));

  RCCHECK(rclc_subscription_init(
    &subscription_arm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(irc_interfaces, msg, Ps4),
    "/ps4_data_arm",
    &qos_profile));

  RCCHECK(rclc_publisher_init_default(
    &ps_data_check,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "/ps_data"));
        
  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscription_arm,
    &ps4_msg_arm,
    &subscription_callback_arm,
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscription_rover,
    &ps4_msg_rover,
    &subscription_callback_rover,
    ON_NEW_DATA));

    ps_data.data.size = 4;
    ps_data.data.capacity = 4;
    ps_data.data.data = ps_data_buffer;

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscription_arm, &node);
  rcl_subscription_fini(&subscription_rover, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}


//-----------------------------------------------------
void setup() {

  delay(2000);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);


  state = WAITING_AGENT;

  
  Serial7.begin(115200);
  // Serial5.begin(38400);  // GNSS UART (pins 20 RX, 21 TX)
  Wire1.begin();
  Wire2.begin();

  stopMotors();

 

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.println("Ganpati Bappa Morya!!");
  st = millis();
}

//-----------------------------------------------------
void loop() {
  
 switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(25));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if(state == WAITING_AGENT || state == AGENT_AVAILABLE || state == AGENT_DISCONNECTED || (millis() - last_nav_cmd_time) > 2000) {
    L_joystick_x_rover = 0;
    L_joystick_y_rover = 0;

    buttons_rover = 0;

    L_joystick_x_arm = 0;
    L_joystick_y_arm = 0;
    R_joystick_y_arm = 0;

    buttons_arm = 0;
  }
  
  // Extract NAV bytes
  nav_data[0] = buttons_rover;  // ASTRO BUTTONS
  nav_data[1] = L_joystick_x_rover;  // X
  nav_data[2] = L_joystick_y_rover;  // Y

  buttons = nav_data[0];

  arm_data[0] = buttons_arm;
  arm_data[1] = L_joystick_x_arm;
  arm_data[2] = L_joystick_y_arm;
  arm_data[3] = R_joystick_y_arm;
  arm_data[4] = buttons_rover; 

  // Send arm data to arm Teensy
  Wire1.beginTransmission(ARM_TEENSY_ADDR);
  Wire1.write(arm_data, 5);
  Wire1.endTransmission();

  // Convert joystick values
  int X = nav_data[1];
  int Y = nav_data[2];

  x = (X > 127) ? X - 256 : X;
  y = (Y > 127) ? Y - 256 : Y;

  // Deadzone
  if (abs(x) < DEADZONE) x = 0;
  if (abs(y) < DEADZONE) y = 0;

  // Differential drive mixing
  int leftSpeed = constrain(map(y + x, -127, 127, -PWM_MAX, PWM_MAX), -PWM_MAX, PWM_MAX);
  int rightSpeed = constrain(map(y - x, -127, 127, -PWM_MAX, PWM_MAX), -PWM_MAX, PWM_MAX);

  if (x == 0 && y == 0) {
    stopMotors();
  }
  //  else if (!pwmFlag) {
  //   // stopMotors();
  //   x = prev_x;
  //   y = prev_y;
  //   Serial.println("asdfghjk");
  // } 
  else {
    setLeftMotors(leftSpeed);
    setRightMotors(rightSpeed);
  }

  prev_x = x;
  prev_y = y;


  // delay(5);
}

