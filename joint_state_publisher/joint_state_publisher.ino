#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>

#include "servo.h"


// Wrap functions in this to check if ROS is still running throughout
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// Define Constants
constexpr uint8_t N = 2; // Number of joints
constexpr double LINEAR_TICKS_2_M  = 0.06/1024.0; // Conversion factor from ticks to lin dist
constexpr double ROT_TICKS_2_RAD  = 2 * M_PI/4096.0; // Conversion factor from ticks to rad
constexpr unsigned int TIMER_TIMEOUT = RCL_S_TO_NS(1.0 / 25.0); // 1/f_pub


// Define 
rcl_publisher_t publisher;
sensor_msgs__msg__JointState state_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

double last_lin_pos;
uint64_t last_ts;
uint16_t offset_lin, offset_rot;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  //create init_options
  while(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }

  // create node
  while(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }


  // create publisher
  while(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_state") != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }


  // create timer,
  while(rclc_timer_init_default(
    &timer,
    &support,
    TIMER_TIMEOUT,
    timer_callback) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }


  // create executor
  while(rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }

  while(rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void)! rcl_publisher_fini(&publisher, &node);
  (void)! rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  (void)! rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// Loop that runs if an error occurs. Blink the LED to let the user know
void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Callback function for the timer, publish states
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    // Get current timestamp
    int64_t now = rmw_uros_epoch_nanos();
    state_msg.header.stamp.sec = RCL_NS_TO_S(now);
    state_msg.header.stamp.nanosec = now - RCL_S_TO_NS(state_msg.header.stamp.sec);
    
    // Write the current joint positions
    state_msg.position.data[0] = -LINEAR_TICKS_2_M * (analogRead(A9) - offset_lin);
    state_msg.position.data[1] = ROT_TICKS_2_RAD * (read_current_pos(1) - offset_rot);

    // Write the current joint velocities
    state_msg.velocity.data[0] = (state_msg.position.data[0]-last_lin_pos) / (now - last_ts) * 1e9;
    state_msg.velocity.data[1] = ROT_TICKS_2_RAD * read_current_vel(1);

    last_ts = now;
    last_lin_pos = state_msg.position.data[0];
    
    // Publish
    RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
  }
}

void setup() {

  InitServos();
  
  // Init MicroRos
  set_microros_transports();
  
  // Init user com LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(500);

  state = WAITING_AGENT;

  // Init joint message
  state_msg.position.size = N;
  state_msg.position.capacity = N;
  state_msg.position.data = (double *) malloc(N * sizeof(double));
  
  state_msg.velocity.size = N;
  state_msg.velocity.capacity = N;
  state_msg.velocity.data = (double *) malloc(N * sizeof(double));

  state_msg.name.size = N;
  state_msg.name.capacity = N;
  state_msg.name.data = (rosidl_runtime_c__String*) malloc(N*sizeof(rosidl_runtime_c__String)); 

  state_msg.name.data[0].capacity = 10;
  state_msg.name.data[0].data = (char *) malloc(10 * sizeof(char));
  sprintf(state_msg.name.data[0].data, "linear");
  state_msg.name.data[0].size = strlen(state_msg.name.data[0].data);
   
  state_msg.name.data[1].capacity = 10;
  state_msg.name.data[1].data = (char *) malloc(10 * sizeof(char));
  sprintf(state_msg.name.data[1].data, "revolute");
  state_msg.name.data[1].size = strlen(state_msg.name.data[1].data);

  // Init last pos and ts
  last_lin_pos = LINEAR_TICKS_2_M * analogRead(A9);
  last_ts = rmw_uros_epoch_nanos();

  // Init offsets
  offset_lin = analogRead(A9);
  offset_rot = read_current_pos(1);
}

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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_BUILTIN, 1);
  } else {
    digitalWrite(LED_BUILTIN, 0);
  }
}
