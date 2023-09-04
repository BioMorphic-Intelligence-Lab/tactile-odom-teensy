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
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Define Constants
constexpr uint8_t N = 2; // Number of joints
constexpr double LINEAR_TICKS_2_M  = 0.06/1024.0; // Conversion factor from ticks to lin dist
constexpr double ROT_TICKS_2_RAD  = 2 * M_PI/4096.0; // Conversion factor from ticks to rad
constexpr unsigned int TIMER_TIMEOUT = RCL_S_TO_NS(1.0 / 100.0); // 1/f_pub


// Define 
rcl_publisher_t publisher;
sensor_msgs__msg__JointState state;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

double last_lin_pos;
uint64_t last_ts;
uint16_t offset_lin, offset_rot;

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
    state.header.stamp.sec = RCL_NS_TO_S(now);
    state.header.stamp.nanosec = now - RCL_S_TO_NS(state.header.stamp.sec);
    
    // Write the current joint positions
    state.position.data[0] = LINEAR_TICKS_2_M * (analogRead(A9) - offset_lin);
    state.position.data[1] = ROT_TICKS_2_RAD * (read_current_pos(1) - offset_rot);

    // Write the current joint velocities
    state.velocity.data[0] = (state.position.data[0]-last_lin_pos) / (now - last_ts) * 1e9;
    state.velocity.data[1] = ROT_TICKS_2_RAD * read_current_vel(1);

    last_ts = now;
    last_lin_pos = state.position.data[0];
    
    // Publish
    RCSOFTCHECK(rcl_publish(&publisher, &state, NULL));
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

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_state"));

  // create timer,
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    TIMER_TIMEOUT,
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Init joint message
  state.position.size = N;
  state.position.capacity = N;
  state.position.data = (double *) malloc(N * sizeof(double));
  
  state.velocity.size = N;
  state.velocity.capacity = N;
  state.velocity.data = (double *) malloc(N * sizeof(double));

  state.name.size = N;
  state.name.capacity = N;
  state.name.data = (rosidl_runtime_c__String*) malloc(N*sizeof(rosidl_runtime_c__String)); 

  state.name.data[0].capacity = 10;
  state.name.data[0].data = (char *) malloc(10 * sizeof(char));
  sprintf(state.name.data[0].data, "linear");
  state.name.data[0].size = strlen(state.name.data[0].data);
   
  state.name.data[1].capacity = 10;
  state.name.data[1].data = (char *) malloc(10 * sizeof(char));
  sprintf(state.name.data[1].data, "revolute");
  state.name.data[1].size = strlen(state.name.data[1].data);

  // Init last pos and ts
  last_lin_pos = LINEAR_TICKS_2_M * analogRead(A9);
  last_ts = rmw_uros_epoch_nanos();

  // Init offsets
  offset_lin = analogRead(A9);
  offset_rot = read_current_pos(1);
}

void loop() {
  // On each we execute once
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
