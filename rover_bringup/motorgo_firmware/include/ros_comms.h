#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

namespace ROS_COMMS
{
rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_timer_t timer;

rcl_node_t node;

rcl_publisher_t left_wheel_vel_pub;
rcl_publisher_t right_wheel_vel_pub;

// Wheel vel callbacks
std::function<float(void)> left_wheel_vel_callback;
std::function<float(void)> right_wheel_vel_callback;
std_msgs__msg__Float32 msg_left;
std_msgs__msg__Float32 msg_right;
//

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

struct wifi_config_t
{
  char* ssid;
  char* password;
  IPAddress agent_ip;
  uint16_t agent_port;
};

// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

void setup_wifi_comms(char* ssid, char* password, IPAddress agent_ip,
                      uint16_t agent_port)
{
  // Configure wifi transport
  //   TODO: Confirm that we are using udp4 transport
  Serial.println("Setting up wifi comms");
  Serial.println(ssid);
  Serial.println(password);
  Serial.println(agent_ip);
  Serial.println(agent_port);
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    msg_left.data = left_wheel_vel_callback();
    msg_right.data = right_wheel_vel_callback();
    RCSOFTCHECK(rcl_publish(&left_wheel_vel_pub, &msg_left, NULL));
    RCSOFTCHECK(rcl_publish(&right_wheel_vel_pub, &msg_right, NULL));
  }
}

void set_left_wheel_vel_callback(std::function<float(void)> callback)
{
  left_wheel_vel_callback = callback;
}

void set_right_wheel_vel_callback(std::function<float(void)> callback)
{
  right_wheel_vel_callback = callback;
}

void init(String node_name, wifi_config_t wifi_config, int publish_rate)
{
  // Set UDP4 as transport
  setup_wifi_comms(wifi_config.ssid, wifi_config.password, wifi_config.agent_ip,
                   wifi_config.agent_port);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &left_wheel_vel_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/rover_base/wheel_velocity/left"));

  // Create publisher for right wheel velocity
  RCCHECK(rclc_publisher_init_best_effort(
      &right_wheel_vel_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/rover_base/wheel_velocity/right"));

  //   // create timer,

  //   Compute timemout in ms from publish rate (hz)
  const unsigned int timer_timeout = 1000 / publish_rate;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void spin() { RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50))); }

}  // namespace ROS_COMMS