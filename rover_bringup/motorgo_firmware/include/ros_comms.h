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

rcl_node_t node;

rcl_publisher_t left_wheel_vel_pub;
rcl_publisher_t right_wheel_vel_pub;

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
  String ssid;
  String password;
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

void setup_wifi_comms(String ssid, String password, IPAddress agent_ip,
                      uint16_t agent_port)
{
  // Configure wifi transport
  //   TODO: Confirm that we are using udp4 transport
  set_microros_wifi_transports(ssid.c_str(), password.c_str(), agent_ip,
                               agent_port);
  delay(2000);
}

void create_wheel_vel_publishers())
{
  // create publisher
  RCCHECK(rclc_publisher_init_default(publisher, &node, type_support,
                                      topic_name.c_str()));
}

void init(String node_name, wifi_config_t wifi_config)
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
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/rover_base/wheel_velocity/left"));

  // Create publisher for right wheel velocity
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/rover_base/wheel_velocity/right"));
}

}  // namespace ROS_COMMS