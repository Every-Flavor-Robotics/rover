#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "ros_comms.h"

ROS_COMMS::wifi_config_t wifi_params;
wifi_params.ssid = WIFI_SSID;
wifi_params.password = WIFI_PASSWORD;
wifi_params.agent_ip = IPAddress(192, 168, 0, 15);
wifi_params.agent_port = 8888;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_timer_t timer;

// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup()
{
  // Configure serial transport

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  ROS_COMMS::init("rear_wheels", wifi_params)

      msg.data = 0;
}

void loop()
{
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}