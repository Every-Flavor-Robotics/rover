#include <Arduino.h>
#include <Ticker.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>

// Import lpf
#include "common/lowpass_filter.h"
#include "motorgo_mini.h"

// MotorGo Config
MotorGo::MotorGoMini* motorgo;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

MotorGo::PIDParameters position_pid_params;

// MotorGo loop task config
TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
void loop_foc(void* pvParameters);

// // Add low pass filter for commands
// LowPassFilter left_command_filter(0.01);
// LowPassFilter right_command_filter(0.01);

// UDP configuration
WiFiUDP udp;
IPAddress agent_ip = IPAddress(192, 168, 0, 15);
uint16_t agent_port = 8008;
int pub_freq = 80;

float left = 0.0;
float right = 0.0;

bool enabled = false;

// Make a packed struct for the wheel velocities
const int ODOMETRY_DATA_SIZE = 4 * sizeof(float);
typedef union
{
  struct
  {
    float left_position;
    float right_position;
    float left_velocity;
    float right_velocity;
  } __attribute__((packed));

  uint8_t raw[ODOMETRY_DATA_SIZE];
} odometry_data_t;

const int WHEEL_VELOCITIES_COMMANDS_SIZE = 2 * sizeof(float);
typedef union
{
  struct
  {
    float left;
    float right;
  } __attribute__((packed));

  uint8_t raw[WHEEL_VELOCITIES_COMMANDS_SIZE];
} wheel_velocities_commands_t;

float left_wheel_vel_callback()
{
  left += 0.01;
  return left;
}

float right_wheel_vel_callback()
{
  right += 0.02;
  return right;
}

// Timer for sending wheel velocities
Ticker publish_timer;

void send_wheel_velocities()
{
  // Create odometry data message
  odometry_data_t odometry;
  odometry.left_position = motorgo->get_ch0_position();
  odometry.right_position = motorgo->get_ch1_position();
  odometry.left_velocity = motorgo->get_ch0_velocity();
  odometry.right_velocity = motorgo->get_ch1_velocity();

  udp.beginPacket(agent_ip, agent_port);
  udp.write(odometry.raw, ODOMETRY_DATA_SIZE);
  udp.endPacket();
}

void setup()
{
  Serial.begin(115200);
  delay(3000);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  motor_params_ch0.pole_pairs = 11;
  motor_params_ch0.power_supply_voltage = 8.0;
  motor_params_ch0.voltage_limit = 8.0;
  motor_params_ch0.current_limit = 300;
  motor_params_ch0.velocity_limit = 100;
  motor_params_ch0.calibration_voltage = 1.0;
  motor_params_ch0.reversed = false;

  motor_params_ch1 = motor_params_ch0;
  motor_params_ch1.reversed = true;

  motorgo = new MotorGo::MotorGoMini();

  motorgo->init_ch0(motor_params_ch0, false);
  motorgo->init_ch1(motor_params_ch1, false);

  position_pid_params.lpf_time_constant = 0.01;

  velocity_pid_params_ch0.p = 7;
  velocity_pid_params_ch0.i = 0.10;
  velocity_pid_params_ch0.d = 0.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.01;

  velocity_pid_params_ch1.p = 7;
  velocity_pid_params_ch1.i = 0.10;
  velocity_pid_params_ch1.d = 0.0;
  velocity_pid_params_ch1.lpf_time_constant = 0.01;

  motorgo->set_velocity_controller_ch0(velocity_pid_params_ch0);
  motorgo->set_velocity_controller_ch1(velocity_pid_params_ch1);

  motorgo->set_position_controller_ch0(position_pid_params);
  motorgo->set_position_controller_ch1(position_pid_params);

  motorgo->set_control_mode_ch0(MotorGo::ControlMode::Velocity);
  motorgo->set_control_mode_ch1(MotorGo::ControlMode::Velocity);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println(WiFi.localIP());

  // Setup UDP connection
  udp.begin(agent_port);

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  // Attach timer for sending wheel velocities
  publish_timer.attach_ms(1000 / pub_freq, send_wheel_velocities);

  // Enable motors
  motorgo->enable_ch0();
  motorgo->enable_ch1();
  enabled = true;
}

void loop_foc(void* pvParameters)
{
  Serial.print("Loop FOC running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    motorgo->loop_ch0();
    motorgo->loop_ch1();

    //   Print loop frequency
    esp_task_wdt_reset();
  }
}

long last_time = millis();
void loop()
{
  // Disable motors if no UDP packets are received for 1 second
  if (millis() - last_time > 1000 && enabled)
  {
    motorgo->disable_ch0();
    motorgo->disable_ch1();
    enabled = false;
  }
  else if (!enabled && millis() - last_time < 1000)
  {
    motorgo->enable_ch0();
    motorgo->enable_ch1();

    enabled = true;
  }

  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    // Buffer to hold incoming data
    char buffer[WHEEL_VELOCITIES_COMMANDS_SIZE];

    // Read the packet into the buffer
    int len = udp.read(buffer, sizeof(buffer));
    if (len > 0)
    {
      last_time = millis();

      // Cast the buffer to the struct
      wheel_velocities_commands_t* wheelCommands =
          reinterpret_cast<wheel_velocities_commands_t*>(buffer);

      //   float left_velocity = wheelCommands->left;
      //   float right_velocity = wheelCommands->right;

      // Set the wheel velocities
      //   motorgo->set_target_velocity_ch0(
      //       left_command_filter(wheelCommands->left));
      //   motorgo->set_target_velocity_ch1(
      //       right_command_filter(wheelCommands->right));

      motorgo->set_target_velocity_ch0(wheelCommands->left);
      motorgo->set_target_velocity_ch1(wheelCommands->right);

      //   // Print the received data
      //   Serial.print("Received - Left: ");
      //   Serial.print(wheelCommands->left);
      //   Serial.print(", Right: ");
      //   Serial.println(wheelCommands->right);
    }
  }

  vTaskDelay(1);
}
