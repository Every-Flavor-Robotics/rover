
#include <Arduino.h>
#include <esp_task_wdt.h>

#include "common/lowpass_filter.h"
#include "configurable.h"
#include "motorgo_mini.h"
#include "pid_manager.h"

// Provided by environment variables
// UPDATE THESE VALUES
// String WIFI_SSID = "YOUR_SSID";
// String WIFI_PASSWORD = "YOUR_PASSWORD";

TaskHandle_t motorgo_loop_task;

MotorGo::MotorGoMini* motorgo_mini;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

MotorGo::PIDParameters velocity_controller_params;
LowPassFilter velocity_lpf(0.3);

// // declare PID manager object
MotorGo::PIDManager pid_manager;

// Instantiate pid motorgo pid params
MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

long last_time;
float command = 0.0;
bool on_or_off = false;
LowPassFilter command_filter(0.3);

void enable_motors_callback(bool value)
{
  if (value)
  {
    Serial.println("Enabling motors");
    motorgo_mini->enable_ch0();
    motorgo_mini->enable_ch1();
  }
  else
  {
    Serial.println("Disabling motors");
    motorgo_mini->disable_ch0();
    motorgo_mini->disable_ch1();
  }
}

// Function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

// Start time
long start_time;
unsigned long startTime = micros();  // Start time of the loop
double loop_hz = 0.0;                // Loop frequency

// Task1code: blinks an LED every 1000 ms
void loop_motorgo(void* pvParameters)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    motorgo_mini->loop_ch0();
    motorgo_mini->loop_ch1();

    //   Compute loop frequency
    //   Accumulate loop hz
    loop_hz = 0.9 * loop_hz + 0.1 * 1000000.0 / (micros() - startTime);
    startTime = micros();

    //   Print loop frequency
    freq_println("Loop frequency: " + String(loop_hz), 10);
    esp_task_wdt_reset();
  }
}

void setup()
{
  Serial.begin(115200);

  delay(3000);

  // Setup motor parameters
  motor_params_ch0.pole_pairs = 11;
  motor_params_ch0.power_supply_voltage = 10.0;
  motor_params_ch0.voltage_limit = 5.0;
  motor_params_ch0.current_limit = 300;
  motor_params_ch0.velocity_limit = 100.0;
  motor_params_ch0.calibration_voltage = 1.0;

  motor_params_ch1.pole_pairs = 11;
  motor_params_ch1.power_supply_voltage = 8.0;
  motor_params_ch1.voltage_limit = 5.0;
  motor_params_ch1.current_limit = 300;
  motor_params_ch1.velocity_limit = 100.0;
  motor_params_ch1.calibration_voltage = 1.0;

  velocity_pid_params_ch1.p = 0.5;
  velocity_pid_params_ch1.i = 0.0;
  velocity_pid_params_ch1.d = 0.00;
  velocity_pid_params_ch1.lpf_time_constant = 0.1;

  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  bool enable_foc_studio = false;
  motorgo_mini->init_ch0(motor_params_ch0, calibrate, enable_foc_studio);
  motorgo_mini->init_ch1(motor_params_ch0, calibrate, enable_foc_studio);

  // Instantiate controllers
  motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
  motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch1);

  pid_manager.add_controller(
      "/ch0/velocity", velocity_pid_params_ch0,
      []()
      {
        motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
        motorgo_mini->reset_velocity_controller_ch0();
      });

  pid_manager.add_controller(
      "/ch1/velocity", velocity_pid_params_ch1,
      []()
      {
        motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch1);
        motorgo_mini->reset_velocity_controller_ch1();
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // initialize the PID manager
  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  //   Set closed-loop position mode
  motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Velocity);
  motorgo_mini->set_control_mode_ch1(MotorGo::ControlMode::Velocity);

  //   // Loop motor controls at 200Hz
  //   motorgo_loop_timer.attach_ms(2,
  //                                []()
  //                                {
  //                                  motorgo_mini->loop_ch0();
  //                                  motorgo_mini->loop_ch1();
  //                                });

  start_time = millis();
  startTime = micros();

  xTaskCreatePinnedToCore(
      loop_motorgo,       /* Task function. */
      "Task2",            /* name of task. */
      10000,              /* Stack size of task */
      NULL,               /* parameter of the task */
      1,                  /* priority of the task */
      &motorgo_loop_task, /* Task handle to keep track of created task */
      1);                 /* pin task to core 1 */

  // enable controllers and prepare for the loop
  motorgo_mini->enable_ch0();
  motorgo_mini->enable_ch1();
}

void loop()
{
  if (millis() - last_time > 1000)
  {
    last_time = millis();
    command = 10.0 * on_or_off;

    on_or_off = !on_or_off;
    Serial.println("Command: " + String(command));
  }
  // Run Ch0
  //   motorgo_mini->loop_ch0();
  //   motorgo_mini->loop_ch1();

  float output = command_filter(command);

  motorgo_mini->set_target_velocity_ch0(output);
  motorgo_mini->set_target_velocity_ch1(output);

  vTaskDelay(5 / portTICK_PERIOD_MS);
}
