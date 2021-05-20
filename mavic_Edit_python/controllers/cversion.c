/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Simplistic drone control:
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use a cubic function applied on the vertical difference to stabilize the robot vertically.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

#include <webots/lidar.h>
#include <webots/distance_sensor.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char **argv)
{
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices.
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
  // WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");  // Not used in this example.

  // init lidar
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, timestep);

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};

  //Distance Sensor
  WbDeviceTag distance_sensor = wb_robot_get_device("distancesensor");
  wb_distance_sensor_enable(distance_sensor, timestep);

  int m;
  for (m = 0; m < 4; ++m)
  {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  // Display the welcome message.
  printf("Start the drone...\n");

  // Wait one second.
  while (wb_robot_step(timestep) != -1)
  {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  // Display manual control message.
  /*printf("You can control the drone with your computer keyboard:\n");
  printf("- 'up': move forward.\n");
  printf("- 'down': move backward.\n");
  printf("- 'right': turn right.\n");
  printf("- 'left': turn left.\n");
  printf("- 'shift + up': increase the target altitude.\n");
  printf("- 'shift + down': decrease the target altitude.\n");
  printf("- 'shift + right': strafe right.\n");
  printf("- 'shift + left': strafe left.\n");*/

  // Constants, empirically found.
  const double k_vertical_thrust = 68.5; // with this thrust, the drone lifts.
  const double k_vertical_offset = 0.6;  // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vertical_p = 3.0;       // P constant of the vertical PID.
  const double k_roll_p = 50.0;          // P constant of the roll PID.
  const double k_pitch_p = 30.0;         // P constant of the pitch PID.

  // Variables.
  double target_altitude = 13.5; // The target altitude. Can be changed by the user.

  int counter = 0;

  // Main loop
  while (wb_robot_step(timestep) != -1)
  {
    const double time = wb_robot_get_time(); // in seconds.

    // Retrieve robot position using the sensors.
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0] + M_PI / 2.0;
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double altitude = wb_gps_get_values(gps)[1];
    const double x_coordinate = wb_gps_get_values(gps)[0];
    const double z_coordinate = wb_gps_get_values(gps)[2];
    const double roll_acceleration = wb_gyro_get_values(gyro)[0];
    const double pitch_acceleration = wb_gyro_get_values(gyro)[1];

    // Blink the front LEDs alternatively with a 1 second rate.
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_acceleration);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_acceleration);

    // Transform the keyboard input to disturbances on the stabilization algorithm.
    double roll_disturbance = 0.0;  // Positive move left, negative move right
    double pitch_disturbance = 0.0; //Go forward for positive
    double yaw_disturbance = 0.0;   //Rotate horizontally (+ve clockwise)

    double dist = wb_distance_sensor_get_value(distance_sensor);
    double direction_multiplier = 1.0;

    if (dist > 200)
    {
      direction_multiplier = -1.0;
      printf("Changing Direction \n");

      target_altitude = 0;
    }

    //Algorithm here
    if (fabs(altitude - target_altitude) < 0.5)
    {

      //Save image every second at height 3
      if ((time - (int)time) == 0)
      {
        char x_string[20];
        char z_string[20];
        char y_string[20];
        char final_filename[100];
        sprintf(x_string, "%.1f", x_coordinate);
        sprintf(y_string, "%.1f", altitude);
        sprintf(z_string, "%.1f", z_coordinate);

        snprintf(final_filename, sizeof(final_filename), "%s_%s_%s.jpg", x_string, y_string, z_string);

        wb_camera_save_image(camera, final_filename, 100);
        printf("Image Saved with name %s \n", final_filename);
        counter++;
      }

      if ((((int)time) % 3) == 0)
      {
        wb_lidar_enable_point_cloud(lidar);

        wb_lidar_get_point_cloud(lidar);
      }
      else
      {
        wb_lidar_disable_point_cloud(lidar);
      }

      //travel right as long as wall is detected
      if (dist < 100)
      {

        yaw_disturbance = -0.0002; //adjust yaw a bit

        printf("%0.2f Right \n", dist);
        roll_disturbance = -1.5;

        printf("%0.2f Fallback \n", dist);
        pitch_disturbance = -1;
      }
      else if (dist > 100 && direction_multiplier != -1)
      {
        //go closer if wall is far away
        printf("%0.2f Front \n", dist);
        pitch_disturbance = 1;
      }
    }
    else
    {
      printf("%0.2f go to altitude %0.2f \n", dist, target_altitude);
    }

    // Compute the roll, pitch, yaw and vertical inputs.
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Actuate the motors taking into consideration all the computed inputs.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
