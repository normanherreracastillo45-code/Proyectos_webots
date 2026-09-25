/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  A really simple controller which moves the MyBot robot,
 *               avoids the walls and turns the camera on.
 */

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <webots/gps.h>
#include <webots/camera_recognition_object.h>
#include <string.h>
#include <stdio.h>
#define SPEED 6
#define TIME_STEP 64

int main() {
  wb_robot_init(); /* necessary to initialize webots stuff */

  /* Get and enable the distance sensors. */
  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  /* get and enable camera */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 2 * TIME_STEP);
  wb_camera_recognition_enable(camera, 2 * TIME_STEP); /* <-- AGREGA ESTA LÍNEA */
  /* Obtener y habilitar el GPS */
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    /* Get distance sensor values */
    double ds0_value = wb_distance_sensor_get_value(ds0);
    double ds1_value = wb_distance_sensor_get_value(ds1);

    /* This is used to refresh the camera. */
    wb_camera_get_image(camera);
    
    /* --- INICIO DEL BLOQUE NUEVO --- */
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    if (number_of_objects > 0) {
      const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
      const double *gps_values = wb_gps_get_values(gps); /* <-- Lee el GPS */
      
      for (int i = 0; i < number_of_objects; ++i) {
        if (strstr(objects[i].model, "pedestrian") != NULL) {
          printf("Objeto humano detectado en -> X: %.2lf, Y: %.2lf, Z: %.2lf\n", gps_values[0], gps_values[1], gps_values[2]);
        } else if (strstr(objects[i].model, "tree") != NULL) {
          printf("Objeto árbol detectado en -> X: %.2lf, Y: %.2lf, Z: %.2lf\n", gps_values[0], gps_values[1], gps_values[2]);
        }
      }
    }
    /* --- FIN DEL BLOQUE NUEVO --- */

    /* Compute the motor speeds */
    double left_speed, right_speed;
    if (ds1_value > 500) {
      /*
       * If both distance sensors are detecting something, this means that
       * we are facing a wall. In this case we need to move backwards.
       */
      if (ds0_value > 500) {
        left_speed = -SPEED;
        right_speed = -SPEED / 2;
      } else {
        /*
         * We turn proportionnaly to the sensors value because the
         * closer we are from the wall, the more we need to turn.
         */
        left_speed = -ds1_value / 100;
        right_speed = (ds0_value / 100) + 0.5;
      }
    } else if (ds0_value > 500) {
      left_speed = (ds1_value / 100) + 0.5;
      right_speed = -ds0_value / 100;
    } else {
      /*
       * If nothing was detected we can move forward at maximal speed.
       */
      left_speed = SPEED;
      right_speed = SPEED;
    }

    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  return 0;
}
