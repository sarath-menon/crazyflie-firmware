/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * internal_log_param_api.c - App layer application of the internal log
 *  and param api
 */

#define DEBUG_MODULE "INTERNLOGPARAM"

extern "C" {
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
}

void appMain() {
  DEBUG_PRINT(
      "This is the App layer example of the internal log param api...\n");
  logVarId_t idYaw = logGetVarId("stateEstimate", "yaw");

  logVarId_t id_gyro_x = logGetVarId("gyro", "x");
  logVarId_t id_gyro_y = logGetVarId("gyro", "x");
  logVarId_t id_gyro_z = logGetVarId("gyro", "x");

  float yaw{};

  float gyro_x{};
  float gyro_y{};
  float gyro_z{};

  paramVarId_t idEstimator = paramGetVarId("stabilizer", "estimator");
  uint8_t estimator_type = 0;

  while (1) {
    vTaskDelay(M2T(100));

    // Get the logging data
    yaw = logGetFloat(idYaw);

    gyro_x = logGetFloat(id_gyro_x);
    gyro_y = logGetFloat(id_gyro_y);
    gyro_z = logGetFloat(id_gyro_z);

    // DEBUG_PRINT("Yaw is now: %f deg\n", (double)yaw);

    DEBUG_PRINT("gyro is now: %.3f  %.3f  %.3f \n", (double)gyro_x,
                (double)gyro_y, (double)gyro_z);

    // // Get parameter value
    // estimator_type = paramGetInt(idEstimator);
    // DEBUG_PRINT("Estimator type is now: %d deg\n", estimator_type);

    // // Set a parameter value
    // //  Note, this will influence the flight quality if you change estimator
    // uint8_t new_value = 2;
    // paramSetInt(idEstimator, new_value);
  }
}