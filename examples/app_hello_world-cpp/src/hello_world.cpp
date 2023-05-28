/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 * hello_world.c - App layer application of a simple hello world debug print
 * every 2 seconds.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <cassert>
#include <string>

extern "C" {
#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "log.h"
#include "param.h"
#include "stabilizer.h"

#include "controller.h"
#include "sensors.h"
}

#define DEBUG_MODULE "HELLOWORLD"

// State variables for the stabilizer
static sensorData_t sensorData;

class MyClass {
public:
  int myNum;
  std::string myString;
};

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  DEBUG_PRINT(
      "This is the App layer example of the internal log param api...\n");
  logVarId_t idYaw = logGetVarId("stateEstimate", "yaw");
  float yaw = 0.0f;

  // Get the logging data
  yaw = logGetFloat(idYaw);
  DEBUG_PRINT("Yaw is now: %f deg\n", (double)yaw);

  controllerInit(ControllerTypeAutoSelect);

  while (1) {
    vTaskDelay(M2T(2000));
    DEBUG_PRINT("Hello World!\n");

    // update sensorData struct (for logging variables)
    sensorsAcquire(&sensorData);
  }
}
