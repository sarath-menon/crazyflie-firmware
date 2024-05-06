#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"


// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));

    // Remove the DEBUG_PRINT.
    // DEBUG_PRINT("Hello World!\n");
  }
}

// The new controller goes here --------------------------------------------
// Move the includes to the the top of the file if you want to
#include "controller.h"

// Call the PID controller in this example to make it possible to fly. When you implement you own controller, there is
// no need to include the pid controller.
#include "controller_pid.h"

void controllerOutOfTreeInit() {
  // Initialize your controller data here...

  // Call the PID controller instead in this example to make it possible to fly
  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Implement your controller here...

  // Call the PID controller instead in this example to make it possible to fly
  controllerPid(control, setpoint, sensors, state, tick);
}
