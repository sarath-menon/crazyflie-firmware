
#include <math.h>

#include "param.h"
#include "log.h"
#include "controller_rls.h"
#include "physicalConstants.h"
#include "platform_defaults.h"
#include "utils.h"
#include "math_lib.h"
#include "attitude_controller.h"

#define ATTITUDE_UPDATE_DT (float)(1.0f / ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

// Global state variable used in the
// firmware as the only instance and in bindings
// to hold the default values
static controllerRls_t g_self = {
    .mass = CF_MASS,
    .massThrust = 132000,

};

static float capAngle(float angle)
{
  float result = angle;

  while (result > 180.0f)
  {
    result -= 360.0f;
  }

  while (result < -180.0f)
  {
    result += 360.0f;
  }

  return result;
}

void controllerRlsInit(controllerRls_t *self)
{
  // copy default values (bindings), or does nothing (firmware)
  *self = g_self;

  // // initialize A,B matrices for the LQR controller
  // initialize_matrices(self->A, self->B, self->mass);

  // Initialize K_star for LQR
  float K_star[M_][N_] = {
      {0, 0, 0.98, 0, 0, 0.25, 0, 0, 0},
      {0, -3.2, 0, 0, -2.0, 0, 4.0, 0, 0},
      {3.2, 0, 0, 2.0, 0, 0, 0, 4.0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 2.3}};

  for (int i = 0; i < M_; i++)
  {
    for (int j = 0; j < N_; j++)
    {
      self->K_star[i][j] = K_star[i][j];
    }
  }

  // // Initialize each matrix in S_target_aug_all to an identity matrix
  // for (int w = 0; w < W_RLS; w++)
  // {
  //   for (int k = 0; k < W_RLS; k++)
  //   {
  //     create_identity_matrix((float *)self->S_target_aug_all[w][k], N_OF_INTEREST);
  //   }
  // }

  // // Initialize matrix p
  // create_identity_matrix((float *)self->P, N_OF_INTEREST);
  // for (int i = 0; i < N_OF_INTEREST; i++)
  // {
  //   self->P[i][i] = 1e-5;
  // }

  attitudeControllerInit(ATTITUDE_UPDATE_DT);
}

bool controllerRlsTest(controllerRls_t *self)
{
  return true;
}

void controllerRls(controllerRls_t *self, control_t *control, const setpoint_t *setpoint,
                   const sensorData_t *sensors,
                   const state_t *state,
                   const stabilizerStep_t stabilizerStep)
{
  float errorArray[N_];
  float control_input[M_];

  control->controlMode = controlModeLegacy;

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep))
  {

    // dt = (float)(1.0f/ATTITUDE_RATE);

    // position, velocity, attitude setpoints
    float setpointArray[N_] = {
        setpoint->position.x, setpoint->position.y, setpoint->position.z,
        setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z,
        setpoint->attitude.roll, setpoint->attitude.pitch, setpoint->attitude.yaw};

    // positon, velocity, attitude, attitude rate states
    float stateArray[N_] = {
        state->position.x, state->position.y, state->position.z,
        state->velocity.x, state->velocity.y, state->velocity.z,
        state->attitude.roll, state->attitude.pitch, state->attitude.yaw};

    // compute error
    for (int i = 0; i < N_; i++)
    {
      errorArray[i] = setpointArray[i] - stateArray[i];
    }

    compute_setpoint_viaLQR(self->K_star, errorArray, state->attitude.yaw, control_input);

    // add feedforward thrust to LQR output
    self->cmd_thrust = control_input[0] + self->mass * GRAVITY_MAGNITUDE;
    rateDesired.roll = control_input[1];
    rateDesired.pitch = control_input[2];
    rateDesired.yaw = control_input[3];

    // predict_future_targets(self, setpoint);

    // float future_disturbance_feedback[4][1] = {0};
    // for (int i = 0; i < W_RLS; i++)
    // {
    //   for (int j = 0; j < 4; j++)
    //   {
    //     for (int k = 0; k < N_OF_INTEREST; k++)
    //     {
    //       future_disturbance_feedback[j][0] -= self->M_optimal_all[j][k] * self->disturbances_predicted[i][k];
    //     }
    //   }
    // }

    // // add to LQR output
    // self->cmd_thrust += future_disturbance_feedback[0][0];
    // self->cmd_roll_rate += future_disturbance_feedback[1][0];
    // self->cmd_pitch_rate += future_disturbance_feedback[2][0];
    // self->cmd_yaw_rate += future_disturbance_feedback[3][0];

    // Attitude rate controller: copied from controller_pid.c
    attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);

    float yawMaxDelta = attitudeControllerGetYawMaxDelta();
    if (yawMaxDelta != 0.0f)
    {
      float delta = capAngle(attitudeDesired.yaw - state->attitude.yaw);
      // keep the yaw setpoint within +/- yawMaxDelta from the current yaw
      if (delta > yawMaxDelta)
      {
        attitudeDesired.yaw = state->attitude.yaw + yawMaxDelta;
      }
      else if (delta < -yawMaxDelta)
      {
        attitudeDesired.yaw = state->attitude.yaw - yawMaxDelta;
      }
    }
    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  actuatorThrust = setpoint->thrust;
  attitudeDesired.roll = setpoint->attitude.roll;
  attitudeDesired.pitch = setpoint->attitude.pitch;

  attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                       attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                       &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

  // Overwrite rateDesired with the setpoint. Also reset the PID to avoid error buildup, which can lead to unstable
  rateDesired.roll = setpoint->attitudeRate.roll;
  rateDesired.pitch = setpoint->attitudeRate.pitch;

  attitudeControllerResetRollAttitudePID();
  attitudeControllerResetPitchAttitudePID();

  // TODO: Investigate possibility to subtract gyro drift.
  attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                                   rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

  attitudeControllerGetActuatorOutput(&control->roll,
                                      &control->pitch,
                                      &control->yaw);

  control->yaw = -control->yaw;

  self->cmd_roll = control->roll;
  self->cmd_pitch = control->pitch;
  self->cmd_yaw = control->yaw;
  self->r_roll = radians(sensors->gyro.x);
  self->r_pitch = -radians(sensors->gyro.y);
  self->r_yaw = radians(sensors->gyro.z);
  self->accelz = sensors->acc.z;

  control->thrust = self->cmd_thrust;

  if (control->thrust == 0)
  {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    self->cmd_roll = control->roll;
    self->cmd_pitch = control->pitch;
    self->cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

void controllerRlsFirmwareInit(void)
{
  controllerRlsInit(&g_self);
}

bool controllerRlsFirmwareTest(void)
{
  return controllerRlsTest(&g_self);
}

void controllerRlsFirmware(control_t *control, const setpoint_t *setpoint,
                           const sensorData_t *sensors,
                           const state_t *state,
                           const stabilizerStep_t stabilizerStep)
{
  controllerRls(&g_self, control, setpoint, sensors, state, stabilizerStep);
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
LOG_ADD(LOG_FLOAT, cmd_thrust, &g_self.cmd_thrust)
 */
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &g_self.cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &g_self.cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &g_self.cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &g_self.r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &g_self.r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &g_self.r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &g_self.accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll, &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch, &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw, &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate, &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate, &rateDesired.yaw)
LOG_GROUP_STOP(controller)
