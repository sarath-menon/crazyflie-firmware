
#include <math.h>

#include "param.h"
#include "log.h"
#include "position_controller.h"
#include "controller_rls.h"
#include "physicalConstants.h"
#include "platform_defaults.h"
#include "utils.h"
#include "math_lib.h"

// Global state variable used in the
// firmware as the only instance and in bindings
// to hold the default values
static controllerRls_t g_self = {
    .mass = CF_MASS,
    .massThrust = 132000,

    // XY Position PID
    .kp_xy = 0.4,  // P
    .kd_xy = 0.2,  // D
    .ki_xy = 0.05, // I
    .i_range_xy = 2.0,

    // Z Position
    .kp_z = 1.25, // P
    .kd_z = 0.4,  // D
    .ki_z = 0.05, // I
    .i_range_z = 0.4,

    // Attitude
    .kR_xy = 70000, // P
    .kw_xy = 20000, // D
    .ki_m_xy = 0.0, // I
    .i_range_m_xy = 1.0,

    // Yaw
    .kR_z = 60000, // P
    .kw_z = 12000, // D
    .ki_m_z = 500, // I
    .i_range_m_z = 1500,

    // roll and pitch angular velocity
    .kd_omega_rp = 200, // D

    // Helper variables
    .i_error_x = 0,
    .i_error_y = 0,
    .i_error_z = 0,

    .i_error_m_x = 0,
    .i_error_m_y = 0,
    .i_error_m_z = 0,
};

void controllerRlsReset(controllerRls_t *self)
{
  self->i_error_x = 0;
  self->i_error_y = 0;
  self->i_error_z = 0;
  self->i_error_m_x = 0;
  self->i_error_m_y = 0;
  self->i_error_m_z = 0;
}

void controllerRlsInit(controllerRls_t *self)
{
  // copy default values (bindings), or does nothing (firmware)
  *self = g_self;

  // // initialize A,B matrices for the LQR controller
  // initialize_matrices(self->A, self->B, self->mass);

  // Initialize K_star for LQR
  float K_star[M][N] = {
      {0, 0, 0.98, 0, 0, 0.25, 0, 0, 0},
      {0, -3.2, 0, 0, -2.0, 0, 4.0, 0, 0},
      {3.2, 0, 0, 2.0, 0, 0, 0, 4.0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 2.3}};

  for (int i = 0; i < M; i++)
  {
    for (int j = 0; j < N; j++)
    {
      self->K_star[i][j] = K_star[i][j];
    }
  }

  // Initialize each matrix in S_target_aug_all to an identity matrix
  for (int w = 0; w < W_RLS; w++)
  {
    for (int k = 0; k < W_RLS; k++)
    {
      create_identity_matrix((float *)self->S_target_aug_all[w][k], N_OF_INTEREST);
    }
  }

  // Initialize matrix p
  create_identity_matrix((float *)self->P, N_OF_INTEREST);
  for (int i = 0; i < N_OF_INTEREST; i++)
  {
    self->P[i][i] = 1e-5;
  }

  controllerRlsReset(self);
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
  float errorArray[N];
  float control_input[M];

  control->controlMode = controlModeLegacy;

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep))
  {
    return;
  }

  // dt = (float)(1.0f/ATTITUDE_RATE);

  // position, velocity, attitude setpoints
  float setpointArray[9] = {
      setpoint->position.x, setpoint->position.y, setpoint->position.z,
      setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z,
      setpoint->attitude.roll, setpoint->attitude.pitch, setpoint->attitude.yaw};

  // positon, velocity, attitude, attitude rate states
  float stateArray[9] = {
      state->position.x, state->position.y, state->position.z,
      state->velocity.x, state->velocity.y, state->velocity.z,
      state->attitude.roll, state->attitude.pitch, state->attitude.yaw};

  // compute error
  for (int i = 0; i < N; i++)
  {
    errorArray[i] = setpointArray[i] - stateArray[i];
  }

  compute_setpoint_viaLQR(self->K_star, errorArray, state->attitude.yaw, control_input);

  // add feedforward thrust to LQR output
  self->cmd_thrust = control_input[0] + self->mass * GRAVITY_MAGNITUDE;
  self->cmd_roll_rate = control_input[1];
  self->cmd_pitch_rate = control_input[2];
  self->cmd_yaw_rate = control_input[3];

  predict_future_targets(self, setpoint);

  float future_disturbance_feedback[4][1] = {0};
  for (int i = 0; i < W_RLS; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      for (int k = 0; k < N_OF_INTEREST; k++)
      {
        future_disturbance_feedback[j][0] -= self->M_optimal_all[j][k] * self->disturbances_predicted[i][k];
      }
    }
  }

  // add to LQR output
  self->cmd_thrust += future_disturbance_feedback[0][0];
  self->cmd_roll_rate += future_disturbance_feedback[1][0];
  self->cmd_pitch_rate += future_disturbance_feedback[2][0];
  self->cmd_yaw_rate += future_disturbance_feedback[3][0];
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
 * Tunning variables for the full state Rls Controller
 */
PARAM_GROUP_START(ctrlMel)
/**
 * @brief Position P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_xy, &g_self.kp_xy)
/**
 * @brief Position D-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_xy, &g_self.kd_xy)
/**
 * @brief Position I-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_xy, &g_self.ki_xy)
/**
 * @brief Attitude maximum accumulated error (roll and pitch)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_xy, &g_self.i_range_xy)
/**
 * @brief Position P-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_z, &g_self.kp_z)
/**
 * @brief Position D-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_z, &g_self.kd_z)
/**
 * @brief Position I-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_z, &g_self.ki_z)
/**
 * @brief Position maximum accumulated error (vertical z plane)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_z, &g_self.i_range_z)
/**
 * @brief total mass [kg]
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mass, &g_self.mass)
/**
 * @brief Force to PWM stretch factor
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, massThrust, &g_self.massThrust)
/**
 * @brief Attitude P-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kR_xy, &g_self.kR_xy)
/**
 * @brief Attitude P-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kR_z, &g_self.kR_z)
/**
 * @brief Attitude D-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kw_xy, &g_self.kw_xy)
/**
 * @brief Attitude D-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kw_z, &g_self.kw_z)
/**
 * @brief Attitude I-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_m_xy, &g_self.ki_m_xy)
/**
 * @brief Attitude I-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_m_z, &g_self.ki_m_z)
/**
 * @brief Angular velocity D-Gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_omega_rp, &g_self.kd_omega_rp)
/**
 * @brief Attitude maximum accumulated error (roll and pitch)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_m_xy, &g_self.i_range_m_xy)
/**
 * @brief Attitude maximum accumulated error (yaw)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_m_z, &g_self.i_range_m_z)
PARAM_GROUP_STOP(ctrlMel)

/**
 * Logging variables for the command and reference signals for the
 * Rls controller
 */
LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, cmd_thrust, &g_self.cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &g_self.cmd_roll_rate)
LOG_ADD(LOG_FLOAT, cmd_pitch, &g_self.cmd_pitch_rate)
LOG_ADD(LOG_FLOAT, cmd_yaw, &g_self.cmd_yaw_rate)
LOG_ADD(LOG_FLOAT, r_roll, &g_self.r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &g_self.r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &g_self.r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &g_self.accelz)
LOG_ADD(LOG_FLOAT, zdx, &g_self.z_axis_desired.x)
LOG_ADD(LOG_FLOAT, zdy, &g_self.z_axis_desired.y)
LOG_ADD(LOG_FLOAT, zdz, &g_self.z_axis_desired.z)
LOG_ADD(LOG_FLOAT, i_err_x, &g_self.i_error_x)
LOG_ADD(LOG_FLOAT, i_err_y, &g_self.i_error_y)
LOG_ADD(LOG_FLOAT, i_err_z, &g_self.i_error_z)
LOG_GROUP_STOP(ctrlMel)
