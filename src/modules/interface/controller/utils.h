#ifndef __CONTROLLER_RLS_UTILS_H__
#define __CONTROLLER_RLS_UTILS_H__

#include "controller_rls.h"

void initialize_matrices(float A[N_][N_], float B[N_][M_], float m);

void compute_setpoint_viaLQR(float K_star[M_][N_], float error_inertial[N_], float curr_yaw, float u[M_]);

void predict_future_targets(controllerRls_t *self, const setpoint_t *setpoint);

void RLS_update(controllerRls_t *self);

float thrust_newton_to_cmd_light(float thrust);

#endif //__CONTROLLER_RLS_UTILS_H__
