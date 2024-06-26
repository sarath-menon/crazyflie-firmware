#ifndef __CONTROLLER_RLS_UTILS_H__
#define __CONTROLLER_RLS_UTILS_H__

#include "controller_rls.h"

void initialize_matrices(float A[N][N], float B[N][M], float m);

void compute_setpoint_viaLQR(float K_star[M][N], float error_inertial[N], float curr_yaw, float u[M]);

// void predict_future_targets(controllerRls_t *self, const setpoint_t *setpoint);

// void RLS_update(controllerRls_t *self);

#endif //__CONTROLLER_RLS_UTILS_H__
