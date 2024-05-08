#ifndef __CONTROLLER_RLS_H__
#define __CONTROLLER_RLS_H__

#include "stabilizer_types.h"
#include "math3d.h"

#define N_ 9
#define M_ 4
#define DELTA_T 0.002f
#define W_RLS 5 // prediction horizon
#define N_OF_INTEREST 6

static const int IDX_OF_INTEREST[] = {0, 1, 2, 3, 4, 5};

#define MAX_ERROR_XY 0.3f
#define MAX_ERROR_Z 0.4f
#define MAX_ERROR_YAW 1.0472f // approximately 60 degrees in radians

typedef struct
{
    float mass;
    float massThrust;

    float A[N_][N_];
    float B[N_][M_];

    float K_star[M_][N_];
    // float S_target_aug_all[W_RLS][W_RLS][N_OF_INTEREST][N_OF_INTEREST];
    float P[N_OF_INTEREST][N_OF_INTEREST];

    float disturbances_predicted[W_RLS][N_OF_INTEREST];
    float M_optimal_all[4][N_];

    float cmd_thrust;
    float cmd_roll;
    float cmd_pitch;
    float cmd_yaw;
    float r_roll;
    float r_pitch;
    float r_yaw;
    float accelz;
} controllerRls_t;

void controllerRlsInit(controllerRls_t *self);
bool controllerRlsTest(controllerRls_t *self);
void controllerRls(controllerRls_t *self, control_t *control, const setpoint_t *setpoint,
                   const sensorData_t *sensors,
                   const state_t *state,
                   const stabilizerStep_t stabilizerStep);

#ifdef CRAZYFLIE_FW

void controllerRlsFirmwareInit(void);
bool controllerRlsFirmwareTest(void);
void controllerRlsFirmware(control_t *control, const setpoint_t *setpoint,
                           const sensorData_t *sensors,
                           const state_t *state,
                           const stabilizerStep_t stabilizerStep);

#endif // CRAZYFLIE_FW

#endif //__CONTROLLER_RLS_H__
