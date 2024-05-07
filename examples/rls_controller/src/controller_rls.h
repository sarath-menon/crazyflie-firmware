#ifndef __CONTROLLER_RLS_H__
#define __CONTROLLER_RLS_H__

#include "stabilizer_types.h"
#include "math3d.h"

#define N 9
#define M 4
#define DELTA_T 0.002f

#define MAX_ERROR_XY 0.3f
#define MAX_ERROR_Z 0.4f
#define MAX_ERROR_YAW 1.0472f // approximately 60 degrees in radians



typedef struct {
    float mass;
    float massThrust;

    float A[N][N];
    float B[N][M];
    float K_star[M][N];

    // XY Position PID
    float kp_xy;      // P
    float kd_xy;      // D
    float ki_xy;      // I
    float i_range_xy;

    // Z Position
    float kp_z;       // P
    float kd_z;       // D
    float ki_z;       // I
    float i_range_z;

    // Attitude
    float kR_xy;      // P
    float kw_xy;      // D
    float ki_m_xy;    // I
    float i_range_m_xy;

    // Yaw
    float kR_z;       // P
    float kw_z;       // D
    float ki_m_z;     // I
    float i_range_m_z;

    // roll and pitch angular velocity
    float kd_omega_rp; // D

    // Helper variables
    float i_error_x;
    float i_error_y;
    float i_error_z;

    float prev_omega_roll;
    float prev_omega_pitch;
    float prev_setpoint_omega_roll;
    float prev_setpoint_omega_pitch;

    float i_error_m_x;
    float i_error_m_y;
    float i_error_m_z;

    // Logging variables
    struct vec z_axis_desired;

    float cmd_thrust;
    float cmd_roll_rate;
    float cmd_pitch_rate;
    float cmd_yaw_rate;
    float r_roll;
    float r_pitch;
    float r_yaw;
    float accelz;
} controllerRls_t;

void controllerRlsInit(controllerRls_t* self);
bool controllerRlsTest(controllerRls_t* self);
void controllerRls(controllerRls_t* self, control_t *control, const setpoint_t *setpoint,
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
