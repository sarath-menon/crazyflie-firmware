#include "utils.h"
#include "stabilizer_types.h"
#include "physicalConstants.h"
#include "commander.h"

void initialize_matrices(float A[N][N], float B[N][M], float m)
{
    // Initialize matrix A_outer
    float A_outer[N][N] = {
        {0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, GRAVITY_MAGNITUDE, 0},
        {0, 0, 0, 0, 0, 0, -GRAVITY_MAGNITUDE, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0}};

    // Initialize matrix B_outer
    float B_outer[N][M] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {1 / m, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};

    // Compute A = I + DELTA_T * A_outer
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            A[i][j] = (float)(i == j ? 1.0 : 0.0) + DELTA_T * A_outer[i][j];
        }
    }

    // Compute B = DELTA_T * B_outer
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < M; j++)
        {
            B[i][j] = DELTA_T * (float)B_outer[i][j];
        }
    }
}

void compute_setpoint_viaLQR(float K_star[M][N], float error_inertial[N], float curr_yaw, float u[M])
{

    float error_body[N];

    float sinyaw = sin(curr_yaw);
    float cosyaw = cos(curr_yaw);

    // compute error in inertial frame
    float error_x_inertial = fmin(fmax(error_inertial[0], -MAX_ERROR_XY), MAX_ERROR_XY);
    float error_y_inertial = fmin(fmax(error_inertial[1], -MAX_ERROR_XY), MAX_ERROR_XY);
    float error_z_inertial = fmin(fmax(error_inertial[2], -MAX_ERROR_Z), MAX_ERROR_Z);
    // float error_yaw_inertial = fmin(fmax(error_inertial[8], -MAX_ERROR_YAW), MAX_ERROR_YAW);

    // convert error to body frame
    error_body[0] = error_x_inertial * cosyaw + error_y_inertial * sinyaw;
    error_body[1] = error_y_inertial * cosyaw - error_x_inertial * sinyaw;
    error_body[2] = error_z_inertial;
    error_body[3] = error_inertial[3] * cosyaw + error_inertial[4] * sinyaw;
    error_body[4] = error_inertial[4] * cosyaw - error_inertial[3] * sinyaw;

    // compute control input in body frame
    for (int i = 0; i < 4; i++)
    {
        u[i] = 0;
        for (int j = 0; j < 1; j++)
        {
            u[i] += -K_star[i][j] * error_body[j];
        }
    }
}

void predict_future_targets(setpoint_t *setpoint)
{

    // uint32_t N_setpoints_recv = commanderGetNSetpointsReceived();

    // // positon and velocity setpoints
    // float curr_target_state[6] = {
    //     setpoint->position.x,
    //     setpoint->position.y,
    //     setpoint->position.z,
    //     setpoint->velocity.x,
    //     setpoint->velocity.y,
    //     setpoint->velocity.z,
    // };

    // float last_pred_target_state_full[N] = {
    //     setpoint->position.x, setpoint->position.y, setpoint->position.z,
    //     setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z,
    //     setpoint->attitude.roll, setpoint->attitude.pitch, setpoint->attitude.yaw};

    // for (int k = 0; k < W_RLS; k++)
    // {
    //     // Calculate learner index for cyclic learning rate adjustment
    //     int learner_idx = (N_setpoints_recv - 1) % k;
    // }
}