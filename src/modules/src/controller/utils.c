#include "utils.h"
#include "stabilizer_types.h"
#include "physicalConstants.h"
#include "commander.h"

void initialize_matrices(float A[N_][N_], float B[N_][M_], float m)
{
    // Initialize matrix A_outer
    float A_outer[N_][N_] = {
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
    float B_outer[N_][M_] = {
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
    for (int i = 0; i < N_; i++)
    {
        for (int j = 0; j < N_; j++)
        {
            A[i][j] = (float)(i == j ? 1.0 : 0.0) + DELTA_T * A_outer[i][j];
        }
    }

    // Compute B = DELTA_T * B_outer
    for (int i = 0; i < N_; i++)
    {
        for (int j = 0; j < M_; j++)
        {
            B[i][j] = DELTA_T * (float)B_outer[i][j];
        }
    }
}
float wrap_angle(float angle)
{
    while (angle > (float)M_PI)
    {
        angle -= (float)(2 * M_PI);
    }
    while (angle < (float)(-M_PI))
    {
        angle += (float)(2 * M_PI);
    }
    return angle;
}

float thrust_newton_to_cmd_light(float thrust)
{
    float cmd_16bit = thrust * MOTOR_CMD_SCALING_CONST;
    if (cmd_16bit < MOTOR_CMD_MIN)
    {
        cmd_16bit = MOTOR_CMD_MIN;
    }
    else if (cmd_16bit > MOTOR_CMD_MAX)
    {
        cmd_16bit = MOTOR_CMD_MAX;
    }

    return cmd_16bit;
}


float thrust_newton_to_cmd(float thrust)
{
    float motor_poly[3] = {5.484560e-4, 1.032633e-6, 2.130295e-11};
    float discriminant = powf(motor_poly[1], 2) - 4 * motor_poly[2] * (motor_poly[0] - thrust);
    float cmd_16bit = (-motor_poly[1] + sqrtf(fmaxf(0, discriminant))) / (2 * motor_poly[2]);

    if (cmd_16bit < MOTOR_CMD_MIN)
    {
        cmd_16bit = MOTOR_CMD_MIN;
    }
    else if (cmd_16bit > MOTOR_CMD_MAX)
    {
        cmd_16bit = MOTOR_CMD_MAX;
    }

    return cmd_16bit;
}



// void compute_setpoint_viaLQR(float K_star[M_][N_], float error_inertial[N_], float curr_yaw, float u[M_])
// {

//     float error_body[N_];

//     float max_error_xy = 0.3f;
//     float max_error_z = 0.4f;
//     // float max_error_yaw = DEG2RAD(60);

//     float error_x_inertial = fmin(fmax(error_inertial[0], -max_error_xy), max_error_xy);
//     float error_y_inertial = fmin(fmax(error_inertial[1], -max_error_xy), max_error_xy);
//     float error_z_inertial = fmin(fmax(error_inertial[2], -max_error_z), max_error_z);
//     // float error_yaw_inertial = wrap_angle(error_inertial[8]);

//     error_body[0] = error_x_inertial;
//     error_body[1] = error_y_inertial;
//     error_body[2] = error_z_inertial;
//     error_body[3] = error_inertial[3];
//     error_body[4] = error_inertial[4];

//     // compute control input in body frame
//     for (int i = 0; i < M_; i++)
//     {
//         u[i] = 0;
//         for (int j = 0; j < 1; j++)
//         {
//             u[i] += -K_star[i][j] * error_body[j];
//         }
//     }
// }

void compute_setpoint_viaLQR(float K_star[M_][N_], float error_inertial[N_], float curr_yaw, float u[M_])
{

    float max_error_xy = 0.3f;
    float max_error_z = 0.4f;
    float max_error_z_dot = 0.2f;

    float error_body[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    float error_x_inertial = fmaxf(fminf(error_inertial[0], max_error_xy), -max_error_xy);
    float error_y_inertial = fmaxf(fminf(error_inertial[1], max_error_xy), -max_error_xy);

    float error_z_inertial = fmaxf(fminf(error_inertial[2], max_error_z), -max_error_z);
    float error_z_dot_inertial = fmaxf(fminf(error_inertial[5], max_error_z_dot), -max_error_z_dot);

    float sinyaw = sinf(curr_yaw);
    float cosyaw = cosf(curr_yaw);

    error_body[0] = error_x_inertial * cosyaw + error_y_inertial * sinyaw;
    error_body[1] = error_y_inertial * cosyaw - error_x_inertial * sinyaw;
    error_body[2] = error_z_inertial;

    error_body[3] = error_inertial[3] * cosyaw + error_inertial[4] * sinyaw;
    error_body[4] = error_inertial[4] * cosyaw - error_inertial[3] * sinyaw;
    error_body[5] = error_z_dot_inertial;

    for (int i = 0; i < 4; i++)
    {
        u[i] = 0;
        for (int j = 0; j < 9; j++)
        {
            u[i] += -K_star[i][j] * error_body[j];
        }
    }
}

// void predict_future_targets(controllerRls_t *self, const setpoint_t *setpoint)
// {

//     uint32_t N_setpoints_recv = commanderGetNSetpointsReceived();

//     // positon and velocity setpoints
//     float curr_target_state[N_OF_INTEREST] = {
//         setpoint->position.x,
//         setpoint->position.y,
//         setpoint->position.z,
//         setpoint->velocity.x,
//         setpoint->velocity.y,
//         setpoint->velocity.z,
//     };

//     float last_pred_target_state_full[N_] = {
//         setpoint->position.x, setpoint->position.y, setpoint->position.z,
//         setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z,
//         setpoint->attitude.roll, setpoint->attitude.pitch, setpoint->attitude.yaw};

//     // Arrays to store pred_target_state and disturbance from each iteration
//     float pred_target_state_full[W_RLS][N_OF_INTEREST];
//     ;

//     for (int k = 0; k < W_RLS; k++)
//     {
//         // Calculate learner index for cyclic learning rate adjustment
//         int learner_idx = (N_setpoints_recv - 1) % k;

//         // Define a submatrix for the k-step prediction
//         float S_target_k_step[N_OF_INTEREST][N_OF_INTEREST];

//         // Populate the submatrix using the augmented target matrices for the specified learner index
//         for (int i = 0; i < N_OF_INTEREST; i++)
//         {
//             for (int j = 0; j < N_OF_INTEREST; j++)
//             {
//                 // Retrieve the value from the augmented matrix of the previous time step
//                 S_target_k_step[i][j] = self->S_target_aug_all[k - 1][learner_idx][i][j];
//             }
//         }

//         // Matrix multiplication: S_target_k_step x curr_target_state
//         float pred_target_state[N_OF_INTEREST];

//         for (int i = 0; i < N_OF_INTEREST; i++)
//         {
//             pred_target_state[i] = 0; // Initialize the predicted state element
//             for (int j = 0; j < N_OF_INTEREST; j++)
//             {
//                 pred_target_state[i] += S_target_k_step[i][j] * curr_target_state[j];
//             }
//         }

//         // Store the computed pred_target_state for this iteration
//         for (int i = 0; i < N_OF_INTEREST; i++)
//         {
//             pred_target_state_full[k][i] = pred_target_state[i];
//         }

//         float pred_target_state_full_complete[N_] = {0}; // Initialize a full state vector with zeros

//         // Insert the predicted state into the full state vector
//         for (int i = 0; i < N_OF_INTEREST; i++)
//         {
//             pred_target_state_full_complete[IDX_OF_INTEREST[i]] = pred_target_state_full[k][i];
//         }

//         float disturbance[N_OF_INTEREST];

//         // Perform matrix multiplication and subtraction: A * last_pred_target_state_full - pred_target_state_full[k]
//         for (int i = 0; i < N_OF_INTEREST; i++)
//         {
//             disturbance[i] = 0; // Initialize the disturbance element
//             for (int j = 0; j < N_OF_INTEREST; j++)
//             {
//                 disturbance[i] += self->A[i][j] * last_pred_target_state_full[j];
//             }
//             disturbance[i] -= pred_target_state_full[k][i];
//         }

//         // append disturbance to  disturbances_predicted
//         for (int i = 0; i < N_OF_INTEREST; i++)
//         {
//             self->disturbances_predicted[k][i] += disturbance[i];
//         }

//         // Update the last predicted state to the current one for the next iteration
//         for (int i = 0; i < N_OF_INTEREST; i++)
//         {
//             last_pred_target_state_full[i] = pred_target_state_full[k][i];
//         }
//     }
// }

// void RLS_update(controllerRls_t *self)
// {
//     // uint32_t N_setpoints_recv = commanderGetNSetpointsReceived();
// }