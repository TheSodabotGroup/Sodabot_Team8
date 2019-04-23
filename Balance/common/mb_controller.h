#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "../mobilebot/mobilebot.h"
#define CFG_PATH "pid.cfg"

int mb_initialize_controller();
int mb_load_controller_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry, pid_parameters_t* pid_params);
int mb_destroy_controller();
void controller_pid_init(controller_pid_t* controller);
void controller_set_pid(controller_pid_t* controller, float Kp, float Ki, float Kd, float Tf, float Ts);
float controller_march(controller_pid_t* controller, float input, float min, float max);

controller_pid_t inner_loop, outer_loop, turning_loop;
pid_parameters_t left_pid_params;
pid_parameters_t right_pid_params;
pid_parameters_t fwd_vel_pid_params;
pid_parameters_t turn_vel_pid_params;

#endif

