#include "../mobilebot/mobilebot.h"



/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller(){
    controller_pid_init(&inner_loop);
    controller_pid_init(&outer_loop);
    controller_pid_init(&turning_loop);
    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){

    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your cascaded PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/
float phi_last = 0;
float psi_last = 0;
int mb_controller_update(mb_state_t* mb_state, 
                         mb_setpoints_t* mb_setpoints, 
                         mb_odometry_t* mb_odometry, 
                         pid_parameters_t* pid_params)
{   
    mb_state->phi = 0.5*((float)mb_state->left_encoder_total + (float)mb_state->right_encoder_total)/(GEAR_RATIO*ENCODER_RES)*(2*3.14159);
    float phi_ref = phi_last + mb_setpoints->fwd_velocity * DT / (0.5*WHEEL_DIAMETER);
    float psi_ref = psi_last + mb_setpoints->turn_velocity * DT;
    psi_last = psi_ref;
    phi_last = phi_ref;

    // Outer Loop
    controller_set_pid(&outer_loop, pid_params->kp2, pid_params->ki2, pid_params->kd2, pid_params->tf2, DT);

    float theta_r = -pid_params->gain2*controller_march(&outer_loop, phi_ref - mb_state->phi , -0.5, 0.5);
    // float theta_r = 0;

    // float theta_r = controller_march(&outer_loop, 0 - mb_state->phi , -0.3, 0.3);

    // Inner Loop
    controller_set_pid(&inner_loop, pid_params->kp1, pid_params->ki1, pid_params->kd1, pid_params->tf1, DT);
    // TODO: tb_angles not sure
    float duty = 1.25*controller_march(&inner_loop, theta_r- mb_state->tb_angles[0]+0.06 , -1.0, 1.0);
    // float duty = controller_march(&inner_loop, -mb_state->tb_angles[0]+0.05 , -1.0, 1.0);


    // // Turning Loop
    controller_set_pid(&turning_loop, pid_params->kp3, pid_params->ki3, pid_params->kd3, pid_params->tf3, DT);
    float diff_duty = controller_march(&turning_loop, psi_ref - mb_odometry->unclamped_theta  , -0.8, 0.8);
    // float diff_duty = controller_march(&turning_loop, mb_angle_diff_radians(mb_odometry->theta, psi_ref), -0.8, 0.8);

    // if(duty + diff_duty < -1.0) mb_state->left_cmd = -1.0;
    // else if(duty + diff_duty > 1.0) mb_state->left_cmd = 1.0;
    // else mb_state->left_cmd = duty + diff_duty;

    // if(duty - diff_duty > 1.0) mb_state->right_cmd = 1.0;
    // else if(duty - diff_duty < -1.0) mb_state->right_cmd = -1.0;
    // else mb_state->right_cmd = duty - diff_duty;

    mb_state->left_cmd = duty + diff_duty;
    mb_state->right_cmd = duty - diff_duty;

    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){

    return 0;
}

void controller_pid_init(controller_pid_t* controller){
    controller->e_0 = 0;
    controller->e_1 = 0;
    controller->e_2 = 0;
    controller->u_0 = 0;
    controller->u_1 = 0;
    controller->u_2 = 0;
}

void controller_set_pid(controller_pid_t* controller, float Kp, float Ki, float Kd, float Tf, float Ts){
    controller->Kp = Kp;
    controller->Ki = Ki;
    controller->Kd = Kd;
    controller->Tf = Tf;
    controller->Ts = Ts;
}

float controller_march(controller_pid_t* controller, float input, float min, float max){
    controller->e_2 = controller->e_1;
    controller->e_1 = controller->e_0;
    controller->e_0 = input;
    controller->u_2 = controller->u_1;
    controller->u_1 = controller->u_0;
    
    float Kp = controller->Kp;
    float Ki = controller->Ki;
    float Kd = controller->Kd;
    float Tf = controller->Tf;
    float Ts = controller->Ts;
    float e_0 = controller->e_0;
    float e_1 = controller->e_1;
    float e_2 = controller->e_2;
    float u_1 = controller->u_1;
    float u_2 = controller->u_2;
    float a = 1 + 0.5*Ts/Tf;
    float b = 1 - 0.5*Ts/Tf;
    
    float U = 2*u_1 - b*u_2;
    float KP = Kp * (a*e_0 - 2*e_1 + b*e_2);
    float KI = Ki * (0.5*Ts*a*e_0 + 0.5*Ts*Ts/Tf*e_1 - 0.5*Ts*b*e_2);
    float KD = Kd * (1/Tf*e_0 - 2/Tf*e_1 + 1/Tf*e_2);
    
    float u_0 = (U + KP + KD + KI)/a;
    if(u_0 < min) u_0 = min;
    if(u_0 > max) u_0 = max;
    controller->u_0 = u_0;
    return u_0;
}
