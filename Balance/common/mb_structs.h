#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

#include <inttypes.h>

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float tb_angles[3]; // DMP filtered angles, tb_angles[3] is heading
    float accel[3]; // units of m/s^2
    float gyro[3];  // units of degrees/s
    float mag[3];   // units of uT
    float temp;     // units of degrees Celsius
    float last_yaw;
    float phi;
    
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    int64_t left_encoder_total;  //total encoder ticks since running
    int64_t right_encoder_total;
    
    float fwd_velocity;
    float turn_velocity;
    float left_velocity;
    float right_velocity;

    float opti_x;               // Optitrack coordinates 
    float opti_y;               // (if using optitrack for ground truth)
    float opti_theta;           // Optitrack heading

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed

    //Debugging Purpose Objects
    float Kp_err;
    float Ki_err;
    float Kd_err;
    float theta_ref;

};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float theta;    //orientation from initialization in rad
    float unclamped_theta; //orientation withouth clamping function in rad
};


typedef struct pid_parameters pid_parameters_t;
struct pid_parameters{
    float kp1;
    float ki1;
    float kd1;
    float tf1;
    float gain1;

    float kp2;
    float ki2;
    float kd2;
    float tf2;
    float gain2;
    
    float kp3;
    float ki3;
    float kd3;
    float tf3;
};

typedef struct controller_pid controller_pid_t;
struct controller_pid{
    float Kp; 
    float Ki; 
    float Kd; 
    float Tf;
    float Ts;  
    float e_0;   
    float e_1; 
    float e_2; 
    float u_0;   
    float u_1; 
    float u_2;

    //Debugging Purpose Objects
    float Kp_err;
    float Ki_err;
    float Kd_err;
};


#endif