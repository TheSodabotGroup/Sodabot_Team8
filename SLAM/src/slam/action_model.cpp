#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    del_rot1 = 0.0;
    del_trans = 0.0;
    del_rot2 = 0.0;
    past_odometry.utime = 0;
    past_odometry.x = 0.0;
    past_odometry.y = 0.0;
    past_odometry.theta = 0.0;
    past_odometry.tb_angles[0] = 0.0;
    past_odometry.tb_angles[1] = 0.0;
    past_odometry.tb_angles[2] = 0.0;
    

    alpha_1 = 0.01;
    alpha_2 = 0.2;
    alpha_3 = 0.1;
    alpha_4 = 0.01;

}

void ActionModel::SetAction(const pose_xyt_t& odometry)
{
    past_odometry = odometry;
}



bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    float del_x = odometry.x - past_odometry.x;
    float del_y = odometry.y - past_odometry.y;
    float del_theta = wrap_to_pi(odometry.theta - past_odometry.theta);

    // Check going forward or backwards
    del_rot1 = wrap_to_pi(std::atan2(del_y, del_x) - past_odometry.theta);
    bool moving_backwards = del_rot1 > 1.57 || del_rot1 < -1.57;
    if(moving_backwards) del_rot1 = wrap_to_pi(del_rot1 + 3.14159);

    // Check if robot has moved
    const float threshold = 0.02;
    if(abs(del_x) >= threshold || abs(del_y) >= threshold){
        // Compute(update) the amount of change based on new & old  odometry data
        if(moving_backwards) del_trans = -1 * std::sqrt(del_x*del_x + del_y*del_y);
        else del_trans = std::sqrt(del_x*del_x + del_y*del_y);
        del_rot2 = wrap_to_pi(del_theta - del_rot1);
    }else{
        del_rot1 = del_theta;
        if(moving_backwards) del_trans = -1 * std::sqrt(del_x*del_x + del_y*del_y);
        else del_trans = std::sqrt(del_x*del_x + del_y*del_y);
        del_rot2 = 0;
    }
    // Update past odometry
    past_odometry = odometry;
    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t proposed_particle;

    // Estimate the amount of change in each motion by subtracting the noise
    float h_del_rot1  = del_rot1  - sampleNormal(alpha_1 * del_rot1 * del_rot1 
                                               + alpha_2 * del_trans * del_trans);
    float h_del_trans = del_trans - sampleNormal(alpha_3 * del_trans * del_trans 
                                               + alpha_4 * del_rot1 * del_rot1
                                               + alpha_4 * del_rot2 * del_rot2);
    float h_del_rot2  = del_rot2  - sampleNormal(alpha_1 * del_rot2 * del_rot2
                                               + alpha_2 * del_trans * del_trans);
   
    // Use current pose to estimate new pose
    proposed_particle.pose.x = sample.pose.x + h_del_trans * std::cos(sample.pose.theta + h_del_rot1);
    proposed_particle.pose.y = sample.pose.y + h_del_trans * std::sin(sample.pose.theta + h_del_rot1);
    proposed_particle.pose.theta = wrap_to_pi(sample.pose.theta + h_del_rot1 + h_del_rot2); 
    proposed_particle.pose.utime = past_odometry.utime;
    proposed_particle.pose.tb_angles[0] = past_odometry.tb_angles[0];
    proposed_particle.pose.tb_angles[1] = past_odometry.tb_angles[1];
    proposed_particle.pose.tb_angles[2] = past_odometry.tb_angles[2];
    // Set new parent_pose
    proposed_particle.parent_pose = sample.pose;

    // Restore weight
    proposed_particle.weight = sample.weight;
    
    return proposed_particle;
}

float ActionModel::sampleNormal(float b)
{
    std::normal_distribution<float> distribution(0.0, std::sqrt(b));
    return distribution(generator);
}
