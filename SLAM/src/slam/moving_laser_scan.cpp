#include <slam/moving_laser_scan.hpp>
#include <common/interpolation.hpp>
#include <lcmtypes/lidar_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
    
MovingLaserScan::MovingLaserScan(const lidar_t& scan,
                                 const pose_xyt_t&      beginPose,
                                 const pose_xyt_t&      endPose,
                                 int                    rayStride)
{
    // Ensure a valid scan was received before processing the rays
    if(scan.num_ranges > 0)
    {
        // The stride must be at least one, or else can't iterate through the scan
        if(rayStride < 1)
        {
            rayStride = 1;
        }

        // std::cout<<"Starting MLS !\n";
        for(int n = 0; n < scan.num_ranges; n += rayStride)
        {
            if(scan.ranges[n] > 0.15f) //all ranges less than a robot radius are invalid
            {
                pose_xyt_t rayPose = interpolate_pose_by_time(scan.times[n], beginPose, endPose); //@Wei: assume done by roger

                adjusted_ray_t ray;

                // ray.origin.x = rayPose.x;
                // ray.origin.y = rayPose.y;
                // ray.range    = scan.ranges[n];
                // ray.theta    = wrap_to_pi(rayPose.theta - scan.thetas[n]);

                ray.originDepth = 0.10; // @Wei: add origin's z
                ray.body_angle = wrap_to_pi(rayPose.tb_angles[0]); // @Wei: add body angle
                float phi = rayPose.tb_angles[0];
                float offset = 0.2 * std::sin(phi);
                float tmp = sin(scan.thetas[n])*sin(scan.thetas[n])*cos(phi)*cos(phi) + cos(scan.thetas[n])*cos(scan.thetas[n]);
                ray.range    = scan.ranges[n] * cos(phi)/ sqrt(tmp);
                ray.theta    = wrap_to_pi(rayPose.theta - scan.thetas[n]);
         //std::cout<<"Scan Range :"<<scan.ranges[n]<<"  Ray Range :"<<ray.range<<"\n";
                ray.origin.x = rayPose.x + offset * std::cos(ray.theta);
                ray.origin.y = rayPose.y + offset * std::sin(ray.theta);

                adjustedRays_.push_back(ray);

            }
        }
        // std::cout<<"Finished MLS !\n";
    }
}

