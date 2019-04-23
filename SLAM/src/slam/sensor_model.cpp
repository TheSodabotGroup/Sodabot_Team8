#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}

double findLogOdds(const adjusted_ray_t& ray, const OccupancyGrid& map){
    Point<float> cnt_position = ray.origin;
    cnt_position.x += ray.range * std::cos(ray.theta);
    cnt_position.y += ray.range * std::sin(ray.theta);

    Point<int> target_cell = global_position_to_grid_cell(cnt_position, map);

    if(!map.isCellInGrid(target_cell.x,target_cell.y)){
        return 0.0;
    }

    return map.logOdds(target_cell.x, target_cell.y) + 128;
}

double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    
    Point<int> particle_position =  global_position_to_grid_cell(Point<double>(sample.pose.x,sample.pose.y), map);

    if (map.logOdds(particle_position.x, particle_position.y) >= 0){
        return 0.0;
    }
    
    // TODO: Add odometry to the argument and do something
    

    //----------------------------------------------------

    /* MovingLaserScan() -> adjusted_ray_t*/
    MovingLaserScan rays = MovingLaserScan(scan, sample.parent_pose, sample.pose, 7);
    double q = 0;
    for(size_t i = 0; i < rays.size(); i++){
        q += findLogOdds(rays.at(i), map);
    }
    return q/rays.size()/256.0;

}
