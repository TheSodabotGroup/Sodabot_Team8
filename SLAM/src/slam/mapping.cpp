#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <cmath>
#include <vector>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
    init_flag = false;
}

std::vector<Point<int>> BresenhamUpgrade(int& x0, int& y0, int& x1, int& y1){
    std::vector<Point<int>> p_vector;
    float theta = std::atan2(y1-y0, x1-x0);
    p_vector.push_back(Point<int>(x0, y0));
    float x = x0;
    float y = y0;
    int dir_x = x1 > x0 ? 1:-1;
    int dir_y = y1 > y0 ? 1:-1;  
    int int_x = 0;
    int int_y = 0;  

    while(true){
        x += std::cos(theta);
        y += std::sin(theta);

        int_x = static_cast<int>(x);
        int_y = static_cast<int>(y);

        if(p_vector.back().x == int_x && p_vector.back().y == int_y){
            continue;
        }

        if( dir_x * (x1 - int_x) <= 0 && dir_y * (y1 - int_y) <= 0 ){
            p_vector.push_back(Point<int>(x1, y1));
            break;
        }

        p_vector.push_back(Point<int>(int_x,int_y));   
    }

    return p_vector;
}

int8_t clamp(int input){
    if (input > 127) return 127;
    else if(input < -128) return -128;
    return input;
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

    /* Initiate the flag for saving previous pose */
    if(!init_flag){
        begin_pose = pose;
        init_flag = true;
        return;
    }
    
    /* MovingLaserScan() -> adjusted_ray_t*/
    MovingLaserScan mls = MovingLaserScan(scan, begin_pose, pose, 1);

    /* Update cell odds */
    for(size_t i = 0; i < mls.size(); i++){
        /* get adjusted ray from MovingLaserScan */
        adjusted_ray_t ray = mls.at(i);
        bool obstacleDetected = true;
        float range = ray.range;
        float theta = ray.theta;

        /* apply the threshold to the detected range*/
        if(range >= kMaxLaserDistance_){
            range = kMaxLaserDistance_;
            obstacleDetected = false;
        }

        /* x, y in cell coordinate */
        Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
        Point<float> target = ray.origin;
        target.x += range * std::cos(theta);
        target.y += range * std::sin(theta);
        Point<int> end_cell = global_position_to_grid_cell(target, map);
        
        /* update the detected cell */
        int8_t odds = obstacleDetected ? kHitOdds_:-kMissOdds_;
        map.setLogOdds(end_cell.x, end_cell.y, clamp(map.logOdds(end_cell.x, end_cell.y) + odds));
        
        /* update cells between the range */
        if(map.isCellInGrid(end_cell.x, end_cell.y)){
            std::vector<Point<int>> p_vector = BresenhamUpgrade(start_cell.x, start_cell.y, end_cell.x, end_cell.y);
            for(auto p: p_vector){
                map.setLogOdds(p.x, p.y, clamp(map.logOdds(p.x, p.y) -kMissOdds_));
            }
        }
    }
    /* update the begin pose */
    begin_pose = pose;

}
