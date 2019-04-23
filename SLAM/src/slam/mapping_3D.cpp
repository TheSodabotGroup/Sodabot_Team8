#include <slam/mapping_3D.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/occupancy_grid_3D.hpp> // @Wei
#include <common/grid_utils.hpp>
#include <numeric>
#include <cmath>
#include <vector>
#include <math.h> // round rint
#include <cstring> // memset

Mapping3D::Mapping3D(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
    PrevPose.utime = 0;
    PrevPose.x = 0;
    PrevPose.y = 0;
    PrevPose.theta = 0;
    for(int i = 0; i < 3;i++)
        PrevPose.tb_angles[i] = 0;
}



void Mapping3D::updateMap(const lidar_t& scan, 
                        const pose_xyt_t& pose,
                        OccupancyGrid3D& map)
{

    MovingLaserScan moving_laser_scan(scan,PrevPose,pose,1);
    // cout<<"Im updating the 3D map !";
    for(size_t i = 0; i < moving_laser_scan.size();i++){

        adjusted_ray_t this_ray = moving_laser_scan[i];
        if(this_ray.range <= kMaxLaserDistance_){

            double x_goal = this_ray.origin.x + this_ray.range* cos(this_ray.body_angle) * cos(this_ray.theta);
            double y_goal = this_ray.origin.y + this_ray.range* sin(this_ray.theta);
            double z_goal = this_ray.originDepth + this_ray.range* sin(this_ray.body_angle); // @ Wei

            Point<int> init_grid_cell = global_position_to_grid_cell(this_ray.origin, map);
            Point<int> goal_grid_cell = global_position_to_grid_cell(Point<double>(x_goal,y_goal), map);
            int x0 = init_grid_cell.x;
            int y0 = init_grid_cell.y;
            int x1 = goal_grid_cell.x;
            int y1 = goal_grid_cell.y;

            int z0 = rint(this_ray.originDepth/map.metersPerCell()); //@Wei: by default z0 = 2;
            int z1 = rint(z_goal/map.metersPerCell());

            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;
            int sz = z0 < z1 ? 1 : -1; //@ Wei


            int dx = abs(x1 - x0);
            int dy = abs(y1 - y0);
            int dz = abs(z1 - z0);

            int new_odd = std::min(127, map.logOdds(x1,y1,z1) + kHitOdds_);
            map.setLogOdds(x1,y1,z1,new_odd);


            if (dx >= dy && dx >= dz){      
                int p1 = 2 * dy - dx; 
                int p2 = 2 * dz - dx; 
                while (x0 != x1){
                    x0 += sx; 
                    if (p1 >= 0){ 
                        y0 += sy; 
                        p1 -= 2 * dx;
                    }
                    if (p2 >= 0){
                        z0 += sz; 
                        p2 -= 2 * dx;
                    } 
                    p1 += 2 * dy; 
                    p2 += 2 * dz; 

                    if(!map.isCellInGrid(x0,y0,z0)) continue;
                    int new_odd = std::max(-128, map.logOdds(x0,y0,z0) - kMissOdds_);
                    map.setLogOdds(x0,y0,z0,new_odd);
                }
            }
          
            else if (dy >= dx && dy >= dz){       
                int p1 = 2 * dx - dy; 
                int p2 = 2 * dz - dy; 
                while (y0 != y1){ 
                    y0 += sy; 
                    if (p1 >= 0){ 
                        x0 += sx ;
                        p1 -= 2 * dy;
                    } 
                    if (p2 >= 0){ 
                        z0 += sz; 
                        p2 -= 2 * dy;
                    } 
                    p1 += 2 * dx;
                    p2 += 2 * dz;
                    if(!map.isCellInGrid(x0,y0,z0)) continue;
                    int new_odd = std::max(-128, map.logOdds(x0,y0,z0) - kMissOdds_);
                    map.setLogOdds(x0,y0,z0,new_odd);
                }
            }
          
            else{        
                int p1 = 2 * dy - dz; 
                int p2 = 2 * dx - dz; 
                while (z0 != z1){ 
                    z0 += sz; 
                    if (p1 >= 0){ 
                        y0 += sy; 
                        p1 -= 2 * dz;
                    } 
                    if (p2 >= 0){ 
                        x0 += sx; 
                        p2 -= 2 * dz; 
                    }
                    p1 += 2 * dy; 
                    p2 += 2 * dx;
                    if(!map.isCellInGrid(x0,y0,z0)) continue;
                    int new_odd = std::max(-128, map.logOdds(x0,y0,z0) - kMissOdds_);
                    map.setLogOdds(x0,y0,z0,new_odd);
                } 
            }
        }
    }
}
 
