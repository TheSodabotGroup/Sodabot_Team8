#ifndef MAPPING_OCCUPANCY_GRID_3D_HPP
#define MAPPING_OCCUPANCY_GRID_3D_HPP

#include <common/point.hpp>
#include <lcmtypes/occupancy_grid_t.hpp>
#include <lcmtypes/occupancy_grid_3D_t.hpp> //@Wei: 3D occupancy lcm
#include <algorithm>
#include <cstdint>
#include <vector>

typedef int8_t CellOdds;   ///< Type used to represent the data in a cell

class OccupancyGrid;//@Wei: forward declaration

class OccupancyGrid3D
{
public:
    
    OccupancyGrid3D(void);
    
    OccupancyGrid3D(float widthInMeters,
                    float heightInMeters,
                    float depthInMeters, // @Wei 3D depth
                    float metersPerCell);
    
    
    // Accessors for the properties of the grid
    int   widthInCells (void) const { return width_; }
    float widthInMeters(void) const { return width_ * metersPerCell_; }
    
    int   heightInCells (void) const { return height_; }
    float heightInMeters(void) const { return height_ * metersPerCell_; }

    int   depthInCells (void) const { return depth_; } //@Wei: depth functions
    float depthInMeters(void) const { return depth_ * metersPerCell_; }
    
    float metersPerCell(void) const { return metersPerCell_; }
    float cellsPerMeter(void) const { return cellsPerMeter_; }
    
    Point<float> originInGlobalFrame(void) const { return globalOrigin_; }
    float originDepthInGlobal(void) const { return globalOriginDepth_; } //@Wei: Additional origin depth value
    
    void setOrigin(float x, float y, float z);

    /**
    * reset resets all cells into the grid to equal odds. All cells will have logOdds == 0.
    */
    void reset(void);
    
    bool isCellInGrid(int x, int y, int z) const;
    
    CellOdds logOdds(int x, int y, int z) const;
    
    void setLogOdds(int x, int y, int z, CellOdds logOdds);
    
    CellOdds& operator()(int x, int y, int z)       { return layers_[z](x, y); } // layers[z] is 2d occupancy grid object
    
    CellOdds  operator()(int x, int y, int z) const { return layers_[z](x, y); }

    OccupancyGrid operator[](int z) const {return layers_[z]; }

    occupancy_grid_3D_t toLCM(void) const; //@Wei 3D occupancy grid lcm type!
    
    void fromLCM(const occupancy_grid_3D_t& gridMessage);
    
    bool saveToFile(const std::string& filename) const;
    
    bool loadFromFile(const std::string& filename);
    
private:
    
    std::vector<OccupancyGrid> layers_;
    
    int width_;                 ///< Width of the grid in cells
    int height_;                ///< Height of the grid in cells
    int depth_;
    
    float metersPerCell_;
    float cellsPerMeter_;
    
    Point<float> globalOrigin_; 
    float globalOriginDepth_;         ///< Origin of the grid in global coordinates
    
};

#endif // MAPPING_OCCUPANCY_GRID_3D_HPP
