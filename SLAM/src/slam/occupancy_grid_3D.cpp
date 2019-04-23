#include <slam/occupancy_grid.hpp>
#include <slam/occupancy_grid_3D.hpp>
#include <fstream>
#include <cassert>

#include <iostream>
using namespace std;


OccupancyGrid3D::OccupancyGrid3D(void)
: width_(0)
, height_(0)
, depth_(9) // @Wei

, metersPerCell_(0.05f)
, cellsPerMeter_(1.0 / metersPerCell_)
, globalOrigin_(0, 0)
, globalOriginDepth_(0) // @Wei
{
}


OccupancyGrid3D::OccupancyGrid3D(float widthInMeters,
                                 float heightInMeters,
                                 float depthInMeters,
                                 float metersPerCell)
: depth_(9)
, metersPerCell_(metersPerCell)
, globalOrigin_(-widthInMeters/2.0f, -heightInMeters/2.0f)
, globalOriginDepth_(0) 

{
    // assert is redundant, since checked at every layer, but just for double check
    assert(widthInMeters  > 0.0f);
    assert(heightInMeters > 0.0f);
    assert(metersPerCell_ <= widthInMeters);
    assert(metersPerCell_ <= heightInMeters);
    
    cellsPerMeter_ = 1.0f / metersPerCell_;
    width_         = widthInMeters * cellsPerMeter_;
    height_        = heightInMeters * cellsPerMeter_;
    
    layers_.clear();
    for(int i = 0; i < depth_; i++){
        OccupancyGrid layer(widthInMeters,heightInMeters,metersPerCell);
        layers_.push_back(layer);
    }
}

void OccupancyGrid3D::setOrigin(float x, float y, float z){

    // reset(); // redundant !
    globalOrigin_.x -= x;
    globalOrigin_.y -= y;
    globalOriginDepth_ -= z;
    for(int i = 0; i < depth_; i++){
        layers_[i].setOrigin(x, y); // reset() included !
    }


}

void OccupancyGrid3D::reset(void)
{
    cout << "reset!\n";
    for(int i = 0; i < depth_; i++)
        layers_[i].reset();
}


bool OccupancyGrid3D::isCellInGrid(int x, int y, int z) const
{ 
    bool xCoordIsValid = (x >= 0) && (x < width_);
    bool yCoordIsValid = (y >= 0) && (y < height_);
    bool zCoordIsValid = (z >= 0) && (z < depth_);
    return xCoordIsValid && yCoordIsValid && zCoordIsValid;
}


CellOdds OccupancyGrid3D::logOdds(int x, int y, int z) const
{
    if(isCellInGrid(x, y, z))
        return operator()(x, y, z);
    
    return 0;
}


void OccupancyGrid3D::setLogOdds(int x, int y, int z, CellOdds value)
{
    if(isCellInGrid(x, y, z))
        operator()(x, y, z) = value;
}


occupancy_grid_3D_t OccupancyGrid3D::toLCM(void) const
{
    occupancy_grid_3D_t grid;

    grid.origin_x        = globalOrigin_.x;
    grid.origin_y        = globalOrigin_.y;
    grid.origin_z        = globalOriginDepth_;

    grid.meters_per_cell = metersPerCell_;
    grid.width           = width_;
    grid.height          = height_;
    grid.depth           = depth_;

    grid.layers.resize(depth_);
    for(int i = 0; i < depth_;i++)
        grid.layers[i]   = layers_[i].toLCM();
    
    return grid;
}


void OccupancyGrid3D::fromLCM(const occupancy_grid_3D_t& gridMessage)
{
    globalOrigin_.x = gridMessage.origin_x;
    globalOrigin_.y = gridMessage.origin_y;
    globalOriginDepth_ = gridMessage.origin_z;
    metersPerCell_  = gridMessage.meters_per_cell;
    cellsPerMeter_  = 1.0f / gridMessage.meters_per_cell;
    height_         = gridMessage.height;
    width_          = gridMessage.width;
    depth_          = gridMessage.depth;

    layers_.resize(depth_);
    for(int i = 0; i < depth_;i++)
        layers_[i].fromLCM(gridMessage.layers[i]);
}


bool OccupancyGrid3D::saveToFile(const std::string& filename) const
{
    std::ofstream out(filename);
    if(!out.is_open())
    {
        std::cerr << "ERROR: OccupancyGrid::saveToFile: Failed to save to " << filename << '\n';
        return false;
    }
    
    // Write header
    // @Wei, globalOrigin_ is Point, which is 2D so very sorry we cannot change the header unless we define Point3D
    out << globalOrigin_.x << ' ' << globalOrigin_.y << ' ' << globalOriginDepth_ << '\n'
        << width_ << ' ' << height_ << ' ' << depth_ << ' ' << metersPerCell_ << '\n';
    
    // Write out each cell value
    for(int z = 0; z < depth_; ++z)
    {
        for(int y = 0; y < height_; ++y)
        {
            for(int x = 0; x < width_; ++x)
            {
                // Unary plus forces output to be a a number rather than a character
                 out << +logOdds(x, y, z) << ' ';
            }
            out << '\n';
        }
        out << '\n';
    }

    
    return out.good();
}


bool OccupancyGrid3D::loadFromFile(const std::string& filename)
{
    std::ifstream in(filename);
    if(!in.is_open())
    {
        std::cerr << "ERROR: OccupancyGrid::loadFromFile: Failed to load from " << filename << '\n';
        return false;
    }
    
    width_ = -1;
    height_ = -1;
    
    // Read header
    in  >> globalOrigin_.x >> globalOrigin_.y  >> globalOriginDepth_
        >> width_ >> height_ >> depth_ >> metersPerCell_;
    
    // Check sanity of values
    assert(width_ > 0);
    assert(height_ > 0);
    assert(depth_ > 0);
    assert(metersPerCell_ > 0.0f);
    
    // Allocate new memory for the grid
    layers_.resize(depth_);
    for(int i = 0;i < depth_;i++)
        layers_[i].accessCells().resize(width_ * height_);
    
    // Read in each cell value
    int odds = 0; // read in as an int so it doesn't convert the number to the corresponding ASCII code
    for(int z = 0; z < depth_; ++z)
    {
        for(int y = 0; y < height_; ++y)
        {
            for(int x = 0; x < width_; ++x)
            {
                in >> odds;
                setLogOdds(x, y, z, odds);
            }
        }
    }
    
    return true;
}
