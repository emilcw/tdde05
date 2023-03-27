#include <tf/transform_listener.h>

#include <air_lab2/GetFloatMap.h>

#include <air_lab2/extensible_grid.h>

/**
 * This class manages a fully extensible Digital Elevation Map
 */
class DEM {
  /**
   * Represent a value in the DEM
   */
  struct Cell
  {
    Cell() : elevation(NAN), samples(0) {}
    std::atomic<double> elevation; ///< elevation at the given cell (NAN if not set yet)
    std::size_t samples; ///< number of samples used for calculating the elevation
  };
public:
  /**
   * Create a DEM with:
   * 
   * @p _cell_size the size (in m) of one cell in the DEM
   * @p _initial_radius the radius to initialise around the robot
   * @p _initial_relative_elevation the relative elevation around the initial position
   */
  DEM(double _cell_size, double _initial_radius, double _initial_relative_elevation) : m_map(nullptr), m_cell_size(_cell_size), m_initial_radius(_initial_radius), m_initial_relative_elevation(_initial_relative_elevation)
  {
    m_map = new extensible_grid<Cell>(m_cell_size);
  }
  
  /**
   * Call this function to fill in the result of air_lab2/GetFloatMap
   */
  bool requestMap(air_lab2::GetFloatMapResponse& _resp)
  {
    _resp.map.header.frame_id = "odom";
    _resp.map.header.stamp    = ros::Time::now();
    _resp.map.info.resolution  = m_cell_size;
    _resp.map.info.width       = m_map->get_rows_count();
    _resp.map.info.height      = m_map->get_columns_count();
    _resp.map.info.origin.position.x = m_map->get_origin_x();
    _resp.map.info.origin.position.y = m_map->get_origin_y();
    
    _resp.map.data.resize(_resp.map.info.width*_resp.map.info.height, NAN);
    
    for(extensible_grid<Cell>::subgrid_const_iterator cit = m_map->cbegin(); cit != m_map->cend(); ++cit)
    {
      if(cit.is_valid())
      {
        for(int y = 0; y < cit.get_size(); ++y)
        {
          for(int x = 0; x < cit.get_size(); ++x)
          {
            _resp.map.data[x + cit.get_left()  + (y + cit.get_top()) * _resp.map.info.width] = cit(x, y).elevation;
          }
        }
      }
    }
    return true;
  }
  
  /**
   * Ensure that the area around the robot is initialised
   */
  void ensureInitialise(const tf::StampedTransform& pcToOdomTf)
  {
    if(m_initial_radius >  0)
    {
      tf::Vector3 c = pcToOdomTf * tf::Vector3(0, 0, 0);
      double elevation = c.getZ() + m_initial_relative_elevation;
      
      for(double x = c.getX() - m_initial_radius; x < c.getX() + m_initial_radius; x += m_cell_size)
      {
        for(double y = c.getY() - m_initial_radius; y < c.getY() + m_initial_radius; y += m_cell_size)
        {
          Cell& c = m_map->get_value_ref(x, y);
          c.elevation = elevation; // set the elevation without setting the sample, so it will be overriden immediately
        }
      }

      m_initial_radius = 0;
    }
  }
  
  enum class update_type
  {
    average, maximum
  };
  
  /**
   * Update the elevation for a given position
   * 
   * @p v is the coordinate of a point from a point cloud (should be transform in odom coordinates)
   */
  void updateElevationAt(const tf::Vector3& v, update_type _update_type = update_type::average)
  {
    Cell& c = m_map->get_value_ref(v.getX(), v.getY());
    if(c.samples == 0)
    {
      c.elevation = v.getZ();
      c.samples = 1;
    } else {
      switch(_update_type)
      {
        case update_type::average:
          c.elevation = (v.getZ() + c.elevation * c.samples) / (c.samples + 1);
          ++c.samples;
          break;
        case update_type::maximum:
          c.elevation = std::max<double>(v.getZ(), c.elevation);
          c.samples = 1;
          break;
      }
    }
  }
  
private:
  extensible_grid<Cell>* m_map;
  double m_cell_size;
  double m_initial_radius;
  double m_initial_relative_elevation;
};
