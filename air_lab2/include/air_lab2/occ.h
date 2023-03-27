#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <air_lab2/extensible_grid.h>

/**
 * This class manages a fully extensible Occupancy Grid
 */
class OCC {
  /**
   * Represent a value in the OCC
   */
  struct Cell
  {
    Cell() : count_obstacle(0), samples(0) {}
    int count_obstacle; ///< number of time an obstacle has been seen
    std::size_t samples; ///< number of samples used for calculating the elevation
  };
public:
  /**
   * Create an Occupancy Grid with:
   *
   * @p _cell_size the size (in m) of one cell in the Occupancy Grid
   * @p _initial_radius the radius to initialise around the robot
   */
  OCC(double _cell_size, double _initial_radius) : m_map(nullptr), m_cell_size(_cell_size), m_initial_radius(_initial_radius)
  {
    m_map = new extensible_grid<Cell>(m_cell_size);
  }

  /**
   * Call this function to fill in the result of air_lab2/GetFloatMap
   */
  bool requestMap(nav_msgs::GetMapResponse& _resp)
  {
    _resp.map.header.frame_id = "odom";
    _resp.map.header.stamp    = ros::Time::now();
    _resp.map.info.resolution  = m_cell_size;
    _resp.map.info.width       = m_map->get_rows_count();
    _resp.map.info.height      = m_map->get_columns_count();
    _resp.map.info.origin.position.x = m_map->get_origin_x();
    _resp.map.info.origin.position.y = m_map->get_origin_y();

    _resp.map.data.resize(_resp.map.info.width*_resp.map.info.height, -1);

    for(extensible_grid<Cell>::subgrid_const_iterator cit = m_map->cbegin(); cit != m_map->cend(); ++cit)
    {
      if(cit.is_valid())
      {
        for(int y = 0; y < cit.get_size(); ++y)
        {
          for(int x = 0; x < cit.get_size(); ++x)
          {
            const Cell& cell = cit(x, y);
            _resp.map.data[x + cit.get_left()  + (y + cit.get_top()) * _resp.map.info.width] = (cell.samples == 0) ? -1 : (cell.count_obstacle * 100) / cell.samples;
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

      for(double x = c.getX() - m_initial_radius; x < c.getX() + m_initial_radius; x += m_cell_size)
      {
        for(double y = c.getY() - m_initial_radius; y < c.getY() + m_initial_radius; y += m_cell_size)
        {
          m_map->get_value_ref(x, y).samples = 1;
        }
      }
      m_initial_radius = 0;
    }
  }

  /**
   * Update the elevation for a given position
   *
   * @p v is the coordinate of a point from a point cloud (should be transform in odom coordinates)
   */
  void rayTrace(double _skip, const tf::Vector3& _origin, const tf::Vector3& _direction, double _length, bool _obstacle)
  {
    for(double r = _skip; r < _length - m_cell_size; r += m_cell_size)
    {
      tf::Vector3 v = _origin + r * _direction;
      m_map->get_value_ref(v.x(), v.y()).samples += 1;
    }
    if(_obstacle)
    {
      tf::Vector3 v = _origin + _length * _direction;
      Cell& c = m_map->get_value_ref(v.x(), v.y());
      c.count_obstacle += 1;
      c.samples += 1;
    }
  }

private:
  extensible_grid<Cell>* m_map;
  double m_cell_size;
  double m_initial_radius;
  double m_initial_relative_elevation;
};
