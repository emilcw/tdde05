#include <nav_msgs/GetMap.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <air_lab2/Mathematics/GteApprOrthogonalPlane3.h>

#include <cmath>
#include <tf/transform_listener.h>

template<typename _T_>
class StateValidityChecker;

template<>
class StateValidityChecker<nav_msgs::GetMap> : public ompl::base::StateValidityChecker
{
public:
  StateValidityChecker(const nav_msgs::GetMap::Response& _grid, ompl::base::SpaceInformation *si, bool _allow_unknown) :  ompl::base::StateValidityChecker(si), m_map(_grid.map), m_allow_unknown(_allow_unknown)
  {}
  virtual bool isValid(const ompl::base::State* state) const
  {
    const ompl::base::SE2StateSpace::StateType* ss = state->as<ompl::base::SE2StateSpace::StateType>();

    // hard coded parameters for the husky size
    const double width = 0.6;
    const double length = 0.8;
    const double half_width = width / 2.0;
    const double half_length = length / 2.0;

    tf::Vector3 r_translation(ss->getX(), ss->getY(), 0);
    tf::Quaternion q;
    q.setEuler(ss->getYaw(), 0, 0);
    tf::Matrix3x3 Q(q);

    for(float x = -half_length; x <= half_length; x += m_map.info.resolution)
    {
      for(float y = -half_width; y <= half_width; y += m_map.info.resolution)
      {
        tf::Vector3 a = r_translation + Q * tf::Vector3(x, y, 0);
        int o = occupancy_at(a.getX(), a.getY());
        if( o == -1 or o > 20)
        {
          return false;
        }
      }
    }
    return true;
  }
private:
  /// @return 
  int occupancy_at(double _x, double _y) const
  {
    int x = (_x - m_map.info.origin.position.x) / m_map.info.resolution;
    int y = (_y - m_map.info.origin.position.y) / m_map.info.resolution;

    if(m_allow_unknown)
    {

      for(int k = 0; k < 1; ++k)
      {
        for(int j = -k; j <= k; ++j)
        {
          for(int i = -k; i <= k; ++i)
          {
            double e = occupancy_safe_at(x + i, y + j);

            if(not std::isnan(e))
            {
              return e;
            }
          }
        }
      }
    }
    
    return occupancy_safe_at(x, y);
  }
  int occupancy_safe_at(int _i, int _j) const
  {
    if(_i >= 0 and
        _j >= 0 and
        _i < m_map.info.width and
        _j < m_map.info.height)
    {
      return m_map.data[ _i + _j * m_map.info.width ];
    }
    else
    {
      return -1;
    }
  }
private:
  const nav_msgs::OccupancyGrid m_map;
  bool m_allow_unknown;
};
