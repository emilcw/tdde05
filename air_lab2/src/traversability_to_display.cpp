#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service.h>

#include <ompl/base/SpaceInformation.h>

#include <nav_msgs/OccupancyGrid.h>

#include <chrono>
#include <thread>

#include "air_lab2/state_validity_checker.h"

class TraversabilitytoDisplay {
public:
  TraversabilitytoDisplay(const ros::NodeHandle& _nodeHandle) : m_nodeHandle(_nodeHandle)
  {
    m_occPub = m_nodeHandle.advertise<nav_msgs::OccupancyGrid>("traversability_display", 1, true);
  }

  template<typename _ServiceT_>
  void publish_map()
  {
    _ServiceT_ req;
    if(ros::service::call<_ServiceT_>("map_request", req))
    {
      ROS_INFO("Start computing traversability map...");
      nav_msgs::OccupancyGrid ogrid;
      ogrid.info = req.response.map.info;
      ogrid.header  = req.response.map.header;
      ogrid.data.resize(req.response.map.data.size());

      ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
      ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
      StateValidityChecker<_ServiceT_> svc(req.response, si.get(), true);

      ompl::base::SE2StateSpace stateSpace;
      ompl::base::SE2StateSpace::StateType* ss = dynamic_cast<ompl::base::SE2StateSpace::StateType*>(stateSpace.allocState());
      assert(ss);
      for(int sx = 0; sx < ogrid.info.width; ++sx)
      {
        for(int sy = 0; sy < ogrid.info.height; ++sy)
        {
          double x = sx * ogrid.info.resolution + 0.5 + ogrid.info.origin.position.x;
          double y = sy * ogrid.info.resolution + 0.5 + ogrid.info.origin.position.y;

          ss->setXY(x ,y);
          ogrid.data[sx + sy * ogrid.info.width] = svc.isValid(ss) ? 0 : 100;

        }
      }
      stateSpace.freeState(ss);

      m_occPub.publish(ogrid);
      ROS_INFO("Finished computing traversability map!");
    } else {
      ROS_ERROR("Failed to get map from server!");
    }

  }


private:
  ros::NodeHandle m_nodeHandle;
  ros::Publisher m_occPub;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability_to_display");
  ros::NodeHandle n;

  TraversabilitytoDisplay ptd(n);

  while(ros::ok())
  {
    ptd.publish_map<nav_msgs::GetMap>();
    for(int i = 0; i < 100; ++i)
    {
      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
  }
  return 0;

}
