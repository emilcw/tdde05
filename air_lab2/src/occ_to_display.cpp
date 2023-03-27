#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service.h>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/GetMap.h>

#include <chrono>
#include <thread>

class OCCtoDisplay {
public:
  OCCtoDisplay(const ros::NodeHandle& _nodeHandle) : m_nodeHandle(_nodeHandle)
  {
    m_occPub = m_nodeHandle.advertise<nav_msgs::OccupancyGrid>("map_display", 1, true);
    m_timer = m_nodeHandle.createTimer(ros::Duration(1), &OCCtoDisplay::publish_map, this);
  }

  void publish_map(const ros::TimerEvent&)
  {
    nav_msgs::GetMap req;
    if(ros::service::call<nav_msgs::GetMap>("map_request", req))
    {
      m_occPub.publish(req.response.map);
    } else {
      ROS_ERROR("Failed to get map from server!");
    }

  }


private:
  ros::NodeHandle m_nodeHandle;
  ros::Publisher m_occPub;
  ros::Timer m_timer;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occ_to_display");
  ros::NodeHandle n;

  OCCtoDisplay ptd(n);

  ros::spin();
  return 0;
}
