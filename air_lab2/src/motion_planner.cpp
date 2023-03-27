#include "air_lab2/motion_planner_interface.h"
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <string>
#include <math.h>
#include <atomic>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetPlan.h>
#include <air_lab2/extensible_grid.h>

class MotionPlanner{
  ros::ServiceServer mp_pathRequest;

  public:
    MotionPlanner(const ros::NodeHandle& _nodeHandle)
    : m_nodeHandle(_nodeHandle)
    {
      motion_planner_interface = new MotionPlannerInterface();
      mp_pathRequest = m_nodeHandle.advertiseService("plan_path",&MotionPlanner::pathService, this);
    }


bool pathService(nav_msgs::GetPlan::Request& _req, nav_msgs::GetPlan::Response& _resp)
  {
    std::vector<geometry_msgs::PoseStamped> poses;
    double start_x = _req.start.pose.position.x;
    double start_y = _req.start.pose.position.y;
    double dest_x = _req.goal.pose.position.x;
    double dest_y = _req.goal.pose.position.y;
    std::string frame_id = _req.start.header.frame_id;

    std::vector<std::pair<double, double>> path =  motion_planner_interface->planPath<nav_msgs::GetMap>(start_x, start_y, dest_x, dest_y, &_req.start.header.frame_id);

    for (int i = 0; i < path.size(); i++) {
      double x = path[i].first;
      double y = path[i].second;
      geometry_msgs::PoseStamped p;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.header.frame_id = frame_id;
      std::cout << "Going to" << x <<", " <<y << std::endl;

      //Absolute value in atan2?
      double yaw_angle;
      if(i==0){
        double x_1 = path[i+1].first;
        double y_1 = path[i+1].second;
        yaw_angle = atan2((y_1 - y),(x_1 - x));
      }else if(i == path.size()-1){
        double x_1 = path[i-1].first;
        double y_1 = path[i-1].second;
        yaw_angle = atan2((y - y_1),(x - x_1));
      }else{
        double x_2 = path[i+1].first;
        double y_2 = path[i+1].second;
        double x_1 = path[i-1].first;
        double y_1 = path[i-1].second;
        yaw_angle = atan2((y_2 - y_1),(x_2 - x_1));
      }


      tf::Quaternion orientation;
      orientation.setRPY( 0, 0, yaw_angle);
      p.pose.orientation.x = orientation.getX();
      p.pose.orientation.y = orientation.getY();
      p.pose.orientation.z = orientation.getZ();
      p.pose.orientation.w = orientation.getW();


      poses.push_back(p);
    }
    _resp.plan.poses = poses;
    _resp.plan.header.frame_id = frame_id;

    return true;

  }

  private:
    ros::NodeHandle m_nodeHandle;
    MotionPlannerInterface* motion_planner_interface;
  };

  int main(int argc, char** argv){
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;

  MotionPlanner ptd(n);

  ros::spin();
  return 0;
}
