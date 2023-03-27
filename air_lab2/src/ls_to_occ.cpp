#include "air_lab2/occ.h"
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <math.h>

#include <atomic>

#include <sensor_msgs/LaserScan.h>

class LStoOCC {
  // Class Variables
  tf::TransformListener m_tfListener;
  ros::ServiceServer m_mapRequest;
  ros::Subscriber m_lsSub;

  public:
    LStoOCC(const ros::NodeHandle& _nodeHandle)
    : m_nodeHandle(_nodeHandle), m_occ(nullptr),
    m_cell_size(0.1), m_robot_size(0.0)
    {
      // Fill in
      ros::NodeHandle private_nodehandle("~");
      private_nodehandle.getParam("grid_cell_size", grid_cell_size);
      private_nodehandle.getParam("robot_size", robot_size);
      m_occ = new OCC(grid_cell_size, 2*robot_size);
      m_lsSub = m_nodeHandle.subscribe("scan", 1,
                &LStoOCC::laserScanCallback, this);

      m_mapRequest = m_nodeHandle.advertiseService("map_request",&LStoOCC::mapService, this);
    }


    void laserScanCallback(const sensor_msgs::LaserScanPtr& _message)
    {
      // STEP 1
      tf::StampedTransform transform;
      std::string destination_frame_id = "odom";
      std::string source_frame_id = _message->header.frame_id;
      ros::Time time_stamp = _message->header.stamp;

      try
      {
          ros::Duration timeout(1.0);
          m_tfListener.waitForTransform(destination_frame_id, source_frame_id, time_stamp, timeout);
          m_tfListener.lookupTransform(destination_frame_id, source_frame_id, time_stamp, transform);
      }catch(tf::TransformException& ex){
          ROS_ERROR_STREAM( "Failed to get the transformation: "
          << ex.what() << ", quitting callback");
          return;
      }

      // STEP 2
      m_occ->ensureInitialise(transform);

      // STEP 3
      tf::Vector3 Origin = transform * tf::Vector3(0, 0, 0);
      tf::Vector3 Direction;

      int i = 0;
      for (float alpha = _message->angle_min; alpha < _message->angle_max; alpha += _message->angle_increment){

          Direction = ((transform * tf::Vector3(cos(alpha), sin(alpha), 0)) - Origin).normalized();
          if(isnan(_message->ranges[i])){
            m_occ->rayTrace(robot_size, Origin, Direction, _message->range_max, false);

          }else{
            m_occ->rayTrace(robot_size, Origin, Direction, _message->ranges[i], true);

          }
          i++;
      }
  }


    bool mapService(nav_msgs::GetMapRequest& _req, nav_msgs::GetMapResponse& _resp)
    {
      return m_occ->requestMap(_resp);
    }

  private:
    ros::NodeHandle m_nodeHandle;
    OCC* m_occ;
    double m_cell_size, m_robot_size;

    // Fill in
    double grid_cell_size = 0.1;
    double robot_size = 1.0;


    };

    int main(int argc, char** argv){
    ros::init(argc, argv, "ls_to_occ");
    ros::NodeHandle n;

    LStoOCC ptd(n);

    ros::spin();
    return 0;
}
