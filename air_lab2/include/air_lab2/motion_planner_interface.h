#include <ros/service.h>


#include <ompl/base/Goal.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "state_validity_checker.h"

class MotionPlannerInterface
{
public:
  MotionPlannerInterface()
  {
  }
  /**
   * Plan a path between (@p _start_x, @p _start_y) and (@p _dest_x, @p _dest_y), @p _frame_id contains the frame_id of the path.
   *
   * @return a vector of pair with the waypoints
   */
  template<typename _ServiceT_>
  std::vector<std::pair<double, double>> planPath(double _start_x, double _start_y, double _dest_x, double _dest_y, std::string* _frame_id)
  {
    std::vector<std::pair<double, double>> path;

    namespace ob = ompl::base;
    namespace og = ompl::geometric;

    _ServiceT_ req;
    if(ros::service::call<_ServiceT_>("map_request", req))
    {
      * _frame_id = req.response.map.header.frame_id;
      ob::StateSpacePtr space(new ob::SE2StateSpace());
      ob::RealVectorBounds bounds(2);
      bounds.setLow(0, req.response.map.info.origin.position.x);
      bounds.setHigh(0, req.response.map.info.resolution * req.response.map.info.width + req.response.map.info.origin.position.x);
      bounds.setLow(1, req.response.map.info.origin.position.y);
      bounds.setHigh(1, req.response.map.info.resolution * req.response.map.info.height + req.response.map.info.origin.position.y);
      space->as<ob::SE2StateSpace>()->setBounds(bounds);

      ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
      si->setStateValidityCheckingResolution(req.response.map.info.resolution / space->getMaximumExtent());

      ob::StateValidityCheckerPtr svc(new StateValidityChecker<_ServiceT_>(req.response, si.get(), true));
      si->setStateValidityChecker(svc);

      ob::ScopedState<> start(space);
      start[0] = _start_x;
      start[1] = _start_y;
      start[2] = 0;

      start.print();

      ob::ScopedState<> goal(space);
      goal[0] = _dest_x;
      goal[1] = _dest_y;
      goal[2] = 0;

      //  Create an instance of ompl::base::ProblemDefinition
      ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

      //  Set the start and goal states for the problem definition.
      pdef->setStartAndGoalStates(start, goal);

//       ob::PlannerPtr planner(new og::RRTConnect(si));

      ob::PlannerPtr planner(new og::RRTstar(si));
      // Set the problem instance for our planner to solve
      planner->setProblemDefinition(pdef);
      planner->setup();


      ob::PlannerStatus solved = planner->solve(1.0);

      if (solved)
      {

        og::PathSimplifierPtr psp(new og::PathSimplifier(si));
        og::PathGeometric pg = *static_cast<og::PathGeometric*>(pdef->getSolutionPath().get());
        psp->simplify(pg, 60.0);
        psp->reduceVertices(pg);
        for(ob::State* s : pg.getStates())
        {
          const ob::SE2StateSpace::StateType* ss = s->as<ob::SE2StateSpace::StateType>();
          path.push_back({ss->getX(), ss->getY()});
        }

      }



    } else {
      ROS_ERROR("Failed to get map from server!");
    }
    return path;

  }
private:
  ros::NodeHandle m_nodeHandle;
  ros::ServiceServer m_planRequest;
};
