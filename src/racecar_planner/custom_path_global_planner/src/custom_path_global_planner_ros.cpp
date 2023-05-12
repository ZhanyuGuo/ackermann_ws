#include <pluginlib/class_list_macros.h>
#include "custom_path_global_planner_ros.h"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(custom_path_global_planner::CustomPathGlobalPlannerROS, nav_core::BaseGlobalPlanner)

using namespace std;

// Default Constructor
namespace custom_path_global_planner
{

CustomPathGlobalPlannerROS::CustomPathGlobalPlannerROS() : initialized_(false)
{
}

CustomPathGlobalPlannerROS::CustomPathGlobalPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : CustomPathGlobalPlannerROS()
{
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void CustomPathGlobalPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  initialize(name, costmapRos->getCostmap(), costmapRos->getGlobalFrameID());
}

void CustomPathGlobalPlannerROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{
  if (!initialized_)
  {
    frame_id_ = frame_id;

    ros::NodeHandle private_nh("~/" + name);

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

    initialized_ = true;
  }
  else
  {
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }
}

bool CustomPathGlobalPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                                          const geometry_msgs::PoseStamped& goal,
                                          std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  // clear existing plan
  plan.clear();

  /********
   * Line *
   ********/
  // plan.push_back(start);
  // for (int i = 0; i < 20; i++)
  // {
  //   geometry_msgs::PoseStamped new_goal = goal;
  //   new_goal.pose.position.x = start.pose.position.x + (0.5 * i);
  //   new_goal.pose.position.y = start.pose.position.y + (0.5 * i);

  //   plan.push_back(new_goal);
  // }
  // plan.push_back(goal);

  /**********
   * Circle *
   **********/
  double x = start.pose.position.x;
  double y = start.pose.position.y;

  double x_c = 0.0;
  double y_c = 0.0;
  double r = 3.0;

  std::vector<geometry_msgs::PoseStamped> path;
  int path_size = 100;
  path.reserve(path_size);

  std::vector<geometry_msgs::PoseStamped>::iterator min_iter;
  double min_dis = 1000.0;

  for (int i = 0; i < path_size; i++)
  {
    double theta = (2.0 * M_PI * i) / path_size;
    geometry_msgs::PoseStamped new_goal = goal;
    double x_d = x_c + r * std::cos(theta);
    double y_d = y_c + r * std::sin(theta);

    new_goal.pose.position.x = x_d;
    new_goal.pose.position.y = y_d;

    path.push_back(new_goal);

    double dis = std::hypot(x_d - x, y_d - y);
    if (dis < min_dis)
    {
      min_iter = path.end() - 1;
      min_dis = dis;
    }
  }

  plan.assign(min_iter, path.end());
  plan.insert(plan.end(), path.begin(), min_iter);

  // publish plan for visualization in rviz
  publishPlan(plan);

  return true;
}

void CustomPathGlobalPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  // create visulized path plan
  nav_msgs::Path gui_plan;
  gui_plan.poses.resize(plan.size());
  gui_plan.header.frame_id = frame_id_;
  gui_plan.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < plan.size(); i++)
    gui_plan.poses[i] = plan[i];

  // publish plan to rviz
  plan_pub_.publish(gui_plan);
}

};  // namespace custom_path_global_planner