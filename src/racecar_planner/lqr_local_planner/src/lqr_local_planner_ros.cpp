#include "lqr_local_planner_ros.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(lqr_local_planner::LqrLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace lqr_local_planner
{
/**
 * @brief Construct a new LqrLocalPlannerROS object
 */
LqrLocalPlannerROS::LqrLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
}

/**
 * @brief Construct a new LqrLocalPlannerROS object
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
LqrLocalPlannerROS::LqrLocalPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the LqrLocalPlannerROS object
 */
LqrLocalPlannerROS::~LqrLocalPlannerROS()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void LqrLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // ROS_WARN("LqrLocalPlannerROS::initialize");
  if (!initialized_)
  {
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    initialized_ = true;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    nh.param("p_window", p_window_, 0.5);
    nh.param("o_window", o_window_, 1.0);

    nh.param("p_precision", p_precision_, 0.2);
    nh.param("o_precision", o_precision_, 0.5);

    nh.param("max_v", max_v_, 1.0);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    nh.param("max_w", max_w_, 1.0);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.0);

    nh.param("wheelbase", wheelbase_, 0.5);

    // nh.param("k_v_p", k_v_p_, 1.00);
    // nh.param("k_v_i", k_v_i_, 0.01);
    // nh.param("k_v_d", k_v_d_, 0.10);

    // nh.param("k_w_p", k_w_p_, 1.00);
    // nh.param("k_w_i", k_w_i_, 0.01);
    // nh.param("k_w_d", k_w_d_, 0.10);

    // nh.param("k_theta", k_theta_, 0.5);

    // e_v_ = i_v_ = 0.0;
    // e_w_ = i_w_ = 0.0;

    // odom_helper_ = new base_local_planner::OdometryHelperRos("/odom");
    // target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    // current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    // base_frame_ = "base_link";
    // plan_index_ = 0;
    // x_ = y_ = theta_ = 0.0;

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    ROS_INFO("LQR planner initialized!");
  }
  else
  {
    ROS_WARN("LQR planner has already been initialized.");
  }
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool LqrLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // ROS_WARN("LqrLocalPlannerROS::setPlan");
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // ROS_INFO("Got new plan");

  // set new plan
  global_plan_ = orig_global_plan;

  // reset plan parameters
  plan_index_ = 0;  // NOTE: ~~set to 3 to avoid getting wrong closest point on the path~~
  goal_reached_ = false;

  return true;
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to
 * the base
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return  true if a valid trajectory was found, else false
 */
bool LqrLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // ROS_WARN("LqrLocalPlannerROS::computeVelocityCommands");
  if (!initialized_)
  {
    ROS_ERROR("LQR planner has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_ERROR("LQR planner goal reached without motion.");
    return true;
  }

  // current pose
  costmap_ros_->getRobotPose(current_ps_);
  x_ = current_ps_.pose.position.x;
  y_ = current_ps_.pose.position.y;
  theta_ = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]
  // ROS_WARN("theta_ %.2f", theta_);

  double theta_d, theta_dir, theta_trj;
  double b_x_d, b_y_d;  // desired x, y in base frame
  double e_theta;

  while (plan_index_ < global_plan_.size())
  {
    target_ps_ = global_plan_[plan_index_];
    double x_d = target_ps_.pose.position.x;
    double y_d = target_ps_.pose.position.y;

    // from robot to plan point
    theta_dir = atan2((y_d - y_), (x_d - x_));  // [-pi, pi]

    int next_plan_index = plan_index_ + 1;
    if (next_plan_index < global_plan_.size())
      // theta on the trajectory
      theta_trj = atan2((global_plan_[next_plan_index].pose.position.y - y_d),
                        (global_plan_[next_plan_index].pose.position.x - x_d));

    // if the difference is greater than PI, it will get a wrong result
    if (std::fabs(theta_trj - theta_dir) > M_PI)
    {
      // add 2*PI to the smaller one
      if (theta_trj > theta_dir)
        theta_dir += 2 * M_PI;
      else
        theta_trj += 2 * M_PI;
    }

    // weighting between two angle
    theta_d = (1 - k_theta_) * theta_trj + k_theta_ * theta_dir;
    regularizeAngle(theta_d);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_d);
    tf2::convert(q, target_ps_.pose.orientation);

    // transform from map into base_frame
    getTransformedPosition(target_ps_, b_x_d, b_y_d);

    e_theta = theta_d - theta_;
    regularizeAngle(e_theta);

    if (std::hypot(b_x_d, b_y_d) > p_window_)  // || std::fabs(e_theta) > o_window_
      break;

    plan_index_++;
  }

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get final goal orientation - Quaternion to Euler
  // final_rpy_ = getEulerAngles(global_plan_.back());
  // ROS_WARN("final_rpy_[2] = %.2f", final_rpy_[2]);

  // position reached
  if (getGoalPositionDistance(global_plan_.back(), x_, y_) < p_precision_)
  {
    // double fe_theta = final_rpy_[2] - theta_;
    // regularizeAngle(fe_theta);

    // orientation reached
    // if (std::fabs(fe_theta) < o_precision_)
    // {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    goal_reached_ = true;
    // }
    // // orientation not reached
    // else
    // {
    //   cmd_vel.linear.x = 0.0;
    //   cmd_vel.angular.z = AngularPIDController(base_odom, final_rpy_[2], theta_);
    // }
  }
  // // large angle, turn first
  // else if (std::fabs(e_theta) > M_PI_2)
  // {
  //   cmd_vel.linear.x = 0.0;
  //   cmd_vel.angular.z = AngularPIDController(base_odom, theta_d, theta_);
  // }
  // posistion not reached
  else
  {
    // cmd_vel.linear.x = LinearPIDController(base_odom, b_x_d, b_y_d);
    // cmd_vel.angular.z = AngularPIDController(base_odom, theta_d, theta_);
  }

  ROS_INFO("velocity = %.2f m/s, omega = %.2f rad/s", cmd_vel.linear.x, cmd_vel.angular.z);

  // publish next target_ps_ pose
  target_ps_.header.frame_id = "map";
  target_ps_.header.stamp = ros::Time::now();
  target_pose_pub_.publish(target_ps_);

  // publish robot pose
  current_ps_.header.frame_id = "map";
  current_ps_.header.stamp = ros::Time::now();
  current_pose_pub_.publish(current_ps_);

  return true;
}

/**
 * @brief LQR controller in linear
 * @param base_odometry odometry of the robot, to get velocity
 * @param b_x_d         desired x in body frame
 * @param b_y_d         desired y in body frame
 * @return  linear velocity
 */
double LqrLocalPlannerROS::LqrController(nav_msgs::Odometry& base_odometry, double b_x_d, double b_y_d)
{
  double v = std::hypot(base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);
  double v_d = std::hypot(b_x_d, b_y_d) / d_t_;
  if (std::fabs(v_d) > max_v_)
    v_d = std::copysign(max_v_, v_d);
  // ROS_WARN("v_d: %.2f", v_d);

  double e_v = v_d - v;
  i_v_ += e_v * d_t_;
  double d_v = (e_v - e_v_) / d_t_;
  e_v_ = e_v;

  double v_inc = k_v_p_ * e_v + k_v_i_ * i_v_ + k_v_d_ * d_v;

  if (std::fabs(v_inc) > max_v_inc_)
    v_inc = std::copysign(max_v_inc_, v_inc);

  double v_cmd = v + v_inc;
  if (std::fabs(v_cmd) > max_v_)
    v_cmd = std::copysign(max_v_, v_cmd);
  else if (std::fabs(v_cmd) < min_v_)
    v_cmd = std::copysign(min_v_, v_cmd);

  // ROS_INFO("v_d: %.2lf, e_v: %.2lf, i_v: %.2lf, d_v: %.2lf, v_cmd: %.2lf", v_d, e_v, i_v_, d_v, v_cmd);

  return v_cmd;
}

/**
 * @brief LQR controller in angular
 * @param base_odometry odometry of the robot, to get velocity
 * @param theta_d       desired theta
 * @param theta         current theta
 * @return  angular velocity
 */
// double LqrLocalPlannerROS::AngularPIDController(nav_msgs::Odometry& base_odometry, double theta_d, double theta)
// {
//   double e_theta = theta_d - theta;
//   regularizeAngle(e_theta);

//   double w_d = e_theta / d_t_;
//   if (std::fabs(w_d) > max_w_)
//     w_d = std::copysign(max_w_, w_d);
//   // ROS_WARN("w_d: %.2f", w_d);

//   double w = base_odometry.twist.twist.angular.z;
//   double e_w = w_d - w;
//   i_w_ += e_w * d_t_;
//   double d_w = (e_w - e_w_) / d_t_;
//   e_w_ = e_w;

//   double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

//   if (std::fabs(w_inc) > max_w_inc_)
//     w_inc = std::copysign(max_w_inc_, w_inc);

//   double w_cmd = w + w_inc;
//   if (std::fabs(w_cmd) > max_w_)
//     w_cmd = std::copysign(max_w_, w_cmd);
//   else if (std::fabs(w_cmd) < min_w_)
//     w_cmd = std::copysign(min_w_, w_cmd);

//   // ROS_INFO("w_d: %.2lf, e_w: %.2lf, i_w: %.2lf, d_w: %.2lf, w_cmd: %.2lf", w_d, e_w, i_w_, d_w, w_cmd);

//   return w_cmd;
// }

/**
 * @brief  Check if the goal pose has been achieved
 *
 * @return True if achieved, false otherwise
 */
bool LqrLocalPlannerROS::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  if (!costmap_ros_->getRobotPose(current_ps_))
  {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("Goal reached");
    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
    return true;
  }

  if (plan_index_ > global_plan_.size() - 1)
  {
    if (getGoalPositionDistance(global_plan_.back(), x_, y_) < p_precision_)  // && std::fabs(final_rpy_[2] - theta_)
                                                                              // < o_precision_
    {
      goal_reached_ = true;
      robotStops();
      e_v_ = i_v_ = 0.0;
      e_w_ = i_w_ = 0.0;
      ROS_INFO("Goal reached");
    }
  }
  return goal_reached_;
}

/**
 * @brief Get the distance to the goal
 * @param goal_ps global goal PoseStamped
 * @param x       global current x
 * @param y       global current y
 * @return the distance to the goal
 */
double LqrLocalPlannerROS::getGoalPositionDistance(const geometry_msgs::PoseStamped& goal_ps, double x, double y)
{
  return std::hypot(x - goal_ps.pose.position.x, y - goal_ps.pose.position.y);
}

/**
 * @brief Get the Euler Angles from PoseStamped
 * @param ps  PoseStamped to calculate
 * @return  roll, pitch and yaw in XYZ order
 */
// std::vector<double> LqrLocalPlannerROS::getEulerAngles(geometry_msgs::PoseStamped& ps)
// {
//   std::vector<double> EulerAngles;
//   EulerAngles.resize(3, 0.0);

//   tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
//   tf2::Matrix3x3 m(q);

//   m.getRPY(EulerAngles[0], EulerAngles[1], EulerAngles[2]);
//   return EulerAngles;
// }

/**
 * @brief Regularize angle to [-pi, pi]
 * @param angle the angle (rad) to regularize
 */
void LqrLocalPlannerROS::regularizeAngle(double& angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;

  while (angle <= -M_PI)
    angle += 2 * M_PI;
}
}  // namespace lqr_local_planner
