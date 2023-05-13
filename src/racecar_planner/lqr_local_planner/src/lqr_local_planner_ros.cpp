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
    // nh.param("o_window", o_window_, 1.0);

    nh.param("p_precision", p_precision_, 0.2);
    // nh.param("o_precision", o_precision_, 0.5);

    nh.param("max_v", max_v_, 1.0);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    nh.param("max_w", max_w_, 1.0);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.0);

    nh.param("wheelbase", wheelbase_, 0.5);
    nh.param("max_iter", max_iter_, 100);

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
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    // base_frame_ = "base_link";
    plan_index_ = 1;  // NOTE: start from 1 to calculate curvature
    x_ = y_ = theta_ = 0.0;

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
  plan_index_ = 1;  // NOTE: start from 1 to calculate curvature
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
  // ROS_WARN("x = %.2f, y = %.2f, theta = %.2f", x_, y_, theta_);

  double x_d, y_d, theta_d;
  while (plan_index_ < global_plan_.size() - 1)
  {
    target_ps_ = global_plan_[plan_index_];
    x_d = target_ps_.pose.position.x;
    y_d = target_ps_.pose.position.y;

    if (std::hypot(x_d - x_, y_d - y_) > p_window_)
      break;

    plan_index_++;
  }

  theta_d = atan2((global_plan_[plan_index_ + 1].pose.position.y - y_d),
                  (global_plan_[plan_index_ + 1].pose.position.x - x_d));
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_d);
  tf2::convert(q, target_ps_.pose.orientation);

  double v_d = max_v_;
  double kappa = calcCurature();
  double w_d = std::atan2(kappa * wheelbase_, 1);
  // ROS_WARN("kappa = %.2f, w_d = %.2f", kappa, w_d);

  // position reached
  if (getGoalPositionDistance(global_plan_.back(), x_, y_) < p_precision_)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    goal_reached_ = true;
  }
  else
  {
    cmd_vel.linear.x = v_d;
    cmd_vel.angular.z = LqrController(x_d, y_d, theta_d, v_d, w_d);
  }

  // ROS_INFO("velocity = %.2f m/s, omega = %.2f rad/s", cmd_vel.linear.x, cmd_vel.angular.z);

  // publish next target_ps_ pose
  target_ps_.header.frame_id = "map";
  target_ps_.header.stamp = ros::Time::now();
  target_pose_pub_.publish(target_ps_);

  // // publish robot pose
  current_ps_.header.frame_id = "map";
  current_ps_.header.stamp = ros::Time::now();
  current_pose_pub_.publish(current_ps_);

  return true;
}

double LqrLocalPlannerROS::calcCurature()
{
  double ax = global_plan_[plan_index_ - 1].pose.position.x;
  double ay = global_plan_[plan_index_ - 1].pose.position.y;
  double bx = global_plan_[plan_index_].pose.position.x;
  double by = global_plan_[plan_index_].pose.position.y;
  double cx = global_plan_[plan_index_ + 1].pose.position.x;
  double cy = global_plan_[plan_index_ + 1].pose.position.y;

  double a = std::hypot(bx - cx, by - cy);
  double b = std::hypot(cx - ax, cy - ay);
  double c = std::hypot(ax - bx, ay - by);

  double cosB = (a * a + c * c - b * b) / (2 * a * c);
  double sinB = std::sin(std::acos(cosB));
  double kappa = 2 * sinB / b;

  return kappa;
}

/**
 * @brief LQR controller in linear
 * @param x_d desired x
 * @param y_d desired y
 * @param theta_d desired theta
 * @param v_d desired velocity
 * @param w_d desired steering angle
 * @return steering angle command
 */
double LqrLocalPlannerROS::LqrController(double x_d, double y_d, double theta_d, double v_d, double w_d)
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

  Eigen::Vector3d p_d(x_d, y_d, theta_d);
  Eigen::Vector2d u_d(v_d, w_d);
  Eigen::Vector3d e_p = Eigen::Vector3d(x_, y_, theta_) - p_d;
  regularizeAngle(e_p(2));

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
  A(0, 0) = 1.0;
  A(0, 2) = -d_t_ * u_d[0] * std::sin(theta_d);
  A(1, 1) = 1.0;
  A(1, 2) = d_t_ * u_d[0] * std::cos(theta_d);
  A(2, 2) = 1.0;

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 1);
  B(2, 0) = u_d[0] * d_t_ / wheelbase_ / std::pow(std::cos(w_d), 2);

  double eps = 0.01;
  double diff = std::numeric_limits<double>::max();

  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();

  int num_iter = 0;
  while (num_iter++ < max_iter_ && diff > eps)
  {
    Eigen::MatrixXd Pn = AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
    diff = ((Pn - P).array().abs()).maxCoeff();
    P = Pn;
  }
  // ROS_WARN("num_iter: %d, diff: %.2f", num_iter, diff);

  Eigen::MatrixXd feed_back = -((R + BT * P * B).inverse() * BT * P * A) * e_p;
  double w_cmd = feed_back(0, 0) + w_d;
  regularizeAngle(w_cmd);

  return w_cmd;
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
    // e_v_ = i_v_ = 0.0;
    // e_w_ = i_w_ = 0.0;
    return true;
  }

  if (plan_index_ > global_plan_.size() - 1)
  {
    if (getGoalPositionDistance(global_plan_.back(), x_, y_) < p_precision_)  // && std::fabs(final_rpy_[2] - theta_)
                                                                              // < o_precision_
    {
      goal_reached_ = true;
      robotStops();
      // e_v_ = i_v_ = 0.0;
      // e_w_ = i_w_ = 0.0;
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
