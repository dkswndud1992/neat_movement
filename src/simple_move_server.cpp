#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <neat_movement/SimpleMoveAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <neat_movement/SimpleMoveParamsConfig.h>
#include <cmath>

class SimpleMoveServer
{
public:
  SimpleMoveServer(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      as_(nh_, "simple_move", boost::bind(&SimpleMoveServer::executeCallback, this, _1), false),
      current_linear_vel_(0.0),
      current_angular_vel_(0.0),
      obstacle_speed_ratio_(1.0)
  {
    // Load parameters
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");

    // Dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<neat_movement::SimpleMoveParamsConfig>(private_nh_);
    dynamic_reconfigure::Server<neat_movement::SimpleMoveParamsConfig>::CallbackType cb;
    cb = boost::bind(&SimpleMoveServer::reconfigureCallback, this, _1, _2);
    dsrv_->setCallback(cb);

    // Publishers and subscribers
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &SimpleMoveServer::odomCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &SimpleMoveServer::scanCallback, this);

    // Start action server
    as_.start();
    ROS_INFO("SimpleMoveServer started");
  }

  ~SimpleMoveServer()
  {
    delete dsrv_;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  actionlib::SimpleActionServer<neat_movement::SimpleMoveAction> as_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;

  dynamic_reconfigure::Server<neat_movement::SimpleMoveParamsConfig>* dsrv_;
  neat_movement::SimpleMoveParamsConfig config_;

  std::string odom_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string base_frame_;

  geometry_msgs::PoseStamped current_pose_;
  double current_linear_vel_;
  double current_angular_vel_;
  double obstacle_speed_ratio_;
  ros::Time last_cmd_time_;

  void reconfigureCallback(neat_movement::SimpleMoveParamsConfig &config, uint32_t level)
  {
    config_ = config;
    ROS_INFO("Reconfigure request received");
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    // Check obstacles in front/back based on robot dimensions
    double min_distance = std::numeric_limits<double>::max();
    
    int num_ranges = msg->ranges.size();
    double angle_increment = msg->angle_increment;
    double angle_min = msg->angle_min;
    
    // Check forward arc (±45 degrees)
    double check_angle = M_PI / 4.0;
    
    for (int i = 0; i < num_ranges; ++i)
    {
      double angle = angle_min + i * angle_increment;
      
      // Check if angle is in forward sector
      if (std::abs(angle) < check_angle)
      {
        double range = msg->ranges[i];
        if (std::isfinite(range) && range > msg->range_min && range < msg->range_max)
        {
          // Account for robot width
          double lateral_offset = std::abs(range * std::sin(angle));
          if (lateral_offset < config_.robot_width / 2.0)
          {
            min_distance = std::min(min_distance, range);
          }
        }
      }
    }

    // Calculate speed ratio based on obstacle distance
    if (min_distance < config_.stop_zone_distance)
    {
      obstacle_speed_ratio_ = 0.0;
    }
    else if (min_distance < config_.decel_zone_distance)
    {
      double ratio = (min_distance - config_.stop_zone_distance) / 
                     (config_.decel_zone_distance - config_.stop_zone_distance);
      obstacle_speed_ratio_ = config_.decel_zone_speed_ratio * ratio;
    }
    else
    {
      obstacle_speed_ratio_ = 1.0;
    }
  }

  void executeCallback(const neat_movement::SimpleMoveGoalConstPtr& goal)
  {
    ROS_INFO("Received new goal");
    
    ros::Rate rate(config_.control_frequency);
    neat_movement::SimpleMoveFeedback feedback;
    neat_movement::SimpleMoveResult result;

    geometry_msgs::PoseStamped target = goal->target_pose;
    bool drive_forward = goal->drive_forward;

    // Phase 1: Rotate to target direction
    if (!rotateToTarget(target, drive_forward, rate, feedback))
    {
      result.success = false;
      result.message = "Rotation failed or preempted";
      as_.setAborted(result);
      stopRobot();
      return;
    }

    // Phase 2: Drive straight to target
    if (!driveToTarget(target, drive_forward, rate, feedback))
    {
      result.success = false;
      result.message = "Drive failed or preempted";
      as_.setAborted(result);
      stopRobot();
      return;
    }

    // Success
    stopRobot();
    result.success = true;
    result.message = "Goal reached";
    as_.setSucceeded(result);
    ROS_INFO("Goal reached successfully");
  }

  bool rotateToTarget(const geometry_msgs::PoseStamped& target, bool drive_forward, 
                     ros::Rate& rate, neat_movement::SimpleMoveFeedback& feedback)
  {
    ROS_INFO("Starting rotation phase");
    
    double prev_error = 0.0;
    last_cmd_time_ = ros::Time::now();

    while (ros::ok())
    {
      if (as_.isPreemptRequested())
      {
        return false;
      }

      // Calculate target yaw
      double dx = target.pose.position.x - current_pose_.pose.position.x;
      double dy = target.pose.position.y - current_pose_.pose.position.y;
      double target_yaw = std::atan2(dy, dx);
      
      // If driving backward, add 180 degrees
      if (!drive_forward)
      {
        target_yaw = std::atan2(-dy, -dx);
      }

      // Current yaw
      double current_yaw = tf2::getYaw(current_pose_.pose.orientation);

      // Angular error
      double error = target_yaw - current_yaw;
      
      // Normalize to [-pi, pi]
      while (error > M_PI) error -= 2.0 * M_PI;
      while (error < -M_PI) error += 2.0 * M_PI;

      // Check if rotation is complete
      if (std::abs(error) < config_.rotation_tolerance)
      {
        ROS_INFO("Rotation complete");
        stopRobot();
        return true;
      }

      // PD control
      double dt = (ros::Time::now() - last_cmd_time_).toSec();
      if (dt < 0.001) dt = 0.001;
      
      double derivative = (error - prev_error) / dt;
      double angular_vel = config_.rotation_kp * error + config_.rotation_kd * derivative;

      // Apply angular velocity limits
      angular_vel = std::max(-config_.max_angular_vel, std::min(config_.max_angular_vel, angular_vel));

      // Apply acceleration limits
      double max_vel_change = config_.max_angular_accel * dt;
      double vel_diff = angular_vel - current_angular_vel_;
      if (std::abs(vel_diff) > max_vel_change)
      {
        angular_vel = current_angular_vel_ + std::copysign(max_vel_change, vel_diff);
      }

      // Publish command
      geometry_msgs::Twist cmd;
      cmd.angular.z = angular_vel;
      cmd_vel_pub_.publish(cmd);

      current_angular_vel_ = angular_vel;
      prev_error = error;
      last_cmd_time_ = ros::Time::now();

      // Publish feedback
      feedback.current_pose = current_pose_;
      double distance = std::sqrt(dx * dx + dy * dy);
      feedback.distance_to_goal = distance;
      as_.publishFeedback(feedback);

      rate.sleep();
    }

    return false;
  }

  bool driveToTarget(const geometry_msgs::PoseStamped& target, bool drive_forward,
                    ros::Rate& rate, neat_movement::SimpleMoveFeedback& feedback)
  {
    ROS_INFO("Starting drive phase");
    
    last_cmd_time_ = ros::Time::now();

    while (ros::ok())
    {
      if (as_.isPreemptRequested())
      {
        return false;
      }

      // Calculate distance to goal
      double dx = target.pose.position.x - current_pose_.pose.position.x;
      double dy = target.pose.position.y - current_pose_.pose.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      // Check if goal reached
      if (distance < config_.xy_goal_tolerance)
      {
        ROS_INFO("Drive complete");
        stopRobot();
        return true;
      }

      // Determine desired speed
      double max_speed = drive_forward ? config_.max_linear_vel_forward : config_.max_linear_vel_backward;
      double desired_speed = max_speed;

      // Apply obstacle-based speed ratio
      desired_speed *= obstacle_speed_ratio_;

      // Apply direction
      if (!drive_forward)
      {
        desired_speed = -desired_speed;
      }

      // Apply acceleration/deceleration limits
      double dt = (ros::Time::now() - last_cmd_time_).toSec();
      if (dt < 0.001) dt = 0.001;

      double vel_diff = desired_speed - current_linear_vel_;
      double max_accel = (vel_diff > 0) ? config_.max_linear_accel : config_.max_linear_decel;
      double max_vel_change = max_accel * dt;

      if (std::abs(vel_diff) > max_vel_change)
      {
        desired_speed = current_linear_vel_ + std::copysign(max_vel_change, vel_diff);
      }

      // Publish command
      geometry_msgs::Twist cmd;
      cmd.linear.x = desired_speed;
      cmd_vel_pub_.publish(cmd);

      current_linear_vel_ = desired_speed;
      last_cmd_time_ = ros::Time::now();

      // Publish feedback
      feedback.current_pose = current_pose_;
      feedback.distance_to_goal = distance;
      as_.publishFeedback(feedback);

      rate.sleep();
    }

    return false;
  }

  void stopRobot()
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd);
    current_linear_vel_ = 0.0;
    current_angular_vel_ = 0.0;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_move_server");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  SimpleMoveServer server(nh, private_nh);
  ros::spin();

  return 0;
}
