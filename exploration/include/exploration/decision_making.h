#ifndef DECISION_MAKING_H_RF_
#define DECISION_MAKING_H_RF_

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Robot
{
 public:
  Robot(const std::string &name);
  ~Robot();
  bool is_ready()
;
  void initialize();
  void move(const geometry_msgs::Pose &pose);
  bool getRobotPose();
  double get_cost(const geometry_msgs::Pose &goal) const;
  void get_less_cost(const std::vector<geometry_msgs::Pose> &frontiers, 
		     double &cost_min,
		     geometry_msgs::Pose &frontier);

 private:
  std::string name_;
  tf::TransformListener tf_;
  geometry_msgs::PoseStamped pose_;
  MoveBaseClient *move_base_client_;
  ros::Subscriber costmap_subscriber_;
  bool on_action_;
  bool initialized_;
};

class DecisionMaking
{
 public:
  DecisionMaking(ros::NodeHandle &n, tf::TransformListener& tf);
  ~DecisionMaking();
  void initialize();
 protected:
  void frontiers_callback(const geometry_msgs::PoseArray &frontiers);
  bool pose_equal(const geometry_msgs::Pose &a, 
		  const geometry_msgs::Pose &b) const;
  /*  void get_cost_by_frontiers(const geometry_msgs::PoseStamped 
			     &robot_pose,
			     const geometry_msgs::PoseArray
			     &frontiers,
			     std::vector<double> &cost_robot);
  *//*
  bool worldToMap(const double wx, const double wy, 
		  unsigned int& mx, unsigned int& my) const;
  double get_cost(const geometry_msgs::PoseStamped &robot_pose,
		  const geometry_msgs::Pose goal);
    */
 private:
  ros::NodeHandle nh;
  tf::TransformListener tf_;
  ros::Subscriber frontier_subscriber_;
  
  std::vector<Robot *> robots_;
  std::vector<geometry_msgs::Pose> frontiers_reach;
  bool initialized_;
};

#endif
