#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include "navfn/navfn_ros.h"

#include <iostream>
#include <fstream>
#include <stdio.h>

using std::ofstream;
using std::endl;

double x_goals[15] = {7.7107, 10.6194, 24.2291, 24.0688, 24.3618, 24.3742, 24.8803, 17.5337, 16.9325, 16.8404, 16.7939, 11.8312, 11.1705, 18.0573, 10.5424};
double y_goals[15] = {-0.0256, 11.4103, 21.1727, 15.6072,  9.5433,  3.2581, -2.6946, -4.6317, -11.6154, -17.4720, -23.4382, -22.1283, -7.8764, 22.0136, 16.9826};

namespace costmap_2d {
  class Costmap2DNode {
  public:
    Costmap2DNode(tf::TransformListener& tf) : costmap_ros_("costmap", tf){}
  private:
    Costmap2DROS costmap_ros_;
  };
};

int main(int argc, char** argv){
  ros::init(argc, argv, "costmap_node");

  tf::TransformListener tf(ros::Duration(10));

  costmap_2d::Costmap2DROS* costmap_node;
  costmap_node = new costmap_2d::Costmap2DROS("costmap",tf);

  navfn::NavfnROS navfn;
  navfn.initialize("my_navfn_planner", costmap_node);

  //ROS_INFO("MAP SIZE: %d, %d", costmap_node->getSizeInCellsX(), costmap_node->getSizeInCellsY());

  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;

  for (int startNo = 0; startNo < 15; startNo++) {
    for (int goalNo = 0; goalNo < 15; goalNo++) {
               
      start.header.seq = 0;  
      start.header.stamp = ros::Time::now();
      start.header.frame_id = "/map";

      goal.header.seq = 1;    
      goal.header.stamp = ros::Time::now();
      goal.header.frame_id = "/map";

      start.pose.position.x = x_goals[startNo];
      start.pose.position.y = y_goals[startNo];
      start.pose.position.z = 0;
      start.pose.orientation.x = 0;
      start.pose.orientation.y = 0;
      start.pose.orientation.z = 0;
      start.pose.orientation.w = 0;

      goal.pose.position.x = x_goals[goalNo];
      goal.pose.position.y = y_goals[goalNo];
      goal.pose.position.z = 0;
      goal.pose.orientation.x = 0;
      goal.pose.orientation.y = 0;
      goal.pose.orientation.z = 0;
      goal.pose.orientation.w = 0;

      std::vector<geometry_msgs::PoseStamped> plan;

      navfn.makePlan(start, goal, 0.1, plan);
      navfn.publishPlan(plan,1,0,0,0);

      ROS_INFO("START %d GOAL %d PLAN SIZE: %d", startNo, goalNo, (int)plan.size());

      char buffer[150];
      sprintf(buffer,"/home/will/Data/SBlock/level7_plans/path_start_%02d_goal_%02d.csv",startNo,goalNo);
      ofstream logFile;
      logFile.open(buffer);
      for (int i = 0; i < (int)plan.size(); i++) {
	logFile << plan[i].pose.position.x << "," << plan[i].pose.position.y << endl;
      }
      logFile.close();
      ROS_INFO("Saved path to %s", buffer);

      ros::Duration(0.5).sleep();

    }
  }


  ros::spin();

  delete costmap_node;

  return(0);
}



