#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <exploration/exploration.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ExplorationPlanner
{
public:
  ExplorationPlanner(const std::string &name,
                           costmap_2d::Costmap2DROS &costmap_ros,
                           dwa_local_planner::DWAPlannerROS &dwa_planner_ros,
                           ros::NodeHandle &nh,
                           tf::TransformListener &tf)
  {
      Exploration walker;
      walker.initialized(name, costmap_ros,
                         dwa_planner_ros, nh, tf);
      tf::Stamped<tf::Pose> robot_pose_tf;
      costmap_ros.getRobotPose(robot_pose_tf);

      geometry_msgs::PoseStamped start;
      tf::poseStampedTFToMsg(robot_pose_tf, start);
      std::vector<geometry_msgs::PoseStamped> plan;
      while(walker.doExploration(start, plan))
      {
        geometry_msgs::PoseStamped thisgoal = plan.back();
        MoveBaseClient action_client("move_base", true);

      while(!action_client.waitForServer(ros::Duration(5.0)))
      {
          ROS_INFO("Waiting for the move_base action server...");
      }
      move_base_msgs::MoveBaseGoal goal_line;

      goal_line.target_pose.header.frame_id = "map";
      goal_line.target_pose.header.stamp = ros::Time::now();
      ROS_INFO("Start point %f %f goal %f %f",
               start.pose.position.x, start.pose.position.y,
               thisgoal.pose.position.x, thisgoal.pose.position.y);

      goal_line.target_pose.pose.position.x = thisgoal.pose.position.x;
      goal_line.target_pose.pose.position.y = thisgoal.pose.position.y;
      goal_line.target_pose.pose.position.z = thisgoal.pose.position.z;
      goal_line.target_pose.pose.orientation.w = thisgoal.pose.orientation.w;
      goal_line.target_pose.pose.orientation.x = thisgoal.pose.orientation.x;
      goal_line.target_pose.pose.orientation.y = thisgoal.pose.orientation.y;
      goal_line.target_pose.pose.orientation.z = thisgoal.pose.orientation.z;

      action_client.sendGoal(goal_line);
      actionlib::SimpleClientGoalState state =
              action_client.getState();
      do
      {
          bool finished_before_timeout =
                  action_client.waitForResult(ros::Duration(5.0));
          state = action_client.getState();
          ROS_INFO("[Exploration] The turtlebot didn't finish its move... %s",
                   state.toString().c_str());

      }while(!state.isDone());
      if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          ROS_INFO("[Exploration] The turtlebot didn't finish its move... %s",
                   state.toString().c_str());
      }
      tf::poseStampedTFToMsg(robot_pose_tf, start);
      }

      costmap_ros.getCostmap()->saveMap("/home/turtle/turtle_bot/map.pgm");
   }

 protected:
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "exploration");

  //  ROS_INFO("[Exploration] Initialise");
  ros::NodeHandle nh;

  tf::TransformListener tfl_(ros::Duration(10));

  costmap_2d::Costmap2DROS costmap_ros("global_costmap", tfl_);
  costmap_ros.start();

  costmap_2d::Costmap2DROS local_costmap_ros("local_costmap", tfl_);
  local_costmap_ros.start();
  dwa_local_planner::DWAPlannerROS dwa_planner_ros;
  dwa_planner_ros.initialize("dwa_planner", &tfl_, &local_costmap_ros);
  ExplorationPlanner walker("exploration", costmap_ros,
                            dwa_planner_ros, nh, tfl_);
  ros::spin();
  return EXIT_SUCCESS;
}

