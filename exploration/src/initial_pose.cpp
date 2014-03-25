#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

class InitialPose
{
public:
  InitialPose() 
  {
  }
  
  void setPose();
};

void InitialPose::setPose()
{
  ros::NodeHandle private_nh("~");
  ROS_INFO("[DecisionMaking] - initialize : start...");
  double x = 0.0;
  if(!private_nh.getParam("x", x))
    x = 0.0;
    
  double y = 0.0;
  if(!private_nh.getParam("y", y))
    y = 0.0;
  double theta = 0.0;
  if(!private_nh.getParam("a", theta))
    theta = 0.0;
  ros::NodeHandle prefix_nh;
  tf::TransformListener tf_(ros::Duration(10.0));
  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  std::string robot_name_;


  private_nh.param("robot_name", robot_name_, std::string(""));

  std::string robot_base_frame_ = robot_name_ + "/base_link";
  // we need to make sure that the transform between the robot base frame and the global frame is available
  while (ros::ok()
	 && !tf_.waitForTransform("/map", robot_base_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01),
							                                &tf_error))
    {
      ros::spinOnce();
      if (last_error + ros::Duration(5.0) < ros::Time::now())
	{
	  ROS_WARN("[InitialPose] - setPose : Waiting on transform from %s to map to become available before running costmap, tf error: %s",
		   robot_base_frame_.c_str(),
		   tf_error.c_str());
	  last_error = ros::Time::now();
	}
    }


  ROS_INFO("InitialPose to : (%f,%f) angle : %f rad", x, y, theta);
  ros::Publisher pose_publisher_ = ros::NodeHandle().advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  ros::Rate loop_rate(10);
  while(ros::ok())
    {
      geometry_msgs::PoseWithCovarianceStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.pose.pose.position.x = x;
      pose.pose.pose.position.y = y;
      pose.pose.pose.position.z = 0.0;
      tf::Quaternion quat;
      quat.setRPY(0.0, 0.0, theta);
      tf::quaternionTFToMsg(quat,
			    pose.pose.pose.orientation);
      pose.header.frame_id = "/map";
      pose_publisher_.publish(pose);
      loop_rate.sleep();
    }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "initial_pose");
  InitialPose pose;
  pose.setPose();
  ros::Duration(2.0).sleep();
  return EXIT_SUCCESS;
}
