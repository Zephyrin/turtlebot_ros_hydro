
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

class InitialPose
{
public:
  InitialPose();
  ~InitialPose();
protected:
  bool amclPoseCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void sendInitialPose();

private:
  ros::Publisher initialPosePub_;
  tf::TransformBroadcaster m_transformBroadcaster;
  ros::ServiceServer m_pauseOdomSrv;
  ros::ServiceServer m_resumeOdomSrv;
  ros::ServiceServer m_odomOffsetSrv;
  ros::ServiceServer m_setOdomPoseSrv;
  ros::NodeHandle m_nh;
  ros::NodeHandle m_privateNh;

  //for base_footprint_frame: broadcasts frame between right and left foot, coinciding with the ground
  message_filters::Subscriber<sensor_msgs::JointState> * m_jsSub;
  tf::MessageFilter<sensor_msgs::JointState> * m_jsFilter;
  tf::TransformBroadcaster m_brBaseFootPrint;
  tf::TransformListener m_listener;

  geometry_msgs::TransformStamped m_odomTransformMsg;
  tf::Pose m_odomPose; // current "real" odometry pose in original (Nao) odom frame
  tf::Transform m_odomOffset; // offset on odometry origin
  nav_msgs::Odometry m_odom;

  std::string m_odomFrameId;
  std::string m_baseFrameId;
  std::string m_lfootFrameId;
  std::string m_rfootFrameId;
  std::string m_imuTopic;
  std::string m_baseFootPrintID;
  bool m_useIMUAngles;
  bool m_paused;
  double m_lastOdomTime;

  double m_lastWx;
  double m_lastWy;
  double m_lastWz;

  tf::Pose m_targetPose;
  bool m_mustUpdateOffset;
  bool m_initializeFromIMU;
  bool m_initializeFromOdometry;
  bool m_isInitialized;
};

InitialPose::InitialPose()
{
}

OdometryRemap::~OdometryRemap()
{
}

bool OdometryRemap::pauseOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "initialpose_pub");
  InitialPose initialPose();
  ros::spin();

  return 0;
}
