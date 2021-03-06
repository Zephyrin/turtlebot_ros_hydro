#include "exploration/decision_making.h"
#include <XmlRpcValue.h>
#include <visualization_msgs/Marker.h>

Robot::Robot(const std::string &name)
:
  name_(name),
  tf_(ros::Duration(10)),
  pose_(),
  move_base_client_(NULL),
  on_action_(false),
  initialized_(false)
{

}

Robot::~Robot()
{
  if(move_base_client_ != NULL)
    {
      delete move_base_client_;
      move_base_client_ = NULL;
    }
}

void Robot::initialize()
{
  if(initialized_)
    {
      ROS_INFO("[Robot] - initiliase : Robot initialize twice...");
      return;
    }
  ros::NodeHandle private_nh("~");
  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  
  // we need to make sure that the transform between the robot base frame and the global frame is available
  while (ros::ok()
	 && !tf_.waitForTransform("/map", "/" + name_ 
				  + "/base_footprint", 
				  ros::Time(), 
				  ros::Duration(0.1), 
				  ros::Duration(0.01),
				  &tf_error))
    {
      ros::spinOnce();
      if (last_error + ros::Duration(5.0) < ros::Time::now())
	{
	  ROS_WARN("[BuildMap] - Waiting on transform from %s to map to become available before running the map, tf error: %s",
		   name_.c_str(), tf_error.c_str());
	  last_error = ros::Time::now();
	}
    }
  while(!getRobotPose());
  move_base_client_ = new MoveBaseClient("/" + name_ + "/move_base", 
					 true);

  //  costmap_subscriber_ = private_nh.subscribe("/" + name_ + "/move_base/global_costmap", 1);
  initialized_ = true;
}

void Robot::move(const geometry_msgs::Pose &pose)
{
  while(!move_base_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server...");
    }
  move_base_msgs::MoveBaseGoal goal;
  std::string robot_name = name_;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = pose.position.x;
  goal.target_pose.pose.position.y = pose.position.y;
  goal.target_pose.pose.position.z = pose.position.z;
  goal.target_pose.pose.orientation.w = 1; /*pose.orientation.w;
  goal.target_pose.pose.orientation.x = pose.orientation.x;
  goal.target_pose.pose.orientation.y = pose.orientation.y;
  goal.target_pose.pose.orientation.z = pose.orientation.z;
					   */
  ROS_DEBUG("[Robot] - move : %s try to move to (%f,%f) in map", robot_name.c_str(), pose.position.x, pose.position.y);
  on_action_ = true;
  move_base_client_->sendGoal(goal);
}

bool Robot::is_ready()
{ 
  if(on_action_)
    {
      actionlib::SimpleClientGoalState state = 
	move_base_client_->getState();
      if(state.isDone())
	{
	  on_action_ = false;
	  getRobotPose();
	  if(state == actionlib::SimpleClientGoalState::ABORTED)
	    { 
	      ROS_INFO("[Robot] - is_ready : %s stop with ABORTED", 
		       name_.c_str());
	    }
	  else if(state == actionlib::SimpleClientGoalState::REJECTED)
	    {
	      ROS_INFO("[Robot] - is_ready : %s stop with REJECTED", 
		       name_.c_str());
	    }
	  else if(state == actionlib::SimpleClientGoalState::LOST)
	    {
	      ROS_INFO("[Robot] - is_ready : %s Lost :S", name_.c_str());
	    }
	}
    }
  return !on_action_; 
}

bool Robot::getRobotPose() 
{
  tf::Stamped<tf::Pose> global_pose;
  global_pose.setIdentity();

  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = name_ + "/base_footprint";
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now(); // save time for checking tf delay later 
  //get the global pose of the robot
  try
    {
      ros::Time last_error = ros::Time::now();
      std::string tf_error;
      
      // we need to make sure that the transform between the robot base frame and the global frame is available
      while (ros::ok()
	     && !tf_.waitForTransform("/map", "/" + name_ 
				      + "/base_footprint", 
				      ros::Time(), 
				      ros::Duration(0.1), 
				      ros::Duration(0.01),
				      &tf_error))
	{
	  ros::spinOnce();
	  if (last_error + ros::Duration(5.0) < ros::Time::now())
	    {
	      ROS_WARN("[BuildMap] - Waiting on transform from %s to map to become available before running the map, tf error: %s",
		       name_.c_str(), tf_error.c_str());
	      last_error = ros::Time::now();
	    }
	}
      
      tf_.transformPose("/map", 
			robot_pose, global_pose);
      tf::poseStampedTFToMsg(global_pose, pose_);
      ROS_DEBUG("[Robot] - getRobotPose : new pose at (%f,%f,%f)", pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
    }
  catch (tf::LookupException& ex)
    {
      ROS_ERROR("No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
  catch (tf::ConnectivityException& ex)
    {
      ROS_ERROR("Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
  catch (tf::ExtrapolationException& ex)
    {
      ROS_ERROR("Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() > 0.2)
    {
      ROS_WARN_THROTTLE(1.0,
			"Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
     
			current_time.toSec(), global_pose.stamp_.toSec(), 0.2);
      return false;
    }
  
  return true;
}

void Robot::get_less_cost(const std::vector<geometry_msgs::Pose> &frontiers, 
			  double &cost_min,
			  geometry_msgs::Pose &frontier)
{
  std::vector<geometry_msgs::Pose>::const_iterator it;
  for(it = frontiers.begin(); it != frontiers.end(); it ++)
    {
      double cost = get_cost(*it);
      if(cost_min == -1 && cost > 0)
	{
	  cost_min = cost;
	  frontier = *it;
	}
      else if(cost > 0 && cost < cost_min)
	{
	  cost_min = cost;
	  frontier = *it;
	}
    }
}

double Robot::get_cost(const geometry_msgs::Pose &goal) const
{
  // TODO change to calc with map.
  /*
  unsigned int rx, ry, gx, gy;
  
  if(worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, 
		rx, ry))
    if(worldToMap(goal.position.x, goal.position.y, gx, gy))
      return (rx * gx) / 2 + (ry * gy) / 2;
  */
  double rx, ry, gx, gy;
  rx = pose_.pose.position.x;
  ry = pose_.pose.position.y;
  gx = goal.position.x;
  gy = goal.position.y;
  double ret = (rx + gx / 2 + ry + gy / 2);
  if(ret < 0)
    ret *= -1;
  return ret;
}


DecisionMaking::DecisionMaking(ros::NodeHandle &n, 
			       tf::TransformListener& tf)
  :
  nh(n),
  tf_(ros::Duration(10)),
  frontier_subscriber_(),
  robots_(),
  initialized_(false)
{
}

DecisionMaking::~DecisionMaking()
{
  for(int i = 0; i < robots_.size(); i++)
    delete robots_[i];
  robots_.clear();
  initialized_ = false;
}

void DecisionMaking::initialize()
{
  if(initialized_)
    {
      ROS_INFO("[DecisionMaking] - initialize : DecisionMaking initialize twice :S");
      return;
    }
  ros::NodeHandle private_nh("~/decision");
  ROS_INFO("[DecisionMaking] - initialize : start...");
  XmlRpc::XmlRpcValue robot_list;
  bool ret = private_nh.getParam("/decision/robots_name", robot_list);
  if(!ret && robot_list.size() < 1)
    {
      ROS_INFO("[DecisionMaking] - initialize : No robot found. Please add robot in yaml file.");
      return;
    }
  for (unsigned int i = 0; i < robot_list.size(); ++i) 
    {
      Robot *robot = new Robot(static_cast<std::string>(robot_list[i]));
      robots_.push_back(robot);
    }

  ROS_INFO("[DecisionMaking] - initialize : number of robot : %zu", robots_.size());

  std::vector<Robot *>::iterator it;
  for(it = robots_.begin(); it != robots_.end(); it ++)
    {
      (*it)->initialize();
    }
  frontier_subscriber_ = nh.subscribe("/frontiers", 1, 
				      &DecisionMaking::
				      frontiers_callback, 
				      this);

  initialized_ = true;
  ROS_INFO("[DecisionMaking] - initialize : Done");
}

void DecisionMaking::frontiers_callback(const geometry_msgs::PoseArray &frontiers)
{
  ROS_INFO("[DecisionMaking] - frontiers_callback : start with %lu frontiers to explore and %lu already explore", frontiers.poses.size(), frontiers_reach.size());
  // Get cost by frontier for each robot.
  // If on robot is one the road then its vector of cost is empty.
  std::vector<geometry_msgs::Pose> frontiers_poses(frontiers.poses.begin(), frontiers.poses.end());
  std::vector<Robot *> robots;
  std::vector<Robot *>::iterator it;
  std::vector<geometry_msgs::Pose>::iterator it_r;
  std::vector<geometry_msgs::Pose>::iterator it_p;
  bool erase = false;
  it_p = frontiers_poses.begin(); 
  while(it_p != frontiers_poses.end())
    {
      erase = false;
      it_r = frontiers_reach.begin();
      while( it_r != frontiers_reach.end())
	{
	  if(pose_equal(*it_r, *it_p))
	    {
	      frontiers_poses.erase(it_p);
	      erase = true;
	      it_r = frontiers_reach.end();
	    }
	  else
	    it_r ++;
	}
      if(!erase)
	it_p ++;
    }
  /*
  if(frontiers_reach.size() > 0 && frontiers_poses.size() > 0)
    {
      std::vector<geometry_msgs::Pose>::iterator it_re, it_pe, it_b;
      bool del_f_poses = true;
      if(frontiers_reach.size() > frontiers_poses.size())
	{
	  it_p = frontiers_poses.begin();
	  it_r = frontiers_reach.begin();
	  it_re = frontiers_reach.end();
	  it_pe = frontiers_poses.end();	  
	}
      else
	{
	  del_f_poses = false;
	  it_r = frontiers_poses.begin();
	  it_p = frontiers_reach.begin();
	  it_pe = frontiers_reach.end();
	  it_re = frontiers_poses.end();
	}
      it_b = it_r;
      while(it_p != it_pe)
	{
	  bool ndel = true;
	  while(it_r != it_re)
	    {
	      if(pose_equal(*it_r, *it_p))
		{
		  if(del_f_poses)
		    {
		      ndel = false;
		      it_p = frontiers_poses.erase(it_p);
		      it_r = frontiers_reach.end();
		    }
		  else
		    {
		      it_r = frontiers_poses.erase(it_r);
		      it_r = frontiers_poses.end();
		    }
		  
		}
	      else
		it_r ++;
	    }
	  it_r = it_b;
	  if(ndel)
	    it_p ++;
	}
    }
*/
  ROS_INFO("[DecisionMaking] - frontiers_callback : there is %lu frontiers to explore", frontiers_poses.size());
  for(it = robots_.begin(); it < robots_.end(); it++)
    {
      if((*it)->is_ready())
	{
	  robots.push_back(*it);
	}
    }
  // Find the less goal for all robots.

  while(robots.size() > 0 && frontiers_poses.size() > 0)
    {
      double cost_min = -1;
      geometry_msgs::Pose frontier;
      std::vector<Robot *>::iterator robot = robots.end();
      for(it = robots.begin(); it < robots.end(); it++)
	{
	  double cost_min_r = cost_min;
	  geometry_msgs::Pose frontier_r;
	  (*it)->get_less_cost(frontiers_poses, cost_min_r, frontier_r);
	  if((cost_min == -1 && cost_min_r > 0) || (cost_min > cost_min_r && cost_min_r > 0))
	    {
	      cost_min = cost_min_r;
	      frontier = frontier_r;
	      robot = it;
	    }
	}
      if(robot != robots.end())
	{
	  (*robot)->move(frontier);
	  frontiers_reach.push_back(frontier);
	  for(it_p = frontiers_poses.begin(); 
	      it_p != frontiers_poses.end(); it_p ++)
	    {
	      if(pose_equal(*it_p, frontier))
		{
		  it_p = frontiers_poses.erase(it_p);
		  if(it_p == frontiers_poses.end())
		    break;
		}
	    }
	  robots.erase(robot);
	}
      else
	{
	  ROS_INFO("Don t find a robot");
	  break;
	}
    }
}

bool DecisionMaking::pose_equal(const geometry_msgs::Pose &a, 
				const geometry_msgs::Pose &b) const
{
  return a.position.x == b.position.x 
    && a.position.y == b.position.y 
    && a.position.z == b.position.z 
    && a.orientation.x == b.orientation.x
    && a.orientation.y == b.orientation.y
    && a.orientation.z == b.orientation.z
    && a.orientation.w == b.orientation.w;
}
/*
void DecisionMaking::get_cost_by_frontiers(const geometry_msgs
					   ::PoseStamped &robot_pose, 
					   const geometry_msgs::PoseArray
					   &frontiers, 
					   std::vector<double> &cost_robot)
{
  for(int i = 0; i < frontiers.poses.size(); i++)
    {
      cost_robot.push_back(get_cost(robot_pose, frontiers.poses.at(i)));
    }
}

bool DecisionMaking::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
    return false;
}


bool DecisionMaking::is_ready_to_move(const int &robot)
{
  MoveBaseClient *action_client = moves_.at(robot);
  
  actionlib::SimpleClientGoalState state = action_client->getState();
  if(state.isDone())
    {
      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	  return true;
	}
      else if(state == actionlib::SimpleClientGoalState::ABORTED)
	{ 
	  ROS_INFO("stop with ABORTED");
	  return true;
	}
      else if(state == actionlib::SimpleClientGoalState::REJECTED)
	{
	  ROS_INFO("stop with REJECTED");
	  return true;
	}
      else if(state == actionlib::SimpleClientGoalState::LOST)
	{
	  ROS_INFO("Lost :S");
	  return true;
	}
    }
  return true;
}
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Frontier");
  ros::NodeHandle n;
  tf::TransformListener tf_(ros::Duration(10));
  DecisionMaking decision(n, tf_);
  decision.initialize();
  ros::spin();

  return EXIT_SUCCESS;
}
