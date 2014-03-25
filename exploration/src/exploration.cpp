#include "exploration/exploration.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#define STRAIGHT_COST 100
#define DIAGONAL_COST 141

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Exploration::Exploration()
  :
  costmap_ros_(NULL),
  costmap_(NULL),
  map_data_(NULL),
  map_width_(0),
  map_height_(0),
  num_map_cells_(0)
{
}

void Exploration::initialized(const std::string &name, costmap_2d::Costmap2DROS &costmap_2d_ros,
                              dwa_local_planner::DWAPlannerROS &dwa_planner_ros,
                               ros::NodeHandle &nh, tf::TransformListener &tf)
{
  ROS_INFO("[Exploration] Initialise");
  this->name = name;
  costmap_ros_ = &costmap_2d_ros;
  dwa_planner_ros_ = &dwa_planner_ros;
  this->setupMapData();
  
  ros::NodeHandle private_nh_("~/" + name);

  dyn_rec_server_.reset(new dynamic_reconfigure::Server<exploration::ExplorationPlannerConfig>());

  dyn_rec_server_->setCallback(boost::bind(&Exploration::dynRecParamCallback, this, _1, _2));


  visualization_pub_ = private_nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    /*
  path_service_client_ = nh.serviceClient<nav_msgs::GetPlan>("trajectory");

    */
  
  //  exploration_plan_service_server_ = nh.advertiseService("exploration_path", &Exploration::doExploration, this);

  //  exploration_plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/NavfnROS/plan",2);
  /*
  mapSub = nh.subscribe("map", 1, 
  			&Exploration::doExploration,
			this);
  */
  vis_.reset(new ExplorationTransformVis("exploration_transform"));

  obstacle_vis_.reset(new ExplorationTransformVis("obstacle_transform"));
  while(doExploration())
    {
    }
  ROS_INFO("[Exploration] Initialise done...");
  costmap_->saveMap("~/map.pgm");
}

void Exploration::dynRecParamCallback(exploration::ExplorationPlannerConfig &config, uint32_t level)
{
  p_plan_in_unknown_ = config.plan_in_unknown;
  p_use_inflated_obs_ = config.use_inflated_obstacles;
  p_goal_angle_penalty_ = config.goal_angle_penalty;
  p_alpha_ = config.security_constant;
  p_dist_for_goal_reached_ = config.dist_for_goal_reached;
  p_same_frontier_dist_ = config.same_frontier_distance;
  p_min_frontier_size_ = config.min_frontier_size;
  p_min_obstacle_dist_ = config.min_obstacle_dist * STRAIGHT_COST;
  p_obstacle_cutoff_dist_ = config.obstacle_cutoff_distance;
}


bool Exploration::doExploration(/*const nav_msgs::OccupancyGrid &OccupancyGrid*/)
{
  ROS_INFO("[Exploration] doExploration");

  tf::Stamped<tf::Pose> robot_pose_tf;
  costmap_ros_->getRobotPose(robot_pose_tf);

  setupMapData();
  resetMaps();
  clearFrontiers();

  geometry_msgs::PoseStamped start;
  tf::poseStampedTFToMsg(robot_pose_tf, start);

  std::vector<geometry_msgs::PoseStamped> plan;
  std::vector<geometry_msgs::PoseStamped> goals;

  // create obstacle tranform
  buildobstacle_trans_array_(p_use_inflated_obs_);
  
  // search for frontiers
  if(findFrontiers(goals))
    {
      
      ROS_INFO("[Exploration] doExploration: found %u frontiers!", (unsigned int)goals.size());
    }
  else 
    {
      ROS_INFO("[Exploration] doExploration: no frontiers have been found!");
      //      delete mapSub;
      return false;
      //return doInnerExploration(start,plan);
    }

  if(!buildexploration_trans_array_(start, goals, false))
    {
      ROS_INFO("[Exploration] doExploration: no exploration have been found!");
      // delete mapSub;
      return false;
    }
  else
    {
      //      ROS_INFO("[Exploration] doExploration: exploration found %u!", (unsigned int) exploration_trans_array_.get());
    }

  if(!getTrajectory(start,goals,plan))
    {
      ROS_INFO("[Exploration] exploration: could not plan to frontier, starting inner-exploration");
      return false;
    }
  if(!plan.empty())
    {
      geometry_msgs::PoseStamped thisgoal = plan.back();
      unsigned int mx,my;
      costmap_->worldToMap(thisgoal.pose.position.x,thisgoal.pose.position.y,mx,my);
      
      previous_goal_ = costmap_->getIndex(mx,my);
      
      previous_goals.push_back(previous_goal_);
      int count = std::count(previous_goals.begin(), previous_goals.end(), previous_goal_);
      if(count > 10)
	{
	  ROS_INFO("[Exploration] - doExploration: try to reach same frontier too many time... ABORTING");
	  return false;
	}
      MoveBaseClient action_client("move_base", true);

      while(!action_client.waitForServer(ros::Duration(5.0)))
	{
	  ROS_INFO("Waiting for the move_base action server...");
	}
      move_base_msgs::MoveBaseGoal goal_line;
    
      goal_line.target_pose.header.frame_id = "/map";
      goal_line.target_pose.header.stamp = ros::Time::now();
      ROS_INFO("Start point %f %f goal %f %f", start.pose.position.x, start.pose.position.y, thisgoal.pose.position.x, thisgoal.pose.position.y);

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
      count = 0;
      do
	{
	  bool finished_before_timeout = 
	    action_client.waitForResult(ros::Duration(5.0));
	  state = action_client.getState();
	  //	  ROS_INFO("[Exploration] The turtlebot didn't finish its move... %s", state.toString().c_str());
	  ros::spinOnce();
	  count ++;
	}
      while(!state.isDone() && count < 100);
      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
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
      else 
	{
	  action_client.cancelAllGoals();
	  ROS_INFO("done count %d", count);
	  return true;
	}
      //        }
      //      else
      //        {
      //            ROS_INFO("[Exploration] Make plan did'nt work!");
      //        }
    }
  
  ROS_INFO("[Exploration] End - doExploration");  
  return true;
}

void Exploration::setupMapData()
{
  ROS_INFO("[Exploration - setupMapData] begin...");

  costmap_ = costmap_ros_->getCostmap();
  if ((this->map_width_ != costmap_->getSizeInCellsX()) 
      || (this->map_height_ != costmap_->getSizeInCellsY()))
  {
//      costmap_ros_->pause();
//      costmap_ros_->resume();
      map_width_ = costmap_->getSizeInCellsX();
      map_height_ = costmap_->getSizeInCellsY();
      num_map_cells_ = map_width_ * map_height_;
      ros::spinOnce();
      ROS_INFO("[Exploration] setupMapData : Resize costmap : %u ", num_map_cells_);
      // initialize exploration_trans_array_, obstacle_trans_array_, goalMap and frontier_map_array_
    
      exploration_trans_array_.reset(new unsigned int[num_map_cells_]);
      obstacle_trans_array_.reset(new unsigned int[num_map_cells_]);
      is_goal_array_.reset(new bool[num_map_cells_]);
      frontier_map_array_.reset(new int[num_map_cells_]);
    
      clearFrontiers();
      resetMaps();
    
  }
  map_data_ = costmap_->getCharMap();
  ROS_INFO("[Exploration - setupMapData] end.");
}

void Exploration::resetMaps()
{
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(obstacle_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);
}

void Exploration::clearFrontiers()
{
  std::fill_n(frontier_map_array_.get(), num_map_cells_, 0);
}

bool Exploration::getTrajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, std::vector<geometry_msgs::PoseStamped> &plan)
{
  ROS_DEBUG("[Exploration] getTrajectory");

  // setup start positions
  unsigned int mx,my;

  if(!costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mx,my))
    {
      ROS_WARN("[Exploration] The start coordinates are outside the costmap!");
      return false;
    }

  int currentPoint = costmap_->getIndex(mx,my);
  int nextPoint = currentPoint;

  geometry_msgs::PoseStamped trajPoint;
  std::string global_frame = costmap_ros_->getGlobalFrameID();
  trajPoint.header.frame_id = global_frame;
  while(!is_goal_array_[currentPoint])
    {
      int thisDelta;
      int adjacentPoints[8];
      getAdjacentPoints(currentPoint,adjacentPoints);

      int maxDelta = 0;

      for(int i = 0; i < 8; ++i)
	{
	  if(isFree(adjacentPoints[i]))
	    {
	      thisDelta = exploration_trans_array_[currentPoint] - exploration_trans_array_[adjacentPoints[i]];
	      if(thisDelta > maxDelta)
		{
		  maxDelta = thisDelta;
		  nextPoint = adjacentPoints[i];
		}
	    }
	}

      // This happens when there is no valid exploration transform data at the start point for example
      if(maxDelta == 0)
	{
	  ROS_WARN("[Exploration] No path to the goal could be found by following gradient!");
	  return false;
	}


      // make trajectory point
      unsigned int sx,sy,gx,gy;
      costmap_->indexToCells((unsigned int)currentPoint,sx,sy);
      costmap_->indexToCells((unsigned int)nextPoint,gx,gy);
      double wx,wy;
      costmap_->mapToWorld(sx,sy,wx,wy);
      
      trajPoint.pose.position.x = wx;
      trajPoint.pose.position.y = wy;
      trajPoint.pose.position.z = 0.0;

      // assign orientation
      int dx = gx-sx;
      int dy = gy-sy;
      double yaw_path = std::atan2(dy,dx);
      trajPoint.pose.orientation.x = 0.0;
      trajPoint.pose.orientation.y = 0.0;
      trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
      trajPoint.pose.orientation.w = cos(yaw_path*0.5f);
      
      plan.push_back(trajPoint);

      currentPoint = nextPoint;
      maxDelta = 0;
    }
  ROS_DEBUG("[Exploration] END: getTrajectory. Plansize %u", (unsigned int)plan.size());
  return !plan.empty();
}

bool Exploration::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers)
{
  std::vector<geometry_msgs::PoseStamped> empty_vec;
  return findFrontiers(frontiers,empty_vec);
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 */
bool Exploration::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers, std::vector<geometry_msgs::PoseStamped> &noFrontiers)
{
  // get latest costmap
  clearFrontiers();

  // list of all frontiers in the occupancy grid
  std::vector<int> allFrontiers;

  // check for all cells in the occupancy grid whether or not they are frontier cells
  for(unsigned int i = 0; i < num_map_cells_; ++i)
    {
      if(isFrontier(i))
	{
	  
	  allFrontiers.push_back(i);
	}
    }

  for(unsigned int i = 0; i < allFrontiers.size(); ++i)
    {
      if(!isFrontierReached(allFrontiers[i]))
	{
	  geometry_msgs::PoseStamped finalFrontier;
	  double wx,wy;
	  unsigned int mx,my;
	  costmap_->indexToCells(allFrontiers[i], mx, my);
	  costmap_->mapToWorld(mx,my,wx,wy);
	  std::string global_frame = costmap_ros_->getGlobalFrameID();
	  finalFrontier.header.frame_id = global_frame;
	  finalFrontier.pose.position.x = wx;
	  finalFrontier.pose.position.y = wy;
	  finalFrontier.pose.position.z = 0.0;
	  
	  double yaw = getYawToUnknown(costmap_->getIndex(mx,my));
	  
	  //if(frontier_is_valid)
	  // {

	  finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	  frontiers.push_back(finalFrontier);
	}
      //}
    }
  
  //  return true;

  // value of the next blob
  int nextBlobValue = 1;
  std::list<int> usedBlobs;

  for(unsigned int i = 0; i < allFrontiers.size(); ++i)
    {

      // get all adjacent blobs to the current frontier point
      int currentPoint = allFrontiers[i];
      int adjacentPoints[8];
      getAdjacentPoints(currentPoint,adjacentPoints);

      std::list<int> blobs;

      for(int j = 0; j < 8; j++)
	{
	  if(isValid(adjacentPoints[j]) && (frontier_map_array_[adjacentPoints[j]] > 0))
	    {
	      blobs.push_back(frontier_map_array_[adjacentPoints[j]]);
	    }
	}
      blobs.unique();

      if(blobs.empty())
	{
	  // create new blob
	  frontier_map_array_[currentPoint] = nextBlobValue;
	  usedBlobs.push_back(nextBlobValue);
	  nextBlobValue++;
	} 
      else 
	{
	  // merge all found blobs
	  int blobMergeVal = 0;

	  for(std::list<int>::iterator adjBlob = blobs.begin(); adjBlob != blobs.end(); ++adjBlob)
	    {
	      if(adjBlob == blobs.begin())
		{
		  blobMergeVal = *adjBlob;
		  frontier_map_array_[currentPoint] = blobMergeVal;
		} 
	      else 
		{
		  for(unsigned int k = 0; k < allFrontiers.size(); k++)
		    {
		      if(frontier_map_array_[allFrontiers[k]] == *adjBlob)
			{
			  usedBlobs.remove(*adjBlob);
			  frontier_map_array_[allFrontiers[k]] = blobMergeVal;
			}
		    }
		}
	    }
	}
    }

  int id = 1;
  
  bool visualization_requested = false; //(visualization_pub_.getNumSubscribers() > 0);
  
  // summarize every blob into a single point (maximum obstacle_trans_array_ value)
  for(std::list<int>::iterator currentBlob = usedBlobs.begin(); currentBlob != usedBlobs.end(); ++currentBlob)
    {
      int current_frontier_size = 0;
      int max_obs_idx = 0;
      
      for(unsigned int i = 0; i < allFrontiers.size(); ++i)
	{
	  int point = allFrontiers[i];
	  
	  if(frontier_map_array_[point] == *currentBlob)
	    {
	      current_frontier_size++;
	      if(obstacle_trans_array_[point] > obstacle_trans_array_[allFrontiers[max_obs_idx]])
		{
		  max_obs_idx = i;
		}
	    }
	}

      if(current_frontier_size < p_min_frontier_size_)
	{
	  continue;
	}
      
      int frontier_point = allFrontiers[max_obs_idx];
      unsigned int x,y;
      costmap_->indexToCells(frontier_point,x,y);

      // check if frontier is valid (not to close to robot and not in noFrontiers vector
      bool frontier_is_valid = true;

      if(isFrontierReached(frontier_point))
	{
	  frontier_is_valid = false;
	}

      for(size_t i = 0; i < noFrontiers.size(); ++i)
	{
	  const geometry_msgs::PoseStamped& noFrontier = noFrontiers[i];
	  unsigned int mx,my;
	  costmap_->worldToMap(noFrontier.pose.position.x,noFrontier.pose.position.y,mx,my);
	  int no_frontier_point = costmap_->getIndex(x,y);
	  if(isSameFrontier(frontier_point,no_frontier_point))
	    {
	      frontier_is_valid = false;
	    }
	}

      geometry_msgs::PoseStamped finalFrontier;
      double wx,wy;
      costmap_->mapToWorld(x,y,wx,wy);
      std::string global_frame = costmap_ros_->getGlobalFrameID();
      finalFrontier.header.frame_id = global_frame;
      finalFrontier.pose.position.x = wx;
      finalFrontier.pose.position.y = wy;
      finalFrontier.pose.position.z = 0.0;

      double yaw = getYawToUnknown(costmap_->getIndex(x,y));
      
      if(frontier_is_valid)
	{

	  finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	  frontiers.push_back(finalFrontier);
	}

      // visualization (export to method?)
    if(visualization_requested)
      {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "hector_exploration_planner";
	marker.id = id++;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = wx;
	marker.pose.position.y = wy;
	marker.pose.position.z = 0.0;
	marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0;
	
	if(frontier_is_valid)
	  {
	    marker.color.r = 0.0;
	    marker.color.g = 1.0;
	  }
	else
	  {
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	  }

	marker.color.b = 0.0;
	marker.lifetime = ros::Duration(5,0);
	//	visualization_pub_.publish(marker);
      }
    
    }
  return !frontiers.empty();
}


bool Exploration::isFrontier(const int &point)
{
  if(isFree(point))
    {
      int adjacentPoints[8];
      getAdjacentPoints(point, adjacentPoints);

      for(int i = 0; i < 8; ++i)
	{
	  if(isValid(adjacentPoints[i]))
	    {
	      if(map_data_[adjacentPoints[i]] 
		 == costmap_2d::NO_INFORMATION)
		{
		  int no_inf_count = 0;
		  int noInfPoints[8];
		  getAdjacentPoints(adjacentPoints[i], 
				    noInfPoints);
		  for(int j = 0; j < 8; j++)
		    {
		      if( isValid(noInfPoints[j]) && 
			  map_data_[noInfPoints[j]] 
			  == costmap_2d::NO_INFORMATION)
			{
			  ++no_inf_count;

			  if(no_inf_count > 2)
			    {
			      return true;
			    }
			}
		    }
		}
	    }
	}
    }
  return false;
}

bool Exploration::isSameFrontier(int frontier_point1, int frontier_point2)
{
  unsigned int fx1,fy1;
  unsigned int fx2,fy2;
  double wfx1,wfy1;
  double wfx2,wfy2;
  costmap_->indexToCells(frontier_point1,fx1,fy1);
  costmap_->indexToCells(frontier_point2,fx2,fy2);
  costmap_->mapToWorld(fx1,fy1,wfx1,wfy1);
  costmap_->mapToWorld(fx2,fy2,wfx2,wfy2);

  double dx = wfx1 - wfx2;
  double dy = wfy1 - wfy2;

  if((dx*dx) + (dy*dy) < (p_same_frontier_dist_*p_same_frontier_dist_))
    {
      return true;
    }
  return false;
}

bool Exploration::isFree(const int &point) const 
{
  if(isValid(point))
    {
      if(map_data_[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
	{
	  return true;
	}
      if(map_data_[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
	{
	  return true;
	}

      if(map_data_[point] == costmap_2d::NO_INFORMATION)
	{
	  return true;
	}
    }
  return false;
}

bool Exploration::isValid(const int &p) const
{
  return (p >= 0);
}

void Exploration::getAdjacentPoints(const int &point, int points[])
{
  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);
  points[4] = upleft(point);
  points[5] = upright(point);
  points[6] = downright(point);
  points[7] = downleft(point);
}

int Exploration::left(const int & point) const 
{
  // only go left if no index error and if current point is not already on the left boundary
  if((point % map_width_ != 0))
    {
      return point-1;
    }
  return -1;
}

int Exploration::right(const int & point) const
{
  if((point + 1) % map_width_ != 0)
    {
      return point+1;
    }
  return -1;
}

int Exploration::up(const int & point) const
{
  if(point >= (int)map_width_)
    {
      return point-map_width_;
    }
  return -1;
}

int Exploration::upleft(const int & point) const 
{
  if((point % map_width_ != 0) && (point >= (int)map_width_))
    {
      return point-1-map_width_;
    }
  return -1;
}

int Exploration::upright(const int & point) const
{
  if((point >= (int)map_width_) && ((point + 1) % (int)map_width_ != 0))
    {
      return point-map_width_+1;
    }
  return -1;
}

int Exploration::down(const int & point) const 
{
  if((point/map_width_) < (map_height_-1))
    {
      return point+map_width_;
    }
  return -1;
}

int Exploration::downleft(const int & point) const 
{
  if(((point/map_width_) < (map_height_-1)) && (point % map_width_ != 0))
    {
      return point+map_width_-1;
    }
  return -1;
}

int Exploration::downright(const int & point) const 
{
  if(((point + 1) % map_width_ != 0) && ((point/map_width_) < (map_height_-1)))
    {
      return point+map_width_+1;
    }
  return -1;
}

bool Exploration::buildexploration_trans_array_(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, bool useAnglePenalty){

  ROS_DEBUG("[Exploration] buildexploration_trans_array_");

  // reset exploration transform
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);

  std::queue<int> myqueue;

  size_t num_free_goals = 0;

  // initialize goals
  for(unsigned int i = 0; i < goals.size(); ++i)
    {
      // setup goal positions
      unsigned int mx,my;

      if(!costmap_->worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my))
	{
	  //ROS_WARN("[hector_exploration_planner] The goal coordinates are outside the costmap!");
	  continue;
	}

      int goal_point = costmap_->getIndex(mx,my);

      // Ignore free goal for the moment, check after iterating over all goals if there is not valid one at all
      if(!isFree(goal_point))
	{
	  continue;
	}
      else
	{
	  ++num_free_goals;
	}
      
      unsigned int init_cost = 0;
      if(useAnglePenalty)
	{
	  init_cost = angleDanger(angleDifference(start,goals[i])) * getDistanceWeight(start,goals[i]);
	}

      exploration_trans_array_[goal_point] = init_cost;
      
      // do not punish previous frontiers (oscillation)
      if(useAnglePenalty && isValid(previous_goal_))
	{
	  if(isSameFrontier(goal_point, previous_goal_))
	    {
	      ROS_DEBUG("[Exploration] same frontier: init with 0");
	      exploration_trans_array_[goal_point] = 0;
	    }
	}

      ROS_DEBUG("[Exploration] Goal init cost: %d, point: %d", exploration_trans_array_[goal_point], goal_point);
      is_goal_array_[goal_point] = true;
      myqueue.push(goal_point);
    }

  if (num_free_goals == 0)
    {
      ROS_WARN("[Exploration] All goal coordinates for exploration transform invalid (occupied or out of bounds), aborting.");
      return false;
    }
  else
    {
      ROS_INFO("num_free_goals : %zu", num_free_goals);
    }
  // exploration transform algorithm
  while(myqueue.size())
    {
      int point = myqueue.front();
      myqueue.pop();

      unsigned int minimum = exploration_trans_array_[point];

      int straightPoints[4];
      getStraightPoints(point,straightPoints);
      int diagonalPoints[4];
      getDiagonalPoints(point,diagonalPoints);

      // calculate the minimum exploration value of all adjacent cells
      for (int i = 0; i < 4; ++i) 
	{
	  if (isFree(straightPoints[i])) 
	    {
	      unsigned int neighbor_cost = minimum + STRAIGHT_COST + cellDanger(straightPoints[i]);

	      if (exploration_trans_array_[straightPoints[i]] > neighbor_cost) 
		{
		  exploration_trans_array_[straightPoints[i]] = neighbor_cost;
		  myqueue.push(straightPoints[i]);
		}
	    }

	  if (isFree(diagonalPoints[i])) 
	    {
	      unsigned int neighbor_cost = minimum + DIAGONAL_COST + cellDanger(diagonalPoints[i]);

	      if (exploration_trans_array_[diagonalPoints[i]] > neighbor_cost) 
		{
		  exploration_trans_array_[diagonalPoints[i]] = neighbor_cost;
		  myqueue.push(diagonalPoints[i]);
		}
	    }
	}
    }

  ROS_DEBUG("[Exploration] END: buildexploration_trans_array_");
  
  vis_->publishVisOnDemand(*costmap_, exploration_trans_array_.get());
  
  return true;
}


bool Exploration::buildobstacle_trans_array_(bool use_inflated_obstacles)
{
  ROS_DEBUG("[Exploration] buildobstacle_trans_array_");
  std::queue<int> myqueue;

  // init obstacles
  for(unsigned int i=0; i < num_map_cells_; ++i)
    {
      if(map_data_[i] == costmap_2d::LETHAL_OBSTACLE)
	{
	  myqueue.push(i);
	  obstacle_trans_array_[i] = 0;
	} 
      else if(use_inflated_obstacles)
	{
	  if(map_data_[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
	    {
	      myqueue.push(i);
	      obstacle_trans_array_[i] = 0;
	    }
	}
    }

  unsigned int obstacle_cutoff_value = static_cast<unsigned int>((p_obstacle_cutoff_dist_ / costmap_->getResolution()) * STRAIGHT_COST + 0.5);

  // obstacle transform algorithm
  while(myqueue.size())
    {
      int point = myqueue.front();
      myqueue.pop();

      unsigned int minimum = obstacle_trans_array_[point];
      if (minimum > obstacle_cutoff_value) continue;

      int straightPoints[4];
      getStraightPoints(point,straightPoints);
      int diagonalPoints[4];
      getDiagonalPoints(point,diagonalPoints);

      // check all 8 directions
      for(int i = 0; i < 4; ++i)
	{
	  if (isValid(straightPoints[i]) && (obstacle_trans_array_[straightPoints[i]] > minimum + STRAIGHT_COST)) 
	    {
	      obstacle_trans_array_[straightPoints[i]] = minimum + STRAIGHT_COST;
	      myqueue.push(straightPoints[i]);
	    }
	  if (isValid(diagonalPoints[i]) && (obstacle_trans_array_[diagonalPoints[i]] > minimum + DIAGONAL_COST)) 
	    {
	      obstacle_trans_array_[diagonalPoints[i]] = minimum + DIAGONAL_COST;
	      myqueue.push(diagonalPoints[i]);
	    }
	}
    }

  ROS_DEBUG("[Exploration] END: buildobstacle_trans_array_");
  
  obstacle_vis_->publishVisOnDemand(*costmap_, obstacle_trans_array_.get());
  
  return true;
}

inline void Exploration::getStraightPoints(int point, int points[])
{
  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);
}

inline void Exploration::getDiagonalPoints(int point, int points[])
{
  points[0] = upleft(point);
  points[1] = upright(point);
  points[2] = downright(point);
  points[3] = downleft(point);
}

bool Exploration::frontierDetection()
{

  return false;
}

float Exploration::angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{
  // setup start positions
  unsigned int mxs,mys;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);

  unsigned int gx,gy;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);

  int goal_proj_x = gx-mxs;
  int goal_proj_y = gy-mys;

  float start_angle = tf::getYaw(start.pose.orientation);
  float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

  float both_angle = 0;
  if(start_angle > goal_angle)
    {
      both_angle = start_angle - goal_angle;
    } 
  else 
    {
      both_angle = goal_angle - start_angle;
    }

  if(both_angle > M_PI)
    {
      both_angle = (M_PI - std::abs(start_angle)) + (M_PI - std::abs(goal_angle));
    }

  return both_angle;
}

unsigned int Exploration::angleDanger(float angle)
{
  float angle_fraction = std::pow(angle,3);///M_PI;
  unsigned int result = static_cast<unsigned int>(p_goal_angle_penalty_ * angle_fraction);
  return result;
}

float Exploration::getDistanceWeight(const geometry_msgs::PoseStamped &point1, const geometry_msgs::PoseStamped &point2)
{
  float distance = std::sqrt(std::pow(point1.pose.position.x - point2.pose.position.x,2) + std::pow(point1.pose.position.y - point2.pose.position.y,2));
  if(distance < 0.5)
    {
      return 5.0;
    } 
  return 1;
}

inline unsigned int Exploration::cellDanger(int point)
{
  if ((int)obstacle_trans_array_[point] <= p_min_obstacle_dist_)
    {
      return static_cast<unsigned int>(p_alpha_ * std::pow(p_min_obstacle_dist_ - obstacle_trans_array_[point], 2) + .5);
    }
  return 0;
}


double Exploration::getYawToUnknown(int point)
{
  int adjacentPoints[8];
  getAdjacentPoints(point,adjacentPoints);

  int max_obs_idx = 0;
  unsigned int max_obs_dist = 0;
  
  for(int i = 0; i < 8; ++i)
    {
      if(isValid(adjacentPoints[i])){
	if(map_data_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION)
	  {
	    if(obstacle_trans_array_[adjacentPoints[i]] > max_obs_dist)
	      {
		max_obs_idx = i;
		max_obs_dist = obstacle_trans_array_[adjacentPoints[i]];
	      }
	  }
      }
    }

  int orientationPoint = adjacentPoints[max_obs_idx];
  unsigned int sx,sy,gx,gy;
  costmap_->indexToCells((unsigned int)point,sx,sy);
  costmap_->indexToCells((unsigned int)orientationPoint,gx,gy);
  int x = gx-sx;
  int y = gy-sy;
  double yaw = std::atan2(y,x);

  return yaw;
}

bool Exploration::isFrontierReached(int point)
{
  tf::Stamped<tf::Pose> robotPose;
  if(!costmap_ros_->getRobotPose(robotPose))
    {
      ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
    }
  geometry_msgs::PoseStamped robotPoseMsg;
  tf::poseStampedTFToMsg(robotPose, robotPoseMsg);
  
  unsigned int fx,fy;
  double wfx,wfy;
  costmap_->indexToCells(point,fx,fy);
  costmap_->mapToWorld(fx,fy,wfx,wfy);
  
  
  double dx = robotPoseMsg.pose.position.x - wfx;
  double dy = robotPoseMsg.pose.position.y - wfy;

  if ( (dx*dx) + (dy*dy) < (p_dist_for_goal_reached_*p_dist_for_goal_reached_)) 
    {
      ROS_DEBUG("[Exploration]: frontier is within the squared range of: %f", p_dist_for_goal_reached_);
      return true;
    }
  return false;
}

void Exploration::spin()
{
    ros::Rate rate(10); // Specify the FSM loop rate in Hz
    while (ros::ok())
    { // Keep spinning loop until user presses Ctrl+C

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exploration");

  //  ROS_INFO("[Exploration] Initialise");
  ros::NodeHandle nh;
  
  tf::TransformListener tfl_(ros::Duration(10));
  
  costmap_2d::Costmap2DROS costmap_ros("global_costmap", tfl_);
  costmap_ros.start();
  std::cout << costmap_ros.getGlobalFrameID() << std::endl;
  std::cout << costmap_ros.getBaseFrameID() << std::endl;
  costmap_2d::Costmap2DROS local_costmap_ros("local_costmap", tfl_);
  local_costmap_ros.start();
  dwa_local_planner::DWAPlannerROS dwa_planner_ros;
  dwa_planner_ros.initialize("dwa_planner", &tfl_, &local_costmap_ros);
  Exploration walker;
  walker.initialized("exploration", costmap_ros, dwa_planner_ros, nh, tfl_);

  // Multithread...
  //ros::MultiThreadedSpinner spinner(2); // Use 4 threads
  //spinner.spin(); //
  // walker.spin();
  
  return EXIT_SUCCESS;
}
