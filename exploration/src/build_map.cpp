#include "exploration/build_map.h"

BuildMap::BuildMap()
:
  nh_(),
  tf_(),
  map_publisher_(),
  maps_subscriber_(),
  maps_data_(),
  robots_(),
  map_(),
  count_(0),
  create_map_(false),
  initialized_(false)
{
}

BuildMap::~BuildMap()
{
  /*  if(maps_data_ != NULL)
    {
      delete maps_data_;
      maps_data_ = NULL;
    }
  */
  initialized_ = false;
}

void BuildMap::initialize()
{
  if(initialized_)
    {
      ROS_INFO("[BuildMap] - initialize : DecisionMaking initialize twice :S");
      return;
    }
  ros::NodeHandle private_nh("~/map_build");
  ROS_INFO("[DecisionMaking] - initialize : start...");
  XmlRpc::XmlRpcValue robot_list;
  bool ret = private_nh.getParam("/map_build/robots_name", robot_list);
  if(!ret && robot_list.size() < 1)
    {
      ROS_INFO("[BuildMap] - initialize : No robot found. Please add robot in yaml file.");
      return;
    }
  for (unsigned int i = 0; i < robot_list.size(); ++i) 
    {
      robots_.push_back(static_cast<std::string>(robot_list[i]));
    }

  ROS_INFO("[BuildMap] - initialize : number of robot : %zu", robots_.size());
  std::vector<std::string>::iterator it;
  for(it = robots_.begin(); it != robots_.end(); it ++)
    {
      ros::Time last_error = ros::Time::now();
      std::string tf_error;

      // we need to make sure that the transform between the robot base frame and the global frame is available
      while (ros::ok()
	     && !tf_.waitForTransform("/map", "/" + *it + "/base_footprint", 
				      ros::Time(), 
				      ros::Duration(0.1), 
				      ros::Duration(0.01),
				      &tf_error))
	{
	  ros::spinOnce();
	  if (last_error + ros::Duration(5.0) < ros::Time::now())
	    {
	      ROS_WARN("[BuildMap] - Waiting on transform from %s to map to become available before running the map, tf error: %s",
		       (*it).c_str(), tf_error.c_str());
	      last_error = ros::Time::now();
	    }
	}
      ROS_INFO("[BuildMap] - initialize : subscribe topic : /%s/map",
	       (*it).c_str());
      maps_subscriber_.push_back(
				 nh_.subscribe("/" + *it + "/map", 1, 
					       &BuildMap::map_callback, 
					       this));
    }
  map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1);
  //  maps_data_ = new nav_msgs::OccupancyGrid[robots_.size()];
  map_.data.clear();
  initialized_ = true;
}

void BuildMap::loop()
{
  if(initialized_ == false)
    {
      ROS_INFO("[BuildMap] - loop : need to be initialized before running the loop...");
      return;
    }
  ros::Rate loop_rate(10);
  while(ros::ok())
    {
      setup_map_data();
      // Get maps 
      while(ros::ok() && maps_data_.size() < robots_.size())
	ros::spinOnce();
      create_map_ = true;
      create_map();
 
      create_map_ = false;
      loop_rate.sleep();
    }
}

void BuildMap::setup_map_data()
{
  maps_data_.clear();
}

void BuildMap::map_callback(const nav_msgs::OccupancyGrid &map_data)
{

  std::string map_frame_id = map_data.header.frame_id;
  maps_data_.push_back(map_data);
  /*
  std::vector<std::string>::iterator itRobot;
  bool find = false;
  for(int i = 0; i < robots_.size(); i++)
    {
      if(map_frame_id.find(robots_.at(i)) < map_frame_id.length())
	{
	  find = true;
	  maps_data_[i] = map_data;
	  break;
	}
    }
  if(!find)
    {
      ROS_INFO("[BuildMap] - map_callback : find a map wich does not register : %s", map_frame_id.c_str());
      return;
    }
  */
}

void BuildMap::create_map()
{
  map_.header.frame_id = "/map";
  map_.header.stamp = ros::Time::now();
  map_.info.map_load_time = ros::Time::now();
  resolution_ = map_.info.resolution = maps_data_[0].info.resolution;
  map_.info.origin.position.z = maps_data_[0].info.origin.position.z;

  map_.info.origin.orientation.x = 
    maps_data_[0].info.origin.orientation.x;
  map_.info.origin.orientation.y = 
    maps_data_[0].info.origin.orientation.y;
  map_.info.origin.orientation.z = 
    maps_data_[0].info.origin.orientation.z;
  map_.info.origin.orientation.w = 
    maps_data_[0].info.origin.orientation.w;
  bool change = false;
  for(unsigned int i = 0; i < maps_data_.size(); i++)
    {
      if(map_.info.width < maps_data_[i].info.width)
	{
	  size_x_ = map_.info.width = maps_data_[i].info.width;
	  change = true;
	}
      if(map_.info.height < maps_data_[i].info.height)
	{
	  size_y_ = map_.info.height = maps_data_[i].info.height;
	  change = true;
	}
      if(maps_data_[i].info.origin.position.x < origin_x_)
	{
	  //project the new origin into the grid
	  /*
	  int cell_ox;
	  double new_origin_x = maps_data_[i].info.origin.position.x;
	  cell_ox = int((new_origin_x - origin_x_) / resolution_);
	  */
	  //compute the associated world coordinates for the origin cell
	  //because we want to keep things grid-aligned
	  //double new_grid_ox;
	  //	  new_grid_ox = origin_x_ + cell_ox * resolution_;

	  origin_x_ = map_.info.origin.position.x = 
	    maps_data_[i].info.origin.position.x;// new_grid_ox;
	  change = true;
	}
      if(maps_data_[i].info.origin.position.y < origin_y_)
	{
	  /*
	  int cell_oy;
	  double new_origin_y = maps_data_[i].info.origin.position.y;
	  cell_oy = int((new_origin_y - origin_y_) / resolution_);
	  double new_grid_oy;
	  new_grid_oy = origin_y_ + cell_oy * resolution_;
	  */
	  origin_y_ = map_.info.origin.position.y = 
	    maps_data_[i].info.origin.position.y; //new_grid_oy;
	  change = true;
	}
    }

  
  if(change)
    {
      ROS_INFO("[BuildMap] - create_map : new size : %ux%u and origin : %f,%f", map_.info.width, map_.info.height, origin_x_, origin_y_);
      map_.data.resize(map_.info.width*map_.info.height);
      fill(map_.data.begin(), map_.data.end(), -1);
    }
  if(count_ > 10)
    {
      fill(map_.data.begin(), map_.data.end(), -1);
      count_ = 0;
    }
  count_ ++;
  for(unsigned int i = 0; i < maps_data_.size(); i ++)
    {
      double origin_x = maps_data_[i].info.origin.position.x;
      double origin_y = maps_data_[i].info.origin.position.y;
      unsigned int x, y;
      if(worldToMap(origin_x, origin_y, x, y))
	{
	  for(int j = x; j < maps_data_[i].info.width + x; j ++)
	    {
	      for(int k = y; k < maps_data_[i].info.height + y; k++)
		{
		  unsigned int cell_world = k * size_x_ + j;
		  unsigned int cell_in_map = (k - y) * maps_data_[i].info.width + (j - x);
		  if(map_.data[cell_world] == -1)
		    {
		      map_.data[cell_world] = maps_data_[i].data[cell_in_map];
		    }
		  else if(map_.data[cell_world] > 0 
			   && 
			   maps_data_[i].data[cell_in_map] 
			   > map_.data[cell_world]
			    )
		    map_.data[cell_world] = maps_data_[i].data[cell_in_map];
		}
	    }
	}
    }
  map_publisher_.publish(map_);
}

void BuildMap::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool BuildMap::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;
  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_)
    return true;
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Frontier");
  BuildMap map;
  map.initialize();
  map.loop();

  return EXIT_SUCCESS;
}
