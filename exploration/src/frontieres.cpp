#include "exploration/frontiers.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>

Frontier::Frontier(ros::NodeHandle &n)
  :
  nh(n),
  map_subscriber_(),
  frontiers_markers_publisher_(),
  frontiers_publisher_(),
  frontiers_(),
  frontiers_group_(),
  map_data_(),
  map_width_(0),
  map_height_(0),
  num_map_cells_(0),
  resolution_(0.0),
  p_same_frontier_dist_(2.5),
  initialized_(false)
{

}

Frontier::~Frontier()
{
}

void Frontier::initialize()
{
  if(initialized_)
    {
      ROS_INFO("[Frontier] - initialize : Frontier initialize twice :S");
      return;
    }
  map_subscriber_ = nh.subscribe("/map", 1, &Frontier::map_callback, this);
  // Display frontiers on rviz
  frontiers_markers_publisher_ = nh.advertise<visualization_msgs::Marker>
    ("frontiers_marker", 1);
  // Publish frontiers on decision node
  frontiers_publisher_ = nh.advertise<geometry_msgs::PoseArray >
    ("frontiers", 1);
  initialized_ = true;
  ROS_INFO("[Frontier] - initialize : Done");
}

void Frontier::map_callback(const nav_msgs::OccupancyGrid &map_data)
{
  ROS_INFO("[Frontier] - map_callback : start detection frontier's");
  tf::TransformListener listener;
  map_data_  = map_data;
  setup_map_data();

  frontier_detection();
  frontier_group();
  print_frontiers();
  publish_frontiers();
  ROS_INFO("[Frontier] - map_callback : Frontiers detected : %zu points, there are %zu group frontiers', with map size : %u cells", frontiers_.size(), frontiers_group_.size(), num_map_cells_);
}

void Frontier::frontier_group()
{
  std::vector<unsigned int>::iterator start, find;
  std::vector<unsigned int> frontiers(frontiers_);
  start = frontiers.begin();
  find = start + 1;
  std::vector<unsigned int> same_frontier;
  while(start != frontiers.end())
    {
      if(*start != -1)
	same_frontier.push_back(*start);
      while(find != frontiers.end())
	{
	  if(*find != -1)
	    {
	      if(is_same_frontier(*start, *find))
		{
		  same_frontier.push_back(*find);
		  *find = -1;
		}
	    }
	  find ++;
	}
      frontier_point(same_frontier);
      same_frontier.clear();
      start ++;
      find = start + 1;
    }
}

void Frontier::frontier_point(std::vector<unsigned int> &same_frontier)
{
  if(same_frontier.size() < 1)
    return;

  unsigned int x, y;
  std::vector<unsigned int>::iterator point;
  point = same_frontier.begin();
  index_to_cells(*point, x, y);
  point ++;
  while(point != same_frontier.end())
    {
      unsigned int px, py;
      index_to_cells(*point, px, py);
      x = x + px;
      y = y + py;
      point ++;
    }
  unsigned int avg = 0;
  x = x / same_frontier.size();
  y = y / same_frontier.size();
  ROS_INFO("Same frontier avg : %u,%u and width and height : %u,%u, and %u", x, y, map_width_, map_height_, same_frontier.size());
  cells_to_index(x, y, avg);
  frontiers_group_.push_back(avg);
}

bool Frontier::is_same_frontier(const unsigned int &frontier_point1, 
				   const unsigned int &frontier_point2)
{
  unsigned int fx1,fy1;
  unsigned int fx2,fy2;
  double wfx1,wfy1;
  double wfx2,wfy2;
  index_to_cells(frontier_point1, fx1, fy1);
  index_to_cells(frontier_point2,fx2,fy2);
  map_to_world(fx1,fy1,wfx1,wfy1);
  map_to_world(fx2,fy2,wfx2,wfy2);

  double dx = wfx1 - wfx2;
  double dy = wfy1 - wfy2;

  if((dx*dx) + (dy*dy) < (p_same_frontier_dist_*p_same_frontier_dist_))
    {
      return true;
    }
  return false;
}


void Frontier::print_frontiers()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "frontiers_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  for(std::vector<unsigned int>::iterator it = frontiers_.begin(); 
      it != frontiers_.end();
      it ++)
    {
      unsigned int x, y;
      index_to_cells(*it, x, y);
      geometry_msgs::Point p;
      map_to_world(x, y, p.x, p.y);
      
      p.z = 0;
      marker.points.push_back(p);
    }
  
  marker.scale.x = 0.07;
  marker.scale.y = 0.07;  
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(5,0);
  frontiers_markers_publisher_.publish(marker);
  marker.points.clear();
  marker.color.g = 1.0;
  marker.color.r = 0.0;
  marker.id = 1;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  for(std::vector<unsigned int>::iterator it = frontiers_group_.begin();
      it != frontiers_group_.end();
      it ++)
    {
      unsigned int x, y;
      index_to_cells(*it, x, y);
      geometry_msgs::Point p;
      map_to_world(x, y, p.x, p.y);
      p.z = 0;
      marker.points.push_back(p);
    }
  frontiers_markers_publisher_.publish(marker);
}

void Frontier::publish_frontiers()
{
  geometry_msgs::PoseArray final_points;
  final_points.header.stamp = ros::Time::now();
  final_points.header.frame_id = "/map";
  
  for(std::vector<unsigned int>::iterator it = frontiers_group_.begin();
      it != frontiers_group_.end();
      it ++)
    {
      unsigned int x, y;
      index_to_cells(*it, x, y);
      geometry_msgs::Pose p;
      map_to_world(x, y, p.position.x, p.position.y);
      p.position.z = 0;
      p.orientation.x = 0;
      p.orientation.y = 0;
      p.orientation.z = 0;
      p.orientation.w = 1;

      final_points.poses.push_back(p);
    }
  frontiers_publisher_.publish(final_points);
}

void Frontier::setup_map_data()
{
  map_width_ = map_data_.info.width;
  map_height_ = map_data_.info.height;
  resolution_ = map_data_.info.resolution;
  num_map_cells_ = map_width_ * map_height_;
  frontiers_.clear();
  frontiers_group_.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "frontiers_marker";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETE;
  frontiers_markers_publisher_.publish(marker);
  marker.id = 1;
  frontiers_markers_publisher_.publish(marker);
}

void Frontier::frontier_detection() 
{
  for(unsigned int i = 0; i < num_map_cells_; i++)
    {
      if(is_frontier(i))
	frontiers_.push_back(i);
    }
}

bool Frontier::is_frontier(const unsigned int &point)
{
  if(isFree(point))
    {
      int adjacentPoints[8];
      getAdjacentPoints(point, adjacentPoints);

      for(unsigned int i = 0; i < 8; ++i)
	{
	  if(isValid(adjacentPoints[i]))
	    {
	      if(map_data_.data[adjacentPoints[i]] 
		 == -1)
		{
		  int no_inf_count = 0;
		  int noInfPoints[8];
		  getAdjacentPoints(adjacentPoints[i], 
				    noInfPoints);
		  for(unsigned int j = 0; j < 8; j++)
		    {
		      if(isValid(noInfPoints[j]) && 
			 map_data_.data[noInfPoints[j]] == -1)
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

bool Frontier::isFree(const int &point) const 
{
  if(isValid(point))
    {
      if(map_data_.data[point] == 0)
	{
	  return true;
	}
    }
  return false;
}


void Frontier::getAdjacentPoints(const unsigned int &point, int points[])
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

bool Frontier::isValid(const int &p) const
{
  return (p >= 0);
}

int Frontier::left(const unsigned int & point) const 
{
  // only go left if no index error and if current point is not already on the left boundary
  if((point % map_width_ != 0))
    {
      return point-1;
    }
  return -1;
}

int Frontier::right(const unsigned int & point) const
{
  if((point + 1) % map_width_ != 0)
    {
      return point+1;
    }
  return -1;
}

int Frontier::up(const unsigned int & point) const
{
  if(point >= (int)map_width_)
    {
      return point-map_width_;
    }
  return -1;
}

int Frontier::upleft(const unsigned int & point) const 
{
  if((point % map_width_ != 0) && (point >= (int)map_width_))
    {
      return point-1-map_width_;
    }
  return -1;
}

int Frontier::upright(const unsigned int & point) const
{
  if((point >= (int)map_width_) && ((point + 1) % (int)map_width_ != 0))
    {
      return point-map_width_+1;
    }
  return -1;
}

int Frontier::down(const unsigned int & point) const 
{
  if((point/map_width_) < (map_height_-1))
    {
      return point+map_width_;
    }
  return -1;
}

int Frontier::downleft(const unsigned int & point) const 
{
  if(((point/map_width_) < (map_height_-1)) && (point % map_width_ != 0))
    {
      return point+map_width_-1;
    }
  return -1;
}

int Frontier::downright(const unsigned int & point) const 
{
  if(((point + 1) % map_width_ != 0) && ((point/map_width_) < (map_height_-1)))
    {
      return point+map_width_+1;
    }
  return -1;
}

void Frontier::index_to_cells(const unsigned int &index, 
			      unsigned int &x, unsigned int &y)
{
  y = index / map_width_;
  x = index - (y * map_width_);
}

void Frontier::cells_to_index(const unsigned int &x, 
			      const unsigned int &y,
			      unsigned int &index)
{
  index = y * map_width_ + x;
}

void Frontier::map_to_world(const unsigned int &x, const unsigned int &y,
			    double &fx, double &fy)
{
  fx = map_data_.info.origin.position.x + (x + 0.5) * resolution_;
  fy = map_data_.info.origin.position.y + (y + 0.5) * resolution_;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "Frontier");
  ros::NodeHandle n;
  Frontier frontier(n);
  frontier.initialize();
  ros::spin();
  return EXIT_SUCCESS;
}
