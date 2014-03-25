#ifndef FRONTIERS_H_RF_
#define FRONTIERS_H_RF_

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>

class Frontier
{
 public:
  Frontier(ros::NodeHandle &n);
  ~Frontier();
  void initialize();
 protected:
  void map_callback(const nav_msgs::OccupancyGrid &map_data);
  void setup_map_data();
  void frontier_detection();
  bool is_frontier(const unsigned int & point);
  bool is_same_frontier(const unsigned int &frontier_point1, 
			const unsigned int & frontier_point2);
  void frontier_group();
  void frontier_point(std::vector<unsigned int> &same_frontier);
  void print_frontiers();
  void publish_frontiers();
  void getAdjacentPoints(const unsigned int &point, int points[]);
  bool isValid(const int &p) const;
  bool isFree(const int &p) const;
  int left(const unsigned int & point) const;
  int up(const unsigned int & point) const;
  int right(const unsigned int & point) const;
  int upleft(const unsigned int & point) const;
  int upright(const unsigned int & point) const;
  int down(const unsigned int & point) const;
  int downleft(const unsigned int & point) const;
  int downright(const unsigned int & point) const;

  void index_to_cells(const unsigned int &index, 
		      unsigned int &x, unsigned int &y);
  void cells_to_index(const unsigned int &x, const unsigned int &y,
		      unsigned int &index);
  void map_to_world(const unsigned int &x, const unsigned int &y, 
		    double &fx, double &fy);
 private:
  ros::NodeHandle nh;
  ros::Subscriber map_subscriber_;
  ros::Publisher frontiers_markers_publisher_;
  ros::Publisher frontiers_publisher_;
  std::vector<unsigned int> frontiers_;
  std::vector<unsigned int> frontiers_group_;
  nav_msgs::OccupancyGrid map_data_;
  unsigned int map_width_;
  unsigned int map_height_;
  unsigned int num_map_cells_;
  float resolution_;
  double p_same_frontier_dist_;
  bool initialized_;
};

#endif
