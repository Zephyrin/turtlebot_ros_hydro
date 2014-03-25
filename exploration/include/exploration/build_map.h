#ifndef BUILD_MAP_H_RF_
#define BUILD_MAP_H_RF_

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>

class BuildMap
{
 public:
  BuildMap();
  ~BuildMap();
  void initialize();
  void loop();
 protected:
  void map_callback(const nav_msgs::OccupancyGrid &map_data);
  void create_map();
  void setup_map_data();
  void mapToWorld(unsigned int mx, unsigned int my, 
		  double& wx, double& wy) const;
  bool worldToMap(double wx, double wy, 
		  unsigned int& mx, unsigned int& my) const;

 private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  ros::Publisher map_publisher_;
  std::vector<ros::Subscriber> maps_subscriber_;
  std::vector<nav_msgs::OccupancyGrid> maps_data_;
  std::vector<std::string> robots_;
  nav_msgs::OccupancyGrid map_;
  double origin_x_;
  double origin_y_;
  double resolution_;
  unsigned int size_x_;
  unsigned int size_y_;
  char count_;
  bool create_map_;
  bool initialized_;
};

#endif
