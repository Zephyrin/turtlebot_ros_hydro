#ifndef EXPLORATION_H_RF_
#define EXPLORATION_H_RF_

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <visualization_msgs/Marker.h>
#include <string>

#include <exploration/ExplorationPlannerConfig.h>
#include <exploration/vis_exploration.h>
class Exploration
{
 public:
  Exploration();
  void spin();
  void initialized(const std::string &name, costmap_2d::Costmap2DROS &costmap_2d, dwa_local_planner::DWAPlannerROS &dwa_planner_ros, ros::NodeHandle &nh, tf::TransformListener &tf);

 protected:
  bool doExploration(/*const nav_msgs::OccupancyGrid &OccupancyGrid*/); //nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &resp);

  bool getTrajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, std::vector<geometry_msgs::PoseStamped> &plan);

  bool findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers, std::vector<geometry_msgs::PoseStamped> &noFrontiers);
  bool findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers);

  void print_frontiers(std::vector<geometry_msgs::PoseStamped> &plan);

  bool frontierDetection();

  bool isFrontier(const int &p);
  
  bool isSameFrontier(int frontier_point1, int frontier_point2);

  bool isFree(const int &p) const;

  bool isValid(const int &p) const;
  
  void getAdjacentPoints(const int &point, int points[]);

  bool buildexploration_trans_array_(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, bool useAnglePenalty);

  bool buildobstacle_trans_array_(bool use_inflated_obstacles);

  void setupMapData();
  void resetMaps();
  void clearFrontiers();
  void dynRecParamCallback(exploration::ExplorationPlannerConfig &config, uint32_t level);
  void getStraightPoints(int point, int points[]);
  void getDiagonalPoints(int point, int points[]);

  unsigned int cellDanger(int point);

  float getDistanceWeight(const geometry_msgs::PoseStamped &point1, const geometry_msgs::PoseStamped &point2);

  unsigned int angleDanger(float angle);


  float angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal);

  double getYawToUnknown(int point);

  bool isFrontierReached(int point);

  int left(const int & point) const;
  int up(const int & point) const;
  int right(const int & point) const;
  int upleft(const int & point) const;
  int upright(const int & point) const;
  int down(const int & point) const;
  int downleft(const int & point) const;
  int downright(const int & point) const;
  
 private:
  std::string name;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  
  dwa_local_planner::DWAPlannerROS *dwa_planner_ros_;

  boost::shared_array<unsigned int> obstacle_trans_array_;
  boost::shared_array<int> frontier_map_array_;
  boost::shared_array<unsigned int> exploration_trans_array_;
  boost::shared_array<bool> is_goal_array_;

  /*  ros::ServiceServer exploration_plan_service_server_;
  ros::Publisher exploration_plan_pub_;

  ros::ServiceClient path_service_client_;
  */

  ros::Publisher visualization_pub_;
  std::vector<visualization_msgs::Marker> plan_backup;

  const unsigned char* map_data_;

  int previous_goal_;
  std::vector<int> previous_goals;
  unsigned int map_width_;
  unsigned int map_height_;
  unsigned int num_map_cells_;

  boost::shared_ptr<dynamic_reconfigure::Server<exploration::ExplorationPlannerConfig> > dyn_rec_server_;

  boost::shared_ptr<ExplorationTransformVis> vis_;
  boost::shared_ptr<ExplorationTransformVis> obstacle_vis_;

  // Parameters
  bool p_plan_in_unknown_;
  bool p_use_inflated_obs_;
  int p_goal_angle_penalty_;
  int p_min_obstacle_dist_;
  int p_min_frontier_size_;
  double p_alpha_;
  double p_dist_for_goal_reached_;
  double p_same_frontier_dist_;
  double p_obstacle_cutoff_dist_;
};

#endif
