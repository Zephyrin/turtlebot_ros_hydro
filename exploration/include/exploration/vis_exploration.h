#ifndef EXPLORATION_TRANSFORM_VIS_H___
#define EXPLORATION_TRANSFORM_VIS_H___

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>


class ExplorationTransformVis
{
  public:
    ExplorationTransformVis(const std::string& topic_name)
      {
	ros::NodeHandle pnh("~");
	exploration_transform_pointcloud_pub_ = pnh.advertise<sensor_msgs::PointCloud>(topic_name, 2, false);
      }

    virtual ~ExplorationTransformVis()
      {}

    void publishVisOnDemand(const costmap_2d::Costmap2D& costmap, const unsigned int* exploration_array)
    {
      if (exploration_transform_pointcloud_pub_.getNumSubscribers() > 0){
        unsigned int size_x = costmap.getSizeInCellsX();
        unsigned int size_y = costmap.getSizeInCellsY();
        unsigned int size = size_x * size_y;

        unsigned int max = 0;

        for (size_t i = 0; i < size; ++i){
          if ((exploration_array[i] < INT_MAX) && (exploration_array[i] > max)){
            max = exploration_array[i];
          }
        }

        float max_f = static_cast<float>(max);

	sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = "/map";
        cloud.header.stamp = ros::Time::now();

        double x_world, y_world;

	geometry_msgs::Point32 point;

        for (size_t x = 0; x < size_x; ++x){
          for (size_t y = 0; y < size_y; ++y){

            unsigned int index = costmap.getIndex(x,y);

            if (exploration_array[index] < INT_MAX){

              costmap.mapToWorld(x,y, x_world, y_world);
              point.x = x_world;
              point.y = y_world;
              point.z = static_cast<float>(exploration_array[index])/max_f;

              cloud.points.push_back(point);
            }

          }
        }
        exploration_transform_pointcloud_pub_.publish(cloud);
      }
    }

  protected:

    ros::Publisher exploration_transform_pointcloud_pub_;
  };



#endif
