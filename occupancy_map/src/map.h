#ifndef MAP_H
#define MAP_H
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/filter.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <map>
#include <ctime>
#include <string>
#include <sstream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <ctype.h>
#include <nav_msgs/Odometry.h>

using namespace std;

struct GridIndex{
    int x;
    int y;
};

struct WorldPos{
    double x;
    double y;
};

#endif
