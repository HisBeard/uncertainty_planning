#include "map.h"
#include <ctime>
//initial parameters of map
double min_height = -1.3;
double max_height = 2;
double origin_x = 0.0; // equal to x_min
double origin_y = 0.0; //equal to y_min
double resolution = 0.2;
const int map_width = 150;  
const int map_height = 100;
double grid_origin_x = -(map_width * resolution)/2 + 15.0;
double grid_origin_y = -(map_height * resolution)/2;
double log_occ = 4; 
double log_free = -1;
std::vector<double> log_value(map_width * map_height,0);
Eigen::Vector3d RobotPose = {0,0,0}; //魔卡托坐标系下车辆的姿态

using namespace std;

// bresemham算法计算激光击中障碍物的路中空闲栅格有哪些
std::vector<GridIndex> Bresemham(int x0, int y0, int x1, int y1)
{
  vector<GridIndex> gridIndexVector;
  GridIndex tmpIndex;
    //
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX)
    {
      y += ystep;
      error -= deltaX;
    }

    if(pointX == x1 && pointY == y1) continue;

    tmpIndex.x = pointX;
    tmpIndex.y = pointY;

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
  
}

//雷达坐标系下坐标转世界坐标
WorldPos Lidar2World(const double x0,const double y0,Eigen::Vector3d RobotPose)
{
  WorldPos temp_world;
  Eigen::Matrix<double,2,1> world_frame = Eigen::Matrix<double,2,1>::Zero();
  Eigen::Matrix<double,2,1> lidar_frame = Eigen::Matrix<double,2,1>::Zero();
  Eigen::Matrix<double,2,2> rotation_matrix = Eigen::Matrix<double,2,2>::Zero();
  Eigen::Matrix<double,2,1> transformation_matrix = Eigen::Matrix<double,2,1>::Zero();
  lidar_frame << x0,y0;
  rotation_matrix <<  cos(RobotPose[2]),-sin(RobotPose[2]),sin(RobotPose[2]), cos(RobotPose[2]);
  transformation_matrix << RobotPose[0],RobotPose[1];
  world_frame = rotation_matrix * lidar_frame + transformation_matrix;
  temp_world.x = world_frame[0];
  temp_world.y = world_frame[1];
  return temp_world; 
}

//世界坐标转栅格坐标
GridIndex World2Grid(double x, double y)
{
  GridIndex temp;
  temp.x = std::ceil((x - grid_origin_x) / resolution);
  temp.y = std::ceil((y - grid_origin_y) / resolution);

  return temp;
}

int toLinear(GridIndex index)
{
 return (int)( index.y * map_width + index.x);
}

bool isInsideGrid(GridIndex index) {
  // ROS_INFO("x %d y %d lx %d ly %d", index.x, index.y, ((int)(grid_origin_x) + map_width / 2), ((int)(grid_origin_y) + map_height/2));
  if (index.x >= 150 || 
    index.x < 0) {
    return false;
  }
  if (index.y >= 90 || 
    index.y < -10) {
    return false;
  }
  return true;
}

// 更新每个栅格的log值
void UpdateLog(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,vector<double>& log_value)
{
  for(int i = 0 ; i != cloud -> points.size(); ++i){
      //此处需要对点云高度做个限制，过滤掉地面点云和偏离的点云
    if(isnan(cloud ->points[i].x) || isnan(cloud ->points[i].y) || isnan(cloud ->points[i].z)){
      continue;
    }
    if(cloud ->points[i].z < max_height && cloud ->points[i].z > min_height)
    {
     WorldPos ToWorld = Lidar2World(cloud->points[i].x, cloud->points[i].y, RobotPose);
     GridIndex ToGrid = World2Grid(ToWorld.x,ToWorld.y);
     if (!isInsideGrid(ToGrid)) continue;
     int linear = toLinear(ToGrid);
            // 增加linear判断条件，防止越界
     if (linear >= 0 && linear < log_value.size())
     {
      log_value[linear] += log_occ;
     }
    GridIndex current_grid = World2Grid(RobotPose[0],RobotPose[1]);
    vector<GridIndex> temp_vec = Bresemham(current_grid.x,current_grid.y,ToGrid.x,ToGrid.y);
    for(int j = 0 ; j != temp_vec.size(); ++j)
    {
      if (toLinear(temp_vec[j]) >= 0 && toLinear(temp_vec[j]) < log_value.size())
      {
        log_value[toLinear(temp_vec[j])] += log_free;
      }
    }
  }
}
    // std::cout << "log updated." << endl;
}

//计算概率
double calProbability(double log_value)
{
  return 1 - 1 /(1 + exp(log_value)); 
}

// 初始化地图
void mapInit(nav_msgs::OccupancyGrid& rosMap)
{
  rosMap.header.seq = 1;
  rosMap.header.frame_id="ego_vehicle";
  rosMap.info.resolution = resolution;
  rosMap.info.origin.position.x = origin_x;
  rosMap.info.origin.position.y = origin_y;
  rosMap.info.origin.position.z = 0.0;
  rosMap.info.origin.orientation.x = 0.0;
  rosMap.info.origin.orientation.y = 0.0;
  rosMap.info.origin.orientation.z = 0.0;
  rosMap.info.origin.orientation.w = 0.0;
  rosMap.info.width = map_width;
  rosMap.info.height = map_height;
  rosMap.data.resize(rosMap.info.width * rosMap.info.height);
  cout << "Map Initiated already."  << endl;
}

// 地图更新
void updateMap(nav_msgs::OccupancyGrid& rosMap)
{
  rosMap.header.seq ++;
  rosMap.header.stamp = ros::Time::now();
  rosMap.info.origin.position.x = origin_x;
  rosMap.info.origin.position.y = origin_y;
  rosMap.info.origin.position.z = 0.0;
  rosMap.info.origin.orientation.x = 0.0;
  rosMap.info.origin.orientation.y = 0.0;
  rosMap.info.origin.orientation.z = 0.0;
  rosMap.info.origin.orientation.w = 0.0;
  rosMap.info.height = map_height;
  rosMap.info.width = map_width;

  for(int i = 0 ; i != rosMap.info.width * rosMap.info.height; ++i)
  {
    if(log_value[i] != 0 )
    {
     double probablity = calProbability(log_value[i]);
     rosMap.data[i] = probablity * 100;
    }
   else
   {
      rosMap.data[i] = -1;     //未知栅格
    }
  }

    //更新记录文件log_value值
    // ofstream out("log_value.txt");
    // out.trunc;
    // for(int i = 0 ; i != map_height; ++i){
    //   for(int j = 0 ; j != map_width; ++j){
    //     if(log_value[i * map_height + j] == 0){
    //       out << -1 << " ";
    //     }
    //     else{
    //       out << setprecision(2) << calProbability(log_value[i * map_height + j]) << " ";
    //     }
    //   }
    //   out << std::endl;
    // }
    // out.close();

        log_value.clear();
        log_value.resize(map_width * map_height,0);
      }

      void downSample(pcl::PointCloud<pcl::PointXYZI>::Ptr raw, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
      {
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setLeafSize(0.3f, 0.3f, 0.3f);
        sor.setInputCloud(raw);
        sor.filter(*cloud);
  // cout << "Original cloud size: " << raw->points.size() << ". Filtering cloud size:"
  // << cloud-> points.size() << endl;
      }

//地面分割
      void segGround(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
      {
  	//pcl::PointCloud<pcl::PointXYZI>::Ptr outputby(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); //存储输出的模型的系数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //存储内点
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	//提取平面参数设置
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型，检测平面
	seg.setMethodType(pcl::SAC_RANSAC);      //设置方法【聚类或随机样本一致性】
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.1);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);    //分割操作
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
  extract.setNegative(true);
	extract.filter(*cloud);//提取对于索引的点云 内点
}

class occupancyGridMap{

public:
  occupancyGridMap()
  {
  // pub_map = nh.advertise<nav_msgs::OccupancyGrid>("occupancyGridMap",10);
  // sub_lidar = nh.subscribe("/hesai/pandar",10,&occupancyGridMap::callBack,this);
  // sub_geo = nh.subscribe("/odom",10,&occupancyGridMap::geometryCall,this);
    pub_map = nh.advertise<nav_msgs::OccupancyGrid>("LiDARGridMap",1);
    sub_lidar = nh.subscribe("/carla/ego_vehicle/lidar/lidar1/point_cloud",1,&occupancyGridMap::callBack,this);
    sub_geo = nh.subscribe("/carla/ego_vehicle/odometry",1,&occupancyGridMap::geometryCall,this);
    mapInit(gridMap);
  }

  void callBack(sensor_msgs::PointCloud2 cloud)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI max_pt,min_pt;
    pcl::fromROSMsg(cloud,*temp_cloud);
    segGround(temp_cloud);
    downSample(temp_cloud, currentCloud);
    pcl::getMinMax3D(*currentCloud,min_pt,max_pt);
    origin_x = grid_origin_x;
    origin_y = grid_origin_y;
    UpdateLog(currentCloud,log_value);
    updateMap(gridMap);
    pub_map.publish(gridMap);
  }

  void geometryCall(const nav_msgs::Odometry& message)
  {
  // // transform Quaternion to roll, pitch, yaw
  // tf::Quaternion quat;
  // tf::quaternionMsgToTF(message.pose.pose.orientation, quat);
  // double roll, pitch, yaw;  // 定义存储rpy的容器
  // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  // 进行转换

  // RobotPose[0] = message.pose.pose.position.x;
  // RobotPose[1] = message.pose.pose.position.y;
  // RobotPose[2] = yaw;
  }

private:
  ros::NodeHandle nh;
  ros::Publisher pub_map;
  ros::Subscriber sub_lidar;
  ros::Subscriber sub_geo;
  nav_msgs::OccupancyGrid gridMap;
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"map");
  occupancyGridMap test;
  cout << "Start!" << endl;
  ros::spin();
  return 0;
}

