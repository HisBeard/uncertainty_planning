#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <dynamic_reconfigure/server.h>
#include <map_engine/map_engine_Config.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <time.h>

#include <thread>

#include "tf/transform_datatypes.h"//转换函数头文件
#include "DBSCAN.h"

#define THREADS_NUM 16

using namespace grid_map;
using namespace std;

double SIGMA_X;
double SIGMA_Y;
double SIGMA_THETA;

/* List of dynamic parameters */

// Dynamic parameter server callback function
void dynamicParamCallback(map_engine::map_engine_Config& config, uint32_t level)
{
    SIGMA_X = config.sigma_x;
    SIGMA_Y = config.sigma_y;
    SIGMA_THETA = config.sigma_theta;

}

inline double nomal2(double x, double y, double miu1, double miu2, double sigma1, double sigma2, double r) {
    return 1.0/(sqrt(1-r*r)*(2*M_PI*sigma1*sigma2)) * exp((-1/(2*(1-r*r)))*((x-miu1)*(x-miu1)/(sigma1*sigma1)-2*r*(x-miu1)*(y-miu2)/(sigma1*sigma2)+(y-miu2)*(y-miu2)/(sigma2*sigma2)));
}

double get_wall_time() 
{ 
  struct timeval time ; 
  if (gettimeofday(&time,NULL)){ 
    return 0; 
  } 
  return (double)time.tv_sec + (double)time.tv_usec * .000001; 
}

class LocalCostmap
{
public:

    LocalCostmap(ros::NodeHandle& nodeHandle)
        : nodeHandle_(nodeHandle)
    {

        // subscribe
        sub_global_map_ = nodeHandle_.subscribe("/map", 1, &LocalCostmap::globalMapCallback,this);
        sub_odom_ = nodeHandle_.subscribe("/carla/ego_vehicle/odometry", 1, &LocalCostmap::odomCallback, this);
        sub_lidar_ = nodeHandle_.subscribe("/LiDARGridMap", 1, &LocalCostmap::lidarMapCallback, this);

        // advertiser
        pub_local_costmap_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("vehicle_map", 1, true);
        pub_for_planner_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("for_planner_map", 1, true);
        pub_global_costmap_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("global_map", 1, true);

        // visualization for Rviz
        visual_vehicle_map_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("test_map", 1, true);

        // global_map 
        global_map_.add("global_map");
        global_map_.setFrameId("map");
        global_map_.setGeometry(Length(301.2, 301.2), 0.2, Position(93.14,-205.96));
        ROS_INFO("Created global map with size %f x %f m (%i x %i cells).",
            global_map_.getLength().x(), global_map_.getLength().y(),
            global_map_.getSize()(0), global_map_.getSize()(1));

        // vehicle_map
        vehicle_map_.add("vehicle_map");
        vehicle_map_.add("uncertainty_map");
        vehicle_map_.add("lidar_grid_map");
        vehicle_map_.add("swell_map");
        vehicle_map_.add("ellipse_map");
        vehicle_map_.setFrameId("ego_vehicle");
        vehicle_map_.setGeometry(Length(30, 20), 0.2 , Position(15, 0));
        ROS_INFO("Created vehicle map with size %f x %f m (%i x %i cells).",
            vehicle_map_.getLength().x(), vehicle_map_.getLength().y(),
            vehicle_map_.getSize()(0), vehicle_map_.getSize()(1));

        // dynamic server
        f = boost::bind(&dynamicParamCallback, _1, _2);
        server.setCallback(f);

    }

    ~LocalCostmap()
    {

    }

    // get global map from topic
    void globalMapCallback(const nav_msgs::OccupancyGrid& message)
    {
        grid_map::GridMapRosConverter::fromOccupancyGrid(message, "global_map", this->global_map_);

        // visualGlobalMap();
    }

    void odomCallback(const nav_msgs::Odometry& message)
    {
        double Vx_og, Vy_og, Vtheta_og; // vehicle position and heading in the frame Og
        double Cx, Cy; // coodinates of the cell i of grid in the frame Ov
        double x_og, y_og; // position of each cell i in the global frame

        // double sigma_x = SIGMA_X; // x-axis uncertainty (meter)
        // double sigma_y = SIGMA_Y; // y-axis uncertainty (meter)
        // double sigma_theta = SIGMA_THETA; // heading uncertainty (rad)
        sigma_x = SIGMA_X; // x-axis uncertainty (meter)
        sigma_y = SIGMA_Y; // y-axis uncertainty (meter)
        sigma_theta = SIGMA_THETA; // heading uncertainty (rad)

        // transform Quaternion to roll, pitch, yaw
        tf::Quaternion quat;
        tf::quaternionMsgToTF(message.pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // get vehicle current pose
        Vx_og = message.pose.pose.position.x;
        Vy_og = message.pose.pose.position.y;
        Vtheta_og = yaw;
        if (Vtheta_og < 0) {
            Vtheta_og += 6.28318530718;
        }

        // temp variances for efficiency
        double sin_Vtheta_og = sin(Vtheta_og);
        double cos_Vtheta_og = cos(Vtheta_og);

        clock_t start,finish;
        double totaltime;
        start=clock();
        vector<double> times;
 
        ros::Time start_time = ros::Time::now();

        // get the initial probability of vehicle frame from priori map and LiDAR

        for (GridMapIterator it(vehicle_map_); !it.isPastEnd(); ++it)
        {
            Position position;
            vehicle_map_.getPosition(*it, position);
            Cx = position.x();
            Cy = position.y();

            x_og = (Cx * cos_Vtheta_og - Cy * sin_Vtheta_og) + Vx_og;
            y_og = (Cx * sin_Vtheta_og + Cy * cos_Vtheta_og) + Vy_og;
            
            // ROS_INFO("%f, %f, %f, %f, %f, %f", Cx, Cy, Cx * cos_Vtheta_og - Cy * sin_Vtheta_og, (Cx * sin_Vtheta_og + Cy * cos_Vtheta_og), x_og, y_og);
            vehicle_map_.at("vehicle_map", *it) = global_map_.atPosition("global_map", Position(x_og, y_og));
            if (vehicle_map_.at("lidar_grid_map", *it) > 95) {
                vehicle_map_.at("vehicle_map", *it) = vehicle_map_.at("lidar_grid_map", *it);
            }
        }
        // visualVehicleMap("vehicle_map");

        finish = clock();
        times.push_back((double)(finish-start)/CLOCKS_PER_SEC); // times[0]
        times.push_back(0.0); // times[1]
        times.push_back(0.0); // times[2]
        times.push_back(0.0); // times[3]


        // ROS_INFO("Initiate map time : %f s", ros::Time::now().toSec() - start_time.toSec());

        // propagate uncertainty
        
        std::thread threads[THREADS_NUM];
        // spawn threads:
        for (int i = 0; i < THREADS_NUM; ++i)
        {
            threads[i] = std::thread(&LocalCostmap::propagateUncertainty, this, i * (15000 / THREADS_NUM), sin_Vtheta_og, cos_Vtheta_og, Cx, Cy);
        }
        for (auto& th : threads) th.join();

        ros::Time end_time = ros::Time::now();
        ROS_INFO("cost time : %f s", end_time.toSec() - start_time.toSec());
        finish=clock();
        times[3] = (double)(finish-start)/CLOCKS_PER_SEC;
        // ROS_INFO("times[0] %f times[1] %f times[2] %f times[3] %f", times[0], times[1] / count1, times[2] / count1, times[3]);
        // ROS_INFO("times[0] %f times[3] %f", times[0], times[3]);

        // Publish vehicle map.

        vehicle_map_.setTimestamp(start_time.toNSec());
        nav_msgs::OccupancyGrid vehicle_map_message, for_planner_message;
        GridMapRosConverter::toOccupancyGrid(vehicle_map_ , "uncertainty_map", 0, 100, vehicle_map_message);
        for_planner_message = vehicle_map_message;
        for_planner_message.info.origin = message.pose.pose;
        pub_local_costmap_.publish(vehicle_map_message);
        pub_for_planner_.publish(for_planner_message);
        // ROS_INFO_THROTTLE(1.0, "vehicle grid map (timestamp %f) published.", vehicle_map_message.header.stamp.toSec());

    }

    void lidarMapCallback(const nav_msgs::OccupancyGrid& message) {
        grid_map::GridMapRosConverter::fromOccupancyGrid(message, "lidar_grid_map", vehicle_map_);
        Position position;
        Index index;
        // vector< pair<float, float> > positions;  
        for (grid_map::GridMapIterator iterator(vehicle_map_); !iterator.isPastEnd(); ++iterator)
        {
            if (vehicle_map_.at("lidar_grid_map", *iterator) > 90)
            {
                vehicle_map_.getPosition(*iterator, position);
                vehicle_map_.getIndex(position, index);
                // float dx, dy;
                // position.x() + 0.2 > 15 ? dx = position.x() : dx = position.x() + 0.2;
                // position.y() + 0.2 > 10 ? dy = position.y() : dy = position.y() + 0.2;
                // Position p1(dx, dy);
                // Position p2(dx, dy);
                // Position p3(dx, dy);
                // vehicle_map_.atPosition("lidar_grid_map", p1) = vehicle_map_.atPosition("lidar_grid_map", position);
                // vehicle_map_.atPosition("lidar_grid_map", p2) = vehicle_map_.atPosition("lidar_grid_map", position);
                // vehicle_map_.atPosition("lidar_grid_map", p3) = vehicle_map_.atPosition("lidar_grid_map", position);

                // ROS_INFO("1 %f %f", position.x(), position.y());
                Index submapStartIndex(index[0] - 1, index[1] - 1);
                Index submapBufferSize(3, 3);
                for (grid_map::SubmapIterator iterator1(vehicle_map_, submapStartIndex, submapBufferSize);
                    !iterator1.isPastEnd(); ++iterator1) {
                    vehicle_map_.at("swell_map", *iterator1) = vehicle_map_.at("lidar_grid_map", *iterator);
                    Position temp;
                    vehicle_map_.getPosition(*iterator1, temp);
                    // ROS_INFO("2 %f %f", temp.x(), temp.y());
                }
            }
        }

        for (grid_map::GridMapIterator iterator(vehicle_map_); !iterator.isPastEnd(); ++iterator)
        {
            if (vehicle_map_.at("swell_map", *iterator) > 90)
            {
                vehicle_map_.at("lidar_grid_map", *iterator) = vehicle_map_.at("swell_map", *iterator);
            }
        }
        vehicle_map_.clear("swell_map");


        // vector< pair<float, float> > area;
        // area = dbscan::DBSCAN(positions, 1.2, 4);

        // for (int i = 0; i < area.size(); i++)
        // {
        //     Position temp_position(area[i].first, area[i].second);
        //     vehicle_map_.atPosition("lidar_grid_map", temp_position) = 100;
        // }

        // visualVehicleMap("lidar_grid_map");
    }

    vector<double> getConfidenceEllipse(Eigen::Matrix2f cov, double chisquare_val=2.4477) {
        vector<double> params;

        Eigen::EigenSolver<Eigen::Matrix2f> es(cov);

        // Get the eigenvalues and eigenvectors
        Eigen::Matrix2f D = es.pseudoEigenvalueMatrix();
        Eigen::Matrix2f V = es.pseudoEigenvectors();

        // find the bigger eigen value
        int major_index, minor_index;

        if (D(0,0) > D(1,1)) {
            major_index = 0;
            minor_index = 1;
        } else {
            major_index = 1;
            minor_index = 0;
        }

        // Calculate the angle between the largest eigenvector and the x-axis
        double angle = atan2(V(major_index,1), V(major_index,0));
        if (angle < 0) {
            angle += 6.28318530718;
        }

        // Calculate the size of the minor and major axes
        double half_major_axis = chisquare_val * sqrt(D(major_index,major_index));
        double half_minor_axis = chisquare_val * sqrt(D(minor_index,minor_index));

        // cout << D << endl;
        // cout << V << endl;

        params.push_back(half_major_axis);
        params.push_back(half_minor_axis);
        params.push_back(angle);
        // if (sqrt(D(minor_index,minor_index)) == )
        // ROS_INFO("angle %f, major %f, minor %f", angle, sqrt(D(major_index,major_index)), sqrt(D(minor_index,minor_index)));
        return params;
    }

    void visualGlobalMap() {
        ros::Time time = ros::Time::now();
        global_map_.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid global_map_message;
        GridMapRosConverter::toOccupancyGrid(global_map_, "global_map", 0, 100, global_map_message);
        pub_global_costmap_.publish(global_map_message);
    }

    void visualVehicleMap(string layer) {
        nav_msgs::OccupancyGrid map_message;
        GridMapRosConverter::toOccupancyGrid(vehicle_map_, layer, 0, 100, map_message);
        visual_vehicle_map_.publish(map_message);
        vehicle_map_.clear(layer);
    }

    void propagateUncertainty(int index, double sin_Vtheta_og, double cos_Vtheta_og, double Cx, double Cy) {
        int count1 = 0;
        double numerator = 0;
        double denominator = 0;
        double sigma_x_i, sigma_y_i;
        double u, v, t, rou;
        vector<double> ellipse_params;
        GridMapIterator it(vehicle_map_);

        for (int i = 0; i < index; i++) {
            ++it;
        }

        for (; !it.isPastEnd(); ++it) {
            clock_t m_timer = clock();
            if (count1++ == (15000 / THREADS_NUM)) break;
            numerator = 0;
            denominator = 0;
            Position position;
            vehicle_map_.getPosition(*it, position);
            Cx = position.x();
            Cy = position.y();
            // ROS_INFO("Cx %f Cy %f", Cx, Cy);

            u = pow((-sin_Vtheta_og * Cx - cos_Vtheta_og * Cy), 2);
            v = pow((cos_Vtheta_og * Cx - sin_Vtheta_og * Cy), 2);
            t = sin_Vtheta_og * cos_Vtheta_og * (pow(Cx, 2) - pow(Cy, 2))
                + Cx * Cy * (pow(sin_Vtheta_og, 2) - pow(cos_Vtheta_og, 2));

            sigma_x_i = sqrt(pow(sigma_x, 2) + pow(sigma_theta, 2) * u);
            sigma_y_i = sqrt(pow(sigma_y, 2) + pow(sigma_theta, 2) * v);
            rou = pow(sigma_theta, 2) * t / (sigma_x_i * sigma_y_i);

            // finish = clock();
            // times[1] += (double)(finish-m_timer)/CLOCKS_PER_SEC;

            Eigen::Matrix2f cov_i;
            double a, b, c;
            a = pow(sigma_x_i, 2);
            b = rou * sigma_x_i * sigma_y_i;
            c = pow(sigma_y_i, 2);
            cov_i << a, b, b, c;
            // ROS_INFO("%f, %f, %f", a, b, c);

            ellipse_params = getConfidenceEllipse(cov_i);

            // finish = clock();
            // times[1] += (double)(finish-m_timer)/CLOCKS_PER_SEC;

            // ROS_INFO("u1 %f u2 %f o1 %f o2 %f rou %f", Cx, Cy, sigma_x_i, sigma_y_i, rou);
            int count = 0;
            double calcu = 0;
            for (grid_map::EllipseIterator iterator(vehicle_map_, position, Length(2*ellipse_params[0],2*ellipse_params[1]), ellipse_params[2]);
                !iterator.isPastEnd(); ++iterator) {
                double f_i;
                double Cxj, Cyj;

                Position position_j;
                vehicle_map_.getPosition(*iterator, position_j);
                Cxj = position_j.x();
                Cyj = position_j.y();

                // f_i = exp(-(pow((Cxj-Cx) / sigma_x_i, 2) - 2 * rou * ((Cxj-Cx) / sigma_x_i) * ((Cyj-Cy) / sigma_y_i) + pow((Cyj-Cy) / sigma_y_i, 2))
                //     / (2 * (1 - pow(rou, 2))))
                //     / (2 * M_PI * sigma_x_i * sigma_y_i * sqrt(1 - pow(rou, 2)));

                f_i = nomal2(Cxj, Cyj, Cx, Cy, sigma_x_i, sigma_y_i, rou);

                if (f_i == 0) {
                    // ROS_INFO("%lf", f_i);
                }
                
                numerator += f_i * vehicle_map_.atPosition("vehicle_map", position_j);
                denominator += f_i;
                count++;
                calcu += f_i;
                // vehicle_map_.at("ellipse_map", *iterator) = 0;
            }

            // finish=clock();
            // times[2] += (double)(finish-m_timer)/CLOCKS_PER_SEC;

            if (0 == count) {
                vehicle_map_.at("uncertainty_map", *it) = vehicle_map_.at("vehicle_map", *it);
                continue;
            }

            // visualVehicleMap("ellipse_map");

            // ROS_INFO("count %d", count);
            // ROS_INFO("count1 %d, %f", count1, calcu);
            // ROS_INFO("numerator %f, denominator %f, uncertainty %f", numerator, denominator, numerator / denominator);
            // if (denominator == 0) {
            //     vehicle_map_.at("uncertainty_map", *it) = vehicle_map_.atPosition("vehicle_map", position);
            // }
            vehicle_map_.at("uncertainty_map", *it) = numerator / denominator;
        }
    }


private:
    //! ROS nodehandle.
    ros::NodeHandle& nodeHandle_;

    // subscriber
    ros::Subscriber sub_global_map_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_lidar_;

    // publisher.
    ros::Publisher pub_local_costmap_;
    ros::Publisher pub_global_costmap_;
    ros::Publisher pub_for_planner_;

    // visualization for Rviz
    ros::Publisher visual_vehicle_map_;

    // grid map
    grid_map::GridMap global_map_;
    grid_map::GridMap vehicle_map_;

    // dynamic reconfigure
    dynamic_reconfigure::Server<map_engine::map_engine_Config> server;
    dynamic_reconfigure::Server<map_engine::map_engine_Config>::CallbackType f;

    // uncertainty
    double sigma_x;
    double sigma_y;
    double sigma_theta;

};


int main(int argc, char** argv)
{
    // Initialize node and subscriber/publisher.
    ros::init(argc, argv, "local_costmap");
    ros::NodeHandle nh("~");
    LocalCostmap localCostmap(nh);
    ros::spin();

    return 0;

}