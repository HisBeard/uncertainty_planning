/** frenet_optimal_trajectory_planner.cc
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of Optimal trajectory planning in Frenet Coordinate Algorithm
 * Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
 */

#include "frenet_optimal_trajectory_planner.h"

// using namespace grid_map;

namespace fop
{

FrenetOptimalTrajectoryPlanner::TestResult::TestResult() : count(0), total_fix_cost(0.0), total_dyn_cost(0.0), total_dist(0.0)
{
  this->numbers = std::vector<int>(5, int(0));
  this->total_numbers = std::vector<int>(5, int(0));
  this->time = std::vector<double>(6, double(0));
  this->total_time = std::vector<double>(6, double(0));
  this->numbers.shrink_to_fit();
  this->total_numbers.shrink_to_fit();
  this->time.shrink_to_fit();
  this->total_time.shrink_to_fit();
}

void FrenetOptimalTrajectoryPlanner::TestResult::updateCount(const std::vector<int> numbers, const std::vector<std::chrono::_V2::system_clock::time_point> timestamps,
                                                             const double fix_cost, const double dyn_cost, const double dist)
{
  if (numbers.size() != 5 || timestamps.size() != 6)
  {
    std::cout << "Recorded TestResult for this planning iteration is invalid" << std::endl;
    return;
  }
  
  this->count++;

  // Update the numbers for the current iteration
  this->numbers = numbers;

  // Add the current numbers to total numbers
  std::transform(this->total_numbers.begin(), this->total_numbers.end(), numbers.begin(), this->total_numbers.begin(), std::plus<int>());
  
  // Calculate the elapsed_time for the current iteration, in [ms]
  for (int i = 0; i < timestamps.size() - 1; i++)
  {
    const std::chrono::duration<double, std::milli> elapsed_time = timestamps[i+1] - timestamps[i];
    this->time[i] = elapsed_time.count();
  }
  const std::chrono::duration<double, std::milli> elapsed_time = timestamps.back() - timestamps.front();
  this->time.back() = elapsed_time.count();

  // Add the current elapsed_time to total time, in [ms]
  std::transform(this->total_time.begin(), this->total_time.end(), this->time.begin(), this->total_time.begin(), std::plus<double>());

  // Add the current optimal cost & distance from previous best
  total_fix_cost += fix_cost;
  total_dyn_cost += dyn_cost;
  total_dist += dist;
  // cost_history.emplace_back(cost);
  // dist_history.emplace_back(dist);
}

void FrenetOptimalTrajectoryPlanner::TestResult::printSummary()
{
  const int count = std::max(1, int(this->count));
  // Print Summary for this iteration
  std::cout << " " << std::endl;
  std::cout << "Summary: This Planning Iteration (iteration no." << this->count << ")" << std::endl;
  std::cout << "Step 1 : Generated               " << this->numbers[0] << " Trajectories in " << this->time[0] << " ms" << std::endl;
  std::cout << "Step 2 : Converted               " << this->numbers[1] << " Trajectories in " << this->time[1] << " ms" << std::endl;
  std::cout << "Step 3 : Computed Cost for       " << this->numbers[2] << " Trajectories in " << this->time[2] << " ms" << std::endl;
  std::cout << "Step 4 : Checked Constraints for " << this->numbers[3] << " Trajectories in " << this->time[3] << " ms" << std::endl;
  std::cout << "Step 5 : Checked Collisions for  " << this->numbers[4] << " PolygonPairs in " << this->time[4] << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << this->time[5] << " ms (or " << 1000/this->time[5] << " Hz)" << std::endl;
  std::cout << "Dist   : Distance to History Best" << this->total_dist << std::endl;

  // Print Summary for average performance
  std::cout << " " << std::endl;
  std::cout << "Summary: Average Performance (" << this->count << " iterations so far)" << std::endl;
  std::cout << "Step 1 : Generated               " << this->total_numbers[0]/this->count << " Trajectories in " << this->total_time[0]/this->count << " ms" << std::endl;
  std::cout << "Step 2 : Converted               " << this->total_numbers[1]/this->count << " Trajectories in " << this->total_time[1]/this->count << " ms" << std::endl;
  std::cout << "Step 3 : Computed Cost for       " << this->total_numbers[2]/this->count << " Trajectories in " << this->total_time[2]/this->count << " ms" << std::endl;
  std::cout << "Step 4 : Checked Constraints for " << this->total_numbers[3]/this->count << " Trajectories in " << this->total_time[3]/this->count << " ms" << std::endl;
  std::cout << "Step 5 : Checked Collisions for  " << this->total_numbers[4]/this->count << " PolygonPairs in " << this->total_time[4]/this->count << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << this->total_time[5]/this->count << " ms (or " << 1000/(this->total_time[5]/this->count) << " Hz)" << std::endl;
  std::cout << "Cost   : Optimal's Fix Cost      " << this->total_fix_cost/count << std::endl;
  std::cout << "Cost   : Optimal's Dyn Cost      " << this->total_dyn_cost/count << std::endl;
  std::cout << "Cost   : Optimal's Total Cost    " << (this->total_fix_cost + this->total_dyn_cost)/count << std::endl;
  std::cout << "Dist   : Distance to History Best" << this->total_dist/count << std::endl;
}

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner()
{
  this->settings_ = Setting();
  this->test_result_ = TestResult();
}

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner(FrenetOptimalTrajectoryPlanner::Setting& settings)
{
  this->settings_ = settings;
  this->test_result_ = TestResult();
}

void FrenetOptimalTrajectoryPlanner::updateSettings(Setting& settings)
{
  this->settings_ = settings;
}

std::pair<Path, Spline2D> FrenetOptimalTrajectoryPlanner::generateReferenceCurve(const fop::Lane& lane)
{
  Path ref_path = Path();
  auto cubic_spline = fop::Spline2D(lane);

  std::vector<double> s;
  for (double i = 0; i < cubic_spline.s_.back(); i += 0.1)
  {
    s.emplace_back(i);
  }

  for (int i = 0; i < s.size(); i++)
  {
    fop::VehicleState state = cubic_spline.calculatePosition(s[i]);
    ref_path.x.emplace_back(state.x);
    ref_path.y.emplace_back(state.y);
    ref_path.yaw.emplace_back(cubic_spline.calculateYaw(s[i]));
  }

  return std::pair<Path, Spline2D>{ref_path, cubic_spline};
}

std::vector<fop::FrenetPath> 
FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, const int lane_id,
                                                      const double left_width, const double right_width, const double current_speed, 
                                                      const autoware_msgs::DetectedObjectArray& obstacles, const bool check_collision, const bool use_async)
{
  // Clear the canidate trajectories from the last planning cycle
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> empty;
  std::swap(candidate_trajs_, empty);
  
  // Initialize a series of results to be recorded
  std::vector<int> numbers;
  std::vector<std::chrono::_V2::system_clock::time_point> timestamps;
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  const auto obstacle_trajs = predictTrajectories(obstacles);

  // Sample a list of FrenetPaths
  all_trajs_ = std::make_shared<std::vector<fop::FrenetPath>>(generateFrenetPaths(frenet_state, lane_id, left_width, right_width, current_speed));
  numbers.push_back(all_trajs_->size());
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Convert to global paths
  const int num_conversion_checks = calculateGlobalPaths(*all_trajs_, cubic_spline);
  numbers.push_back(num_conversion_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Compute costs
  const int num_cost_checks = computeCosts(*all_trajs_, current_speed);
  numbers.push_back(num_cost_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  int num_iter = 0;
  int num_constraint_checks = 0;
  int num_collision_checks = 0;

  bool best_traj_found = false;
  while(!best_traj_found && !candidate_trajs_.empty())
  {
    best_traj_ = candidate_trajs_.top();
    candidate_trajs_.pop();

    // Check the constraints
    num_constraint_checks++;
    bool is_safe;
    if (!checkConstraints(best_traj_))
    {
      continue;
    }
    else
    {
      // Check for collisions
      if (check_collision)
      {
        is_safe = checkCollisions(best_traj_, obstacle_trajs, obstacles, use_async, num_collision_checks);
      }
      else
      {
        std::cout << "Collision Checking Skipped" << std::endl;
        is_safe = true;
      }
    }
    
    if (is_safe)
    {
      best_traj_found = true;
      std::cout << "FOP: Best Traj Found" << std::endl;
      break;
    }
  }

  double fix_cost = 0.0;
  double dyn_cost = 0.0;
  double dist = 0.0;
  if (best_traj_found && prev_best_traj_.collision_passed) // ensure the previous best exists
  {
    fix_cost = best_traj_.fix_cost;
    dyn_cost = best_traj_.dyn_cost;
    for (int i = 0; i < 3; i++)
    {
      const double l = best_traj_.idx(i) - prev_best_idx_(i);
      dist += std::pow(l, 2);
    }
  }

  numbers.push_back(num_constraint_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  numbers.push_back(num_collision_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  test_result_.updateCount(std::move(numbers), std::move(timestamps), fix_cost, dyn_cost, dist);
  test_result_.printSummary();

  if (best_traj_found)
  {
    prev_best_traj_ = best_traj_;
    prev_best_idx_ = best_traj_.idx;
    
    return std::vector<FrenetPath>{1, best_traj_};
  }
  else
  {
    return std::vector<FrenetPath>{};
  }
}

std::vector<fop::FrenetPath> 
FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, const int lane_id,
                                                      const double left_width, const double right_width, const double current_speed, 
                                                      const nav_msgs::OccupancyGrid& map, const double x_center, const double y_center, const bool check_collision, const bool use_async)
{
  // Clear the candidate trajectories from the last planning cycle
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> empty;
  std::swap(candidate_trajs_, empty);
  
  // Initialize a series of results to be recorded
  std::vector<int> numbers;
  std::vector<std::chrono::_V2::system_clock::time_point> timestamps;
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // transform Quaternion to roll, pitch, yaw
  tf::Quaternion quat;
  tf::quaternionMsgToTF(map.info.origin.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  if (yaw < 0) {
    yaw += 6.28318530718;
  }

  vehicle_map_.x_center = x_center;
  vehicle_map_.y_center = y_center;


  // map.info.origin.position.x = 0;
  // map.info.origin.position.y = 0;
  // map.info.origin.position.z = 0;
  // map.info.origin.orientation.x = 0;
  // map.info.origin.orientation.y = 0;
  // map.info.origin.orientation.z = 0;
  // map.info.origin.orientation.w = 0;


  // grid_map::GridMapRosConverter::fromOccupancyGrid(map, "uncertainty_map", vehicle_map_);
  // for (int i = 0; i < map.info.width * map.info.height; i++) {
  //   ROS_INFO("%d" , map.data[i]);
  // }
  // for (grid_map::GridMapIterator iterator(vehicle_map_); !iterator.isPastEnd(); ++iterator) {
  //   vehicle_map_.at("uncertainty_map", *iterator) = 1;
  //   ROS_INFO("%d map", vehicle_map_.at("uncertainty_map", *iterator));
  // }

  // store message data to vehicle_map_
  vehicle_map_.Vx_og = map.info.origin.position.x;
  vehicle_map_.Vy_og = map.info.origin.position.y;
  vehicle_map_.width = map.info.width;
  vehicle_map_.height = map.info.height;
  vehicle_map_.heading = yaw;
  for (std::vector<int8_t>::const_reverse_iterator iterator = map.data.rbegin();
    iterator != map.data.rend(); ++iterator)
  {
    size_t i = std::distance(map.data.rbegin(), iterator);
    vehicle_map_.data[(map.info.width-1)-i%map.info.width][i/map.info.width] = *iterator;
    // ROS_INFO("%d %d %d", map.info.width-1-i%map.info.width, i/map.info.width, *iterator);
    // ros::Duration(0.01).sleep();
  }

  // Sample a list of FrenetPaths
  all_trajs_ = std::make_shared<std::vector<fop::FrenetPath>>(generateFrenetPaths(frenet_state, lane_id, left_width, right_width, current_speed));
  numbers.push_back(all_trajs_->size());
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Convert to global paths
  const int num_conversion_checks = calculateGlobalPaths(*all_trajs_, cubic_spline);
  numbers.push_back(num_conversion_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Compute costs
  const int num_cost_checks = computeCosts(*all_trajs_, current_speed);
  numbers.push_back(num_cost_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  int num_iter = 0;
  int num_constraint_checks = 0;
  int num_collision_checks = 0;

  bool best_traj_found = false;
  while(!best_traj_found && !candidate_trajs_.empty())
  {
    best_traj_ = candidate_trajs_.top();
    candidate_trajs_.pop();

    // Check the constraints
    num_constraint_checks++;
    bool is_safe;
    if (!checkConstraints(best_traj_))
    {
      continue;
    }
    else
    {
      // Check for collisions
      if (false)
      {
        // is_safe = checkCollisions(best_traj_, obstacle_trajs, obstacles, use_async, num_collision_checks);
      }
      else
      {
        std::cout << "Collision Checking Skipped" << std::endl;
        is_safe = true;
      }
    }
    
    if (is_safe)
    {
      best_traj_found = true;
      std::cout << "FOP: Best Traj Found" << std::endl;
      break;
    }
  }

  double fix_cost = 0.0;
  double dyn_cost = 0.0;
  double dist = 0.0;
  if (best_traj_found && prev_best_traj_.collision_passed) // ensure the previous best exists
  {
    fix_cost = best_traj_.fix_cost;
    dyn_cost = best_traj_.dyn_cost;
    for (int i = 0; i < 3; i++)
    {
      const double l = best_traj_.idx(i) - prev_best_idx_(i);
      dist += std::pow(l, 2);
    }
  }

  numbers.push_back(num_constraint_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  numbers.push_back(num_collision_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  test_result_.updateCount(std::move(numbers), std::move(timestamps), fix_cost, dyn_cost, dist);
  test_result_.printSummary();

  if (best_traj_found)
  {
    prev_best_traj_ = best_traj_;
    prev_best_idx_ = best_traj_.idx;
    
    return std::vector<FrenetPath>{1, best_traj_};
  }
  else
  {
    return std::vector<FrenetPath>{};
  }
}

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::generateFrenetPaths(const fop::FrenetState& frenet_state, const int lane_id,
                                                                                 const double left_bound, const double right_bound, const double current_speed)
{
  // list of frenet paths generated
  std::vector<fop::FrenetPath> frenet_trajs;
  std::vector<double> goal_ds;

  int idx_i = 0;
  // generate different goals with a lateral offset
  const double delta_width = (left_bound - settings_.center_offset)/((settings_.num_width - 1)/2);
  for (double d = right_bound; d <= left_bound; d += delta_width)  // left being positive
  {
    int idx_k = 0;

    // calculate lateral offset cost
    const double lat_norm = std::max(std::pow(left_bound - settings_.center_offset, 2), std::pow(right_bound - settings_.center_offset, 2));
    const double lat_cost = settings_.k_diff*std::pow(d - settings_.center_offset, 2)/lat_norm;

    // generate d_t polynomials
    const double delta_t = (settings_.max_t - settings_.min_t)/(settings_.num_t - 1);
    for (double T = settings_.min_t; T <= settings_.max_t; T += delta_t)
    {
      int idx_j = 0;

      // calculate time cost (encourage longer planning horizon)
      const double time_cost = settings_.k_time*(1 - (T - settings_.min_t)/(settings_.max_t - settings_.min_t));

      fop::FrenetPath frenet_traj = fop::FrenetPath();
      frenet_traj.lane_id = lane_id;
      frenet_traj.o = -1;

      // start lateral state [d, d_d, d_dd]
      std::vector<double> start_d;
      start_d.emplace_back(frenet_state.d);
      start_d.emplace_back(frenet_state.d_d);
      start_d.emplace_back(frenet_state.d_dd);

      // end lateral state [d, d_d, d_dd]
      std::vector<double> end_d;
      end_d.emplace_back(d);
      end_d.emplace_back(0.0);
      end_d.emplace_back(0.0);

      // generate lateral quintic polynomial
      fop::QuinticPolynomial lateral_quintic_poly = fop::QuinticPolynomial(start_d, end_d, T);

      // store the this lateral trajectory into frenet_traj
      for (double t = 0.0; t <= T; t += settings_.tick_t)
      {
        frenet_traj.t.emplace_back(t);
        frenet_traj.d.emplace_back(lateral_quintic_poly.calculatePoint(t));
        frenet_traj.d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
        frenet_traj.d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
        frenet_traj.d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
      }

      // generate longitudinal quartic polynomial
      const double delta_speed = (settings_.highest_speed - settings_.lowest_speed)/(settings_.num_speed - 1);
      for (double sample_speed = settings_.lowest_speed; sample_speed <= settings_.highest_speed; sample_speed += delta_speed)
      {
        if (sample_speed <= 0)  // ensure target speed is positive
        {
          continue;
        }

        // copy the longitudinal path over
        fop::FrenetPath target_frenet_traj = frenet_traj;

        // start longitudinal state [s, s_d, s_dd]
        std::vector<double> start_s;
        start_s.emplace_back(frenet_state.s);
        start_s.emplace_back(frenet_state.s_d);
        start_s.emplace_back(0.0);

        // end longitudinal state [s_d, s_dd]
        std::vector<double> end_s;
        end_s.emplace_back(sample_speed);
        end_s.emplace_back(0.0);

        // generate longitudinal quartic polynomial
        fop::QuarticPolynomial longitudinal_quartic_poly = fop::QuarticPolynomial(start_s, end_s, T);

        // store the this longitudinal trajectory into target_frenet_traj
        for (double t = 0.0; t <= T; t += settings_.tick_t)
        {
          target_frenet_traj.s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
          target_frenet_traj.s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
          target_frenet_traj.s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
          target_frenet_traj.s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
        }

        // calculate speed cost
        const double speed_cost = settings_.k_diff*pow((settings_.highest_speed - sample_speed)/settings_.highest_speed, 2);
        // fixed cost terms
        target_frenet_traj.fix_cost = settings_.k_lat * lat_cost 
                                    + settings_.k_lon * (time_cost + speed_cost);

        target_frenet_traj.idx = Eigen::Vector3i(idx_i, idx_j, idx_k);
        frenet_trajs.emplace_back(target_frenet_traj);

        idx_j++;
      }

      idx_k++;
    }

    idx_i++;
  }

  return frenet_trajs;
}

int FrenetOptimalTrajectoryPlanner::calculateGlobalPaths(std::vector<fop::FrenetPath>& frenet_traj_list, fop::Spline2D& cubic_spline)
{
  double sin_Vtheta_og = sin(vehicle_map_.heading);
  double cos_Vtheta_og = cos(vehicle_map_.heading);

  int num_checks = 0;
  for (int i = 0; i < frenet_traj_list.size(); i++)
  {
    // calculate global positions
    for (int j = 0; j < frenet_traj_list[i].s.size(); j++)
    {
      fop::VehicleState state = cubic_spline.calculatePosition(frenet_traj_list[i].s[j]);
      double i_yaw = cubic_spline.calculateYaw(frenet_traj_list[i].s[j]);
      const double di = frenet_traj_list[i].d[j];
      const double frenet_x = state.x + di * cos(i_yaw + M_PI / 2.0);
      const double frenet_y = state.y + di * sin(i_yaw + M_PI / 2.0);
      // ROS_INFO("x %f y %f", frenet_x, frenet_y);
      if (!fop::isLegal(frenet_x) || !fop::isLegal(frenet_y))
      {
        frenet_traj_list[i].o = -1;
        break;
      }
      else
      {
        frenet_traj_list[i].x.emplace_back(frenet_x);
        frenet_traj_list[i].y.emplace_back(frenet_y);
        double dx, dy;  // difference of x,y between frenet and vechicle origin in global frame
        dx = frenet_x - vehicle_map_.Vx_og;
        dy = frenet_y - vehicle_map_.Vy_og;
        double Cx, Cy;  // coordination of x,y in vehicle frame
        Cx = dx * cos_Vtheta_og + dy * sin_Vtheta_og;
        Cy = dx * sin_Vtheta_og - dy * cos_Vtheta_og;
        // ROS_INFO("%f %f %f %f", dx, dy, Cx, Cy);
        // for (grid_map::GridMapIterator iterator(vehicle_map_); !iterator.isPastEnd(); ++iterator) {
        //   ROS_INFO("%d map", vehicle_map_.at("uncertainty_map", *iterator));
        // }
        int index_x, index_y; // index of planned points in vehicle frame
        index_x = (int)(Cx / vehicle_map_.resolution);
        index_y = vehicle_map_.height / 2 + (int)(Cy / vehicle_map_.resolution);
        if (Cx >= 0 && Cx <= vehicle_map_.width && Cy >= (vehicle_map_.y_center - vehicle_map_.height/2)
          && Cy <= (vehicle_map_.y_center + vehicle_map_.height/2))
        {
          for (int k = index_x; k < index_x; k++) {
            for (int l = index_y; l < index_y; l++) {
              if (k < 0 || k > vehicle_map_.width-1 || l < 0 || l > vehicle_map_.height-1)
              {
                continue;
              }
              if (vehicle_map_.data[k][l] > frenet_traj_list[i].o)
              {
                frenet_traj_list[i].o = vehicle_map_.data[k][l];
              }
            }
          }
          // ROS_INFO("%d %d %d",(int)(Cx * 5), 50+(int)(Cy * 5), frenet_traj_list[i].o);
          // if (vehicle_map_[index_x][index_y] > frenet_traj_list[i].o) {
          //   frenet_traj_list[i].o = vehicle_map_[index_x][index_y];
          //   // ROS_INFO("%d %d %d",(int)(Cx * 5), 50+(int)(Cy * 5), frenet_traj_list[i].o);
          // }
          // ROS_INFO("%d  dsasasd", vehicle_map_.atPosition("uncertainty_map", tempPos));
          // if (vehicle_map_.atPosition("uncertainty_map", tempPos) > frenet_traj_list[i].o)
          // {
          //   frenet_traj_list[i].o = vehicle_map_.atPosition("uncertainty_map", tempPos);
          //   ROS_INFO("%d   test", frenet_traj_list[i].o);
          // }
        }
      }
    }
    // ROS_INFO("%d ", frenet_traj_list[i].o);
    // calculate yaw and ds
    for (int j = 0; j < frenet_traj_list[i].x.size() - 1; j++)
    {
      const double dx = frenet_traj_list[i].x[j+1] - frenet_traj_list[i].x[j];
      const double dy = frenet_traj_list[i].y[j+1] - frenet_traj_list[i].y[j];
      frenet_traj_list[i].yaw.emplace_back(atan2(dy, dx));
      frenet_traj_list[i].ds.emplace_back(sqrt(dx * dx + dy * dy));
    }

    frenet_traj_list[i].yaw.emplace_back(frenet_traj_list[i].yaw.back());
    frenet_traj_list[i].ds.emplace_back(frenet_traj_list[i].ds.back());

    // calculate curvature
    for (int j = 0; j < frenet_traj_list[i].yaw.size() - 1; j++)
    {
      double yaw_diff = fop::unifyAngleRange(frenet_traj_list[i].yaw[j+1] - frenet_traj_list[i].yaw[j]);
      frenet_traj_list[i].c.emplace_back(yaw_diff / frenet_traj_list[i].ds[j]);
    }

    num_checks++;
  }

  return num_checks;
}

/**
 * @brief Checks whether frenet paths are safe to execute based on constraints 
 * @param traj the trajectory to be checked
 * @return true if trajectory satisfies constraints
 */
bool FrenetOptimalTrajectoryPlanner::checkConstraints(FrenetPath& traj)
{
  bool passed = true;
  for (int i = 0; i < traj.c.size(); i++)
  {
    if (!std::isnormal(traj.x[i]) || !std::isnormal(traj.y[i]))
    {
      passed = false;
      // std::cout << "Condition 0: Contains ilegal values" << std::endl;
      break;
    }
    else if (traj.s_d[i] > settings_.max_speed)
    {
      passed = false;
      // std::cout << "Condition 1: Exceeded Max Speed" << std::endl;
      break;
    }
    else if (traj.s_dd[i] > settings_.max_accel)
    {
      passed = false;
      // std::cout << "Condition 2: Exceeded Max Acceleration" << std::endl;
      break;
    }
    else if (traj.s_dd[i] < settings_.max_decel)
    {
      passed = false;
      // std::cout << "Condition 3: Exceeded Max Deceleration" << std::endl;
      break;
    }
    // else if (std::abs(traj.c[i]) > settings_.max_curvature)
    // {
    //   passed = false;
    //   std::cout << "Condition 4: Exceeded max curvature = " << settings_.max_curvature
    //             << ". Curr curvature = " << (traj.c[i]) << std::endl;
    //   break;
    // }
  }

  traj.constraint_passed = passed;
  return passed;
}


int FrenetOptimalTrajectoryPlanner::computeCosts(std::vector<fop::FrenetPath>& frenet_trajs, const double curr_speed)
{
  int num_checks = 0;
  for (auto& traj : frenet_trajs)
  {
    // calculate jerk costs
    double jerk_s, jerk_d = 0.0;
    double jerk_sqr_s, jerk_sqr_d = 0.0;
    // for (int i = 0; i < traj.t.size(); i++)
    // {
    //   // calculate total squared jerks
    //   jerk_sqr_s += std::pow(traj.s_ddd.back()/settings_.max_jerk_s, 2);
    //   jerk_s += std::abs(traj.s_ddd.back()/settings_.max_jerk_s);
    //   jerk_sqr_d += std::pow(traj.d_ddd.back()/settings_.max_jerk_d, 2);
    //   jerk_d += std::abs(traj.d_ddd.back()/settings_.max_jerk_d);
    // }

    jerk_sqr_s += std::pow(traj.s_ddd.back()/settings_.max_jerk_s, 2);
    jerk_s += std::abs(traj.s_ddd.back()/settings_.max_jerk_s);
    jerk_sqr_d += std::pow(traj.d_ddd.back()/settings_.max_jerk_d, 2);
    jerk_d += std::abs(traj.d_ddd.back()/settings_.max_jerk_d);

    const double jerk_cost_s = jerk_sqr_s/jerk_s;
    const double jerk_cost_d = jerk_sqr_d/jerk_d;
    const double occupancy_cost = traj.o/100;
    
    // ROS_INFO("%d ",traj.o);

    if (traj.o > 75)
    {
      traj.dyn_cost = 9;
    } else
    {
      traj.dyn_cost = settings_.k_occ * occupancy_cost + settings_.k_jerk * (settings_.k_lon * jerk_cost_s + settings_.k_lat * jerk_cost_d);
    }
    // traj.dyn_cost = settings_.k_jerk * (settings_.k_lon * jerk_cost_s + settings_.k_lat * jerk_cost_d);
    traj.final_cost = traj.fix_cost + traj.dyn_cost;

    num_checks++;

    candidate_trajs_.push(traj);
  }

  return num_checks;
}

bool FrenetOptimalTrajectoryPlanner::checkCollisions(FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs, 
                                                     const autoware_msgs::DetectedObjectArray& obstacles, 
                                                     const bool use_async, int& num_checks)
{
  if (use_async)
  {
    std::future<std::pair<bool, int>> collision_check = std::async(std::launch::async, &FrenetOptimalTrajectoryPlanner::checkTrajCollision, this, 
                                                                   ego_traj, obstacle_trajs, obstacles, settings_.safety_margin_lon, settings_.safety_margin_lat);
    const auto result = collision_check.get();
    ego_traj.collision_passed = result.first;
    num_checks += result.second;
  }
  else
  {
    const auto result = checkTrajCollision(ego_traj, obstacle_trajs, obstacles, settings_.safety_margin_lon, settings_.safety_margin_lat);
    ego_traj.collision_passed = result.first;
    num_checks += result.second;
  }

  return ego_traj.collision_passed;
}

/**
 * @brief Check for collisions at each point along a frenet path
 *
 * @param ego_traj the path to check
 * @param obstacles obstacles to check against for collision
 * @param margin collision margin in [m]
 * @return false if there is a collision along the path. Otherwise, true
 */
std::pair<bool, int> FrenetOptimalTrajectoryPlanner::checkTrajCollision(const FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs,
                                                                        const autoware_msgs::DetectedObjectArray& obstacles,
                                                                        const double margin_lon, const double margin_lat)
{
  int num_checks = 0;
  geometry_msgs::Polygon ego_rect, obstacle_rect;
  for (int i = 0; i < obstacles.objects.size(); i++)
  {
    const int num_steps = ego_traj.x.size();
    // Check for collisions between ego and obstacle trajectories
    for (int j = 0; j < num_steps; j++)
    {
      num_checks++;
      const double vehicle_center_x = ego_traj.x[j] + Vehicle::Lr() * cos(ego_traj.yaw[j]);
      const double vehicle_center_y = ego_traj.y[j] + Vehicle::Lr() * sin(ego_traj.yaw[j]);

      ego_rect = sat_collision_checker_.construct_rectangle(vehicle_center_x, vehicle_center_y, ego_traj.yaw[j], 
                                                            settings_.vehicle_length, settings_.vehicle_width, 0.0, 0.0);
      obstacle_rect = sat_collision_checker_.construct_rectangle(obstacle_trajs[i].x[j], obstacle_trajs[i].y[j], obstacle_trajs[i].yaw[j], 
                                                                 obstacles.objects[i].dimensions.x, obstacles.objects[i].dimensions.y, margin_lon, margin_lat);

      if (sat_collision_checker_.check_collision(ego_rect, obstacle_rect))
      {
        return std::pair<bool, int>{false, num_checks};
      }
    }
  }

  return std::pair<bool, int>{true, num_checks};
}

std::vector<Path> FrenetOptimalTrajectoryPlanner::predictTrajectories(const autoware_msgs::DetectedObjectArray& obstacles)
{
  std::vector<Path> obstacle_trajs;

  for (const auto& obstacle : obstacles.objects)
  {
    Path obstacle_traj;

    obstacle_traj.x.push_back(obstacle.pose.position.x);
    obstacle_traj.y.push_back(obstacle.pose.position.y);
    tf2::Quaternion q_tf2(obstacle.pose.orientation.x, obstacle.pose.orientation.y,
                          obstacle.pose.orientation.z, obstacle.pose.orientation.w);
    tf2::Matrix3x3 m(q_tf2.normalize());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    obstacle_traj.yaw.push_back(yaw);
    const double v = magnitude(obstacle.velocity.linear.x, obstacle.velocity.linear.y, obstacle.velocity.linear.z);
    obstacle_traj.v.push_back(v);
    
    const int steps = settings_.max_t/settings_.tick_t;
    for (int i = 0; i < steps; i++)
    {
      obstacle_traj.x.push_back(obstacle_traj.x.back() + v*settings_.tick_t*std::cos(yaw));
      obstacle_traj.x.push_back(obstacle_traj.y.back() + v*settings_.tick_t*std::sin(yaw));
      obstacle_traj.yaw.push_back(yaw);
      obstacle_traj.v.push_back(v);
    }

    obstacle_trajs.emplace_back(obstacle_traj);
  }

  return obstacle_trajs;
}

}  // namespace fop