# Necessary processes

## run CARLA

conda activate carla && cd CARLA && SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl

## run ros-bridge
conda deactivate &&
cd carla-ros-bridge/catkin_ws/ && source devel/setup.bash && roslaunch carla_ad_demo carla_ad_demo.launch

## run occupancy_map

cd catkin_ws/ &&
source devel/setup.bash &&
rosrun occupancy_map map_realtime

## run map_engine

cd catkin_ws/src/map_engine/maps &&
rosrun map_server map_server Town02.yaml

cd catkin_ws &&
source devel/setup.bash &&
rosrun map_engine local_costmap

cd catkin_ws/ &&
source devel/setup.bash &&
roslaunch map_engine map_engine.launch

## run frenet_optimal_planner

cd catkin_ws &&
source devel/setup.bash &&
roslaunch frenet_optimal_planner frenet_optimal_planner.launch

# complile all packages

catkin_make -DCMAKE_BUILD_TYPE=Release

## compile grid_map

catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="frenet_optimal_planner;map_engine;occupancy_map" -DCMAKE_BUILD_TYPE=Release

## compile occupancy_map-------------------------

catkin_make -DCATKIN_WHITELIST_PACKAGES="occupancy_map" -DCATKIN_BLACKLIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Debug

## compile map_engine---------------------------

cd catkin_ws/ &&
catkin_make -DCATKIN_WHITELIST_PACKAGES="map_engine" -DCATKIN_BLACKLIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Debug

## compile frenet_optimal_planner----------------

cd catkin_ws &&
catkin_make -DCATKIN_WHITELIST_PACKAGES="frenet_optimal_planner" -DCATKIN_BLACKLIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Debug

## scripts

put my_spawn_npc.py in /CARLA/PythonAPI/examples/my_spawn_npc.py
put carla_ad_demo in /carla-ros-bridge/ros-bridge/carla_ad_demo
