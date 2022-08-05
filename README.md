# 1. run CARLA

conda activate carla && cd CARLA && SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl

# 2. run ros-bridge

conda deactivate &&
cd carla-ros-bridge/catkin_ws/ && source devel/setup.bash && roslaunch carla_ad_demo carla_ad_demo.launch

# 3. run map_engine

roslaunch map_engine map_engine.launch

# 4. run occupancy_map

rosrun occupancy_map map_realtime

# 5. run frenet_optimal_planner

roslaunch frenet_optimal_planner frenet_optimal_planner.launch

# compile

catkin_make -DCMAKE_BUILD_TYPE=Release