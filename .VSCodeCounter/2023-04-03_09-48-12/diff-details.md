# Diff Details

Date : 2023-04-03 09:48:12

Directory /home/nvidia/code/movebase_comment/teb_local_planner

Total : 133 files,  3863 codes, 2738 comments, 1497 blanks, all 8098 lines

[Summary](results.md) / [Details](details.md) / [Diff Summary](diff.md) / Diff Details

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [base_local_planner/CHANGELOG.rst](/base_local_planner/CHANGELOG.rst) | reStructuredText | -173 | 0 | -29 | -202 |
| [base_local_planner/blp_plugin.xml](/base_local_planner/blp_plugin.xml) | XML | -7 | 0 | -1 | -8 |
| [base_local_planner/cfg/BaseLocalPlanner.cfg](/base_local_planner/cfg/BaseLocalPlanner.cfg) | Properties | -33 | -3 | -22 | -58 |
| [base_local_planner/cfg/LocalPlannerLimits.cfg](/base_local_planner/cfg/LocalPlannerLimits.cfg) | Properties | 0 | -7 | -4 | -11 |
| [base_local_planner/include/base_local_planner/costmap_model.h](/base_local_planner/include/base_local_planner/costmap_model.h) | C++ | -19 | -75 | -9 | -103 |
| [base_local_planner/include/base_local_planner/footprint_helper.h](/base_local_planner/include/base_local_planner/footprint_helper.h) | C++ | -22 | -56 | -10 | -88 |
| [base_local_planner/include/base_local_planner/goal_functions.h](/base_local_planner/include/base_local_planner/goal_functions.h) | C++ | -41 | -100 | -13 | -154 |
| [base_local_planner/include/base_local_planner/latched_stop_rotate_controller.h](/base_local_planner/include/base_local_planner/latched_stop_rotate_controller.h) | C++ | -55 | -22 | -17 | -94 |
| [base_local_planner/include/base_local_planner/line_iterator.h](/base_local_planner/include/base_local_planner/line_iterator.h) | C++ | -96 | -29 | -19 | -144 |
| [base_local_planner/include/base_local_planner/local_planner_limits.h](/base_local_planner/include/base_local_planner/local_planner_limits.h) | C++ | -73 | -37 | -12 | -122 |
| [base_local_planner/include/base_local_planner/local_planner_util.h](/base_local_planner/include/base_local_planner/local_planner_util.h) | C++ | -37 | -44 | -31 | -112 |
| [base_local_planner/include/base_local_planner/map_cell.h](/base_local_planner/include/base_local_planner/map_cell.h) | C++ | -15 | -44 | -9 | -68 |
| [base_local_planner/include/base_local_planner/map_grid.h](/base_local_planner/include/base_local_planner/map_grid.h) | C++ | -53 | -118 | -30 | -201 |
| [base_local_planner/include/base_local_planner/map_grid_cost_function.h](/base_local_planner/include/base_local_planner/map_grid_cost_function.h) | C++ | -40 | -82 | -18 | -140 |
| [base_local_planner/include/base_local_planner/map_grid_cost_point.h](/base_local_planner/include/base_local_planner/map_grid_cost_point.h) | C++ | -24 | -33 | -4 | -61 |
| [base_local_planner/include/base_local_planner/map_grid_visualizer.h](/base_local_planner/include/base_local_planner/map_grid_visualizer.h) | C++ | -20 | -45 | -7 | -72 |
| [base_local_planner/include/base_local_planner/obstacle_cost_function.h](/base_local_planner/include/base_local_planner/obstacle_cost_function.h) | C++ | -34 | -43 | -13 | -90 |
| [base_local_planner/include/base_local_planner/odometry_helper_ros.h](/base_local_planner/include/base_local_planner/odometry_helper_ros.h) | C++ | -26 | -53 | -14 | -93 |
| [base_local_planner/include/base_local_planner/oscillation_cost_function.h](/base_local_planner/include/base_local_planner/oscillation_cost_function.h) | C++ | -25 | -47 | -18 | -90 |
| [base_local_planner/include/base_local_planner/planar_laser_scan.h](/base_local_planner/include/base_local_planner/planar_laser_scan.h) | C++ | -14 | -40 | -4 | -58 |
| [base_local_planner/include/base_local_planner/point_grid.h](/base_local_planner/include/base_local_planner/point_grid.h) | C++ | -106 | -189 | -32 | -327 |
| [base_local_planner/include/base_local_planner/prefer_forward_cost_function.h](/base_local_planner/include/base_local_planner/prefer_forward_cost_function.h) | C++ | -18 | -36 | -11 | -65 |
| [base_local_planner/include/base_local_planner/simple_scored_sampling_planner.h](/base_local_planner/include/base_local_planner/simple_scored_sampling_planner.h) | C++ | -22 | -79 | -18 | -119 |
| [base_local_planner/include/base_local_planner/simple_trajectory_generator.h](/base_local_planner/include/base_local_planner/simple_trajectory_generator.h) | C++ | -57 | -96 | -21 | -174 |
| [base_local_planner/include/base_local_planner/trajectory.h](/base_local_planner/include/base_local_planner/trajectory.h) | C++ | -24 | -80 | -15 | -119 |
| [base_local_planner/include/base_local_planner/trajectory_cost_function.h](/base_local_planner/include/base_local_planner/trajectory_cost_function.h) | C++ | -22 | -56 | -13 | -91 |
| [base_local_planner/include/base_local_planner/trajectory_inc.h](/base_local_planner/include/base_local_planner/trajectory_inc.h) | C++ | -10 | -33 | -5 | -48 |
| [base_local_planner/include/base_local_planner/trajectory_planner.h](/base_local_planner/include/base_local_planner/trajectory_planner.h) | C++ | -126 | -211 | -49 | -386 |
| [base_local_planner/include/base_local_planner/trajectory_planner_ros.h](/base_local_planner/include/base_local_planner/trajectory_planner_ros.h) | C++ | -83 | -114 | -35 | -232 |
| [base_local_planner/include/base_local_planner/trajectory_sample_generator.h](/base_local_planner/include/base_local_planner/trajectory_sample_generator.h) | C++ | -14 | -51 | -11 | -76 |
| [base_local_planner/include/base_local_planner/trajectory_search.h](/base_local_planner/include/base_local_planner/trajectory_search.h) | C++ | -13 | -48 | -11 | -72 |
| [base_local_planner/include/base_local_planner/twirling_cost_function.h](/base_local_planner/include/base_local_planner/twirling_cost_function.h) | C++ | -13 | -43 | -9 | -65 |
| [base_local_planner/include/base_local_planner/velocity_iterator.h](/base_local_planner/include/base_local_planner/velocity_iterator.h) | C++ | -48 | -42 | -10 | -100 |
| [base_local_planner/include/base_local_planner/voxel_grid_model.h](/base_local_planner/include/base_local_planner/voxel_grid_model.h) | C++ | -69 | -92 | -19 | -180 |
| [base_local_planner/include/base_local_planner/world_model.h](/base_local_planner/include/base_local_planner/world_model.h) | C++ | -40 | -63 | -12 | -115 |
| [base_local_planner/msg/Position2DInt.msg](/base_local_planner/msg/Position2DInt.msg) | ROS Message | -2 | 0 | 0 | -2 |
| [base_local_planner/out/linecount.json](/base_local_planner/out/linecount.json) | JSON | -425 | 0 | 0 | -425 |
| [base_local_planner/package.xml](/base_local_planner/package.xml) | XML | -43 | 0 | -8 | -51 |
| [base_local_planner/setup.py](/base_local_planner/setup.py) | Python | -7 | -1 | -3 | -11 |
| [base_local_planner/src/costmap_model.cpp](/base_local_planner/src/costmap_model.cpp) | C++ | -65 | -55 | -26 | -146 |
| [base_local_planner/src/footprint_helper.cpp](/base_local_planner/src/footprint_helper.cpp) | C++ | -169 | -50 | -30 | -249 |
| [base_local_planner/src/goal_functions.cpp](/base_local_planner/src/goal_functions.cpp) | C++ | -179 | -45 | -22 | -246 |
| [base_local_planner/src/latched_stop_rotate_controller.cpp](/base_local_planner/src/latched_stop_rotate_controller.cpp) | C++ | -194 | -40 | -46 | -280 |
| [base_local_planner/src/local_planner_limits/__init__.py](/base_local_planner/src/local_planner_limits/__init__.py) | Python | -19 | -12 | -11 | -42 |
| [base_local_planner/src/local_planner_util.cpp](/base_local_planner/src/local_planner_util.cpp) | C++ | -68 | -40 | -21 | -129 |
| [base_local_planner/src/map_cell.cpp](/base_local_planner/src/map_cell.cpp) | C++ | -15 | -33 | -5 | -53 |
| [base_local_planner/src/map_grid.cpp](/base_local_planner/src/map_grid.cpp) | C++ | -230 | -42 | -38 | -310 |
| [base_local_planner/src/map_grid_cost_function.cpp](/base_local_planner/src/map_grid_cost_function.cpp) | C++ | -77 | -41 | -14 | -132 |
| [base_local_planner/src/map_grid_visualizer.cpp](/base_local_planner/src/map_grid_visualizer.cpp) | C++ | -52 | -33 | -11 | -96 |
| [base_local_planner/src/obstacle_cost_function.cpp](/base_local_planner/src/obstacle_cost_function.cpp) | C++ | -86 | -44 | -23 | -153 |
| [base_local_planner/src/odometry_helper_ros.cpp](/base_local_planner/src/odometry_helper_ros.cpp) | C++ | -54 | -41 | -12 | -107 |
| [base_local_planner/src/oscillation_cost_function.cpp](/base_local_planner/src/oscillation_cost_function.cpp) | C++ | -114 | -44 | -21 | -179 |
| [base_local_planner/src/point_grid.cpp](/base_local_planner/src/point_grid.cpp) | C++ | -350 | -116 | -91 | -557 |
| [base_local_planner/src/point_grid_node.cpp](/base_local_planner/src/point_grid_node.cpp) | C++ | -129 | -52 | -41 | -222 |
| [base_local_planner/src/prefer_forward_cost_function.cpp](/base_local_planner/src/prefer_forward_cost_function.cpp) | C++ | -13 | -9 | -7 | -29 |
| [base_local_planner/src/simple_scored_sampling_planner.cpp](/base_local_planner/src/simple_scored_sampling_planner.cpp) | C++ | -95 | -47 | -12 | -154 |
| [base_local_planner/src/simple_trajectory_generator.cpp](/base_local_planner/src/simple_trajectory_generator.cpp) | C++ | -181 | -83 | -33 | -297 |
| [base_local_planner/src/trajectory.cpp](/base_local_planner/src/trajectory.cpp) | C++ | -39 | -33 | -9 | -81 |
| [base_local_planner/src/trajectory_planner.cpp](/base_local_planner/src/trajectory_planner.cpp) | C++ | -666 | -179 | -157 | -1,002 |
| [base_local_planner/src/trajectory_planner_ros.cpp](/base_local_planner/src/trajectory_planner_ros.cpp) | C++ | -402 | -105 | -111 | -618 |
| [base_local_planner/src/twirling_cost_function.cpp](/base_local_planner/src/twirling_cost_function.cpp) | C++ | -7 | -6 | -6 | -19 |
| [base_local_planner/src/voxel_grid_model.cpp](/base_local_planner/src/voxel_grid_model.cpp) | C++ | -207 | -59 | -49 | -315 |
| [base_local_planner/test/footprint_helper_test.cpp](/base_local_planner/test/footprint_helper_test.cpp) | C++ | -99 | -17 | -30 | -146 |
| [base_local_planner/test/gtest_main.cpp](/base_local_planner/test/gtest_main.cpp) | C++ | -7 | -6 | -6 | -19 |
| [base_local_planner/test/line_iterator_test.cpp](/base_local_planner/test/line_iterator_test.cpp) | C++ | -48 | -28 | -6 | -82 |
| [base_local_planner/test/map_grid_test.cpp](/base_local_planner/test/map_grid_test.cpp) | C++ | -163 | -11 | -33 | -207 |
| [base_local_planner/test/trajectory_generator_test.cpp](/base_local_planner/test/trajectory_generator_test.cpp) | C++ | -12 | -6 | -9 | -27 |
| [base_local_planner/test/utest.cpp](/base_local_planner/test/utest.cpp) | C++ | -137 | -48 | -31 | -216 |
| [base_local_planner/test/velocity_iterator_test.cpp](/base_local_planner/test/velocity_iterator_test.cpp) | C++ | -125 | -35 | -19 | -179 |
| [base_local_planner/test/wavefront_map_accessor.h](/base_local_planner/test/wavefront_map_accessor.h) | C++ | -30 | -11 | -10 | -51 |
| [teb_local_planner/CHANGELOG.rst](/teb_local_planner/CHANGELOG.rst) | reStructuredText | 349 | 0 | 42 | 391 |
| [teb_local_planner/README.md](/teb_local_planner/README.md) | Markdown | 36 | 0 | 21 | 57 |
| [teb_local_planner/cfg/TebLocalPlannerReconfigure.cfg](/teb_local_planner/cfg/TebLocalPlannerReconfigure.cfg) | Properties | 307 | 20 | 117 | 444 |
| [teb_local_planner/cfg/rviz_test_optim.rviz](/teb_local_planner/cfg/rviz_test_optim.rviz) | YAML | 183 | 0 | 1 | 184 |
| [teb_local_planner/cmake_modules/FindG2O.cmake](/teb_local_planner/cmake_modules/FindG2O.cmake) | CMake | 87 | 0 | 11 | 98 |
| [teb_local_planner/cmake_modules/FindSUITESPARSE.cmake](/teb_local_planner/cmake_modules/FindSUITESPARSE.cmake) | CMake | 111 | 0 | 23 | 134 |
| [teb_local_planner/include/teb_local_planner/distance_calculations.h](/teb_local_planner/include/teb_local_planner/distance_calculations.h) | C++ | 257 | 123 | 85 | 465 |
| [teb_local_planner/include/teb_local_planner/equivalence_relations.h](/teb_local_planner/include/teb_local_planner/equivalence_relations.h) | C++ | 17 | 71 | 16 | 104 |
| [teb_local_planner/include/teb_local_planner/g2o_types/base_teb_edges.h](/teb_local_planner/include/teb_local_planner/g2o_types/base_teb_edges.h) | C++ | 106 | 127 | 46 | 279 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_acceleration.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_acceleration.h) | C++ | 369 | 237 | 129 | 735 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_dynamic_obstacle.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_dynamic_obstacle.h) | C++ | 51 | 79 | 24 | 154 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h) | C++ | 92 | 103 | 36 | 231 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_obstacle.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_obstacle.h) | C++ | 122 | 125 | 49 | 296 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_prefer_rotdir.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_prefer_rotdir.h) | C++ | 33 | 64 | 19 | 116 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_shortest_path.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_shortest_path.h) | C++ | 23 | 54 | 13 | 90 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_time_optimal.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_time_optimal.h) | C++ | 37 | 62 | 18 | 117 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_velocity.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_velocity.h) | C++ | 138 | 93 | 53 | 284 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h) | C++ | 68 | 74 | 25 | 167 |
| [teb_local_planner/include/teb_local_planner/g2o_types/edge_via_point.h](/teb_local_planner/include/teb_local_planner/g2o_types/edge_via_point.h) | C++ | 35 | 68 | 18 | 121 |
| [teb_local_planner/include/teb_local_planner/g2o_types/penalties.h](/teb_local_planner/include/teb_local_planner/g2o_types/penalties.h) | C++ | 91 | 87 | 16 | 194 |
| [teb_local_planner/include/teb_local_planner/g2o_types/vertex_pose.h](/teb_local_planner/include/teb_local_planner/g2o_types/vertex_pose.h) | C++ | 67 | 135 | 28 | 230 |
| [teb_local_planner/include/teb_local_planner/g2o_types/vertex_timediff.h](/teb_local_planner/include/teb_local_planner/g2o_types/vertex_timediff.h) | C++ | 45 | 83 | 18 | 146 |
| [teb_local_planner/include/teb_local_planner/graph_search.h](/teb_local_planner/include/teb_local_planner/graph_search.h) | C++ | 74 | 106 | 36 | 216 |
| [teb_local_planner/include/teb_local_planner/h_signature.h](/teb_local_planner/include/teb_local_planner/h_signature.h) | C++ | 221 | 149 | 59 | 429 |
| [teb_local_planner/include/teb_local_planner/homotopy_class_planner.h](/teb_local_planner/include/teb_local_planner/homotopy_class_planner.h) | C++ | 120 | 380 | 94 | 594 |
| [teb_local_planner/include/teb_local_planner/homotopy_class_planner.hpp](/teb_local_planner/include/teb_local_planner/homotopy_class_planner.hpp) | C++ | 42 | 38 | 16 | 96 |
| [teb_local_planner/include/teb_local_planner/misc.h](/teb_local_planner/include/teb_local_planner/misc.h) | C++ | 47 | 87 | 19 | 153 |
| [teb_local_planner/include/teb_local_planner/obstacles.h](/teb_local_planner/include/teb_local_planner/obstacles.h) | C++ | 539 | 377 | 201 | 1,117 |
| [teb_local_planner/include/teb_local_planner/optimal_planner.h](/teb_local_planner/include/teb_local_planner/optimal_planner.h) | C++ | 117 | 526 | 101 | 744 |
| [teb_local_planner/include/teb_local_planner/planner_interface.h](/teb_local_planner/include/teb_local_planner/planner_interface.h) | C++ | 43 | 135 | 31 | 209 |
| [teb_local_planner/include/teb_local_planner/pose_se2.h](/teb_local_planner/include/teb_local_planner/pose_se2.h) | C++ | 149 | 202 | 56 | 407 |
| [teb_local_planner/include/teb_local_planner/recovery_behaviors.h](/teb_local_planner/include/teb_local_planner/recovery_behaviors.h) | C++ | 29 | 84 | 22 | 135 |
| [teb_local_planner/include/teb_local_planner/robot_footprint_model.h](/teb_local_planner/include/teb_local_planner/robot_footprint_model.h) | C++ | 341 | 326 | 112 | 779 |
| [teb_local_planner/include/teb_local_planner/teb_config.h](/teb_local_planner/include/teb_local_planner/teb_config.h) | C++ | 277 | 93 | 60 | 430 |
| [teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h](/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h) | C++ | 109 | 342 | 71 | 522 |
| [teb_local_planner/include/teb_local_planner/timed_elastic_band.h](/teb_local_planner/include/teb_local_planner/timed_elastic_band.h) | C++ | 125 | 436 | 99 | 660 |
| [teb_local_planner/include/teb_local_planner/timed_elastic_band.hpp](/teb_local_planner/include/teb_local_planner/timed_elastic_band.hpp) | C++ | 91 | 71 | 28 | 190 |
| [teb_local_planner/include/teb_local_planner/visualization.h](/teb_local_planner/include/teb_local_planner/visualization.h) | C++ | 60 | 176 | 46 | 282 |
| [teb_local_planner/include/teb_local_planner/visualization.hpp](/teb_local_planner/include/teb_local_planner/visualization.hpp) | C++ | 153 | 47 | 25 | 225 |
| [teb_local_planner/launch/test_optim_node.launch](/teb_local_planner/launch/test_optim_node.launch) | XML | 4 | 2 | 5 | 11 |
| [teb_local_planner/msg/FeedbackMsg.msg](/teb_local_planner/msg/FeedbackMsg.msg) | ROS Message | 4 | 5 | 7 | 16 |
| [teb_local_planner/msg/TrajectoryMsg.msg](/teb_local_planner/msg/TrajectoryMsg.msg) | ROS Message | 2 | 1 | 4 | 7 |
| [teb_local_planner/msg/TrajectoryPointMsg.msg](/teb_local_planner/msg/TrajectoryPointMsg.msg) | ROS Message | 4 | 9 | 9 | 22 |
| [teb_local_planner/package.xml](/teb_local_planner/package.xml) | XML | 44 | 0 | 13 | 57 |
| [teb_local_planner/scripts/cmd_vel_to_ackermann_drive.py](/teb_local_planner/scripts/cmd_vel_to_ackermann_drive.py) | Python | 39 | 4 | 23 | 66 |
| [teb_local_planner/scripts/export_to_mat.py](/teb_local_planner/scripts/export_to_mat.py) | Python | 61 | 21 | 31 | 113 |
| [teb_local_planner/scripts/export_to_svg.py](/teb_local_planner/scripts/export_to_svg.py) | Python | 124 | 74 | 47 | 245 |
| [teb_local_planner/scripts/publish_dynamic_obstacle.py](/teb_local_planner/scripts/publish_dynamic_obstacle.py) | Python | 44 | 5 | 19 | 68 |
| [teb_local_planner/scripts/publish_test_obstacles.py](/teb_local_planner/scripts/publish_test_obstacles.py) | Python | 48 | 9 | 20 | 77 |
| [teb_local_planner/scripts/publish_viapoints.py](/teb_local_planner/scripts/publish_viapoints.py) | Python | 26 | 3 | 18 | 47 |
| [teb_local_planner/scripts/visualize_velocity_profile.py](/teb_local_planner/scripts/visualize_velocity_profile.py) | Python | 50 | 6 | 21 | 77 |
| [teb_local_planner/src/graph_search.cpp](/teb_local_planner/src/graph_search.cpp) | C++ | 196 | 89 | 58 | 343 |
| [teb_local_planner/src/homotopy_class_planner.cpp](/teb_local_planner/src/homotopy_class_planner.cpp) | C++ | 567 | 165 | 118 | 850 |
| [teb_local_planner/src/obstacles.cpp](/teb_local_planner/src/obstacles.cpp) | C++ | 130 | 50 | 35 | 215 |
| [teb_local_planner/src/optimal_planner.cpp](/teb_local_planner/src/optimal_planner.cpp) | C++ | 999 | 127 | 188 | 1,314 |
| [teb_local_planner/src/recovery_behaviors.cpp](/teb_local_planner/src/recovery_behaviors.cpp) | C++ | 58 | 41 | 20 | 119 |
| [teb_local_planner/src/teb_config.cpp](/teb_local_planner/src/teb_config.cpp) | C++ | 285 | 61 | 50 | 396 |
| [teb_local_planner/src/teb_local_planner_ros.cpp](/teb_local_planner/src/teb_local_planner_ros.cpp) | C++ | 895 | 155 | 183 | 1,233 |
| [teb_local_planner/src/test_optim_node.cpp](/teb_local_planner/src/test_optim_node.cpp) | C++ | 195 | 81 | 50 | 326 |
| [teb_local_planner/src/timed_elastic_band.cpp](/teb_local_planner/src/timed_elastic_band.cpp) | C++ | 464 | 64 | 107 | 635 |
| [teb_local_planner/src/visualization.cpp](/teb_local_planner/src/visualization.cpp) | C++ | 375 | 63 | 80 | 518 |
| [teb_local_planner/teb_local_planner_plugin.xml](/teb_local_planner/teb_local_planner_plugin.xml) | XML | 13 | 0 | 2 | 15 |
| [teb_local_planner/test/teb_basics.cpp](/teb_local_planner/test/teb_basics.cpp) | C++ | 56 | 6 | 11 | 73 |

[Summary](results.md) / [Details](details.md) / [Diff Summary](diff.md) / Diff Details