# Details

Date : 2023-03-13 20:04:29

Directory /home/nvidia/code/movebase_comment/base_local_planner

Total : 70 files,  5981 codes, 3483 comments, 1506 blanks, all 10970 lines

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [base_local_planner/CHANGELOG.rst](/base_local_planner/CHANGELOG.rst) | reStructuredText | 173 | 0 | 29 | 202 |
| [base_local_planner/blp_plugin.xml](/base_local_planner/blp_plugin.xml) | XML | 7 | 0 | 1 | 8 |
| [base_local_planner/cfg/BaseLocalPlanner.cfg](/base_local_planner/cfg/BaseLocalPlanner.cfg) | Properties | 33 | 3 | 22 | 58 |
| [base_local_planner/cfg/LocalPlannerLimits.cfg](/base_local_planner/cfg/LocalPlannerLimits.cfg) | Properties | 0 | 7 | 4 | 11 |
| [base_local_planner/include/base_local_planner/costmap_model.h](/base_local_planner/include/base_local_planner/costmap_model.h) | C++ | 19 | 75 | 9 | 103 |
| [base_local_planner/include/base_local_planner/footprint_helper.h](/base_local_planner/include/base_local_planner/footprint_helper.h) | C++ | 22 | 56 | 10 | 88 |
| [base_local_planner/include/base_local_planner/goal_functions.h](/base_local_planner/include/base_local_planner/goal_functions.h) | C++ | 41 | 100 | 13 | 154 |
| [base_local_planner/include/base_local_planner/latched_stop_rotate_controller.h](/base_local_planner/include/base_local_planner/latched_stop_rotate_controller.h) | C++ | 55 | 22 | 17 | 94 |
| [base_local_planner/include/base_local_planner/line_iterator.h](/base_local_planner/include/base_local_planner/line_iterator.h) | C++ | 96 | 29 | 19 | 144 |
| [base_local_planner/include/base_local_planner/local_planner_limits.h](/base_local_planner/include/base_local_planner/local_planner_limits.h) | C++ | 73 | 37 | 12 | 122 |
| [base_local_planner/include/base_local_planner/local_planner_util.h](/base_local_planner/include/base_local_planner/local_planner_util.h) | C++ | 37 | 44 | 31 | 112 |
| [base_local_planner/include/base_local_planner/map_cell.h](/base_local_planner/include/base_local_planner/map_cell.h) | C++ | 15 | 44 | 9 | 68 |
| [base_local_planner/include/base_local_planner/map_grid.h](/base_local_planner/include/base_local_planner/map_grid.h) | C++ | 53 | 118 | 30 | 201 |
| [base_local_planner/include/base_local_planner/map_grid_cost_function.h](/base_local_planner/include/base_local_planner/map_grid_cost_function.h) | C++ | 40 | 82 | 18 | 140 |
| [base_local_planner/include/base_local_planner/map_grid_cost_point.h](/base_local_planner/include/base_local_planner/map_grid_cost_point.h) | C++ | 24 | 33 | 4 | 61 |
| [base_local_planner/include/base_local_planner/map_grid_visualizer.h](/base_local_planner/include/base_local_planner/map_grid_visualizer.h) | C++ | 20 | 45 | 7 | 72 |
| [base_local_planner/include/base_local_planner/obstacle_cost_function.h](/base_local_planner/include/base_local_planner/obstacle_cost_function.h) | C++ | 34 | 43 | 13 | 90 |
| [base_local_planner/include/base_local_planner/odometry_helper_ros.h](/base_local_planner/include/base_local_planner/odometry_helper_ros.h) | C++ | 26 | 53 | 14 | 93 |
| [base_local_planner/include/base_local_planner/oscillation_cost_function.h](/base_local_planner/include/base_local_planner/oscillation_cost_function.h) | C++ | 25 | 47 | 18 | 90 |
| [base_local_planner/include/base_local_planner/planar_laser_scan.h](/base_local_planner/include/base_local_planner/planar_laser_scan.h) | C++ | 14 | 40 | 4 | 58 |
| [base_local_planner/include/base_local_planner/point_grid.h](/base_local_planner/include/base_local_planner/point_grid.h) | C++ | 106 | 189 | 32 | 327 |
| [base_local_planner/include/base_local_planner/prefer_forward_cost_function.h](/base_local_planner/include/base_local_planner/prefer_forward_cost_function.h) | C++ | 18 | 36 | 11 | 65 |
| [base_local_planner/include/base_local_planner/simple_scored_sampling_planner.h](/base_local_planner/include/base_local_planner/simple_scored_sampling_planner.h) | C++ | 22 | 79 | 18 | 119 |
| [base_local_planner/include/base_local_planner/simple_trajectory_generator.h](/base_local_planner/include/base_local_planner/simple_trajectory_generator.h) | C++ | 57 | 96 | 21 | 174 |
| [base_local_planner/include/base_local_planner/trajectory.h](/base_local_planner/include/base_local_planner/trajectory.h) | C++ | 24 | 80 | 15 | 119 |
| [base_local_planner/include/base_local_planner/trajectory_cost_function.h](/base_local_planner/include/base_local_planner/trajectory_cost_function.h) | C++ | 22 | 56 | 13 | 91 |
| [base_local_planner/include/base_local_planner/trajectory_inc.h](/base_local_planner/include/base_local_planner/trajectory_inc.h) | C++ | 10 | 33 | 5 | 48 |
| [base_local_planner/include/base_local_planner/trajectory_planner.h](/base_local_planner/include/base_local_planner/trajectory_planner.h) | C++ | 126 | 211 | 49 | 386 |
| [base_local_planner/include/base_local_planner/trajectory_planner_ros.h](/base_local_planner/include/base_local_planner/trajectory_planner_ros.h) | C++ | 83 | 114 | 35 | 232 |
| [base_local_planner/include/base_local_planner/trajectory_sample_generator.h](/base_local_planner/include/base_local_planner/trajectory_sample_generator.h) | C++ | 14 | 51 | 11 | 76 |
| [base_local_planner/include/base_local_planner/trajectory_search.h](/base_local_planner/include/base_local_planner/trajectory_search.h) | C++ | 13 | 48 | 11 | 72 |
| [base_local_planner/include/base_local_planner/twirling_cost_function.h](/base_local_planner/include/base_local_planner/twirling_cost_function.h) | C++ | 13 | 43 | 9 | 65 |
| [base_local_planner/include/base_local_planner/velocity_iterator.h](/base_local_planner/include/base_local_planner/velocity_iterator.h) | C++ | 48 | 42 | 10 | 100 |
| [base_local_planner/include/base_local_planner/voxel_grid_model.h](/base_local_planner/include/base_local_planner/voxel_grid_model.h) | C++ | 69 | 92 | 19 | 180 |
| [base_local_planner/include/base_local_planner/world_model.h](/base_local_planner/include/base_local_planner/world_model.h) | C++ | 40 | 63 | 12 | 115 |
| [base_local_planner/msg/Position2DInt.msg](/base_local_planner/msg/Position2DInt.msg) | ROS Message | 2 | 0 | 0 | 2 |
| [base_local_planner/out/linecount.json](/base_local_planner/out/linecount.json) | JSON | 425 | 0 | 0 | 425 |
| [base_local_planner/package.xml](/base_local_planner/package.xml) | XML | 43 | 0 | 8 | 51 |
| [base_local_planner/setup.py](/base_local_planner/setup.py) | Python | 7 | 1 | 3 | 11 |
| [base_local_planner/src/costmap_model.cpp](/base_local_planner/src/costmap_model.cpp) | C++ | 65 | 55 | 26 | 146 |
| [base_local_planner/src/footprint_helper.cpp](/base_local_planner/src/footprint_helper.cpp) | C++ | 169 | 50 | 30 | 249 |
| [base_local_planner/src/goal_functions.cpp](/base_local_planner/src/goal_functions.cpp) | C++ | 179 | 45 | 22 | 246 |
| [base_local_planner/src/latched_stop_rotate_controller.cpp](/base_local_planner/src/latched_stop_rotate_controller.cpp) | C++ | 194 | 40 | 46 | 280 |
| [base_local_planner/src/local_planner_limits/__init__.py](/base_local_planner/src/local_planner_limits/__init__.py) | Python | 19 | 12 | 11 | 42 |
| [base_local_planner/src/local_planner_util.cpp](/base_local_planner/src/local_planner_util.cpp) | C++ | 68 | 40 | 21 | 129 |
| [base_local_planner/src/map_cell.cpp](/base_local_planner/src/map_cell.cpp) | C++ | 15 | 33 | 5 | 53 |
| [base_local_planner/src/map_grid.cpp](/base_local_planner/src/map_grid.cpp) | C++ | 230 | 42 | 38 | 310 |
| [base_local_planner/src/map_grid_cost_function.cpp](/base_local_planner/src/map_grid_cost_function.cpp) | C++ | 77 | 41 | 14 | 132 |
| [base_local_planner/src/map_grid_visualizer.cpp](/base_local_planner/src/map_grid_visualizer.cpp) | C++ | 52 | 33 | 11 | 96 |
| [base_local_planner/src/obstacle_cost_function.cpp](/base_local_planner/src/obstacle_cost_function.cpp) | C++ | 86 | 44 | 23 | 153 |
| [base_local_planner/src/odometry_helper_ros.cpp](/base_local_planner/src/odometry_helper_ros.cpp) | C++ | 54 | 41 | 12 | 107 |
| [base_local_planner/src/oscillation_cost_function.cpp](/base_local_planner/src/oscillation_cost_function.cpp) | C++ | 114 | 44 | 21 | 179 |
| [base_local_planner/src/point_grid.cpp](/base_local_planner/src/point_grid.cpp) | C++ | 350 | 116 | 91 | 557 |
| [base_local_planner/src/point_grid_node.cpp](/base_local_planner/src/point_grid_node.cpp) | C++ | 129 | 52 | 41 | 222 |
| [base_local_planner/src/prefer_forward_cost_function.cpp](/base_local_planner/src/prefer_forward_cost_function.cpp) | C++ | 13 | 9 | 7 | 29 |
| [base_local_planner/src/simple_scored_sampling_planner.cpp](/base_local_planner/src/simple_scored_sampling_planner.cpp) | C++ | 95 | 47 | 12 | 154 |
| [base_local_planner/src/simple_trajectory_generator.cpp](/base_local_planner/src/simple_trajectory_generator.cpp) | C++ | 181 | 83 | 33 | 297 |
| [base_local_planner/src/trajectory.cpp](/base_local_planner/src/trajectory.cpp) | C++ | 39 | 33 | 9 | 81 |
| [base_local_planner/src/trajectory_planner.cpp](/base_local_planner/src/trajectory_planner.cpp) | C++ | 666 | 179 | 157 | 1,002 |
| [base_local_planner/src/trajectory_planner_ros.cpp](/base_local_planner/src/trajectory_planner_ros.cpp) | C++ | 402 | 105 | 111 | 618 |
| [base_local_planner/src/twirling_cost_function.cpp](/base_local_planner/src/twirling_cost_function.cpp) | C++ | 7 | 6 | 6 | 19 |
| [base_local_planner/src/voxel_grid_model.cpp](/base_local_planner/src/voxel_grid_model.cpp) | C++ | 207 | 59 | 49 | 315 |
| [base_local_planner/test/footprint_helper_test.cpp](/base_local_planner/test/footprint_helper_test.cpp) | C++ | 99 | 17 | 30 | 146 |
| [base_local_planner/test/gtest_main.cpp](/base_local_planner/test/gtest_main.cpp) | C++ | 7 | 6 | 6 | 19 |
| [base_local_planner/test/line_iterator_test.cpp](/base_local_planner/test/line_iterator_test.cpp) | C++ | 48 | 28 | 6 | 82 |
| [base_local_planner/test/map_grid_test.cpp](/base_local_planner/test/map_grid_test.cpp) | C++ | 163 | 11 | 33 | 207 |
| [base_local_planner/test/trajectory_generator_test.cpp](/base_local_planner/test/trajectory_generator_test.cpp) | C++ | 12 | 6 | 9 | 27 |
| [base_local_planner/test/utest.cpp](/base_local_planner/test/utest.cpp) | C++ | 137 | 48 | 31 | 216 |
| [base_local_planner/test/velocity_iterator_test.cpp](/base_local_planner/test/velocity_iterator_test.cpp) | C++ | 125 | 35 | 19 | 179 |
| [base_local_planner/test/wavefront_map_accessor.h](/base_local_planner/test/wavefront_map_accessor.h) | C++ | 30 | 11 | 10 | 51 |

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)