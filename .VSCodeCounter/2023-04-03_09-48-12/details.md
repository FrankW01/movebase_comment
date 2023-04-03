# Details

Date : 2023-04-03 09:48:12

Directory /home/nvidia/code/movebase_comment/teb_local_planner

Total : 63 files,  9844 codes, 6221 comments, 3003 blanks, all 19068 lines

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
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

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)