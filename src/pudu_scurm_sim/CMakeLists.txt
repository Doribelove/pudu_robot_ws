cmake_minimum_required(VERSION 3.8)
project(pudu_scurm_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/generate_3d_map.py
  scripts/ground_truth_map_builder.py
  scripts/localization_error.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
