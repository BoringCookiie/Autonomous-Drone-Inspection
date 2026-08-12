cmake_minimum_required(VERSION 3.8)
project(uas_earthen_inspection)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install executable scripts
install(PROGRAMS
  uas_earthen_inspection/per_waypoint_capture_node.py
  uas_earthen_inspection/detection_node.py
  uas_earthen_inspection/rag_knowledge_base.py
  uas_earthen_inspection/revisit_waypoint_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (launch files, config, launch scripts)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
