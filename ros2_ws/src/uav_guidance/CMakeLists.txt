cmake_minimum_required(VERSION 3.8)
project(uav_guidance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install package.xml so ROS can discover the package
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install python package/module
ament_python_install_package(${PROJECT_NAME})

# Install executable script
install(PROGRAMS
  uav_guidance/offboard_hello.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME offboard_hello
)

ament_package()

