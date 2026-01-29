cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

find_package(ament_cmake REQUIRED)

# Install package.xml so ROS can discover the package
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY uav_bringup/launch
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files so 'ros2 launch uav_bringup offboard_sitl.launch.py' can find them
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)




ament_package()

