cmake_minimum_required(VERSION 3.8)
project(simple_racing)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
#安装py脚本
install(PROGRAMS
  scripts/send_gear.py
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()