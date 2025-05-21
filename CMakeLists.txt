cmake_minimum_required(VERSION 3.5)
project(manipulator_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)



add_executable(driver 
# common file
src/main.cpp 
src/canDriver.cpp  
src/ManipulatorInterface.cpp 
src/ManipulatorPlanner.cpp
src/kinematicsController.cpp
#canopen related file
canopenAPP/GlobalVariables.cpp
canopenAPP/ProfPosition.cpp
canopenAPP/ProfTorque.cpp
canopenAPP/InterPosition.cpp
canopenAPP/canopen.cpp
canopenAPP/nmt.cpp
canopenAPP/OD.cpp
canopenAPP/pdo.cpp
canopenAPP/sdo.cpp
canopenAPP/sync.cpp
)

target_link_libraries(driver  
  ${PROJECT_SOURCE_DIR}/thirdLib/canlib/libcanlib.so
)  
# target link desired packages
ament_target_dependencies(driver rclcpp std_msgs sensor_msgs)
install(TARGETS  driver
  DESTINATION lib/${PROJECT_NAME}
  )




include_directories(
${PROJECT_SOURCE_DIR}/thirdLib/canlib 
${PROJECT_SOURCE_DIR}/canopenAPP 
include)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
