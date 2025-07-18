cmake_minimum_required(VERSION 3.8)
project(rov_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Manip.msg"
  "msg/VideoWidgetSwitch.msg"
  "msg/Flooding.msg"
  "msg/Temperature.msg"
  "msg/IPAddress.msg"
  "msg/VehicleState.msg"
  "msg/ValveManip.msg"
  "msg/Heartbeat.msg"
  "msg/MissionTimerTick.msg"
  "msg/FloatData.msg"
  "msg/FloatCommand.msg"
  "msg/FloatSerial.msg"
  "msg/FloatSingle.msg"
  "msg/Intrinsics.msg"
  "srv/AutonomousFlight.srv"
  "srv/GeneratePhotosphere.srv"
  "srv/MissionTimerSet.srv"
  "srv/CameraManage.srv"
  "srv/VehicleArming.srv"
  "srv/TakePhotosphere.srv"
  DEPENDENCIES std_msgs
)

ament_package()
