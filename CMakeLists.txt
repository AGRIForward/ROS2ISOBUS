cmake_minimum_required(VERSION 3.8)
project(ros2_isobus)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(address_manager_node
  src/AddressManager/AddressManagerNode.cpp
  src/AddressManager/AddressManager.cpp)
target_include_directories(address_manager_node PRIVATE include src/AddressManager ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(address_manager_node rclcpp std_msgs rosidl_default_runtime)

add_executable(tecu_node
  src/TECUClient/TECUClientNode.cpp
  src/TECUClient/TECUClass2Client.cpp
  src/TECUClient/TECUClass2ClientNode.cpp
  src/TECUClient/TECUClass3Client.cpp
  src/TECUClient/TECUClass3ClientNode.cpp)
target_include_directories(tecu_node PRIVATE include src/TECUClient ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(tecu_node rclcpp std_msgs geometry_msgs rosidl_default_runtime)

add_executable(test_panel_node
  src/TestPanel/TestPanelNode.cpp)
target_include_directories(test_panel_node PRIVATE include src ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(test_panel_node
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  rosidl_default_runtime)

add_executable(nmea2000_node
  src/NMEA2000Client/NMEA2000ClientNode.cpp
  src/NMEA2000Client/NMEA2000Client.cpp)
target_include_directories(nmea2000_node PRIVATE include src/NMEA2000Client ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(nmea2000_node rclcpp std_msgs sensor_msgs geometry_msgs diagnostic_msgs rosidl_default_runtime)

add_executable(can_bridge_node 
  src/CanBridge/CanBridgeNode.cpp
  src/CanBridge/CanBridge.cpp
  src/CanBridge/TransportProtocol.cpp)
target_include_directories(can_bridge_node PRIVATE include src/CanBridge ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(can_bridge_node rclcpp std_msgs rosidl_default_runtime)

add_executable(diagnostics_node
  src/Diagnostics/Diagnostics.cpp
  src/Diagnostics/DiagnosticsNode.cpp)
target_include_directories(diagnostics_node PRIVATE include src/Diagnostics ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(diagnostics_node rclcpp rosidl_default_runtime)

add_executable(tim_client_node
  src/TIMClient/TIMClientNode.cpp
  src/TIMClient/DummyAuthProvider.cpp
  src/TIMClient/AuthLibProvider.cpp
  src/TIMClient/TIMClient.cpp)
target_include_directories(tim_client_node PRIVATE include src/TIMClient ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(tim_client_node rclcpp rosidl_default_runtime)
ament_target_dependencies(tim_client_node ament_index_cpp)

# AuthLib direct dependency for AuthLibProvider.
# If AuthLib package is not available, remove this block and remove AuthLibProvider.cpp
# from tim_client_node sources above.
set(AUTHLIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/AuthLib")
target_include_directories(tim_client_node PRIVATE "${AUTHLIB_ROOT}/include")
target_include_directories(tim_client_node PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/CryptoLib/include")
# NOTE: in this package delivery, libauthlib.so is a text linker script.
# Use the real ELF shared object directly.
set(AUTHLIB_SO "${AUTHLIB_ROOT}/lib/libauthlib.so.1.5.4")
target_link_libraries(tim_client_node "${AUTHLIB_SO}")
set_target_properties(tim_client_node PROPERTIES
  BUILD_RPATH "${AUTHLIB_ROOT}/lib"
  INSTALL_RPATH "$ORIGIN/..")

install(TARGETS
  address_manager_node
  nmea2000_node
  tecu_node
  test_panel_node
  can_bridge_node
  diagnostics_node
  tim_client_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  "${AUTHLIB_ROOT}/lib/libauthlib.so.1"
  "${AUTHLIB_ROOT}/lib/libauthlib.so.1.5.4"
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py")

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml")

install(DIRECTORY third_party/AuthLib/cert/
  DESTINATION share/${PROJECT_NAME}/third_party/AuthLib/cert)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IsobusFrame.msg"
  "msg/IsobusTpFrame.msg"
  "msg/IsobusAddressStatus.msg"
  "msg/IsobusAddressEntry.msg"
  "msg/IsobusAddressBook.msg"
  "msg/TecuWheelSpeed.msg"
  "msg/TecuGroundSpeed.msg"
  "msg/TecuRearHitchStatus.msg"
  "msg/TecuRearPtoStatus.msg"
  "msg/TecuSteeringWheel.msg"
  "msg/TecuSteeringValveStatus.msg"
  "msg/TecuGuidanceStatus.msg"
  "msg/TecuCruiseStatus.msg"
  "msg/TecuCruiseCommand.msg"
  "msg/TecuGuidanceCommand.msg"
  "msg/TecuRearHitchCommand.msg"
  "msg/TecuRearPtoCommand.msg"
  "msg/AuxValveCommand.msg"
  "msg/AuxValveStatus.msg"
  "msg/TimCruiseCommand.msg"
  "msg/TimCruiseStatus.msg"
  "msg/TimCurvatureCommand.msg"
  "msg/TimCurvatureStatus.msg"
  "msg/TimRearHitchCommand.msg"
  "msg/TimRearHitchStatus.msg"
  "msg/TimRearPtoCommand.msg"
  "msg/TimRearPtoStatus.msg"
  "msg/TimAuxValveCommand.msg"
  "msg/TimAuxValveStatus.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(address_manager_node "${cpp_typesupport_target}")
target_link_libraries(can_bridge_node "${cpp_typesupport_target}")
target_link_libraries(nmea2000_node "${cpp_typesupport_target}")
target_link_libraries(tecu_node "${cpp_typesupport_target}")
target_link_libraries(test_panel_node "${cpp_typesupport_target}")
target_link_libraries(diagnostics_node "${cpp_typesupport_target}")
target_link_libraries(tim_client_node "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

ament_package()
