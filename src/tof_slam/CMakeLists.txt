cmake_minimum_required(VERSION 3.8)
project(tof_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# ROS version auto-detection: $ENV{ROS_VERSION} = 1 or 2 (default: 2)
# ============================================================================
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VER $ENV{ROS_VERSION})
else()
  set(ROS_VER 2)
  message(STATUS "ROS_VERSION not set, defaulting to ROS 2")
endif()
message(STATUS "Building for ROS ${ROS_VER}")

# ============================================================================
# Common (ROS-independent) dependencies
# ============================================================================
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found: ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# ============================================================================
# Core library: common (no ROS, shared types + Lie groups)
# ============================================================================
add_library(tof_slam_common STATIC
  src/common/lie/so3.cpp
  src/common/lie/se3.cpp
  src/common/types/state.cpp
  src/common/se3.cpp
  src/common/plane_fitting.cpp
)
target_include_directories(tof_slam_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unordered_dense>
)
target_link_libraries(tof_slam_common PUBLIC Eigen3::Eigen)
target_compile_features(tof_slam_common PUBLIC cxx_std_17)

# ============================================================================
# Core library: frontend (LIO pipeline, no ROS)
# ============================================================================
add_library(tof_slam_frontend STATIC
  src/frontend/filter/voxel_grid.cpp
  src/frontend/robust/pko.cpp
  src/frontend/estimator/imu_propagator.cpp
  src/frontend/estimator/gravity_init.cpp
  src/frontend/estimator/lidar_jacobian.cpp
  src/frontend/estimator/iekf_updater.cpp
  src/frontend/estimator/imu_preintegration.cpp
  src/frontend/estimator/fixed_lag_smoother.cpp
  src/frontend/estimator/motion_undistort.cpp
  src/frontend/estimator/correspondence_finder.cpp
  src/frontend/estimator/lio_estimator.cpp
  src/frontend/map/surfel_map.cpp
  src/frontend/map/point_voxel_map.cpp
  src/frontend/policy/scene_classifier.cpp
)
target_include_directories(tof_slam_frontend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unordered_dense>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty>
)
target_link_libraries(tof_slam_frontend PUBLIC
  tof_slam_common Eigen3::Eigen spdlog::spdlog ${PCL_LIBRARIES} pthread)
target_include_directories(tof_slam_frontend PUBLIC ${PCL_INCLUDE_DIRS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(tof_slam_frontend PUBLIC OpenMP::OpenMP_CXX)
endif()
target_compile_features(tof_slam_frontend PUBLIC cxx_std_17)

# ============================================================================
# ROS2-only libraries: frontend_w, mapping, backend
# These are NOT linked by the ROS1 production binary (tofslam_node).
# Moved inside if(ROS_VER EQUAL 2) guard to skip compilation for ROS1 builds.
# ============================================================================

# ============================================================================
# Core test sources list (common + frontend + frontend_w, ROS-independent)
# ============================================================================
set(CORE_TEST_NAMES
  so3_test se3_test point_types_test state_test filter_test pko_test
  imu_propagator_test gravity_init_test lidar_jacobian_test motion_undistort_test
  surfel_map_test correspondence_finder_test iekf_updater_test lio_estimator_test
  boundary_hash_test anisotropic_iekf_test
)
set(CORE_TEST_SOURCES
  test/common/lie/so3_test.cpp
  test/common/lie/se3_test.cpp
  test/common/types/point_types_test.cpp
  test/common/types/state_test.cpp
  test/frontend/filter/filter_test.cpp
  test/frontend/robust/pko_test.cpp
  test/frontend/estimator/imu_propagator_test.cpp
  test/frontend/estimator/gravity_init_test.cpp
  test/frontend/estimator/lidar_jacobian_test.cpp
  test/frontend/estimator/motion_undistort_test.cpp
  test/frontend/map/surfel_map_test.cpp
  test/frontend/estimator/correspondence_finder_test.cpp
  test/frontend/estimator/iekf_updater_test.cpp
  test/frontend/estimator/lio_estimator_test.cpp
  test/frontend/diag/boundary_hash_test.cpp
  test/frontend/estimator/anisotropic_iekf_test.cpp
)
set(CORE_TEST_LIBS
  tof_slam_common tof_slam_common tof_slam_common tof_slam_common
  tof_slam_frontend tof_slam_frontend tof_slam_frontend tof_slam_frontend
  tof_slam_frontend tof_slam_frontend tof_slam_frontend tof_slam_frontend
  tof_slam_frontend tof_slam_frontend tof_slam_frontend tof_slam_frontend
)

# ============================================================================
# ROS 2 build
# ============================================================================
if(ROS_VER EQUAL 2)
  find_package(Ceres REQUIRED)
  find_package(ament_cmake REQUIRED)

  # -- frontend_w (LWO wheel-odometry pipeline) --
  add_library(tof_slam_frontend_w STATIC
    src/frontend_w/estimator/lwo_estimator.cpp
    src/frontend_w/estimator/lwo_iekf_updater.cpp
    src/frontend_w/estimator/lwo_state.cpp
    src/frontend_w/estimator/wheel_propagator.cpp
    src/frontend_w/measurement/ground_constraint.cpp
    src/frontend_w/measurement/wheel_measurement.cpp
  )
  target_include_directories(tof_slam_frontend_w PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unordered_dense>
  )
  target_link_libraries(tof_slam_frontend_w PUBLIC
    tof_slam_common tof_slam_frontend Eigen3::Eigen spdlog::spdlog)
  target_compile_features(tof_slam_frontend_w PUBLIC cxx_std_17)

  # -- mapping (iVox, keyframe, occupancy grid, map aligner) --
  add_library(tof_slam_mapping STATIC
    src/mapping/ivox_map.cpp
    src/mapping/keyframe_manager.cpp
    src/mapping/occupancy_grid.cpp
    src/mapping/map_aligner.cpp
  )
  target_include_directories(tof_slam_mapping PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unordered_dense>
  )
  target_link_libraries(tof_slam_mapping PUBLIC
    tof_slam_common Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS} spdlog::spdlog)
  target_include_directories(tof_slam_mapping PUBLIC ${PCL_INCLUDE_DIRS})

  # -- backend (pose graph, loop detector, map saver, PGO) --
  add_library(tof_slam_backend STATIC
    src/backend/pose_graph.cpp
    src/backend/loop_detector.cpp
    src/backend/place_recognition.cpp
    src/backend/loop_closure_manager.cpp
    src/backend/map_saver.cpp
    src/backend/pgo_debug_logger.cpp
    src/backend/pgo_convergence_monitor.cpp
  )
  target_include_directories(tof_slam_backend PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(tof_slam_backend PUBLIC
    tof_slam_common tof_slam_mapping Eigen3::Eigen
    ${PCL_LIBRARIES} Ceres::ceres ${OpenCV_LIBS} spdlog::spdlog)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(pcl_ros REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  # RViz2 plugin (optional)
  find_package(rviz_common QUIET)
  find_package(pluginlib QUIET)
  if(rviz_common_FOUND AND pluginlib_FOUND)
    set(BUILD_RVIZ_PLUGIN TRUE)
  else()
    set(BUILD_RVIZ_PLUGIN FALSE)
    message(STATUS "rviz_common or pluginlib not found — skipping RViz2 panel plugin")
  endif()

  set(ROS2_DEPS
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
    pcl_ros
    visualization_msgs
    std_srvs
  )

  # Propagate ament deps to ROS-independent mapping/backend libs so that
  # downstream ament_target_dependencies on tof_slam_ros resolves correctly.
  ament_target_dependencies(tof_slam_mapping PUBLIC ${ROS2_DEPS})
  ament_target_dependencies(tof_slam_backend PUBLIC ${ROS2_DEPS})

  # --- ROS2 adapter library (sensors + ros/ integration) ---
  add_library(tof_slam_ros STATIC
    src/sensors/tof_preprocessor.cpp
    src/sensors/imu_preprocessor.cpp
    src/sensors/wheel_prior_builder.cpp
    src/ros/point_cloud_adapter.cpp
    src/ros/imu_adapter.cpp
    src/ros/csv_logger.cpp
    src/ros/state_publisher.cpp
    src/ros/param_loader.cpp
    src/ros/backend_runner.cpp
  )
  target_include_directories(tof_slam_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unordered_dense>
  )
  target_link_libraries(tof_slam_ros PUBLIC
    tof_slam_frontend
    tof_slam_frontend_w
    tof_slam_backend
    tof_slam_mapping
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    yaml-cpp
    spdlog::spdlog
  )
  if(OpenMP_CXX_FOUND)
    target_link_libraries(tof_slam_ros PUBLIC OpenMP::OpenMP_CXX)
  endif()
  ament_target_dependencies(tof_slam_ros PUBLIC ${ROS2_DEPS})

  # --- ROS2 executable ---
  add_executable(tofslam_node
    src/tofslam_main.cpp
    src/ros/slam_node.cpp
  )
  target_include_directories(tofslam_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(tofslam_node
    tof_slam_ros
    tof_slam_frontend
    tof_slam_frontend_w
  )
  ament_target_dependencies(tofslam_node ${ROS2_DEPS})

  # --- RViz2 panel plugin ---
  if(BUILD_RVIZ_PLUGIN)
    add_library(tof_slam_rviz_plugin SHARED
      src/rviz/save_map_panel.cpp
    )
    target_include_directories(tof_slam_rviz_plugin PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(tof_slam_rviz_plugin
      rclcpp
      std_srvs
      rviz_common
      pluginlib
    )
    pluginlib_export_plugin_description_file(rviz_common rviz_plugin.xml)

    install(TARGETS tof_slam_rviz_plugin
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    install(FILES rviz_plugin.xml
      DESTINATION share/${PROJECT_NAME}
    )
  endif()

  # --- Install ---
  install(TARGETS
    tof_slam_common tof_slam_frontend tof_slam_frontend_w
    tof_slam_mapping tof_slam_backend tof_slam_ros
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS tofslam_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/
    DESTINATION include
  )
  install(DIRECTORY launch/ros2/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
  install(DIRECTORY scripts/
    DESTINATION share/${PROJECT_NAME}/scripts
    USE_SOURCE_PERMISSIONS
  )

  # --- Testing ---
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    list(LENGTH CORE_TEST_NAMES _n_tests)
    math(EXPR _last "${_n_tests} - 1")
    foreach(_i RANGE ${_last})
      list(GET CORE_TEST_NAMES ${_i} _tname)
      list(GET CORE_TEST_SOURCES ${_i} _tsrc)
      list(GET CORE_TEST_LIBS ${_i} _tlib)
      ament_add_gtest(${_tname} ${_tsrc})
      target_link_libraries(${_tname} ${_tlib})
    endforeach()

    # --- Frontend_W tests ---
    ament_add_gtest(lwo_state_test
      test/frontend_w/estimator/lwo_state_test.cpp)
    target_link_libraries(lwo_state_test tof_slam_frontend_w)

    ament_add_gtest(wheel_propagator_test
      test/frontend_w/estimator/wheel_propagator_test.cpp)
    target_link_libraries(wheel_propagator_test tof_slam_frontend_w)

    ament_add_gtest(ground_constraint_test
      test/frontend_w/measurement/ground_constraint_test.cpp)
    target_link_libraries(ground_constraint_test tof_slam_frontend_w)

    # --- ROS2 adapter tests ---
    ament_add_gtest(point_cloud_adapter_test test/ros/point_cloud_adapter_test.cpp)
    target_link_libraries(point_cloud_adapter_test tof_slam_ros tof_slam_frontend)
    ament_target_dependencies(point_cloud_adapter_test rclcpp sensor_msgs)

    ament_add_gtest(imu_adapter_test test/ros/imu_adapter_test.cpp)
    target_link_libraries(imu_adapter_test tof_slam_ros tof_slam_frontend)
    ament_target_dependencies(imu_adapter_test rclcpp sensor_msgs)
  endif()

  ament_package()

# ============================================================================
# ROS 1 build
# ============================================================================
elseif(ROS_VER EQUAL 1)
  set(_ros1_required_deps
    roscpp sensor_msgs nav_msgs geometry_msgs
    tf2 tf2_ros tf2_geometry_msgs
    pcl_conversions pcl_ros visualization_msgs
  )
  find_package(catkin REQUIRED COMPONENTS ${_ros1_required_deps})

  # Optional: livox_ros_driver2 (not needed for standard PointCloud2 datasets)
  find_package(livox_ros_driver2 QUIET)
  if(livox_ros_driver2_FOUND)
    message(STATUS "livox_ros_driver2 found — Livox CustomMsg support enabled")
    list(APPEND _ros1_required_deps livox_ros_driver2)
    add_definitions(-DHAS_LIVOX_ROS_DRIVER2)
    # Re-find catkin with livox_ros_driver2 included so catkin_INCLUDE_DIRS is updated
    find_package(catkin REQUIRED COMPONENTS ${_ros1_required_deps})
  else()
    message(STATUS "livox_ros_driver2 NOT found — Livox CustomMsg support disabled (PointCloud2 only)")
  endif()

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tof_slam_common tof_slam_frontend tof_slam_ros1
    CATKIN_DEPENDS ${_ros1_required_deps}
    DEPENDS Eigen3 PCL
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # --- ROS1 adapter library ---
  add_library(tof_slam_ros1 STATIC
    src/ros1/point_cloud_adapter.cpp
    src/ros1/livox_custom_msg_adapter.cpp
    src/ros1/imu_adapter.cpp
    src/ros1/tof_preprocessor.cpp
    src/ros1/imu_preprocessor.cpp
  )
  target_include_directories(tof_slam_ros1 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unordered_dense>
  )
  target_link_libraries(tof_slam_ros1 PUBLIC
    tof_slam_frontend
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    yaml-cpp
    spdlog::spdlog
    ${catkin_LIBRARIES}
  )
  if(OpenMP_CXX_FOUND)
    target_link_libraries(tof_slam_ros1 PUBLIC OpenMP::OpenMP_CXX)
  endif()
  # Ensure message headers are generated before compiling
  if(livox_ros_driver2_FOUND)
    add_dependencies(tof_slam_ros1 livox_ros_driver2_generate_messages_cpp)
  endif()

  # --- ROS1 executable ---
  add_executable(tofslam_node
    src/ros1/main.cpp
    src/ros1/slam_node.cpp
  )
  target_include_directories(tofslam_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(tofslam_node tof_slam_ros1 tof_slam_frontend ${catkin_LIBRARIES})

  # --- Install ---
  install(TARGETS tof_slam_common tof_slam_frontend tof_slam_ros1 tofslam_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  install(DIRECTORY launch/ros1/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

  # --- Testing ---
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    list(LENGTH CORE_TEST_NAMES _n_tests)
    math(EXPR _last "${_n_tests} - 1")
    foreach(_i RANGE ${_last})
      list(GET CORE_TEST_NAMES ${_i} _tname)
      list(GET CORE_TEST_SOURCES ${_i} _tsrc)
      list(GET CORE_TEST_LIBS ${_i} _tlib)
      catkin_add_gtest(${_tname} ${_tsrc})
      target_link_libraries(${_tname} ${_tlib})
    endforeach()

  endif()

else()
  message(FATAL_ERROR "Unsupported ROS_VERSION: ${ROS_VER}. Must be 1 or 2.")
endif()
