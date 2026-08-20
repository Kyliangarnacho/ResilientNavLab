cmake_minimum_required(VERSION 3.8)
project(resilient_nav_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/motion_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_test
)

install(
  PROGRAMS scripts/phase9_mapping_route.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME phase9_mapping_route
)

install(
  FILES scripts/motion_safety.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_motion_test
    test/test_motion_test.py
  )

  ament_add_pytest_test(
    test_simulation_resources
    test/test_simulation_resources.py
  )
endif()

ament_package()
