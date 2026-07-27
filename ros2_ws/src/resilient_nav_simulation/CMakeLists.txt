cmake_minimum_required(VERSION 3.8)
project(resilient_nav_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_simulation_resources
    test/test_simulation_resources.py
  )
endif()

ament_package()
