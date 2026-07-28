cmake_minimum_required(VERSION 3.8)
project(resilient_nav_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_description_resources
    test/test_description_resources.py
  )
endif()

ament_package()
