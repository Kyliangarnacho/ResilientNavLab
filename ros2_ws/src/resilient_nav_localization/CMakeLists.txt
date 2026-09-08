cmake_minimum_required(VERSION 3.8)
project(resilient_nav_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/wheel_odometry_uncertainty.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_odometry_uncertainty
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_localization_resources
    test/test_localization_resources.py
  )

  ament_add_pytest_test(
    test_wheel_odometry_uncertainty
    test/test_wheel_odometry_uncertainty.py
  )
endif()

ament_package()
