cmake_minimum_required(VERSION 3.8)
project(inspire_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/inspire_hand_command
  scripts/inspire_hand_teleop
  scripts/rtsp_image_publisher
  scripts/video_file_image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  description
  launch
  rviz
  test_data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Keep behavior tests and XML/CMake lint active, but do not enforce this
  # environment's quote/docstring style policy on tutorial Python examples.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hand_mapping test/test_hand_mapping.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_urdf_builder test/test_urdf_builder.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_package_contracts test/test_package_contracts.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
