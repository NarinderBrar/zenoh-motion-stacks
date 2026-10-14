cmake_minimum_required(VERSION 3.22)
project(motion_stacks VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ZMS_FETCH_MISSING_DEPENDENCIES "Fetch dependencies unavailable on the system" OFF)
include(cmake/dependencies.cmake)

add_library(zms_contract contract/src/Contract.cpp)
add_library(motion_stacks::contract ALIAS zms_contract)
set_target_properties(zms_contract PROPERTIES EXPORT_NAME contract)
target_include_directories(zms_contract PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/contract/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(zms_contract PUBLIC nlohmann_json::nlohmann_json)

add_library(zms_contract_transport contract/src/ZenohClient.cpp)
add_library(motion_stacks::contract_transport ALIAS zms_contract_transport)
set_target_properties(zms_contract_transport PROPERTIES EXPORT_NAME contract_transport)
target_include_directories(zms_contract_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/contract/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(zms_contract_transport PUBLIC zms_contract zenohcxx::zenohc)

add_library(zms_navigation_core navigation/src/Navigation.cpp navigation/src/GridPlanner.cpp navigation/src/PlanningWorker.cpp)
add_library(motion_stacks::navigation_core ALIAS zms_navigation_core)
set_target_properties(zms_navigation_core PROPERTIES EXPORT_NAME navigation_core)
target_include_directories(zms_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/navigation/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(zms_navigation_core PUBLIC zms_contract)

add_library(zms_manipulation_core manipulation/src/JointModel.cpp)
add_library(motion_stacks::manipulation_core ALIAS zms_manipulation_core)
set_target_properties(zms_manipulation_core PROPERTIES EXPORT_NAME manipulation_core)
target_include_directories(zms_manipulation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/manipulation/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(zms_manipulation_core PUBLIC zms_contract PRIVATE urdfdom::urdf_parser)

add_executable(zms_navigation navigation/src/main.cpp)
target_link_libraries(zms_navigation PRIVATE zms_navigation_core zms_contract_transport)

add_executable(zms_manipulation manipulation/src/main.cpp)
target_link_libraries(zms_manipulation PRIVATE zms_manipulation_core zms_contract_transport)

add_executable(zms_cli tools/zms_cli/main.cpp)
target_link_libraries(zms_cli PRIVATE zms_contract_transport)

add_executable(zms_mock_robot tools/zms_mock_robot/main.cpp)
target_link_libraries(zms_mock_robot PRIVATE zms_contract zenohcxx::zenohc)

if(ZMS_INSTALLABLE)
install(TARGETS zms_contract zms_contract_transport zms_navigation_core zms_manipulation_core zms_navigation zms_manipulation zms_cli zms_mock_robot
  EXPORT motion_stacksTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY contract/include/ navigation/include/ manipulation/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT motion_stacksTargets
  FILE motion_stacksTargets.cmake
  NAMESPACE motion_stacks::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motion_stacks)
configure_package_config_file(cmake/motion_stacksConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/motion_stacksConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motion_stacks)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/motion_stacksConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/motion_stacksConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/motion_stacksConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motion_stacks)
install(DIRECTORY config/examples/ DESTINATION ${CMAKE_INSTALL_DATADIR}/motion_stacks/examples)
install(DIRECTORY docs/ DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(FILES README.md LICENSE DESTINATION ${CMAKE_INSTALL_DOCDIR})

set(CPACK_PACKAGE_NAME motion-stacks)
set(CPACK_PACKAGE_VENDOR "Motion Stacks contributors")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Standalone native Zenoh motion stacks")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_GENERATOR TGZ)
include(CPack)
else()
  message(STATUS "Install and package targets are disabled because dependencies were fetched into this build tree")
endif()

if(BUILD_TESTING)
  option(ZMS_ENABLE_LIVE_TESTS "Register native Zenoh integration tests" OFF)
  find_package(GTest REQUIRED)
  add_executable(zms_unit_tests tests/unit/main.cpp)
  target_link_libraries(zms_unit_tests PRIVATE zms_navigation_core zms_manipulation_core GTest::gtest_main)
  add_test(NAME zms_unit_tests COMMAND zms_unit_tests)
  set_tests_properties(zms_unit_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  add_executable(zms_conformance tests/conformance/main.cpp)
  target_link_libraries(zms_conformance PRIVATE zms_contract_transport GTest::gtest_main)
  add_test(NAME zms_conformance COMMAND zms_conformance)
  add_executable(zms_navigation_integration tests/integration/Navigation.cpp)
  target_link_libraries(zms_navigation_integration PRIVATE zms_contract_transport GTest::gtest_main)
  if(ZMS_ENABLE_LIVE_TESTS)
    add_test(NAME zms_navigation_integration COMMAND zms_navigation_integration)
  endif()
endif()
