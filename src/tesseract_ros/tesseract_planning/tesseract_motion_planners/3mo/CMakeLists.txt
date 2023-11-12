find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)

# MMMO
add_library(
  ${PROJECT_NAME}_3mo
  src/3mo_motion_planner.cpp
  src/3mo_utils.cpp
  src/profile/3mo_planner_plan_profile.cpp
  src/profile/3mo_planner_ik_plan_profile.cpp
  )
target_link_libraries(${PROJECT_NAME}_3mo PUBLIC ${PROJECT_NAME}_core Boost::boost)
target_compile_options(${PROJECT_NAME}_3mo PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_3mo PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_3mo PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_3mo ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_3mo PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_3mo
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_3mo PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install targets
install_targets(TARGETS ${PROJECT_NAME}_3mo)
