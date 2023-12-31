find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)

if(NOT TARGET GTest::GTest)
  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_LIBRARIES})
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "gtest.lib")
    else()
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest.so")
    endif()
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_MAIN_LIBRARIES})
    set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "gtest_main.lib")
    else()
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest_main.so")
    endif()
  endif()
endif()

include(GoogleTest)

# OMPL Planning Test/Example Program
if(TESSERACT_BUILD_OMPL)
  add_executable(${PROJECT_NAME}_ompl_unit ompl_planner_tests.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_ompl_unit
    PRIVATE GTest::GTest
            GTest::Main
            ${PROJECT_NAME}_ompl
            tesseract::tesseract_support
            ${PROJECT_NAME}_simple)
  target_compile_options(${PROJECT_NAME}_ompl_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_ompl_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_ompl_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_ompl_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_ompl_unit
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${PROJECT_NAME}_ompl_unit)
  add_dependencies(${PROJECT_NAME}_ompl_unit ${PROJECT_NAME}_ompl)
  add_dependencies(run_tests ${PROJECT_NAME}_ompl_unit)

  # OMPL Constrained Planning Test/Example Program if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
  # add_executable(${PROJECT_NAME}_ompl_constrained_unit ompl_constrained_planner_tests.cpp)
  # target_link_libraries(${PROJECT_NAME}_ompl_constrained_unit PRIVATE Boost::boost Boost::serialization Boost::system
  # Boost::filesystem Boost::program_options GTest::GTest GTest::Main ${PROJECT_NAME}_ompl tesseract::tesseract_support
  # trajopt::trajopt) tesseract_target_compile_options(${PROJECT_NAME}_ompl_constrained_unit PRIVATE)
  # tesseract_clang_tidy(${PROJECT_NAME}_ompl_constrained_unit)
  # tesseract_code_coverage(${PROJECT_NAME}_ompl_constrained_unit ALL EXCLUDE ${COVERAGE_EXCLUDE})
  # tesseract_gtest_discover_tests(${PROJECT_NAME}_ompl_constrained_unit)
  # add_dependencies(${PROJECT_NAME}_ompl_constrained_unit ${PROJECT_NAME}_ompl) add_dependencies(run_tests
  # ${PROJECT_NAME}_ompl_constrained_unit) endif()
endif()

# SimplePlanner Tests
add_executable(${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit simple_planner_fixed_size_interpolation.cpp)
target_link_libraries(
  ${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::tesseract_support
          ${PROJECT_NAME}_simple)
target_compile_options(${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit
                       PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit
                           PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit PRIVATE
                   VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit)
add_dependencies(${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit ${PROJECT_NAME}_simple)
add_dependencies(run_tests ${PROJECT_NAME}_simple_planner_fixed_size_interpolation_unit)

add_executable(${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit
               simple_planner_fixed_size_assign_position.cpp)
target_link_libraries(
  ${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_simple
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit
                       PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit
                           PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit PRIVATE
                   VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit)
add_dependencies(${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit ${PROJECT_NAME}_simple)
add_dependencies(run_tests ${PROJECT_NAME}_simple_planner_fixed_size_assign_position_unit)

add_executable(${PROJECT_NAME}_simple_planner_lvs_interpolation_unit simple_planner_lvs_interpolation.cpp)
target_link_libraries(
  ${PROJECT_NAME}_simple_planner_lvs_interpolation_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_simple
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_simple_planner_lvs_interpolation_unit
                       PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_simple_planner_lvs_interpolation_unit
                           PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_simple_planner_lvs_interpolation_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_simple_planner_lvs_interpolation_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_simple_planner_lvs_interpolation_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_simple_planner_lvs_interpolation_unit)
add_dependencies(${PROJECT_NAME}_simple_planner_lvs_interpolation_unit ${PROJECT_NAME}_simple)
add_dependencies(run_tests ${PROJECT_NAME}_simple_planner_lvs_interpolation_unit)

# TrajOpt Planner Tests
if(TESSERACT_BUILD_TRAJOPT)
  add_executable(${PROJECT_NAME}_trajopt_unit trajopt_planner_tests.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_trajopt_unit
    PRIVATE GTest::GTest
            GTest::Main
            tesseract::tesseract_support
            ${PROJECT_NAME}_trajopt
            ${PROJECT_NAME}_simple)
  target_compile_options(${PROJECT_NAME}_trajopt_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                              ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_trajopt_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_trajopt_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_trajopt_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_trajopt_unit
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${PROJECT_NAME}_trajopt_unit)
  add_dependencies(${PROJECT_NAME}_trajopt_unit ${PROJECT_NAME}_trajopt)
  add_dependencies(run_tests ${PROJECT_NAME}_trajopt_unit)
endif()

# Descartes Planner Tests
if(TESSERACT_BUILD_DESCARTES)
  add_executable(${PROJECT_NAME}_descartes_unit descartes_planner_tests.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_descartes_unit
    PRIVATE GTest::GTest
            GTest::Main
            tesseract::tesseract_support
            tesseract::tesseract_kinematics_opw
            ${PROJECT_NAME}_descartes
            ${PROJECT_NAME}_simple)
  target_compile_options(${PROJECT_NAME}_descartes_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_descartes_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_descartes_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_descartes_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_descartes_unit
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${PROJECT_NAME}_descartes_unit)
  add_dependencies(${PROJECT_NAME}_descartes_unit ${PROJECT_NAME}_descartes)
  add_dependencies(run_tests ${PROJECT_NAME}_descartes_unit)
endif()

# Utils Tests
add_executable(${PROJECT_NAME}_utils_unit utils_test.cpp)
target_link_libraries(
  ${PROJECT_NAME}_utils_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::tesseract_support
          tesseract::tesseract_command_language
          ${PROJECT_NAME}_core
          ${PROJECT_NAME}_simple)
target_compile_options(${PROJECT_NAME}_utils_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                          ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_utils_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_utils_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_utils_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_utils_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_utils_unit)
add_dependencies(${PROJECT_NAME}_utils_unit ${PROJECT_NAME}_core)
add_dependencies(run_tests ${PROJECT_NAME}_utils_unit)

# Serialize Tests
if(TESSERACT_BUILD_OMPL AND TESSERACT_BUILD_TRAJOPT AND TESSERACT_BUILD_DESCARTES)
  add_executable(${PROJECT_NAME}_serialize_unit serialize_test.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_serialize_unit
    PRIVATE GTest::GTest
            GTest::Main
            ${PROJECT_NAME}_trajopt
            ${PROJECT_NAME}_ompl
            ${PROJECT_NAME}_descartes)
  target_compile_options(${PROJECT_NAME}_serialize_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_serialize_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_serialize_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_serialize_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_serialize_unit
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${PROJECT_NAME}_serialize_unit)
  add_dependencies(
    ${PROJECT_NAME}_serialize_unit
    ${PROJECT_NAME}_trajopt
    ${PROJECT_NAME}_ompl
    ${PROJECT_NAME}_descartes)
  add_dependencies(run_tests ${PROJECT_NAME}_serialize_unit)
endif()

# Profile Dictionary Tests
add_executable(${PROJECT_NAME}_profile_dictionary_unit profile_dictionary_tests.cpp)
target_link_libraries(${PROJECT_NAME}_profile_dictionary_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_core)
target_compile_options(${PROJECT_NAME}_profile_dictionary_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_profile_dictionary_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_profile_dictionary_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_profile_dictionary_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_profile_dictionary_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_profile_dictionary_unit)
add_dependencies(${PROJECT_NAME}_profile_dictionary_unit ${PROJECT_NAME}_core)
add_dependencies(run_tests ${PROJECT_NAME}_profile_dictionary_unit)
