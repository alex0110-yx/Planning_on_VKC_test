find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(Boost REQUIRED)
find_package(
  Qt5
  COMPONENTS Core
             Gui
             Widgets
             Svg
  REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

qt5_wrap_cpp(COMMON_WIDGETS_headers_MOC include/tesseract_qt/common/cartesian_editor_widget.h)

qt5_wrap_ui(COMMON_WIDGETS_headers_UI include/tesseract_qt/common/cartesian_editor_widget.ui)

qt5_add_resources(
  COMMON_WIDGETS_resources_RCC
  include/tesseract_qt/common/resources/tesseract_qt_resources.qrc
  include/tesseract_qt/common/resources/themes/dark/qdarkstyle_dark.qrc
  include/tesseract_qt/common/resources/themes/light/qdarkstyle_light.qrc)

add_library(
  ${PROJECT_NAME}_common SHARED
  ${COMMON_WIDGETS_headers_MOC}
  ${COMMON_WIDGETS_headers_UI}
  ${COMMON_WIDGETS_resources_RCC}
  src/calibration_info_standard_item.cpp
  src/cartesian_editor_widget.cpp
  src/collision_margin_data_standard_item.cpp
  src/contact_managers_plugin_info_standard_item.cpp
  src/kinematics_plugin_info_standard_item.cpp
  src/namespace_standard_item.cpp
  src/transform_standard_item.cpp
  src/pairs_collision_margin_data_standard_item.cpp
  src/plugin_info_standard_item.cpp
  src/position_standard_item.cpp
  src/quaternion_standard_item.cpp
  src/rgba_standard_item.cpp
  src/standard_item_utils.cpp
  src/plugin_info_container_standard_item.cpp
  src/entity_manager.cpp
  src/entity_container.cpp
  src/joint_trajectory_set.cpp
  src/svg_util.cpp
  src/theme_utils.cpp
  src/icon_utils.cpp
  src/type_standard_item.cpp
  src/vector_double_standard_item.cpp
  src/vector_string_standard_item.cpp
  src/joint_state_standard_item.cpp
  src/joint_trajectory_standard_item.cpp
  src/manipulator_info_standard_item.cpp
  src/utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}_common
  PUBLIC tesseract::tesseract_common
         tesseract::tesseract_environment
         tesseract::tesseract_command_language
         Boost::boost
         Qt5::Core
         Qt5::Gui
         Qt5::Widgets
         Qt5::Svg)
target_include_directories(
  ${PROJECT_NAME}_common PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/common>" "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install
install_targets(TARGETS ${PROJECT_NAME}_common)
