find_package(tesseract_common REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

qt5_wrap_cpp(MANIPULATION_WIDGET_headers_MOC include/tesseract_qt/manipulation/manipulation_widget.h)
qt5_wrap_ui(MANIPULATION_WIDGET_headers_UI include/tesseract_qt/manipulation/manipulation_widget.ui)

add_library(${PROJECT_NAME}_manipulation SHARED ${MANIPULATION_WIDGET_headers_MOC} ${MANIPULATION_WIDGET_headers_UI}
                                                src/manipulation_widget.cpp)

target_link_libraries(
  ${PROJECT_NAME}_manipulation
  PUBLIC tesseract::tesseract_environment
         tesseract::tesseract_scene_graph
         tesseract::tesseract_common
         Qt5::Core
         Qt5::Widgets
         Qt5::Gui
         ${PROJECT_NAME}_common
         ${PROJECT_NAME}_scene_graph
         ${PROJECT_NAME}_joint_state_slider)
target_include_directories(
  ${PROJECT_NAME}_manipulation
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/manipulation>"
         "$<INSTALL_INTERFACE:include>")

# add_subdirectory(demo)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install
install_targets(TARGETS ${PROJECT_NAME}_manipulation)
