cmake_minimum_required(VERSION 3.20)
project(turtlelib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
# here: we find the eigen library as per the instruction
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Doxygen)
find_package(Catch2 REQUIRED)

add_library(turtlelib src/geometry2d.cpp src/se2d.cpp src/svg.cpp src/diff_drive.cpp)
target_compile_features(turtlelib PUBLIC cxx_std_20)

add_executable(converter app/converter.cpp)
target_link_libraries(converter turtlelib)

add_executable(test_geometry2d test/test_geometry2d.cpp)
target_link_libraries(test_geometry2d turtlelib Catch2::Catch2WithMain)

add_executable(test_se2d test/test_se2d.cpp)
target_link_libraries(test_se2d turtlelib Catch2::Catch2WithMain)

add_executable(test_svg test/test_svg.cpp)
target_link_libraries(test_svg turtlelib Catch2::Catch2WithMain)

add_executable(test_diff_drive test/test_diff_drive.cpp)
target_link_libraries(test_diff_drive turtlelib Catch2::Catch2WithMain)

add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main turtlelib)

target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

install(DIRECTORY include/ DESTINATION include)

# Create the targets
install(TARGETS turtlelib EXPORT turtlelib-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the CMake config file so other packages can find_package(turtlelib)
install(EXPORT turtlelib-targets
  FILE turtlelibConfig.cmake
  NAMESPACE turtlelib::
  DESTINATION lib/cmake/turtlelib
)

##################Citation[1]###################
add_library(turtlelib::turtlelib ALIAS turtlelib)
################################################

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)


# build the documentation
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()