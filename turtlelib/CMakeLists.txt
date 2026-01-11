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
find_package(ament_cmake REQUIRED)
find_package(Doxygen)

add_library(turtlelib src/libfile1.cpp src/libfile2.cpp)
target_compile_features(turtlelib PUBLIC cxx_std_20)


target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

install(DIRECTORY include DESTINATION include)

# Create the targets
install(TARGETS turtlelib EXPORT turtlelib-targets)

##################Citation[1]###################
add_library(turtlelib::turtlelib ALIAS turtlelib)

ament_export_targets(turtlelib-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
################################################

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# build the documentation
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()
ament_package()