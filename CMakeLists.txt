cmake_minimum_required(VERSION 3.18)

project(roboflex_dvs)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# -------------------- 
# Resolve dependencies

include(FetchContent)

# Locate the installed roboflex_core and its dependencies

# We're gonna use FetchContent to get and build this, instead of find_package
# find_package(roboflex_core REQUIRED)

# download and build roboflex_core
FetchContent_Declare(roboflex_core
    GIT_REPOSITORY https://github.com/flexrobotics/roboflex.git
    GIT_TAG        main
)
set(BUILD_ROBOFLEX_PYTHON_EXT OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(roboflex_core)

FetchContent_Declare(roboflex_visualization
    GIT_REPOSITORY https://github.com/flexrobotics/roboflex_visualization.git
    GIT_TAG        main
)
#set(BUILD_ROBOFLEX_PYTHON_EXT OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(roboflex_visualization)

# download and build cyusb_linux
FetchContent_Declare(cyusb_linux
    GIT_REPOSITORY https://github.com/cpboyd/cyusb_linux
    GIT_TAG        master
)

# We want to be able to use ${flatbuffers_SOURCE_DIR}
FetchContent_GetProperties(cyusb_linux)
if(NOT cyusb_linux_POPULATED)
    FetchContent_Populate(cyusb_linux)
endif()

add_library(cyusb STATIC 
    ${cyusb_linux_SOURCE_DIR}/lib/libcyusb.cpp
)
target_include_directories(cyusb PUBLIC
    $<BUILD_INTERFACE:${cyusb_linux_SOURCE_DIR}/include>
)
set_property(TARGET cyusb PROPERTY 
    POSITION_INDEPENDENT_CODE ON
)


# -------------------- 
# Define the library

add_library(roboflex_dvs STATIC
    src/dvs.cpp
    include/roboflex_dvs/dvs.h
)

# Set some properties on our library
set_property(TARGET roboflex_dvs PROPERTY 
    POSITION_INDEPENDENT_CODE ON
)

# Include directories when we compile our library
target_include_directories(roboflex_dvs PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
)

# Link against the necessary libraries
target_link_libraries(roboflex_dvs PUBLIC 
    roboflex_core
    usb-1.0
    cyusb
)

target_compile_definitions(roboflex_dvs PRIVATE 
    # gotta raise the limit, which by default is 128k
    EIGEN_STACK_ALLOCATION_LIMIT=153600
)


# -------------------- 
# Examples

# stream_from_dvs example
add_executable(stream_from_dvs_cpp examples/stream_from_dvs_cpp.cpp)
target_link_libraries(stream_from_dvs_cpp PRIVATE 
    roboflex_core 
    roboflex_dvs
    roboflex_visualization
)
target_compile_definitions(stream_from_dvs_cpp PRIVATE 
    EIGEN_STACK_ALLOCATION_LIMIT=153600
)

add_executable(dvsconf examples/dvsconf.cpp)
target_link_libraries(dvsconf cyusb usb-1.0 pthread)


# -------------------- 
# install

# If you need to install the roboflex_dvs library
install(TARGETS roboflex_dvs
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/roboflex_dvs
    DESTINATION include
)


# --------------------
# build python bindings

add_subdirectory(python)
