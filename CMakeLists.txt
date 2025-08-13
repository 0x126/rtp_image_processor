cmake_minimum_required(VERSION 3.10)
project(RealtimeImageProcessor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform detection
if(EXISTS "/etc/nv_tegra_release")
    set(JETSON_PLATFORM ON)
    add_definitions(-DJETSON_PLATFORM)
endif()

# Dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED 
    gstreamer-1.0>=1.14
    gstreamer-app-1.0>=1.14
)

# Source files
set(SOURCES
    src/image_processor.cpp
)

# Library creation
add_library(imageprocessor SHARED ${SOURCES})

target_include_directories(imageprocessor 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${GST_INCLUDE_DIRS}
)

target_link_libraries(imageprocessor
    PRIVATE
        ${GST_LIBRARIES}
)

target_compile_options(imageprocessor PRIVATE ${GST_CFLAGS_OTHER})

# Executable
add_executable(image_processor_demo examples/main.cpp)
target_link_libraries(image_processor_demo imageprocessor)

# Installation settings
install(TARGETS imageprocessor
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
)

# Tests
enable_testing()
# add_subdirectory(tests)  # Uncomment when tests directory has CMakeLists.txt