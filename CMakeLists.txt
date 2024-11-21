cmake_minimum_required(VERSION 3.10)

find_package(CDT CONFIG REQUIRED)

project(triangulate VERSION 1.0)

set(LXSDK_PATH "../LXSDK-000025")

# Get all source and headers for lxsdk
file(GLOB LXSDK_SOURCES ${LXSDK_PATH}/common/*.cpp)
file(GLOB LXSDK_HEADERS ${LXSDK_PATH}/include/*.h?)

add_library(lxsdk
    STATIC
        ${LXSDK_SOURCES}
)

set_target_properties(lxsdk 
    PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY lib
        POSITION_INDEPENDENT_CODE ON
)

target_compile_definitions(lxsdk
    PRIVATE
        GL_SILENCE_DEPRECATION
)

target_compile_features(lxsdk
    PRIVATE
        cxx_std_17
)

target_include_directories(lxsdk
    PRIVATE
        ${LXSDK_PATH}/include
)

add_library(triangulate
    SHARED
        "source/triangulate_initialize.cpp"
        "source/triangulate_command.cpp"
        "source/triangulate_meshop.cpp"
        "source/triangulate_helper.cpp"
)

target_compile_features(triangulate
    PRIVATE
        cxx_std_17
)

# Set your CDT include directry
target_include_directories(triangulate 
    PUBLIC
        ${LXSDK_PATH}/include
)

# Set your CDT libraray
target_link_libraries(triangulate
    PUBLIC
	    lxsdk
    PRIVATE
        CDT::CDT
)

# Install plugins and configs into kit directry
install(
    FILES
        "index.cfg"
        "index.html"
    DESTINATION
        "."
)
install(
    DIRECTORY
        "Icons"
    DESTINATION
        "."
)
install(
    DIRECTORY
        "images"
    DESTINATION
        "."
)
install(
    TARGETS
        triangulate
    DESTINATION
        "extra"
)
