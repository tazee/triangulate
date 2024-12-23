cmake_minimum_required(VERSION 3.10)

project(triangulate VERSION 1.0)
include(FetchContent)

#set(LXSDK_PATH "../LXSDK-000025")
#set(CDT_PATH "../CDT")
set(BOOST_PATH "../boost_1_87_0")

if (NOT LXSDK_PATH)
    FetchContent_Declare( 
        lxsdk
        URL https://thefoundry.s3.us-east-1.amazonaws.com/products/modo/16.1v5/lxsdk16.1v5_692025.zip
    )
    FetchContent_Populate(lxsdk)
    message(STATUS "FetchContent lxsdk directory: ${lxsdk_SOURCE_DIR}")
    set(LXSDK_PATH ${lxsdk_SOURCE_DIR})
endif()

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

if (NOT CDT_PATH)
    FetchContent_Declare( 
        cdt
        GIT_REPOSITORY https://github.com/artem-ogre/CDT.git
        GIT_TAG        1.4.0
    )
    FetchContent_Populate(cdt)
    message(STATUS "FetchContent cdt directory: ${cdt_SOURCE_DIR}")
    set(CDT_PATH ${cdt_SOURCE_DIR})
endif()

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
        ${BOOST_PATH}
        ${CDT_PATH}/CDT/include
        ${CDT_PATH}/CDT/extras
)

# Set your CDT libraray
target_link_libraries(triangulate
    PUBLIC
	    lxsdk
)

# Install plugins and configs into kit directry
install(
    FILES
        "index.cfg"
        "index.html"
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
)
install(
    DIRECTORY
        "Icons"
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
)
install(
    DIRECTORY
        "images"
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
)
install(
    TARGETS
        triangulate
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}/extra
)
