cmake_minimum_required(VERSION 3.16)
project(UTGPIO VERSION 0.0.6)

option(BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread")

find_package(UTCore REQUIRED)

file(GLOB HEADERS include/ut/gpio/*.h)
file(GLOB SOURCES include/ut/gpio/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
add_library(UT::GPIO ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PUBLIC UT::Core)

set_target_properties(
    ${PROJECT_NAME} PROPERTIES 
        PUBLIC_HEADER "${HEADERS}"
        EXPORT_NAME GPIO
)

target_include_directories(
    ${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/ut/gpio>
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    PUBLIC_HEADER
        DESTINATION "include/ut/gpio"
    LIBRARY
        DESTINATION "lib"
)

install(
    EXPORT ${PROJECT_NAME}Targets
    NAMESPACE UT::
    FILE "${PROJECT_NAME}Targets.cmake"
    DESTINATION "lib/cmake/${PROJECT_NAME}"
)

# Generate local "${PROJECT_NAME}Targets.cmake"
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake)
export(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "lib/cmake/${PROJECT_NAME}"
)

if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()