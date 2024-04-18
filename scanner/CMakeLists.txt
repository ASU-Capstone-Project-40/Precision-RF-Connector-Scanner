cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0074 NEW)    # respect <PACKAGE>_ROOT variables in "find_package"
include(CMakePrintHelpers)

set(CMAKE_CXX_STANDARD 17)

# Add paths to check for cmake modules:
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(scanner VERSION 0.0)
include_directories("C:/Program Files/Basler/pylon 7/Development/include" "$ENV{BOOST_ROOT}")
find_package(pylon REQUIRED)
find_package(pylonDataProcessing REQUIRED)

add_executable(scanner
    src/scanner.cpp
    include/fixedConnector.precipe
    include/ResultData.h
    include/OutputObserver.h
)

get_filename_component(FIXED_CONNECTOR_RECIPE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/fixedConnector.precipe" ABSOLUTE
)

get_filename_component(MOBILE_CONNECTOR_RECIPE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/mobileConnector.precipe" ABSOLUTE
)

target_compile_definitions(scanner
        PRIVATE
        FIXED_CONNECTOR_RECIPE="${FIXED_CONNECTOR_RECIPE}"
        MOBILE_CONNECTOR_RECIPE="${MOBILE_CONNECTOR_RECIPE}"
        )

target_link_libraries(scanner
    PRIVATE
    pylon
    pylon::DataProcessing
)

install( TARGETS scanner )