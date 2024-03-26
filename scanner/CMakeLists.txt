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
    include/scannerV1.precipe
    include/ResultData.h
    include/OutputObserver.h
)

get_filename_component(PYLON_RECIPE
    "${CMAKE_CURRENT_SOURCE_DIR}/scannerV1.precipe" ABSOLUTE
)

target_compile_definitions(scanner
        PRIVATE
        PYLON_RECIPE="${PYLON_RECIPE}"
        )

target_link_libraries(scanner
    PRIVATE
    pylon
    pylon::DataProcessing
)

install( TARGETS scanner )