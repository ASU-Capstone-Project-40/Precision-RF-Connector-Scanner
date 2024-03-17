cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0074 NEW)    # respect <PACKAGE>_ROOT variables in "find_package"
include(CMakePrintHelpers)

set(CMAKE_CXX_STANDARD 17)

# Add paths to check for cmake modules:
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(image_reconSample VERSION 0.0)
include_directories("C:/Program Files/Basler/pylon 7/Development/include")
find_package(pylon REQUIRED)
find_package(pylonDataProcessing REQUIRED)

add_executable(image_reconSample
    RecipeSample.cpp
    image_recon.precipe
    ResultData.h
    OutputObserver.h
)

get_filename_component(PYLON_RECIPE
    "${CMAKE_CURRENT_SOURCE_DIR}/image_recon.precipe" ABSOLUTE
)

target_compile_definitions(image_reconSample
        PRIVATE
        PYLON_RECIPE="${PYLON_RECIPE}"
        )

target_link_libraries(image_reconSample
    PRIVATE
    pylon
    pylon::DataProcessing
)

install( TARGETS image_reconSample )