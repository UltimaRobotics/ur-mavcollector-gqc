cmake_minimum_required(VERSION 3.20)
project(MAVLinkDataCollector VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MAVLink libraries are in thirdparty folder
set(MAVLINK_V1_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/c_library_v1)
set(MAVLINK_V2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/c_library_v2)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${MAVLINK_V1_DIR})
include_directories(${MAVLINK_V2_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

# Source files
set(SOURCES
    src/main.cpp
    src/Fact.cpp
    src/FactMetaData.cpp
    src/FactGroup.cpp
    src/MAVLinkUdpConnection.cpp
    src/Vehicle.cpp
    src/ParameterManager.cpp
    src/VehicleFactGroup.cpp
    src/VehicleGPSFactGroup.cpp
    src/VehicleGPS2FactGroup.cpp
    src/VehicleBatteryFactGroup.cpp
    src/VehicleSystemStatusFactGroup.cpp
    src/VehicleRCFactGroup.cpp
    src/VehicleVibrationFactGroup.cpp
    src/VehicleTemperatureFactGroup.cpp
    src/VehicleEstimatorStatusFactGroup.cpp
    src/VehicleWindFactGroup.cpp
)

# Header files
set(HEADERS
    include/Fact.h
    include/FactMetaData.h
    include/FactGroup.h
    include/MAVLinkUdpConnection.h
    include/Vehicle.h
    include/ParameterManager.h
    include/VehicleFactGroup.h
    include/VehicleGPSFactGroup.h
    include/VehicleGPS2FactGroup.h
    include/VehicleBatteryFactGroup.h
    include/VehicleSystemStatusFactGroup.h
    include/VehicleRCFactGroup.h
    include/VehicleVibrationFactGroup.h
    include/VehicleTemperatureFactGroup.h
    include/VehicleEstimatorStatusFactGroup.h
    include/VehicleWindFactGroup.h
)

# Create executable
add_executable(MAVLinkDataCollector ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(MAVLinkDataCollector 
    pthread
)

# Compiler flags
target_compile_options(MAVLinkDataCollector PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -O2
)

# Set output directory
set_target_properties(MAVLinkDataCollector PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Install target
install(TARGETS MAVLinkDataCollector
    RUNTIME DESTINATION bin
)