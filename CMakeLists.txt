cmake_minimum_required (VERSION 3.13)

project(CANMolinaroAnalyzer)

add_definitions( -DLOGIC2 )

# enable generation of compile_commands.json, helpful for IDEs to locate include files.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# custom CMake Modules are located in the cmake directory.
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(ExternalAnalyzerSDK)

set(SOURCES
src/CANMolinaroAnalyzer.cpp
src/CANMolinaroAnalyzer.h
src/CANMolinaroAnalyzerResults.cpp
src/CANMolinaroAnalyzerResults.h
src/CANMolinaroAnalyzerSettings.cpp
src/CANMolinaroAnalyzerSettings.h
src/CANMolinaroSimulationDataGenerator.cpp
src/CANMolinaroSimulationDataGenerator.h
)

add_analyzer_plugin(${PROJECT_NAME} SOURCES ${SOURCES})
