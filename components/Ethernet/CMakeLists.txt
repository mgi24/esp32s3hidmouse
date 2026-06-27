# CMakeFiles.txt inside "SparkFun_u-blox_GNSS" folder
cmake_minimum_required(VERSION 3.5)
file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c")
idf_component_register(SRCS ${SRCS}
                    INCLUDE_DIRS "src/."
                    REQUIRES "arduino-esp32" # Library requires Arduino
                    )