cmake_minimum_required(VERSION 3.15)
project(ais2203-rvr)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/develop/conan.cmake"
            "${CMAKE_BINARY_DIR}/conan.cmake"
            TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

set(DEPS
        opencv/4.5.5
        zmqpp/4.2.0
        boost/1.80.0
        zlib/1.2.12                         # Specify to resolve opencv and boost dependency conflict
        nlohmann_json/3.11.2
        )

conan_cmake_autodetect(settings)
conan_cmake_run(
        REQUIRES
        ${DEPS}
        GENERATORS cmake
        BUILD missing
        BASIC_SETUP CMAKE_TARGETS
        OPTIONS opencv:contrib=True         # Needed for Aruco module
        SETTINGS ${settings}
)

link_libraries(
        CONAN_PKG::opencv
        CONAN_PKG::zmqpp
        CONAN_PKG::boost
        CONAN_PKG::nlohmann_json
)

include_directories(include/AIS2203-rvr)

add_library(imagestream ImageStream.cpp)
add_library(control Control.cpp)
target_link_libraries(control imagestream)

add_subdirectory(examples)

file(COPY resources/ DESTINATION ${CMAKE_BINARY_DIR}/bin/resources)

add_executable(main_gui gui.cpp)
target_link_libraries(main_gui control)
