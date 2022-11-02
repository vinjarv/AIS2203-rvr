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
        zlib/1.2.12
        )



conan_cmake_run(
        REQUIRES
        ${DEPS}
        GENERATORS cmake
        BUILD missing
        BASIC_SETUP CMAKE_TARGETS
        OPTIONS opencv:contrib=True         # Needed for Aruco module
)

link_libraries(
        CONAN_PKG::opencv
        CONAN_PKG::zmqpp
        CONAN_PKG::boost
)

add_subdirectory(examples)


file(COPY resources/ DESTINATION ${CMAKE_BINARY_DIR}/bin/resources)