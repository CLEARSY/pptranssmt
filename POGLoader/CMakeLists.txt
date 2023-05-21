cmake_minimum_required(VERSION 3.16)
project(POGLoader)

set(POG_HEADERS
    pog.h
)

set(POG_SRC
    pog.cpp
)

message(STATUS "BAST_SOURCE_DIR ${BAST_SOURCE_DIR}")
message(STATUS "LIBBAST_SOURCE_DIR ${LIBBAST_SOURCE_DIR}")

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
	    ${LIBBAST_SOURCE_DIR}
)

add_library(POGLoader
    ${POG_SRC}
    ${POG_HEADERS}
)

target_link_libraries(POGLoader BAST)

