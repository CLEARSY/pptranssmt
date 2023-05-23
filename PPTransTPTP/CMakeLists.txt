cmake_minimum_required(VERSION 3.25)
set(CMAKE_CXX_STANDARD 17)

project(PPTRANSTPTP)

set(PPTRANSTPTP_HEADERS
        utils.h
        decomposition.h
        ppTransTPTP.h
        ppTransTPTPNonIncr.h
        )

set(PPTRANSTPTP_SRC
        utils.cpp
        decomposition.cpp
        ppTransTPTP.cpp
        ppTransTPTPNonIncr.cpp
        ppTransConsoleTPTP.cpp
        )

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${LIBBAST_SOURCE_DIR}
        ${POGLoader_SOURCE_DIR}
)

add_executable(ppTransTPTP
        ${PPTRANSTPTP_SRC}
        )

target_link_libraries(ppTransTPTP POGLoader BAST)
