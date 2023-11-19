cmake_minimum_required(VERSION 3.25)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "/usr/local/Cellar/qt@5/5.15.8_2" ${CMAKE_PREFIX_PATH})

project(PPTRANSTPTP)

find_package(Qt5Core)
find_package(Qt5Xml)

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
        ${BAST_SOURCE_DIR}
        ${POGLoader_SOURCE_DIR}
)

add_executable(ppTransTPTP
        ${PPTRANSTPTP_SRC}
        )

target_link_libraries(ppTransTPTP POGLoader BAST Qt5::Core Qt5::Xml)

add_executable(TPTPfilter
        TPTPfilter.cpp
        )