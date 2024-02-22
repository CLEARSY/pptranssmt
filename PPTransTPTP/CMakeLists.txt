project(PPTRANSTPTP)

set(PPTRANSTPTP_HEADERS
        utils.h
        decomposition.h
        ppTransTPTP.h
        ppTransTPTPNonIncr.h
        )

set(PPTRANSTPTP_SOURCES
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

add_executable(ppTransTPTP ${PPTRANSTPTP_SOURCES})

target_link_libraries(ppTransTPTP
        -Bstatic POGLoader_LIB BAST_LIB
        -Bdynamic Qt5::Core Qt5::Xml)

add_executable(TPTPfilter TPTPfilter.cpp)
