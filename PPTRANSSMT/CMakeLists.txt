project(PPTRANSSMT)

set(PPTRANSSMT_HEADERS
    utils.h
    decomposition.h
    ppTrans.h
    ppTransIncr.h
    ppTransNonIncr.h
    )

set(PPTRANSSMT_SOURCES
    utils.cpp
    decomposition.cpp
    ppTrans.cpp
    ppTransIncr.cpp
    ppTransNonIncr.cpp
    ppTransSmt.cpp
    )

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${BAST_SOURCE_DIR}
    ${POGLoader_SOURCE_DIR}
    )

add_executable(ppTransSmt ${PPTRANSSMT_HEADERS} ${PPTRANSSMT_SOURCES})

target_link_libraries(ppTransSmt
    -Bstatic POGLoader_LIB BAST_LIB
    -Bdynamic Qt5::Core Qt5::Xml)
