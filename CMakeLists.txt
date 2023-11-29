cmake_minimum_required(VERSION 3.16)
project(pog-translators)

# require the compiler to use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/qt@5/5.15.10_1" ${CMAKE_PREFIX_PATH})
find_package(Qt5 COMPONENTS Core REQUIRED)


# configure compilers
if(CMAKE_CXX_COMPILER_ID STREQUAL GNU OR CMAKE_CXX_COMPILER_ID MATCHES Clang$)
  add_compile_options(-Wall)
  add_compile_options(-fno-threadsafe-statics)
endif()

# compile command database
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set build type if not set
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

add_subdirectory(BAST)
add_subdirectory(POGLoader)
# add_subdirectory(POGPrinter)
add_subdirectory(PPTRANSSMT)
add_subdirectory(PPTransTPTP)
