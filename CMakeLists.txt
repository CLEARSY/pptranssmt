cmake_minimum_required(VERSION 3.16)
project(pog-translators)

# require the compiler to use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt5 COMPONENTS Core Xml REQUIRED)


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

if(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
  set(PROJECT_IS_TOP_LEVEL TRUE)
else()
  set(PROJECT_IS_TOP_LEVEL FALSE)
endif()

if(PROJECT_IS_TOP_LEVEL)
  foreach(submodule
      "${PROJECT_SOURCE_DIR}/BAST")
    if(NOT EXISTS "${submodule}/CMakeLists.txt")
      message(FATAL_ERROR "Submodule ${submodule} is missing: update submodules and try again.")
      message(FATAL_ERROR "Command : git submodule update --init --recursive.")
    endif()
  endforeach(submodule)
  add_subdirectory(BAST)
endif()

add_subdirectory(POGLoader)
add_subdirectory(PPTRANSSMT)
add_subdirectory(PPTransTPTP)
