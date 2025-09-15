cmake_minimum_required(VERSION 3.14)
project(MAPF_ALGORITHMS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

include_directories(
    ${pybind11_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
file(GLOB SOURCES "src/*.cpp")

pybind11_add_module(mapf bind.cpp ${SOURCES})
target_link_libraries(mapf PRIVATE pybind11::embed ${Python3_LIBRARIES})