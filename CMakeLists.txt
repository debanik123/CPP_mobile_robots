# only for cmake --version >= 3.5.1
cmake_minimum_required(VERSION 3.5.1)
project(CPP_mobile_robots)

# set the C++14 standard
set(CMAKE_CXX_STANDARD 14)

# I../includes
# include_directories(includes)

# puts all .cpp files inside src to the SOURCES variable
# file(GLOB SOURCES src/*.cpp)

# compiles the files defined by SOURCES to generante the executable defined by EXEC
add_executable(basic_vector src/basic_vector.cpp)
add_executable(bfs_opt src/bfs_opt.cpp)
add_executable(bfs src/bfs.cpp)
add_executable(A_star_algo_opt src/A_star_algo_opt.cpp)
add_executable(basic_lambda src/basic_lambda.cpp)
