cmake_minimum_required(VERSION 2.8.9)
project(blue_demo)

add_subdirectory(duck-detector)
add_subdirectory(mood-module)
add_subdirectory(movement-control)
add_subdirectory(slide-module)
add_subdirectory(state-machine)

message("************************************* CARDTRACKER NEEDS SEPARATE BUILD ***************************")
