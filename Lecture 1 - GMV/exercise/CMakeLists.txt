project(app)
cmake_minimum_required(VERSION 3.8)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7at2hf_neon)

#If defined SDKTARGETSYSROOT, then change CMAKE_FIND_ROOT_PATH
set(SDK_SYSROOT_DEFINED $ENV{SDKTARGETSYSROOT})
if(SDK_SYSROOT_DEFINED)
set(CMAKE_FIND_ROOT_PATH $ENV{SDKTARGETSYSROOT})
endif()

#Compiler flags
set(CMAKE_CXX_FLAGS "-std=c++14 -fstack-protector-all  -Wall ${CMAKE_CXX_FLAGS}")

find_package(Boost 1.67 COMPONENTS system thread chrono filesystem program_options date_time timer regex REQUIRED)


#Project files
file(GLOB_RECURSE app_SOURCES "src/*.c*")
file(GLOB_RECURSE app_HEADERS "src/*.h")

set (app_INCLUDE_DIRS "src")
foreach (_headerFile ${app_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND app_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES app_INCLUDE_DIRS)


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${app_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#Executable
add_executable(app
   ${app_SOURCES}
)

#Link
target_link_libraries(app
   rt m pthread 
   ${Boost_LIBRARIES}
   dpylogconfig
   dpymcu
   dpygnss
   dpyutils
   dpygeofencing
   dpysms
)








 



