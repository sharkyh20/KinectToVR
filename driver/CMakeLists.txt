cmake_minimum_required(VERSION 3.5)
project(OpenVR_Examples CXX)


set(OPENVR_LIB_DIR ${CMAKE_SOURCE_DIR}/openvr/lib/win64 CACHE STRING "OpenVR .lib directory")
set(OPENVR_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/openvr/headers CACHE STRING "OpenVR include directory")

set(Boost_INCLUDE_DIR "C:\\Users\\YOUR_USN\\Desktop\\boost_1_72_0")
set(Boost_LIBRARY_DIR "C:\\Users\\YOUR_USN\\Desktop\\boost_1_72_0\\lib64-msvc-14.2")

find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/D_UNICODE>")
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/DUNICODE>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/D_UNICODE>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/DUNICODE>")

add_subdirectory(KinectToVR)






