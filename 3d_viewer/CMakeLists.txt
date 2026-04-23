cmake_minimum_required(VERSION 3.10)
project(3d_viewer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glfw3 QUIET)

if(NOT TARGET glfw)
  find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h PATHS /opt/homebrew/include /usr/local/include)
  find_library(GLFW_LIBRARY glfw PATHS /opt/homebrew/lib /usr/local/lib)
  if(NOT GLFW_INCLUDE_DIR OR NOT GLFW_LIBRARY)
    message(FATAL_ERROR "glfw3 not found. Install glfw to build 3d_viewer.")
  endif()
endif()

find_library(OPENGL_FRAMEWORK OpenGL REQUIRED)
find_library(COCOA_FRAMEWORK Cocoa REQUIRED)
find_library(IOKIT_FRAMEWORK IOKit REQUIRED)
find_library(COREVIDEO_FRAMEWORK CoreVideo REQUIRED)

add_executable(3d_viewer_gl 3d_viewer_gl.cc)

if(TARGET glfw)
  target_link_libraries(3d_viewer_gl PRIVATE glfw)
else()
  target_include_directories(3d_viewer_gl PRIVATE ${GLFW_INCLUDE_DIR})
  target_link_libraries(3d_viewer_gl PRIVATE ${GLFW_LIBRARY})
endif()

target_link_libraries(3d_viewer_gl
  PRIVATE
  ${OPENGL_FRAMEWORK}
  ${COCOA_FRAMEWORK}
  ${IOKIT_FRAMEWORK}
  ${COREVIDEO_FRAMEWORK}
)
