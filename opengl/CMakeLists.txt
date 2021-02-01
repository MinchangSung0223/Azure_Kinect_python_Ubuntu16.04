# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

find_package(OpenGL REQUIRED)
include_directories(
    .
    ${OPENGL_INCLUDE_DIRS}
)

add_library(viewer_opengl_shared SHARED
    main.cpp
    texture.cpp
    viewerwindow.cpp
    transformation_helpers.cpp
    transformation_helpers.h)

target_link_libraries(viewer_opengl_shared PRIVATE
    k4a::k4a
    glfw::glfw
    imgui::imgui
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo
    ${OPENGL_LIBRARIES})

