cmake_minimum_required(VERSION 3.11.0)

project("fluids" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glm CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(raylib CONFIG REQUIRED)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

message(STATUS "Add Dear ImGui")
FetchContent_Declare(
    ImGui
    GIT_REPOSITORY https://github.com/ocornut/imgui
    GIT_TAG cb16be3a3fc1f9cd146ae24d52b615f8a05fa93d
    GIT_PROGRESS TRUE
) # v1.90.9
FetchContent_MakeAvailable(ImGui)
FetchContent_GetProperties(ImGui SOURCE_DIR IMGUI_DIR)
add_library(
    imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp)
target_include_directories(imgui INTERFACE ${imgui_SOURCE_DIR})

message(STATUS "Add rlImGui")
FetchContent_Declare(
    rlImGui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui
    GIT_TAG d765c1ef3d37cf939f88aaa272a59a2713d654c9
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(rlImGui)
FetchContent_GetProperties(rlImGui SOURCE_DIR RLIMGUI_DIR)
add_library(rlimgui STATIC ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
target_link_libraries(rlimgui PRIVATE imgui raylib)
target_include_directories(rlimgui INTERFACE ${rlimgui_SOURCE_DIR})

add_executable(${PROJECT_NAME}_flip flip.cpp)
target_include_directories(${PROJECT_NAME}_flip PUBLIC include)
target_link_libraries(${PROJECT_NAME}_flip fmt::fmt raylib glm::glm imgui rlimgui)

add_executable(${PROJECT_NAME}_sph sph.cpp)
target_include_directories(${PROJECT_NAME}_sph PUBLIC include)
target_link_libraries(${PROJECT_NAME}_sph fmt::fmt raylib glm::glm imgui rlimgui)

add_executable(test_quadtree test_quadtree.cpp)
target_include_directories(test_quadtree PUBLIC include)
target_link_libraries(test_quadtree fmt::fmt raylib glm::glm imgui rlimgui)
