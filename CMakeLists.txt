cmake_minimum_required(VERSION 3.10)
project(DynamicMazeSolver VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(include)
find_package(OpenGL REQUIRED)

if(WIN32)
    set(PLATFORM_LIBS opengl32 gdi32 user32 kernel32)
endif()

set(SOURCES
    src/main.cpp
    src/Config.cpp
    src/Grid.cpp
    src/Pathfinder.cpp
    src/AStarPathfinder.cpp
    src/DynamicAStarPathfinder.cpp
    src/Agent.cpp
    src/SimpleRenderer.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${PLATFORM_LIBS})

if(MINGW)
    target_link_options(${PROJECT_NAME} PRIVATE -static-libgcc -static-libstdc++ -mwindows)
endif()
