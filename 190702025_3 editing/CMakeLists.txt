cmake_minimum_required(VERSION 3.15)
project(MeshEditor_Dev)

set(CMAKE_CXX_STANDARD 17)

find_package(glfw3 CONFIG REQUIRED)

find_package(glad CONFIG REQUIRED)

find_package(imgui CONFIG REQUIRED)

find_package(assimp CONFIG REQUIRED)
    
find_package(OpenGL REQUIRED )
 
find_package(CGAL CONFIG REQUIRED Core)

find_package(glm CONFIG REQUIRED)

find_package(Eigen3 CONFIG REQUIRED)

find_package(libigl CONFIG REQUIRED)

find_package(GLEW  CONFIG REQUIRED)

add_executable(viewer 
Header.h 

main.cpp


)

target_link_libraries(viewer PRIVATE 

igl::igl_core igl::igl_glfw igl::igl_imgui igl::igl_opengl igl_copyleft::igl_copyleft_core
    CGAL::CGAL
    CGAL::CGAL_Core
    glfw          
    glad::glad
    imgui::imgui
    assimp::assimp
    OpenGL::GL
    glm::glm
    Eigen3::Eigen
    GLEW::GLEW
)