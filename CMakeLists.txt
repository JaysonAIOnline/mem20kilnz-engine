cmake_minimum_required(VERSION 3.16)
project(kiln LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "kiln is Linux-only")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(kiln_core
  src/scene/scene.cpp
  src/scene/edit.cpp
  src/scene/rig.cpp
  src/ops/ops.cpp
  src/agent/agent.cpp
  src/io/gltf.cpp
  src/render/software.cpp
  src/rpc/rpc.cpp
  src/app.cpp
)
target_include_directories(kiln_core PUBLIC src third_party)

add_executable(kiln src/main.cpp)
target_link_libraries(kiln PRIVATE kiln_core)

option(KILN_GUI "Build Vulkan + Dear ImGui frontend" ON)

if(KILN_GUI)
  find_package(Vulkan)
  include(FetchContent)
  FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
  )
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_WAYLAND ON CACHE BOOL "" FORCE)
  FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8
  )
  if(Vulkan_FOUND)
    FetchContent_MakeAvailable(glfw imgui)
    add_library(imgui_lib
      ${imgui_SOURCE_DIR}/imgui.cpp
      ${imgui_SOURCE_DIR}/imgui_draw.cpp
      ${imgui_SOURCE_DIR}/imgui_tables.cpp
      ${imgui_SOURCE_DIR}/imgui_widgets.cpp
      ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
      ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
    )
    target_include_directories(imgui_lib PUBLIC
      ${imgui_SOURCE_DIR}
      ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(imgui_lib PUBLIC glfw Vulkan::Vulkan)
    target_sources(kiln PRIVATE src/gui/gui.cpp)
    target_compile_definitions(kiln PRIVATE KILN_GUI)
    target_link_libraries(kiln PRIVATE imgui_lib Vulkan::Vulkan glfw)
    find_program(GLSLC glslc)
    find_program(GLSLANG_VALIDATOR glslangValidator)
    set(SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
    set(SHADER_OUT ${CMAKE_CURRENT_BINARY_DIR}/shaders)
    file(MAKE_DIRECTORY ${SHADER_OUT})
    if(GLSLC)
      add_custom_command(OUTPUT ${SHADER_OUT}/mesh.vert.spv
        COMMAND ${GLSLC} -fshader-stage=vert ${SHADER_DIR}/mesh.vert -o ${SHADER_OUT}/mesh.vert.spv
        DEPENDS ${SHADER_DIR}/mesh.vert)
      add_custom_command(OUTPUT ${SHADER_OUT}/mesh.frag.spv
        COMMAND ${GLSLC} -fshader-stage=frag ${SHADER_DIR}/mesh.frag -o ${SHADER_OUT}/mesh.frag.spv
        DEPENDS ${SHADER_DIR}/mesh.frag)
      add_custom_target(kiln_shaders ALL DEPENDS ${SHADER_OUT}/mesh.vert.spv ${SHADER_OUT}/mesh.frag.spv)
      add_dependencies(kiln kiln_shaders)
    elseif(GLSLANG_VALIDATOR)
      add_custom_command(OUTPUT ${SHADER_OUT}/mesh.vert.spv
        COMMAND ${GLSLANG_VALIDATOR} -V ${SHADER_DIR}/mesh.vert -o ${SHADER_OUT}/mesh.vert.spv
        DEPENDS ${SHADER_DIR}/mesh.vert)
      add_custom_command(OUTPUT ${SHADER_OUT}/mesh.frag.spv
        COMMAND ${GLSLANG_VALIDATOR} -V ${SHADER_DIR}/mesh.frag -o ${SHADER_OUT}/mesh.frag.spv
        DEPENDS ${SHADER_DIR}/mesh.frag)
      add_custom_target(kiln_shaders ALL DEPENDS ${SHADER_OUT}/mesh.vert.spv ${SHADER_OUT}/mesh.frag.spv)
      add_dependencies(kiln kiln_shaders)
    else()
      message(WARNING "glslc/glslangValidator not found — compile shaders with scripts/compile_shaders.sh")
    endif()
  else()
    message(WARNING "Vulkan not found — building headless kiln only")
  endif()
endif()

install(TARGETS kiln RUNTIME DESTINATION bin)
install(DIRECTORY shaders DESTINATION share/kiln)
