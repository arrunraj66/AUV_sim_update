cmake_minimum_required(VERSION 3.16)
project(auvsim CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(AUVSIM_ENABLE_SANITIZERS "Enable sanitizer instrumentation for a dedicated debug build" OFF)

if(AUVSIM_ENABLE_SANITIZERS)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(SANITIZERS "address,undefined")
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(SANITIZERS "${SANITIZERS},leak")
    endif()
    add_compile_options(-O1 -g -fno-omit-frame-pointer -fsanitize=${SANITIZERS})
    add_link_options(-O1 -g -fno-omit-frame-pointer -fsanitize=${SANITIZERS})
    message(STATUS "Auvsim sanitizers enabled: ${SANITIZERS}")
  else()
    message(WARNING "Sanitizers requested but compiler does not support them.")
  endif()
endif()

find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

# Dear ImGui (the GUI library) is downloaded automatically the first time.
include(FetchContent)
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.90.9)
FetchContent_MakeAvailable(imgui)

add_library(imgui_lib STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui_lib PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${SDL2_INCLUDE_DIRS})

add_executable(auvsim src/main.cpp)
target_include_directories(auvsim PRIVATE src)
target_link_libraries(auvsim imgui_lib ${SDL2_LIBRARIES} OpenGL::GL)
# Add the physics library
add_executable(physics_test src/physics_test.cpp)
target_include_directories(physics_test PRIVATE src)

add_executable(water_test src/water_test.cpp)
target_compile_features(water_test PRIVATE cxx_std_17)

# After every successful build, save a timestamped copy of the sources to snapshots/.
add_custom_target(snapshot ALL
  COMMAND ${CMAKE_SOURCE_DIR}/scripts/snapshot.sh ${CMAKE_SOURCE_DIR}
  DEPENDS auvsim physics_test water_test
  COMMENT "Snapshotting sources")
add_executable(auvdesigner src/designer_main.cpp)
target_include_directories(auvdesigner PRIVATE src)
target_link_libraries(auvdesigner imgui_lib ${SDL2_LIBRARIES} OpenGL::GL)

add_executable(design_spec_test src/design_spec_test.cpp)
target_include_directories(design_spec_test PRIVATE src)

add_executable(design_manoeuvre_test src/design_manoeuvre_test.cpp)
target_include_directories(design_manoeuvre_test PRIVATE src)

add_executable(real_models_test src/real_models_test.cpp)
target_include_directories(real_models_test PRIVATE src)

add_executable(design_integration_test src/design_integration_test.cpp)
target_include_directories(design_integration_test PRIVATE src)

add_executable(designer_gui_test src/designer_gui_test.cpp)
target_include_directories(designer_gui_test PRIVATE src)
target_link_libraries(designer_gui_test imgui_lib ${SDL2_LIBRARIES} OpenGL::GL)

add_executable(propwash_test src/propwash_test.cpp)
target_include_directories(propwash_test PRIVATE src)
target_link_libraries(propwash_test imgui_lib ${SDL2_LIBRARIES} OpenGL::GL)
