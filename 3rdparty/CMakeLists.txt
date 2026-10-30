# Extra CMake bindings for 3rdparty libraries that don't have them.

include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

set(thirdparty_ROOT ${CMAKE_SOURCE_DIR}/3rdparty)

set(SDL_ROOT ${thirdparty_ROOT}/SDL)
set(ImGui_ROOT ${thirdparty_ROOT}/ImGui)
set(Eigen_ROOT ${thirdparty_ROOT}/Eigen)
set(PicoJson_ROOT ${thirdparty_ROOT}/PicoJSON)
set(MinHook_ROOT ${thirdparty_ROOT}/MinHook)

find_package(Vulkan REQUIRED)

# Make sure SDL is built static, not shared.
set(SDL_STATIC ON)
set(SDL_SHARED OFF)
# GameInput.dll is not present on every Windows install; SDL must load it at runtime instead of importing it.
set(HAVE_GAMEINPUT_LIB 0 CACHE INTERNAL "" FORCE)

add_subdirectory(${SDL_ROOT} EXCLUDE_FROM_ALL)

add_library(vulkan INTERFACE)

target_link_libraries(vulkan INTERFACE ${Vulkan_LIBRARIES})
target_include_directories(vulkan INTERFACE ${Vulkan_INCLUDE_DIRS})

add_library(ImGui STATIC
    ${ImGui_ROOT}/backends/imgui_impl_vulkan.cpp
    ${ImGui_ROOT}/backends/imgui_impl_sdl3.cpp
	${CMAKE_SOURCE_DIR}/src/overlay/ImGui/imgui_impl_openvr.cpp
    ${ImGui_ROOT}/imgui.cpp
	${ImGui_ROOT}/imgui_draw.cpp
	${ImGui_ROOT}/imgui_tables.cpp
	${ImGui_ROOT}/imgui_widgets.cpp
)

target_include_directories(ImGui PUBLIC ${ImGui_ROOT} ${CMAKE_SOURCE_DIR}/src/overlay/ImGui)
target_link_libraries(ImGui PUBLIC SDL3::SDL3 vulkan OpenVR::API)

add_library(Eigen INTERFACE)

target_include_directories(Eigen INTERFACE ${Eigen_ROOT})

# TODO replace PicoJSON with different impl
add_library(PicoJSON INTERFACE)

target_include_directories(PicoJSON INTERFACE ${PicoJson_ROOT})

# Linux hooks the vtable directly (see Hooking.h), so MinHook is Windows only.
if (WIN32)
add_library(MinHook STATIC
    ${MinHook_ROOT}/src/hde/hde32.c
	${MinHook_ROOT}/src/hde/hde64.c
	${MinHook_ROOT}/src/buffer.c
	${MinHook_ROOT}/src/hook.c
	${MinHook_ROOT}/src/trampoline.c
)

target_include_directories(MinHook PUBLIC ${MinHook_ROOT}/include)
endif ()
