# Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md
cmake_minimum_required(VERSION 3.20)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(SpaceSync)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The Vulkan validation layer adds extra reporting that also surfaces validation    
# issues originating from external sources (e.g. SteamVR). Defaults on.
option(ENABLE_VULKAN_VALIDATION "Enable Vulkan validation layers" OFF)

set(OpenVR_ROOT ${CMAKE_SOURCE_DIR}/3rdparty/OpenVR)

find_package(OpenVR REQUIRED)

# Include rest of the CMake bindings
add_subdirectory(3rdparty)

message(STATUS "ENABLE_VULKAN_VALIDATION = ${ENABLE_VULKAN_VALIDATION}")

add_executable(SpaceSync
    "src/overlay/Main.cpp"
    "src/overlay/VulkanRenderer.cpp"
    "src/overlay/ImGuiWindow.cpp"
    "src/overlay/Calibration.cpp"
    "src/overlay/Configuration.cpp"
    "src/overlay/IPCClient.cpp"
    "src/overlay/UserInterface.cpp"
    "src/overlay/Theme.cpp"
    "src/overlay/TrackingPreview.cpp"
    "src/overlay/Lighthouse.cpp"
    "src/overlay/Sound.cpp"
    "src/overlay/EmbeddedFiles.cpp"
)

target_compile_features(SpaceSync PUBLIC cxx_std_20)

target_include_directories(SpaceSync
    PRIVATE
        "${CMAKE_SOURCE_DIR}/include/overlay"
        "${CMAKE_SOURCE_DIR}/include/shared"
)

target_compile_definitions(SpaceSync
    PRIVATE
        _HAS_DEPRECATED_NEGATORS=1
        _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
)

if (ENABLE_VULKAN_VALIDATION)
    target_compile_definitions(SpaceSync PRIVATE ENABLE_VULKAN_VALIDATION)
endif()

if (MSVC)
    target_compile_options(SpaceSync PRIVATE /bigobj)
endif()

# Windows executable icon
if (WIN32)
    target_sources(SpaceSync PRIVATE "resources/winicon.rc")
endif()


target_link_libraries(SpaceSync
    PRIVATE
        OpenVR::API
        ImGui
        Eigen
        PicoJSON
        winmm
        windowsapp
)

add_library(driver_spacesync SHARED
    "src/driver/Main.cpp"
    "src/driver/ServerTrackedDeviceProvider.cpp"
    "src/driver/IPCServer.cpp"
    "src/driver/InterfaceHookInjector.cpp"
    "src/driver/Hooking.cpp"
    "src/driver/Logging.cpp"
    "src/driver/KalmanFilter.cpp"
)

target_compile_features(driver_spacesync PRIVATE cxx_std_20)

target_include_directories(driver_spacesync PRIVATE
    "${CMAKE_SOURCE_DIR}/include/driver"
    "${CMAKE_SOURCE_DIR}/include/shared"
)

target_link_libraries(driver_spacesync PRIVATE
    OpenVR::Driver
    MinHook
)

set_property(TARGET driver_spacesync PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
set_target_properties(driver_spacesync PROPERTIES PREFIX "")