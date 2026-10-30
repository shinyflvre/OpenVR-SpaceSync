# Note: this bitness check does not work for universal builds.
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_openvr_bitness 64)
else ()
    set(_openvr_bitness 32)
endif ()

set(_openvr_platform_base)
if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(_openvr_platform_base osx)
    # SteamVR only supports 32-bit on OS X
    set(OpenVR_PLATFORM osx32)
else ()
    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
        set(_openvr_platform_base linux)
    elseif (WIN32)
        set(_openvr_platform_base win)
    endif ()
    set(OpenVR_PLATFORM ${_openvr_platform_base}${_openvr_bitness})
endif ()

find_path(OpenVR_INCLUDE_DIR
    NAMES
        openvr.h
        openvr_driver.h
    PATH_SUFFIXES
        headers
    DOC
        "OpenVR include directory")
mark_as_advanced(OpenVR_INCLUDE_DIR)

find_library(OpenVR_LIBRARY
    NAMES
        openvr_api
    PATHS
        ${OpenVR_ROOT}
    PATH_SUFFIXES
        "lib/${OpenVR_PLATFORM}"
    DOC
        "OpenVR API library")
mark_as_advanced(OpenVR_LIBRARY)

find_file(OpenVR_BINARY
    NAMES
        openvr_api.dll
    PATHS
        ${OpenVR_ROOT}
    PATH_SUFFIXES
        "bin/${OpenVR_PLATFORM}"
    DOC
        "OpenVR API library binary")
mark_as_advanced(OpenVR_BINARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenVR
        REQUIRED_VARS OpenVR_LIBRARY OpenVR_INCLUDE_DIR)

if (OpenVR_FOUND)
    set(OpenVR_INCLUDE_DIRS "${OpenVR_INCLUDE_DIR}")
    set(OpenVR_LIBRARIES "${OpenVR_LIBRARY}")

    # OpenVR API requires the client DLL.
    if (NOT TARGET OpenVR::API)
        add_library(OpenVR::API SHARED IMPORTED)
        if (WIN32)
            set_target_properties(OpenVR::API
                PROPERTIES
                    IMPORTED_IMPLIB "${OpenVR_LIBRARY}"
                    IMPORTED_LOCATION "${OpenVR_BINARY}"
                    INTERFACE_INCLUDE_DIRECTORIES "${OpenVR_INCLUDE_DIR}")
        else ()
            # Linux: the .so is both the link library and the runtime binary.
            set_target_properties(OpenVR::API
                PROPERTIES
                    IMPORTED_LOCATION "${OpenVR_LIBRARY}"
                    IMPORTED_NO_SONAME TRUE
                    INTERFACE_INCLUDE_DIRECTORIES "${OpenVR_INCLUDE_DIR}")
        endif ()
    endif ()

    # OpenVR Driver API is header only.
    if (NOT TARGET OpenVR::Driver)
        add_library(OpenVR::Driver INTERFACE IMPORTED)
        set_target_properties(OpenVR::Driver
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${OpenVR_INCLUDE_DIR}")
    endif ()
endif ()

unset(_openvr_bitness)
unset(_openvr_platform_base)
