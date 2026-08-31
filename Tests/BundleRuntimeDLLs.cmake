# Copies the runtime DLLs that the test executables depend on (SDL2, Freetype and the whole
# MinGW / HarfBuzz / GLib chain) next to the executables. This lets them be started from Explorer,
# cmd or PowerShell without C:\msys64\ucrt64\bin on PATH, and lets the directory be copied to a
# machine that has no MSYS2 installation.
#
# Invoked as a post-build step by Tests/CMakeLists.txt on Windows.

cmake_minimum_required(VERSION 3.16)

# CMake 4.0 normalizes paths before matching the exclude regexes; opt in explicitly so the
# script is quiet on new CMake and still works on older versions that lack the policy.
if(POLICY CMP0207)
    cmake_policy(SET CMP0207 NEW)
endif()

file(GLOB EXECUTABLES "${EXE_DIR}/*.exe")
if(NOT EXECUTABLES)
    return()
endif()

file(GET_RUNTIME_DEPENDENCIES
    EXECUTABLES ${EXECUTABLES}
    RESOLVED_DEPENDENCIES_VAR RESOLVED
    UNRESOLVED_DEPENDENCIES_VAR UNRESOLVED
    DIRECTORIES "${MINGW_BIN}"
    PRE_EXCLUDE_REGEXES "api-ms-win-.*" "ext-ms-.*"
    POST_EXCLUDE_REGEXES ".*[Ss][Yy][Ss][Tt][Ee][Mm]32.*" ".*[Ww][Ii][Nn][Ss][Xx][Ss].*"
)

foreach(dep IN LISTS RESOLVED)
    file(COPY "${dep}" DESTINATION "${EXE_DIR}")
endforeach()

list(LENGTH RESOLVED bundled)
message(STATUS "Bundled ${bundled} runtime DLL(s) into ${EXE_DIR}")

if(UNRESOLVED)
    message(WARNING "Could not resolve these runtime dependencies: ${UNRESOLVED}")
endif()
