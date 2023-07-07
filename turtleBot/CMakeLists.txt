# Copyright (c) Yunus M. Agamawi and Anil Vithala Rao.  All Rights Reserved

# ==============================================================================
# Build instructions
# ==============================================================================

# Target name
set(target turtleBot)

# Exit here if required dependencies are not met
message(STATUS "Example ${target}")

# ------------------------------------------------------------------------------
# Sources
# ------------------------------------------------------------------------------

set(include_path "${CMAKE_CURRENT_SOURCE_DIR}")
set(source_path "${CMAKE_CURRENT_SOURCE_DIR}")

set(lib_sources
    cgpops_main.cpp
    cgpops_main.hpp
    cgpops_gov.cpp
    cgpops_gov.hpp
    cgpopsAuxDec.hpp
    cgpopsAuxExt.hpp
    )

set(exe_sources
    cgpops_wrapper.cpp
    )

# ------------------------------------------------------------------------------
# Include dirs
# ------------------------------------------------------------------------------

set(include_dirs ${include_path})

# ------------------------------------------------------------------------------
# Compile definitions / options
# ------------------------------------------------------------------------------

set(compile_definitions
    EXAMPLE_NAME=\"${target}Solution\"
    )
set(compile_options 
    cgpops_compiler_flags
    )

# ------------------------------------------------------------------------------
# Libraries
# ------------------------------------------------------------------------------

set(libraries
    cgpops::cgpops
    )
if(UNIX)
	list(APPEND libraries ${STD_FS_LIB})
endif()


# ------------------------------------------------------------------------------
# Create targets
# ------------------------------------------------------------------------------
cgpops_example(
    TARGET
    ${target}
    LIB_SOURCES
    ${lib_sources}
    EXE_SOURCES
    ${exe_sources}
    INCLUDE_DIRS
    ${include_dirs}
    LIBRARIES
    ${libraries}
    COMPILE_DEFINITIONS
    ${compile_definitions}
    COMPILE_OPTIONS
    ${compile_options}
)

