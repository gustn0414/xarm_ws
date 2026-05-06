#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "xarm_1s_hardware::xarm_1s_hardware" for configuration ""
set_property(TARGET xarm_1s_hardware::xarm_1s_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(xarm_1s_hardware::xarm_1s_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libxarm_1s_hardware.so"
  IMPORTED_SONAME_NOCONFIG "libxarm_1s_hardware.so"
  )

list(APPEND _cmake_import_check_targets xarm_1s_hardware::xarm_1s_hardware )
list(APPEND _cmake_import_check_files_for_xarm_1s_hardware::xarm_1s_hardware "${_IMPORT_PREFIX}/lib/libxarm_1s_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
