#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rslidar_msg::rslidar_msg__rosidl_typesupport_cpp" for configuration ""
set_property(TARGET rslidar_msg::rslidar_msg__rosidl_typesupport_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rslidar_msg::rslidar_msg__rosidl_typesupport_cpp PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_cpp::rosidl_typesupport_cpp;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librslidar_msg__rosidl_typesupport_cpp.so"
  IMPORTED_SONAME_NOCONFIG "librslidar_msg__rosidl_typesupport_cpp.so"
  )

list(APPEND _cmake_import_check_targets rslidar_msg::rslidar_msg__rosidl_typesupport_cpp )
list(APPEND _cmake_import_check_files_for_rslidar_msg::rslidar_msg__rosidl_typesupport_cpp "${_IMPORT_PREFIX}/lib/librslidar_msg__rosidl_typesupport_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
