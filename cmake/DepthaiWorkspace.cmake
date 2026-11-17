# One existing SDK for every ROS package: /usr/local on the vehicle PC.
# Include this before loading ament dependencies which may import DepthAI.
include_guard(DIRECTORY)

set(F1TENTH_DEPTHAI_PREFIX "/usr/local" CACHE PATH
  "Prefix of the existing DepthAI C++ SDK (never built or installed here)")
get_filename_component(F1TENTH_DEPTHAI_PREFIX "${F1TENTH_DEPTHAI_PREFIX}" ABSOLUTE)

# Ignore an old depthai_DIR cache and the separate SDK in ROS underlays. Mixing the
# headers from one SDK with another SDK's shared library fails at runtime.
unset(_f1tenth_depthai_config_dir CACHE)
unset(_f1tenth_depthai_config_dir)
find_path(_f1tenth_depthai_config_dir
  NAMES depthaiConfig.cmake depthai-config.cmake
  PATHS
    "${F1TENTH_DEPTHAI_PREFIX}/lib/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/lib64/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/share/depthai/cmake"
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
if(NOT _f1tenth_depthai_config_dir)
  message(FATAL_ERROR
    "The existing DepthAI C++ SDK config was not found in ${F1TENTH_DEPTHAI_PREFIX}. "
    "Check the installed SDK location and F1TENTH_DEPTHAI_PREFIX; see DEPTHAI_SETUP.md. "
    "The copied depthai-core source tree is not configured or built, and the ROS "
    "DepthAI installation is not used as a fallback.")
endif()

set(depthai_DIR "${_f1tenth_depthai_config_dir}" CACHE PATH
  "Existing DepthAI SDK used by all workspace packages" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${F1TENTH_DEPTHAI_PREFIX}")
find_package(depthai 3.6 CONFIG REQUIRED
  PATHS "${depthai_DIR}" NO_DEFAULT_PATH)

get_target_property(_f1tenth_depthai_type depthai::core TYPE)
if(NOT _f1tenth_depthai_type STREQUAL "SHARED_LIBRARY")
  message(FATAL_ERROR
    "The SDK selected in ${F1TENTH_DEPTHAI_PREFIX} is not a shared DepthAI SDK. "
    "Check the existing SDK selection; no SDK build or installation is performed.")
endif()

# Resolve the imported library for the shell hook as well as the RPATH.
get_target_property(_f1tenth_depthai_library depthai::core IMPORTED_LOCATION)
if(NOT _f1tenth_depthai_library)
  get_target_property(_f1tenth_depthai_configs depthai::core IMPORTED_CONFIGURATIONS)
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _f1tenth_depthai_build_type)
  foreach(_config ${_f1tenth_depthai_build_type} RELEASE RELWITHDEBINFO
      NOCONFIG ${_f1tenth_depthai_configs})
    get_target_property(_f1tenth_depthai_library depthai::core
      "IMPORTED_LOCATION_${_config}")
    if(_f1tenth_depthai_library)
      break()
    endif()
  endforeach()
endif()
if(NOT _f1tenth_depthai_library OR NOT EXISTS "${_f1tenth_depthai_library}")
  message(FATAL_ERROR "The existing DepthAI shared library is missing: "
    "${_f1tenth_depthai_library}. Check the SDK config in ${depthai_DIR}; "
    "no SDK build or installation is performed.")
endif()
# Check the exported path, allowing an installed SDK's ordinary library symlinks.
get_filename_component(_f1tenth_depthai_prefix_path "${F1TENTH_DEPTHAI_PREFIX}" ABSOLUTE)
get_filename_component(_f1tenth_depthai_library_path "${_f1tenth_depthai_library}" ABSOLUTE)
file(RELATIVE_PATH _f1tenth_depthai_relative_library
  "${_f1tenth_depthai_prefix_path}" "${_f1tenth_depthai_library_path}")
if(IS_ABSOLUTE "${_f1tenth_depthai_relative_library}" OR
    _f1tenth_depthai_relative_library MATCHES "^\\.\\./")
  message(FATAL_ERROR "The DepthAI package points outside the selected SDK prefix: "
    "${_f1tenth_depthai_library}. Check F1TENTH_DEPTHAI_PREFIX and ${depthai_DIR}.")
endif()
get_filename_component(F1TENTH_DEPTHAI_LIBRARY_DIR
  "${_f1tenth_depthai_library}" DIRECTORY)

list(PREPEND CMAKE_BUILD_RPATH "${F1TENTH_DEPTHAI_LIBRARY_DIR}")
list(PREPEND CMAKE_INSTALL_RPATH "${F1TENTH_DEPTHAI_LIBRARY_DIR}")

# Existing camera/BEV launch files read this metadata for their child process.
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/depthai_runtime_dir.txt"
  CONTENT "$<TARGET_FILE_DIR:depthai::core>\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/depthai_runtime_dir.txt"
  DESTINATION share/${PROJECT_NAME})

# Also cover ros2 run, standalone viewers and component containers. Register
# the shell hook without a .dsv shortcut so re-sourcing moves the SDK to the
# front even when an older SDK has since been prepended to the environment.
if(APPLE)
  set(F1TENTH_DEPTHAI_LOADER_VARIABLE DYLD_LIBRARY_PATH)
else()
  set(F1TENTH_DEPTHAI_LOADER_VARIABLE LD_LIBRARY_PATH)
endif()
ament_environment_hooks("${CMAKE_CURRENT_LIST_DIR}/f1tenth_depthai.sh.in")
message(STATUS "[${PROJECT_NAME}] Workspace DepthAI config: ${depthai_DIR}")
message(STATUS "[${PROJECT_NAME}] Workspace DepthAI library: ${_f1tenth_depthai_library}")
