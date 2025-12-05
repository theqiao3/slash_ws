# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dll_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dll_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dll_FOUND FALSE)
  elseif(NOT dll_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dll_FOUND FALSE)
  endif()
  return()
endif()
set(_dll_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dll_FIND_QUIETLY)
  message(STATUS "Found dll: 1.0.0 (${dll_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dll' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dll_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dll_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dll_DIR}/${_extra}")
endforeach()
