# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_smrr_multinav_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED smrr_multinav_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(smrr_multinav_FOUND FALSE)
  elseif(NOT smrr_multinav_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(smrr_multinav_FOUND FALSE)
  endif()
  return()
endif()
set(_smrr_multinav_CONFIG_INCLUDED TRUE)

# output package information
if(NOT smrr_multinav_FIND_QUIETLY)
  message(STATUS "Found smrr_multinav: 0.0.0 (${smrr_multinav_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'smrr_multinav' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${smrr_multinav_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(smrr_multinav_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${smrr_multinav_DIR}/${_extra}")
endforeach()
