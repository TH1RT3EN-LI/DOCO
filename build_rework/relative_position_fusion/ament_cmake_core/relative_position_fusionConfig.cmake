# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_relative_position_fusion_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED relative_position_fusion_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(relative_position_fusion_FOUND FALSE)
  elseif(NOT relative_position_fusion_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(relative_position_fusion_FOUND FALSE)
  endif()
  return()
endif()
set(_relative_position_fusion_CONFIG_INCLUDED TRUE)

# output package information
if(NOT relative_position_fusion_FIND_QUIETLY)
  message(STATUS "Found relative_position_fusion: 0.0.1 (${relative_position_fusion_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'relative_position_fusion' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${relative_position_fusion_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(relative_position_fusion_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${relative_position_fusion_DIR}/${_extra}")
endforeach()
