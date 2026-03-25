# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ul_decision_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ul_decision_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ul_decision_FOUND FALSE)
  elseif(NOT ul_decision_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ul_decision_FOUND FALSE)
  endif()
  return()
endif()
set(_ul_decision_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ul_decision_FIND_QUIETLY)
  message(STATUS "Found ul_decision: 0.0.0 (${ul_decision_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ul_decision' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ul_decision_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ul_decision_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ul_decision_DIR}/${_extra}")
endforeach()
