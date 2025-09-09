#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AGILEX_PIPER_ROS2_CONTROL_EXPORT __attribute__ ((dllexport))
    #define AGILEX_PIPER_ROS2_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define AGILEX_PIPER_ROS2_CONTROL_EXPORT __declspec(dllexport)
    #define AGILEX_PIPER_ROS2_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef AGILEX_PIPER_ROS2_CONTROL_BUILDING_LIBRARY
    #define AGILEX_PIPER_ROS2_CONTROL_PUBLIC AGILEX_PIPER_ROS2_CONTROL_EXPORT
  #else
    #define AGILEX_PIPER_ROS2_CONTROL_PUBLIC AGILEX_PIPER_ROS2_CONTROL_IMPORT
  #endif
  #define AGILEX_PIPER_ROS2_CONTROL_PUBLIC_TYPE AGILEX_PIPER_ROS2_CONTROL_PUBLIC
  #define AGILEX_PIPER_ROS2_CONTROL_LOCAL
#else
  #define AGILEX_PIPER_ROS2_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define AGILEX_PIPER_ROS2_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define AGILEX_PIPER_ROS2_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define AGILEX_PIPER_ROS2_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AGILEX_PIPER_ROS2_CONTROL_PUBLIC
    #define AGILEX_PIPER_ROS2_CONTROL_LOCAL
  #endif
  #define AGILEX_PIPER_ROS2_CONTROL_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif
