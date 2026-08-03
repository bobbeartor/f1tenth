#ifndef LANE_MASK__VISIBILITY_CONTROL_HPP_
#define LANE_MASK__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LANE_MASK_EXPORT __attribute__((dllexport))
    #define LANE_MASK_IMPORT __attribute__((dllimport))
  #else
    #define LANE_MASK_EXPORT __declspec(dllexport)
    #define LANE_MASK_IMPORT __declspec(dllimport)
  #endif
  #ifdef LANE_MASK_BUILDING_DLL
    #define LANE_MASK_PUBLIC LANE_MASK_EXPORT
  #else
    #define LANE_MASK_PUBLIC LANE_MASK_IMPORT
  #endif
  #define LANE_MASK_LOCAL
#else
  #define LANE_MASK_EXPORT __attribute__((visibility("default")))
  #define LANE_MASK_IMPORT
  #if __GNUC__ >= 4
    #define LANE_MASK_PUBLIC __attribute__((visibility("default")))
    #define LANE_MASK_LOCAL __attribute__((visibility("hidden")))
  #else
    #define LANE_MASK_PUBLIC
    #define LANE_MASK_LOCAL
  #endif
#endif

#endif  // LANE_MASK__VISIBILITY_CONTROL_HPP_
