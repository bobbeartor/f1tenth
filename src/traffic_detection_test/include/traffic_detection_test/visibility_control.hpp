#ifndef TRAFFIC_DETECTION_TEST__VISIBILITY_CONTROL_HPP_
#define TRAFFIC_DETECTION_TEST__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAFFIC_DETECTION_TEST_EXPORT __attribute__((dllexport))
    #define TRAFFIC_DETECTION_TEST_IMPORT __attribute__((dllimport))
  #else
    #define TRAFFIC_DETECTION_TEST_EXPORT __declspec(dllexport)
    #define TRAFFIC_DETECTION_TEST_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAFFIC_DETECTION_TEST_BUILDING_DLL
    #define TRAFFIC_DETECTION_TEST_PUBLIC TRAFFIC_DETECTION_TEST_EXPORT
  #else
    #define TRAFFIC_DETECTION_TEST_PUBLIC TRAFFIC_DETECTION_TEST_IMPORT
  #endif
#else
  #define TRAFFIC_DETECTION_TEST_EXPORT __attribute__((visibility("default")))
  #define TRAFFIC_DETECTION_TEST_IMPORT
  #if __GNUC__ >= 4
    #define TRAFFIC_DETECTION_TEST_PUBLIC \
      __attribute__((visibility("default")))
  #else
    #define TRAFFIC_DETECTION_TEST_PUBLIC
  #endif
#endif

#endif  // TRAFFIC_DETECTION_TEST__VISIBILITY_CONTROL_HPP_
