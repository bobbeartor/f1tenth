#ifndef LINE_DETACTOR__VISIBILITY_CONTROL_HPP_
#define LINE_DETACTOR__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LINE_DETACTOR_EXPORT __attribute__((dllexport))
    #define LINE_DETACTOR_IMPORT __attribute__((dllimport))
  #else
    #define LINE_DETACTOR_EXPORT __declspec(dllexport)
    #define LINE_DETACTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef LINE_DETACTOR_BUILDING_DLL
    #define LINE_DETACTOR_PUBLIC LINE_DETACTOR_EXPORT
  #else
    #define LINE_DETACTOR_PUBLIC LINE_DETACTOR_IMPORT
  #endif
#else
  #define LINE_DETACTOR_EXPORT __attribute__((visibility("default")))
  #define LINE_DETACTOR_IMPORT
  #if __GNUC__ >= 4
    #define LINE_DETACTOR_PUBLIC __attribute__((visibility("default")))
  #else
    #define LINE_DETACTOR_PUBLIC
  #endif
#endif

#endif  // LINE_DETACTOR__VISIBILITY_CONTROL_HPP_
