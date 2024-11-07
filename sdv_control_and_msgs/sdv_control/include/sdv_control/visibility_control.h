#ifndef sdv_control__VISIBILITY_CONTROL_H_
#define sdv_control__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define sdv_control_EXPORT __attribute__ ((dllexport))
    #define sdv_control_IMPORT __attribute__ ((dllimport))
  #else
    #define sdv_control_EXPORT __declspec(dllexport)
    #define sdv_control_IMPORT __declspec(dllimport)
  #endif
  #ifdef sdv_control_BUILDING_LIBRARY
    #define sdv_control_PUBLIC sdv_control_EXPORT
  #else
    #define sdv_control_PUBLIC sdv_control_IMPORT
  #endif
  #define sdv_control_PUBLIC_TYPE sdv_control_PUBLIC
  #define sdv_control_LOCAL
#else
  #define sdv_control_EXPORT __attribute__ ((visibility("default")))
  #define sdv_control_IMPORT
  #if __GNUC__ >= 4
    #define sdv_control_PUBLIC __attribute__ ((visibility("default")))
    #define sdv_control_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define sdv_control_PUBLIC
    #define sdv_control_LOCAL
  #endif
  #define sdv_control_PUBLIC_TYPE
#endif

#endif  // sdv_control__VISIBILITY_CONTROL_H_
