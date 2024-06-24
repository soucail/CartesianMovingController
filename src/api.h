#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define CartesianMovingController_DLLIMPORT __declspec(dllimport)
#  define CartesianMovingController_DLLEXPORT __declspec(dllexport)
#  define CartesianMovingController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define CartesianMovingController_DLLIMPORT __attribute__((visibility("default")))
#    define CartesianMovingController_DLLEXPORT __attribute__((visibility("default")))
#    define CartesianMovingController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define CartesianMovingController_DLLIMPORT
#    define CartesianMovingController_DLLEXPORT
#    define CartesianMovingController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef CartesianMovingController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define CartesianMovingController_DLLAPI
#  define CartesianMovingController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef CartesianMovingController_EXPORTS
#    define CartesianMovingController_DLLAPI CartesianMovingController_DLLEXPORT
#  else
#    define CartesianMovingController_DLLAPI CartesianMovingController_DLLIMPORT
#  endif // CartesianMovingController_EXPORTS
#  define CartesianMovingController_LOCAL CartesianMovingController_DLLLOCAL
#endif // CartesianMovingController_STATIC