#ifndef __OU_OU_DLL_H_INCLUDED
#define __OU_OU_DLLASSERT_H_INCLUDED

#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DLL_OU
    #ifdef __GNUC__
      #define OU_VISIBLE __attribute__ ((dllexport))
    #else
      #define OU_VISIBLE __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define OU_VISIBLE __attribute__ ((dllimport))
    #else
      #define OU_VISIBLE __declspec(dllimport)
    #endif
  #endif
  #define OU_HIDDEN
#else
  #if __GNUC__ >= 4
    #define OU_VISIBLE __attribute__ ((visibility ("default")))
    #define OU_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define OU_VISIBLE
    #define OU_HIDDEN
  #endif
#endif

#endif
