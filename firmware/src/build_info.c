/*
 * build_info.c
 *
 * Recompiled on every build (makefile.init rewrites build_info_gen.h), so
 * __DATE__/__TIME__ are the build time, not the time this file last changed.
 */

#include "build_info.h"

#if defined(__has_include)
 #if __has_include("build_info_gen.h")
  #include "build_info_gen.h"
 #endif
#endif

#ifndef BUILD_INFO_GIT_HASH
 #define BUILD_INFO_GIT_HASH  "nogit" /* Built without firmware/makefile.init. */
 #define BUILD_INFO_GIT_DIRTY (1)
#endif

#if BUILD_INFO_GIT_DIRTY
 #define BUILD_INFO_DIRTY_SUFFIX "-dirty"
#else
 #define BUILD_INFO_DIRTY_SUFFIX ""
#endif

char const g_build_id[] = BUILD_INFO_GIT_HASH BUILD_INFO_DIRTY_SUFFIX " " __DATE__ " " __TIME__;
