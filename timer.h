#pragma once

#ifdef __GNUC__
#include <execinfo.h>
#include <sys/time.h>
#endif

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#endif

inline double getCurrentTime()
{
#ifdef _WIN32
  SYSTEMTIME tp; GetSystemTime(&tp);
  /*
     Please note: we are not handling the "leap year" issue.
 */
  size_t numSecsSince2020
      = tp.wSecond
      + (60ull) * tp.wMinute
      + (60ull * 60ull) * tp.wHour
      + (60ull * 60ul * 24ull) * tp.wDay
      + (60ull * 60ul * 24ull * 365ull) * (tp.wYear - 2020);
  return double(numSecsSince2020 + tp.wMilliseconds * 1e-3);
#else
  struct timeval tp; gettimeofday(&tp,nullptr);
  return double(tp.tv_sec) + double(tp.tv_usec)/1E6;
#endif
}

