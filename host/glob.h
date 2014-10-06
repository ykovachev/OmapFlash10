#ifndef __GLOB_H
#define __GLOB_H

#include "types.h"

#ifndef _WIN32
#define NOPARITY            0
#define ODDPARITY           1
#define EVENPARITY          2
#define MARKPARITY          3
#define SPACEPARITY         4

#define ONESTOPBIT          0
#define ONE5STOPBITS        1
#define TWOSTOPBITS         2
#endif	// _WIN32

#ifdef _MSC_VER
#define STRICMP  stricmp
#define SNPRINTF _snprintf
#define VSNPRINTF _vsnprintf
#else
#define STRICMP  strcasecmp
#define SNPRINTF snprintf
#define VSNPRINTF vsnprintf
#endif // _MSC_VER
#endif	// __GLOB_H
