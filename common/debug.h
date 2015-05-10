#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <stdio.h>

#ifdef DEBUG
    #define dprintf(fmt, ...) fprintf(stderr, fmt, ...)
#else
    #define dprintf(fmt, ...) 
#endif

#ifdef DEBUG
    #define dprint(s) fprintf(stderr, "%s", s)
#else
    #define dprint(s) 
#endif

#endif
