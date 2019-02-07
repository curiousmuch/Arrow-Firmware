
/* direwolf.h - Common stuff used many places. */

// TODO:   include this file first before anything else in each .c file.


#ifndef DIREWOLF_H
#define DIREWOLF_H

#include <stdio.h>
	
#define SLEEP_SEC(n) sleep(n)
#define SLEEP_MS(n) usleep((n)*1000)

#ifndef G_UNKNOWN
#include "latlong.h"
#endif

/* Conversion Macros */
#define DW_METERS_TO_FEET(x) ((x) == G_UNKNOWN ? G_UNKNOWN : (x) * 3.2808399)
#define DW_FEET_TO_METERS(x) ((x) == G_UNKNOWN ? G_UNKNOWN : (x) * 0.3048)
#define DW_KM_TO_MILES(x) ((x) == G_UNKNOWN ? G_UNKNOWN : (x) * 0.621371192)

#define DW_KNOTS_TO_MPH(x) ((x) == G_UNKNOWN ? G_UNKNOWN : (x) * 1.15077945)
#define DW_KNOTS_TO_METERS_PER_SEC(x) ((x) == G_UNKNOWN ? G_UNKNOWN : (x) * 0.51444444444)
#define DW_MPH_TO_KNOTS(x) ((x) == G_UNKNOWN ? G_UNKNOWN : (x) * 0.868976)
#define DW_MPH_TO_METERS_PER_SEC(x) ((x) == G_UNKNOWN ? G_UNKNOWN : (x) * 0.44704)

#define DW_MBAR_TO_INHG(x) ((x) == G_UNKNOWN ? G_UNKNOWN : (x) * 0.0295333727)


#define SOCK_SEND(s,data,size) send(s,data,size, MSG_NOSIGNAL)
#define SOCK_RECV(s,data,size) recv(s,data,size,0)


/* Platform differences for string functions. */

// Don't recall why for everyone.
char *strcasestr(const char *S, const char *FIND);

// These prevent /usr/include/gps.h from providing its own definition.
#define HAVE_STRLCAT 1
#define HAVE_STRLCPY 1


#define DEBUG_STRL 0

#if DEBUG_STRL

#define strlcpy(dst,src,siz) strlcpy_debug(dst,src,siz,__FILE__,__func__,__LINE__)
#define strlcat(dst,src,siz) strlcat_debug(dst,src,siz,__FILE__,__func__,__LINE__)

size_t strlcpy_debug(char *__restrict__ dst, const char *__restrict__ src, size_t siz, const char *file, const char *func, int line);
size_t strlcat_debug(char *__restrict__ dst, const char *__restrict__ src, size_t siz, const char *file, const char *func, int line);

#else

#define strlcpy(dst,src,siz) strlcpy_debug(dst,src,siz)
#define strlcat(dst,src,siz) strlcat_debug(dst,src,siz)

size_t strlcpy_debug(char *__restrict__ dst, const char *__restrict__ src, size_t siz);
size_t strlcat_debug(char *__restrict__ dst, const char *__restrict__ src, size_t siz);

#endif  /* DEBUG_STRL */

#endif   /* ifndef DIREWOLF_H */
