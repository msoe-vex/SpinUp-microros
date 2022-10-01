#ifndef _ROS_CLOCK_GETTIME_H_
#define _ROS_CLOCK_GETTIME_H_

#include <sys/time.h>
#include "api.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
#endif

int clock_gettime(clockid_t unused, struct timespec *tp);

#ifdef __cplusplus
}
#endif

#endif  // _ROS_CLOCK_GETTIME_H_
