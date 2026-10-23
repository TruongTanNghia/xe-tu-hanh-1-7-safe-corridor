#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include "main.h"

#define NANOSECONDS_PER_SECOND 1000000000LL
#define NANOSECONDS_PER_MILLISECOND 1000000LL

int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
  (void) clock_id;

  if (tp == NULL) {
    return -1;
  }

  uint32_t tick_ms = HAL_GetTick();

  tp->tv_sec = (time_t)(tick_ms / 1000U);
  tp->tv_nsec = (long)((tick_ms % 1000U) * NANOSECONDS_PER_MILLISECOND);

  return 0;
}


int _gettimeofday(struct timeval *tv, void *tzvp)
{
  (void) tzvp;

  if (tv == NULL) {
    return -1;
  }

  uint32_t tick_ms = HAL_GetTick();

  tv->tv_sec = (time_t)(tick_ms / 1000U);
  tv->tv_usec = (suseconds_t)((tick_ms % 1000U) * 1000U);

  return 0;
}
