/** \file os.c
 * \brief Implementation of cd_os, a collection of os routines.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012-2013 Carnegie Mellon University */

/* This module (cd_os) is part of libcd.
 *
 * This module of libcd is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This module of libcd is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * A copy of the GNU General Public License is provided with libcd
 * (license-gpl.txt) and is also available at <http://www.gnu.org/licenses/>.
 */

/* note: we need POSIX.1b support in time.h */
#include <time.h>
#include "os.h"

int cd_os_timespec_set_zero(struct timespec * t)
{
   t->tv_sec = 0;
   t->tv_nsec = 0;
   return 0;
}

int cd_os_timespec_add(struct timespec * dst, const struct timespec * src)
{
   dst->tv_sec += src->tv_sec;
   dst->tv_nsec += src->tv_nsec;
   if (dst->tv_nsec > 999999999)
   {
      dst->tv_sec += 1;
      dst->tv_nsec -= 1000000000;
   }
   return 0;
}

int cd_os_timespec_sub(struct timespec * dst, const struct timespec * src)
{
   dst->tv_sec -= src->tv_sec;
   dst->tv_nsec -= src->tv_nsec;
   if (dst->tv_nsec < 0)
   {
      dst->tv_sec -= 1;
      dst->tv_nsec += 1000000000;
   }
   return 0;
}

double cd_os_timespec_double(const struct timespec * src)
{
   return src->tv_sec + ((double)(src->tv_nsec))/1000000000.0;
}
