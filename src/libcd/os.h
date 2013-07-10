/** \file os.h
 * \brief Interface to cd_os, a collection of os routines.
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

/* requires time.h (with POSIX.1b) */

/* elapsed time functions (to be moved to cd_os)
 * useful for timing code;
 * see clock_gettime();
 * we use struct timespec, from the 1993 edition of the POSIX.1b standard
 * (IEEE Standard 1003.1b-1993)
 * note how these functions mirror those from cd_mat (-: */

int cd_os_timespec_set_zero(struct timespec * t);
int cd_os_timespec_add(struct timespec * dst, const struct timespec * src);
int cd_os_timespec_sub(struct timespec * dst, const struct timespec * src);
double cd_os_timespec_double(const struct timespec * src);

/* ugh, I should really get inlining working
 * (see http://gcc.gnu.org/onlinedocs/gcc/Inline.html) */

#define CD_OS_TIMESPEC_SET_ZERO(t) do { (t)->tv_sec = 0; (t)->tv_nsec = 0; } while (0)

#define CD_OS_TIMESPEC_ADD(dst, src) do { (dst)->tv_sec += (src)->tv_sec; (dst)->tv_nsec += (src)->tv_nsec; \
   if ((dst)->tv_nsec > 999999999) { (dst)->tv_sec += 1; (dst)->tv_nsec -= 1000000000; } } while (0)

#define CD_OS_TIMESPEC_SUB(dst, src) do { (dst)->tv_sec -= (src)->tv_sec; (dst)->tv_nsec -= (src)->tv_nsec; \
   if ((dst)->tv_nsec < 0) { (dst)->tv_sec -= 1; (dst)->tv_nsec += 1000000000; } } while (0)

#define CD_OS_TIMESPEC_DOUBLE(src) ((src)->tv_sec + ((double)((src)->tv_nsec))/1000000000.0)
