/** \file util.c
 * \brief Implementation of cd_util, a set of general utility functions.
 * \author Christopher Dellin
 * \date 2010-2012
 */

/* (C) Copyright 2010-2013 Carnegie Mellon University */

/* This module (cd_util) is part of libcd.
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "util.h"

void cd_util_exitmsg(int status, const char * templ, ...)
{
   va_list ap;
   va_start(ap, templ);
   vfprintf((status==EXIT_SUCCESS)?stdout:stderr, templ, ap);
   exit(status);
   va_end(ap);
   return;
}

double cd_util_rand_double(double min, double max)
{
   return min + (max - min) * ((double) rand() / (double) RAND_MAX);
}

/* From http://c-faq.com/lib/rand.931117.html
 * scs@eskimo.com (Steve Summit) 
 * Wed, 17 Nov 1993 06:10:36 GMT
 * Newsgroups: comp.lang.c
 * Subject: Re: help with random #s! */
int cd_util_rand_int(unsigned int n)
{
   return (double) rand() / ((double) RAND_MAX + 1) * n;
}
