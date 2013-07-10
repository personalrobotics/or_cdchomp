/** \file util.h
 * \brief Interface to cd_util, a set of general utility functions.
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

#ifndef CD_UTIL_H
#define CD_UTIL_H

#define cd_util_round(in) floor((in) + 0.5)

void cd_util_exitmsg(int status, const char * templ, ...);

/* Note: This is inclusive on both ends! */
double cd_util_rand_double(double min, double max);

int cd_util_rand_int(unsigned int n);

#endif /* CD_UTIL_H */
