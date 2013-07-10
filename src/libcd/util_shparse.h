/** \file util_shparse.h
 * \brief Interface to cd_util_shparse, a simple POSIX shell argument parser.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012-2013 Christopher Dellin */

/* This module (cd_util_shparse) is part of libcd.
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

/* Simple POSIX shell argument parsing;
 * that is, turns a string like:
 *   grep -v -e can -e "can't" -e 'Say "Hi!"' -e "do\\don't"
 * into an argv array like:
 *   grep, -v, -e, can, -e, can't, -e, Say "Hi!", -e, do\don't
 * Characters that have special meaning are:
 *   space characters, newline, backslash, double quote, single quote
 */

int cd_util_shparse(char * in, int * argcp, char *** argvp);
