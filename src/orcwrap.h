/** \file orcwrap.h
 * \brief Interface to orcwrap, an OpenRAVE interface command parser.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012-2013 Carnegie Mellon University */

/* This module (orcdchomp) is part of libcd.
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

/* requires:
 * #include <istream>
 * #include <ostream>
 * #include <boost/function.hpp>
 */

/* This is a tiny generic library which parses OpenRAVE interface command
 * strings into an argument array using shell-like quoting
 * (using libcd's util_shparse) */

boost::function<bool (std::ostream&, std::istream&)>
   orcwrap(boost::function<int (int, char * [], std::ostream&)> fn);

boost::function<bool (std::ostream&, std::istream&)>
   orcwrap(const char * cmd, boost::function<int (int, char * [], std::ostream&)> fn);
