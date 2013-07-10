/** \file orcwrap.cpp
 * \brief Implementation of orcwrap, an OpenRAVE interface command parser.
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

#include <istream>
#include <ostream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <openrave/openrave.h>

extern "C" {
#include "libcd/util_shparse.h"
}

#include "orcwrap.h"

static bool orcwrap_call(
   const char * cmd,
   boost::function<int (int, char * [], std::ostream&)> fn,
   std::ostream& sout, std::istream& sinput)
{
   char * in;
   int argc;
   char ** argv;
   int ret;
   
   std::ostringstream oss;
   oss << cmd << " " << sinput.rdbuf();
   in = (char *) malloc(strlen(oss.str().c_str())+1);
   if (!in) return false;
   strcpy(in, oss.str().c_str());
   
   cd_util_shparse(in, &argc, &argv);
   
   try
   {
      ret = fn(argc, argv, sout);
   }
   catch (...)
   {
      free(in);
      free(argv);
      throw;
   }
   
   free(in);
   free(argv);
   return (ret == 0) ? true : false;
}

boost::function<bool (std::ostream&, std::istream&)>
orcwrap(boost::function<int (int, char * [], std::ostream&)> fn)
{
   return boost::bind(orcwrap_call,"openrave_command",fn,_1,_2);
}

boost::function<bool (std::ostream&, std::istream&)>
orcwrap(const char * cmd, boost::function<int (int, char * [], std::ostream&)> fn)
{
   return boost::bind(orcwrap_call,cmd,fn,_1,_2);
}

