/** \file orcdchomp_kdata.h
 * \brief Interface to orcdchomp_kdata, a parser for sphere data provided
 *        with an OpenRAVE kinbody XML file.
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
 *  - openrave/openrave.h
 * */

namespace orcdchomp
{

struct sphereelem
{
   struct sphereelem * next;
   struct sphere * s;
};

struct sphere
{
   /* parsed from xml */
   char linkname[32];
   double pos[3];
   double radius;
};


/* the kinbody-attached data class */
class kdata : public OpenRAVE::XMLReadable
{
public:
   struct sphereelem * sphereelems;
   kdata();
   ~kdata();
};


/* the kdata-parser */
class kdata_parser : public OpenRAVE::BaseXMLReader
{
public:
   boost::shared_ptr<kdata> d;
   bool inside_spheres;

   kdata_parser(boost::shared_ptr<kdata> passed_d, const OpenRAVE::AttributesList& atts);
   virtual OpenRAVE::XMLReadablePtr GetReadable();
   virtual ProcessElement startElement(const std::string& name, const OpenRAVE::AttributesList& atts);
   virtual void characters(const std::string& ch);
   virtual bool endElement(const std::string& name);
};

} /* namespace orcdchomp */
