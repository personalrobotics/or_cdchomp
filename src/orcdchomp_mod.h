/** \file orcdchomp_mod.h
 * \brief Interface to the orcdchomp module, an implementation of CHOMP
 *        using libcd.
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

#include "orcwrap.h"

namespace orcdchomp
{

struct sdf;

/* the module itself */
class mod : public OpenRAVE::ModuleBase
{
public:
   OpenRAVE::EnvironmentBasePtr e; /* filled on module creation */
   int n_sdfs;
   struct sdf * sdfs;
   
   int viewspheres(int argc, char * argv[], std::ostream& sout);
   int computedistancefield(int argc, char * argv[], std::ostream& sout);
   int addfield_fromobsarray(int argc, char * argv[], std::ostream& sout);
   int viewfields(int argc, char * argv[], std::ostream& sout);
   int removefield(int argc, char * argv[], std::ostream& sout);
   int create(int argc, char * argv[], std::ostream& sout);
   int iterate(int argc, char * argv[], std::ostream& sout);
   int gettraj(int argc, char * argv[], std::ostream& sout);
   int destroy(int argc, char * argv[], std::ostream& sout);

   mod(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::ModuleBase(penv)
   {
      __description = "orcdchomp: implementation chomp using libcd";
      RegisterCommand("viewspheres",orcwrap(boost::bind(&mod::viewspheres,this,_1,_2,_3)),"view spheres");
      RegisterCommand("computedistancefield",orcwrap(boost::bind(&mod::computedistancefield,this,_1,_2,_3)),"compute distance field");
      RegisterCommand("addfield_fromobsarray",orcwrap(boost::bind(&mod::addfield_fromobsarray,this,_1,_2,_3)),"compute distance field");
      RegisterCommand("viewfields",orcwrap(boost::bind(&mod::viewfields,this,_1,_2,_3)),"view fields");
      RegisterCommand("removefield",orcwrap(boost::bind(&mod::removefield,this,_1,_2,_3)),"remove distance field from kinbody");
      RegisterCommand("create",orcwrap(boost::bind(&mod::create,this,_1,_2,_3)),"create a chomp run");
      RegisterCommand("iterate",orcwrap(boost::bind(&mod::iterate,this,_1,_2,_3)),"create a chomp run");
      RegisterCommand("gettraj",orcwrap(boost::bind(&mod::gettraj,this,_1,_2,_3)),"create a chomp run");
      RegisterCommand("destroy",orcwrap(boost::bind(&mod::destroy,this,_1,_2,_3)),"create a chomp run");
      
      this->e = penv;
      this->n_sdfs = 0;
      this->sdfs = 0;
   }
   virtual ~mod() {}
   void Destroy() { RAVELOG_INFO("module unloaded from environment\n"); }
   /* This is called on e.LoadProblem(m, 'command') */
   int main(const std::string& cmd) { RAVELOG_INFO("module init cmd: %s\n", cmd.c_str()); return 0; }
};

void run_destroy(struct run * r);

struct tsr
{
   int manipindex;
   char bodyandlink[32];
   double T0w[7];
   double Twe[7];
   double Bw[6][2];
};

int tsr_create_parse(struct tsr ** tp, char * str);
void tsr_destroy(struct tsr * t);

} /* namespace orcdchomp */
