/** \file orcdchomp_mod.cpp
 * \brief Implementation of the orcdchomp module, an implementation of CHOMP
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

#include <time.h>
#include <cblas.h>

extern "C" {
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "libcd/chomp.h"
#include "libcd/grid.h"
#include "libcd/grid_flood.h"
#include "libcd/kin.h"
#include "libcd/mat.h"
#include "libcd/os.h"
#include "libcd/spatial.h"
#include "libcd/util.h"
#include "libcd/util_shparse.h"
}

#include <openrave/openrave.h>
#include <openrave/planningutils.h>

#include "orcdchomp_kdata.h"
#include "orcdchomp_mod.h"

//#define DEBUG_TIMING

#ifdef DEBUG_TIMING
#  define TIC() { struct timespec tic; struct timespec toc; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tic);
#  define TOC(tsptr) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &toc); CD_OS_TIMESPEC_SUB(&toc, &tic); CD_OS_TIMESPEC_ADD(tsptr, &toc); }
#else
#  define TIC()
#  define TOC(tsptr)
#endif

namespace {

std::string sf(const char * fmt, ...)
{
   va_list ap;
   va_start(ap, fmt);
   int size = vsnprintf(0, 0, fmt, ap);
   va_end(ap);
   char * buf = new char[size+1];
   va_start(ap, fmt);
   vsnprintf(buf, size+1, fmt, ap);
   va_end(ap);
   std::string ret = std::string(buf);
   delete[] buf;
   return ret;
}

// was cd_mat_vec_print
std::string sf_vector(const double * a, int n)
{
   int j;
   std::string buf("[");
   for (j=0; j<n; j++)
      buf += sf(" %8.4f", a[j]);
   buf += " ]";
   return buf;
}

/* modified from OpenRAVE::KinBody::ComputeAABB() */
OpenRAVE::AABB KinBodyComputeEnabledAABB(OpenRAVE::KinBodyConstPtr kb)
{
   OpenRAVE::Vector vmin, vmax;
   bool binitialized = false;
   OpenRAVE::AABB ab;
   const std::vector<OpenRAVE::KinBody::LinkPtr> & links = kb->GetLinks();
   for (std::vector<OpenRAVE::KinBody::LinkPtr>::const_iterator
        it=links.begin(); it!=links.end(); it++)
   {
      if (!(*it)->IsEnabled())
         continue;
      ab = (*it)->ComputeAABB();
      if((ab.extents.x == 0)&&(ab.extents.y == 0)&&(ab.extents.z == 0)) {
         continue;
      }
      OpenRAVE::Vector vnmin = ab.pos - ab.extents;
      OpenRAVE::Vector vnmax = ab.pos + ab.extents;
      if( !binitialized ) {
         vmin = vnmin;
         vmax = vnmax;
         binitialized = true;
      }
      else {
         if( vmin.x > vnmin.x ) {
            vmin.x = vnmin.x;
         }
         if( vmin.y > vnmin.y ) {
            vmin.y = vnmin.y;
         }
         if( vmin.z > vnmin.z ) {
            vmin.z = vnmin.z;
         }
         if( vmax.x < vnmax.x ) {
            vmax.x = vnmax.x;
         }
         if( vmax.y < vnmax.y ) {
            vmax.y = vnmax.y;
         }
         if( vmax.z < vnmax.z ) {
            vmax.z = vnmax.z;
         }
      }
   }
   if( !binitialized ) {
      ab.pos = kb->GetTransform().trans;
      ab.extents = OpenRAVE::Vector(0,0,0);
   }
   else {
      ab.pos = (OpenRAVE::dReal)0.5 * (vmin + vmax);
      ab.extents = vmax - ab.pos;
   }
   return ab;
}

} // anonymous namespace


namespace orcdchomp
{

struct sdf
{
   char kinbody_name[256];
   double pose[7]; /* pose of the grid w.r.t. the kinbody frame */
   cd_grid * grid;
};


/* ======================================================================== *
 * useful utilities
 */

int replace_1_to_0(double * val, void * rptr)
{
   if (*val == 1.0)
   {
      *val = 0.0;
      return 1;
   }
   return 0;
}


/* ======================================================================== *
 * module commands
 */

int mod::viewspheres(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int si;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
   OpenRAVE::RobotBasePtr r;
   char buf[1024];
   struct orcdchomp::sphereelem * el;
   struct orcdchomp::sphere * s;
   std::vector<OpenRAVE::KinBodyPtr> vgrabbed;
   OpenRAVE::KinBodyPtr k;
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0, 9, 0)
   std::vector<OpenRAVE::KinBody::LinkPtr> klinks;
   std::vector<OpenRAVE::KinBody::GeometryInfoPtr> vgeometries;
   OpenRAVE::KinBody::GeometryInfoPtr g;
   int klinkindex;
   int geomindex;
   OpenRAVE::KinBody::Link * klink;
#endif
   
   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"robot")==0 && i+1<argc)
      {
         if (r.get()) throw OpenRAVE::openrave_exception("Only one robot can be passed!");
         RAVELOG_DEBUG("Getting robot named |%s|.\n", argv[i+1]);
         r = this->e->GetRobot(argv[++i]);
         if (!r.get()) throw OpenRAVE::openrave_exception("Could not find robot with that name!");
         RAVELOG_DEBUG("Using robot %s.\n", r->GetName().c_str());
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   /* check that we have everything */
   if (!r.get()) { throw OpenRAVE::openrave_exception("Did not pass all required args!"); }
   
   /* view each sphere */
   r->GetGrabbed(vgrabbed);
   /*vgrabbed.push_front(r);*/
   vgrabbed.insert(vgrabbed.begin(), r);
   si = 0;
   for (i=0; i<(int)(vgrabbed.size()); i++)
   {
      k = vgrabbed[i];
      /* get kinbody spheres */
      boost::shared_ptr<orcdchomp::kdata> d = boost::dynamic_pointer_cast<orcdchomp::kdata>(k->GetReadableInterface("orcdchomp"));
      if (d) for (el=d->sphereelems; el; el=el->next)
      {
         s = el->s;
         /* make some sweet spheres! */
         OpenRAVE::KinBodyPtr sbody = OpenRAVE::RaveCreateKinBody(this->e);
         sprintf(buf, "orcdchomp_sphere_%d", si);
         sbody->SetName(buf);
         /* set its dimensions */
         {
            std::vector< OpenRAVE::Vector > svec;
            OpenRAVE::Transform t = k->GetLink(s->linkname)->GetTransform();
            OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos); /* copies 3 values */
            v.w = s->radius; /* radius */
            svec.push_back(v);
            sbody->InitFromSpheres(svec, true);
         }
         /* add the sphere */
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,8,0)
         this->e->Add(sbody);
#else
         this->e->AddKinBody(sbody);
#endif
         si++;
      }
      
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0, 9, 0)
      /* load any spheres from the "spheres" geometry group */
      klinks = k->GetLinks();
      for (klinkindex=0; klinkindex<(int)(klinks.size()); klinkindex++) {
         klink = klinks[klinkindex].get();
         if (klink->GetGroupNumGeometries("spheres") == -1) {
            continue; /* there is no "spheres" group */
         }
         vgeometries = klink->GetGeometriesFromGroup("spheres");
         for (geomindex = 0; geomindex < (int)(vgeometries.size()); geomindex++) {
            g = vgeometries[geomindex];
            if (g->_type != OpenRAVE::GT_Sphere) {
               throw OPENRAVE_EXCEPTION_FORMAT("link %s contains non-spherical geometry in the 'spheres' geometry group", klink->GetName().c_str(), OpenRAVE::ORE_Failed);
            }
            /* make some sweet spheres! */
            OpenRAVE::KinBodyPtr sbody = OpenRAVE::RaveCreateKinBody(this->e);
            sprintf(buf, "orcdchomp_sphere_%d", si);
            sbody->SetName(buf);
            /* set its dimensions */
            {
               std::vector< OpenRAVE::Vector > svec;
               OpenRAVE::Transform t = klink->GetTransform();
               OpenRAVE::Vector v = t * g->_t.trans;
               v.w = g->_vGeomData[0]; /* radius */
               svec.push_back(v);
               sbody->InitFromSpheres(svec, true);
            }
            /* add the sphere */
            this->e->Add(sbody);
            si++;
               
         }
      }
#endif
   }
   
   return 0;
}


/* computedistancefield robot Herb2
 * computes a distance field in the vicinity of the passed kinbody
 * 
 * note: does aabb include disabled bodies? probably, but we might hope not ...
 * */
int mod::computedistancefield(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int err;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   OpenRAVE::KinBodyPtr kinbody;
   /* parameters */
   double cube_extent;
   double aabb_padding;
   char * cache_filename;
   int require_cache;
   /* other */
   double temp;
   OpenRAVE::KinBodyPtr cube;
   size_t idx;
   struct cd_grid * g_obs;
   int gsdf_sizearray[3];
   OpenRAVE::geometry::aabb< OpenRAVE::dReal > aabb;
   double pose_world_gsdf[7];
   double pose_cube[7];
   struct sdf sdf_new;
   struct timespec ticks_tic;
   struct timespec ticks_toc;
   int sdf_data_loaded;
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   cube_extent = 0.02;
   aabb_padding = 0.2;
   cache_filename = 0;
   require_cache = 0;

   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"kinbody")==0 && i+1<argc)
      {
         if (kinbody.get()) throw OpenRAVE::openrave_exception("Only one kinbody can be passed!");
         kinbody = this->e->GetKinBody(argv[++i]);
         if (!kinbody.get()) throw OpenRAVE::openrave_exception("Could not find kinbody with that name!");
      }
      else if (strcmp(argv[i],"aabb_padding")==0 && i+1<argc)
         aabb_padding = atof(argv[++i]);
      else if (strcmp(argv[i],"cube_extent")==0 && i+1<argc)
         cube_extent = atof(argv[++i]);
      else if (strcmp(argv[i],"cache_filename")==0 && i+1<argc)
         cache_filename = argv[++i];
      else if (strcmp(argv[i],"require_cache")==0)
         require_cache = 1;
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   RAVELOG_DEBUG("Using kinbody %s.\n", kinbody->GetName().c_str());
   RAVELOG_DEBUG("Using aabb_padding |%f|.\n", aabb_padding);
   RAVELOG_DEBUG("Using cube_extent |%f|.\n", cube_extent);
   RAVELOG_DEBUG("Using cache_filename |%s|.\n", cache_filename ? cache_filename : "none passed");
   RAVELOG_DEBUG("Using require_cache %s.\n", require_cache ? "true" : "false");

   /* check that we have everything */
   if (!kinbody.get()) throw OpenRAVE::openrave_exception("Did not pass all required args!");
   
   /* make sure we don't already have an sdf loaded for this kinbody */
   for (i=0; i<this->n_sdfs; i++)
      if (strcmp(this->sdfs[i].kinbody_name, kinbody->GetName().c_str()) == 0)
         break;
   if (i<this->n_sdfs)
      throw OpenRAVE::openrave_exception("We already have an sdf for this kinbody!");
   
   /* copy in name */
   if (strlen(kinbody->GetName().c_str())+1 > sizeof(sdf_new.kinbody_name))
      throw OpenRAVE::openrave_exception("ugh, orcdchomp currently doesn't support long kinbody names!");
   strcpy(sdf_new.kinbody_name, kinbody->GetName().c_str());

   /* compute aabb when object is at world origin */
   {
      OpenRAVE::KinBody::KinBodyStateSaver statesaver(kinbody);
      kinbody->SetTransform(OpenRAVE::Transform());
      aabb = KinBodyComputeEnabledAABB(kinbody);
   }
   RAVELOG_DEBUG("    pos: %f %f %f\n", aabb.pos[0], aabb.pos[1], aabb.pos[2]);
   RAVELOG_DEBUG("extents: %f %f %f\n", aabb.extents[0], aabb.extents[1], aabb.extents[2]);

   /* calculate dimension sizes (number of cells) */
   for (i=0; i<3; i++)
   {
      /* 0.15m padding (was 0.3m) on each side
       * (this is the radius of the biggest herb2 spehere)
       * (note: extents are half the side lengths!) */
      gsdf_sizearray[i] = (int) ceil((aabb.extents[i]+aabb_padding) / cube_extent);
      RAVELOG_DEBUG("gsdf_sizearray[%d]: %d\n", i, gsdf_sizearray[i]);
   }
   
   /* Create a new grid located around the current kinbody;
    * per-dimension sizes set above */
   temp = 1.0; /* free space */
   err = cd_grid_create_sizearray(&sdf_new.grid, &temp, sizeof(double), 3, gsdf_sizearray);
   if (err) throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
   
   /* set side lengths */
   for (i=0; i<3; i++)
      sdf_new.grid->lengths[i] = gsdf_sizearray[i] * 2.0 * cube_extent;
   RAVELOG_DEBUG("sdf_new.grid->lengths: %s\n", sf_vector(sdf_new.grid->lengths,3).c_str());
   
   /* set pose of grid w.r.t. kinbody frame */
   cd_kin_pose_identity(sdf_new.pose);
   for (i=0; i<3; i++)
      sdf_new.pose[i] = aabb.pos[i] - 0.5 * sdf_new.grid->lengths[i];
   RAVELOG_DEBUG("pose_gsdf: %s\n", sf_vector(sdf_new.pose,7).c_str());
   
   /* we don't have sdf grid data yet */
   sdf_data_loaded = 0;
   
   /* attempt to load the sdf from file */
   if (cache_filename) do
   {
      FILE * fp;
      RAVELOG_INFO("Reading SDF data for KinBody '%s' from file %s ...\n",
         kinbody->GetName().c_str(), cache_filename);
      fp = fopen(cache_filename, "rb");
      if (!fp) { RAVELOG_ERROR("could not read from file!\n"); break; }
      
      /* check file size */
      fseek(fp, 0L, SEEK_END);
      if (ftell(fp) != sdf_new.grid->cell_size * sdf_new.grid->ncells)
      {
         RAVELOG_ERROR("cached file size %lu bytes doesn't match expected size %lu! recomputing ...\n",
            ftell(fp), sdf_new.grid->cell_size * sdf_new.grid->ncells);
         fclose(fp);
         break;
      }
      fseek(fp, 0L, SEEK_SET);
      /* read grid data */
      i = fread(sdf_new.grid->data, sdf_new.grid->cell_size, sdf_new.grid->ncells, fp);
      if (i != sdf_new.grid->ncells)
      {
         RAVELOG_ERROR("error, couldn't all read the sdf data from the file!\n");
         fclose(fp);
         break;
      }
      fclose(fp);
      sdf_data_loaded = 1;
   } while (0);
   
   if (!sdf_data_loaded)
   {
      if (require_cache)
         throw OpenRAVE::openrave_exception("Field not found from cache, but require_cache flag set!");

      /* create the obstacle grid (same size as sdf_new.grid that we already calculated) */
      err = cd_grid_create_copy(&g_obs, sdf_new.grid);
      if (err)
      {
         cd_grid_destroy(sdf_new.grid);
         throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
      }
      
      /* start timing voxel grid computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);
      
      /* create the cube */
      cube = OpenRAVE::RaveCreateKinBody(this->e);
      cube->SetName("cube");
      
      /* set its dimensions */
      {
         std::vector<OpenRAVE::AABB> vaabbs(1);
         vaabbs[0].extents = OpenRAVE::Vector(cube_extent, cube_extent, cube_extent); /* extents = half side lengths */
         cube->InitFromBoxes(vaabbs, 1);
      }
      
      /* add the cube */
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,8,0)
      this->e->Add(cube);
#else
      this->e->AddKinBody(cube);
#endif
      
      /* get the pose_world_gsdf = pose_world_kinbody * pose_kinbody_gsdf */
      {
         OpenRAVE::Transform t = kinbody->GetTransform();
         pose_world_gsdf[0] = t.trans.x;
         pose_world_gsdf[1] = t.trans.y;
         pose_world_gsdf[2] = t.trans.z;
         pose_world_gsdf[3] = t.rot.y;
         pose_world_gsdf[4] = t.rot.z;
         pose_world_gsdf[5] = t.rot.w;
         pose_world_gsdf[6] = t.rot.x;
         cd_kin_pose_compose(pose_world_gsdf, sdf_new.pose, pose_world_gsdf);
      }

      int collisions = 0;
      
      /* go through the grid, testing for collision as we go;
       * collisions are HUGE_VAL, free are 1.0 */
      RAVELOG_INFO("Computing occupancy grid ...\n");
      for (idx=0; idx<g_obs->ncells; idx++)
      {
         OpenRAVE::Transform t;
         
         if (idx % 100000 == 0)
            RAVELOG_INFO("  idx=%d (%5.1f%%)...\n", (int)idx, (100.0*((double)idx)/((double)g_obs->ncells)));
         
         /* set cube location */
         t.identity();
         cd_kin_pose_identity(pose_cube);
         cd_grid_center_index(g_obs, idx, pose_cube);
         cd_kin_pose_compose(pose_world_gsdf, pose_cube, pose_cube);
         t.trans.x = pose_cube[0];
         t.trans.y = pose_cube[1];
         t.trans.z = pose_cube[2];
         t.rot.y = pose_cube[3];
         t.rot.z = pose_cube[4];
         t.rot.w = pose_cube[5];
         t.rot.x = pose_cube[6];
         cube->SetTransform(t);
         
         /* do collision check */
         if (this->e->CheckCollision(cube))
         {
            *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
            collisions++;
         }
      }

      RAVELOG_INFO("Found %d/%d collisions!\n", collisions, (int)(g_obs->ncells));
      
      /* remove cube */
      this->e->Remove(cube);
      
      /* stop timing voxel grid computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_INFO("Total voxel grid computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* start timing flood fill computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

      /* we assume the point at in the very corner x=y=z is free*/
      RAVELOG_DEBUG("performing flood fill ...\n");
      idx = 0;
      cd_grid_flood_fill(g_obs, idx, 0, (int (*)(void *, void *))replace_1_to_0, 0);
      
      /* change any remaining 1.0 cells to HUGE_VAL (assumed inside of obstacles) */
      for (idx=0; idx<g_obs->ncells; idx++)
         if (*(double *)cd_grid_get_index(g_obs, idx) == 1.0)
            *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
      
      /* stop timing flood fill computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_DEBUG("total flood fill computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* start timing sdf computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

      /* compute the signed distance field (in the module instance) */
      RAVELOG_DEBUG("computing signed distance field ...\n");
      cd_grid_double_bin_sdf(&sdf_new.grid, g_obs);

      /* stop timing sdf computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_DEBUG("total sdf computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* we no longer need the obstacle grid */
      cd_grid_destroy(g_obs);
      
      /* if we were passed a cache_filename, save what we just computed! */
      if (cache_filename)
      {
         FILE * fp;
         RAVELOG_INFO("Saving sdf data to file %s ...\n", cache_filename);
         fp = fopen(cache_filename, "wb");
         i = fwrite(sdf_new.grid->data, sdf_new.grid->cell_size, sdf_new.grid->ncells, fp);
         fclose(fp);
         if (i != sdf_new.grid->ncells)
            RAVELOG_ERROR("Error, couldn't write the sdf data to the file!\n");
      }
   }
   
   /* allocate a new sdf struct, and copy the new one there! */
   this->sdfs = (struct sdf *) realloc(this->sdfs, (this->n_sdfs+1)*sizeof(struct sdf));
   this->sdfs[this->n_sdfs] = sdf_new;
   this->n_sdfs++;
   
   return 0;
}


int mod::addfield_fromobsarray(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int j;
   int err;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   OpenRAVE::KinBodyPtr kinbody;
   /* parameters */
   double * obsarray;
   int sizes[3];
   double lengths[3];
   double pose[7];
   size_t idx;
   double temp;
   struct cd_grid * g_obs;
   int gsdf_sizearray[3];
   double pose_world_gsdf[7];
   struct sdf sdf_new;
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   obsarray = 0;
   for (i=0; i<3; i++) sizes[i] = 0;
   for (i=0; i<3; i++) lengths[i] = 0.0;
   cd_kin_pose_identity(pose);

   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"kinbody")==0 && i+1<argc)
      {
         if (kinbody.get()) throw OpenRAVE::openrave_exception("Only one kinbody can be passed!");
         kinbody = this->e->GetKinBody(argv[++i]);
         if (!kinbody.get()) throw OpenRAVE::openrave_exception("Could not find kinbody with that name!");
      }
      else if (strcmp(argv[i],"obsarray")==0 && i+1<argc)
         sscanf(argv[++i], "%p", &obsarray);
      else if (strcmp(argv[i],"sizes")==0 && i+1<argc)
      {
         char ** sizes_argv;
         int sizes_argc;
         cd_util_shparse(argv[++i], &sizes_argc, &sizes_argv);
         if (sizes_argc != 3) { free(sizes_argv); throw OpenRAVE::openrave_exception("sizes must be length 3!"); }
         for (j=0; j<3; j++)
            sizes[j] = atoi(sizes_argv[j]);
         free(sizes_argv);
      }
      else if (strcmp(argv[i],"lengths")==0 && i+1<argc)
      {
         char ** lengths_argv;
         int lengths_argc;
         cd_util_shparse(argv[++i], &lengths_argc, &lengths_argv);
         if (lengths_argc != 3) { free(lengths_argv); throw OpenRAVE::openrave_exception("lengths must be length 3!"); }
         for (j=0; j<3; j++)
            lengths[j] = atof(lengths_argv[j]);
         free(lengths_argv);
      }
      else if (strcmp(argv[i],"pose")==0 && i+1<argc)
      {
         char ** pose_argv;
         int pose_argc;
         cd_util_shparse(argv[++i], &pose_argc, &pose_argv);
         if (pose_argc != 7) { free(pose_argv); throw OpenRAVE::openrave_exception("pose must be length 7!"); }
         for (j=0; j<7; j++)
            pose[j] = atof(pose_argv[j]);
         free(pose_argv);
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   RAVELOG_DEBUG("Using kinbody %s.\n", kinbody->GetName().c_str());
   RAVELOG_DEBUG("Using obsarray %p.\n", obsarray);
   RAVELOG_DEBUG("Using sizes %d %d %d.\n", sizes[0], sizes[1], sizes[2]);
   RAVELOG_DEBUG("Using lengths %f %f %f.\n", lengths[0], lengths[1], lengths[2]);
   RAVELOG_DEBUG("Using pose %f %f %f %f %f %f %f.\n",
      pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);
   
   /* check that we have everything */
   if (!kinbody.get()) throw OpenRAVE::openrave_exception("Did not pass a kinbody!");
   if (!obsarray) throw OpenRAVE::openrave_exception("Did not pass an obsarray!");
   for (i=0; i<3; i++) if (sizes[i] <= 0) break;
   if (i<3) throw OpenRAVE::openrave_exception("Didn't pass non-zero sizes!");
   for (i=0; i<3; i++) if (lengths[i] <= 0.0) break;
   if (i<3) throw OpenRAVE::openrave_exception("Didn't pass non-zero lengths!");
   cd_kin_pose_normalize(pose);
   
   /* make sure we don't already have an sdf loaded for this kinbody */
   for (i=0; i<this->n_sdfs; i++)
      if (strcmp(this->sdfs[i].kinbody_name, kinbody->GetName().c_str()) == 0)
         break;
   if (i<this->n_sdfs)
      throw OpenRAVE::openrave_exception("We already have an sdf for this kinbody!");
   
   /* copy in name */
   strcpy(sdf_new.kinbody_name, kinbody->GetName().c_str());

   /* set pose of grid wrt kinbody frame */
   cd_mat_memcpy(sdf_new.pose, pose, 7, 1);
   
   /* create obstacle grid */
   temp = 0.0;
   err = cd_grid_create_sizearray(&g_obs, &temp, sizeof(double), 3, sizes);
   if (err) throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
   
   /* take ownership of passed obsarray */
   free(g_obs->data);
   g_obs->data = (char *)obsarray;
      
   /* set side lengths */
   for (i=0; i<3; i++)
      g_obs->lengths[i] = lengths[i];
   
   /* compute sdf from obstacle grid */
   cd_grid_double_bin_sdf(&sdf_new.grid, g_obs);

   /* destroy obstacle grid (also frees passed-in grid) */
   cd_grid_destroy(g_obs);
   
   /* allocate a new sdf struct, and copy the new one there! */
   this->sdfs = (struct sdf *) realloc(this->sdfs, (this->n_sdfs+1)*sizeof(struct sdf));
   this->sdfs[this->n_sdfs] = sdf_new;
   this->n_sdfs++;
   
   return 0;
}

int mod::viewfields(int argc, char * argv[], std::ostream& sout)
{
   int i;

   for (i=0; i<this->n_sdfs; i++)
   {
      OpenRAVE::KinBodyPtr kb;
      OpenRAVE::KinBodyPtr fieldkb;
      OpenRAVE::Transform ortx_kb_fieldkb;
      char buf[32];
      size_t idx;

      /* get corresponding kinbody */
      kb = this->e->GetKinBody(this->sdfs[i].kinbody_name);
      if (!kb)
      {
         throw OPENRAVE_EXCEPTION_FORMAT("KinBody %s referenced by active signed distance field does not exist!\n",
            this->sdfs[i].kinbody_name, OpenRAVE::ORE_Failed);
      }

      /* create the cube */
      fieldkb = OpenRAVE::RaveCreateKinBody(this->e);
      sprintf(buf, "field_%s", this->sdfs[i].kinbody_name);
      fieldkb->SetName(buf);

      /* get aabbs */
      std::vector<OpenRAVE::AABB> vaabbs;

      /* set cube extents */
      OpenRAVE::AABB aabb;
      aabb.extents.x = 0.45 * this->sdfs[i].grid->lengths[0] / this->sdfs[i].grid->sizes[0];
      aabb.extents.y = 0.45 * this->sdfs[i].grid->lengths[1] / this->sdfs[i].grid->sizes[1];
      aabb.extents.z = 0.45 * this->sdfs[i].grid->lengths[2] / this->sdfs[i].grid->sizes[2];

      /* build vaabbs by iterating through field */
      for (idx=0; idx<this->sdfs[i].grid->ncells; idx++)
      {
         double pos_cube[7];
         /* skip if we're outside an obstacle */
         if (*(double *)cd_grid_get_index(this->sdfs[i].grid, idx) > 0.0)
            continue;
         /* get position of cube center w.r.t. field origin */
         cd_grid_center_index(this->sdfs[i].grid, idx, pos_cube);
         /* set position of aabb */
         aabb.pos.x = pos_cube[0];
         aabb.pos.y = pos_cube[1];
         aabb.pos.z = pos_cube[2];
         /* add aabb */
         vaabbs.push_back(aabb);
      }

      /* initialize kinbody */
      fieldkb->InitFromBoxes(vaabbs, 1);

      /* add kinbody to environment */
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,8,0)
      this->e->Add(fieldkb);
#else
      this->e->AddKinBody(fieldkb);
#endif

      /* set transform, relative to kinbody */
      ortx_kb_fieldkb.trans.x = this->sdfs[i].pose[0];
      ortx_kb_fieldkb.trans.y = this->sdfs[i].pose[1];
      ortx_kb_fieldkb.trans.z = this->sdfs[i].pose[2];
      ortx_kb_fieldkb.rot.y = this->sdfs[i].pose[3];
      ortx_kb_fieldkb.rot.z = this->sdfs[i].pose[4];
      ortx_kb_fieldkb.rot.w = this->sdfs[i].pose[5];
      ortx_kb_fieldkb.rot.x = this->sdfs[i].pose[6];
      fieldkb->SetTransform(kb->GetTransform() * ortx_kb_fieldkb);
   }

   return 0;
}

int mod::removefield(int argc, char * argv[], std::ostream& sout)
{
   int i;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   char * kinbody_name;
  
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   kinbody_name = 0;
   
   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"kinbody")==0 && i+1<argc)
      {
         if (kinbody_name) throw OpenRAVE::openrave_exception("Only one kinbody can be passed!");
         kinbody_name = argv[++i];
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   RAVELOG_DEBUG("Using kinbody %s.\n", kinbody_name);
   
   /* search for an sdf rooted to this kinbody */
   for (i=0; i<this->n_sdfs; i++)
      if (strcmp(kinbody_name, this->sdfs[i].kinbody_name) == 0)
         break;
   if (!(i<this->n_sdfs))
      throw OpenRAVE::openrave_exception("kinbody not found, or has no sdf attached!");
   
   /* free the grid */
   cd_grid_destroy(this->sdfs[i].grid);
   
   /* move the subsequent sdfs down into the cleared space */
   for (; i<this->n_sdfs-1; i++)
      this->sdfs[i] = this->sdfs[i+1];
    
   /* realloc to be one smaller! */
   this->sdfs = (struct sdf *) realloc(this->sdfs, (this->n_sdfs-1)*sizeof(struct sdf));
   this->n_sdfs--;
   
   return 0;
}

/* a rooted sdf (fixed in the world) */
struct run_rsdf
{
   double pose_world_gsdf[7];
   double pose_gsdf_world[7];
   struct cd_grid * grid;
};

struct run_sphere
{
   struct run_sphere * next;
   double radius;
   OpenRAVE::KinBody::Link * robot_link;
   int robot_linkindex;
   double pos_wrt_link[3];
};

enum RUN_CON_TYPE
{
  RUN_CON_START,
  RUN_CON_END,
  RUN_CON_ALL
};

/* helper for the tsr constraint callbacks */
struct run_contsr
{
  struct run * r;
  enum RUN_CON_TYPE type;
  int k; /* constraint dimensionalty */
  /* either manip or link should be set */
  OpenRAVE::RobotBase::Manipulator * manip;
  OpenRAVE::KinBody::Link * link;
  struct tsr * tsr;
  int tsr_enabled[6]; /* xyzrpy */
};

/* this encodes a run of chomp, which is set up, iterated, and checked */
struct run
{
   /* the trajectory */
   double * traj;
   int n_points; /* points, including endpoints if any */
   
   /* for interfacing to openrave */
   OpenRAVE::RobotBase * robot;
   int n_adof;
   int floating_base;
   int * adofindices;
   
   /* obstacle parameters */
   double epsilon;
   double epsilon_self;
   double obs_factor;
   double obs_factor_self;
   
   /* all spheres (first n_spheres_active are active) */
   struct run_sphere * spheres;
   int n_spheres;
   int n_spheres_active;
   
   /* space for cached sphere info */
   double * sphere_poss_inactive;
   double * sphere_poss_all; /* [trajpoint(external)][sphereid][xyz] */
   double * sphere_poss; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_vels; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_accs; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_jacs; /* [trajpoint(moving)][sphereid][3xn] */
   
   /* space for cost function computation */
   double * J; /* space for the jacobian; 3xn */
   double * J2;
   
   /* rooted sdfs */
   int n_rsdfs;
   struct run_rsdf * rsdfs;
   
   /* tsr constraints applied */
   int n_contsrs;
   struct run_contsr ** contsrs;
   
   /* optional start stuff */
   struct tsr * start_tsr;
   int start_tsr_enabled[6]; /* xyzrpy */
   int start_tsr_k;
   int (*start_cost)(void * ptr, int n, double * point, double * cost, double * grad);
   void * start_cost_ptr;
   
   /* optional every_n constraint */
   struct tsr * everyn_tsr;
   int everyn_tsr_enabled[6]; /* xyzrpy */
   int everyn_tsr_k;
   
   /* ee_force stuff */
   double ee_force[3];
   double ee_force_at[3];
   double * ee_torque_weights;
   int ee_torque_weights_n;
   
   /* hmc/resampling stuff */
   int use_hmc;
   int hmc_resample_iter;
   double hmc_resample_lambda;
   gsl_rng * rng;
   
   /* timing */
   struct timespec ticks_fk;
   struct timespec ticks_jacobians;
   struct timespec ticks_selfcol;
   struct timespec ticks_pre_velsaccs;
   
   /* logging */
   FILE * fp_dat;
   
   /* the chomp itself */
   struct cd_chomp * c;
   int iter;
};

int sphere_cost_pre(struct run * r, struct cd_chomp * c, int m, double ** T_points)
{
   int ti;
   int ti_mov;
   int i;
   int j;
   int sai;
   struct run_sphere * sact;
   boost::multi_array< OpenRAVE::dReal, 2 > orjacobian;
   double * internal;
   double Jsp[6][7];
   double pose[7];
   double Xm[6][6];
   
   cd_kin_pose_identity(pose);
   
   /* compute positions of all active spheres (including external traj points)
    * output:
    *  - r->sphere_poss_all has sphere locations
    *  - r->sphere_jacs has sphere jacobians */
   for (ti=0; ti<r->n_points; ti++)
   {
      /* put the robot in the config */
      if (r->floating_base)
      {
         /* save this for later;
          * maps root pose derivs to world spatial velocity of robot */
         cd_spatial_pose_jac(&r->traj[ti*c->n + 0], Jsp);
         if (0 && ti == 1)
         {
            RAVELOG_DEBUG("Jsp:\n");
            for (i=0; i<6; i++)
            {
               std::string buf;
               for (j=0; j<7; j++)
                  buf += sf(" % f", Jsp[i][j]);
               RAVELOG_DEBUG("[%s ]\n", buf.c_str());
            }
         }
         /* set base */
         OpenRAVE::Transform t;
         t.trans.x = r->traj[ti*c->n + 0];
         t.trans.y = r->traj[ti*c->n + 1];
         t.trans.z = r->traj[ti*c->n + 2];
         t.rot.y = r->traj[ti*c->n + 3];
         t.rot.z = r->traj[ti*c->n + 4];
         t.rot.w = r->traj[ti*c->n + 5];
         t.rot.x = r->traj[ti*c->n + 6];
         r->robot->SetTransform(t);
         std::vector<OpenRAVE::dReal> vec(&r->traj[ti*c->n+7], &r->traj[(ti+1)*c->n]);
         TIC()
         r->robot->SetActiveDOFValues(vec);
         TOC(&r->ticks_fk)
      }
      else
      {
         std::vector<OpenRAVE::dReal> vec(&r->traj[ti*c->n], &r->traj[(ti+1)*c->n]);
         TIC()
         r->robot->SetActiveDOFValues(vec);
         TOC(&r->ticks_fk)
      }
      
      /* compute positions and jacobians of all active spheres */
      for (sai=0,sact=r->spheres; sai<r->n_spheres_active; sai++,sact=sact->next)
      {
         OpenRAVE::Transform t = sact->robot_link->GetTransform();
         OpenRAVE::Vector v = t * OpenRAVE::Vector(sact->pos_wrt_link); /* copies 3 values */
         /* save sphere positions */
         r->sphere_poss_all[ti*(r->n_spheres_active*3) + sai*3 + 0] = v.x;
         r->sphere_poss_all[ti*(r->n_spheres_active*3) + sai*3 + 1] = v.y;
         r->sphere_poss_all[ti*(r->n_spheres_active*3) + sai*3 + 2] = v.z;
         /* get the moving point index */
         if (c->m == r->n_points - 2)
            ti_mov = ti-1;
         else
            ti_mov = ti;
         /* compute linear jacobians for all moving points */
         if (ti_mov < 0 || c->m <= ti_mov)
            continue;
         TIC()
         r->robot->CalculateJacobian(sact->robot_linkindex, v, orjacobian);
         TOC(&r->ticks_jacobians)
         if (r->floating_base)
         {
            /* get motion transform from world velocity to sphere-aligned-inertial-frame velocity */
            pose[0] = -v.x;
            pose[1] = -v.y;
            pose[2] = -v.z;
            cd_spatial_xm_from_pose(Xm, pose);
            
            /* get the left 3x7 part of the linear jacobian */
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 7, 6,
              1.0, &Xm[3][0],6, &Jsp[0][0],7, 0.0, &r->sphere_jacs[ti_mov*r->n_spheres_active*3*c->n + sai*3*c->n],c->n);
            
            if (0 && ti == 1 && sai == 0)
            {
               RAVELOG_DEBUG("sphere located at: %f %f %f\n", pose[0], pose[1], pose[2]);
               RAVELOG_DEBUG("sphere jacobian:\n");
               for (i=0; i<3; i++)
               {
                  std::string buf;
                  for (j=0; j<7; j++)
                     buf += sf(" % f", r->sphere_jacs[ti_mov*r->n_spheres_active*3*c->n + sai*3*c->n + i*c->n + j]);
                  RAVELOG_DEBUG("[ %s]\n", buf.c_str());
               }
            }
            
#if 1
            /* overwrite with zeros */
            for (i=0; i<3; i++)
               for (j=0; j<7; j++)
                  r->sphere_jacs[ti_mov*r->n_spheres_active*3*c->n + sai*3*c->n + i*c->n + j] *= 0.01;
#endif
           
            /* copy the active columns of orjacobian into our J */
            for (i=0; i<3; i++)
               for (j=0; j<r->n_adof; j++)
                  r->sphere_jacs[ti_mov*r->n_spheres_active*3*c->n + sai*3*c->n + i*c->n + 7+j] = orjacobian[i][r->adofindices[j]];
         }
         else
         {
            /* copy the active columns of orjacobian into our J */
            for (i=0; i<3; i++)
               for (j=0; j<r->n_adof; j++)
                  r->sphere_jacs[ti_mov*r->n_spheres_active*3*c->n + sai*3*c->n + i*c->n + j] = orjacobian[i][r->adofindices[j]];
         }
      }
   }
   
   TIC()
   
   /* compute velocities for all active spheres */
   /* start by computing internal velocities */
   internal = r->sphere_vels;
   if (c->m != r->n_points - 2)
      internal += r->n_spheres_active*3;
   cd_mat_memcpy(internal, r->sphere_poss_all + 2*r->n_spheres_active*3, r->n_points-2, r->n_spheres_active*3);
   cd_mat_sub(internal, r->sphere_poss_all, r->n_points-2, r->n_spheres_active*3);
   cd_mat_scale(internal, r->n_points-2, r->n_spheres_active*3, 1.0/(2.0*c->dt));
   /* next do the start vel */
   if (c->m != r->n_points - 2)
   {
      cd_mat_memcpy(r->sphere_vels, r->sphere_poss_all + r->n_spheres_active*3, 1, r->n_spheres_active*3);
      cd_mat_sub(r->sphere_vels, r->sphere_poss_all, 1, r->n_spheres_active*3);
      cd_mat_scale(r->sphere_vels, 1, r->n_spheres_active*3, 1.0/(c->dt));
   }
   
   /* compute accelerations for all active spheres */
   /* start by computing internal accelerations */
   internal = r->sphere_accs;
   if (c->m != r->n_points - 2)
      internal += r->n_spheres_active*3;
   cd_mat_memcpy(internal, r->sphere_poss_all + r->n_spheres_active*3, r->n_points-2, r->n_spheres_active*3);
   cd_mat_scale(internal, r->n_points-2, r->n_spheres_active*3, -2.0);
   cd_mat_add(internal, r->sphere_poss_all, r->n_points-2, r->n_spheres_active*3);
   cd_mat_add(internal, r->sphere_poss_all + 2*r->n_spheres_active*3, r->n_points-2, r->n_spheres_active*3);
   cd_mat_scale(internal, r->n_points-2, r->n_spheres_active*3, 1.0/(c->dt * c->dt));
   /* simply copy 1st accel into the 0th accel for now */
   if (c->m != r->n_points - 2)
      cd_mat_memcpy(r->sphere_accs, r->sphere_accs + r->n_spheres_active*3, 1, r->n_spheres_active*3);
   
   TOC(&r->ticks_pre_velsaccs)
   
   return 0;
}

int sphere_cost(struct run * r, struct cd_chomp * c, int ti, double * c_point, double * c_vel, double * costp, double * c_grad)
{
   int i;
   int err;
   double * x_vel;
   double x_vel_norm;
   double g_point[3];
   double dist;
   double cost;
   double cost_sphere;
   double g_grad[3];
   double x_grad[3];
   double proj;
   double x_curv[3];
   struct run_sphere * sact;
   struct run_sphere * sact2;
   int sdfi_best;
   double sdfi_best_dist;
   int sai;
   int sai2;
   double v_from_other[3];
   
   /* start with a zero cost and config-space gradient */
   cost = 0.0;
   if (c_grad) cd_mat_set_zero(c_grad, c->n, 1);
   
   /* the cost and its gradient are summed over each active sphere on the robot */
   for (sai=0,sact=r->spheres; sai<r->n_spheres_active; sai++,sact=sact->next)
   {
      cost_sphere = 0.0;

      /* grab the current workspace velocity of the sphere */
      x_vel = r->sphere_vels + ti*(r->n_spheres_active*3) + sai*3;
      x_vel_norm = cblas_dnrm2(3, x_vel, 1);
      
      /* compute which distance field is closest to an obstacle;
       * this finds the field with the smallest value (even negative) */
      sdfi_best_dist = HUGE_VAL;
      sdfi_best = -1;
      for (i=0; i<r->n_rsdfs; i++)
      {
         /* transform sphere center into grid frame */
         cd_kin_pose_compos(r->rsdfs[i].pose_gsdf_world,
            r->sphere_poss + ti*(r->n_spheres_active*3) + sai*3,
            g_point);
         /* get sdf value (from interp) */
         err = cd_grid_double_interp(r->rsdfs[i].grid, g_point, &dist);
         if (err)
            continue; /* not inside of this distance field at all! */
         if (dist < sdfi_best_dist)
         {
            sdfi_best_dist = dist;
            sdfi_best = i;
         }
      }
      if (sdfi_best != -1)
      {
         /* re-transform sphere center into grid frame */
         cd_kin_pose_compos(r->rsdfs[sdfi_best].pose_gsdf_world,
            r->sphere_poss + ti*(r->n_spheres_active*3) + sai*3,
            g_point);
         /* get sdf value (from interp) */
         cd_grid_double_interp(r->rsdfs[sdfi_best].grid, g_point, &dist);
         /* subtract radius to get distance of closest sphere point to closest obstacle */
         dist -= sact->radius;
         
         /* convert to a cost, scaled by sphere velocity;
          * epsilon is distance outside of obstacles where cost starts */
         if (dist < 0.0)
            cost_sphere += x_vel_norm * r->obs_factor * (0.5 * r->epsilon - dist);
         else if (dist < r->epsilon)
            cost_sphere += x_vel_norm * r->obs_factor * (0.5/r->epsilon) * (dist - r->epsilon) * (dist - r->epsilon);
         
         if (c_grad)
         {
            /* get sdf gradient */
            /* this will be a unit vector away from closest obs */
            cd_grid_double_grad(r->rsdfs[sdfi_best].grid, g_point, g_grad);
            /* now in world frame */
            cd_kin_pose_compose_vec(r->rsdfs[sdfi_best].pose_world_gsdf, g_grad, g_grad);
            
            /* convert sdf g_grad to x_grad (w.r.t. cost) according to dist */
            cd_mat_memcpy(x_grad, g_grad, 3, 1);
            if (dist < 0.0)
               cd_mat_scale(x_grad, 3, 1, -1.0);
            else if (dist < r->epsilon)
               cd_mat_scale(x_grad, 3, 1, dist/r->epsilon - 1.0);
            else
               cd_mat_set_zero(x_grad, 3, 1);
            cd_mat_scale(x_grad, 3, 1, x_vel_norm * r->obs_factor);
            
            /* subtract from x_grad its vector projection onto x_vel */
            if (x_vel_norm > 0.000001)
            {
               proj = cblas_ddot(3, x_grad,1, x_vel,1) / (x_vel_norm * x_vel_norm);
               cblas_daxpy(3, -proj, x_vel,1, x_grad,1);
            }
            
            /* compute curvature vector */
            cd_mat_memcpy(x_curv, r->sphere_accs + ti*(r->n_spheres_active*3) + sai*3, 3, 1);
            if (x_vel_norm > 0.000001)
            {
               proj = cblas_ddot(3, x_curv,1, x_vel,1) / (x_vel_norm * x_vel_norm);
               cblas_daxpy(3, -proj, x_vel,1, x_curv,1);
            }
            cd_mat_scale(x_curv, 3, 1, 1.0 / (x_vel_norm * x_vel_norm));
            /* add that in to x_grad */
            cblas_daxpy(3, -cost_sphere, x_curv,1, x_grad,1);
            
            /* multiply into c_grad through JT, scaled by sphere velocity */
            cblas_dgemv(CblasRowMajor, CblasTrans, 3, c->n,
               x_vel_norm, r->sphere_jacs + ti*r->n_spheres_active*3*c->n + sai*3*c->n,c->n, x_grad,1, 1.0, c_grad,1);
         }
      }

      /* consider effects from all other spheres (i.e. self collision) */
      TIC()
      for (sai2=0,sact2=r->spheres; sai2<r->n_spheres; sai2++,sact2=sact2->next)
      {
         /*if (sai2 >= r->n_spheres_active) continue;*/ /* SKIP ACTIVE-INACTIVE SPHERE GRADIENTS FOR NOW */
         
         /* skip spheres on the same link (their Js would be identical anyways) */
         if (sact->robot_linkindex == sact2->robot_linkindex) continue;
         
         /* compute vector from our (sai) location to their (sai2) location;
          * whats the deal with this crazy sphere indexing? */
         cd_mat_memcpy(v_from_other, r->sphere_poss + ti*(r->n_spheres_active*3) + sai*3, 3, 1);
         if (sai2<r->n_spheres_active)
            cd_mat_sub(v_from_other, r->sphere_poss + ti*(r->n_spheres_active*3) + sai2*3, 3, 1);
         else
            cd_mat_sub(v_from_other, r->sphere_poss_inactive + (sai2-r->n_spheres_active)*3, 3, 1);
         dist = cblas_dnrm2(3, v_from_other,1);
         
         /* skip spheres far enough away from us */
         if (dist > sact->radius + sact2->radius + r->epsilon_self) continue;
         
         if (c_grad)
         {
            /* make unit vector (g_grad) away from other sphere */
            g_grad[0] = v_from_other[0] / dist;
            g_grad[1] = v_from_other[1] / dist;
            g_grad[2] = v_from_other[2] / dist;
         }
         
         /* actual distance between sphere surfaces */
         dist -= sact->radius + sact2->radius;
         
         if (costp)
         {
            /* compute the cost */
            if (dist < 0.0)
               cost_sphere += x_vel_norm * r->obs_factor_self * (0.5 * r->epsilon_self - dist);
            else
               cost_sphere += x_vel_norm * r->obs_factor_self * (0.5/r->epsilon_self) * (dist - r->epsilon_self) * (dist - r->epsilon_self);
            /*printf("   from extra sphere-sphere interaction, sphere cost now: %f\n", cost_sphere);*/
         }
         
         if (c_grad)
         {
            /* convert sdf g_grad to x_grad (w.r.t. cost) according to dist */
            cd_mat_memcpy(x_grad, g_grad, 3, 1);
            if (dist < 0.0)
               cd_mat_scale(x_grad, 3, 1, -1.0);
            else if (dist < r->epsilon_self)
               cd_mat_scale(x_grad, 3, 1, dist/r->epsilon_self - 1.0);
            cd_mat_scale(x_grad, 3, 1, x_vel_norm * r->obs_factor_self);
            
            /* subtract from x_grad vector projection onto x_vel */
            if (x_vel_norm > 0.000001)
            {
               proj = cblas_ddot(3, x_grad,1, x_vel,1) / (x_vel_norm * x_vel_norm);
               cblas_daxpy(3, -proj, x_vel,1, x_grad,1);
            }
            
            /* J2 = J - jacobian of other sphere*/
            cd_mat_memcpy(r->J2, r->sphere_jacs + ti*r->n_spheres_active*3*c->n + sai*3*c->n, 3, c->n);
            if (sai2<r->n_spheres_active)
               cd_mat_sub(r->J2, r->sphere_jacs + ti*r->n_spheres_active*3*c->n + sai2*3*c->n, 3, c->n);
            
            /* multiply into c_grad through JT */
            cblas_dgemv(CblasRowMajor, CblasTrans, 3, c->n,
               1.0, r->J2,c->n, x_grad,1, 1.0, c_grad,1);
         }
      }
      TOC(&r->ticks_selfcol)
      
      cost += cost_sphere;
   }
   
   /* we potentially add the ee_force cost and its gradient ... */
   
   if (costp) *costp = cost;
   return 0;
}


int con_tsr(struct run_contsr * contsr, struct cd_chomp * c, int ti, double * point, double * con_val, double * con_jacobian)
{
   int tsri;
   int ki;
   int i;
   int j;
   struct run * r;
   double pose_ee[7];
   double pose_obj[7];
   double pose_ee_obj[7];
   double pose_table_world[7];
   double pose_table_obj[7];
   double xyzypr_table_obj[7];
   
   double Jsp[6][7];
   double * spajac_world;
   int ee_link_index;
   boost::multi_array< OpenRAVE::dReal, 2 > orjacobian;
   double xm_table_world[6][6];
   double jac_inverse[7][6];
   double pose_to_xyzypr_jac[6][7];
   double * full_result;
   double temp6x6a[6][6];
   double temp6x6b[6][6];
   
   r = contsr->r;
   
   /* put the arm in this configuration */
   if (r->floating_base)
   {
      OpenRAVE::Transform t;
      t.trans.x = point[0];
      t.trans.y = point[1];
      t.trans.z = point[2];
      t.rot.y = point[3];
      t.rot.z = point[4];
      t.rot.w = point[5];
      t.rot.x = point[6];
      r->robot->SetTransform(t);
      std::vector<OpenRAVE::dReal> vec(point + 7, point + c->n);
      r->robot->SetActiveDOFValues(vec);
   }
   else
   {
      std::vector<OpenRAVE::dReal> vec(point, point + c->n);
      r->robot->SetActiveDOFValues(vec);
   }
   
   /* First, we calculate the vector value of the constraint function.
    * This is simply the Bw transform (the middle one in the TSR) as xyzypr.
    */
   
   /* get the end-effector transform */
   OpenRAVE::Transform t;
   if (contsr->manip)
      t = contsr->manip->GetEndEffectorTransform();
   else
      t = contsr->link->GetTransform();
   pose_ee[0] = t.trans.x;
   pose_ee[1] = t.trans.y;
   pose_ee[2] = t.trans.z;
   pose_ee[3] = t.rot.y;
   pose_ee[4] = t.rot.z;
   pose_ee[5] = t.rot.w;
   pose_ee[6] = t.rot.x;
   
   /* get the object pose */
   cd_kin_pose_invert(contsr->tsr->Twe, pose_ee_obj);
   cd_kin_pose_compose(pose_ee, pose_ee_obj, pose_obj);
   
   /* get the pose of the world w.r.t. the table */
   cd_kin_pose_invert(contsr->tsr->T0w, pose_table_world);
   
   /* get the pose of the object w.r.t. the table */
   cd_kin_pose_compose(pose_table_world, pose_obj, pose_table_obj);
   
   /* convert to xyzypr */
   cd_kin_pose_to_xyzypr(pose_table_obj, xyzypr_table_obj);
   
   /* fill the constraint value vector */
   ki=0;
   for (tsri=0; tsri<6; tsri++) if (contsr->tsr_enabled[tsri])
   {
      con_val[ki] = xyzypr_table_obj[tsri<3?tsri:8-tsri];
      ki++;
   }
   
   /* Second, we get the constraint Jacobian, wrt the trajectory point vector,
    * whose first 7 components are the root pose (if floating_base is True).
    */
   
   if (con_jacobian)
   {
      /* compute the spatial Jacobian! */
      spajac_world = (double *) malloc(6 * c->n * sizeof(double));
      full_result = (double *) malloc(6 * c->n * sizeof(double));
      
      if (contsr->manip)
         ee_link_index = contsr->manip->GetEndEffector()->GetIndex();
      else
         ee_link_index = contsr->link->GetIndex();
      
      if (r->floating_base)
      {
        /* get floating base, first seven columns of spatial velocity jacobian */
        cd_spatial_pose_jac(point, Jsp);
        for (i=0; i<6; i++)
          cd_mat_memcpy(&spajac_world[i*c->n+0], &Jsp[i][0], 1, 7);
        
        /* copy the active columns of (rotational) orjacobian into our J */
        r->robot->CalculateAngularVelocityJacobian(ee_link_index, orjacobian);
        for (i=0; i<3; i++)
           for (j=0; j<r->n_adof; j++)
              spajac_world[i*c->n+7+j] = orjacobian[i][r->adofindices[j]];
        
        /* copy the active columns of (translational) orjacobian into our J */
        r->robot->CalculateJacobian(ee_link_index, OpenRAVE::Vector(0.0, 0.0, 0.0), orjacobian);
        for (i=0; i<3; i++)
           for (j=0; j<r->n_adof; j++)
              spajac_world[(3+i)*c->n+7+j] = orjacobian[i][r->adofindices[j]];
      }
      else
      {
        /* copy the active columns of (rotational) orjacobian into our J */
        r->robot->CalculateAngularVelocityJacobian(ee_link_index, orjacobian);
        for (i=0; i<3; i++)
           for (j=0; j<r->n_adof; j++)
              spajac_world[i*c->n+j] = orjacobian[i][r->adofindices[j]];
        
        /* copy the active columns of (translational) orjacobian into our J */
        r->robot->CalculateJacobian(ee_link_index, OpenRAVE::Vector(0.0, 0.0, 0.0), orjacobian);
        for (i=0; i<3; i++)
           for (j=0; j<r->n_adof; j++)
              spajac_world[(3+i)*c->n+j] = orjacobian[i][r->adofindices[j]];
      }
      
      /* compute the spatial transform to get velocities in table frame */
      cd_spatial_xm_from_pose(xm_table_world, pose_table_world);
      
      /* calculate the pose derivative Jacobian matrix */
      cd_spatial_pose_jac_inverse(pose_table_obj, jac_inverse);
      
      /* calculate the xyzypr Jacobian matrix */
      cd_kin_pose_to_xyzypr_J(pose_table_obj, pose_to_xyzypr_jac);
      
      /* do the matrix multiplications! */
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 7,
         1.0, *pose_to_xyzypr_jac,7, *jac_inverse,6, 0.0, *temp6x6a,6);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 6,
         1.0, *temp6x6a,6, *xm_table_world,6, 0.0, *temp6x6b,6);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, c->n, 6,
         1.0, *temp6x6b,6, spajac_world,c->n, 0.0, full_result,c->n);
      
      /* fill the constraint Jacbobian matrix,
       * by copying the correct rows of full_result */
      ki=0;
      for (tsri=0; tsri<6; tsri++) if (contsr->tsr_enabled[tsri])
      {
         cd_mat_memcpy(con_jacobian+ki*c->n, full_result+(tsri<3?tsri:8-tsri)*c->n, 1, c->n);
         ki++;
      }
      
      free(spajac_world);
      free(full_result);
   }
   
   return 0;
}


int con_everyn_tsr(struct run * r, struct cd_chomp * c, int ti, double * point, double * con_val, double * con_jacobian)
{
   int tsri;
   int ki;
   int i;
   int j;
   double pose_ee[7];
   double pose_obj[7];
   double pose_ee_obj[7];
   double pose_table_world[7];
   double pose_table_obj[7];
   double xyzypr_table_obj[7];
   
   double Jsp[6][7];
   double * spajac_world;
   int ee_link_index;
   boost::multi_array< OpenRAVE::dReal, 2 > orjacobian;
   double xm_table_world[6][6];
   double jac_inverse[7][6];
   double pose_to_xyzypr_jac[6][7];
   double * full_result;
   double temp6x6a[6][6];
   double temp6x6b[6][6];
   
   /* put the arm in this configuration */
   if (r->floating_base)
   {
      OpenRAVE::Transform t;
      t.trans.x = point[0];
      t.trans.y = point[1];
      t.trans.z = point[2];
      t.rot.y = point[3];
      t.rot.z = point[4];
      t.rot.w = point[5];
      t.rot.x = point[6];
      r->robot->SetTransform(t);
      std::vector<OpenRAVE::dReal> vec(point + 7, point + c->n);
      r->robot->SetActiveDOFValues(vec);
   }
   else
   {
      std::vector<OpenRAVE::dReal> vec(point, point + c->n);
      r->robot->SetActiveDOFValues(vec);
   }
   
   /* First, we calculate the vector value of the constraint function.
    * This is simply the Bw transform (the middle one in the TSR) as xyzypr.
    */
   
   /* get the end-effector transform */
   OpenRAVE::Transform t = r->robot->GetActiveManipulator()->GetEndEffectorTransform();
   pose_ee[0] = t.trans.x;
   pose_ee[1] = t.trans.y;
   pose_ee[2] = t.trans.z;
   pose_ee[3] = t.rot.y;
   pose_ee[4] = t.rot.z;
   pose_ee[5] = t.rot.w;
   pose_ee[6] = t.rot.x;
   
   /* get the object pose */
   cd_kin_pose_invert(r->everyn_tsr->Twe, pose_ee_obj);
   cd_kin_pose_compose(pose_ee, pose_ee_obj, pose_obj);
   
   /* get the pose of the world w.r.t. the table */
   cd_kin_pose_invert(r->everyn_tsr->T0w, pose_table_world);
   
   /* get the pose of the object w.r.t. the table */
   cd_kin_pose_compose(pose_table_world, pose_obj, pose_table_obj);
   
   /* convert to xyzypr */
   cd_kin_pose_to_xyzypr(pose_table_obj, xyzypr_table_obj);
   
   /* fill the constraint value vector */
   ki=0;
   for (tsri=0; tsri<6; tsri++) if (r->everyn_tsr_enabled[tsri])
   {
      con_val[ki] = xyzypr_table_obj[tsri<3?tsri:8-tsri];
      ki++;
   }
   
   /* Second, we get the constraint Jacobian, wrt the trajectory point vector,
    * whose first 7 components are the root pose (if floating_base is True).
    */
   
   if (con_jacobian)
   {
      /* compute the spatial Jacobian! */
      spajac_world = (double *) malloc(6 * c->n * sizeof(double));
      full_result = (double *) malloc(6 * c->n * sizeof(double));
      
      ee_link_index = r->robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
      
      if (r->floating_base)
      {
        /* get floating base, first seven columns of spatial velocity jacobian */
        cd_spatial_pose_jac(point, Jsp);
        for (i=0; i<6; i++)
          cd_mat_memcpy(&spajac_world[i*c->n+0], &Jsp[i][0], 1, 7);
        
        /* copy the active columns of (rotational) orjacobian into our J */
        r->robot->CalculateAngularVelocityJacobian(ee_link_index, orjacobian);
        for (i=0; i<3; i++)
           for (j=0; j<r->n_adof; j++)
              spajac_world[i*c->n+7+j] = orjacobian[i][r->adofindices[j]];
        
        /* copy the active columns of (translational) orjacobian into our J */
        r->robot->CalculateJacobian(ee_link_index, OpenRAVE::Vector(0.0, 0.0, 0.0), orjacobian);
        for (i=0; i<3; i++)
           for (j=0; j<r->n_adof; j++)
              spajac_world[(3+i)*c->n+7+j] = orjacobian[i][r->adofindices[j]];
      }
      else
      {
        /* copy the active columns of (rotational) orjacobian into our J */
        r->robot->CalculateAngularVelocityJacobian(ee_link_index, orjacobian);
        for (i=0; i<3; i++)
           for (j=0; j<r->n_adof; j++)
              spajac_world[i*c->n+j] = orjacobian[i][r->adofindices[j]];
        
        /* copy the active columns of (translational) orjacobian into our J */
        r->robot->CalculateJacobian(ee_link_index, OpenRAVE::Vector(0.0, 0.0, 0.0), orjacobian);
        for (i=0; i<3; i++)
           for (j=0; j<r->n_adof; j++)
              spajac_world[(3+i)*c->n+j] = orjacobian[i][r->adofindices[j]];
      }
      
      /* compute the spatial transform to get velocities in table frame */
      cd_spatial_xm_from_pose(xm_table_world, pose_table_world);
      
      /* calculate the pose derivative Jacobian matrix */
      cd_spatial_pose_jac_inverse(pose_table_obj, jac_inverse);
      
      /* calculate the xyzypr Jacobian matrix */
      cd_kin_pose_to_xyzypr_J(pose_table_obj, pose_to_xyzypr_jac);
      
      /* do the matrix multiplications! */
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 7,
         1.0, *pose_to_xyzypr_jac,7, *jac_inverse,6, 0.0, *temp6x6a,6);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 6,
         1.0, *temp6x6a,6, *xm_table_world,6, 0.0, *temp6x6b,6);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, c->n, 6,
         1.0, *temp6x6b,6, spajac_world,c->n, 0.0, full_result,c->n);
      
      /* fill the constraint Jacbobian matrix,
       * by copying the correct rows of full_result */
      ki=0;
      for (tsri=0; tsri<6; tsri++) if (r->everyn_tsr_enabled[tsri])
      {
         cd_mat_memcpy(con_jacobian+ki*c->n, full_result+(tsri<3?tsri:8-tsri)*c->n, 1, c->n);
         ki++;
      }
      
      free(spajac_world);
      free(full_result);
   }
   
   return 0;
}

int con_start_tsr(struct run * r, struct cd_chomp * c, int ti, double * point, double * con_val, double * con_jacobian)
{
   int tsri;
   int ki;
   int i;
   int j;
   double pose_ee[7];
   double pose_obj[7];
   double pose_ee_obj[7];
   double pose_table_world[7];
   double pose_table_obj[7];
   double xyzypr_table_obj[7];
   
   double * spajac_world;
   int ee_link_index;
   boost::multi_array< OpenRAVE::dReal, 2 > orjacobian;
   double xm_table_world[6][6];
   double jac_inverse[7][6];
   double pose_to_xyzypr_jac[6][7];
   double * full_result;
   double temp6x6a[6][6];
   double temp6x6b[6][6];
   
   /* put the arm in this configuration */
   if (r->floating_base)
   {
      OpenRAVE::Transform t;
      t.trans.x = point[0];
      t.trans.y = point[1];
      t.trans.z = point[2];
      t.rot.y = point[3];
      t.rot.z = point[4];
      t.rot.w = point[5];
      t.rot.x = point[6];
      r->robot->SetTransform(t);
      std::vector<OpenRAVE::dReal> vec(point + 7, point + c->n);
      r->robot->SetActiveDOFValues(vec);
   }
   else
   {
      std::vector<OpenRAVE::dReal> vec(point, point + c->n);
      r->robot->SetActiveDOFValues(vec);
   }
   
   /* get the end-effector transform */
   OpenRAVE::Transform t = r->robot->GetActiveManipulator()->GetEndEffectorTransform();
   pose_ee[0] = t.trans.x;
   pose_ee[1] = t.trans.y;
   pose_ee[2] = t.trans.z;
   pose_ee[3] = t.rot.y;
   pose_ee[4] = t.rot.z;
   pose_ee[5] = t.rot.w;
   pose_ee[6] = t.rot.x;
   
   /* get the object pose */
   cd_kin_pose_invert(r->start_tsr->Twe, pose_ee_obj);
   cd_kin_pose_compose(pose_ee, pose_ee_obj, pose_obj);
   
   /* get the pose of the world w.r.t. the table */
   cd_kin_pose_invert(r->start_tsr->T0w, pose_table_world);
   
   /* get the pose of the object w.r.t. the table */
   cd_kin_pose_compose(pose_table_world, pose_obj, pose_table_obj);
   
   /* convert to xyzypr */
   cd_kin_pose_to_xyzypr(pose_table_obj, xyzypr_table_obj);
   
   /* fill the constraint value vector */
   ki=0;
   for (tsri=0; tsri<6; tsri++) if (r->start_tsr_enabled[tsri])
   {
      con_val[ki] = xyzypr_table_obj[tsri<3?tsri:8-tsri];
      ki++;
   }
   
   if (con_jacobian)
   {
      /* compute the spatial Jacobian! */
      spajac_world = (double *) malloc(6 * c->n * sizeof(double));
      full_result = (double *) malloc(6 * c->n * sizeof(double));
      
      ee_link_index = r->robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
      
      /* copy the active columns of (rotational) orjacobian into our J */
      r->robot->CalculateAngularVelocityJacobian(ee_link_index, orjacobian);
      for (i=0; i<3; i++)
         for (j=0; j<c->n; j++)
            spajac_world[i*c->n+j] = orjacobian[i][r->adofindices[j]];
      
      /* copy the active columns of (translational) orjacobian into our J */
      r->robot->CalculateJacobian(ee_link_index, OpenRAVE::Vector(0.0, 0.0, 0.0), orjacobian);
      for (i=0; i<3; i++)
         for (j=0; j<c->n; j++)
            spajac_world[(3+i)*c->n+j] = orjacobian[i][r->adofindices[j]];
            
      /* compute the spatial transform to get velocities in table frame */
      cd_spatial_xm_from_pose(xm_table_world, pose_table_world);
      
      /* calculate the pose derivative Jacobian matrix */
      cd_spatial_pose_jac_inverse(pose_table_obj, jac_inverse);
      
      /* calculate the xyzypr Jacobian matrix */
      cd_kin_pose_to_xyzypr_J(pose_table_obj, pose_to_xyzypr_jac);
      
      /* do the matrix multiplications! */
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 7,
         1.0, *pose_to_xyzypr_jac,7, *jac_inverse,6, 0.0, *temp6x6a,6);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 6,
         1.0, *temp6x6a,6, *xm_table_world,6, 0.0, *temp6x6b,6);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, c->n, 6,
         1.0, *temp6x6b,6, spajac_world,c->n, 0.0, full_result,c->n);
      
      /* fill the constraint Jacbobian matrix */
      ki=0;
      for (tsri=0; tsri<6; tsri++) if (r->start_tsr_enabled[tsri])
      {
         cd_mat_memcpy(con_jacobian+ki*c->n, full_result+(tsri<3?tsri:8-tsri)*c->n, 1, c->n);
         ki++;
      }
      
      free(spajac_world);
      free(full_result);
   }
   
   return 0;
}

/* implements a cost and gradient on the start point,
 * implemented by the passed start_cost callback function */
int cost_extra_start(struct run * r, struct cd_chomp * c, double * T, double * costp, double * G)
{
   if (!r->start_cost) return 0;
   r->start_cost(r->start_cost_ptr, c->n, T, costp, G);
   return 0;
}

/* runchomp robot Herb2
 * run chomp from the current config to the passed goal config
 * uses the active dofs of the passed robot
 * initialized with a straight-line trajectory
 * */
int mod::create(int argc, char * argv[], std::ostream& sout)
{
   int i;
   unsigned int ui;
   int j;
   int err;
   int nscan;
   struct orcdchomp::sphere * s;
   const char * exc = 0;
   
   /* our object(s) */
   struct run * r = 0;
   struct cd_chomp * c = 0;
   
   /* temporary args from the command line */
   int n_adofgoal = 0; /* size read from arguments */
   double * adofgoal = 0;
   double * basegoal = 0;
   unsigned int seed = 0;
   char * dat_filename = 0;
   
   /* loaded into chomp */
   int m;
   int n; /* chomp dof */
   double lambda = 10.0;
   int use_momentum = 0;
   int D = 1; /* for cd_chomp_create */

   /* lock environment; other temporaries */
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
   std::vector< OpenRAVE::dReal > vec_jlimit_lower;
   std::vector< OpenRAVE::dReal > vec_jlimit_upper;
   OpenRAVE::TrajectoryBasePtr starttraj;
   struct run_sphere * s_inactive_head;
   
   r = (struct run *) malloc(sizeof(struct run));
   if (!r) throw OpenRAVE::openrave_exception("no memory!");
   
   /* initialize */
   r->traj = 0;
   r->n_points = 101;
   r->robot = 0;
   r->n_adof = 0;
   r->floating_base = 0;
   r->adofindices = 0;
   r->epsilon = 0.1; /* in meters */
   r->epsilon_self = 0.04; /* in meters */
   r->obs_factor = 200.0;
   r->obs_factor_self = 10.0;
   r->spheres = 0;
   r->n_spheres = 0;
   r->n_spheres_active = 0;
   r->sphere_poss_inactive = 0;
   r->sphere_poss_all = 0;
   r->sphere_vels = 0;
   r->sphere_accs = 0;
   r->sphere_jacs = 0;
   r->n_rsdfs = 0;
   r->rsdfs = 0;
   r->n_contsrs = 0;
   r->contsrs = 0;
   r->start_tsr = 0;
   for (i=0; i<6; i++) r->start_tsr_enabled[i] = 0;
   r->start_tsr_k = 0;
   r->start_cost = 0;
   r->start_cost_ptr = 0;
   r->everyn_tsr = 0;
   for (i=0; i<6; i++) r->everyn_tsr_enabled[i] = 0;
   r->everyn_tsr_k = 0;
   cd_mat_set_zero(r->ee_force, 3, 1);
   cd_mat_set_zero(r->ee_force_at, 3, 1);
   r->ee_torque_weights = 0;
   r->ee_torque_weights_n = 0;
   r->use_hmc = 0;
   r->hmc_resample_iter = 0;
   r->hmc_resample_lambda = 0.02; /* parameter of exponential distribution over iterations between resamples */
   r->rng = 0;
   r->J = 0;
   r->J2 = 0;
   cd_os_timespec_set_zero(&r->ticks_fk);
   cd_os_timespec_set_zero(&r->ticks_jacobians);
   cd_os_timespec_set_zero(&r->ticks_selfcol);
   cd_os_timespec_set_zero(&r->ticks_pre_velsaccs);
   r->fp_dat = 0;
   r->c = 0;
   r->iter = 0;
   
   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"robot")==0 && i+1<argc)
      {
         
         if (r->robot) { exc = "Only one robot can be passed!"; goto error; }
         OpenRAVE::RobotBasePtr rob = this->e->GetRobot(argv[++i]);
         r->robot = rob.get();
         if (!r->robot) { exc = "Could not find robot with that name!"; goto error; }
      }
      else if (strcmp(argv[i],"adofgoal")==0 && i+1<argc)
      {
         if (adofgoal) { exc = "Only one adofgoal can be passed!"; goto error; }
         if (starttraj.get()) { exc = "Cannot pass both adofgoal and starttraj!"; goto error; }
         {
            char ** adofgoal_argv = 0;
            cd_util_shparse(argv[++i], &n_adofgoal, &adofgoal_argv);
            adofgoal = (double *) malloc(n_adofgoal * sizeof(double));
            for (j=0; j<n_adofgoal; j++)
               adofgoal[j] = atof(adofgoal_argv[j]);
            free(adofgoal_argv);
         }
         RAVELOG_DEBUG("parsed adofgoal: %s\n", sf_vector(adofgoal,n_adofgoal).c_str());
      }
      else if (strcmp(argv[i],"basegoal")==0 && i+1<argc)
      {
         if (basegoal) { exc = "Only one basegoal can be passed!"; goto error; }
         if (starttraj.get()) { exc = "Cannot pass both basegoal and starttraj!"; goto error; }
         {
            char ** basegoal_argv = 0;
            int basegoal_argc;
            cd_util_shparse(argv[++i], &basegoal_argc, &basegoal_argv);
            if (basegoal_argc != 7) { free(basegoal_argv); exc = "basegoal argument must be length 7!"; goto error; }
            basegoal = (double *) malloc(7 * sizeof(double));
            for (j=0; j<7; j++)
               basegoal[j] = atof(basegoal_argv[j]);
            free(basegoal_argv);
         }
         RAVELOG_DEBUG("parsed basegoal: %s\n", sf_vector(basegoal,7).c_str());
      }
      else if (strcmp(argv[i],"floating_base")==0)
         r->floating_base = 1;
      else if (strcmp(argv[i],"con_tsr")==0 && i+2<argc)
      {
         struct run_contsr * contsr;
         char ** contsr_argv = 0;
         int contsr_argc;
         std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips;
         if (!r->robot) { exc = "You must pass robot before any con_tsrs!"; goto error; }
         manips = r->robot->GetManipulators();
         /* parse first arg, TYPE manip MANIPNAME or TYPE link LINKNAME or TYPE (for active manip ee) */
         cd_util_shparse(argv[++i], &contsr_argc, &contsr_argv);
         if (contsr_argc != 1 && contsr_argc != 3) { free(contsr_argv); exc = "con_tsr first argument must be length 1 or 3!"; goto error; }
         contsr = (struct run_contsr *) malloc(sizeof(struct run_contsr));
         if (!contsr) { free(contsr_argv); exc = "memory error!"; goto error; }
         contsr->r = r;
         contsr->tsr = 0;
         contsr->manip = 0;
         contsr->link = 0;
         if (strcmp(contsr_argv[0],"all")==0)
            contsr->type = RUN_CON_ALL;
#if 0 /* we have to redo the trajectory indexing stuff to support these generally */
         else if (strcmp(contsr_argv[0],"start")==0)
            contsr->type = RUN_CON_START;
         else if (strcmp(contsr_argv[0],"end")==0)
            contsr->type = RUN_CON_END;
#endif
         else
            { free(contsr); free(contsr_argv); exc = "con_tsr first arg must be start, end, or all!"; goto error; }
         if (contsr_argc != 3)
         {
            contsr->manip = r->robot->GetActiveManipulator().get();
         }
         else if (strcmp(contsr_argv[1],"manipee")==0)
         {
            /* find matching manip */
            for (ui=0; ui<manips.size(); ui++)
               if (strcmp(contsr_argv[2], manips[ui]->GetName().c_str())==0)
                  break;
            if (!(ui<manips.size()))
              { free(contsr); free(contsr_argv); exc = "con_tsr manip not found!"; goto error; }
            contsr->manip = manips[ui].get();
         }
         else if (strcmp(contsr_argv[1],"link")==0)
         {
            contsr->link = r->robot->GetLink(std::string(contsr_argv[2])).get();
            if (!contsr->link) { free(contsr); free(contsr_argv); exc = "con_tsr link not found!"; goto error; }
         }
         else
           { free(contsr); free(contsr_argv); exc = "con_tsr first arg must be empty, manipee, or link!"; goto error; }
         free(contsr_argv);
         /* parse second arg, the actual tsr */
         err = tsr_create_parse(&contsr->tsr, argv[++i]);
         if (err) { free(contsr); exc = "Cannot parse constraint TSR!"; goto error; }
         /* add to the constraint list! */
         r->contsrs = (struct run_contsr **) realloc(r->contsrs, (r->n_contsrs+1)*sizeof(struct run_contsr *));
         if (!r->contsrs) { free(contsr); exc = "memory error!"; goto error; }
         r->contsrs[r->n_contsrs] = contsr;
         r->n_contsrs++;
      }
      else if (strcmp(argv[i],"start_tsr")==0 && i+1<argc)
      {
         err = tsr_create_parse(&r->start_tsr, argv[++i]);
         if (err) { exc = "Cannot parse start_tsr TSR!"; goto error; }
      }
      else if (strcmp(argv[i],"everyn_tsr")==0 && i+1<argc)
      {
         err = tsr_create_parse(&r->everyn_tsr, argv[++i]);
         if (err) { exc = "Cannot parse everyn_tsr TSR!"; goto error; }
      }
      else if (strcmp(argv[i],"start_cost")==0 && i+1<argc)
      {
         nscan = sscanf(argv[++i], "%p %p", &r->start_cost, &r->start_cost_ptr);
         if (nscan != 2) { exc = "Cannot parse start_cost callback!"; goto error; }
      }
      else if (strcmp(argv[i],"lambda")==0 && i+1<argc)
         lambda = atof(argv[++i]);
      else if (strcmp(argv[i],"starttraj")==0 && i+1<argc)
      {
         if (starttraj.get()) { exc = "Only one starttraj can be passed!"; goto error; }
         if (adofgoal) { exc = "Cannot pass both adofgoal and starttraj!"; goto error; }
         starttraj = RaveCreateTrajectory(this->e);
         std::string my_string(argv[++i]);
         std::istringstream ser_iss(my_string);
         starttraj->deserialize(ser_iss);
      }
      else if (strcmp(argv[i],"n_points")==0 && i+1<argc)
         r->n_points = atoi(argv[++i]);
      else if (strcmp(argv[i],"derivative")==0 && i+1<argc)
         D = atoi(argv[++i]);
      else if (strcmp(argv[i],"use_momentum")==0)
         use_momentum = 1;
      else if (strcmp(argv[i],"use_hmc")==0)
         r->use_hmc = 1;
      else if (strcmp(argv[i],"hmc_resample_lambda")==0 && i+1<argc)
         r->hmc_resample_lambda = atof(argv[++i]);
      else if (strcmp(argv[i],"seed")==0 && i+1<argc)
         sscanf(argv[++i], "%u", &seed);
      else if (strcmp(argv[i],"epsilon")==0 && i+1<argc)
         r->epsilon = atof(argv[++i]);
      else if (strcmp(argv[i],"epsilon_self")==0 && i+1<argc)
         r->epsilon_self = atof(argv[++i]);
      else if (strcmp(argv[i],"obs_factor")==0 && i+1<argc)
         r->obs_factor = atof(argv[++i]);
      else if (strcmp(argv[i],"obs_factor_self")==0 && i+1<argc)
         r->obs_factor_self = atof(argv[++i]);
      else if (strcmp(argv[i],"dat_filename")==0 && i+1<argc)
         dat_filename = argv[++i];
      else if (strcmp(argv[i],"ee_force")==0 && i+1<argc)
      {
         int ee_force_argc;
         char ** ee_force_argv;
         cd_util_shparse(argv[++i], &ee_force_argc, &ee_force_argv);
         if (ee_force_argc == 1)
         {
            r->ee_force[0] = 0.0;
            r->ee_force[1] = 0.0;
            r->ee_force[2] = -atof(ee_force_argv[0]);
         }
         else if (ee_force_argc == 3)
         {
            r->ee_force[0] = atof(ee_force_argv[0]);
            r->ee_force[1] = atof(ee_force_argv[1]);
            r->ee_force[2] = atof(ee_force_argv[2]);
         }
         else
            { exc = "ee_force must be length 1 or 3!"; goto error; }
      }
      else if (strcmp(argv[i],"ee_force_at")==0 && i+1<argc)
      {
         int ee_force_at_argc;
         char ** ee_force_at_argv;
         cd_util_shparse(argv[++i], &ee_force_at_argc, &ee_force_at_argv);
         if (ee_force_at_argc != 3)
            { exc = "ee_force_at must be length 3!"; goto error; }
         r->ee_force_at[0] = atof(ee_force_at_argv[0]);
         r->ee_force_at[1] = atof(ee_force_at_argv[1]);
         r->ee_force_at[2] = atof(ee_force_at_argv[2]);
      }
      else if (strcmp(argv[i],"ee_torque_weights")==0 && i+1<argc)
      {
         char ** my_argv;
         if (r->ee_torque_weights) { exc = "Only one ee_torque_weights can be passed!"; goto error; }
         cd_util_shparse(argv[++i], &r->ee_torque_weights_n, &my_argv);
         r->ee_torque_weights = (double *) malloc(r->ee_torque_weights_n * sizeof(double));
         for (j=0; j<r->ee_torque_weights_n; j++)
            r->ee_torque_weights[j] = atof(my_argv[j]);
         free(my_argv);
         RAVELOG_DEBUG("parsed ee_torque_weights: %s\n",
            sf_vector(r->ee_torque_weights,r->ee_torque_weights_n).c_str());
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   RAVELOG_DEBUG("Using robot %s.\n", r->robot->GetName().c_str());
   if (dat_filename) RAVELOG_DEBUG("Using dat_filename |%s|.\n", dat_filename);
   
   /* check validity of input arguments ... */
   if (!r->robot) { exc = "Did not pass a robot!"; goto error; }
   if (!adofgoal && !starttraj) { exc = "Did not pass either adofgoal or starttraj!"; goto error; }
   if (r->floating_base && !basegoal && !starttraj) { exc = "Passed floating_base with no basegoal!"; goto error; }
   if (!r->floating_base && basegoal) { exc = "Passed basegoal with no floating_base!"; goto error; }
   if (!this->n_sdfs) { exc = "No signed distance fields have yet been computed!"; goto error; }
   if (lambda < 0.01) { exc = "lambda must be >=0.01!"; goto error; }
   if (r->n_points < 3) { exc = "n_points must be >=3!"; goto error; }
   /* i need to go through the two constraint functions above and
    * rework the trajectory and jacobian stuff */
   if (r->floating_base && r->start_tsr) { exc = "floating_base and start_tsr together is not yet implemented!"; goto error; }
   /*if (r->floating_base && r->everyn_tsr) { exc = "floating_base and everyn_tsr together is not yet implemented!"; goto error; }*/
   
   /* get optimizer degrees of freedom */
   r->n_adof = r->robot->GetActiveDOF();
   n = (r->floating_base ? 7 : 0) + r->n_adof; /* floating base pose */
   
   /* check that n_adofgoal matches active dof */
   if (adofgoal && (n_adofgoal != r->n_adof))
   {
      RAVELOG_DEBUG("n_adof: %d; n_adofgoal: %d\n", r->n_adof, n_adofgoal);
      exc = "size of adofgoal does not match active dofs!";
      goto error;
   }
   
   /* check that ee_torque_weights_n matches */
   if (r->ee_torque_weights && (r->ee_torque_weights_n != r->n_adof))
   {
      RAVELOG_DEBUG("n_adof: %d; ee_torque_weights_n: %d\n", r->n_adof, r->ee_torque_weights_n);
      exc = "size of ee_torque_weights does not match active dofs!";
      goto error;
   }
   
   /* allocate adofindices */
   r->adofindices = (int *) malloc(r->n_adof * sizeof(int));
   {
      std::vector<int> vec = r->robot->GetActiveDOFIndices();
      std::string buf;
      for (j=0; j<r->n_adof; j++)
      {
         r->adofindices[j] = vec[j];
         buf += sf(" %d", r->adofindices[j]);
      }
      RAVELOG_DEBUG("adofindices:%s\n", buf.c_str());
   }
   
   /* ensure that if we're doing the ee_force math, the active dofs are
    * only revolute dofs (since we do our own jacobian math) */
   if (r->ee_torque_weights)
   {
      RAVELOG_DEBUG("ee_torque_weights check for revolute only ...\n");
      for (j=0; j<r->n_adof; j++)
      {
         RAVELOG_DEBUG("joint type: %d\n", r->robot->GetJointFromDOFIndex(r->adofindices[j])->GetType());
      }
   }
   
   /* allocate sphere stuff */
   {
      struct run_sphere * s_new;
      struct run_sphere * s_active_tail;
      struct sphereelem * sel;
      OpenRAVE::KinBody::Link * link;
      int linkindex;
      std::vector<OpenRAVE::KinBodyPtr> vgrabbed;
      OpenRAVE::KinBodyPtr k;
      boost::shared_ptr<orcdchomp::kdata> d;
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0, 9, 0)
      std::vector<OpenRAVE::KinBody::LinkPtr> klinks;
      int klinkindex;
      std::vector<OpenRAVE::KinBody::GeometryInfoPtr> vgeometries;
      int geomindex;
      OpenRAVE::KinBody::GeometryInfoPtr g;
      OpenRAVE::KinBody::Link * klink;
#endif

      s_active_tail = 0; /* keep track of first active sphere inserted (will be tail) */

      /* consider the robot kinbody, as well as all grabbed bodies */
      r->robot->GetGrabbed(vgrabbed);
      vgrabbed.insert(vgrabbed.begin(), this->e->GetRobot(r->robot->GetName()));

      for (i=0; i<(int)(vgrabbed.size()); i++)
      {
         std::vector<struct run_sphere *> k_spheres;
         
         k = vgrabbed[i];
         
         /* get kinbody spheres */
         d = boost::dynamic_pointer_cast<orcdchomp::kdata>(k->GetReadableInterface("orcdchomp"));
         if (d) for (sel=d->sphereelems; sel; sel=sel->next)
         {
            /* what robot link is this sphere attached to? */
            if (k.get() == r->robot)
               link = r->robot->GetLink(sel->s->linkname).get();
            else
               link = r->robot->IsGrabbing(k).get();
            if(!link){ throw OPENRAVE_EXCEPTION_FORMAT("link %s in <orcdchomp> does not exist.", sel->s->linkname, OpenRAVE::ORE_Failed); }

            linkindex = link->GetIndex();

            /* make a new sphere */
            s_new = (struct run_sphere *) malloc(sizeof(struct run_sphere));
            s_new->radius = sel->s->radius;
            s_new->robot_link = link;
            s_new->robot_linkindex = linkindex;
            if (k.get()==r->robot)
            {
               cd_mat_memcpy(s_new->pos_wrt_link, sel->s->pos, 3, 1);
            }
            else
            {
               OpenRAVE::Transform T_w_klink = k->GetLink(sel->s->linkname)->GetTransform();
               OpenRAVE::Transform T_w_rlink = link->GetTransform();
               OpenRAVE::Vector v = T_w_rlink.inverse() * T_w_klink * OpenRAVE::Vector(sel->s->pos);
               s_new->pos_wrt_link[0] = v.x;
               s_new->pos_wrt_link[1] = v.y;
               s_new->pos_wrt_link[2] = v.z;
            }
            k_spheres.push_back(s_new);
         }
         
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0, 9, 0)
         /* load any spheres from the "spheres" geometry group */
         klinks = k->GetLinks();
         for (klinkindex=0; klinkindex<(int)(klinks.size()); klinkindex++) {
            klink = klinks[klinkindex].get();
            if (klink->GetGroupNumGeometries("spheres") == -1) {
               continue; /* there is no "spheres" group */
            }
            vgeometries = klink->GetGeometriesFromGroup("spheres");
            for (geomindex = 0; geomindex < (int)(vgeometries.size()); geomindex++) {
               g = vgeometries[geomindex];
               if (g->_type != OpenRAVE::GT_Sphere) {
                  throw OPENRAVE_EXCEPTION_FORMAT("link %s contains non-spherical geometry in the 'spheres' geometry group", klink->GetName().c_str(), OpenRAVE::ORE_Failed);
               }
               
               /* what robot link is this sphere attached to? */
               if (k.get() == r->robot)
                  link = klink;
               else
                  link = r->robot->IsGrabbing(k).get();
               if(!link){ throw OpenRAVE::openrave_exception("link does not exist!"); }

               linkindex = link->GetIndex();

               /* make a new sphere */
               s_new = (struct run_sphere *) malloc(sizeof(struct run_sphere));
               s_new->radius = g->_vGeomData[0];
               s_new->robot_link = link;
               s_new->robot_linkindex = linkindex;
               if (k.get()==r->robot)
               {
                  s_new->pos_wrt_link[0] = g->_t.trans.x;
                  s_new->pos_wrt_link[1] = g->_t.trans.y;
                  s_new->pos_wrt_link[2] = g->_t.trans.z;
               }
               else
               {
                  OpenRAVE::Transform T_w_klink = klink->GetTransform();
                  OpenRAVE::Transform T_w_rlink = link->GetTransform();
                  OpenRAVE::Vector v = T_w_rlink.inverse() * T_w_klink
                     * OpenRAVE::Vector(g->_t.trans.x, g->_t.trans.y, g->_t.trans.z);
                  s_new->pos_wrt_link[0] = v.x;
                  s_new->pos_wrt_link[1] = v.y;
                  s_new->pos_wrt_link[2] = v.z;
               }
               k_spheres.push_back(s_new);
            }
         }
#endif

         if (!k_spheres.size())
            throw OpenRAVE::openrave_exception("no spheres! kinbody does not have a <orcdchomp> tag defined?");

         /* sort spheres between active and inactive */
         for (unsigned int si=0; si<k_spheres.size(); si++)
         {
            s_new = k_spheres[si];
            /* is this link affected by the robot's active dofs? */
            for (j=0; j<n; j++)
               if (r->robot->DoesAffect(r->adofindices[j], s_new->robot_linkindex))
                  break;
            if (j<n || r->floating_base) /* if we're floating, call all spheres active (for now!) */
            {
               /* active; insert at head of r->spheres */
               s_new->next = r->spheres;
               r->spheres = s_new;
               r->n_spheres_active++;
               r->n_spheres++;
               if (!s_active_tail)
                  s_active_tail = s_new;
            }
            else
            {
               /* inactive; insert into s_inactive_head */
               s_new->next = s_inactive_head;
               s_inactive_head = s_new;
               r->n_spheres++;
            }
         }
      }
      
      RAVELOG_DEBUG("found %d active spheres, and %d total spheres.\n", r->n_spheres_active, r->n_spheres);
      
      if (!r->n_spheres_active)
         { exc = "robot active dofs must have at least one sphere!"; goto error; }
      
      /* append the inactive spheres at the end of the active list */
      s_active_tail->next = s_inactive_head;
   }
   
   /* allocate rng */
   r->rng = gsl_rng_alloc(gsl_rng_default);
   gsl_rng_set(r->rng, seed);
   
   if (dat_filename)
   {
      r->fp_dat = fopen(dat_filename, "w");
      if (!r->fp_dat) { exc = "could not open dat_filename file for writing!"; goto error; }
   }
   
   /* check that starttraj has the right ConfigurationSpecification? */
   
   /* initialize the run structure */
   m = r->n_points-2;
   if (r->start_tsr) m++;
   r->J = (double *) malloc(3*n*sizeof(double));
   r->J2 = (double *) malloc(3*n*sizeof(double));
   r->sphere_poss_all = (double *) malloc((r->n_points)*(r->n_spheres_active)*3*sizeof(double));
   if (r->start_tsr)
      r->sphere_poss = r->sphere_poss_all;
   else
      r->sphere_poss = r->sphere_poss_all + (r->n_spheres_active)*3;
   r->sphere_vels = (double *) malloc(m*(r->n_spheres_active)*3*sizeof(double));
   r->sphere_accs = (double *) malloc(m*(r->n_spheres_active)*3*sizeof(double));
   r->sphere_jacs = (double *) malloc(m*(r->n_spheres_active)*(3*n)*sizeof(double));
   
   /* btw, if we're floating, we have no inactive spheres;
    * all spheres are moving, so they're all deemed active;
    * therefore space has to be allocated for all of their positions
    * across all trajectory points */
   r->sphere_poss_inactive = (double *) malloc((r->n_spheres - r->n_spheres_active)*3*sizeof(double));
   {
      int si;
      struct run_sphere * s;
      for (si=0,s=s_inactive_head; si<(r->n_spheres - r->n_spheres_active); si++,s=s->next)
      {
         OpenRAVE::Transform t = s->robot_link->GetTransform();
         OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos_wrt_link); /* copies 3 values */
         r->sphere_poss_inactive[si*3 + 0] = v.x;
         r->sphere_poss_inactive[si*3 + 1] = v.y;
         r->sphere_poss_inactive[si*3 + 2] = v.z;
         RAVELOG_DEBUG("world pos of si=%d is xyz = %f %f %f\n", si, v.x, v.y, v.z);
      }
   }
   
   /* compute the rooted sdfs ... */
   r->n_rsdfs = this->n_sdfs;
   r->rsdfs = (struct run_rsdf *) malloc(this->n_sdfs * sizeof(struct run_rsdf));
   for (i=0; i<this->n_sdfs; i++)
   {
      OpenRAVE::KinBodyPtr sdf_kb = this->e->GetKinBody(this->sdfs[i].kinbody_name);
      if (!sdf_kb)
      {
         throw OPENRAVE_EXCEPTION_FORMAT("KinBody %s referenced by active signed distance field does not exist!\n",
            this->sdfs[i].kinbody_name, OpenRAVE::ORE_Failed);
      }
      r->rsdfs[i].grid = this->sdfs[i].grid;
      OpenRAVE::Transform t = sdf_kb->GetTransform();
      r->rsdfs[i].pose_world_gsdf[0] = t.trans.x;
      r->rsdfs[i].pose_world_gsdf[1] = t.trans.y;
      r->rsdfs[i].pose_world_gsdf[2] = t.trans.z;
      r->rsdfs[i].pose_world_gsdf[3] = t.rot.y;
      r->rsdfs[i].pose_world_gsdf[4] = t.rot.z;
      r->rsdfs[i].pose_world_gsdf[5] = t.rot.w;
      r->rsdfs[i].pose_world_gsdf[6] = t.rot.x;
      cd_kin_pose_compose(r->rsdfs[i].pose_world_gsdf, this->sdfs[i].pose, r->rsdfs[i].pose_world_gsdf);
      cd_kin_pose_invert(r->rsdfs[i].pose_world_gsdf, r->rsdfs[i].pose_gsdf_world);
   }
   
   /* create the trajectory */
   r->traj = (double *) malloc((r->n_points)*n*sizeof(double));
   
   /* initialize trajectory */
   if (starttraj.get())
   {
      RAVELOG_INFO("Initializing from a passed trajectory ...\n");
      if (r->floating_base)
      {
        /* get base transform config spec */
        OpenRAVE::ConfigurationSpecification basetx_spec;
        basetx_spec.AddGroup(boost::str(boost::format("affine_transform %s %d") % r->robot->GetName() % OpenRAVE::DOF_Transform), 7, "linear");
        
        /* get base tx and active dofs */
        for (i=0; i<r->n_points; i++)
        {
           std::vector<OpenRAVE::dReal> vec;
           
           vec.clear();
           starttraj->Sample(vec, i*starttraj->GetDuration()/((r->n_points)-1), basetx_spec);
           r->traj[i*n+0] = vec[0];
           r->traj[i*n+1] = vec[1];
           r->traj[i*n+2] = vec[2];
           r->traj[i*n+3] = vec[4];
           r->traj[i*n+4] = vec[5];
           r->traj[i*n+5] = vec[6];
           r->traj[i*n+6] = vec[3];
           cd_kin_pose_normalize(&r->traj[i*n]);
           
           vec.clear();
           starttraj->Sample(vec, i*starttraj->GetDuration()/((r->n_points)-1), r->robot->GetActiveConfigurationSpecification());
           for (j=0; j<r->n_adof; j++)
              r->traj[i*n+7+j] = vec[j];
        }
      }
      else
      {
        for (i=0; i<r->n_points; i++)
        {
           std::vector<OpenRAVE::dReal> vec;
           starttraj->Sample(vec, i*starttraj->GetDuration()/((r->n_points)-1), r->robot->GetActiveConfigurationSpecification());
           for (j=0; j<r->n_adof; j++)
              r->traj[i*n+j] = vec[j];
        }
      }
   }
   else
   {
      std::vector<OpenRAVE::dReal> start;
      RAVELOG_INFO("Initializing from a straight-line trajectory ...\n");
      cd_mat_set_zero(r->traj, r->n_points, n);
      
      /* fill the starting and ending points */
      if (r->floating_base)
      {
        /* starting point (i=0) */
        OpenRAVE::Transform t = r->robot->GetTransform();
        r->traj[0] = t.trans.x;
        r->traj[1] = t.trans.y;
        r->traj[2] = t.trans.z;
        r->traj[3] = t.rot.y;
        r->traj[4] = t.rot.z;
        r->traj[5] = t.rot.w;
        r->traj[6] = t.rot.x;
        r->robot->GetActiveDOFValues(start);
        for (j=0; j<r->n_adof; j++)
           r->traj[7+j] = start[j];
        /* ending point (i=r->n_points-1) */
        for (j=0; j<7; j++)
           r->traj[(r->n_points-1)*n+j] = basegoal[j];
        for (j=0; j<r->n_adof; j++)
           r->traj[(r->n_points-1)*n+7+j] = adofgoal[j];
      }
      else
      {
        /* starting point (i=0) */
        r->robot->GetActiveDOFValues(start);
        for (j=0; j<r->n_adof; j++)
           r->traj[j] = start[j];
        /* ending point (i=r->n_points-1) */
        for (j=0; j<r->n_adof; j++)
           r->traj[(r->n_points-1)*n+j] = adofgoal[j];
      }
      
      /* interpolate between */
      for (i=0; i<r->n_points; i++)
         for (j=0; j<n; j++)
            r->traj[i*n+j] = r->traj[j] + (r->traj[(r->n_points-1)*n+j]-r->traj[j]) * i/(r->n_points-1);
      
      /* normalize quaternions */
      if (r->floating_base)
        for (i=0; i<r->n_points; i++)
          cd_kin_pose_normalize(&r->traj[i*n]);
   }
   
   /* calculate the dimensionality of each tsr constraint, and tsr_enabled mask */
   for (j=0; j<r->n_contsrs; j++)
   {
      r->contsrs[j]->k = 0;
      for (i=0; i<6; i++)
      {
         if (r->contsrs[j]->tsr->Bw[i][0] == 0.0 && r->contsrs[j]->tsr->Bw[i][1] == 0.0)
         {
            r->contsrs[j]->tsr_enabled[i] = 1;
            r->contsrs[j]->k++;
         }
         else
            r->contsrs[j]->tsr_enabled[i] = 0;
      }
   }
      
   /* calculate the dimensionality of the start_tsr constraint */
   if (r->start_tsr)
   {
      std::string buf;
      r->start_tsr_k = 0;
      for (i=0; i<6; i++)
      {
         if (r->start_tsr->Bw[i][0] == 0.0 && r->start_tsr->Bw[i][1] == 0.0)
         {
            r->start_tsr_enabled[i] = 1;
            r->start_tsr_k++;
         }
         else
            r->start_tsr_enabled[i] = 0;
         buf += sf(" %d", r->start_tsr_enabled[i]);
      }
      RAVELOG_DEBUG("start_tsr%s\n");
   }
   
   /* calculate the dimensionality of the everyn_tsr constraint */
   if (r->everyn_tsr)
   {
      std::string buf;
      r->everyn_tsr_k = 0;
      for (i=0; i<6; i++)
      {
         if (r->everyn_tsr->Bw[i][0] == 0.0 && r->everyn_tsr->Bw[i][1] == 0.0)
         {
            r->everyn_tsr_enabled[i] = 1;
            r->everyn_tsr_k++;
         }
         else
            r->everyn_tsr_enabled[i] = 0;
         buf += sf(" %d", r->everyn_tsr_enabled[i]);
      }
      RAVELOG_DEBUG("everyn_tsr\n");
   }
   
   /* ok, ready to go! create a chomp solver */
   err = cd_chomp_create(&c, m, n, D, &r->traj[(r->start_tsr?0:1)*n], n);
   if (err) { exc = "error creating chomp instance!"; goto error; }
   
   /* evaluate the constraint on the start and end points */
   for (j=0; j<r->n_contsrs; j++)
   {
      double con_val[6];
      /* check constraint at start */
      if (r->contsrs[j]->type == RUN_CON_START || r->contsrs[j]->type == RUN_CON_ALL)
      {
        RAVELOG_DEBUG("evaluating con_tsr[%d] constraint on start point ...\n", j);
        con_tsr(r->contsrs[j], c, 0, r->traj, con_val, 0);
        RAVELOG_DEBUG("con_val: %s\n", sf_vector(con_val,r->contsrs[j]->k).c_str());
      }
      /* check constraint at end */
      if (r->contsrs[j]->type == RUN_CON_END || r->contsrs[j]->type == RUN_CON_ALL)
      {
        RAVELOG_DEBUG("evaluating con_tsr[%d] constraint on end point ...\n", j);
        con_tsr(r->contsrs[j], c, m-1, &r->traj[(r->n_points-1)*n], con_val, 0);
        RAVELOG_DEBUG("con_val: %s\n", sf_vector(con_val,r->contsrs[j]->k).c_str());
      }
   }
   
   /* evaluate the constraint on the start and end points */
   if (r->start_tsr)
   {
      double * con_val;
      con_val = (double *) malloc(r->start_tsr_k * sizeof(double));
      RAVELOG_DEBUG("evaluating start_tsr constraint on first point ...\n");
      con_start_tsr(r, c, 0, r->traj, con_val, 0);
      RAVELOG_DEBUG("con_val: %s\n", sf_vector(con_val,r->start_tsr_k).c_str());
      free(con_val);
   }
   
   /* evaluate the everyn constraint on the start and end points */
   if (r->everyn_tsr)
   {
      double * con_val;
      con_val = (double *) malloc(r->everyn_tsr_k * sizeof(double));
      RAVELOG_DEBUG("evaluating everyn_tsr constraint on middle point m=%d ...\n", m/2);
      con_everyn_tsr(r, c, m/2, &r->traj[(m/2)*n], con_val, 0);
      RAVELOG_DEBUG("con_val: %s\n", sf_vector(con_val,r->everyn_tsr_k).c_str());
      free(con_val);
   }
   
   /* set up trajectory */
   c->dt = 1.0/((r->n_points)-1);
   
   /* set up the starts and ends stuff;
    * we need to rework all of this to support the general con_tsr */
   if (r->start_tsr)
   {
      c->inits[0] = 0;
      cd_chomp_add_constraint(c, r->start_tsr_k, 0, r,
         (int (*)(void * cptr, struct cd_chomp *, int i, double * point, double * con_val, double * con_jacobian))con_start_tsr);
   }
   else
      c->inits[0] = &r->traj[0*n];
   
   c->finals[0] = &r->traj[((r->n_points)-1)*n];
   
   if (r->everyn_tsr)
   {
      int con_i;
      for (con_i=0; con_i<m; con_i+=1)
      {
         RAVELOG_INFO("Adding everyn_tsr constraint to traj point i=%d\n", con_i);
         cd_chomp_add_constraint(c, r->everyn_tsr_k, con_i, r,
            (int (*)(void * cptr, struct cd_chomp *, int i, double * point, double * con_val, double * con_jacobian))con_everyn_tsr);
         /*if (con_i == 51) break;*/
      }
   }
   
   for (j=0; j<r->n_contsrs; j++)
   {
      switch (r->contsrs[j]->type)
      {
      case RUN_CON_START:
         cd_chomp_add_constraint(c, r->contsrs[j]->k, 0, r->contsrs[j],
            (int (*)(void *, struct cd_chomp *, int, double *, double *, double *))con_tsr);
         break;
      case RUN_CON_END:
         cd_chomp_add_constraint(c, r->contsrs[j]->k, m-1, r->contsrs[j],
            (int (*)(void *, struct cd_chomp *, int, double *, double *, double *))con_tsr);
         break;
      case RUN_CON_ALL:
         for (i=0; i<m; i++)
         {
            cd_chomp_add_constraint(c, r->contsrs[j]->k, i, r->contsrs[j],
               (int (*)(void *, struct cd_chomp *, int, double *, double *, double *))con_tsr);
         }
         break;
      }
   }
   
   /* set up obstacle cost */
   c->cptr = r;
   c->cost_pre = (int (*)(void *, struct cd_chomp *, int, double **))sphere_cost_pre;
   c->cost = (int (*)(void *, struct cd_chomp *, int, double *, double *, double *, double *))sphere_cost;
   
   /* set up extra cost */
   if (r->start_cost)
      c->cost_extra = (int (*)(void *, struct cd_chomp *, double *, double *, double *))cost_extra_start;
   
   /*c->lambda = 1000000.0;*/
   c->lambda = lambda;
   /* this parameter affects how fast things settle;
    * 1.0e1 ~ 90% smooth in ~10 iterations
    * bigger, means much slower convergence */
   
   if (use_momentum)
      c->use_momentum = 1;
   
   if (r->use_hmc)
      r->hmc_resample_iter = 0;
      /*hmc_iters_until_resample = log(cd_util_rand_double(0.0, 1.0)) / hmc_resample_lambda;*/
   
   /* set up joint limits (allocated for all moving points) */
   r->robot->GetDOFLimits(vec_jlimit_lower, vec_jlimit_upper);
   if (r->floating_base)
   {
     for (j=0; j<7; j++)
     {
        c->jlimit_lower[j] = -HUGE_VAL;
        c->jlimit_upper[j] =  HUGE_VAL;
     }
     for (j=0; j<r->n_adof; j++)
     {
        c->jlimit_lower[7+j] = vec_jlimit_lower[r->adofindices[j]];
        c->jlimit_upper[7+j] = vec_jlimit_upper[r->adofindices[j]];
     }
   }
   else
   {
     for (j=0; j<r->n_adof; j++)
     {
        c->jlimit_lower[j] = vec_jlimit_lower[r->adofindices[j]];
        c->jlimit_upper[j] = vec_jlimit_upper[r->adofindices[j]];
     }
   }
   
   /* Initialize CHOMP */
   err = cd_chomp_init(c);
   if (err) { exc = "Error initializing chomp instance."; goto error; }
   
   /* save the chomp */
   r->c = c;
   
   /* return pointer to run struct as string */
   {
      char buf[128];
      sprintf(buf, "%p", r);
      sout.write(buf, strlen(buf));
   }
   
error:
   free(adofgoal);
   free(basegoal);

   if (exc)
   {
      run_destroy(r);
      throw OpenRAVE::openrave_exception(exc);
   }

   RAVELOG_DEBUG("create done! returning ...\n");
   return 0;
}

int mod::iterate(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int j;
   int nscan;
   /* input args */
   struct run * r = 0;
   int n_iter = 1;
   double max_time = HUGE_VAL;
   char * trajs_fileformstr = 0;
   /* other */
   struct cd_chomp * c;
   double hmc_alpha;
   double cost_total;
   double cost_obs;
   double cost_smooth;
   char trajs_filename[1024];
   struct timespec ticks;
   struct timespec ticks_tic;
   struct timespec ticks_toc;
   OpenRAVE::TrajectoryBasePtr t;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;

   /* parse arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"run")==0 && i+1<argc)
      {
         if (r) throw OpenRAVE::openrave_exception("Only one r can be passed!");
         nscan = sscanf(argv[++i], "%p", &r);
         if (nscan != 1) throw OpenRAVE::openrave_exception("Could not parse r!");
      }
      else if (strcmp(argv[i],"n_iter")==0 && i+1<argc)
         n_iter = atoi(argv[++i]);
      else if (strcmp(argv[i],"max_time")==0 && i+1<argc)
         max_time = atof(argv[++i]);
      else if (strcmp(argv[i],"trajs_fileformstr")==0 && i+1<argc)
         trajs_fileformstr = argv[++i];
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   /* check argument values */
   if (!r) throw OpenRAVE::openrave_exception("you must pass a created run!");
   if (n_iter < 0) throw OpenRAVE::openrave_exception("n_iter must be >=0!");

   if (trajs_fileformstr) RAVELOG_DEBUG("Using trajs_fileformstr |%s|.\n", trajs_fileformstr);
   
   /* convenience stuff */
   c = r->c;
   
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /* start timing! */
   CD_OS_TIMESPEC_SET_ZERO(&ticks);
   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

   RAVELOG_DEBUG("iterating CHOMP ...\n");
   for (r->iter=0; r->iter<n_iter; r->iter++)
   {
      /* resample momentum if using hmc */
      if (r->use_hmc && r->iter == r->hmc_resample_iter)
      {
         hmc_alpha = 100.0 * exp(0.02 * r->iter);
         RAVELOG_DEBUG("resampling momentum with alpha = %f ...\n", hmc_alpha);
         
         /* the momentum term is now AG */
         for (i=0; i<c->m; i++)
            for (j=0; j<c->n; j++)
               c->AG[i*c->n+j] = gsl_ran_gaussian(r->rng, 1.0/sqrt(hmc_alpha));
         c->leapfrog_first = 1;
         
         /* set new resampling iter */
         r->hmc_resample_iter += 1 + (int) (- log(gsl_rng_uniform(r->rng)) / r->hmc_resample_lambda);
      }
      
      /* dump the intermediate trajectory before each iteration */
      if (trajs_fileformstr)
      {
         if (r->floating_base)
         {
           RAVELOG_ERROR("Error: trajs_fileformstr and floating_base combined is not yet implemented!\n");
           throw OpenRAVE::openrave_exception("Error: trajs_fileformstr and floating_base combined is not yet implemented!");
         }
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
         CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
         CD_OS_TIMESPEC_ADD(&ticks, &ticks_toc);
         sprintf(trajs_filename, trajs_fileformstr, r->iter);
         t = OpenRAVE::RaveCreateTrajectory(this->e);
         t->Init(r->robot->GetActiveConfigurationSpecification());
         for (i=0; i<r->n_points; i++)
         {
            std::vector<OpenRAVE::dReal> vec(&r->traj[i*c->n], &r->traj[i*c->n + c->n]);
            t->Insert(i, vec);
         }
         std::ofstream f(trajs_filename);
         /* have to do this or otherwise precision gets lost */
         f << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
         t->serialize(f);
         f.close();
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);
      }

      int ret = cd_chomp_iterate(c, 1, &cost_total, &cost_obs, &cost_smooth);
      RAVELOG_INFO("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f\n", r->iter, cost_total, cost_obs, cost_smooth);
      if (ret==-1) 
      {
         RAVELOG_ERROR("stuck outside of joint limits\n");
         throw OpenRAVE::openrave_exception("Resulting trajectory is outside of joint limits!");
      }
      
      /* normalize quaternions */
      if (r->floating_base)
        for (i=0; i<r->n_points; i++)
          cd_kin_pose_normalize(&r->traj[i*c->n]);
      
      /* dump stats to data file (note these stats are for trajectory before this iteration) */
      if (r->fp_dat)
      {
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
         CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
         CD_OS_TIMESPEC_ADD(&ticks_toc, &ticks);
         fprintf(r->fp_dat, "%d %f %f %f %f\n",
            r->iter, CD_OS_TIMESPEC_DOUBLE(&ticks_toc), cost_total, cost_obs, cost_smooth);
      }
      
      /* quit if we're over time! */
      {
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
         CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
         CD_OS_TIMESPEC_ADD(&ticks_toc, &ticks);
         if (CD_OS_TIMESPEC_DOUBLE(&ticks_toc) > max_time)
            break;
      }
   }
   
   cd_chomp_iterate(c, 0, &cost_total, &cost_obs, &cost_smooth);
   RAVELOG_INFO("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f [FINAL]\n", r->iter, cost_total, cost_obs, cost_smooth);
   
   RAVELOG_DEBUG("done!\n");

#ifdef DEBUG_TIMING
   /*printf("Clock time for %d iterations: %.8f\n", iter, cd_os_timespec_double(&ticks_iterations));*/
   RAVELOG_INFO("Time breakdown:\n");
   RAVELOG_INFO("  ticks_vels         %.8f\n", cd_os_timespec_double(&c->ticks_vels));
   RAVELOG_INFO("  ticks_callback_pre %.8f\n", cd_os_timespec_double(&c->ticks_callback_pre));
   RAVELOG_INFO("    ticks_fk           %.8f\n", cd_os_timespec_double(&r->ticks_fk));
   RAVELOG_INFO("    ticks_jacobians    %.8f\n", cd_os_timespec_double(&r->ticks_jacobians));
   RAVELOG_INFO("    ticks_pre_velsaccs %.8f\n", cd_os_timespec_double(&r->ticks_pre_velsaccs));
   RAVELOG_INFO("  ticks_callbacks    %.8f\n", cd_os_timespec_double(&c->ticks_callbacks));
   RAVELOG_INFO("    ticks_selfcol      %.8f\n", cd_os_timespec_double(&r->ticks_selfcol));
   RAVELOG_INFO("  ticks_smoothgrad   %.8f\n", cd_os_timespec_double(&c->ticks_smoothgrad));
   RAVELOG_INFO("  ticks_smoothcost   %.8f\n", cd_os_timespec_double(&c->ticks_smoothcost));
#endif
   
   sout << cost_total;

   return 0;
}

int mod::gettraj(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int nscan;
   /* args */
   struct run * r = 0;
   int no_collision_check = 0;
   int no_collision_exception = 0;
   int no_collision_details = 0;
   /* other */
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   OpenRAVE::TrajectoryBasePtr t;
   OpenRAVE::RobotBasePtr boostrobot;
   
   /* parse arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"run")==0 && i+1<argc)
      {
         if (r) throw OpenRAVE::openrave_exception("Only one r can be passed!");
         nscan = sscanf(argv[++i], "%p", &r);
         if (nscan != 1) throw OpenRAVE::openrave_exception("Could not parse r!");
      }
      else if (strcmp(argv[i],"no_collision_check")==0)
         no_collision_check = 1;
      else if (strcmp(argv[i],"no_collision_exception")==0)
         no_collision_exception = 1;
      else if (strcmp(argv[i],"no_collision_details")==0)
         no_collision_details = 1;
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   if (!r) throw OpenRAVE::openrave_exception("you must pass a created run!");
   boostrobot = this->e->GetRobot(r->robot->GetName());
   
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /* create an openrave trajectory from the result, and send to sout */
   t = OpenRAVE::RaveCreateTrajectory(this->e);
   t->Init(r->robot->GetActiveConfigurationSpecification());
   for (i=0; i<r->n_points; i++)
   {
      std::vector<OpenRAVE::dReal> vec(&r->traj[i*r->c->n+(r->floating_base?7:0)], &r->traj[(i+1)*r->c->n]);
      t->Insert(i, vec);
   }
   
   RAVELOG_DEBUG("timing trajectory ...\n");
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
   /* new openrave added a fmaxaccelmult parameter (number 5) */
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,boostrobot,false,1.0,1.0,"LinearTrajectoryRetimer","");
#else
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,boostrobot,false,1.0,"LinearTrajectoryRetimer");
#endif
   if (r->floating_base)
   {
      /* step two: create the affine_transform trajectory,
       * using the timing from the active dof trajectory above */
      OpenRAVE::TrajectoryBasePtr t_pose;
      t_pose = OpenRAVE::RaveCreateTrajectory(this->e);
      OpenRAVE::ConfigurationSpecification spec;
      spec.AddGroup("deltatime", 1, "");
      spec.AddGroup(boost::str(boost::format("affine_transform %s %d") % r->robot->GetName() % OpenRAVE::DOF_Transform), 7, "linear");
      spec.AddGroup(boost::str(boost::format("affine_velocities %s %d") % r->robot->GetName() % OpenRAVE::DOF_Transform), 7, "next");
      t_pose->Init(spec);
      int dofdeltatime_offset = t->GetConfigurationSpecification().GetGroupFromName("deltatime").offset;
      for (i=0; i<r->n_points; i++)
      {
         std::vector<OpenRAVE::dReal> vec(15, 0.0);
         vec[1+0] = r->traj[i*r->c->n+0];
         vec[1+1] = r->traj[i*r->c->n+1];
         vec[1+2] = r->traj[i*r->c->n+2];
         vec[1+4] = r->traj[i*r->c->n+3];
         vec[1+5] = r->traj[i*r->c->n+4];
         vec[1+6] = r->traj[i*r->c->n+5];
         vec[1+3] = r->traj[i*r->c->n+6];
         if (i > 0)
         {
            std::vector<OpenRAVE::dReal> dofvec;
            t->GetWaypoint(i, dofvec);
            double deltatime = dofvec[dofdeltatime_offset];
            vec[0] = deltatime;
            vec[1+7+0] = (r->traj[i*r->c->n+0] - r->traj[(i-1)*r->c->n+0]) / deltatime;
            vec[1+7+1] = (r->traj[i*r->c->n+1] - r->traj[(i-1)*r->c->n+1]) / deltatime;
            vec[1+7+2] = (r->traj[i*r->c->n+2] - r->traj[(i-1)*r->c->n+2]) / deltatime;
            vec[1+7+4] = (r->traj[i*r->c->n+3] - r->traj[(i-1)*r->c->n+3]) / deltatime;
            vec[1+7+5] = (r->traj[i*r->c->n+4] - r->traj[(i-1)*r->c->n+4]) / deltatime;
            vec[1+7+6] = (r->traj[i*r->c->n+5] - r->traj[(i-1)*r->c->n+5]) / deltatime;
            vec[1+7+3] = (r->traj[i*r->c->n+6] - r->traj[(i-1)*r->c->n+6]) / deltatime;
         }
         t_pose->Insert(i, vec);
      }
      /* step three: merge the trajectories
       * (should have identical timings for identical waypoints) */
      std::list<OpenRAVE::TrajectoryBaseConstPtr> trajs;
      trajs.push_back(t);
      trajs.push_back(t_pose);
      t = OpenRAVE::planningutils::MergeTrajectories(trajs);
   }

   if (!no_collision_check)
   {
      RAVELOG_DEBUG("checking trajectory for collision ...\n");
      int collides = 0;
      double time;
      OpenRAVE::CollisionReportPtr report(new OpenRAVE::CollisionReport());
      
      /* get trajectory length */
      int NN = t->GetNumWaypoints();
      int ii = 0;
      double total_dist = 0.0;
      for (ii=0; ii<NN-1; ii++) 
      {
         std::vector< OpenRAVE::dReal > point1;
         t->GetWaypoint(ii, point1);
         std::vector< OpenRAVE::dReal > point2;
         t->GetWaypoint(ii+1, point2);
         double dist = 0.0;
         int total_dof = boostrobot->GetActiveDOF();
         for (int jj=0; jj<total_dof; jj++)
         {
            dist += pow(point1[jj]-point2[jj],2);
         }
         total_dist += sqrt(dist);
      }
      
      double step_dist = 0.04;
      double step_time = t->GetDuration()*step_dist/total_dist;
      
      for (time=0.0; time<t->GetDuration(); time+=step_time)
      {
         std::vector< OpenRAVE::dReal > point;
         t->Sample(point, time);
         r->robot->SetActiveDOFValues(point);
         /* EnvironmentBase::CheckCollision should also check robot's grabbed bodies against the environment;
          * RobotBase::CheckSelfCollision should check robot's grabbed bodies as well
          * (as opposed to EnvironmentBase::CheckSelfCollision, see
          * http://openrave-users-list.185357.n3.nabble.com/Self-Collision-functions-td4026234.html) */
         if (this->e->CheckCollision(boostrobot,report)
            || boostrobot->CheckSelfCollision(report))
         {
            collides = 1;
            if (!no_collision_details) RAVELOG_ERROR("Collision at t=%f: %s\n", time, report->__str__().c_str());
            if (!no_collision_exception) throw OpenRAVE::openrave_exception("Resulting trajectory is in collision!");
         }
      }
      if (collides)
         RAVELOG_ERROR("   trajectory collides!\n");
   }
   
   t->serialize(sout);
   
   return 0;
}

int mod::destroy(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int nscan;
   struct run * r = 0;
   /* parse arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"run")==0 && i+1<argc)
      {
         if (r) throw OpenRAVE::openrave_exception("Only one run can be passed!");
         nscan = sscanf(argv[++i], "%p", &r);
         if (nscan != 1) throw OpenRAVE::openrave_exception("Could not parse r!");
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   if (!r) throw OpenRAVE::openrave_exception("you must pass a created run!");
   run_destroy(r);
   return 0;
}

void run_destroy(struct run * r)
{
   int i;
   free(r->traj);
   free(r->rsdfs);
   free(r->J);
   free(r->J2);
   free(r->sphere_poss_inactive);
   free(r->sphere_poss_all);
   free(r->sphere_vels);
   free(r->sphere_accs);
   free(r->sphere_jacs);
   free(r->adofindices);
   free(r->spheres);
   free(r->ee_torque_weights);
   if (r->fp_dat) fclose(r->fp_dat);
   if (r->rng) gsl_rng_free(r->rng);
   tsr_destroy(r->start_tsr);
   tsr_destroy(r->everyn_tsr);
   for (i=0; i<r->n_contsrs; i++)
   {
     tsr_destroy(r->contsrs[i]->tsr);
     free(r->contsrs[i]);
   }
   free(r->contsrs);
   cd_chomp_free(r->c);
   free(r);
}

int tsr_create_parse(struct tsr ** tp, char * str)
{
   int ret;
   struct tsr * t;
   double AR[3][3];
   double Ad[3];
   double BR[3][3];
   double Bd[3];
   
   t = (struct tsr *) malloc(sizeof(struct tsr));
   if (!t) return -1;
   
   ret = sscanf(str,
      "%d %31s"
      " %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf"
      " %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf"
      " %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
      &t->manipindex,
      t->bodyandlink,
      &AR[0][0], &AR[1][0], &AR[2][0],
      &AR[0][1], &AR[1][1], &AR[2][1],
      &AR[0][2], &AR[1][2], &AR[2][2],
      &Ad[0], &Ad[1], &Ad[2],
      &BR[0][0], &BR[1][0], &BR[2][0],
      &BR[0][1], &BR[1][1], &BR[2][1],
      &BR[0][2], &BR[1][2], &BR[2][2],
      &Bd[0], &Bd[1], &Bd[2],
      &t->Bw[0][0], &t->Bw[0][1],
      &t->Bw[1][0], &t->Bw[1][1],
      &t->Bw[2][0], &t->Bw[2][1],
      &t->Bw[3][0], &t->Bw[3][1],
      &t->Bw[4][0], &t->Bw[4][1],
      &t->Bw[5][0], &t->Bw[5][1]);
   if (ret != 38) { free(t); return -2; }
   
   /* Convert from dr to pose */
   cd_kin_pose_from_dR(t->T0w, Ad, AR);
   
   cd_kin_pose_from_dR(t->Twe, Bd, BR);
   
   *tp = t;
   return 0;
}

void tsr_destroy(struct tsr * t)
{
   free(t);
}

} /* namespace orcdchomp */
