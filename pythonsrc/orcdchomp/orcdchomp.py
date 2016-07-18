# \file orcdchomp.py
# \brief Python interface to orcdchomp.
# \author Christopher Dellin
# \date 2012-2013

# (C) Copyright 2012-2013 Carnegie Mellon University

# This module (orcdchomp) is part of libcd.
#
# This module of libcd is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This module of libcd is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# A copy of the GNU General Public License is provided with libcd
# (license-gpl.txt) and is also available at <http://www.gnu.org/licenses/>.


import types
import openravepy

def bind(mod):
   mod.viewspheres = types.MethodType(viewspheres,mod)
   mod.computedistancefield = types.MethodType(computedistancefield,mod)
   mod.addfield_fromobsarray = types.MethodType(addfield_fromobsarray,mod)
   mod.viewfields = types.MethodType(viewfields,mod)
   mod.removefield = types.MethodType(removefield,mod)
   mod.create = types.MethodType(create,mod)
   mod.iterate = types.MethodType(iterate,mod)
   mod.gettraj = types.MethodType(gettraj,mod)
   mod.destroy = types.MethodType(destroy,mod)
   mod.runchomp = types.MethodType(runchomp,mod)

def shquot(s):
   return "'" + s.replace("'","'\\''") + "'"
   
def viewspheres(mod, robot=None, releasegil=False):
   cmd = 'viewspheres'
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % shquot(robot.GetName())
      else:
         cmd += ' robot %s' % shquot(robot)
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def computedistancefield(mod, kinbody=None, cube_extent=None, aabb_padding=None,
      cache_filename=None, require_cache=None, releasegil=False):
   cmd = 'computedistancefield'
   if kinbody is not None:
      if hasattr(kinbody,'GetName'):
         cmd += ' kinbody %s' % shquot(kinbody.GetName())
      else:
         cmd += ' kinbody %s' % shquot(kinbody)
   if cube_extent is not None:
      cmd += ' cube_extent %f' % cube_extent
   if aabb_padding is not None:
      cmd += ' aabb_padding %f' % aabb_padding
   if cache_filename is not None:
      cmd += ' cache_filename %s' % shquot(cache_filename)
   if require_cache is not None and require_cache:
      cmd += ' require_cache'
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def addfield_fromobsarray(mod, kinbody=None, obsarray=None, sizes=None, lengths=None,
      pose=None, releasegil=False):
   cmd = 'addfield_fromobsarray'
   if kinbody is not None:
      if hasattr(kinbody,'GetName'):
         cmd += ' kinbody %s' % shquot(kinbody.GetName())
      else:
         cmd += ' kinbody %s' % shquot(kinbody)
   if obsarray is not None:
      cmd += ' obsarray %s' % obsarray
   if sizes is not None:
      cmd += ' sizes %s' % shquot(' '.join([str(v) for v in sizes]))
   if lengths is not None:
      cmd += ' lengths %s' % shquot(' '.join([str(v) for v in lengths]))
   if pose is not None:
      cmd += ' pose %s' % shquot(' '.join([str(v) for v in pose]))
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def viewfields(mod, releasegil=False):
   cmd = 'viewfields'
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def removefield(mod, kinbody=None, releasegil=False):
   cmd = 'removefield'
   if kinbody is not None:
      if hasattr(kinbody,'GetName'):
         cmd += ' kinbody %s' % shquot(kinbody.GetName())
      else:
         cmd += ' kinbody %s' % shquot(kinbody)
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def create(mod, robot=None, adofgoal=None, basegoal=None, floating_base=None, lambda_=None,
   starttraj=None, n_points=None,
   con_tsr=None, con_tsrs=None, start_tsr=None, start_cost=None, everyn_tsr=None,
   use_momentum=None, use_hmc=None, hmc_resample_lambda=None, seed=None,
   epsilon=None, epsilon_self=None, obs_factor=None, obs_factor_self=None,
   no_report_cost=None, dat_filename=None, releasegil=False, derivative=None, **kwargs):
   cmd = 'create'
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % shquot(robot.GetName())
      else:
         cmd += ' robot %s' % shquot(robot)
   if adofgoal is not None:
      cmd += ' adofgoal %s' % shquot(' '.join([str(v) for v in adofgoal]))
   if basegoal is not None:
      cmd += ' basegoal %s' % shquot(' '.join([str(v) for v in basegoal]))
   if floating_base is not None and floating_base:
      cmd += ' floating_base'
   if lambda_ is not None:
      cmd += ' lambda %0.04f' % lambda_
   if starttraj is not None:
      in_traj_data = starttraj.serialize(0) # options
      cmd += ' starttraj %s' % shquot(in_traj_data)
   if n_points is not None:
      cmd += ' n_points %d' % n_points
   if con_tsr is not None:
      cmd += ' con_tsr \'%s\' \'%s\'' % (con_tsr[0], con_tsr[1].serialize())
   if con_tsrs is not None:
      for sub_con_tsr in con_tsrs:
         cmd += ' con_tsr \'%s\' \'%s\'' % (sub_con_tsr[0], sub_con_tsr[1].serialize())
   if derivative is not None:
      cmd += ' derivative %d' % derivative
   if start_tsr is not None:
      cmd += ' start_tsr \'%s\'' % start_tsr.serialize()
   if start_cost is not None:
      if isinstance(start_cost, str):
         cmd += ' start_cost \'%s\'' % start_cost
      else:
         cmd += ' start_cost \'%s %s\'' % (start_cost[0], start_cost[1])
   if everyn_tsr is not None:
      cmd += ' everyn_tsr \'%s\'' % everyn_tsr.serialize()
   if use_momentum is not None and use_momentum:
      cmd += ' use_momentum'
   if use_hmc is not None and use_hmc:
      cmd += ' use_hmc'
   if hmc_resample_lambda is not None:
      cmd += ' hmc_resample_lambda %f' % hmc_resample_lambda
   if seed is not None:
      cmd += ' seed %d' % seed
   if epsilon is not None:
      cmd += ' epsilon %f' % epsilon
   if epsilon_self is not None:
      cmd += ' epsilon_self %f' % epsilon_self
   if obs_factor is not None:
      cmd += ' obs_factor %f' % obs_factor
   if obs_factor_self is not None:
      cmd += ' obs_factor_self %f' % obs_factor_self
   if no_report_cost is not None and no_report_cost:
      cmd += ' no_report_cost'
   if dat_filename is not None:
      cmd += ' dat_filename %s' % shquot(dat_filename)
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def iterate(mod, run=None, n_iter=None, max_time=None, trajs_fileformstr=None,
      cost=None, releasegil=False):
   cmd = 'iterate'
   if run is not None:
      cmd += ' run %s' % run
   if n_iter is not None:
      cmd += ' n_iter %d' % n_iter
   if max_time is not None:
      cmd += ' max_time %f' % max_time
   if trajs_fileformstr is not None:
      cmd += ' trajs_fileformstr %s' % shquot(trajs_fileformstr)
   cost_data = mod.SendCommand(cmd, releasegil)
   if cost is not None:
      cost[0] = float(cost_data)

def gettraj(mod, run=None, no_collision_check=None, no_collision_exception=None,
      no_collision_details=None, releasegil=False):
   cmd = 'gettraj'
   if run is not None:
      cmd += ' run %s' % run
   if no_collision_check is not None and no_collision_check:
      cmd += ' no_collision_check'
   if no_collision_exception is not None and no_collision_exception:
      cmd += ' no_collision_exception'
   if no_collision_details is not None and no_collision_details:
      cmd += ' no_collision_details'
   out_traj_data = mod.SendCommand(cmd, releasegil)
   return openravepy.RaveCreateTrajectory(mod.GetEnv(),'').deserialize(out_traj_data)
   
def destroy(mod, run=None, releasegil=False):
   cmd = 'destroy'
   if run is not None:
      cmd += ' run %s' % run
   return mod.SendCommand(cmd, releasegil)

def runchomp(mod,
      # iterate args
      n_iter=None, max_time=None, trajs_fileformstr=None, cost=None,
      # gettraj args
      no_collision_check=None, no_collision_exception=None, no_collision_details=None,
      releasegil=False, **kwargs):
   # pass unknown args to create
   run = create(mod, releasegil=releasegil, **kwargs)
   iterate(mod, run=run, n_iter=n_iter, max_time=max_time, trajs_fileformstr=trajs_fileformstr, cost=cost, releasegil=releasegil)
   traj = gettraj(mod, run=run,
      no_collision_check=no_collision_check,
      no_collision_exception=no_collision_exception,
      no_collision_details=no_collision_details,
      releasegil=releasegil)
   destroy(mod, run=run, releasegil=releasegil)
   return traj
