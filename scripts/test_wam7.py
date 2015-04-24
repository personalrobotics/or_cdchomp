#!/usr/bin/env python
import atexit
import math
import numpy
import openravepy
import orcdchomp.orcdchomp

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
e = openravepy.Environment()
atexit.register(e.Destroy)

e.SetViewer('qtcoin')

# load the orcdchomp module
m_chomp = openravepy.RaveCreateModule(e, 'orcdchomp')
if not m_chomp:
   raise RuntimeError('no chomp module found!')
orcdchomp.orcdchomp.bind(m_chomp)

# table
table = e.ReadKinBodyXMLFile('models/furniture/rolly-table.iv')
e.Add(table)
table.SetTransform([0.70711,0.70711,0,0,0,0,0])

# bottle (and its grasp)
mug = e.ReadKinBodyXMLFile('models/objects/mug3.iv')
e.Add(mug)
mug.SetTransform([1,0,0,0,0,0,0.7])
T_mug_palm = numpy.array(
   [[ 0, -1,  0, 0.000 ],
    [ 0,  0, -1, 0.075 ],
    [ 1,  0,  0, 0.100 ],
    [ 0,  0,  0, 1     ]])

# robot
r = e.ReadRobotXMLFile('barrettwam_withspheres.robot.xml')
e.Add(r)
r.SetTransform([0.70711,0,0.70711,0,-1.0,0,1.0])

# set up active manip, active dofs
r.SetActiveManipulator('arm')
m = r.GetActiveManipulator()
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(r,
   iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
   ikmodel.autogenerate()
r.SetActiveDOFs(m.GetArmIndices())
T_palm_ee = numpy.array(
   [[ 1., 0., 0., 0. ],
    [ 0., 1., 0., 0.  ],
    [ 0., 0., 1., 0.125 ],
    [ 0., 0., 0., 1. ]])

# get IK solution for bottle
T_ee = reduce(numpy.dot, [
   mug.GetTransform(),
   numpy.linalg.inv(T_mug_palm),
   T_palm_ee])

q_goal = m.FindIKSolution(T_ee, 0)
print('q_goal:', q_goal)

# set starting arm configuration
r.SetActiveDOFValues([2.5,-1.8,0.0,2.0,0.0,0.2,0.0])

raw_input('Press [Enter] to view spheres ...')
m_chomp.viewspheres(robot=r)
raw_input('Press [Enter] to clear spheres ...')
for b in e.GetBodies():
   if b.GetName().startswith('orcdchomp_sphere_'):
      e.Remove(b)

# disable the robot to compute the distance field for all other objects ...
raw_input('Press [Enter] compute robot distance field ...')
r.Enable(False)
m_chomp.computedistancefield(kinbody=r,cache_filename='sdf_tablemug.dat')
r.Enable(True)

raw_input('Press [Enter] run chomp ...')
try:
   t = m_chomp.runchomp(robot=r, n_iter=100, lambda_=100.0, obs_factor=500.0,
      adofgoal=q_goal, no_collision_exception=True)
except RuntimeError as ex:
   print ex
   t = None

try:
   while t is not None:
      raw_input('Press [Enter] to run the trajectory, [Ctrl]+[C] to quit ...')
      with e:
         r.GetController().SetPath(t)
except KeyboardInterrupt:
   print
