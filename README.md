# or_cdchomp

`or_cdchomp` is a ROS package providing `orcdchomp`, an implementation of the
[CHOMP] trajectory optimizer for the [OpenRAVE] simulation environment.
Internally, it uses the OpenRAVE-agnostic implementation of CHOMP in [libcd]
(whose code is included in this package in the `src/libcd` directory).

Documentation for the package is provided primarily by way of the following
usage examples.  An example as a runnable script can be located at
`scripts/test_wam7.py`.

## Loading the Module

Once the package has been built, an instance of the module can be created as
follows.  As a generic module, all communication occurs over the SendCommand
interface; however, the package also includes python bindings, which we
attach to the module instance below.  This allows us to run commands as
methods of the module object (e.g. `m_chomp.viewspheres()` below).

    m_chomp = openravepy.RaveCreateModule(e, 'orcdchomp')
    import orcdchomp.orcdchomp
    orcdchomp.orcdchomp.bind(m_chomp)

## Robot spheres

The module uses a sphere model of the robot, which in general differs from
the normal geometry used for collision checking.  This sphere model can be
defined in two ways as specified below.  In either case, the spheres defined
for a robot as understood by the `orcdchomp` module can be viewed using the
following command:

    m_chomp.viewspheres(robot=or_robot_object)

This command will instantiate a new sphere kinbody in the environment for
for each robot sphere (in the location given by the robot's current
configuration).  To remove the spheres, simply remove bodies from the
environment whose names start with the string "`orcdchomp_sphere_`".

### Robot spheres in an orcdchomp XML tag

See the file `scripts/barrettwam_withspheres.robot.xml` for an example.
Here is a snippet:

    <kinbody>
      ..
      <orcdchomp>
        <spheres>
          <!-- shoulder spheres -->
          <sphere link="wam0" pos=" 0.22 0.14 0.346" radius="0.15" />
          <!-- upper arm spheres -->
          <sphere link="wam2" pos=" 0.0  0.0  0.2  " radius="0.06" />
          <sphere link="wam2" pos=" 0.0  0.0  0.3  " radius="0.06" />
      ...
      </orcdchomp>
      ...
    </kinbody>

Here, every sphere is specified relative to a robot link via a
space-separated x y z location and radius.

### Robot spheres in a special geometry group

Newer versions of OpenRAVE support geometry groups which are not used by
default by collision checkers, and CHOMP spheres can also be specified in
this way.

## Environment signed distance fields

The CHOMP algorithm uses signed distance fields to represent environment
obstacles.  The `orcdchomp` module computes and manages these fields
internally, and will use all fields which have been defined to it.

To define (and add) a new field, use the following command:

    m_chomp.computedistancefield(kinbody=r,cache_filename='blah.dat')

This method takes the following named arguments:

* `kinbody` (required): the kinbody whose base link to attach the distance field to.  If the kinbody is later moved, the field will move along with it.
* `cube_extent`: the extent of the cubes which constitute the discrete field.
* `aabb_padding`: the padding (the same in x,y,z) used around the bounding box of the kinbody used to determine the size of the field.
* `cache_filename`: the field, once calculated, will be saved to this file (binary format); it will then be read from this file subsequently.

## Running the optimizer

The module instance can work on any number of optimization runs at a
time.  To manage these runs, one can use the following methods:

    run_handle = m_chomp.create(...)
    m_chomp.iterate(run=run_handle, ...)
    m_chomp.gettraj(run=run_handle, ...)
    m_chomp.destroy(run=run_handle)

The `create()` method takes the bulk of the options, and is used to set
parameters to the optimizer (described below).  Once an optimization run has
been created, it is referenced by a string run handle, which must be passed
to the remaining methods.

The `iterate()` method performs iterations of the CHOMP algorithm.

The `gettraj()` method returns the current trajectory from the optimizer,
and also optionally performs collision checks on the trajectory.

The `destroy()` method simply cleans up internal data structures for the run.

While this API was designed so that `iterate()` and `gettraj()` can be called
multiple times, it is common that the four methods will simply be called
once in turn.  To simplify that use case, the module also provides a single
`runchomp()` method, which encapsulates handling of the run object for you,
and returns the trajectory directly.

## Parameters

The `orcdchomp` `create()` method (as well as the `runchomp()` helper)
supports a large number of parameters to control the behavior of the
optimizer.  Below is a partial list.

* `robot` (required): the robot object whose active dofs to use for planning
* `adofgoal`: the goal dof values for the robot's active dofs (the current configuration is used for the start dof values)
* `basegoal`: the goal pose of the base of the robot (for floating base optimization)
* `floating_base`: whether to do floating base optimization
* `lambda_`: the update rate of the optimizer
* `starttraj`: the starting trajectory to use (defaults to a straight line in configuration space)
* `n_points`: the number of moving points in the waypoint trajectory representation
* `con_tsr`, `con_tsrs`: constraints to be respected (experimantal)
* `start_tsr`: constraint to be respected at the start (deprecated)
* `start_cost`: additional cost function callback for start configurations
* `everyn_tsr`: a constraint to be respected at every `n` waypoints
* `use_momentum`: whether to do second-order optimization (used with HMC)
* `use_hmc`: whether to do Hamiltonian Monte-Carlo randomization
* `hmc_resample_lambda`: the HMC resampling parameter
* `seed`: the HMC random number generator seed
* `epsilon`: the obstacle padding distance
* `epsilon_self`: the self-collision padding distance
* `obs_factor`: the coefficient for environment obstacle cost
* `obs_factor_self`: the coefficient for self-collision obstacle cost
* `no_report_cost`: whether to disable reporting costs at each iteration
* `dat_filename`: a file to dump optimization data to

## Contributors

`or_cdchomp` was primarily developed by Chris Dellin (<cdellin@gmail.com>)
at the [Robotics Institute][ri] at [Carnegie Mellon University][cmu],
both as a member of the [Personal Robotics Lab][pr] with
[Sidd Srinivasa][srinivasa] and as a graduate student of
[Chris Atkeson][atkeson].  Other significant contributions were made by Jen
King, Michael Koval, and Anca Dragan.

[ri]: https://www.ri.cmu.edu/
[cmu]: http://www.cmu.edu/
[pr]: https://personalrobotics.ri.cmu.edu/
[srinivasa]: http://www.cs.cmu.edu/~siddh/
[atkeson]: http://www.cs.cmu.edu/~cga/
[CHOMP]: http://www.ri.cmu.edu/publication_view.html?pub_id=7421
[OpenRAVE]: http://openrave.org/
[libcd]: http://libcd.com/






