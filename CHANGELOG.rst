^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_cdchomp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2015-05-01)
------------------
* added some simple documentation
* Contributors: Chris Dellin

1.0.1 (2015-02-10)
------------------
* expanded error message, and using found kinbody pointer to avoid consistency problems
* merged from libcd upstream (cdellin): reworked distance field caching to just cache data (not kinbody name or sdf parameters) (commit:505)
* added check if kinbody does not exist
* Contributors: Chris Dellin, Christopher Dellin, Evan Shapiro

1.0.0 (2014-10-07)
------------------
* Added atlas dependency.
* Added libblas-dev dependency.
* Preparing for 1.0.0 release.
* updated copyright statement, added floating base python wrapper, and reworked runchomp kwargs handling
* merged from libcd upstream (cdellin): timing now at velocity/accel limits (change limits on robot to change speed) (commit:489)
* merged from libcd upstream (cdellin): added support for floating base (commits:481,491), chomp now can handle an arbitrary number of tsr constraints on both links and manipulators (commit:487)
* merged from libcd upstream (cdellin): added removefield stuff (commit:484)
* merged from libcd upstream (cdellin): cleaned up some grid code (commit:493)
* merged from libcd upstream (cdellin): added consts to cd_kin and cd_mat (commit:492)
* removed commented abort() call
* completed removal of arun joint limit change due to perported performance degradation
* Switched to openrave_plugin.
* added support for spheres via geometry groups
* Only include geometry group code in 0.9.
* Added code to search for (but not create) spheres
* Catkin-ized or_cdchomp.
* undoing arun change to joint limits, since it performs worse
* exposing derivative choice
* Added libgsl as a rosdep requirement for orcdchomp.
* Added release tag to orcdchomp.
* chomp compiles
* Added the correct joint limit fix for CHOMP
* Add exception handling if an invalid linkname is provided to orcdchomp tag
* throwing exception instead of aborting for joint limits
* added gpl license
* updated with copyright notices from libcd r490 (code isnt yet up-to-date though)
* Added the releasegil flag to the Python module.
* fixing bug with sending execute flag
* Moved orcdchomp to pr-ros-pkg.
* committing chris's changes
* added temporary isnan check on smoothness cost)
* length based collisionc check, untested
* linear timer
* orcdchomp now correctly collisions checks for self-collisions, including with grabbed bodies (mirrors orcdchomp proper r481)
* Added liblapacke as a dependency
* Added ability to get the total cost of a chomp path back out.  This change shoudl not be disruptive.  Cost is passed in as an optional parameter.  It should be a list of lenght 1. If the parameter is passed, that list willb e modified to contain the cost of the chomp path.
* Fixed type, and forgotten local header include in orcwrap
* Updating orcdchomp to match libcd r469; removed libcd as a separate package; moved needed files into orcdchomp package
* Upped libcd to fix multi-constraint matrix issue; added every_n constraint to orcdchomp; added code to time sdf computation to orcdchomp
* fixed trajs_fileformstr bug introduced when runchomp was split (r15752)
* reworked orcdchomp into separate functions to create, iterate, and gettraj a chomp run; runchomp simulated in python for backwards compatability!
* Fixed some ee_force bugs ...
* Working on ee_force addition to orcdchomp_mod
* Fixed h->sphere_poss_all bug in orcdchomp ...
* Working on adding ee_force stuff, but other things are broken all of a sudden ...
* upped to libcd r455; improved orcdchomp by adding start_tsr functionality and removing allowlimadj; added a reconfig test to orcdchomp_tests
* Adding the recent cdchomp changes to the pr tree ...
* Updating orcdchomp for cd_chomp r450 ...
* Removed incorrect argc check from argument parsing ...
* Added allowlimadj to orcdchomp; microwave uses it, and uses cbirrt allowlimadj as well!
* Fixed string reference bug (only present on older (lucid) g++ compilers ...)
* Simplifying orcdchomp some more ...
* Added copyright notices to files; added autotools build files for non-ROS users
* Switched from ros_exportenv to openrave_exports
* Fixing up orcdchomp/libcd stuff
* Added hmc stuff to orcdchomp ...
* Now using optional momentum term from cd_chomp r439 ...
* A few adjustments to the planner implementations ...
* Added max_time parameter to orcdchomp, now able to generate raw stats graphs ...
* Made wrapper package for libcd ...
* Some more improvements to orcdchomp for the paper ...
* Reworked orcdchomp, now faster, and with curvature computation.
* Fixed orcdchomp_tests cmake stuff, added timing information to orcdchomp, and cached sdfs ...
* CHOMP is now 3.5x faster!
* Oops! Forgot to check in the orcdchomp changes (to keep it from running into things!)
* Moved orcdchomp and owd_ortraj from cdellin's branch to trunk.
* Converted all orcdchomp compile-time constants to run-time parameters; now building against libcd-r434 ...
* Now correctly checking whether we're actually inside each distance field.
* Modified orcdchomp to use composite distance fields; also reworked cpp structure.
* Committing half-baked work on orcdchomp multi-distance-field revamp ...
* Now correctly raises exception on failure!
* Reverting to old libcd, original OBS_FACTOR ...
* Added bind to cbirrt_problem, libprrave send_for_traj now works with bound methods, chomp now accepts a starting trajectory, and we have more microwave chomp tests!
* Using this sweet Python technique to bind SendCommand serializers to the module object ...
* Spitting orcdchomp tests off to a separate packgae
* Adding a description, and a test script (which should really go in a new package!)
* Fixing some syntax errors
* added simply python interface to orcdchomp ...
* Removed auto-refresh from pkgbuilder
* Fixed silly world origin bug!
* Now checks for collision before returning the trajectory!
* added xmlreader to orcdchomp module, and added spheres to the herb2_padded robot model!
* Added some preliminary cost self-collision stuffs ...
* Added viewspheres command and test script from anca
* orcdchomp working pretty well, now for some more tests!
* Got orcdchomp working with the new libcd chomp!
* Working on orcdchomp ... at some point, I should sit down and better understand this cost stuff ...
* Playing around with a openrave CHOMP from the libcd implementation
* Contributors: Anca Dragan, Arunkumar Byravan, Chris Dellin, Christopher Dellin, Jennifer King, Michael Koval, Mike Vande Weghe, Prasanna Velagapudi
