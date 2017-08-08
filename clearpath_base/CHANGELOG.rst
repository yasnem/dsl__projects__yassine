^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.4 (2013-09-23)
------------------
* Remove install() rule workaround, add version callouts to package.xml

0.4.3 (2013-09-13)
------------------
* Block with select() in data receive loop.
* Explicitly install clearpath.horizon as well as clearpath.
* Typo fix in reverting some of the other bug-hunting experiments.
* Workaround for issue with python modules and installing messages.

0.4.2 (2013-09-11)
------------------
* Switch to an explicit GLOB for building messages, as the implicit one is broken for the build target.
* Remove build dependency on rospy.

0.4.1 (2013-09-10)
------------------
* No change, just re-releasing due to version mixup on build farm.

0.4.0 (2013-09-10)
------------------
* Removed most of the unneeded messages.
* Moved node scripts into /nodes directory.
* Moved node bases into clearpath_node library.
* Node scripts now kinematic.py -> kinematic_node, etc.
* install targets.

0.3.1 (2013-07-19)
------------------
* Added missing build_depend to clearpath_base
* Fixed broken rosdep.
* Bumped version to 0.3.0
