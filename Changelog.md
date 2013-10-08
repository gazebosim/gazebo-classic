## Gazebo 2.0

### Gazebo 2.0.0 (2013-10-08)
1. Change default build type to RelWithDebInfo.
    [Pull Request #617](https://bitbucket.org/osrf/gazebo/pull-request/617)
1. Use `TEST_TYPE` in test names.
    * [Pull request #624](https://bitbucket.org/osrf/gazebo/pull-request/624)
1. Use external SDF library.
  * [Pull request #627](https://bitbucket.org/osrf/gazebo/pull-request/627)

## Gazebo 1.9

### Gazebo 1.9.1 (2013-08-20)
* Deprecate header files that require case-sensitive filesystem (e.g. Common.hh, Physics.hh) [https://bitbucket.org/osrf/gazebo/pull-request/638/fix-for-775-deprecate-headers-that-require]
* Initial support for building on Mac OS X [https://bitbucket.org/osrf/gazebo/pull-request/660/osx-support-for-gazebo-19] [https://bitbucket.org/osrf/gazebo/pull-request/657/cmake-fixes-for-osx]
* Fixes for various issues [https://bitbucket.org/osrf/gazebo/pull-request/635/fix-for-issue-792/diff] [https://bitbucket.org/osrf/gazebo/pull-request/628/allow-scoped-and-non-scoped-joint-names-to/diff] [https://bitbucket.org/osrf/gazebo/pull-request/636/fix-build-dependency-in-message-generation/diff] [https://bitbucket.org/osrf/gazebo/pull-request/639/make-the-unversioned-setupsh-a-copy-of-the/diff] [https://bitbucket.org/osrf/gazebo/pull-request/650/added-missing-lib-to-player-client-library/diff] [https://bitbucket.org/osrf/gazebo/pull-request/656/install-gzmode_create-without-sh-suffix/diff]

### Gazebo 1.9.0 (2013-07-23)
* Use external package [sdformat](https://bitbucket.org/osrf/sdformat) for sdf parsing, refactor the `Element::GetValue*` function calls, and deprecate Gazebo's internal sdf parser [https://bitbucket.org/osrf/gazebo/pull-request/627]
* Improved ROS support ([[Tutorials#ROS_Integration |documentation here]]) [https://bitbucket.org/osrf/gazebo/pull-request/559]
* Added Sonar, Force-Torque, and Tactile Pressure sensors [https://bitbucket.org/osrf/gazebo/pull-request/557], [https://bitbucket.org/osrf/gazebo/pull-request/567]
* Add compile-time defaults for environment variables so that sourcing setup.sh is unnecessary in most cases [https://bitbucket.org/osrf/gazebo/pull-request/620]
* Enable user camera to follow objects in client window [https://bitbucket.org/osrf/gazebo/pull-request/603]
* Install protobuf message files for use in custom messages [https://bitbucket.org/osrf/gazebo/pull-request/614]
* Change default compilation flags to improve debugging [https://bitbucket.org/osrf/gazebo/pull-request/617]
* Change to supported relative include paths [https://bitbucket.org/osrf/gazebo/pull-request/594]
* Fix display of laser scans when sensor is rotated [https://bitbucket.org/osrf/gazebo/pull-request/599]

## Gazebo 1.8

### Gazebo 1.8.7 (2013-07-16)
* Fix bug in URDF parsing of Vector3 elements [https://bitbucket.org/osrf/gazebo/pull-request/613]
* Fix compilation errors with newest libraries [https://bitbucket.org/osrf/gazebo/pull-request/615]

### Gazebo 1.8.6 (2013-06-07)
* Fix inertia lumping in the URDF parser[https://bitbucket.org/osrf/gazebo/pull-request/554]
* Fix for ODEJoint CFM damping sign error [https://bitbucket.org/osrf/gazebo/pull-request/586]
* Fix transport memory growth[https://bitbucket.org/osrf/gazebo/pull-request/584]
* Reduce log file data in order to reduce buffer growth that results in out of memory kernel errors[https://bitbucket.org/osrf/gazebo/pull-request/587]

### Gazebo 1.8.5 (2013-06-04)
* Fix Gazebo build for machines without a valid display.[https://bitbucket.org/osrf/gazebo/commits/37f00422eea03365b839a632c1850431ee6a1d67]

### Gazebo 1.8.4 (2013-06-03)
* Fix UDRF to SDF converter so that URDF gazebo extensions are applied to all collsions in a link.[https://bitbucket.org/osrf/gazebo/pull-request/579]
* Prevent transport layer from locking when a gzclient connects to a gzserver over a connection with high latency.[https://bitbucket.org/osrf/gazebo/pull-request/572]
* Improve performance and fix uninitialized conditional jumps.[https://bitbucket.org/osrf/gazebo/pull-request/571]

### Gazebo 1.8.3 (2013-06-03)
* Fix for gzlog hanging when gzserver is not present or not responsive[https://bitbucket.org/osrf/gazebo/pull-request/577]
* Fix occasional segfault when generating log files[https://bitbucket.org/osrf/gazebo/pull-request/575]
* Performance improvement to ODE[https://bitbucket.org/osrf/gazebo/pull-request/556]
* Fix node initialization[https://bitbucket.org/osrf/gazebo/pull-request/570]
* Fix GPU laser Hz rate reduction when sensor moved away from world origin[https://bitbucket.org/osrf/gazebo/pull-request/566]
* Fix incorrect lighting in camera sensors when GPU laser is subscribe to[https://bitbucket.org/osrf/gazebo/pull-request/563]

### Gazebo 1.8.2 (2013-05-28)
* ODE performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/535][https://bitbucket.org/osrf/gazebo/pull-request/537]
* Fixed tests[https://bitbucket.org/osrf/gazebo/pull-request/538][https://bitbucket.org/osrf/gazebo/pull-request/541][https://bitbucket.org/osrf/gazebo/pull-request/542]
* Fixed sinking vehicle bug[https://bitbucket.org/osrf/drcsim/issue/300] in pull-request[https://bitbucket.org/osrf/gazebo/pull-request/538]
* Fix GPU sensor throttling[https://bitbucket.org/osrf/gazebo/pull-request/536]
* Reduce string comparisons for better performance[https://bitbucket.org/osrf/gazebo/pull-request/546]
* Contact manager performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/543]
* Transport performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/548]
* Reduce friction noise[https://bitbucket.org/osrf/gazebo/pull-request/545]

### Gazebo 1.8.1 (2013-05-22)
* Please note that 1.8.1 contains a bug[https://bitbucket.org/osrf/drcsim/issue/300] that causes interpenetration between objects in resting contact to grow slowly.  Please update to 1.8.2 for the patch.
* Added warm starting[https://bitbucket.org/osrf/gazebo/pull-request/529]
* Reduced console output[https://bitbucket.org/osrf/gazebo/pull-request/533]
* Improved off screen rendering performance[https://bitbucket.org/osrf/gazebo/pull-request/530]
* Performance improvements [https://bitbucket.org/osrf/gazebo/pull-request/535] [https://bitbucket.org/osrf/gazebo/pull-request/537]

### Gazebo 1.8.0 (2013-05-17)
* Fixed slider axis [https://bitbucket.org/osrf/gazebo/pull-request/527]
* Fixed heightmap shadows [https://bitbucket.org/osrf/gazebo/pull-request/525]
* Fixed model and canonical link pose [https://bitbucket.org/osrf/gazebo/pull-request/519]
* Fixed OSX message header[https://bitbucket.org/osrf/gazebo/pull-request/524]
* Added zlib compression for logging [https://bitbucket.org/osrf/gazebo/pull-request/515]
* Allow clouds to be disabled in cameras [https://bitbucket.org/osrf/gazebo/pull-request/507]
* Camera rendering performance [https://bitbucket.org/osrf/gazebo/pull-request/528]


## Gazebo 1.7

### Gazebo 1.7.3 (2013-05-08)
* Fixed log cleanup (again) [https://bitbucket.org/osrf/gazebo/pull-request/511/fix-log-cleanup-logic]

### Gazebo 1.7.2 (2013-05-07)
* Fixed log cleanup [https://bitbucket.org/osrf/gazebo/pull-request/506/fix-gzlog-stop-command-line]
* Minor documentation fix [https://bitbucket.org/osrf/gazebo/pull-request/488/minor-documentation-fix]

### Gazebo 1.7.1 (2013-04-19)
* Fixed tests
* IMU sensor receives time stamped data from links
* Fix saving image frames [https://bitbucket.org/osrf/gazebo/pull-request/466/fix-saving-frames/diff]
* Wireframe rendering in GUI [https://bitbucket.org/osrf/gazebo/pull-request/414/allow-rendering-of-models-in-wireframe]
* Improved logging performance [https://bitbucket.org/osrf/gazebo/pull-request/457/improvements-to-gzlog-filter-and-logging]
* Viscous mud model [https://bitbucket.org/osrf/gazebo/pull-request/448/mud-plugin/diff]

## Gazebo 1.6

### Gazebo 1.6.3 (2013-04-15)
* Fixed a [critical SDF bug](https://bitbucket.org/osrf/gazebo/pull-request/451)
* Fixed a [laser offset bug](https://bitbucket.org/osrf/gazebo/pull-request/449)

### Gazebo 1.6.2 (2013-04-14)
* Fix for fdir1 physics property [https://bitbucket.org/osrf/gazebo/pull-request/429/fixes-to-treat-fdir1-better-1-rotate-into/diff]
* Fix for force torque sensor [https://bitbucket.org/osrf/gazebo/pull-request/447]
* SDF documentation fix [https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match]

### Gazebo 1.6.1 (2013-04-05)
* Switch default build type to Release.

### Gazebo 1.6.0 (2013-04-05)
* Improvements to inertia in rubble pile
* Various Bullet integration advances.
* Noise models for ray, camera, and imu sensors.
* SDF 1.4, which accommodates more physics engine parameters and also some sensor noise models.
* Initial support for making movies from within Gazebo.
* Many performance improvements.
* Many bug fixes.
* Progress toward to building on OS X.

## Gazebo 1.5

### Gazebo 1.5.0 (2013-03-11)
* Partial integration of Bullet
  * Includes: cubes, spheres, cylinders, planes, meshes, revolute joints, ray sensors
* GUI Interface for log writing.
* Threaded sensors.
* Multi-camera sensor.

* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/236 Issue #236]
 * [https://bitbucket.org/osrf/gazebo/issue/507 Issue #507]
 * [https://bitbucket.org/osrf/gazebo/issue/530 Issue #530]
 * [https://bitbucket.org/osrf/gazebo/issue/279 Issue #279]
 * [https://bitbucket.org/osrf/gazebo/issue/529 Issue #529]
 * [https://bitbucket.org/osrf/gazebo/issue/239 Issue #239]
 * [https://bitbucket.org/osrf/gazebo/issue/5 Issue #5]

## Gazebo 1.4

### Gazebo 1.4.0 (2013-02-01)
* New Features:
 * GUI elements to display messages from the server.
 * Multi-floor building editor and creator.
 * Improved sensor visualizations.
 * Improved mouse interactions

* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/16 Issue #16]
 * [https://bitbucket.org/osrf/gazebo/issue/142 Issue #142]
 * [https://bitbucket.org/osrf/gazebo/issue/229 Issue #229]
 * [https://bitbucket.org/osrf/gazebo/issue/277 Issue #277]
 * [https://bitbucket.org/osrf/gazebo/issue/291 Issue #291]
 * [https://bitbucket.org/osrf/gazebo/issue/310 Issue #310]
 * [https://bitbucket.org/osrf/gazebo/issue/320 Issue #320]
 * [https://bitbucket.org/osrf/gazebo/issue/329 Issue #329]
 * [https://bitbucket.org/osrf/gazebo/issue/333 Issue #333]
 * [https://bitbucket.org/osrf/gazebo/issue/334 Issue #334]
 * [https://bitbucket.org/osrf/gazebo/issue/335 Issue #335]
 * [https://bitbucket.org/osrf/gazebo/issue/341 Issue #341]
 * [https://bitbucket.org/osrf/gazebo/issue/350 Issue #350]
 * [https://bitbucket.org/osrf/gazebo/issue/384 Issue #384]
 * [https://bitbucket.org/osrf/gazebo/issue/431 Issue #431]
 * [https://bitbucket.org/osrf/gazebo/issue/433 Issue #433]
 * [https://bitbucket.org/osrf/gazebo/issue/453 Issue #453]
 * [https://bitbucket.org/osrf/gazebo/issue/456 Issue #456]
 * [https://bitbucket.org/osrf/gazebo/issue/457 Issue #457]
 * [https://bitbucket.org/osrf/gazebo/issue/459 Issue #459]

## Gazebo 1.3

### Gazebo 1.3.1 (2012-12-14)
* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/297 Issue #297]
* Other bugs fixed:
 * [https://bitbucket.org/osrf/gazebo/pull-request/164/ Fix light bounding box to disable properly when deselected]
 * [https://bitbucket.org/osrf/gazebo/pull-request/169/ Determine correct local IP address, to make remote clients work properly] 
 * Various test fixes

### Gazebo 1.3.0 (2012-12-03)
* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/233 Issue #233]
 * [https://bitbucket.org/osrf/gazebo/issue/238 Issue #238]
 * [https://bitbucket.org/osrf/gazebo/issue/2 Issue #2]
 * [https://bitbucket.org/osrf/gazebo/issue/95 Issue #95]
 * [https://bitbucket.org/osrf/gazebo/issue/97 Issue #97]
 * [https://bitbucket.org/osrf/gazebo/issue/90 Issue #90]
 * [https://bitbucket.org/osrf/gazebo/issue/253 Issue #253]
 * [https://bitbucket.org/osrf/gazebo/issue/163 Issue #163]
 * [https://bitbucket.org/osrf/gazebo/issue/91 Issue #91]
 * [https://bitbucket.org/osrf/gazebo/issue/245 Issue #245]
 * [https://bitbucket.org/osrf/gazebo/issue/242 Issue #242]
 * [https://bitbucket.org/osrf/gazebo/issue/156 Issue #156]
 * [https://bitbucket.org/osrf/gazebo/issue/78 Issue #78]
 * [https://bitbucket.org/osrf/gazebo/issue/36 Issue #36]
 * [https://bitbucket.org/osrf/gazebo/issue/104 Issue #104]
 * [https://bitbucket.org/osrf/gazebo/issue/249 Issue #249]
 * [https://bitbucket.org/osrf/gazebo/issue/244 Issue #244]
 * [https://bitbucket.org/osrf/gazebo/issue/36 Issue #36]

* New features:
 * Default camera view changed to look down at the origin from a height of 2 meters at location (5, -5, 2).
 * Record state data using the '-r' command line option, playback recorded state data using the '-p' command line option
 * Adjust placement of lights using the mouse.
 * Reduced the startup time.
 * Added visual reference for GUI mouse movements.
 * SDF version 1.3 released (changes from 1.2 listed below):
     - added `name` to `<camera name="cam_name"/>`
     - added `pose` to `<camera><pose>...</pose></camera>`
     - removed `filename` from `<mesh><filename>...</filename><mesh>`, use uri only.
     - recovered `provide_feedback` under `<joint>`, allowing calling `physics::Joint::GetForceTorque` in plugins.
     - added `imu` under `<sensor>`.

## Gazebo 1.2

### Gazebo 1.2.6 (2012-11-08)
* Fixed a transport issue with the GUI. Fixed saving the world via the GUI. Added more documentation. ([https://bitbucket.org/osrf/gazebo/pull-request/43/fixed-a-transport-issue-with-the-gui-fixed/diff pull request #43])
* Clean up mutex usage. ([https://bitbucket.org/osrf/gazebo/pull-request/54/fix-mutex-in-modellistwidget-using-boost/diff pull request #54])
* Fix OGRE path determination ([https://bitbucket.org/osrf/gazebo/pull-request/58/fix-ogre-paths-so-this-also-works-with/diff pull request #58], [https://bitbucket.org/osrf/gazebo/pull-request/68/fix-ogre-plugindir-determination/diff pull request #68])
* Fixed a couple of crashes and model selection/dragging problems ([https://bitbucket.org/osrf/gazebo/pull-request/59/fixed-a-couple-of-crashes-and-model/diff pull request #59])

### Gazebo 1.2.5 (2012-10-22)
* Step increment update while paused fixed ([https://bitbucket.org/osrf/gazebo/pull-request/45/fix-proper-world-stepinc-count-we-were/diff pull request #45])
* Actually call plugin destructors on shutdown ([https://bitbucket.org/osrf/gazebo/pull-request/51/fixed-a-bug-which-prevent-a-plugin/diff pull request #51])
* Don't crash on bad SDF input ([https://bitbucket.org/osrf/gazebo/pull-request/52/fixed-loading-of-bad-sdf-files/diff pull request #52])
* Fix cleanup of ray sensors on model deletion ([https://bitbucket.org/osrf/gazebo/pull-request/53/deleting-a-model-with-a-ray-sensor-did/diff pull request #53])
* Fix loading / deletion of improperly specified models ([https://bitbucket.org/osrf/gazebo/pull-request/56/catch-when-loading-bad-models-joint/diff pull request #56])

### Gazebo 1.2.4 (10-19-2012:08:00:52)
*  Style fixes ([https://bitbucket.org/osrf/gazebo/pull-request/30/style-fixes/diff pull request #30]).
*  Fix joint position control ([https://bitbucket.org/osrf/gazebo/pull-request/49/fixed-position-joint-control/diff pull request #49])

### Gazebo 1.2.3 (10-16-2012:18:39:54)
*  Disabled selection highlighting due to bug ([https://bitbucket.org/osrf/gazebo/pull-request/44/disabled-selection-highlighting-fixed/diff pull request #44]).
*  Fixed saving a world via the GUI.

### Gazebo 1.2.2 (10-16-2012:15:12:22)
*  Skip search for system install of libccd, use version inside gazebo ([https://bitbucket.org/osrf/gazebo/pull-request/39/skip-search-for-system-install-of-libccd/diff pull request #39]).
*  Fixed sensor initialization race condition ([https://bitbucket.org/osrf/gazebo/pull-request/42/fix-sensor-initializaiton-race-condition pull request #42]).

### Gazebo 1.2.1 (10-15-2012:21:32:55)
*  Properly removed projectors attached to deleted models ([https://bitbucket.org/osrf/gazebo/pull-request/37/remove-projectors-that-are-attached-to/diff pull request #37]).
*  Fix model plugin loading bug ([https://bitbucket.org/osrf/gazebo/pull-request/31/moving-bool-first-in-model-and-world pull request #31]).
*  Fix light insertion and visualization of models prior to insertion ([https://bitbucket.org/osrf/gazebo/pull-request/35/fixed-light-insertion-and-visualization-of/diff pull request #35]).
*  Fixed GUI manipulation of static objects ([https://bitbucket.org/osrf/gazebo/issue/63/moving-static-objects-does-not-move-the issue #63] [https://bitbucket.org/osrf/gazebo/pull-request/38/issue-63-bug-patch-moving-static-objects/diff pull request #38]).
*  Fixed GUI selection bug ([https://bitbucket.org/osrf/gazebo/pull-request/40/fixed-selection-of-multiple-objects-at/diff pull request #40])

### Gazebo 1.2.0 (10-04-2012:20:01:20)
*  Updated GUI: new style, improved mouse controls, and removal of non-functional items.
*  Model database: An online repository of models.
*  Numerous bug fixes
*  APT repository hosted at [http://osrfoundation.org OSRF]
*  Improved process control prevents zombie processes

