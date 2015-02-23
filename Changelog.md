## Gazebo 2.0

### Gazebo 2.2.0 (2014-01-10)
1. Fix compilation when using OGRE-1.9 (full support is being worked on)
    * [Issue #994](https://bitbucket.org/osrf/gazebo/issue/994)
    * [Issue #995](https://bitbucket.org/osrf/gazebo/issue/995)
    * [Issue #996](https://bitbucket.org/osrf/gazebo/issue/996)
    * [Pull request #883](https://bitbucket.org/osrf/gazebo/pull-request/883)
1. Added unit test for issue 624.
    * [Issue #624](https://bitbucket.org/osrf/gazebo/issue/624).
    * [Pull request #889](https://bitbucket.org/osrf/gazebo/pull-request/889)
1. Use 3x3 PCF shadows for smoother shadows.
    * [Pull request #887](https://bitbucket.org/osrf/gazebo/pull-request/887)
1. Update manpage copyright to 2014.
    * [Pull request #893](https://bitbucket.org/osrf/gazebo/pull-request/893)
1. Added friction integration test .
    * [Pull request #885](https://bitbucket.org/osrf/gazebo/pull-request/885)
1. Fix joint anchor when link pose is not specified.
    * [Issue #978](https://bitbucket.org/osrf/gazebo/issue/978)
    * [Pull request #862](https://bitbucket.org/osrf/gazebo/pull-request/862)
1. Added (ESC) tooltip for GUI Selection Mode icon.
    * [Issue #993](https://bitbucket.org/osrf/gazebo/issue/993)
    * [Pull request #888](https://bitbucket.org/osrf/gazebo/pull-request/888)
1. Removed old comment about resolved issue.
    * [Issue #837](https://bitbucket.org/osrf/gazebo/issue/837)
    * [Pull request #880](https://bitbucket.org/osrf/gazebo/pull-request/880)
1. Made SimbodyLink::Get* function thread-safe
    * [Issue #918](https://bitbucket.org/osrf/gazebo/issue/918)
    * [Pull request #872](https://bitbucket.org/osrf/gazebo/pull-request/872)
1. Suppressed spurious gzlog messages in ODE::Body
    * [Issue #983](https://bitbucket.org/osrf/gazebo/issue/983)
    * [Pull request #875](https://bitbucket.org/osrf/gazebo/pull-request/875)
1. Fixed Force Torque Sensor Test by properly initializing some values.
    * [Issue #982](https://bitbucket.org/osrf/gazebo/issue/982)
    * [Pull request #869](https://bitbucket.org/osrf/gazebo/pull-request/869)
1. Added breakable joint plugin to support breakable walls.
    * [Pull request #865](https://bitbucket.org/osrf/gazebo/pull-request/865)
1. Used different tuple syntax to fix compilation on OSX mavericks.
    * [Issue #947](https://bitbucket.org/osrf/gazebo/issue/947)
    * [Pull request #858](https://bitbucket.org/osrf/gazebo/pull-request/858)
1. Fixed sonar test and deprecation warning.
    * [Pull request #856](https://bitbucket.org/osrf/gazebo/pull-request/856)
1. Speed up test compilation.
    * Part of [Issue #955](https://bitbucket.org/osrf/gazebo/issue/955)
    * [Pull request #846](https://bitbucket.org/osrf/gazebo/pull-request/846)
1. Added Joint::SetEffortLimit API
    * [Issue #923](https://bitbucket.org/osrf/gazebo/issue/923)
    * [Pull request #808](https://bitbucket.org/osrf/gazebo/pull-request/808)
1. Made bullet output less verbose.
    * [Pull request #839](https://bitbucket.org/osrf/gazebo/pull-request/839)
1. Convergence acceleration and stability tweak to make atlas_v3 stable
    * [Issue #895](https://bitbucket.org/osrf/gazebo/issue/895)
    * [Pull request #772](https://bitbucket.org/osrf/gazebo/pull-request/772)
1. Added colors, textures and world files for the SPL RoboCup environment
    * [Pull request #838](https://bitbucket.org/osrf/gazebo/pull-request/838)
1. Fixed bitbucket_pullrequests tool to work with latest BitBucket API.
    * [Issue #933](https://bitbucket.org/osrf/gazebo/issue/933)
    * [Pull request #841](https://bitbucket.org/osrf/gazebo/pull-request/841)
1. Fixed cppcheck warnings.
    * [Pull request #842](https://bitbucket.org/osrf/gazebo/pull-request/842)
 
### Gazebo 2.1.0 (2013-11-08)
1. Fix mainwindow unit test
    * [Pull request #752](https://bitbucket.org/osrf/gazebo/pull-request/752)
1. Visualize moment of inertia
    * Pull request [#745](https://bitbucket.org/osrf/gazebo/pull-request/745), [#769](https://bitbucket.org/osrf/gazebo/pull-request/769), [#787](https://bitbucket.org/osrf/gazebo/pull-request/787)
    * [Issue #203](https://bitbucket.org/osrf/gazebo/issue/203)
1. Update tool to count lines of code
    * [Pull request #758](https://bitbucket.org/osrf/gazebo/pull-request/758)
1. Implement World::Clear
    * Pull request [#785](https://bitbucket.org/osrf/gazebo/pull-request/785), [#804](https://bitbucket.org/osrf/gazebo/pull-request/804)
1. Improve Bullet support
    * [Pull request #805](https://bitbucket.org/osrf/gazebo/pull-request/805)
1. Fix doxygen spacing
    * [Pull request #740](https://bitbucket.org/osrf/gazebo/pull-request/740)
1. Add tool to generate model images for thepropshop.org
    * [Pull request #734](https://bitbucket.org/osrf/gazebo/pull-request/734)
1. Added paging support for terrains
    * [Pull request #707](https://bitbucket.org/osrf/gazebo/pull-request/707)
1. Added plugin path to LID_LIBRARY_PATH in setup.sh
    * [Pull request #750](https://bitbucket.org/osrf/gazebo/pull-request/750)
1. Fix for OSX
    * [Pull request #766](https://bitbucket.org/osrf/gazebo/pull-request/766)
    * [Pull request #786](https://bitbucket.org/osrf/gazebo/pull-request/786)
    * [Issue #906](https://bitbucket.org/osrf/gazebo/issue/906)
1. Update copyright information
    * [Pull request #771](https://bitbucket.org/osrf/gazebo/pull-request/771)
1. Enable screen dependent tests
    * [Pull request #764](https://bitbucket.org/osrf/gazebo/pull-request/764)
    * [Issue #811](https://bitbucket.org/osrf/gazebo/issue/811)
1. Fix gazebo command line help message
    * [Pull request #775](https://bitbucket.org/osrf/gazebo/pull-request/775)
    * [Issue #898](https://bitbucket.org/osrf/gazebo/issue/898)
1. Fix man page test
    * [Pull request #774](https://bitbucket.org/osrf/gazebo/pull-request/774)
1. Improve load time by reducing calls to RTShader::Update
    * [Pull request #773](https://bitbucket.org/osrf/gazebo/pull-request/773)
    * [Issue #877](https://bitbucket.org/osrf/gazebo/issue/877)
1. Fix joint visualization
    * [Pull request #776](https://bitbucket.org/osrf/gazebo/pull-request/776)
    * [Pull request #802](https://bitbucket.org/osrf/gazebo/pull-request/802)
    * [Issue #464](https://bitbucket.org/osrf/gazebo/issue/464)
1. Add helpers to fix NaN
    * [Pull request #742](https://bitbucket.org/osrf/gazebo/pull-request/742)
1. Fix model resizing via the GUI
    * [Pull request #763](https://bitbucket.org/osrf/gazebo/pull-request/763)
    * [Issue #885](https://bitbucket.org/osrf/gazebo/issue/885)
1. Simplify gzlog test by using sha1
    * [Pull request #781](https://bitbucket.org/osrf/gazebo/pull-request/781)
    * [Issue #837](https://bitbucket.org/osrf/gazebo/issue/837)
1. Enable cppcheck for header files
    * [Pull request #782](https://bitbucket.org/osrf/gazebo/pull-request/782)
    * [Issue #907](https://bitbucket.org/osrf/gazebo/issue/907)
1. Fix broken regression test
    * [Pull request #784](https://bitbucket.org/osrf/gazebo/pull-request/784)
    * [Issue #884](https://bitbucket.org/osrf/gazebo/issue/884)
1. All simbody and dart to pass tests
    * [Pull request #790](https://bitbucket.org/osrf/gazebo/pull-request/790)
    * [Issue #873](https://bitbucket.org/osrf/gazebo/issue/873)
1. Fix camera rotation from SDF
    * [Pull request #789](https://bitbucket.org/osrf/gazebo/pull-request/789)
    * [Issue #920](https://bitbucket.org/osrf/gazebo/issue/920)
1. Fix bitbucket pullrequest command line tool to match new API
    * [Pull request #803](https://bitbucket.org/osrf/gazebo/pull-request/803)
1. Fix transceiver spawn errors in tests
    * [Pull request #811](https://bitbucket.org/osrf/gazebo/pull-request/811)
    * [Pull request #814](https://bitbucket.org/osrf/gazebo/pull-request/814)
 
### Gazebo 2.0.0 (2013-10-08)
1. Refactor code check tool.
    * [Pull Request #669](https://bitbucket.org/osrf/gazebo/pull-request/669)
1. Added pull request tool for Bitbucket.
    * [Pull Request #670](https://bitbucket.org/osrf/gazebo/pull-request/670)
    * [Pull Request #691](https://bitbucket.org/osrf/gazebo/pull-request/671)
1. New wireless receiver and transmitter sensor models.
    * [Pull Request #644](https://bitbucket.org/osrf/gazebo/pull-request/644)
    * [Pull Request #675](https://bitbucket.org/osrf/gazebo/pull-request/675)
    * [Pull Request #727](https://bitbucket.org/osrf/gazebo/pull-request/727)
1. Audio support using OpenAL.
    * [Pull Request #648](https://bitbucket.org/osrf/gazebo/pull-request/648)
    * [Pull Request #704](https://bitbucket.org/osrf/gazebo/pull-request/704)
1. Simplify command-line parsing of gztopic echo output.
    * [Pull Request #674](https://bitbucket.org/osrf/gazebo/pull-request/674)
    * Resolves: [Issue #795](https://bitbucket.org/osrf/gazebo/issue/795)
1. Use UNIX directories through the user of GNUInstallDirs cmake module.
    * [Pull Request #676](https://bitbucket.org/osrf/gazebo/pull-request/676)
    * [Pull Request #681](https://bitbucket.org/osrf/gazebo/pull-request/681)
1. New GUI interactions for object manipulation.
    * [Pull Request #634](https://bitbucket.org/osrf/gazebo/pull-request/634)
1. Fix for OSX menubar.
    * [Pull Request #677](https://bitbucket.org/osrf/gazebo/pull-request/677)
1. Remove internal SDF directories and dependencies.
    * [Pull Request #680](https://bitbucket.org/osrf/gazebo/pull-request/680)
1. Add minimum version for sdformat.
    * [Pull Request #682](https://bitbucket.org/osrf/gazebo/pull-request/682)
    * Resolves: [Issue #818](https://bitbucket.org/osrf/gazebo/issue/818)
1. Allow different gtest parameter types with ServerFixture
    * [Pull Request #686](https://bitbucket.org/osrf/gazebo/pull-request/686)
    * Resolves: [Issue #820](https://bitbucket.org/osrf/gazebo/issue/820)
1. GUI model scaling when using Bullet.
    * [Pull Request #683](https://bitbucket.org/osrf/gazebo/pull-request/683)
1. Fix typo in cmake config.
    * [Pull Request #694](https://bitbucket.org/osrf/gazebo/pull-request/694)
    * Resolves: [Issue #824](https://bitbucket.org/osrf/gazebo/issue/824)
1. Remove gazebo include subdir from pkgconfig and cmake config.
    * [Pull Request #691](https://bitbucket.org/osrf/gazebo/pull-request/691)
1. Torsional spring demo
    * [Pull Request #693](https://bitbucket.org/osrf/gazebo/pull-request/693)
1. Remove repeated call to SetAxis in Joint.cc
    * [Pull Request #695](https://bitbucket.org/osrf/gazebo/pull-request/695)
    * Resolves: [Issue #823](https://bitbucket.org/osrf/gazebo/issue/823)
1. Add test for rotational joints.
    * [Pull Request #697](https://bitbucket.org/osrf/gazebo/pull-request/697)
    * Resolves: [Issue #820](https://bitbucket.org/osrf/gazebo/issue/820)
1. Fix compilation of tests using Joint base class
    * [Pull Request #701](https://bitbucket.org/osrf/gazebo/pull-request/701)
1. Terrain paging implemented.
    * [Pull Request #687](https://bitbucket.org/osrf/gazebo/pull-request/687)
1. Improve timeout error reporting in ServerFixture
    * [Pull Request #705](https://bitbucket.org/osrf/gazebo/pull-request/705)
1. Fix mouse picking for cases where visuals overlap with the laser 
    * [Pull Request #709](https://bitbucket.org/osrf/gazebo/pull-request/709)
1. Fix string literals for OSX 
    * [Pull Request #712](https://bitbucket.org/osrf/gazebo/pull-request/712)
    * Resolves: [Issue #803](https://bitbucket.org/osrf/gazebo/issue/803)
1. Support for ENABLE_TESTS_COMPILATION cmake parameter 
    * [Pull Request #708](https://bitbucket.org/osrf/gazebo/pull-request/708)
1. Updated system gui plugin
    * [Pull Request #702](https://bitbucket.org/osrf/gazebo/pull-request/702)
1. Fix force torque unit test issue
    * [Pull Request #673](https://bitbucket.org/osrf/gazebo/pull-request/673)
    * Resolves: [Issue #813](https://bitbucket.org/osrf/gazebo/issue/813)
1. Use variables to control auto generation of CFlags
    * [Pull Request #699](https://bitbucket.org/osrf/gazebo/pull-request/699)
1. Remove deprecated functions.
    * [Pull Request #715](https://bitbucket.org/osrf/gazebo/pull-request/715)
1. Fix typo in `Camera.cc`
    * [Pull Request #719](https://bitbucket.org/osrf/gazebo/pull-request/719)
    * Resolves: [Issue #846](https://bitbucket.org/osrf/gazebo/issue/846)
1. Performance improvements
    * [Pull Request #561](https://bitbucket.org/osrf/gazebo/pull-request/561)
1. Fix gripper model.
    * [Pull Request #713](https://bitbucket.org/osrf/gazebo/pull-request/713)
    * Resolves: [Issue #314](https://bitbucket.org/osrf/gazebo/issue/314)
1. First part of Simbody integration
    * [Pull Request #716](https://bitbucket.org/osrf/gazebo/pull-request/716)

## Gazebo 1.9

### Gazebo 1.9.3 (2014-01-10)

1. Add thickness to plane to remove shadow flickering.
    * [Pull request #886](https://bitbucket.org/osrf/gazebo/pull-request/886)
1. Temporary GUI shadow toggle fix.
    * [Issue #925](https://bitbucket.org/osrf/gazebo/issue/925)
    * [Pull request #868](https://bitbucket.org/osrf/gazebo/pull-request/868)
1. Fix memory access bugs with libc++ on mavericks.
    * [Issue #965](https://bitbucket.org/osrf/gazebo/issue/965)
    * [Pull request #857](https://bitbucket.org/osrf/gazebo/pull-request/857)
    * [Pull request #881](https://bitbucket.org/osrf/gazebo/pull-request/881)
1. Replaced printf with cout in gztopic hz.
    * [Issue #969](https://bitbucket.org/osrf/gazebo/issue/969)
    * [Pull request #854](https://bitbucket.org/osrf/gazebo/pull-request/854)
1. Add Dark grey material and fix indentation.
    * [Pull request #851](https://bitbucket.org/osrf/gazebo/pull-request/851)
1. Fixed sonar sensor unit test.
    * [Pull request #848](https://bitbucket.org/osrf/gazebo/pull-request/848)
1. Convergence acceleration and stability tweak to make atlas_v3 stable.
    * [Pull request #845](https://bitbucket.org/osrf/gazebo/pull-request/845)
1. Update gtest to 1.7.0 to resolve problems with libc++.
    * [Issue #947](https://bitbucket.org/osrf/gazebo/issue/947)
    * [Pull request #827](https://bitbucket.org/osrf/gazebo/pull-request/827)
1. Fixed LD_LIBRARY_PATH for plugins.
    * [Issue #957](https://bitbucket.org/osrf/gazebo/issue/957)
    * [Pull request #844](https://bitbucket.org/osrf/gazebo/pull-request/844)
1. Fix transceiver sporadic errors.
    * Backport of [pull request #811](https://bitbucket.org/osrf/gazebo/pull-request/811)
    * [Pull request #836](https://bitbucket.org/osrf/gazebo/pull-request/836)
1. Modified the MsgTest to be deterministic with time checks.
    * [Pull request #843](https://bitbucket.org/osrf/gazebo/pull-request/843)
1. Fixed seg fault in LaserVisual.
    * [Issue #950](https://bitbucket.org/osrf/gazebo/issue/950)
    * [Pull request #832](https://bitbucket.org/osrf/gazebo/pull-request/832)
1. Implemented the option to disable tests that need a working screen to run properly.
    * Backport of [Pull request #764](https://bitbucket.org/osrf/gazebo/pull-request/764)
    * [Pull request #837](https://bitbucket.org/osrf/gazebo/pull-request/837)
1. Cleaned up gazebo shutdown.
    * [Pull request #829](https://bitbucket.org/osrf/gazebo/pull-request/829)
1. Fixed bug associated with loading joint child links.
    * [Issue #943](https://bitbucket.org/osrf/gazebo/issue/943)
    * [Pull request #820](https://bitbucket.org/osrf/gazebo/pull-request/820)

### Gazebo 1.9.2 (2013-11-08)
1. Fix enable/disable sky and clouds from SDF
    * [Pull request #809](https://bitbucket.org/osrf/gazebo/pull-request/809])
1. Fix occasional blank GUI screen on startup 
    * [Pull request #815](https://bitbucket.org/osrf/gazebo/pull-request/815])
1. Fix GPU laser when interacting with heightmaps
    * [Pull request #796](https://bitbucket.org/osrf/gazebo/pull-request/796])
1. Added API/ABI checker command line tool
    * [Pull request #765](https://bitbucket.org/osrf/gazebo/pull-request/765])
1. Added gtest version information 
    * [Pull request #801](https://bitbucket.org/osrf/gazebo/pull-request/801])
1. Fix GUI world saving
    * [Pull request #806](https://bitbucket.org/osrf/gazebo/pull-request/806])
1. Enable anti-aliasing for camera sensor
    * [Pull request #800](https://bitbucket.org/osrf/gazebo/pull-request/800])
1. Make sensor noise deterministic
    * [Pull request #788](https://bitbucket.org/osrf/gazebo/pull-request/788])
1. Fix build problem
    * [Issue #901](https://bitbucket.org/osrf/gazebo/issue/901)
    * [Pull request #778](https://bitbucket.org/osrf/gazebo/pull-request/778])
1. Fix a typo in Camera.cc
    * [Pull request #720](https://bitbucket.org/osrf/gazebo/pull-request/720])
    * [Issue #846](https://bitbucket.org/osrf/gazebo/issue/846)
1. Fix OSX menu bar
    * [Pull request #688](https://bitbucket.org/osrf/gazebo/pull-request/688])
1. Fix gazebo::init by calling sdf::setFindCallback() before loading the sdf in gzfactory.
    * [Pull request #678](https://bitbucket.org/osrf/gazebo/pull-request/678])
    * [Issue #817](https://bitbucket.org/osrf/gazebo/issue/817)

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
* Fix UDRF to SDF converter so that URDF gazebo extensions are applied to all collisions in a link.[https://bitbucket.org/osrf/gazebo/pull-request/579]
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

