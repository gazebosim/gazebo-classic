Gazebo - A dynamic multi-robot simulator
----------------------------------------

This is the Gazebo simulator.  Gazebo simulates multiple robots in a
3D environment, with extensive dynamic interaction between objects.

  http://gazebosim.org

Installation
------------
Instructions are located at

  http://gazebosim.org/install

Gazebo cmake parameters available at configuring time:

 - BUILD_TESTING (bool) [default False]
   Include the test suite compilation in the default make call (make all).
 - ENABLE_DIAGNOSTICS
   If this is defined, it will enable diagnostic timers using the macros
   from Diagnostics.hh (see also the standalone diagnostics example):
   DIAG_TIMER_START("name")
   DIAG_TIMER_LAP("name")
   DIAG_TIMER_STOP("name")
 - USE_HOST_CFLAGS (bool) [default True]
   Check the building machine for supported compiler optimizations and use
   them to build the software.
 - USE_UPSTREAM_CFLAGS (bool) [default True]
   Use the recommended gazebo developers compiler optimizations flags.
 - USE_EXTERNAL_TINYXML (bool) [default True]
   Use external copy of tinyxml during the build.
 - USE_EXTERNAL_TINYXML2 (bool) [default True]
   Use external copy of tinyxml2 during the build.
 - USE_LOW_MEMORY_TEST (bool) [default False]
   Use reduced version of tests which need less quantity of RAM memory
   available.
 - FORCE_GRAPHIC_TESTS_COMPILATION (bool) [default False]
   Ignore system checks to look for graphic and acceleration support and
   compile all the test suites.
 - ENABLE_SCREEN_TESTS (bool) [default True]
   Enable or disable tests that need screen rendering to run properly.
   Headless machines or machines with the screen turned off should set this to
   False.
 - USE_PCH (bool) [default False]
   Use GNU Precompiled Headers. Only works with the gnu compiler.

 - ENABLE_TESTS_COMPILATION (DEPRECATED)
   The new behaviour is to call 'make tests' explicitly to compile the test
   suite. Calling 'make' or 'make all' won't compile the tests.


Uninstallation
--------------
Read the uninstallation instructions (http://gazebosim.org/uninstall) in the
online manual for generic instructions.  For most people, the following
sequence will suffice (might need sudo if it installed in /usr):

  $ make uninstall (inside the gazebo/build directory)
