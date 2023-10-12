## Gazebo 11

## Gazebo 11.14.0 (2023-10-06)

1. Visual::SetPose performance improvement / minor fixes
    * [Pull request #3350](https://github.com/gazebosim/gazebo/pull/3350)
   * A contribution from Janosch Machowinski

1. Apply initial sim time also after a reset.
    * [Pull request #3340](https://github.com/gazebosim/gazebo/pull/3340)
   * A contribution from Martin Pecka

1. Remove using namespace SimTK
    * [Pull request #3347](https://github.com/gazebosim/gazebo/pull/3347)
    * A contrubition from Silvio Traversaro

1. Fix build with graphviz 9
    * [Pull request #3345](https://github.com/gazebosim/gazebo/pull/3345)

1. Add support for compiling on windows x64 and x86 with vcpkg-provided dependencies
    * [Pull request #3320](https://github.com/gazebosim/gazebo/pull/3320)
    * [Pull request #3349](https://github.com/gazebosim/gazebo/pull/3349)
   * A contribution from talregev

1. gzclient: improve startup reliability
    * [Pull request #3338](https://github.com/gazebosim/gazebo/pull/3338)

1. CI: add non concurrency to all GitHub workflows
    * [Pull request #3337](https://github.com/gazebosim/gazebo/pull/3337)
   * A contribution from talregev

1. Set `HOME_ENV` according the OS
    * [Pull request #3334](https://github.com/gazebosim/gazebo/pull/3334)
   * A contribution from talregev

1. Allow usage of lambdas as transport subscription callbacks
    * [Pull request #3309](https://github.com/gazebosim/gazebo/pull/3309)
   * A contribution from Patrick Roncagliolo

1. Fix for finding new versions of protobuf
    * [Pull request #3331](https://github.com/gazebosim/gazebo/pull/3331)

## Gazebo 11.13.0 (2023-05-17)

1. Fix wide-angle lens flare occlusion lag
   * [Pull request #3325](https://github.com/gazebosim/gazebo-classic/pull/3325)

1. Add missing spaces to several rendering messages 
   * [Pull request #3322](https://github.com/gazebosim/gazebo-classic/pull/3322)

1. Fix build with ffmpeg 6.0
   * [Pull request #3318](https://github.com/gazebosim/gazebo-classic/pull/3318)

1. Allow user to name a specific light that will generate lens flare
   * [Pull request #3305](https://github.com/gazebosim/gazebo-classic/pull/3305)
   * A contribution from Terry Welsh 

1. JointController: improve thread safety
   * [Pull request #3303](https://github.com/gazebosim/gazebo-classic/pull/3303)

1. Fix pkg-config-related CMake warning
   * [Pull request #3301](https://github.com/gazebosim/gazebo-classic/pull/3301)

1. Fix template specialization in constructors to fix GCC11 build
   * [Pull request #3295](https://github.com/gazebosim/gazebo-classic/pull/3295)
   * A contribution from Ondřej Svoboda

1. Set initial sim time from the command-line
   * [Pull request #3294](https://github.com/gazebosim/gazebo-classic/pull/3294)

1. Fix typo in README
   * [Pull request #3291](https://github.com/gazebosim/gazebo-classic/pull/3291)

1. msgs.cc: add missing <array> include
   * [Pull request #3290](https://github.com/gazebosim/gazebo-classic/pull/3290)

1.  Fix crash with DEM and Camera
   * [Pull request #3279](https://github.com/gazebosim/gazebo-classic/pull/3279)

1.  Fix focal builds: use python3 with check_test_ran.py
   * [Pull request #3278](https://github.com/gazebosim/gazebo-classic/pull/3278)

1. Fix for wide angle lens flare occlusion bug
   * [Pull request #3276](https://github.com/gazebosim/gazebo-classic/pull/3276)
   * With the contribution of Terry Welsh 

1. Fix for opende heightfield and console spam
   * [Pull request #3271](https://github.com/gazebosim/gazebo-classic/pull/3271)

1. Fixes in conda-forge CI
   * [Pull request #3270](https://github.com/gazebosim/gazebo-classic/pull/3270)
   * [Pull request #3287](https://github.com/gazebosim/gazebo-classic/pull/3287)
   * [Pull request #3315](https://github.com/gazebosim/gazebo-classic/pull/3315)

1. Fix Instance() method of Singleton classes
   * [Pull request #3269](https://github.com/gazebosim/gazebo-classic/pull/3269)

1. Fix disappearing shadows when looking from certain angles
   * [Pull request #3267](https://github.com/gazebosim/gazebo-classic/pull/3267)

1. Support Lunar DEMs
   * [Pull request #3257](https://github.com/gazebosim/gazebo-classic/pull/3257)
   * [Pull request #3250](https://github.com/gazebosim/gazebo-classic/pull/3250)
   * [Pull request #3252](https://github.com/gazebosim/gazebo-classic/pull/3252)
   * [Pull request #3258](https://github.com/gazebosim/gazebo-classic/pull/3258)

1. Add support for cross-compilation in Gazebo
   * [Pull request #3190](https://github.com/gazebosim/gazebo-classic/pull/3190)  

## Gazebo 11.12.0 (2022-09-14)

1. BulletLink: add and set force and torque
    * [Pull request #3255](https://github.com/osrf/gazebo/pull/3255)

1. Camera: expose intrinsics parameters
    * [Pull request #3245](https://github.com/osrf/gazebo/pull/3245)

1. Add missing header includes
    * [Pull request #3227](https://github.com/osrf/gazebo/pull/3227)

1. Permit users to override `USE_EXTERNAL_TINYXML2` and `USE_EXTERNAL_TINYXML`
    * [Pull request #3254](https://github.com/osrf/gazebo/pull/3254)

1. CustomPSSMShadowCamera: support custom projection matrix
    * [Pull request #3249](https://github.com/osrf/gazebo/pull/3249)

1. Update Readme and add gazebo logo
    * [Pull request #3233](https://github.com/osrf/gazebo/pull/3233)
    * [Pull request #3226](https://github.com/osrf/gazebo/pull/3226)

1. Wheel plowing approximation in ODEPhysics
    * [Pull request #3164](https://github.com/osrf/gazebo/pull/3164)

1. LensFlare: parameterize number of occlusion steps
    * [Pull request #3234](https://github.com/osrf/gazebo/pull/3234)
    * [Pull request #3241](https://github.com/osrf/gazebo/pull/3241)

1. Support shininess value for each Visual in a Model
    * [Pull request #3235](https://github.com/osrf/gazebo/pull/3235)

1. Fix shininess and add tests
    * [Pull request #3231](https://github.com/osrf/gazebo/pull/3231)

## Gazebo 11.11.0 (2022-05-25)

1. Convert `OGRE_RESOURCE_PATH` with `TO_CMAKE_PATH`
    * [Pull request #3165](https://github.com/osrf/gazebo/pull/3165)
    * A contrubition from Silvio Traversaro

1. Fix compatibility with FFmpeg 5.0
    * [Pull request #3195](https://github.com/osrf/gazebo/pull/3195)
    * A contrubition from Silvio Traversaro

1. Get/Set friction coefficients of wheels
    * [Pull request #3219](https://github.com/osrf/gazebo/pull/3219)

1. Support `<anti_aliasing>` element when setting up camera
    * [Pull request #3201](https://github.com/osrf/gazebo/pull/3201)

1. Added shininess to models
    * [Pull request #3213](https://github.com/osrf/gazebo/pull/3213)
    * [Pull request #3223](https://github.com/osrf/gazebo/pull/3223)

1. Fix missing namespace for string
    * [Pull request #3211](https://github.com/osrf/gazebo/pull/3211)

1. Fix typo in TBB target check in `gazebo-config.cmake.in`
    * [Pull request #3207](https://github.com/osrf/gazebo/pull/3207)

1. Separate cache files for each heightmap LOD
    * [Pull request #3200](https://github.com/osrf/gazebo/pull/3200)

1. Parse `ode_quiet` physics parameter from SDFormat
    * [Pull request #3194](https://github.com/osrf/gazebo/pull/3194)

## Gazebo 11.10.2 (2022-03-19)

1. Support plotting for entities with / in the name
    * [Pull request #3187](https://github.com/osrf/gazebo/pull/3187)

1. Replace deprecated tbb task for tbb >= 2021
    * [Pull request #3174](https://github.com/osrf/gazebo/pull/3174)
    * A contrubition from Alex Dewar and Silvio Traversaro
    * [Pull request #3157](https://github.com/osrf/gazebo/pull/3157)

1. Add CI for compiling gazebo with conda-forge dependencies
    * [Pull request #3186](https://github.com/osrf/gazebo/pull/3186)
    * A contrubition from Silvio Traversaro

1. SearchForStuff: Do not pass /usr/lib to PATH in qwt's find_library
    * [Pull request #3178](https://github.com/osrf/gazebo/pull/3178)
    * A contrubition from Silvio Traversaro

1. Scene: support deletion of Heightmap Visuals
    * [Pull request #3171](https://github.com/osrf/gazebo/pull/3171)

1. ODEJoint: don't SetStiffnessDamping for gearbox
    * [Pull request #3169](https://github.com/osrf/gazebo/pull/3169)

## Gazebo 11.10.1 (2022-01-19)

1. Revert boost/bind.hpp changes in header files
    * [Pull request #3160](https://github.com/osrf/gazebo/pull/3160)

1. Fix duplicate vertex program name Ogre crash
    * [Pull request #3161](https://github.com/osrf/gazebo/pull/3161)

## Gazebo 11.10.0 (2022-01-12)

1. Use boost/bind/bind.hpp to fix warnings on Arch Linux
    * [Pull request #3156](https://github.com/osrf/gazebo/pull/3156)
    * Inspired by a contribution from Alex Dewar <alex.dewar@gmx.co.uk>

1. Load model plugins even after sensor timeout
    * [Pull request #3154](https://github.com/osrf/gazebo/pull/3154)

1. CMake exports: remove -std=c++11 flag
    * [Pull request #3050](https://github.com/osrf/gazebo/pull/3050)
    * A contribution from Guilhem Saurel <guilhem.saurel@laas.fr>

1. Create github action ci
    * [Pull request #3049](https://github.com/osrf/gazebo/pull/3049)
    * A contribution from Tal Regev <tal.regev@gmail.com>

1. Fix Windows build when using vcpkg
    * [Pull request #3132](https://github.com/osrf/gazebo/pull/3132)
    * A contribution from Akash Munagala <akash.munagala@gmail.com>

1. Point light shadows
    * [Pull request #3051](https://github.com/osrf/gazebo/pull/3051)

## Gazebo 11.9.1 (2021-12-02)

1. setup.sh: fix relocatability of `LD_LIBRARY_PATH`
    * [Pull request #3140](https://github.com/osrf/gazebo/pull/3140)

1. Fix setup.sh and install dirs with absolute paths
    * [Pull request #3138](https://github.com/osrf/gazebo/pull/3138)
    * A contribution from: Ben Wolsieffer <benwolsieffer@gmail.com>

1. Add check for function type to avoid undefined asin computation
    * [Pull request #3135](https://github.com/osrf/gazebo/pull/3135)
    * A contribution from: Kaden Jeppesen <kjeppesen1@gmail.com>

1. Workaround for wrong distortion material applied to some cameras (#3136)
    * [Pull request #3136](https://github.com/osrf/gazebo/pull/3136)
    * [Issue #2527](https://github.com/osrf/gazebo/issues/2527)
    * A contribution from: Terry Welsh

1. Fix Gazebo crash in building editor when adding door or window (#2276) (#3129)
    * [Pull request #3129](https://github.com/osrf/gazebo/pull/3129)
    * [Issue #2276](https://github.com/osrf/gazebo/issues/2276)
    * A contribution from: Joep Jansen <joep.w.jansen@gmail.com>

## Gazebo 11.9.0 (2021-10-28)

1. Forward port windows fixes from #2789
    * [Pull request #2985](https://github.com/osrf/gazebo/pull/2985)
    * [Pull request #2789](https://github.com/osrf/gazebo/pull/2789)
    * A contribution from: Tobias Fischer <info@tobiasfischer.info>

1. GUI option to change render rate
    * [Pull request #3127](https://github.com/osrf/gazebo/pull/3127)

1. GaussianNoiseModel: fix for unspecified biasStdDev
    * [Pull request #3083](https://github.com/osrf/gazebo/pull/3083)
    * A contribution from: Ludovic J <unifai@protonmail.com>

1. wind\_demo.world: add required parameter
    * [Pull request #3035](https://github.com/osrf/gazebo/pull/3035)

1. Fix slow loading of spawned model with plugins
    * [Pull request #3126](https://github.com/osrf/gazebo/pull/3126)

1. HeightmapLODPlugin: add server/gui params
    * [Pull request #3120](https://github.com/osrf/gazebo/pull/3120)

1. Fix gzclient starting with black screen
    * [Pull request #3121](https://github.com/osrf/gazebo/pull/3121)

1. Optionally disable "render back faces" for the shadow caster
    * [Pull request #3117](https://github.com/osrf/gazebo/pull/3117)

1. Add more profiling hooks in physics, rendering
    * [Pull request #3119](https://github.com/osrf/gazebo/pull/3119)

1. Load both .so and .dylib plugins on macOS
    * [Pull request #3069](https://github.com/osrf/gazebo/pull/3069)

1. Fix Camera Distortion rounding
    * [Pull request #3114](https://github.com/osrf/gazebo/pull/3114)
    * A contribution from: kjeppesen1 <kjeppesen1@gmail.com>

1. Always publish if flag is passed to Model::SetScale
    * [Pull request #3116](https://github.com/osrf/gazebo/pull/3116)

1. Add slip values for individual wheels
    * [Pull request #3082](https://github.com/osrf/gazebo/pull/3082)

1. Fix standalone marker example
    * [Pull request #3014](https://github.com/osrf/gazebo/pull/3014)
    * A contribution from: Dhruv Maroo <dhruvmaru007@gmail.com>

1. Faster examples build test and other CI fixes
    * [Pull request #3080](https://github.com/osrf/gazebo/pull/3080)

1. Fix pkgconfig prefix with absolute CMAKE_INSTALL_LIBDIR
    * [Pull request #3076](https://github.com/osrf/gazebo/pull/3076)
    * A contribution from: Ben Wolsieffer <benwolsieffer@gmail.com>

1. Add mutex lock in World::Step and World::Fini to fix test
    * [Pull request #3078](https://github.com/osrf/gazebo/pull/3078)

1. 👩‍🌾 Remove bitbucket-pipelines
    * [Pull request #3074](https://github.com/osrf/gazebo/pull/3074)

## Gazebo 11.8.1 (2021-08-24)

1. Fixes for shadow caster shaders
    * [Pull request #3070](https://github.com/osrf/gazebo/pull/3070)

1. Distortion::RefreshCompositor check nonzero params
    * [Pull request #3071](https://github.com/osrf/gazebo/pull/3071)

## Gazebo 11.8.0 (2021-08-17)

1. Find tbb version lower than 2021 with pkg-config
    * [Pull request #3037](https://github.com/osrf/gazebo/pull/3037)

1. Fix build with qwt 6.2
    * [Pull request #3047](https://github.com/osrf/gazebo/pull/3047)

1. Enable hardware PCF for spotlight shadows
    * [Pull request #3036](https://github.com/osrf/gazebo/pull/3036)

1. Apply Force/Torque for nested models
    * [Pull request #3039](https://github.com/osrf/gazebo/pull/3039)
    * A contribution by Cameron Miller

1. Make links within nested models modifiable from GUI Client
    * [Pull request #3031](https://github.com/osrf/gazebo/pull/3031)
    * [Pull request #3059](https://github.com/osrf/gazebo/pull/3059)
    * A contribution by Sonia Jin (Amazon)

1. Make events threadsafe
    * [Pull request #3042](https://github.com/osrf/gazebo/pull/3042)
    * A contribution by Sonia Jin (Amazon)

1. Add relocatable setup.bash script
    * [Pull request #3061](https://github.com/osrf/gazebo/pull/3061)
    * [Issue #3056](https://github.com/osrf/gazebo/issues/3056)

1. Support custom shadow caster materials
    * [Pull request #3048](https://github.com/osrf/gazebo/pull/3048)
    * [Pull request #3067](https://github.com/osrf/gazebo/pull/3067)

1. Distortion camera initialization tests and fix with background color
    * [Issue #2527](https://github.com/osrf/gazebo/issues/2527)
    * [Pull request #3044](https://github.com/osrf/gazebo/pull/3044)
    * [Pull request #3060](https://github.com/osrf/gazebo/pull/3060)

## Gazebo 11.7.0 (2021-06-xx)

1. Qualify `gazebo::util` in `using namespace` declarations.
    * [Pull request #2979](https://github.com/osrf/gazebo/pull/2979)

1. Distortion camera initialization fix
    * [Issue #2527](https://github.com/osrf/gazebo/issues/2527)
    * [Pull request #3033](https://github.com/osrf/gazebo/pull/3033)

1. Use CURL::libcurl instead of cmake variables
    * [Pull request #3030](https://github.com/osrf/gazebo/pull/3030)

1. Allow specifying lens flare and camera distortion texture format
    * [Issue #3005](https://github.com/osrf/gazebo/issues/3005)
    * [Pull request #3009](https://github.com/osrf/gazebo/pull/3009)

1. Camera distortion normalization improvement and fix folding
    * [Pull request #3009](https://github.com/osrf/gazebo/pull/3009)

1. Fix Windows compilation in Server.cc (not -> !)
    * [Pull request #3021](https://github.com/osrf/gazebo/pull/3021)

## Gazebo 11.6.0 (2021-06-09)

1. Enable output of gzerr for SDF sibling elements of any type with same name,
   following the SDF 1.7 specification.
   Environment variable GAZEBO11_BACKWARDS_COMPAT_WARNINGS_ERRORS can be set to
   use the previous behaviour and do not report these problems.
    * [Pull request #3017](https://github.com/osrf/gazebo/pull/3017)

1. Allow specifying lens flare and camera distortion texture format
    * [Issue #3005](https://github.com/osrf/gazebo/issues/3005)
    * [Pull request #3007](https://github.com/osrf/gazebo/pull/3007)

1.  Initialize the worldPoseDirty flag in Collision.cc
    * A contribution from Zachary Michaels zmichaels11@gmail.com>
    * [Pull request #2999](https://github.com/osrf/gazebo/pull/2999)

1.  Compile test plugins as MODULE instead of STATIC
    * [Pull request #2897](https://github.com/osrf/gazebo/pull/2897)

1.  Add warning if GAZEBO_RESOURCE_PATH may not be set correctly
    * [Pull request #2991](https://github.com/osrf/gazebo/pull/2991)

## Gazebo 11.5.1 (2021-05-05)

1. Avoid range-loop-construct in TopicManager
    * [Pull request #2983](https://github.com/osrf/gazebo/pull/2983)

1. Check for nullptr in TopicManager::ConnectPubToSub
    * A contribution from Emerson Knapp <emerson.b.knapp@gmail.com> (Amazon)
    * [Pull request #2978](https://github.com/osrf/gazebo/pull/2978)
    * [Issue #2875](https://github.com/osrf/gazebo/issues/2875)

## Gazebo 11.5.0 (2021-04-20)

1. Specify wide angle camera cube map texture format
    * [Pull request #2960](https://github.com/osrf/gazebo/pull/2960)
    * [Issue #2928](https://github.com/osrf/gazebo/issues/2928)

1. Protect DepthCameraPlugin globals with a mutex
    * [Pull request #2949](https://github.com/osrf/gazebo/pull/2949)

1. Avoid deadlock in ConnectionManager::Stop
    * [Pull request #2950](https://github.com/osrf/gazebo/pull/2950)

1. Optimize collision checking in ODE
    * [Pull request #2945](https://github.com/osrf/gazebo/pull/2945)

1. Fix size of spotlight visual
    * [Pull request #2947](https://github.com/osrf/gazebo/pull/2947)

1. Fix depth camera breaking shadows
    * [Pull request #2907](https://github.com/osrf/gazebo/pull/2907)

1. Fix color channel of point clouds from DepthCamera
    * [Pull request #2853](https://github.com/osrf/gazebo/pull/2853)

1. GpuRaySensor: validate scene existence
    * [Pull request #2937](https://github.com/osrf/gazebo/pull/2937)

1. LensFlare: allow inheritance
    * [Pull request #2965](https://github.com/osrf/gazebo/pull/2965)
    * [Pull request #2975](https://github.com/osrf/gazebo/pull/2975)

1. Silence message conversion warning messages
    * [Pull request #2963](https://github.com/osrf/gazebo/pull/2963)
    * [Pull request #2973](https://github.com/osrf/gazebo/pull/2973)


## Gazebo 11.4.0 (2021-04-01)

1. Restore HeightmapShape::SetHeight implementation (#2955)
    * [Pull request #2955](https://github.com/osrf/gazebo/pull/2955)
    * [BitBucket pull request 3210](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3210)

1. Lens flare cleanup and colorization
    * [Pull request #2927](https://github.com/osrf/gazebo/pull/2927)
    * [Pull request #2954](https://github.com/osrf/gazebo/pull/2954)

1. Fix build on Linux with conda-forge dependencies
    * [Pull request #2944](https://github.com/osrf/gazebo/pull/2944)

1. Fix build issues with OpenAL
    * [Pull request #2941](https://github.com/osrf/gazebo/pull/2941)
    * [Pull request #2943](https://github.com/osrf/gazebo/pull/2943)

1. Do not overwrite default linker flags
    * [Pull request #2922](https://github.com/osrf/gazebo/pull/2922)

1. Fix support for camera with single channel floating point image format
    * [Pull request #2918](https://github.com/osrf/gazebo/pull/2918)

1. Fix performance metrics deadlock with multiple types of sensors
    * [Pull request #2917](https://github.com/osrf/gazebo/pull/2917)
    * [Issue #2902](https://github.com/osrf/gazebo/issues/2902)

1. Generate spot light shadow maps
    * [Pull request #2914](https://github.com/osrf/gazebo/pull/2914)

1. Fix `find_package(gazebo)` on Windows
    * [Pull request #2906](https://github.com/osrf/gazebo/pull/2906)
    * [Issue #2905](https://github.com/osrf/gazebo/issues/2905)

1. Fix namespace usage in console macros
    * [Pull request #2892](https://github.com/osrf/gazebo/pull/2892)
    * [Issue #2896](https://github.com/osrf/gazebo/issues/2896)

1. Ensure relocatable package config files
    * [Pull request #2879](https://github.com/osrf/gazebo/pull/2879)
    * [Issue #2755](https://github.com/osrf/gazebo/issues/2755)
    * [Issue #2782](https://github.com/osrf/gazebo/issues/2782)

1. DARTHeightmapShape: correctly load asymmetric terrains
    * [Pull request #2818](https://github.com/osrf/gazebo/pull/2818)

1. Set friction params in DARTCollision for dart 6.10
    * [Pull request #2781](https://github.com/osrf/gazebo/pull/2781)

## Gazebo 11.3.0 (2020-11-26)

1. Added profiler to gazebo::rendering and gzclient
    * [Pull request #2837](https://github.com/osrf/gazebo/pull/2837)

1. SimpleTrackedVehiclePlugin: fix for boost 1.74
    * [Pull request #2862](https://github.com/osrf/gazebo/pull/2862)

1. Add support to compile the gazebo executable on Windows
    * [Pull request #2864](https://github.com/osrf/gazebo/pull/2864)

1. Support platforms in which qwt headers are not installed in a qwt directory
    * [Issue #2886](https://github.com/osrf/gazebo/issues/2886)
    * [Pull request #2887](https://github.com/osrf/gazebo/pull/2887)

1. Warn instead of fail for non-Earth Dem's on 20.04
    * [Pull request #2882](https://github.com/osrf/gazebo/pull/2882)

1. SearchForStuff: On Apple platforms do not search for uuid library
    * [Pull request #2878](https://github.com/osrf/gazebo/pull/2878)

1. Fix usage of relative paths with environment variables
    * [Pull request #2890](https://github.com/osrf/gazebo/pull/2890)

1. Support resource files with spaces
    * [Pull request #2877](https://github.com/osrf/gazebo/pull/2877)

1. Update TinyOBJLoader to v2.0.0rc8
    * [Pull request #2885](https://github.com/osrf/gazebo/pull/2885)

## Gazebo 11.2.0 (2020-09-30)

1. Fix assumptions that CMAKE\_INSTALL\_\*DIR paths are relative
    * [Pull request #2778](https://github.com/osrf/gazebo/pull/2778)

1. Accept relative paths in SDF files
    * [Pull request #2765](https://github.com/osrf/gazebo/pull/2765)
    * [Pull request #2839](https://github.com/osrf/gazebo/pull/2839)

1. Add support for frame semantics with nested models in SDFormat 1.7
    * [Pull request #2824](https://github.com/osrf/gazebo/pull/2824)

1. Fix Actor collision if loop / auto_start false
    * [Pull request #2773](https://github.com/osrf/gazebo/pull/2773)

1. Allow gazebo to download models from Fuel in the sdf files, and worlds from command line
    * [Pull request #2822](https://github.com/osrf/gazebo/pull/2822)

1. Publish performance metrics
    * [Pull request #2819](https://github.com/osrf/gazebo/pull/2819)

1. Find OGRE correctly in a system with pkg-config but without OGRE .pc files
    * [Pull request #2719](https://github.com/osrf/gazebo/pull/2719)

## Gazebo 11.1.0 (2020-08-12)

1. Synchronize time stepping of physics and sensors with `--lockstep`
    * [Issue #2736](https://github.com/osrf/gazebo/issues/2736)
    * [Pull request #2791](https://github.com/osrf/gazebo/pull/2791)
    * [Pull request #2799](https://github.com/osrf/gazebo/pull/2799)
    * [Pull request #2802](https://github.com/osrf/gazebo/pull/2802)
    * [Pull request #2761](https://github.com/osrf/gazebo/pull/2761)
    * [Pull request #2746](https://github.com/osrf/gazebo/pull/2746)

1. Enable DART support in gazebo11 .deb packages
    * [Issue #2752](https://github.com/osrf/gazebo/issues/2752)

1. LensFlare: initialize OGRE compositors during plugin initialization
    * [Pull request #2764](https://github.com/osrf/gazebo/pull/2764)

1. Add gazebo common profiler
    * [Pull request #2776](https://github.com/osrf/gazebo/pull/2776)

1. Fix pkg-config boost entries for Ubuntu Focal
    * [Pull request #2797](https://github.com/osrf/gazebo/pull/2797)

1. Fix corruption when a URDF file is included from a SDFormat 1.6 model
    * [Pull request 2734](https://github.com/osrf/gazebo/pull/2734)

1. Preserve `GAZEBO_MASTER_URI` if set before setup.sh
    * [GitHub pull request 2737](https://github.com/osrf/gazebo/pull/2737)

1. Fix sensor update rate throttling when new sensors are spawned
    * [Pull request #2784](https://github.com/osrf/gazebo/pull/2784)

1. Fix crash when collision size is zero
    * [Pull request #2768](https://github.com/osrf/gazebo/pull/2768)

1. LensFlare: initialize OGRE compositors during plugin initialization
    * [Pull request #2764](https://github.com/osrf/gazebo/pull/2764)

1. Fix pkg-config boost entries for Ubuntu Focal
    * [Pull request #2751](https://github.com/osrf/gazebo/pull/2751)

1. Fixes for ARM: FindSSE, TrackedVehiclePlugin and PluginInterfaceTest
    * [Pull request #2754](https://github.com/osrf/gazebo/pull/2754)
    * [Pull request #2748](https://github.com/osrf/gazebo/pull/2748)

1. Fix multiple reflectance maps and improve performance
    * [Pull request #2742](https://github.com/osrf/gazebo/pull/2742)

1. Fixed fails for OSX: Added using namespace boost::placeholders
    * [Pull request #2809](https://github.com/osrf/gazebo/pull/2809)

1. Add SetHeight method to HeightmapShape class
    * [BitBucket pull request 3210](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3210)

1. Fix problem with automoc in CMake 3.17
    * [BitBucket pull request 3201](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3201/)

1. Fix bounding box calculation for visuals that have links with pose offset
    * [BitBucket pull request 3196](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3196)

1. Added reflectance to depth camera sensor
    * [BitBucket pull request 3194](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3194/)

1. Added normals to depth camera sensor
    * [BitBucket pull request 3193](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3193/)

1. Prevent crash when subscribing to depth camera image topic
    * [BitBucket pull request 3197](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3197)

## Gazebo 11.0.0 (2020-01-30)

1. Update to ignition citadel dependencies, c++17, cmake 3.10.
    * [BitBucket pull request 3139](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3139)
    * [BitBucket pull request 3160](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3160)
    * [BitBucket pull request 3161](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3161)

1. Support SDFormat 1.7 frame semantics with libsdformat9.
    * [BitBucket pull request 3133](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3133)
    * [BitBucket pull request 3189](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3189)

1. Add helper function `PhysicsEngine::any_cast` to handle std::any.
    * [BitBucket pull request 3147](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3147)

1. Create Scene::UpdatePoses API to allow physics to directly update scene poses
   for server-side rendering, though it is not yet enabled.
    * [BitBucket pull request 3180](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3180)

1. Add virtual functions to Sensor and other ABI changes needed to synchronize
   physics and rendering sensors.
    * [BitBucket pull request 3184](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3184)

1. Modify build system to install dll in `<prefix>/bin` by default on Windows.
    * [BitBucket pull request 3144](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3144)

1. MeshManager: add .stlb file extension support.
    * [BitBucket pull request 3124](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3124)

1. SVG parsing: add lineto commands v,V,h,H.
    * [BitBucket pull request 3110](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3110)

1. IOManager: PIMPL-ize class and use `atomic_int` for reference count.
    * [BitBucket pull request 3167](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3167)

1. MeshManager: PIMPL-ize class.
    * [BitBucket pull request 3171](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3171)

1. Add ABI-breaking change from pr 2768 `protected ODEJoint::angleOffset[]`
    * [BitBucket pull request 3185](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3185)
    * [BitBucket pull request 2768](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2768)

1. Play logs as close to real time as possible.
    * [BitBucket pull request 3179](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3179)

1. Actor: fix distortion when loaded with BVH animation.
    * [BitBucket pull request 2957](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2957)
    * [BitBucket pull request 3183](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3183)

1. Fix many Ubuntu bionic compiler warnings and codecheck complaints
    * [BitBucket pull request 3145](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3145)

1. Rename OpenAL types to `remove _struct`
    * [BitBucket pull request 3154](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3154)

1. Update the gtest fork to fix c++17 bugs
    * [BitBucket pull request 3168](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3168)

1. Cross port dynamic bias noise parameters from ign-sensors (random walk)
    * [BitBucket pull request 3181](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3181)

## Gazebo 10

## Gazebo 10.x.x (202x-xx-xx)

1. DARTHeightmapShape: correctly load asymmetric terrains
    * [Pull request #2818](https://github.com/osrf/gazebo/pull/2818)

1. Set friction params in DARTCollision for dart 6.10
    * [Pull request #2781](https://github.com/osrf/gazebo/pull/2781)

1. Fix problem with automoc in CMake 3.17
    * [BitBucket pull request 3206](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3206/)

1. Added normals to depth camera sensor
    * [BitBucket pull request 3193](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3193/)

1. Prevent crash when subscribing to depth camera image topic
    * [BitBucket pull request 3197](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3197)

## Gazebo 10.2.0 (2020-01-31)

1. Fix gazebo build and run on Windows, Ogre 1.10
    * [BitBucket pull request 3174](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3174)

1. Fix missing road segments in camera sensors
    * [BitBucket pull request 3182](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3182)

1. Workaround for race condition when setting model scale.
    * [BitBucket pull request 3159](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3159)

1. Fix compilation of plugins with tbb and qt 5.14.
    * [BitBucket pull request 3164](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3164)
    * [Issue #2681](https://github.com/osrf/gazebo/issues/2681)

1. Fix plugin loading in example by fixing uninitialized variable in World and calling `sensors::run_once()`.
    * [BitBucket pull request 3059](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3059)
    * [BitBucket pull request 3173](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3173)

1. Windows: reduce WinSock header inclusion to limit name conflicts.
    * [BitBucket pull request 3158](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3158)

1. Windows patches to build default
    * [BitBucket pull request 3065](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3065)

1. Don't pass GCC linker options to Visual Studio linker.
    * [BitBucket pull request 3153](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3153)

1. Fix deadlock between `World::OnRequest` and `TopicManager::AddNode`.
    * [BitBucket pull request 3155](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3155)
    * [Issue #2679](https://github.com/osrf/gazebo/issues/2679)

1. Don't pass GCC linker options to Visual Studio linker.
    * [BitBucket pull request 3141](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3141)

1. Fix CMake 3.14 regression in `MSVC`/`PKG_CONFIG_FOUND` workaround.
    * [BitBucket pull request 3125](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3125)

1. ODEBallJoint: reduce console output.
    * [BitBucket pull request 3132](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3132)

1. VariableGearboxPlugin: use splines to support arbitrary smooth input-output gearbox profiles.
    * [BitBucket pull request 3073](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3073)

1. Fix moving model files in StaticMapPlugin
    * [BitBucket pull request 3123](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3123)

1. Fix crash when loading submesh with no bone assignments.
    * [BitBucket pull request 3122](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3122)

1. Fix cmake warnings about multi-line strings.
    * [BitBucket pull request 3138](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3138)
    * [Issue #2664](https://github.com/osrf/gazebo/issues/2664)

1. MeshManager: add .stlb file extension support
    * [BitBucket pull request 3124](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3124)
    * [BitBucket pull request 3128](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3128)

1. SystemPaths: fix race condition in PathDelimiter initialization, `missing call to sdf::addURIPath`.
    * [BitBucket pull request 3170](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3170)

1. Actor: update collision pose when using ActorPlugin.
    * [BitBucket pull request 3108](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3108)
    * [Issue #2433](https://github.com/osrf/gazebo/issues/2433)

1. Add Camera PreRender and PostRender events
    * [BitBucket pull request 3118](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3118)

1. Fix ColladaLoader wrong node weights caused by buffer overflow bug
    * [BitBucket pull request 3115](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3115)

1. ColladaLoader: use default value of 1 for stride parameter when unset.
    * [BitBucket pull request 3112](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3112)

1. TopicManager: lock subscriberMutex anywhere subscribedNodes is used
    * [BitBucket pull request 3096](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3096)

1. Export `OGRE-*` cmake variables in addition to `OGRE_*` variables
    * [BitBucket pull request 3109](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3109)

1. Fix race conditions in `Master::ProcessMessage` and `Publisher::OnPublishComplete`
    * [BitBucket pull request 3103](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3103)

1. Fix compilation of tests without DART installed.
    * [BitBucket pull request 3075](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3075)

1. Fix build with DART 6.9, using 32-bit float for dart heightmap scale.
    * [BitBucket pull request 3106](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3106)
    * [BitBucket pull request 3107](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3107)

1. Fix kinematic loops for DART 6.8, reverting to dart 6.7 behavior
    * [BitBucket pull request 3101](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3101)

1. Enable extra kinematic loop test for DART 6.8+
    * [BitBucket pull request 3104](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3104)

1. Lens flare: use light world pose at each time step instead of only at initialization
    * [BitBucket pull request 3093](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3093)

1. LinkPlot3dPlugin: read optional `<model>` tag to find links in nested models
    * [BitBucket pull request 3095](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3095)

1. Make the GPU laser warp artifact transparent
    * [BitBucket pull request 3100](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3100)

1. Added support for tracked vehicles
    * [BitBucket pull request 2652](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2652)
    * [BitBucket pull request 3116](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3116)
    * [BitBucket pull request 3140](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3140)
    * [BitBucket pull request 3148](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3148)
    * [BitBucket pull request 3149](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3149)
    * [Issue #863](https://github.com/osrf/gazebo/issues/863)

1. Fix windows plugin visibility.
    * [BitBucket pull request 3072](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3072)

1. Fix loading sdf with orthographic projection camera
    * [BitBucket pull request 3098](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3098)

1. Avoid windows macro `DELETE` conflict with `ignition::fuel_tools::REST::DELETE`.
    * [BitBucket pull request 3143](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3143)

2. Fixes for finding OGRE with CMake config files.
    * [BitBucket pull request 3126](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3126)

1. Fix missing link of ignition-common in `gazebo_common`
    * [BitBucket pull request 3127](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3127)

1. Find TBB in cmake config files.
    * [BitBucket pull request 3135](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3135)

1. Fix texture loading on OGRE 1.11, 1.12.
    * [BitBucket pull request 3150](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3150)

1. Fix linking of opende with `HAVE_BULLET`.
    * [BitBucket pull request 3151](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3151)

1. Allow gazebo rendering to compile with Ogre 1.11 / 1.12
    * [BitBucket pull request 3129](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3129)
    * [BitBucket pull request 3130](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3130)

1. Fix compilation against Ogre compiled in debug mode
    * [BitBucket pull request 3131](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3131)
    * [BitBucket pull request 3134](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3134)
    * [BitBucket pull request 3142](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3142)

1. SonarSensor: allow spherical collision shape.
    * [BitBucket pull request 3038](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3038)
    * [BitBucket pull request 3169](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3169)
    * [BitBucket pull request 3172](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3172)

1. Backport camera intrinsics feature
    * [BitBucket pull request 3099](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3099)

1. Fix CMake 3.14 regression in `MSVC`/`PKG_CONFIG_FOUND` workaround.
    * [BitBucket pull request 3152](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3152)

## Gazebo 10.1.0 (2019-03-28)

1. Refactor ODE gearbox joint implementation to match hinge joint
    * [BitBucket pull request 3048](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3048)

1. Fix kinematic loops for DART 6.7 and later
    * [BitBucket pull request 3086](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3086)
    * [Issue 2605](https://github.com/osrf/gazebo/issues/2605)

1. Windows: enable dynamic linking.
    * [BitBucket pull request 3068](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3068)

1. Windows: fixing path-related issues.
    * [BitBucket pull request 3069](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3069)

1. Windows: add setup.bat.in helper script template
    * [BitBucket pull request 3070](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3070)

1. Fix mal-formed pkgconfig file: don't prepend duplicate -l
    * [BitBucket pull request 3080](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3080)
    * [Issue 2600](https://github.com/osrf/gazebo/issues/2600)

1. Windows: fix test compilation
    * [BitBucket pull request 3082](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3082)

1. Heightmap: cast shadows if `<cast_shadows>` tag is set
    * [BitBucket pull request 3083](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3083)

1. Windows: ignore disabled interfaces in `Connection::GetLocalEndpoint()`
    * [BitBucket pull request 3079](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3079)

1. Update trigger\_light plugin example to use ignition-transport
    * [BitBucket pull request 3077](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3077)

1. Fix ColladaLoader to support mixamo models and fix skeleton animation loading
    * [BitBucket pull request 3084](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3084)
    * [BitBucket pull request 3071](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3071)
    * [Issue 2582](https://github.com/osrf/gazebo/issues/2582)

## Gazebo 10.0.0 (2019-01-31)

1. Improve performance of IntrospectionManager
    * [BitBucket pull request #3055](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3055)

1. Add geometry field to sonar.proto and private dataPtr to Actor class
    * [BitBucket pull request #3067](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3067)

1. Dart heightmaps with bullet and ODE collision detectors
    * [BitBucket pull request #2956](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2956)
    * [BitBucket pull request #3066](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3066)

1. Add record\_resources field to gazebo log msgs
    * [BitBucket pull request #2797](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2797)
    * [BitBucket pull request #3008](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3008)

1. Add methods to set dynamically Gaussian noise parameters
    * [BitBucket pull request #2931](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2931)

1. Use tension trajectory parameter on actor script animation
    * [BitBucket pull request #3019](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3019)

1. Add command to send a request with gz topic
    * [BitBucket pull request #2907](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2907)

1. Force vertical field of view to be lower than 180 degrees
    * [BitBucket pull request #2909](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2909)

1. Fixed issue which lead to reset of values on pressing enter in Joint Creator
    * [BitBucket pull request #2926](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2926)

1. Removed material block from shapes.world
    * [BitBucket pull request #2925](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2925)

1. Handle signal SIGTERM exactly the same way as SIGINT
    * [BitBucket pull request #2908](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2908)

1. Displaying light direction on the world tree
    * [BitBucket pull request 2912](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2912)

1. Plugin to initialize joint controller parameters
    * [BitBucket pull request #2751](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2751)
    * [Issue #1766](https://github.com/osrf/gazebo/issues/1766)

1. Fix Enter in Link Inspector
    * [BitBucket pull request #2901](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2901)

1. Display ODE contact points on disabled bodies
    * [BitBucket pull request #2709](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2709)
    * [Issue #2175](https://github.com/osrf/gazebo/issues/2175)

1. Update Wind Plugin to support being used by Lift Drag Plugin
    * [BitBucket pull request #2691](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2691)

1. Dart: create joints dynamically, support kinematic loops and HarnessPlugin
    * [BitBucket pull request #2762](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2762)
    * [Issue #903](https://github.com/osrf/gazebo/issues/903)

1. Model Editor: Bounding collision shapes
    * [BitBucket pull request #3004](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3004)

## Gazebo 9

## Gazebo 9.xx.x (202x-xx-xx)

1. Lens flare cleanup and colorization
    * [Pull request #2927](https://github.com/osrf/gazebo/pull/2927)

1. Fix support for camera with single channel floating point image format
    * [Pull request #2918](https://github.com/osrf/gazebo/pull/2918)

1. Fix performance metrics deadlock with multiple types of sensors
    * [Pull request #2917](https://github.com/osrf/gazebo/pull/2917)
    * [Issue #2902](https://github.com/osrf/gazebo/issues/2902)

1. Generate spot light shadow maps
    * [Pull request #2914](https://github.com/osrf/gazebo/pull/2914)

## Gazebo 9.16.0 (2020-11-24)

1. Updated the version of TinyOBJLoader from 1.0.0 to 2.0.0rc8.
    * [Pull request #2885](https://github.com/osrf/gazebo/pull/2885)

1. Support resource files with spaces
    * [Pull request #2877](https://github.com/osrf/gazebo/pull/2877)

1. Fix physics based sensor update rate in lockstep mode
    * [Pull request #2863](https://github.com/osrf/gazebo/pull/2863)

1. Added Profiler to gazebo::rendering and gzclient
    * [Pull request #2836](https://github.com/osrf/gazebo/pull/2836)

1. Fix segfault when deleting an model that's being manipulated
    * [Pull request #2856](https://github.com/osrf/gazebo/pull/2856)

1. SimpleTrackedVehiclePlugin: fix for boost 1.74
    * [Pull request #2865](https://github.com/osrf/gazebo/pull/2865)

1. Add mutex to make Sensor::SetActive threadsafe
    * [Pull request #2871](https://github.com/osrf/gazebo/pull/2871)

## Gazebo 9.15.0 (2020-09-30)

1. More enhancement for Windows build
    * [Pull request #2789](https://github.com/osrf/gazebo/pull/2789)

1. Fixed fails for OSX: Added using namespace boost::placeholders
    * [Pull request #2809](https://github.com/osrf/gazebo/pull/2809)

1. Add profiler
    * [Pull request #2813](https://github.com/osrf/gazebo/pull/2813)

1. ColladaLoader: fix cases where VERTEX and NORMAL share same polylist <p>
    * [Pull request #2825](https://github.com/osrf/gazebo/pull/2825)

1. Fix assumptions that CMAKE\_INSTALL\_\*DIR paths are relative
    * [Pull request #2779](https://github.com/osrf/gazebo/pull/2779)

1. Fix reflectance values for objects without a reflectance map
    * [Pull request #2833](https://github.com/osrf/gazebo/pull/2833)

1. Remove lighting param
    * [Pull request #2840](https://github.com/osrf/gazebo/pull/2840)

1. Publish performance metrics
    * [Pull request #2819](https://github.com/osrf/gazebo/pull/2819)

1. Improve transport::Publisher reliability
    * [Pull request #2725](https://github.com/osrf/gazebo/pull/2725)

## Gazebo 9.14.0 (2020-08-07)

1. Lockstep between sensors and physics
    * [Pull request #2793](https://github.com/osrf/gazebo/pull/2793)

1. Fix race condition on Publisher shutdown
    * [Pull request #2812](https://github.com/osrf/gazebo/pull/2812)

## Gazebo 9.13.2 (2020-07-20)

1. Fix sensor update rate throttling when new sensors are spawned
    * [Pull request #2784](https://github.com/osrf/gazebo/pull/2784)

1. Fix crash when collision size is zero
    * [Pull request #2768](https://github.com/osrf/gazebo/pull/2768)

1. LensFlare: initialize OGRE compositors during plugin initialization
    * [Pull request #2764](https://github.com/osrf/gazebo/pull/2764)

1. Fix pkg-config boost entries for Ubuntu Focal
    * [Pull request #2751](https://github.com/osrf/gazebo/pull/2751)

1. Fixes for ARM: FindSSE, TrackedVehiclePlugin and PluginInterfaceTest
    * [Pull request #2754](https://github.com/osrf/gazebo/pull/2754)
    * [Pull request #2748](https://github.com/osrf/gazebo/pull/2748)

## Gazebo 9.13.1 (2020-05-28)

1. Fix multiple reflectance maps and improve performance
    * [Pull request #2742](https://github.com/osrf/gazebo/pull/2742)

## Gazebo 9.13.0 (2020-04-03)

1. Use target based compile options to specify C++ standard
    * [BitBucket pull request 3199](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3199)

1. Add SetHeight method to HeightmapShape class
    * [BitBucket pull request 3210](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3210)

1. Added GAZEBO\_VISIBLE for WheelTrackedVehiclePlugin
    * [BitBucket pull request 3211](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3211)

1. Fix bounding box calculation for visuals that have links with pose offset
    * [BitBucket pull request 3196](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3196)

1. Added reflectance to depth camera sensor
    * [BitBucket pull request 3194](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3194/)

1. Fix problem with automoc in CMake 3.17
    * [BitBucket pull request 3206](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3206/)

1. Fix macOS gui examples compilation
    * [BitBucket pull request 3209](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3209/)

1. Added normals to depth camera sensor
    * [BitBucket pull request 3193](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3193/)

1. Prevent crash when subscribing to depth camera image topic
    * [BitBucket pull request 3197](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3197)

## Gazebo 9.12.0 (2020-01-31)

1. Fix gazebo9 build and run on Windows, Ogre 1.10
    * [BitBucket pull request 3174](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3174)

1. Added support for flippers in SimpleTrackedVehiclePlugin.
    * [BitBucket pull request 3149](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3149)

1. Fix missing road segments in camera sensors
    * [BitBucket pull request 3182](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3182)

1. Workaround for race condition when setting model scale.
    * [BitBucket pull request 3159](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3159)

1. Fix compilation of plugins with tbb and qt 5.14.
    * [BitBucket pull request 3164](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3164)
    * [Issue #2681](https://github.com/osrf/gazebo/issues/2681)

1. Fix plugin loading in example by fixing uninitialized variable in World and calling `sensors::run_once()`.
    * [BitBucket pull request 3059](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3059)
    * [BitBucket pull request 3173](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3173)

1. Windows: reduce WinSock header inclusion to limit name conflicts.
    * [BitBucket pull request 3158](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3158)

1. Fix deadlock between `World::OnRequest` and `TopicManager::AddNode`.
    * [BitBucket pull request 3155](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3155)
    * [Issue #2679](https://github.com/osrf/gazebo/issues/2679)

1. Don't pass GCC linker options to Visual Studio linker.
    * [BitBucket pull request 3153](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3153)

1. Fix CMake 3.14 regression in `MSVC`/`PKG_CONFIG_FOUND` workaround.
    * [BitBucket pull request 3152](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3152)

1. Add Twist message and use in `cmd_vel_twist` of TrackedVehiclePlugin.
    * [BitBucket pull request 3116](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3116)

1. Allow multiple instances of SimpleTrackedVehiclePlugin.
    * [BitBucket pull request 3140](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3140)
    * [BitBucket pull request 3148](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3148)

1. ODEBallJoint: reduce console output.
    * [BitBucket pull request 3132](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3132)

1. VariableGearboxPlugin: use splines to support arbitrary smooth input-output gearbox profiles.
    * [BitBucket pull request 3073](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3073)

1. Fix moving model files in StaticMapPlugin
    * [BitBucket pull request 3123](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3123)

1. Fix crash when loading submesh with no bone assignments.
    * [BitBucket pull request 3122](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3122)

1. Fix cmake warnings about multi-line strings.
    * [BitBucket pull request 3138](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3138)
    * [Issue #2664](https://github.com/osrf/gazebo/issues/2664)

1. MeshManager: add .stlb file extension support
    * [BitBucket pull request 3124](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3124)
    * [BitBucket pull request 3128](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3128)

1. SystemPaths: fix race condition in PathDelimiter initialization, `missing call to sdf::addURIPath`.
    * [BitBucket pull request 3170](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3170)

1. Actor: update collision pose when using ActorPlugin.
    * [BitBucket pull request 3108](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3108)
    * [Issue #2433](https://github.com/osrf/gazebo/issues/2433)

## Gazebo 9.11.0 (2019-08-29)

1. Add Camera PreRender and PostRender events
    * [BitBucket pull request 3118](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3118)

1. Fix ColladaLoader wrong node weights caused by buffer overflow bug
    * [BitBucket pull request 3115](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3115)


## Gazebo 9.10.0 (2019-07-12)

1. ColladaLoader: use default value of 1 for stride parameter when unset.
    * [BitBucket pull request 3112](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3112)

1. TopicManager: lock subscriberMutex anywhere subscribedNodes is used
    * [BitBucket pull request 3096](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3096)

1. Export `OGRE-*` cmake variables in addition to `OGRE_*` variables
    * [BitBucket pull request 3109](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3109)

1. Fix race conditions in `Master::ProcessMessage` and `Publisher::OnPublishComplete`
    * [BitBucket pull request 3103](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3103)

## Gazebo 9.9.0 (2019-05-23)

1. Backport camera intrinsics feature
    * [BitBucket pull request 3099](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3099)

1. Fix kinematic loops for DART 6.8, reverting to dart 6.7 behavior
    * [BitBucket pull request 3101](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3101)

1. Enable extra kinematic loop test for DART 6.8+
    * [BitBucket pull request 3104](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3104)

1. Lens flare: use light world pose at each time step instead of only at initialization
    * [BitBucket pull request 3093](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3093)

1. LinkPlot3dPlugin: read optional `<model>` tag to find links in nested models
    * [BitBucket pull request 3095](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3095)

1. Refactor ODE gearbox joint implementation to match hinge joint
    * [BitBucket pull request 3048](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3048)

1. Make the GPU laser warp artifact transparent
    * [BitBucket pull request 3100](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3100)

1. Added support for tracked vehicles
    * [BitBucket pull request 2652](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2652)
    * [Issue #863](https://github.com/osrf/gazebo/issues/863)

1. Fix loading sdf with orthographic projection camera
    * [BitBucket pull request 3098](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3098)

## Gazebo 9.8.0 (2019-04-10)

1. Fix kinematic loops for DART 6.7 and later
    * [BitBucket pull request 3086](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3086)
    * [Issue 2605](https://github.com/osrf/gazebo/issues/2605)

1. Port introspection manager performance fix
    * [BitBucket pull request 3074](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3074)

1. Windows: enable dynamic linking.
    * [BitBucket pull request 3068](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3068)

1. Windows: fixing path-related issues.
    * [BitBucket pull request 3069](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3069)

1. Windows: add setup.bat.in helper script template
    * [BitBucket pull request 3070](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3070)

1. Fix mal-formed pkgconfig file: don't prepend duplicate -l
    * [BitBucket pull request 3080](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3080)
    * [Issue 2600](https://github.com/osrf/gazebo/issues/2600)

## Gazebo 9.7.0 (2019-03-13)

1. Windows: fix test compilation
    * [BitBucket pull request 3082](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3082)

1. Heightmap: cast shadows if `<cast_shadows>` tag is set
    * [BitBucket pull request 3083](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3083)

1. Windows: ignore disabled interfaces in `Connection::GetLocalEndpoint()`
    * [BitBucket pull request 3079](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3079)

1. Update trigger\_light plugin example to use ignition-transport
    * [BitBucket pull request 3077](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3077)

1. Fix ColladaLoader to support mixamo models and fix skeleton animation loading
    * [BitBucket pull request 3084](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3084)
    * [BitBucket pull request 3071](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3071)
    * [Issue 2582](https://github.com/osrf/gazebo/issues/2582)

1. Improve gpu laser and its sensor shutdown
    * [BitBucket pull request 3061](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3061)
    * [BitBucket pull request 3026](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3026)

1. Added KeysToCmdVelPlugin for controlling robots using keyboard from gzclient
    * [BitBucket pull request 3057](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3057)

1. Windows patches to build gazebo9
    * [BitBucket pull request 3060](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3060)

1. Add MisalignmentPlugin which reports alignment between two poses
    * [BitBucket pull request 2896](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2896)


## Gazebo 9.6.0 (2018-12-17)

1. Don't search for boost signals component
    * [BitBucket pull request 3050](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3050)
    * [Issue 2577](https://github.com/osrf/gazebo/issues/2577)

1. Fix saving heightmap cache
    * [BitBucket pull request 3044](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3044)
    * [Issue 2572](https://github.com/osrf/gazebo/issues/2572)

1. Fix GUI plugins on Bionic + gz9
    * [BitBucket pull request 3041](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3041)
    * [Issue 2541](https://github.com/osrf/gazebo/issues/2541)

1. Add method to get the link visual elements
    * [BitBucket pull request 3040](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3040)
    * backport of [BitBucket pull request 2900](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2900)

1. Add Plugin::LoadParam to improve plugin interface
    * [BitBucket pull request 3047](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3047)

1. Fix gzclient on mojave with Qt 5.12
    * [BitBucket pull request 3051](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3051)
    * [Issue 2531](https://github.com/osrf/gazebo/issues/2531)

1. Switch Time::Sleep from CLOCK\_REALTIME to CLOCK\_MONOTONIC on Linux
    * [BitBucket pull request 3037](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3037)

1. Change sleep time larger than resolution message from gzerr to gzlog
    * [BitBucket pull request 3036](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3036)

1. Fix DARTHingeJoint::SetAxis implementation (issue 2505)
    * [BitBucket pull request 3005](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3005)
    * [Issue 2505](https://github.com/osrf/gazebo/issues/2505)

1. Plugin to initialize joint controller parameters
    * [BitBucket pull request #3031](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3031)
    * [BitBucket pull request #2751](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2751)
    * [Issue 1766](https://github.com/osrf/gazebo/issues/1766)

1. static_map_plugin.cc: remove backup folder
    * [BitBucket pull request #3023](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3023)

1. Fix regression test build -> gazebo9
    * [BitBucket pull request #3046](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3046)

## Gazebo 9.5.0 (2018-11-19)

1. Fix model bounding box
    * [BitBucket pull request 3033](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3033)

1. Skip skyx in SSAO plugin
    * [BitBucket pull request 3028](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3028)

1. Boost 1.68 support
    * [BitBucket pull request 3030](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3030)

1. Use new sha1.hpp header location for recent boost
    * [BitBucket pull request 3029](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3029)

1. Joint.hh: fix documentation for Set{Upp|Low}erLimit
    * [BitBucket pull request 3027](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3027)

1. Fix for revolute2 joints that prevents links from teleporting to origin
    * [BitBucket pull request 3024](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3024)
    * [Issue 2239](https://github.com/osrf/gazebo/issues/2239)

1. Fix for BulletFixedJoint when used with inertial matrices with non-zero values on their off-diagonal
    * [BitBucket pull request 3010](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3010)

1. Adding WheelSlipPlugin: for adding wheel slip using ODE's contact parameters
    * [BitBucket pull request 2950](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2950)
    * [BitBucket pull request 2976](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2976)
    * [BitBucket pull request 2997](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2997)

1. Adding JointController::SetForce API and extra test for WheelSlipPlugin
    * [BitBucket pull request 2976](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2976)


## Gazebo 9.4.1 (2018-09-19)

1. Revert pr 2923: "Handle signal SIGTERM exactly the same way as SIGINT"
    * [BitBucket pull request 3018](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3018)
    * Reverts [BitBucket pull request 2923](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2923)
    * Some discussion in [BitBucket pull request 3014](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3014)


## Gazebo 9.4.0 (2018-09-18)

1. Fix for the spawning light issue. This fix allows a light's visual to be
   turned on/off.
    * [BitBucket pull request 3011](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3011)

1. Add joystick plugin and demo world
    * [BitBucket pull request 2895](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2895)

1. Support toggling light visuals.
    * [BitBucket pull request 3011](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3011)

1. Improve shutdown speed.
    * [BitBucket pull request 3014](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3014)

1. Fix vertical lidar rays.
    * [BitBucket pull request 3013](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3013)

1. Only use active interfaces in gazebo/transport.
    * [BitBucket pull request 3009](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3009)

1. Trigger the stop event on sigint/sigterm.
    * [BitBucket pull request 2993](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2993)

1. Include SDF header in rendering::Distortion
    * [BitBucket pull request 3012](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3012)

1. More documentation to Model::CreateJoint()
    * [BitBucket pull request 3002](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3002)

1. Improve ODE slip parameter behavior with multiple contact points
    * [BitBucket pull request 2965](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2965)

1. Fix manipulating links in the model editor
    * [BitBucket pull request 2999](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2999)
    * [Issue 2487](https://github.com/osrf/gazebo/issues/2487)

1. LOD skirt length
    * [BitBucket pull request 2968](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2968)

1. Patch for visual message process
    * [BitBucket pull request 2983](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2983)

1. Print joint_cmd deprecation warnings only one time
    * [BitBucket pull request 2966](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2966)
    * [Issue 2393](https://github.com/osrf/gazebo/issues/2393)


## Gazebo 9.3.1 (2018-08-08)

1. Fix for the spawning light issue
    * [BitBucket pull request 3003](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3003)

## Gazebo 9.3.0 (2018-07-28)

1. Add a LED plugin blinking visual objects
    * [BitBucket pull request 2994](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2994)

1. Require ignition-fuel-tools 1.2 when finding package
    * [BitBucket pull request 2992](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2992)
    * [Issue 2494](https://github.com/osrf/gazebo/issues/2494)

1. Add a flashlight plugin blinking lights attached on a model
    * [BitBucket pull request 2961](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2961)

1. Fix manipulating links in the model editor
    * [BitBucket pull request 2996](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2996)
    * [Issue 2487](https://github.com/osrf/gazebo/issues/2487)

## Gazebo 9.2.0 (2018-07-10)

1. Fix SetCrop for multiple cameras and add SetCrop test
    * [BitBucket pull request 2967](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2967)

1. Fix check terrain layer count in height map
    * [BitBucket pull request 2978](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2978)

1. Fix build on homebrew with protobuf 3.6
    * [BitBucket pull request 2984](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2984)

1. Attach lights to links cleanup and deprecate GetLight functions
    * [BitBucket pull request #2871](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2871)


## Gazebo 9.1.1 (2018-06-08)

1. Set the default model database URI to avoid a redirect
    * [BitBucket pull request 2971](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2971)


## Gazebo 9.1.0 (2018-06-01)

1. Fuel: Support models with full Fuel URLs in <uri>
    * [BitBucket pull request 2962](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2962)

1. Fuel: List models by owner on insert menu
    * [BitBucket pull request 2949](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2949)

1. Fueltools useragent
    * [BitBucket pull request 2924](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2924)

1. Env var to enable Ignition Fuel
    * [BitBucket pull request 2860](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2860)

1. Find DART with CONFIG to fix homebrew issue
    * [BitBucket pull request 2919](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2919)
    * [homebrew-simulation issue 384](https://github.com/osrf/homebrew-simulation/issues/384)

1. Added missing OGRE headers
    * [BitBucket pull request 2894](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2894)

1. Handle signal SIGTERM exactly the same way as SIGINT
    * [BitBucket pull request 2923](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2923)

1. Support custom find file callbacks
    * [BitBucket pull request 2948](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2948)

1. Fix empty visual bounding box
    * [BitBucket pull request 2934](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2934)

1. Make override keywords consistent in joint classes to fix clang warnings
    * [BitBucket pull request 2869](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2869)
    * [BitBucket pull request 2881](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2881)

1. Fix BulletHingeJoint limits when child link has off-diagonal inertia
    * [BitBucket pull request 2883](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2883)

1. Print some bullet console warnings only once
    * [BitBucket pull request 2866](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2866)

1. Fix getting joint limits for BulletHingeJoint
    * [BitBucket pull request 2959](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2959)

1. Fix build on hombrew with boost 1.67
    * [BitBucket pull request 2954](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2954)

1. Set the default model database URI to avoid a redirect.
    * [BitBucket pull request 2970](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2970)

1. Save model materials and meshes when logging
    * [BitBucket pull request 2811](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2811)

1. Add Screen Space Ambient Occlusion visual plugin
    * [BitBucket pull request 2916](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2916)
    * [BitBucket pull request 2947](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2947)

1. Fix ray intersection check in Scene::FirstContact
    * [BitBucket pull request 2945](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2945)

1. Fix camera view control inside bounding box of large meshes
    * [BitBucket pull request 2932](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2932)

1. Fix compilation with boost 1.67
    * [BitBucket pull request 2937](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2937)

1. Fix compilation with ffmpeg4
    * [BitBucket pull request 2942](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2942)

1. Fix Joint::SetPosition for HingeJoint
    * [BitBucket pull request 2892](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2892)
    * [Issue 2430](https://github.com/osrf/gazebo/issues/2430)

1. Fix mouse movement ogre assertion error
    * [BitBucket pull request 2928](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2928)

1. use QVERIFY() around qFuzzyCompare statements
    * [BitBucket pull request 2936](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2936)

1. Fix normal maps on ubuntu with OGRE 1.9 and disable on OSX
    * [BitBucket pull request 2917](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2917)

1. Support lens flare occlusion
    * [BitBucket pull request 2915](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2915)

1. Diagnostics: record timing statistics instead of all timestamps
    * [BitBucket pull request 2821](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2821)

1. Add trigger_light example for ContainPlugin tutorial
    * [BitBucket pull request 2918](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2918)
    * [BitBucket pull request 2929](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2929)

1. Do not load model plugins during log playback.
    * [BitBucket pull request 2884](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2884)
    * [Issue 2427](https://github.com/osrf/gazebo/issues/2427)

1. State log file playback can cause a sensor manager assert if there is
   a large period of inactivity. This PR outputs warning messages instead of
   using asserts.
    * [BitBucket pull request 2893](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2893)
    * [BitBucket pull request 2921](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2921)

1. Fix model insertions during log playback.
    * [BitBucket pull request 2890](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2890)
    * [Issue 2297](https://github.com/osrf/gazebo/issues/2297)
    * [Issue 2428](https://github.com/osrf/gazebo/issues/2428)

1. Simplify search logic for Qt5
    * [BitBucket pull request 2911](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2911)
    * [Issue 2419](https://github.com/osrf/gazebo/issues/2419)

1. Fix log recording, only call sdf::initFile once
    * [BitBucket pull request 2885](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2885)
    * [Issue 2425](https://github.com/osrf/gazebo/issues/2425)

1. Ensure sdf inertia values are consistent
    * [BitBucket pull request 2867](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2867)
    * [Issue 2367](https://github.com/osrf/gazebo/issues/2367)

1. Fix gazebo7 + ogre 1.8 build error
    * [BitBucket pull request 2878](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2878)

1. Fix OBJLoader when mesh has invalid material
    * [BitBucket pull request 2888](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2888)

1. Fix clang warnings in LaserView and EnumIface
    * [BitBucket pull request 2891](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2891)

1. Add support for moving geometry to ContainPlugin
    * [BitBucket pull request 2886](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2886)

1. Support python3 with check_test_ran.py
    * [BitBucket pull request 2902](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2902)

1. Don't shut down gazebo when removing a world
    * [BitBucket pull request 2511](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2511)

1. Fix undefined behavior in ODESliderJoint
    * [BitBucket pull request 2905](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2905)

1. Fix loading collada mesh that contains multiple texcoord sets with same offset
    * [BitBucket pull request 2899](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2899)

1. Fix race conditions during client startup, and introduce Node::TryInit()
    * [BitBucket pull request 2897](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2897)

1. Add support for Actor collisions.
    * [BitBucket pull request 2875](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2875)

1. Process insertions and deletions on gz log echo
    * [BitBucket pull request 2608](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2608)
    * [Issue 2136](https://github.com/osrf/gazebo/issues/2136)

1. Added a plugin to detect if an entity is inside a given volume in space
    * [BitBucket pull request 2780](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2780)

1. Add Static Map Plugin for creating textured map model
    * [BitBucket pull request 2834](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2834)

1. Fix deadlock when publishing to ~/light/factory topic
    * [BitBucket pull request 2872](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2872)

1. Added a plugin to detect if an entity is inside a given volume in space
    * [BitBucket pull request 2870](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2870)

1. Load actor plugin on ~/factory
    * [BitBucket pull request 2855](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2855)

1. Add support for 16 bit Grayscale and RGB camera image types.
    * [BitBucket pull request 2852](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2852)

1. Add Visual::SetMaterialShaderParam function for setting shader parameters.
    * [BitBucket pull request 2863](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2863)

1. Adding accessors for velocity in ENU frame for gps sensor
    * [BitBucket pull request 2854](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2854)

1. Fix DEM min elevation
    * [BitBucket pull request 2868](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2868)

1. Update Color Clamp function
    * [BitBucket pull request 2859](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2859)

1. Initialize laser retro value
    * [BitBucket pull request 2841](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2841)

1. Allow marker requests to be received from server plugins.
    * [BitBucket pull request 2858](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2858)


## Gazebo 9.0.0 (2018-01-25)

1. Update to `ign-transport4`, `ign-msgs1`, `ign-math4`. Added dependency on
   only `sdformat6`, removing `sdformat5`.
    * [BitBucket pull request #2843](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2843)

1. Provide option to preserve world velocity in Joint::SetPosition
    * [BitBucket pull request #2814](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2814)
    * [Issue 2111](https://github.com/osrf/gazebo/issues/2111)

1. Rename `BUILD_TYPE_*` macros to `GAZEBO_BUILD_TYPE_*`
    * [BitBucket pull request #2846](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2846)
    * [Issue 2343](https://github.com/osrf/gazebo/issues/2343)

1. Added World::SDF()
    * [BitBucket pull request #2708](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2708)

1. Fix compile error with due to using gazebo::common::Color with sdformat 6
    * [BitBucket pull request #2786](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2786)

1. [Ignition Fuel Tools](https://ignitionrobotics.org/libs/fuel%20tools) integration:
  1. Ignition Fuel support - model list
    * [BitBucket pull request #2796](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2796)
  1. Download a model from Ignition Fuel
    * [BitBucket pull request #2800](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2800)
  1. Export dependency on ignition-fuel-tools in cmake and pkgconfig files if it's found
    * [BitBucket pull request #2850](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2850)

1. Avoid race condition between multiple writers to the same connection
    * A contribution from Hendrik Skubch
    * [BitBucket pull request #2826](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2826)

1. Deprecate gazebo::common::Color
    * [BitBucket pull request #2818](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2818)
    * [BitBucket pull request #2831](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2831)
    * [BitBucket pull request #2837](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2837)
    * [BitBucket pull request #2838](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2838)
    * [BitBucket pull request #2842](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2842)

1. Updates to MovableText
    * [BitBucket pull request #2839](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2839)

1. Mark constructors as explicit to fix cppcheck warnings
    * [BitBucket pull request #2790](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2790)
    * [BitBucket pull request #2792](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2792)
    * [BitBucket pull request #2795](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2795)
    * [BitBucket pull request #2822](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2822)

1. Try finding both ignition math 3 or 4 until we switch to 4
    * [BitBucket pull request #2783](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2783)

1. Replaced use of ignition::msgs::ImageStamped with ignition::msgs::Image
    * [BitBucket pull request #2781](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2781)

1. Fix missing includes for boost lexical cast
    * [BitBucket pull request #2784](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2784)

1. Try finding both sdformat 5 and 6 until we switch to 6
    * [BitBucket pull request #2750](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2750)

1. HarnessPlugin: PIMPL and allow re-attaching
    * [BitBucket pull request #2697](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2697)

1. DART: Update contact information also if physics engine is disabled
    * [BitBucket pull request #2704](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2704)

1. Integration of DART-6
    * [BitBucket pull request #2547](https://github.com/osrf/gazebo/

1. Image Viewer: fix QImage::Format used to display grayscale images
    * A contribution from Julien Lecoeur
    * [BitBucket pull request #2812](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2812)

1. Fix compilation of some tests on Windows
    * A contribution from Silvio Traversaro
    * [BitBucket pull request #2699](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2699)

1. Remove Gazebo 8 deprecations
    * [BitBucket pull request #2605](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2605)
    * [BitBucket pull request #2607](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2607)
    * [BitBucket pull request #2603](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2603)
    * [BitBucket pull request #2604](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2604)
    * [BitBucket pull request #2627](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2627)

1. Bullet: sending feedback on contact points on depth 0 as well
    * [BitBucket pull request #2630](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2630/)

1. Deprecate functions to set linear/angular acceleration
    * [BitBucket pull request #2622](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2622)

1. Added GpuLaserDataIterator
    * [BitBucket pull request #2637](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2637)

1. Added possibility to enforce contact computation
    * [BitBucket pull request #2629](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2629/)

1. Add function to retrieve scoped sensors name in multi-nested model
    * [BitBucket pull request #2676](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2676)


## Gazebo 8

## Gazebo 8.X.X (201X-XX-XX)

1. Use new sha1.hpp header location for recent boost
    * [BitBucket pull request 3029](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3029)

1. Joint.hh: fix documentation for Set{Upp|Low}erLimit
    * [BitBucket pull request 3027](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3027)

1. Fix for revolute2 joints that prevents links from teleporting to origin
    * [BitBucket pull request 3024](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3024)
    * [Issue 2239](https://github.com/osrf/gazebo/issues/2239)

1. Include SDF header in rendering::Distortion
    * [BitBucket pull request 3012](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3012)

1. More documentation to Model::CreateJoint()
    * [BitBucket pull request 3002](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3002)

1. Improve ODE slip parameter behavior with multiple contact points
    * [BitBucket pull request 2965](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2965)

1. Fix for BulletFixedJoint when used with inertial matrices with non-zero values on their off-diagonal
    * [BitBucket pull request 3010](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3010)

1. Fix manipulating links in the model editor
    * [BitBucket pull request 2999](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2999)
    * [Issue 2487](https://github.com/osrf/gazebo/issues/2487)

1. LOD skirt length
    * [BitBucket pull request 2968](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2968)

1. Patch for visual message process
    * [BitBucket pull request 2983](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2983)

1. Print joint_cmd deprecation warnings only one time
    * [BitBucket pull request 2966](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2966)
    * [Issue 2393](https://github.com/osrf/gazebo/issues/2393)

1. Adding WheelSlipPlugin: for adding wheel slip using ODE's contact parameters
    * [BitBucket pull request 2950](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2950)

1. Adding JointController::SetForce API and extra test for WheelSlipPlugin
    * [BitBucket pull request 2976](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2976)


## Gazebo 8.6.0 (2018-06-26)

1. Fix SetCrop for multiple cameras and add SetCrop test
    * [BitBucket pull request 2967](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2967)

1. Fix check terrain layer count in height map
    * [BitBucket pull request 2978](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2978)

1. Fix build on homebrew with protobuf 3.6
    * [BitBucket pull request 2984](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2984)

1. Fix GpuRaySensor vertical rays
    * [BitBucket pull request 2955](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2955)


## Gazebo 8.5.0 (2018-06-08)

1. Fix BulletHingeJoint limits when child link has off-diagonal inertia
    * [BitBucket pull request 2883](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2883)

1. Print some bullet console warnings only once
    * [BitBucket pull request 2866](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2866)

1. Fix getting joint limits for BulletHingeJoint
    * [BitBucket pull request 2959](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2959)

1. Fix build on hombrew with boost 1.67
    * [BitBucket pull request 2954](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2954)

1. Set the default model database URI to avoid a redirect.
    * [BitBucket pull request 2970](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2970)

1. Save model materials and meshes when logging
    * [BitBucket pull request 2811](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2811)

1. Add Screen Space Ambient Occlusion visual plugin
    * [BitBucket pull request 2916](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2916)
    * [BitBucket pull request 2947](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2947)

1. Fix ray intersection check in Scene::FirstContact
    * [BitBucket pull request 2945](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2945)

1. Fix camera view control inside bounding box of large meshes
    * [BitBucket pull request 2932](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2932)

1. Fix compilation with boost 1.67
    * [BitBucket pull request 2937](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2937)

1. Fix compilation with ffmpeg4
    * [BitBucket pull request 2942](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2942)

1. Fix Joint::SetPosition for HingeJoint
    * [BitBucket pull request 2892](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2892)
    * [Issue 2430](https://github.com/osrf/gazebo/issues/2430)

1. Fix mouse movement ogre assertion error
    * [BitBucket pull request 2928](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2928)

1. use QVERIFY() around qFuzzyCompare statements
    * [BitBucket pull request 2936](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2936)

1. Fix normal maps on ubuntu with OGRE 1.9 and disable on OSX
    * [BitBucket pull request 2917](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2917)

1. Support lens flare occlusion
    * [BitBucket pull request 2915](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2915)

1. Diagnostics: record timing statistics instead of all timestamps
    * [BitBucket pull request 2821](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2821)

1. Add trigger_light example for ContainPlugin tutorial
    * [BitBucket pull request 2918](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2918)
    * [BitBucket pull request 2929](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2929)

1. Do not load model plugins during log playback.
    * [BitBucket pull request 2884](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2884)
    * [Issue 2427](https://github.com/osrf/gazebo/issues/2427)

1. State log file playback can cause a sensor manager assert if there is
   a large period of inactivity. This PR outputs warning messages instead of
   using asserts.
    * [BitBucket pull request 2893](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2893)
    * [BitBucket pull request 2921](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2921)

1. Fix model insertions during log playback.
    * [BitBucket pull request 2890](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2890)
    * [Issue 2297](https://github.com/osrf/gazebo/issues/2297)
    * [Issue 2428](https://github.com/osrf/gazebo/issues/2428)

1. Simplify search logic for Qt5
    * [BitBucket pull request 2911](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2911)
    * [Issue 2419](https://github.com/osrf/gazebo/issues/2419)

1. Fix log recording, only call sdf::initFile once
    * [BitBucket pull request 2885](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2885)
    * [Issue 2425](https://github.com/osrf/gazebo/issues/2425)

1. Ensure sdf inertia values are consistent
    * [BitBucket pull request 2867](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2867)
    * [Issue 2367](https://github.com/osrf/gazebo/issues/2367)

1. Fix OBJLoader when mesh has invalid material
    * [BitBucket pull request 2888](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2888)

1. Fix clang warnings in LaserView and EnumIface
    * [BitBucket pull request 2891](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2891)

1. Add support for moving geometry to ContainPlugin
    * [BitBucket pull request 2886](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2886)

1. Support python3 with check_test_ran.py
    * [BitBucket pull request 2902](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2902)

1. Don't shut down gazebo when removing a world
    * [BitBucket pull request 2511](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2511)

1. Fix undefined behavior in ODESliderJoint
    * [BitBucket pull request 2905](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2905)

1. Fix loading collada mesh that contains multiple texcoord sets with same offset
    * [BitBucket pull request 2899](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2899)

1. Fix race conditions during client startup, and introduce Node::TryInit()
    * [BitBucket pull request 2897](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2897)


## Gazebo 8.3.0 (2018-02-10)

1. Add support for Actor collisions.
    * [BitBucket pull request 2875](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2875)

1. Process insertions and deletions on gz log echo
    * [BitBucket pull request 2608](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2608)
    * [Issue 2136](https://github.com/osrf/gazebo/issues/2136)

1. Added a plugin to detect if an entity is inside a given volume in space
    * [BitBucket pull request 2780](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2780)

1. Add Static Map Plugin for creating textured map model
    * [BitBucket pull request 2834](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2834)

1. Added a plugin to detect if an entity is inside a given volume in space
    * [BitBucket pull request 2870](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2870)

1. Load actor plugin on ~/factory
    * [BitBucket pull request 2855](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2855)

1. Add support for 16 bit Grayscale and RGB camera image types.
    * [BitBucket pull request 2852](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2852)

1. Add Visual::SetMaterialShaderParam function for setting shader parameters.
    * [BitBucket pull request 2863](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2863)

1. Adding accessors for velocity in ENU frame for gps sensor
    * [BitBucket pull request 2854](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2854)

1. Fix DEM min elevation
    * [BitBucket pull request 2868](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2868)

1. Update Color Clamp function
    * [BitBucket pull request 2859](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2859)

1. Fix inserting models with invalid submesh
    * [BitBucket pull request 2828](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2828)

1. Move Connection header buffer from heap to stack to avoid race condition.
    * [BitBucket pull request 2844](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2844)

1. Initialize laser retro value
    * [BitBucket pull request 2841](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2841)

1. Shadow improvements
    * [BitBucket pull request 2805](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2805)

1. Add light as child of link
    * [BitBucket pull request 2807](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2807)
    * [BitBucket pull request 2872](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2872)
    * [Issue 900](https://github.com/osrf/gazebo/issues/900)

1. Add camera lens flare effect
    * [BitBucket pull request 2806](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2806)
    * [BitBucket pull request 2829](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2829)

1. Image Viewer: fix QImage::Format used to display grayscale images
    * [BitBucket pull request #2813](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2813)

1. Fix gazebo8 homebrew build (support tinyxml2 6.0.0)
    * [BitBucket pull request 2823](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2823)
    * [ign-common issue 28](https://github.com/ignitionrobotics/ign-common/issues/28)

1. Allow marker requests to be received from server plugins.
    * [BitBucket pull request 2858](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2858)

1. Call DisconnectNewImageFrame in the CameraPlugin destructor
    * [BitBucket pull request 2815](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2815)

1. Add Static Map Plugin for creating textured map model
    * [BitBucket pull request 2834](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2834)

## Gazebo 8.2.0 (2017-12-10)

1. Fix Collision::GetWorldPose for non-canonical links (and friction directions)
    * [BitBucket pull request 2702](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2702)
    * [Issue 2068](https://github.com/osrf/gazebo/issues/2068)

1. Joint control menu highlight active
    * [BitBucket pull request 2747](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2747)
    * [Issue 2307](https://github.com/osrf/gazebo/issues/2307)

1. Fix inserted mesh scale during log playback
    * [BitBucket pull request #2723](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2723)

1. rendering/UNIT_Grid_TEST: Fix test failure due to EXPECT_EQ on floats
    * [BitBucket pull request 2802](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2802)

1. Diagnostics: enable test and don't create so many empty folders
    * [BitBucket pull request 2798](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2798)

1. RenderEngine::SetupResources(): Fix resource locations being added multiple times
    * [BitBucket pull request 2801](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2801)

1. Fix gui and rendering tests for gazebo8 + ogre1.9 on OSX
    * [BitBucket pull request 2793](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2793)

1. Support off-diagonal inertia terms in bullet
    * [BitBucket pull request 2757](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2757)

1. Parallelize ODE physics with threaded islands parameter
    * [BitBucket pull request 2775](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2775)

1. Disable broken dart5 tests on gazebo8 branch
    * [BitBucket pull request 2771](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2771)

1. Fix gazebo7 compile error with boost 1.58 for oculus support
    * [BitBucket pull request 2788](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2788)
    * [Issue 2356](https://github.com/osrf/gazebo/issues/2356)

1. Logical Camera sees nested models
    * [BitBucket pull request 2776](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2776)
    * [Issue 2342](https://github.com/osrf/gazebo/issues/2342)

1. Logical camera uses <topic>
    * [BitBucket pull request 2777](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2777)

1. Removed std::cout logging output on deferred shading
    * [BitBucket pull request 2779](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2779)

1. Update depth camera shaders version
    * [BitBucket pull request 2767](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2767)
    * [Issue 2323](https://github.com/osrf/gazebo/issues/2323)

1. Replaced Ogre::SharedPtr constructor calls with 0 arguments
    * [BitBucket pull request 2772](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2772)

1. Send message to subscribers only once per connection
    * [BitBucket pull request 2763](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2763)

1. Fix disabling mesh cast shadows
    * [BitBucket pull request 2710](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2710)

1. Fix gzclient shutdown segmentation fault with ogre 1.10
    * [BitBucket pull request 2761](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2761)
    * [Issue 2324](https://github.com/osrf/gazebo/issues/2324)

1. Fix right-click segfault
    * [BitBucket pull request 2809](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2809)
    * [Issue 2377](https://github.com/osrf/gazebo/issues/2377)

1. Joint control menu highlight active
    * [BitBucket pull request 2747](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2747)
    * [Issue 2307](https://github.com/osrf/gazebo/issues/2307)

1. Don't use lib prefix for ogre plugins as of ogre1.9
    * [BitBucket pull request 2803](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2803)

1. RenderEngine::SetupResources(): Fix resource locations being added multiple times
    * [BitBucket pull request 2801](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2801)

1. Added and improved communications between the JointControlWidget and JointController
    * [BitBucket pull request 2730](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2730)
    * [Issue 295](https://github.com/osrf/gazebo/issues/295)

1. Add function to retrieve scoped sensors name in multi-nested model
    * [BitBucket pull request 2674](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2674)

1. Backport wide angle camera VM FSAA fix
    * [BitBucket pull request 2711](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2711)

1. Add log record filter options
    * [BitBucket pull request 2715](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2715)
    * [BitBucket pull request 2725](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2725)

1. Fix inertia parameters in friction_spheres.world
    * [BitBucket pull request 2724](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2724)

1. ODE slip parameter example world and test
    * [BitBucket pull request 2717](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2717)

1. Aligned collision and visual geometries for friction_dir_test.world
    * [BitBucket pull request 2726](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2726)

1. Do not display COM or inertia visualizations for static models
    * [BitBucket pull request 2727](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2727)
    * [Issue 2286](https://github.com/osrf/gazebo/issues/2286)

1. Fix index error in VClouds/DataManager.cpp
    * [BitBucket pull request 2722](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2722)

1. Fix orbiting view around heightmap
    * [BitBucket pull request 2688](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2688)
    * [Issue 2049](https://github.com/osrf/gazebo/issues/2049)

1. Fix configure script on windows
    * [BitBucket pull request 2735](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2735)

1. Add option in gui.ini to disable the use of spacenav
    * [BitBucket pull request 2754](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2754)

1. Test which demonstrates Simbody exception when manipulating object twice while paused
    * [BitBucket pull request 2737](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2737)

## Gazebo 8.1.1 (2017-06-05)

1. Add the option --gui-client-plugin to load GUI plugins. Leave -g to load System Plugins.
    * [BitBucket pull request 2716](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2716)
    * [Issue 2279](https://github.com/osrf/gazebo/issues/2279)

1. Remove duplicate material block in ShadowCaster.material
    * [BitBucket pull request 2721](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2721)

1. Fix race condition during Detach of HarnessPlugin
    * [BitBucket pull request 2696](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2696)

1. Added support for pincushion distortion model; fixed bug where
   cameras with different distortion models would have the same distortion.
    * [BitBucket pull request 2678](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2678)

1. Add actors in World as models so they get returned with World::Models()
    * [BitBucket pull request 2706](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2706)
    * [Issue 2271](https://github.com/osrf/gazebo/issues/2271)

1. Refactor tests to use models from world file instead of dynamically spawning models
    * [BitBucket pull request 2689](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2689)

## Gazebo 8.1.0 (2017-05-04)

1. Fixed precompiled headers to work in more use-cases.
    * [BitBucket pull request 2662](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2662)

1. Subdivide large heightmaps to fix LOD and support global texture mapping
    * [BitBucket pull request 2655](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2655)

1. Added <collide_bitmask> support to bullet
    * [BitBucket pull request 2649](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2649)

1. Fix linking when using HDF5_INSTRUMENT for logging ODE data
    * [BitBucket pull request 2669](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2669)
    * [Issue 1841](https://github.com/osrf/gazebo/issues/1841)

1. Force / torque sensor visualization using WrenchVisual
    * [BitBucket pull request 2653](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2653)

1. Cache heightmap tile data
    * [BitBucket pull request 2645](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2645)

1. Add plugin for attaching lights to links in a model
    * [BitBucket pull request 2647](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2647)

1. Support Heightmap LOD
    * [BitBucket pull request 2636](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2636)

1. Support setting shadow texture size
    * [BitBucket pull request 2644](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2644)

1. Fix deprecated sdf warnings produced by PluginToSDF
    * [BitBucket pull request 2646](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2646)
    * [Issue 2202](https://github.com/osrf/gazebo/issues/2202)

1. Added TouchPlugin, which checks if a model has been in contact with another
   model exclusively for a certain time.
    * [BitBucket pull request 2651](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2651)

1. Fixes -inf laser reading being displayed as +inf
    * [BitBucket pull request 2641](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2641)

1. Proper exception handling for animated box example
    * [BitBucket pull request 2618](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2618)

1. Fix examples compilation (#2177)
    * [BitBucket pull request 2634](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2634)
    * [Issue 2177](https://github.com/osrf/gazebo/issues/2177)

1. Fix loading gui plugins and OSX framerate issue
    * [BitBucket pull request 2631](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2631)
    * [Issue 1311](https://github.com/osrf/gazebo/issues/1311)
    * [Issue 2133](https://github.com/osrf/gazebo/issues/2133)

1. Fix ign-math3 deprecation warnings
    * [BitBucket pull request 2612](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2612)
    * [BitBucket pull request 2626](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2626)
    * [BitBucket pull request 2648](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2648)

1. Re-order some gui tests to fix osx failures
    * [BitBucket pull request 2650](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2650)
    * [Issue 2197](https://github.com/osrf/gazebo/issues/2197)


## Gazebo 8.0.0 (2017-01-25)

1. Depend on ignition math3
    * [BitBucket pull request #2588](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2588)

1. Use ignition math with ServerFixture
    * [BitBucket pull request #2552](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2552)

1. Changed the type of `FrictionPyramid::direction1` from `gazebo::math::Vector3` to `ignition::math::Vector3d`.
    * [BitBucket pull request #2548](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2548)

1. Added igntition::transport interfaces to header files
    * [BitBucket pull request #2559](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2559)

1. Added ignition transport dependency, and output camera sensor images on
   an ignition transport topic.
    * [BitBucket pull request #2544](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2544)

1. Fix restoring submesh material transparency
    * [BitBucket pull request #2536](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2536)

1. Updated `gz_log` tool to use `ignition::math`.
    * [BitBucket pull request #2532](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2532)

1. Updated the following rendering classes to use `ignition::math`:
   `FPSViewController`, `JointVisual`, `OculusCamera`, `OrbitViewController`,
   `OrthoViewController`, `Projector`, `UserCamera`, `ViewController`.
    * [BitBucket pull request #2551](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2551)

1. Update examples to use ign-math.
    * [BitBucket pull request #2539](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2539)

1. Update plugins to use ign-math.
    * [BitBucket pull request #2531](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2531)
    * [BitBucket pull request #2534](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2534)
    * [BitBucket pull request #2538](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2538)

1. Use ignition math with `rendering/Distortion` and update function names.
    * [BitBucket pull request #2529](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2529)

1. Updated COMVisual class to use `ignition::math`.
    * [BitBucket pull request #2528](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2528)

1. Deprecate angle API from physics::Joint, in favor of using doubles
    * [BitBucket pull request #2568](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2568)
    * [Issue #553](https://github.com/osrf/gazebo/issues/553)
    * [Issue #1108](https://github.com/osrf/gazebo/issues/1108)

1. PIMPL-ize `gazebo/physics/Gripper` and use ignition-math.
    * [BitBucket pull request #2523](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2523)

1. Added VisualMarkers to the rendering engine. Visual markers support
   programmatic rendering of various shapes in a scene.
    * [BitBucket pull request 2541](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2541)

1. Support version 5 of the DART Physics Engine.
    * [BitBucket pull request #2459](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2459)

1. UserCamera overrides `Camera::Render` to reduce CPU usage.
    * [BitBucket pull request 2480](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2480)

1. Static links no longer subscribe to wrench topics.
    * [BitBucket pull request #2452]((https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2452)

1. Add Gazebo math helper functions to convert to and from Ignition Math
   objects.
    * [BitBucket pull request #2461](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2461)

1. Add video recording of user camera. This change added an optional
   dependency on libavdevice>=56.4.100 for linux systems. When installed,
   libavdevice will allow a user to stream a simulated camera to a video4linux2
   loopback device.
    * [BitBucket pull request #2443](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2443)

1. Removed deprecations
    * [BitBucket pull request #2427]((https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2427)

1. Include basic support for GNU Precompiled Headers to reduce compile time
    * [BitBucket pull request #2268](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2268)

1. Plotting utility
    * [BitBucket pull request #2348](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2348)
    * [BitBucket pull request #2325](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2325)
    * [BitBucket pull request #2382](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2382)
    * [BitBucket pull request #2448](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2448)

1. Renamed `gazebo/gui/SaveDialog` to `gazebo/gui/SaveEntityDialog`. A new
   `SaveDialog` class will be added in a future pull request. The migration
   guide will be updated with that pull request.
    * [BitBucket pull request #2384](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2384)

1. Add FiducialCameraPlugin for Camera Sensors
    * [BitBucket pull request #2350](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2350)

1. Fix Road2d vertices and shadows
    * [BitBucket pull request #2362](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2362)

1. Rearrange GLWidget::OnMouseMove so that the more common use cases it
   fewer if statements. Use std::thread in place of boost in OculusWindow.
   Pragma statements to prevent warnings. Prevent variable hiding in
   WallSegmentItem.
    * [BitBucket pull request #2376](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2376)

1. Use single pixel selection buffer for mouse picking
    * [BitBucket pull request #2335](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2335)

1. Refactor Visual classes
    * [BitBucket pull request #2331](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2331)

1. Windows plugins (with .dll extension) now accepted
    * [BitBucket pull request #2311](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2311)
    * Writing libMyPlugin.so in the sdf file will look for MyPlugin.dll on windows.

1. Add Introspection Manager and Client util
    * [BitBucket pull request #2304](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2304)

1. Refactor Event classes and improve memory management.
    * [BitBucket pull request #2277](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2277)
    * [BitBucket pull request #2317](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2317)
    * [BitBucket pull request #2329](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2329)
    * [gazebo_design Pull request #33](https://github.com/osrf/gazebo_design/pull-requests/33)

1. Remove EntityMakerPrivate and move its members to derived classes
    * [BitBucket pull request #2310](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2310)

1. Conversion between ign-msgs and sdf, for plugin
    * [BitBucket pull request #2403](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2403)

1. Change NULL to nullptr.
    * [BitBucket pull request #2294](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2294)
    * [BitBucket pull request #2297](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2297)
    * [BitBucket pull request #2298](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2298)
    * [BitBucket pull request #2302](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2302)
    * [BitBucket pull request #2295](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2295)
    * [BitBucket pull request #2300](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2300)

1. Fix memory and other issues found from running Coverity.
    * A contribution from Olivier Crave
    * [BitBucket pull request #2241](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2241)
    * [BitBucket pull request #2242](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2242)
    * [BitBucket pull request #2243](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2243)
    * [BitBucket pull request #2244](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2244)
    * [BitBucket pull request #2245](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2245)

1. Deprecate gazebo::math
    * [BitBucket pull request #2594](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2594)
    * [BitBucket pull request #2513](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2513)
    * [BitBucket pull request #2586](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2586)
    * [BitBucket pull request #2326](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2326)
    * [BitBucket pull request #2579](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2579)
    * [BitBucket pull request #2574](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2574)
    * [BitBucket pull request #2426](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2426)
    * [BitBucket pull request #2567](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2567)
    * [BitBucket pull request #2355](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2355)
    * [BitBucket pull request #2407](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2407)
    * [BitBucket pull request #2564](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2564)
    * [BitBucket pull request #2591](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2591)
    * [BitBucket pull request #2425](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2425)
    * [BitBucket pull request #2570](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2570)
    * [BitBucket pull request #2436](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2436)
    * [BitBucket pull request #2556](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2556)
    * [BitBucket pull request #2472](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2472)
    * [BitBucket pull request #2505](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2505)
    * [BitBucket pull request #2583](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2583)
    * [BitBucket pull request #2514](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2514)
    * [BitBucket pull request #2522](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2522)
    * [BitBucket pull request #2565](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2565)
    * [BitBucket pull request #2525](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2525)
    * [BitBucket pull request #2533](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2533)
    * [BitBucket pull request #2543](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2543)
    * [BitBucket pull request #2549](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2549)
    * [BitBucket pull request #2554](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2554)
    * [BitBucket pull request #2560](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2560)
    * [BitBucket pull request #2585](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2585)
    * [BitBucket pull request #2575](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2575)
    * [BitBucket pull request #2563](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2563)
    * [BitBucket pull request #2573](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2573)
    * [BitBucket pull request #2577](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2577)
    * [BitBucket pull request #2581](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2581)
    * [BitBucket pull request #2566](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2566)
    * [BitBucket pull request #2578](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2578)

1. Add Wind support
    * [BitBucket pull request #1985](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1985)
    * A contribution from Olivier Crave

1. Add const accessors to uri path and query
    * [BitBucket pull request #2400](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2400)

1. Server generates unique model names in case of overlap, and added allow_renaming field to factory message.
    * [BitBucket pull request 2301](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2301)
    * [Issue 510](https://github.com/osrf/gazebo/issues/510)

1. Adds an output option to gz log that allows the tool to filter a log file and write to a new log file.
    * [BitBucket pull request #2149](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2149)

1. Add common::URI class
    * [BitBucket pull request #2275](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2275)

1. Update Actor animations by faciliting skeleton visualization, control via a plugin. Also resolves issue #1785.
    * [BitBucket pull request #2219](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2219)

1. Generalize actors to work even if not all elements are specified
    * [BitBucket pull request #2360](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2360)

1. PIMPLize rendering/Grid
    * [BitBucket pull request 2330](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2330)

1. Use only Gazebo's internal version of tinyxml2. The version of tinyxml2 distributed with Ubuntu fails when parsing large log files.
    * [BitBucket pull request #2146](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2146)

1. Moved gazebo ODE includes to have correct include path
    * [BitBucket pull request #2186](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2186)

1. Atmosphere model
    * [BitBucket pull request #1989](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1989)

1. Added static camera when following a model.
    * [BitBucket pull request #1980](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1980)
    * A contribution from Oliver Crave

1. Get plugin info with Ignition transport service
    * [BitBucket pull request #2420](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2420)

1. Support conversions between SDF and protobuf for more sensors.
    * [BitBucket pull request #2118](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2118)

1. Fix ODE Ray-Cylinder collision, and added ability to instantiate stand alone MultiRayShapes.
    * [BitBucket pull request #2122](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2122)

1. Update depth camera sensor to publish depth data over a topic.
    * [BitBucket pull request #2112](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2112)

1. Add color picker to config widget and fix visual and collision duplication.
    * [BitBucket pull request #2381](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2381)

1. Model editor updates

    1. Undo / redo inserting and deleting links
    * [BitBucket pull request #2151](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2151)

    1. Undo / redo inserting and deleting nested models
    * [BitBucket pull request #2229](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2229)

    1. Undo insert / delete joints
    * [BitBucket pull request #2266](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2266)

    1. Undo insert / delete model plugins
    * [BitBucket pull request #2334](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2334)

    1. Undo translate, rotate, snap and align links and nested models
    * [BitBucket pull request #2314](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2314)

    1. Undo scale links
    * [BitBucket pull request #2368](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2368)

1. Google Summer of Code Graphical interface for inserting plugins during simulation.

    1. Display attached model plugins in the world tab / Add subheaders for model links, joints and plugins
    * [BitBucket pull request #2323](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2323)
    * [Issue #1698](https://github.com/osrf/gazebo/issues/1698)

## Gazebo 7

## Gazebo 7.X.X (2019-XX-XX)

## Gazebo 7.16.1 (2019-07-28)

1. Remove X11 call from Gazebo 7
    * [BitBucket pull request 3195](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3195)

1. Update BitBucket links
    * [Pull request 2715](https://github.com/osrf/gazebo/pull/2715)

1. [backport Gazebo7] Fixed crash when collision size is zero
    * [Pull request 2769](https://github.com/osrf/gazebo/pull/2769)

## Gazebo 7.16.0 (2019-09-04)

1. VariableGearboxPlugin: use splines to support arbitrary smooth input-output gearbox profiles
    * [BitBucket pull request 3073](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3073)

1. Backport Camera PreRender and PostRender events
    * [BitBucket pull request 3119](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3119)

## Gazebo 7.15.0 (2018-03-26)

1. Don't search for boost signals component (support boost 1.69)
    * [BitBucket pull request 3089](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3089)
    * [Issue 2577](https://github.com/osrf/gazebo/issues/2577)

1. Refactor ODE gearbox joint implementation to match hinge joint
    * [BitBucket pull request 3048](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3048)

1. Use new sha1.hpp header location for recent boost (support boost 1.68)
    * [BitBucket pull request 3029](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3029)

1. Add MisalignmentPlugin which reports alignment between two poses
    * [BitBucket pull request 2896](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2896)

1. More documentation to Model::CreateJoint()
    * [BitBucket pull request 3002](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3002)

1. Improve ODE slip parameter behavior with multiple contact points
    * [BitBucket pull request 2965](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2965)

1. Fix for BulletFixedJoint when used with inertial matrices with non-zero values on their off-diagonal
    * [BitBucket pull request 3010](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/3010)

1. Adding WheelSlipPlugin: for adding wheel slip using ODE's contact parameters
    * [BitBucket pull request 2950](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2950)
    * [BitBucket pull request 2976](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2976)
    * [BitBucket pull request 2997](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2997)

1. Adding JointController::SetForce API and extra test for WheelSlipPlugin
    * [BitBucket pull request 2976](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2976)


## Gazebo 7.14.0 (2018-07-27)

1. Fix manipulating links in the model editor
    * [BitBucket pull request 2999](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2999)
    * [Issue 2487](https://github.com/osrf/gazebo/issues/2487)

1. LOD skirt length
    * [BitBucket pull request 2968](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2968)

1. Patch for visual message process
    * [BitBucket pull request 2983](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2983)

1. Fix build on homebrew with protobuf 3.6
    * [BitBucket pull request 2984](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2984)

1. Fix GpuRaySensor vertical rays
    * [BitBucket pull request 2955](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2955)


## Gazebo 7.13.1 (2018-06-15)

1. Fix check terrain layer count in height map
    * [BitBucket pull request 2978](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2978)


## Gazebo 7.13.0 (2018-06-08)

1. Update model database URI
    * [BitBucket pull request 2969](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2969)

1. Fix getting joint limits for BulletHingeJoint
    * [BitBucket pull request 2959](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2959)

1. Save model materials and meshes when logging
    * [BitBucket pull request 2811](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2811)

1. Fix build on hombrew with boost 1.67
    * [BitBucket pull request 2954](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2954)

1. Add Screen Space Ambient Occlusion visual plugin
    * [BitBucket pull request 2916](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2916)
    * [BitBucket pull request 2947](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2947)

1. Fix ray intersection check in Scene::FirstContact
    * [BitBucket pull request 2945](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2945)

1. Fix camera view control inside bounding box of large meshes
    * [BitBucket pull request 2932](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2932)

1. Fix compilation with boost 1.67
    * [BitBucket pull request 2937](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2937)

1. Fix compilation with ffmpeg4
    * [BitBucket pull request 2942](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2942)

1. Fix Joint::SetPosition for HingeJoint
    * [BitBucket pull request 2892](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2892)
    * [Issue 2430](https://github.com/osrf/gazebo/issues/2430)

1. Use QVERIFY() around qFuzzyCompare statements
    * [BitBucket pull request 2936](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2936)


## Gazebo 7.12.0 (2018-04-11)

1. Fix mouse movement ogre assertion error
    * [BitBucket pull request 2928](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2928)

1. Fix normal maps on ubuntu with OGRE 1.9 and disable on OSX
    * [BitBucket pull request 2917](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2917)

1. Support lens flare occlusion
    * [BitBucket pull request 2915](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2915)

1. Fix log recording, only call sdf::initFile once
    * [BitBucket pull request 2889](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2889)
    * [Issue 2425](https://github.com/osrf/gazebo/issues/2425)

1. Fix OBJLoader when mesh has invalid material
    * [BitBucket pull request 2888](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2888)

1. Fix clang warnings in LaserView and EnumIface
    * [BitBucket pull request 2891](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2891)

1. Add support for moving geometry to ContainPlugin
    * [BitBucket pull request 2886](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2886)

1. Support python3 with check_test_ran.py
    * [BitBucket pull request 2902](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2902)

1. Fix undefined behavior in ODESliderJoint
    * [BitBucket pull request 2905](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2905)

1. Fix loading collada mesh that contains multiple texcoord sets with same offset
    * [BitBucket pull request 2899](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2899)

1. Fix race conditions during client startup, and introduce Node::TryInit()
    * [BitBucket pull request 2897](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2897)

1. Diagnostics: record timing statistics instead of all timestamps
    * [BitBucket pull request 2821](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2821)

1. Backport pull request #2890 to gazebo7 (fix logging)
    * [BitBucket pull request 2933](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2933)
    * [Issue 2441](https://github.com/osrf/gazebo/issues/2441)

1. Add trigger_light example for ContainPlugin tutorial
    * [BitBucket pull request 2918](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2918)
    * [BitBucket pull request 2929](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2929)

1. Backport pull request #2884 to gazebo7 (disable model plugin during playback)
    * [BitBucket pull request 2927](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2927)
    * [Issue 2427](https://github.com/osrf/gazebo/issues/2427)

## Gazebo 7.11.0 (2018-02-12)

1. Fix gazebo7 + ogre 1.8 build error
    * [BitBucket pull request 2878](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2878)

1. Process insertions and deletions on gz log echo
    * [BitBucket pull request 2608](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2608)
    * [Issue 2136](https://github.com/osrf/gazebo/issues/2136)

1. Add Static Map Plugin for creating textured map model
    * [BitBucket pull request 2834](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2834)

## Gazebo 7.10.0 (2018-02-07)

1. Add support for 16 bit Grayscale and RGB camera image types.
    * [BitBucket pull request 2852](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2852)

1. Added a plugin to detect if an entity is inside a given volume in space
    * [BitBucket pull request 2780](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2780)

1. Add Visual::SetMaterialShaderParam function for setting shader parameters.
    * [BitBucket pull request 2863](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2863)

1. Adding accessors for velocity in ENU frame for gps sensor
    * [BitBucket pull request 2854](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2854)

1. Fix DEM min elevation
    * [BitBucket pull request 2868](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2868)

1. Update Color Clamp function
    * [BitBucket pull request 2859](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2859)

1. Move Connection header buffer from heap to stack to avoid race condition.
    * [BitBucket pull request 2844](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2844)

1. Initialize laser retro value
    * [BitBucket pull request 2841](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2841)

1. Shadow improvements
    * [BitBucket pull request 2805](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2805)

1. Add light as child of link
    * [BitBucket pull request 2807](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2807)
    * [BitBucket pull request 2872](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2872)
    * [Issue 900](https://github.com/osrf/gazebo/issues/900)

1. Add camera lens flare effect
    * [BitBucket pull request 2806](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2806)
    * [BitBucket pull request 2829](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2829)

1. Fix inserting models with invalid submesh
    * [BitBucket pull request 2828](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2828)

1. Call DisconnectNewImageFrame in the CameraPlugin destructor
    * [BitBucket pull request 2815](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2815)

1. Fix gazebo7 homebrew build (tinyxml2 6.0.0)
    * [BitBucket pull request 2824](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2824)


## Gazebo 7.9.0 (2017-11-22)

1. Diagnostics: enable test and don't create so many empty folders
    * [BitBucket pull request 2798](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2798)

1. Parallelize ODE physics with threaded islands parameter
    * [BitBucket pull request 2775](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2775)

1. Logical camera uses <topic>
    * [BitBucket pull request 2777](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2777)

1. Support off-diagonal inertia terms in bullet
    * [BitBucket pull request 2757](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2757)

1. Add option in gui.ini to disable the use of spacenav
    * [BitBucket pull request 2754](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2754)

1. Fix disabling mesh cast shadows
    * [BitBucket pull request 2710](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2710)

1. Do not display COM or inertia visualizations for static models
    * [BitBucket pull request 2727](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2727)
    * [Issue 2286](https://github.com/osrf/gazebo/issues/2286)

1. Fix Collision::GetWorldPose for non-canonical links (and friction directions)
    * [BitBucket pull request 2702](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2702)
    * [Issue 2068](https://github.com/osrf/gazebo/issues/2068)

1. Fix orbiting view around heightmap
    * [BitBucket pull request 2688](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2688)
    * [Issue 2049](https://github.com/osrf/gazebo/issues/2049)

1. Logical Camera sees nested models
    * [BitBucket pull request 2776](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2776)
    * [Issue 2342](https://github.com/osrf/gazebo/issues/2342)

1. Aligned collision and visual geometries for friction_dir_test.world
    * [BitBucket pull request 2726](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2726)

1. Test which demonstrates Simbody exception when manipulating object twice while paused
    * [BitBucket pull request 2737](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2737)

1. Send message to subscribers only once per connection
    * [BitBucket pull request 2763](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2763)

1. Update depth camera shaders version
    * [BitBucket pull request 2767](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2767)
    * [Issue 2323](https://github.com/osrf/gazebo/issues/2323)

1. Fix gazebo7 compile error with boost 1.58 for oculus support
    * [BitBucket pull request 2788](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2788)
    * [Issue 2356](https://github.com/osrf/gazebo/issues/2356)

1. Fix gui and rendering tests for gazebo7 + ogre1.9 on OSX
    * [BitBucket pull request 2793](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2793)

1. Fix right-click segfault
    * [BitBucket pull request 2809](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2809)
    * [Issue 2377](https://github.com/osrf/gazebo/issues/2377)

## Gazebo 7.8.1 (2017-06-08)

1. ODE slip parameter example world and test
    * [BitBucket pull request 2717](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2717)

1. Fix inserted mesh scale during log playback
    * [BitBucket pull request #2723](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2723)

## Gazebo 7.8.0 (2017-06-02)

1. Add log record filter options
    * [BitBucket pull request 2715](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2715)

1. Backport wide angle camera VM FSAA fix
    * [BitBucket pull request 2711](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2711)

1. Add function to retrieve scoped sensors name in multi-nested model
    * [BitBucket pull request 2674](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2674)

## Gazebo 7.7.0 (2017-05-04)

1. Fix race condition during Detach of HarnessPlugin
    * [BitBucket pull request 2696](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2696)

1. Added support for pincushion distortion model; fixed bug where
   cameras with different distortion models would have the same distortion.
    * [BitBucket pull request 2678](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2678)

1. Added <collide_bitmask> support to bullet
    * [BitBucket pull request 2649](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2649)

1. Fix linking when using HDF5_INSTRUMENT for logging ODE data
    * [BitBucket pull request 2669](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2669)
    * [Issue 1841](https://github.com/osrf/gazebo/issues/1841)

1. Subdivide large heightmaps to fix LOD and support global texture mapping
    * [BitBucket pull request 2655](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2655)

## Gazebo 7.6.0 (2017-03-20)

1. Force / torque sensor visualization using WrenchVisual
    * [BitBucket pull request 2653](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2653)

1. Cache heightmap tile data
    * [BitBucket pull request 2645](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2645)

1. Add plugin for attaching lights to links in a model
    * [BitBucket pull request 2647](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2647)
    * [Issue 900](https://github.com/osrf/gazebo/issues/900)

1. Support Heightmap LOD
    * [BitBucket pull request 2636](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2636)

1. Support setting shadow texture size
    * [BitBucket pull request 2644](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2644)

1. Fix deprecated sdf warnings produced by PluginToSDF
    * [BitBucket pull request 2646](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2646)

1. Added TouchPlugin, which checks if a model has been in contact with another
   model exclusively for a certain time.
    * [BitBucket pull request 2651](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2651)

1. Fixes -inf laser reading being displayed as +inf
    * [BitBucket pull request 2641](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2641)

1. Fix memory leaks in tests
    * [BitBucket pull request 2639](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2639)

1. Remove end year from copyright
    * [BitBucket pull request 2614](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2614)

## Gazebo 7.5.0 (2017-01-11)

1. Remove qt4 webkit in gazebo7 (used for HotkeyDialog).
    * [BitBucket pull request 2584](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2584)

1. Support configuring heightmap sampling level
    * [BitBucket pull request 2519](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2519)

1. Fix `model.config` dependency support, and add ability to reference
   textures using a URI.
    * [BitBucket pull request 2517](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2517)

1. Fix DEM heightmap size, collision, scale
    * [BitBucket pull request 2477](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2477)

1. Create ode_quiet parameter to silence solver messages
    * [BitBucket pull request 2512](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2512)

1. Update QT render loop to throttle based on UserCamera::RenderRate.
    * [BitBucket pull request 2476](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2476)
    * [Issue 1560](https://github.com/osrf/gazebo/issues/1560)

1. Generate visualization on demand, instead of on load. This helps to
   reduce load time.
    * [BitBucket pull request 2457](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2457)

1. Added a plugin to teleoperate joints in a model with the keyboard.
    * [BitBucket pull request 2490](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2490)

1. Add GUI items to change the user camera clip distance
    * [BitBucket pull request 2470](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2470)
    * [Issue 2064](https://github.com/osrf/gazebo/issues/2064)

1. Support custom material scripts for heightmaps
    * [BitBucket pull request 2473](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2473)

1. Sim events plugin accepts custom topics
    * [BitBucket pull request 2535](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2535)

1. Model Editor: Show / hide collisions
    * [BitBucket pull request 2503](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2503)

1. Model Editor: Show / hide visuals
    * [BitBucket pull request 2516](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2516)

1. Model Editor: Show / hide link frames
    * [BitBucket pull request 2521](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2521)

## Gazebo 7.4.0 (2016-10-11)

1. Add test for HarnessPlugin, reduce likelihood of race condition
    * [BitBucket pull request 2431](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2431)
    * [Issue 2034](https://github.com/osrf/gazebo/issues/2034)

1. Add `syntax = proto2` in proto files to fix some protobuf3 warnings
    * [BitBucket pull request 2456](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2456)

1. Add support for loading wavefront obj mesh files
    * [BitBucket pull request 2454](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2454)

1. Added filesystem operations to the common library. Additions include
   `cwd`, `exists`, `isDirectory`, `isFile`, `copyFile`, and `moveFile`.
    * [BitBucket pull request 2417](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2417)

1. Fix loading collada files with multiple texture coordinates.
    * [BitBucket pull request 2413](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2413)

1. Added visualization of minimum range to laservisual.
    * [BitBucket pull request 2412](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2412)
    * [Issue 2018](https://github.com/osrf/gazebo/issues/2018)

1. Use precision 2 for FPS display in TimePanel
    * [BitBucket pull request 2405](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2405)

1. Switch ImuSensor::worldToReference transform from Pose to Quaternion
    * [BitBucket pull request 2410](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2410)
    * [Issue 1959](https://github.com/osrf/gazebo/issues/1959)

1. Include Boost_LIBRARIES  in the linking of gazebo_physics
    * [BitBucket pull request 2402](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2402)

1. Backported KeyboardGUIPlugin and msgs::Any
    * [BitBucket pull request 2416](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2416)

1. Use XML_SUCCESS enum instead of XML_NO_ERROR, which has been deleted in tinyxml2 4.0
    * [BitBucket pull request 2397](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2397)

1. Ignore ffmpeg deprecation warnings to clean up CI since they are noted in #2002
    * [BitBucket pull request 2388](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2388)

1. Added a visual blinking plugin
    * [BitBucket pull request 2394](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2394)

1. Fix InertiaVisual for non-diagonal inertia matrices
    * [BitBucket pull request 2354](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2354)

## Gazebo 7.3.1 (2016-07-13)

1. Fix homebrew test failure of UNIT_ApplyWrenchDialog_TEST
    * [BitBucket pull request 2393](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2393)

1. Fix MainWindow crash when window is minimized and maximized
    * [BitBucket pull request 2392](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2392)
    * [Issue 2003](https://github.com/osrf/gazebo/issues/2003)

## Gazebo 7.3.0 (2016-07-12)

1. Fix selecting ApplyWrenchVisual's force torque visuals
    * [BitBucket pull request 2377](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2377)
    * [Issue 1999](https://github.com/osrf/gazebo/issues/1999)

1. Use ignition math in gazebo::msgs
    * [BitBucket pull request 2389](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2389)

1. Parse command-line options for GUI plugins in Server to fix parsing of
   positional argument for world file.
   This fixes command-line parsing for `gazebo -g gui_plugin.so`.
    * [BitBucket pull request 2387](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2387)

1. Added a harness plugin that supports lowering a model at a controlled rate
    * [BitBucket pull request 2346](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2346)

1. Fix ogre log test on xenial+nvidia
    * [BitBucket pull request 2374](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2374)

1. Redirect QT messages to Gazebo's console message handling system.
    * [BitBucket pull request 2375](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2375)

1. Fix buoyancy plugin when multiple link tags are used within the plugin
    * [BitBucket pull request 2369](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2369)

1. Remove contact filters with names that contain `::`
    * [BitBucket pull request 2363](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2363)
    * [Issue 1805](https://github.com/osrf/gazebo/issues/1805)

1. Fix Model Manipulator switching between local and global frames
    * [BitBucket pull request 2361](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2361)

1. Remove duplicate code from cmake config file caused by bad merge
    * [BitBucket pull request 2347](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2347)

1. Properly cleanup pointers when destroying a world with joints.
    * [BitBucket pull request 2309](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2309)

1. Fix right click view options after deleting and respawning a model.
    * [BitBucket pull request 2349](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2349)
    * [Issue 1985](https://github.com/osrf/gazebo/issues/1985)

1. Implement missing function: LogicalCamera::Topic()
    * [BitBucket pull request 2343](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2343)
    * [Issue 1980](https://github.com/osrf/gazebo/issues/1980)

## Gazebo 7.2.0 (2016-06-13)

1. Backport single pixel selection buffer for mouse picking
    * [BitBucket pull request 2338](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2338)

1. Prevent mouse pan and orbit from deselecting entities in model editor
    * [BitBucket pull request 2333](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2333)

1. Handle model manipulation tool RTS shortcuts in keyPress
    * [BitBucket pull request 2312](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2312)

1. Reset ODE joint force feedback after world reset
    * [BitBucket pull request 2255](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2255)

1. Update model editor snap to grid modifier key
    * [BitBucket pull request 2259](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2259)
    * [Issue #1583](https://github.com/osrf/gazebo/issues/1583)

1. PIMPLize gui/model/ModelEditorPalette
    * [BitBucket pull request 2279](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2279)

1. Properly cleanup pointers when destroying a blank world.
    * [BitBucket pull request 2220](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2220)

1. Properly cleanup pointers when destroying a world with models and lights.
    * [BitBucket pull request 2263](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2263)

1. Fix view control mouse focus in model editor
    * [BitBucket pull request 2315](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2315)
    * [Issue #1791](https://github.com/osrf/gazebo/issues/1791)

1. Server generates unique model names in case of overlap
    * [BitBucket pull request 2296](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2296)
    * [Issue 510](https://github.com/osrf/gazebo/issues/510)

1. Model Editor: Select and align nested models
    * [BitBucket pull request 2282](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2282)

## Gazebo 7.1.0 (2016-04-07)

1. fix: remove back projection
    * [BitBucket pull request 2201](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2201)
    * A contribution from Yuki Furuta

1. Fix oculus 2 camera field of view
    * [BitBucket pull request 2157](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2157)

1. Added BeforePhysicsUpdate world event
    * [BitBucket pull request 2128](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2128)
    * A contribution from Martin Pecka

1. Update `gz sdf -c` command line tool to use the new `sdf::convertFile` API.
    * [BitBucket pull request #2227](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2227)

1. Backport depth camera OSX fix
    * [BitBucket pull request 2233](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2233)

1. Feat load collision.sdf only once
    * [BitBucket pull request 2236](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2236)

1. Update gui/building/Item API
    * [BitBucket pull request 2228](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2228)

1. Semantic version class to compare model versions in the model database.
    * [BitBucket pull request 2207](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2207)

1. Backport issue 1834 fix to gazebo7
    * [BitBucket pull request 2222](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2222)

1. Backport ImagesView_TEST changes
    * [BitBucket pull request 2217](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2217)

1. Backport pull request #2189 (mutex in Transport::Conection)
    * [BitBucket pull request 2208](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2208)

1. Process insertions on World::SetState
    * [BitBucket pull request #2200](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2200)

1. Process deletions on World::SetState
    * [BitBucket pull request #2204](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2204)

1. Fix ray-cylinder collision
    * [BitBucket pull request 2124](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2124)

1. Fix editing physics parameters in gzclient, update test
    * [BitBucket pull request 2192](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2192)

1. Fix Audio Decoder test failure
    * [BitBucket pull request 2193](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2193)

1. Add layers to building levels
    * [BitBucket pull request 2180](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2180)

1. Allow dynamically adding links to a model.
    * [BitBucket pull request #2185](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2185)

1. Fix editing physics parameters in gzclient, update test
    * [BitBucket pull request #2192](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2192)
    * [Issue #1876](https://github.com/osrf/gazebo/issues/1876)

1. Model database selects the latest model version.
    * [BitBucket pull request #2207](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2207)

1. Only link relevant libraries to tests
    * [BitBucket pull request 2130](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2130)

1. PIMPLize gui/model/ModelCreator
    * [BitBucket pull request 2171](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2171)

1. backport warning and test fixes from pull request #2177
    * [BitBucket pull request 2179](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2179)

1. Prevent xml parser error from crashing LogPlay on osx -> gazebo7
    * [BitBucket pull request 2174](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2174)

1. PIMPLize gui/building/ScaleWidget
    * [BitBucket pull request 2164](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2164)

1. Fix using Shift key while scaling inside the model editor
    * [BitBucket pull request 2165](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2165)

1. Backport fix for ign-math explicit constructors -> gazebo7
    * [BitBucket pull request 2163](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2163)

1. Display physics engine type in the GUI
    * [BitBucket pull request #2155](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2155)
    * [Issue #1121](https://github.com/osrf/gazebo/issues/1121)
    * A contribution from Mohamd Ayman

1. Fix compilation against ffmpeg3 (libavcodec)
    * [BitBucket pull request #2154](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2154)

1. Append a missing </gazebo_log> tag to log files when played.
    * [BitBucket pull request #2143](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2143)

1. Add helper function QTestFixture::ProcessEventsAndDraw
    * [BitBucket pull request #2147](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2147)

1. Add qt resources to gazebo gui library
    * [BitBucket pull request 2134](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2134)

1. Undo scaling during simulation
    * [BitBucket pull request #2108](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2108)

1. Fix SensorManager::SensorContainer::RunLoop sensor update time assertion
    * [BitBucket pull request #2115](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2115)

1. Fix use of not initialized static attribute in Light class
    * [BitBucket pull request 2075](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2075)
    * A contribution from Silvio Traversaro

1. Install GuiTypes header
    * [BitBucket pull request 2106](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2106)

1. Removes one function call and replaces a manual swap with std::swap in ODE heightfield.
    * [BitBucket pull request #2114](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2114)

1. New world event: BeforePhysicsUpdate
    * [BitBucket pull request #2128](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2128)
    * [Issue #1851](https://github.com/osrf/gazebo/issues/1851)

1. Model editor: Fix setting relative pose after alignment during joint creation.
    * [Issue #1844](https://github.com/osrf/gazebo/issues/1844)
    * [BitBucket pull request #2150](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2150)

1. Model editor: Fix saving and spawning model with its original name
    * [BitBucket pull request #2183](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2183)

1. Model editor: Fix inserting custom links
    * [BitBucket pull request #2222](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2222)
    * [Issue #1834](https://github.com/osrf/gazebo/issues/1834)

1. Model editor: Reset visual / collision insertion / deletion
    * [BitBucket pull request #2254](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2254)
    * [Issue #1777](https://github.com/osrf/gazebo/issues/1777)
    * [Issue #1852](https://github.com/osrf/gazebo/issues/1852)

1. Building editor: Add layers to building levels
    * [BitBucket pull request #2180](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2180)
    * [Issue #1806](https://github.com/osrf/gazebo/issues/1806)

1. Building editor: Update gui/building/Item API
    * [BitBucket pull request #2228](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2228)

## Gazebo 7.0.0 (2016-01-25)

1. Add FollowerPlugin
    * [BitBucket pull request #2085](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2085)

1. Fix circular dependency so that physics does not call the sensors API.
    * [BitBucket pull request #2089](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2089)
    * [Issue #1516](https://github.com/osrf/gazebo/issues/1516)

1. Add Gravity and MagneticField API to World class to match sdformat change.
    * [SDFormat pull request 247](https://github.com/osrf/sdformat/pull-requests/247)
    * [Issue #1823](https://github.com/osrf/gazebo/issues/1823)
    * [BitBucket pull request #2090](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2090)

1. Use opaque pointers and deprecate functions in the rendering library
    * [BitBucket pull request #2069](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2069)
    * [BitBucket pull request #2064](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2064)
    * [BitBucket pull request #2066](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2066)
    * [BitBucket pull request #2069](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2069)
    * [BitBucket pull request #2074](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2074)
    * [BitBucket pull request #2076](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2076)
    * [BitBucket pull request #2070](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2070)
    * [BitBucket pull request #2071](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2071)
    * [BitBucket pull request #2084](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2084)
    * [BitBucket pull request #2073](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2073)

1. Use opaque pointers for the Master class.
    * [BitBucket pull request #2036](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2036)

1. Use opaque pointers in the gui library
    * [BitBucket pull request #2057](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2057)
    * [BitBucket pull request #2037](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2037)
    * [BitBucket pull request #2052](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2052)
    * [BitBucket pull request #2053](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2053)
    * [BitBucket pull request #2028](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2028)
    * [BitBucket pull request #2051](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2051)
    * [BitBucket pull request #2027](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2027)
    * [BitBucket pull request #2026](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2026)
    * [BitBucket pull request #2029](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2029)
    * [BitBucket pull request #2042](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2042)

1. Use more opaque pointers.
    * [BitBucket pull request #2022](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2022)
    * [BitBucket pull request #2025](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2025)
    * [BitBucket pull request #2043](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2043)
    * [BitBucket pull request #2044](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2044)
    * [BitBucket pull request #2065](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2065)
    * [BitBucket pull request #2067](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2067)
    * [BitBucket pull request #2079](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2079)

1. Fix visual transparency issues
    * [BitBucket pull request #2031](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2031)
    * [Issue #1726](https://github.com/osrf/gazebo/issues/1726)
    * [Issue #1790](https://github.com/osrf/gazebo/issues/1790)

1. Implemented private data pointer for the RTShaderSystem class. Minimized shader updates to once per render update.
    * [BitBucket pull request #2003](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2003)

1. Updating physics library to use ignition math.
    * [BitBucket pull request #2007](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2007)

1. Switching to ignition math for the rendering library.
    * [BitBucket pull request #1993](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1993)
    * [BitBucket pull request #1994](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1994)
    * [BitBucket pull request #1995](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1995)
    * [BitBucket pull request #1996](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1996)

1. Removed deprecations
    * [BitBucket pull request #1992]((https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1992)

1. Add ability to set the pose of a visual from a link.
    * [BitBucket pull request #1963](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1963)

1. Copy visual visibility flags on clone
    * [BitBucket pull request #2008](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2008)

1. Publish camera sensor image size when rendering is not enabled
    * [BitBucket pull request #1969](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1969)

1. Added Poissons Ratio and Elastic Modulus for ODE.
    * [BitBucket pull request #1974](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1974)

1. Update rest web plugin to publish response messages and display login user name in toolbar.
    * [BitBucket pull request #1956](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1956)

1. Improve overall speed of log playback. Added new functions to LogPlay.
   Use tinyxml2 for playback.
    * [BitBucket pull request #1931](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1931)

1. Improve SVG import. Added support for transforms in paths.
    * [BitBucket pull request #1981](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1981)

1. Enter time during log playback
    * [BitBucket pull request #2000](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2000)

1. Added Ignition Transport dependency.
    * [BitBucket pull request #1930](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1930)

1. Make latched subscribers receive the message only once
    * [Issue #1789](https://github.com/osrf/gazebo/issues/1789)
    * [BitBucket pull request #2019](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2019)

1. Implemented transport clear buffers
    * [BitBucket pull request #2017](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2017)

1. KeyEvent constructor should be in a source file. Removed a few visibility
flags from c functions. Windows did not like `CPPTYPE_*` in
`gazebo/gui/ConfigWidget.cc`, so I replaced it with `TYPE_*`.
    * [BitBucket pull request #1943](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1943)

1. Added wide angle camera sensor.
    * [BitBucket pull request #1866](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1866)

1. Change the `near` and `far` members of `gazebo/msgs/logical_camera_sensors.proto` to `near_clip` and `far_clip`
    + [BitBucket pull request #1942](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1942)

1. Resolve issue #1702
    * [Issue #1702](https://github.com/osrf/gazebo/issues/1702)
    * [BitBucket pull request #1905](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1905)
    * [BitBucket pull request #1913](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1913)
    * [BitBucket pull request #1914](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1914)

1. Update physics when the world is reset
    * [BitBucket pull request #1903](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1903)

1. Light and light state for the server side
    * [BitBucket pull request #1920](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1920)

1. Add scale to model state so scaling works on log/playback.
    * [BitBucket pull request #2020](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2020)

1. Added tests for WorldState
    * [BitBucket pull request #1968](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1968)

1. Rename Reset to Reset Time in time widget
    * [BitBucket pull request #1892](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1892)
    * [Issue #1730](https://github.com/osrf/gazebo/issues/1730)

1. Set QTestfFxture to verbose
    * [BitBucket pull request #1944](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1944)
    * [Issue #1756](https://github.com/osrf/gazebo/issues/1756)

1. Added torsional friction
    * [BitBucket pull request #1831](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1831)

1. Support loading and spawning nested models
    * [BitBucket pull request #1868](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1868)
    * [BitBucket pull request #1895](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1895)

1. Undo user motion commands during simulation, added physics::UserCmdManager and gui::UserCmdHistory.
    * [BitBucket pull request #1934](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1934)

1. Forward user command messages for undo.
    * [BitBucket pull request #2009](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2009)

1. Undo reset commands during simulation, forwarding commands
    * [BitBucket pull request #1986](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1986)

1. Undo apply force / torque during simulation
    * [BitBucket pull request #2030](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2030)

1. Add function to get the derived scale of a Visual
    * [BitBucket pull request #1881](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1881)

1. Added EnumIface, which supports iterators over enums.
    * [BitBucket pull request #1847](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1847)

1. Added RegionEventBoxPlugin - fires events when models enter / exit the region
    * [BitBucket pull request #1856](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1856)

1. Added tests for checking the playback control via messages.
    * [BitBucket pull request #1885](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1885)

1. Added LoadArgs() function to ServerFixture for being able to load a server
using the same arguments used in the command line.
    * [BitBucket pull request #1874](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1874)

1. Added battery class, plugins and test world.
    * [BitBucket pull request #1872](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1872)

1. Display gearbox and screw joint properties in property tree
    * [BitBucket pull request #1838](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1838)

1. Set window flags for dialogs and file dialogs
    * [BitBucket pull request #1816](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1816)

1. Fix minimum window height
    * [BitBucket pull request #1977](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1977)
    * [Issue #1706](https://github.com/osrf/gazebo/issues/1706)

1. Add option to reverse alignment direction
    * [BitBucket pull request #2040](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2040)
    * [Issue #1242](https://github.com/osrf/gazebo/issues/1242)

1. Fix unadvertising a publisher - only unadvertise topic if it is the last publisher.
    * [BitBucket pull request #2005](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2005)
    * [Issue #1782](https://github.com/osrf/gazebo/issues/1782)

1. Log playback GUI for multistep, rewind, forward and seek
    * [BitBucket pull request #1791](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1791)

1. Added Apply Force/Torque movable text
    * [BitBucket pull request #1789](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1789)

1. Added cascade parameter (apply to children) for Visual SetMaterial, SetAmbient, SetEmissive, SetSpecular, SetDiffuse, SetTransparency
    * [BitBucket pull request #1851](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1851)

1. Tweaks to Data Logger, such as multiline text edit for path
    * [BitBucket pull request #1800](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1800)

1. Added TopToolbar and hide / disable several widgets according to WindowMode
    * [BitBucket pull request #1869](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1869)

1. Added Visual::IsAncestorOf and Visual::IsDescendantOf
    * [BitBucket pull request #1850](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1850)

1. Added msgs::PluginFromSDF and tests
    * [BitBucket pull request #1858](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1858)

1. Added msgs::CollisionFromSDF msgs::SurfaceFromSDF and msgs::FrictionFromSDF
    * [BitBucket pull request #1900](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1900)

1. Added hotkeys chart dialog
    * [BitBucket pull request #1835](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1835)

1. Space bar to play / pause
    * [BitBucket pull request #2023](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2023)
    * [Issue #1798](https://github.com/osrf/gazebo/issues/1798)

1. Make it possible to create custom ConfigWidgets
    * [BitBucket pull request #1861](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1861)

1. AddItem / RemoveItem / Clear enum config widgets
    * [BitBucket pull request #1878](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1878)

1. Make all child ConfigWidgets emit signals.
    * [BitBucket pull request #1884](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1884)

1. Refactored makers
    * [BitBucket pull request #1828](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1828)

1. Added gui::Conversions to convert between Gazebo and Qt
    * [BitBucket pull request #2034](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2034)

1. Model editor updates
    1. Support adding model plugins in model editor
    * [BitBucket pull request #2060](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2060)

    1. Added support for copying and pasting top level nested models
    * [BitBucket pull request #2006](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2006)

    1. Make non-editable background models white in model editor
    * [BitBucket pull request #1950](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1950)

    1. Choose / swap parent and child links in joint inspector
    * [BitBucket pull request #1887](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1887)
    * [Issue #1500](https://github.com/osrf/gazebo/issues/1500)

    1. Presets combo box for Vector3 config widget
    * [BitBucket pull request #1954](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1954)

    1. Added support for more joint types (gearbox and fixed joints).
    * [BitBucket pull request #1794](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1794)

    1. Added support for selecting links and joints, opening context menu and inspectors in Schematic View.
    * [BitBucket pull request #1787](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1787)

    1. Color-coded edges in Schematic View to match joint color.
    * [BitBucket pull request #1781](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1781)

    1. Scale link mass and inertia when a link is scaled
    * [BitBucket pull request #1836](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1836)

    1. Add density widget to config widget and link inspector
    * [BitBucket pull request #1978](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1978)

    1. Added icons for child and parent link in joint inspector
    * [BitBucket pull request #1953](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1953)

    1. Load and save nested models
    * [BitBucket pull request #1894](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1894)

    1. Display model plugins on the left panel and added model plugin inspector
    * [BitBucket pull request #1863](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1863)

    1. Context menu and deletion for model plugins
    * [BitBucket pull request #1890](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1890)

    1. Delete self from inspector
    * [BitBucket pull request #1904](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1904)
    * [Issue #1543](https://github.com/osrf/gazebo/issues/1543)

    1. Apply inspector changes in real time and add reset button
    * [BitBucket pull request #1945](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1945)
    * [Issue #1472](https://github.com/osrf/gazebo/issues/1472)

    1. Set physics to be paused when exiting model editor mode
    * [BitBucket pull request #1893](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1893)
    * [Issue #1734](https://github.com/osrf/gazebo/issues/1734)

    1. Add Insert tab to model editor
    * [BitBucket pull request #1924](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1924)

    1. Support inserting nested models from model maker
    * [BitBucket pull request #1982](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1982)

    1. Added joint creation dialog
    * [BitBucket pull request #2021](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2021)

    1. Added reverse checkboxes to joint creation dialog
    * [BitBucket pull request #2086](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2086)

    1. Use opaque pointers in the model editor
    * [BitBucket pull request #2056](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2056)
    * [BitBucket pull request #2059](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2059)
    * [BitBucket pull request #2087](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2087)

    1. Support joint creation between links in nested model.
    * [BitBucket pull request #2080](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2080)

1. Building editor updates

    1. Use opaque pointers in the building editor
    * [BitBucket pull request #2041](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2041)
    * [BitBucket pull request #2039](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2039)
    * [BitBucket pull request #2055](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2055)
    * [BitBucket pull request #2032](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2032)
    * [BitBucket pull request #2082](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2082)
    * [BitBucket pull request #2038](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2038)
    * [BitBucket pull request #2033](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2033)

    1. Use opaque pointers for GrabberHandle, add *LinkedGrabbers functions
    * [BitBucket pull request #2034](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2034)

    1. Removed unused class: BuildingItem
    * [BitBucket pull request #2045](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2045)

    1. Use opaque pointers for BuildingModelManip, move attachment logic to BuildingMaker
    * [BitBucket pull request #2046](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2046)

    1. Use opaque pointers for all Dialog classes, add conversion from QPointF, move common logic to BaseInspectorDialog.
    * [BitBucket pull request #2083](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2083)

## Gazebo 6.0

### Gazebo 6.7.0 (201X-01-12)

1. Add vector3 and quaternion rendering conversions
    * [BitBucket pull request 2276](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2276)

1. Reverse view angle widget left and right view
    * [BitBucket pull request 2265](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2265)
    * [Issue 1924](https://github.com/osrf/gazebo/issues/1924)

1. Fix race condition in ~TimePanelPrivate (#1919)
    * [BitBucket pull request 2250](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2250)

1. Prevent orthographic camera from resetting zoom after animation
    * [BitBucket pull request 2267](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2267)
    * [Issue #1927](https://github.com/osrf/gazebo/issues/1927)

1. Fix MeshToSDF missing scale issue
    * [BitBucket pull request 2258](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2258)
    * [Issue #1925](https://github.com/osrf/gazebo/issues/1925)

1. Register Qt metatypes in gui tests
    * [BitBucket pull request 2273](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2273)

1. Fix resetting model to initial pose
    * [BitBucket pull request 2307](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2307)
    * [Issue #1960](https://github.com/osrf/gazebo/issues/1960)


### Gazebo 6.6.0 (2016-04-07)

1. fix: remove back projection
    * [BitBucket pull request 2201](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2201)
    * A contribution from Yuki Furuta

1. Backport depth camera OSX fix and test
    * [BitBucket pull request 2230](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2230)

1. Add missing tinyxml includes (gazebo6)
    * [BitBucket pull request 2218](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2218)

1. Fix ray-cylinder collision in ode
    * [BitBucket pull request 2125](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2125)

1. backport fixes for ffmpeg3 to gazebo6 (from pull request #2154)
    * [BitBucket pull request 2162](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2162)

1. Install shapes_bitmask.world
    * [BitBucket pull request 2104](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2104)

1. Add gazebo_client to gazebo.pc (gazebo6)
    * [BitBucket pull request 2102](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2102)

1. Fix removing multiple camera sensors that have the same camera name
    * [BitBucket pull request 2081](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2081)

1. Ensure that LINK_FRAME_VISUAL arrow components are deleted (#1812)
    * [BitBucket pull request 2078](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2078)

1. add migration notes for gazebo::setupClient to gazebo::client::setup
    * [BitBucket pull request 2068](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2068)

1. Update inertia properties during simulation: part 2
    * [BitBucket pull request 1984](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1984)

1. Fix minimum window height
    * [BitBucket pull request 2002](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2002)

1. Backport gpu laser test fix
    * [BitBucket pull request 1999](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1999)

1. Relax physics tolerances for single-precision bullet (gazebo6)
    * [BitBucket pull request 1997](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1997)

1. Fix minimum window height
    * [BitBucket pull request 1998](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1998)

1. backport model editor fixed joint option to gazebo6
    * [BitBucket pull request 1957](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1957)

1. Update shaders once per render update
    * [BitBucket pull request 1991](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1991)

1. Relax physics tolerances for single-precision bullet
    * [BitBucket pull request 1976](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1976)

1. Fix visual transparency issues
    * [BitBucket pull request 1967](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1967)

1. fix memory corruption in transport/Publisher.cc
    * [BitBucket pull request 1951](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1951)

1. Add test for SphericalCoordinates::LocalFromGlobal
    * [BitBucket pull request 1959](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1959)

### Gazebo 6.5.1 (2015-10-29)

1. Fix removing multiple camera sensors that have the same camera name.
    * [BitBucket pull request #2081](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2081)
    * [Issue #1811](https://github.com/osrf/gazebo/issues/1811)

1. Backport model editor toolbar fixed joint option from [BitBucket pull request #1794](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1794)
    * [BitBucket pull request #1957](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1957)

1. Fix minimum window height
    * Backport of [BitBucket pull request #1977](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1977)
    * [BitBucket pull request #1998](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1998)
    * [Issue #1706](https://github.com/osrf/gazebo/issues/1706)

1. Fix visual transparency issues
    * [BitBucket pull request #1967](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1967)
    * [Issue #1726](https://github.com/osrf/gazebo/issues/1726)

### Gazebo 6.5.0 (2015-10-22)

1. Added ability to convert from spherical coordinates to local coordinates.
    * [BitBucket pull request #1955](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1955)

### Gazebo 6.4.0 (2015-10-14)

1. Fix ABI problem. Make `Sensor::SetPose` function non virtual.
    * [BitBucket pull request #1947](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1947)

1. Update inertia properties during simulation
    * [BitBucket pull request #1909](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1909)
    * [Design document](https://github.com/osrf/gazebo_design/blob/master/inertia_resize/inertia_resize.md)

1. Fix transparency correction for opaque materials
    * [BitBucket pull request #1946](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1946/fix-transparency-correction-for-opaque/diff)

### Gazebo 6.3.0 (2015-10-06)

1. Added `Sensor::SetPose` function
    * [BitBucket pull request #1935](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1935)

### Gazebo 6.2.0 (2015-10-02)

1. Update physics when the world is reset
    * Backport of [BitBucket pull request #1903](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1903)
    * [BitBucket pull request #1916](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1916)
    * [Issue #101](https://github.com/osrf/gazebo/issues/101)

1. Added Copy constructor and assignment operator to MouseEvent
    * [BitBucket pull request #1855](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1855)

### Gazebo 6.1.0 (2015-08-02)

1. Added logical_camera sensor.
    * [BitBucket pull request #1845](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1845)

1. Added RandomVelocityPlugin, which applies a random velocity to a model's link.
    * [BitBucket pull request #1839](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1839)

1. Sim events for joint position, velocity and applied force
    * [BitBucket pull request #1849](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1849)

### Gazebo 6.0.0 (2015-07-27)

1. Added magnetometer sensor. A contribution from Andrew Symington.
    * [BitBucket pull request #1788](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1788)

1. Added altimeter sensor. A contribution from Andrew Symington.
    * [BitBucket pull request #1792](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1792)

1. Implement more control options for log playback:
  1. Rewind: The simulation starts from the beginning.
  1. Forward: The simulation jumps to the end of the log file.
  1. Seek: The simulation jumps to a specific point specified by its simulation
  time.
    * [BitBucket pull request #1737](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1737)

1. Added Gazebo splash screen
    * [BitBucket pull request #1745](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1745)

1. Added a transporter plugin which allows models to move from one location
   to another based on their location and the location of transporter pads.
    * [BitBucket pull request #1738](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1738)

1. Implement forward/backwards multi-step for log playback. Now, the semantics
of a multi-step while playing back a log session are different from a multi-step
during a live simulation. While playback, a multi-step simulates all the
intermediate steps as before, but the client only perceives a single step.
E.g: You have a log file containing a 1 hour simulation session. You want to
jump to the minute 00H::30M::00S to check a specific aspect of the simulation.
You should not see continuous updates until minute 00H:30M:00S. Instead, you
should visualize a single jump to the specific instant of the simulation that
you are interested.
    * [BitBucket pull request #1623](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1623)

1. Added browse button to log record dialog.
    * [BitBucket pull request #1719](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1719)

1. Improved SVG support: arcs in paths, and contours made of multiple paths.
    * [BitBucket pull request #1608](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1608)

1. Added simulation iterations to the world state.
    * [BitBucket pull request #1722](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1722)

1. Added multiple LiftDrag plugins to the cessna_demo.world to allow the Cessna
C-172 model to fly.
    * [BitBucket pull request #1715](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1715)

1. Added a plugin to control a Cessna C-172 via messages (CessnaPlugin), and a
GUI plugin to test this functionality with the keyboard (CessnaGUIPlugin). Added
world with the Cessna model and the two previous plugins loaded
(cessna_demo.world).
    * [BitBucket pull request #1712](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1712)

1. Added world with OSRF building and an elevator
    * [BitBucket pull request #1697](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1697)

1. Fixed collide bitmask by changing default value from 0x1 to 0xffff.
    * [BitBucket pull request #1696](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1696)

1. Added a plugin to control an elevator (ElevatorPlugin), and an OccupiedEvent plugin that sends a message when a model is within a specified region.
    * [BitBucket pull request #1694](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1694)
    * [BitBucket pull request #1775](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1775)

1. Added Layers tab and meta information for visuals.
    * [BitBucket pull request #1674](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1674)

1. Added countdown behavior for common::Timer and exposed the feature in TimerGUIPlugin.
    * [BitBucket pull request #1690](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1690)

1. Added BuoyancyPlugin for simulating the buoyancy of an object in a column of fluid.
    * [BitBucket pull request #1622](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1622)

1. Added ComputeVolume function for simple shape subclasses of Shape.hh.
    * [BitBucket pull request #1605](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1605)

1. Add option to parallelize the ODE quickstep constraint solver,
which solves an LCP twice with different parameters in order
to corrected for position projection errors.
    * [BitBucket pull request #1561](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1561)

1. Get/Set user camera pose in GUI.
    * [BitBucket pull request #1649](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1649)
    * [Issue #1595](https://github.com/osrf/gazebo/issues/1595)

1. Added ViewAngleWidget, removed hard-coded reset view and removed MainWindow::Reset(). Also added GLWidget::GetSelectedVisuals().
    * [BitBucket pull request #1768](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1768)
    * [Issue #1507](https://github.com/osrf/gazebo/issues/1507)

1. Windows support. This consists mostly of numerous small changes to support
compilation on Windows.
    * [BitBucket pull request #1616](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1616)
    * [BitBucket pull request #1618](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1618)
    * [BitBucket pull request #1620](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1620)
    * [BitBucket pull request #1625](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1625)
    * [BitBucket pull request #1626](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1626)
    * [BitBucket pull request #1627](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1627)
    * [BitBucket pull request #1628](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1628)
    * [BitBucket pull request #1629](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1629)
    * [BitBucket pull request #1630](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1630)
    * [BitBucket pull request #1631](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1631)
    * [BitBucket pull request #1632](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1632)
    * [BitBucket pull request #1633](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1633)
    * [BitBucket pull request #1635](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1635)
    * [BitBucket pull request #1637](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1637)
    * [BitBucket pull request #1639](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1639)
    * [BitBucket pull request #1647](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1647)
    * [BitBucket pull request #1650](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1650)
    * [BitBucket pull request #1651](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1651)
    * [BitBucket pull request #1653](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1653)
    * [BitBucket pull request #1654](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1654)
    * [BitBucket pull request #1657](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1657)
    * [BitBucket pull request #1658](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1658)
    * [BitBucket pull request #1659](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1659)
    * [BitBucket pull request #1660](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1660)
    * [BitBucket pull request #1661](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1661)
    * [BitBucket pull request #1669](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1669)
    * [BitBucket pull request #1670](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1670)
    * [BitBucket pull request #1672](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1672)
    * [BitBucket pull request #1682](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1682)
    * [BitBucket pull request #1683](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1683)

1. Install `libgazebo_server_fixture`. This will facilitate tests external to the main gazebo repository. See `examples/stand_alone/test_fixture`.
    * [BitBucket pull request #1606](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1606)

1. Laser visualization renders light blue for rays that do not hit obstacles, and dark blue for other rays.
    * [BitBucket pull request #1607](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1607)
    * [Issue #1576](https://github.com/osrf/gazebo/issues/1576)

1. Add VisualType enum to Visual and clean up visuals when entity is deleted.
    * [BitBucket pull request #1614](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1614)

1. Alert user of connection problems when using the REST service plugin
    * [BitBucket pull request #1655](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1655)
    * [Issue #1574](https://github.com/osrf/gazebo/issues/1574)

1. ignition-math is now a dependency.
    + [http://ignitionrobotics.org/libraries/math](http://ignitionrobotics.org/libraries/math)
    + [Gazebo::math migration](https://github.com/osrf/gazebo/src/583edbeb90759d43d994cc57c0797119dd6d2794/ign-math-migration.md)

1. Detect uuid library during compilation.
    * [BitBucket pull request #1655](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1655)
    * [Issue #1572](https://github.com/osrf/gazebo/issues/1572)

1. New accessors in LogPlay class.
    * [BitBucket pull request #1577](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1577)

1. Added a plugin to send messages to an existing website.
   Added gui::MainWindow::AddMenu and msgs/rest_error, msgs/rest_login, msgs rest/post
    * [BitBucket pull request #1524](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1524)

1. Fix deprecation warnings when using SDFormat 3.0.2, 3.0.3 prereleases
    * [BitBucket pull request #1568](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1568)

1. Use GAZEBO_CFLAGS or GAZEBO_CXX_FLAGS in CMakeLists.txt for example plugins
    * [BitBucket pull request #1573](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1573)

1. Added Link::OnWrenchMsg subscriber with test
    * [BitBucket pull request #1582](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1582)

1. Show/hide GUI overlays using the menu bar.
    * [BitBucket pull request #1555](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1555)

1. Added world origin indicator rendering::OriginVisual.
    * [BitBucket pull request #1700](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1700)

1. Show/hide toolbars using the menu bars and shortcut.
   Added MainWindow::CloneAction.
   Added Window menu to Model Editor.
    * [BitBucket pull request #1584](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1584)

1. Added event to show/hide toolbars.
    * [BitBucket pull request #1707](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1707)

1. Added optional start/stop/reset buttons to timer GUI plugin.
    * [BitBucket pull request #1576](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1576)

1. Timer GUI Plugin: Treat negative positions as positions from the ends
    * [BitBucket pull request #1703](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1703)

1. Added Visual::GetDepth() and Visual::GetNthAncestor()
    * [BitBucket pull request #1613](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1613)

1. Added a context menu for links
    * [BitBucket pull request #1589](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1589)

1. Separate TimePanel's display into TimeWidget and LogPlayWidget.
    * [BitBucket pull request #1564](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1564)

1. Display confirmation message after log is saved
    * [BitBucket pull request #1646](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1646)

1. Added LogPlayView to display timeline and LogPlaybackStatistics message type.
    * [BitBucket pull request #1724](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1724)

1. Added Time::FormattedString and removed all other FormatTime functions.
    * [BitBucket pull request #1710](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1710)

1. Added support for Oculus DK2
    * [BitBucket pull request #1526](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1526)

1. Use collide_bitmask from SDF to perform collision filtering
    * [BitBucket pull request #1470](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1470)

1. Pass Coulomb surface friction parameters to DART.
    * [BitBucket pull request #1420](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1420)

1. Added ModelAlign::SetHighlighted
    * [BitBucket pull request #1598](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1598)

1. Added various Get functions to Visual. Also added a ConvertGeometryType function to msgs.
    * [BitBucket pull request #1402](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1402)

1. Get and Set visibility of SelectionObj's handles, with unit test.
    * [BitBucket pull request #1417](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1417)

1. Set material of SelectionObj's handles.
    * [BitBucket pull request #1472](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1472)

1. Add SelectionObj::Fini with tests and make Visual::Fini virtual
    * [BitBucket pull request #1685](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1685)

1. Allow link selection with the mouse if parent model already selected.
    * [BitBucket pull request #1409](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1409)

1. Added ModelRightMenu::EntityTypes.
    * [BitBucket pull request #1414](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1414)

1. Scale joint visuals according to link size.
    * [BitBucket pull request #1591](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1591)
    * [Issue #1563](https://github.com/osrf/gazebo/issues/1563)

1. Added Gazebo/CoM material.
    * [BitBucket pull request #1439](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1439)

1. Added arc parameter to MeshManager::CreateTube
    * [BitBucket pull request #1436](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1436)

1. Added View Inertia and InertiaVisual, changed COMVisual to sphere proportional to mass.
    * [BitBucket pull request #1445](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1445)

1. Added View Link Frame and LinkFrameVisual. Visual::SetTransparency goes into texture_unit.
    * [BitBucket pull request #1762](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1762)
    * [Issue #853](https://github.com/osrf/gazebo/issues/853)

1. Changed the position of Save and Cancel buttons on editor dialogs
    * [BitBucket pull request #1442](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1442)
    * [Issue #1377](https://github.com/osrf/gazebo/issues/1377)

1. Fixed Visual material updates
    * [BitBucket pull request #1454](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1454)
    * [Issue #1455](https://github.com/osrf/gazebo/issues/1455)

1. Added Matrix3::Inverse() and tests
    * [BitBucket pull request #1481](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1481)

1. Implemented AddLinkForce for ODE.
    * [BitBucket pull request #1456](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1456)

1. Updated ConfigWidget class to parse enum values.
    * [BitBucket pull request #1518](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1518)

1. Added PresetManager to physics libraries and corresponding integration test.
    * [BitBucket pull request #1471](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1471)

1. Sync name and location on SaveDialog.
    * [BitBucket pull request #1563](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1563)

1. Added Apply Force/Torque dialog
    * [BitBucket pull request #1600](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1600)

1. Added Apply Force/Torque visuals
    * [BitBucket pull request #1619](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1619)

1. Added Apply Force/Torque OnMouseRelease and ActivateWindow
    * [BitBucket pull request #1699](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1699)

1. Added Apply Force/Torque mouse interactions, modes, activation
    * [BitBucket pull request #1731](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1731)

1. Added inertia pose getter for COMVisual and COMVisual_TEST
    * [BitBucket pull request #1581](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1581)

1. Model editor updates
    1. Joint preview using JointVisuals.
    * [BitBucket pull request #1369](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1369)

    1. Added inspector for configuring link, visual, and collision properties.
    * [BitBucket pull request #1408](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1408)

    1. Saving, exiting, generalizing SaveDialog.
    * [BitBucket pull request #1401](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1401)

    1. Inspectors redesign
    * [BitBucket pull request #1586](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1586)

    1. Edit existing model.
    * [BitBucket pull request #1425](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1425)

    1. Add joint inspector to link's context menu.
    * [BitBucket pull request #1449](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1449)
    * [Issue #1443](https://github.com/osrf/gazebo/issues/1443)

    1. Added button to select mesh file on inspector.
    * [BitBucket pull request #1460](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1460)
    * [Issue #1450](https://github.com/osrf/gazebo/issues/1450)

    1. Renamed Part to Link.
    * [BitBucket pull request #1478](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1478)

    1. Fix snapping inside editor.
    * [BitBucket pull request #1489](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1489)
    * [Issue #1457](https://github.com/osrf/gazebo/issues/1457)

    1. Moved DataLogger from Window menu to the toolbar and moved screenshot button to the right.
    * [BitBucket pull request #1665](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1665)

    1. Keep loaded model's name.
    * [BitBucket pull request #1516](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1516)
    * [Issue #1504](https://github.com/osrf/gazebo/issues/1504)

    1. Added ExtrudeDialog.
    * [BitBucket pull request #1483](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1483)

    1. Hide time panel inside editor and keep main window's paused state.
    * [BitBucket pull request #1500](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1500)

    1. Fixed pose issues and added ModelCreator_TEST.
    * [BitBucket pull request #1509](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1509)
    * [Issue #1497](https://github.com/osrf/gazebo/issues/1497)
    * [Issue #1509](https://github.com/osrf/gazebo/issues/1509)

    1. Added list of links and joints.
    * [BitBucket pull request #1515](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1515)
    * [Issue #1418](https://github.com/osrf/gazebo/issues/1418)

    1. Expose API to support adding items to the palette.
    * [BitBucket pull request #1565](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1565)

    1. Added menu for toggling joint visualization
    * [BitBucket pull request #1551](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1551)
    * [Issue #1483](https://github.com/osrf/gazebo/issues/1483)

    1. Add schematic view to model editor
    * [BitBucket pull request #1562](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1562)

1. Building editor updates
    1. Make palette tips tooltip clickable to open.
    * [BitBucket pull request #1519](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1519)
    * [Issue #1370](https://github.com/osrf/gazebo/issues/1370)

    1. Add measurement unit to building inspectors.
    * [BitBucket pull request #1741](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1741)
    * [Issue #1363](https://github.com/osrf/gazebo/issues/1363)

    1. Add `BaseInspectorDialog` as a base class for inspectors.
    * [BitBucket pull request #1749](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1749)

## Gazebo 5.0

### Gazebo 5.4.0 (2017-01-17)

1. Check FSAA support when creating camera render textures
    * [BitBucket pull request 2442](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2442)
    * [Issue #1837](https://github.com/osrf/gazebo/issues/1837)

1. Fix mouse picking with transparent visuals
    * [BitBucket pull request 2305](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2305)
    * [Issue #1956](https://github.com/osrf/gazebo/issues/1956)

1. Backport fix for DepthCamera visibility mask
    * [BitBucket pull request 2286](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2286)
    * [BitBucket pull request 2287](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2287)

1. Backport sensor reset fix
    * [BitBucket pull request 2272](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2272)
    * [Issue #1917](https://github.com/osrf/gazebo/issues/1917)

1. Fix model snap tool highlighting
    * [BitBucket pull request 2293](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2293)
    * [Issue #1955](https://github.com/osrf/gazebo/issues/1955)

### Gazebo 5.3.0 (2015-04-07)

1. fix: remove back projection
    * [BitBucket pull request 2201](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2201)
    * A contribution from Yuki Furuta

1. Backport depth camera OSX fix and test
    * [BitBucket pull request 2230](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2230)

1. Add missing tinyxml includes
    * [BitBucket pull request 2216](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2216)

1. backport fixes for ffmpeg3 to gazebo5 (from pull request #2154)
    * [BitBucket pull request 2161](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2161)

1. Check for valid display using xwininfo -root
    * [BitBucket pull request 2111](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2111)

1. Don't search for sdformat4 on gazebo5, since gazebo5 can't handle sdformat protocol 1.6
    * [BitBucket pull request 2092](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2092)

1. Fix minimum window height
    * [BitBucket pull request 2002](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2002)

1. Relax physics tolerances for single-precision bullet
    * [BitBucket pull request 1976](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1976)

1. Try finding sdformat 4 in gazebo5 branch
    * [BitBucket pull request 1972](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1972)

1. Fix_send_message (backport of pull request #1951)
    * [BitBucket pull request 1964](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1964)
    * A contribution from Samuel Lekieffre

1. Export the media path in the cmake config file.
    * [BitBucket pull request 1933](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1933)

1. Shorten gearbox test since it is failing via timeout on osx
    * [BitBucket pull request 1937](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1937)

### Gazebo 5.2.1 (2015-10-02)

1. Fix minimum window height
    * Backport of [BitBucket pull request #1977](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1977)
    * [BitBucket pull request #2002](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2002)
    * [Issue #1706](https://github.com/osrf/gazebo/issues/1706)

### Gazebo 5.2.0 (2015-10-02)

1. Initialize sigact struct fields that valgrind said were being used uninitialized
    * [BitBucket pull request #1809](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1809)

1. Add missing ogre includes to ensure macros are properly defined
    * [BitBucket pull request #1813](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1813)

1. Use ToSDF functions to simplify physics_friction test
    * [BitBucket pull request #1808](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1808)

1. Added lines to laser sensor visualization
    * [BitBucket pull request #1742](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1742)
    * [Issue #935](https://github.com/osrf/gazebo/issues/935)

1. Fix BulletSliderJoint friction for bullet 2.83
    * [BitBucket pull request #1686](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1686)

1. Fix heightmap model texture loading.
    * [BitBucket pull request #1592](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1592)

1. Disable failing pr2 test for dart
    * [BitBucket pull request #1540](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1540)
    * [Issue #1435](https://github.com/osrf/gazebo/issues/1435)

### Gazebo 5.1.0 (2015-03-20)
1. Backport pull request #1527 (FindOGRE.cmake for non-Debian systems)
    * [BitBucket pull request #1532](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1532)

1. Respect system cflags when not using USE_UPSTREAM_CFLAGS
    * [BitBucket pull request #1531](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1531)

1. Allow light manipulation
    * [BitBucket pull request #1529](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1529)

1. Allow sdformat 2.3.1+ or 3+ and fix tests
    * [BitBucket pull request #1484](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1484)

1. Add Link::GetWorldAngularMomentum function and test.
    * [BitBucket pull request #1482](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1482)

1. Preserve previous GAZEBO_MODEL_PATH values when sourcing setup.sh
    * [BitBucket pull request #1430](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1430)

1. Implement Coulomb joint friction for DART
    * [BitBucket pull request #1427](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1427)
    * [Issue #1281](https://github.com/osrf/gazebo/issues/1281)

1. Fix simple shape normals.
    * [BitBucket pull request #1477](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1477)
    * [Issue #1369](https://github.com/osrf/gazebo/issues/1369)

1. Use Msg-to-SDF conversion functions in tests, add ServerFixture::SpawnModel(msgs::Model).
    * [BitBucket pull request #1466](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1466)

1. Added Model Msg-to-SDF conversion functions and test.
    * [BitBucket pull request #1429](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1429)

1. Added Joint Msg-to-SDF conversion functions and test.
    * [BitBucket pull request #1419](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1419)

1. Added Visual, Material Msg-to-SDF conversion functions and ShaderType to string conversion functions.
    * [BitBucket pull request #1415](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1415)

1. Implement Coulomb joint friction for BulletSliderJoint
    * [BitBucket pull request #1452](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1452)
    * [Issue #1348](https://github.com/osrf/gazebo/issues/1348)

### Gazebo 5.0.0 (2015-01-27)
1. Support for using [digital elevation maps](http://gazebosim.org/tutorials?tut=dem) has been added to debian packages.

1. C++11 support (C++11 compatible compiler is now required)
    * [BitBucket pull request #1340](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1340)

1. Implemented private data pointer for the World class.
    * [BitBucket pull request #1383](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1383)

1. Implemented private data pointer for the Scene class.
    * [BitBucket pull request #1385](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1385)

1. Added a events::Event::resetWorld event that is triggered when World::Reset is called.
    * [BitBucket pull request #1332](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1332)
    * [Issue #1375](https://github.com/osrf/gazebo/issues/1375)

1. Fixed `math::Box::GetCenter` functionality.
    * [BitBucket pull request #1278](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1278)
    * [Issue #1327](https://github.com/osrf/gazebo/issues/1327)

1. Added a GUI timer plugin that facilitates the display and control a timer inside the Gazebo UI.
    * [BitBucket pull request #1270](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1270)

1. Added ability to load plugins via SDF.
    * [BitBucket pull request #1261](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1261)

1. Added GUIEvent to hide/show the left GUI pane.
    * [BitBucket pull request #1269](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1269)

1. Modified KeyEventHandler and GLWidget so that hotkeys can be suppressed by custom KeyEvents set up by developers
    * [BitBucket pull request #1251](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1251)

1. Added ability to read the directory where the log files are stored.
    * [BitBucket pull request #1277](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1277)

1. Implemented a simulation cloner
    * [BitBucket pull request #1180](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1180/clone-a-simulation)

1. Added GUI overlay plugins. Users can now write a Gazebo + QT plugin that displays widgets over the render window.
    * [BitBucket pull request #1181](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1181)

1. Change behavior of Joint::SetVelocity, add Joint::SetVelocityLimit(unsigned int, double)
    * [BitBucket pull request #1218](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1218)
    * [Issue #964](https://github.com/osrf/gazebo/issues/964)

1. Implement Coulomb joint friction for ODE
    * [BitBucket pull request #1221](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1221)
    * [Issue #381](https://github.com/osrf/gazebo/issues/381)

1. Implement Coulomb joint friction for BulletHingeJoint
    * [BitBucket pull request #1317](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1317)
    * [Issue #1348](https://github.com/osrf/gazebo/issues/1348)

1. Implemented camera lens distortion.
    * [BitBucket pull request #1213](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1213)

1. Kill rogue gzservers left over from failed INTEGRATION_world_clone tests
   and improve robustness of `UNIT_gz_TEST`
    * [BitBucket pull request #1232](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1232)
    * [Issue #1299](https://github.com/osrf/gazebo/issues/1299)

1. Added RenderWidget::ShowToolbar to toggle visibility of top toolbar.
    * [BitBucket pull request #1248](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1248)

1. Fix joint axis visualization.
    * [BitBucket pull request #1258](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1258)

1. Change UserCamera view control via joysticks. Clean up rate control vs. pose control.
   see UserCamera::OnJoyPose and UserCamera::OnJoyTwist. Added view twist control toggle
   with joystick button 1.
    * [BitBucket pull request #1249](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1249)

1. Added RenderWidget::GetToolbar to get the top toolbar and change its actions on ModelEditor.
    * [BitBucket pull request #1263](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1263)

1. Added accessor for MainWindow graphical widget to GuiIface.
    * [BitBucket pull request #1250](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1250)

1. Added a ConfigWidget class that takes in a google protobuf message and generates widgets for configuring the fields in the message
    * [BitBucket pull request #1285](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1285)

1. Added GLWidget::OnModelEditor when model editor is triggered, and MainWindow::OnEditorGroup to manually uncheck editor actions.
    * [BitBucket pull request #1283](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1283)

1. Added Collision, Geometry, Inertial, Surface Msg-to-SDF conversion functions.
    * [BitBucket pull request #1315](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1315)

1. Added "button modifier" fields (control, shift, and alt) to common::KeyEvent.
    * [BitBucket pull request #1325](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1325)

1. Added inputs for environment variable GAZEBO_GUI_INI_FILE for reading a custom .ini file.
    * [BitBucket pull request #1252](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1252)

1. Fixed crash on "permission denied" bug, added insert_model integration test.
    * [BitBucket pull request #1329](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1329/)

1. Enable simbody joint tests, implement `SimbodyJoint::GetParam`, create
   `Joint::GetParam`, fix bug in `BulletHingeJoint::SetParam`.
    * [BitBucket pull request #1404](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1404/)

1. Building editor updates
    1. Fixed inspector resizing.
    * [BitBucket pull request #1230](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1230)
    * [Issue #395](https://github.com/osrf/gazebo/issues/395)

    1. Doors and windows move proportionally with wall.
    * [BitBucket pull request #1231](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1231)
    * [Issue #368](https://github.com/osrf/gazebo/issues/368)

    1. Inspector dialogs stay on top.
    * [BitBucket pull request #1229](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1229)
    * [Issue #417](https://github.com/osrf/gazebo/issues/417)

    1. Make model name editable on palette.
    * [BitBucket pull request #1239](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1239)

    1. Import background image and improve add/delete levels.
    * [BitBucket pull request #1214](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1214)
    * [Issue #422](https://github.com/osrf/gazebo/issues/422)
    * [Issue #361](https://github.com/osrf/gazebo/issues/361)

    1. Fix changing draw mode.
    * [BitBucket pull request #1233](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1233)
    * [Issue #405](https://github.com/osrf/gazebo/issues/405)

    1. Tips on palette's top-right corner.
    * [BitBucket pull request #1241](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1241)

    1. New buttons and layout for the palette.
    * [BitBucket pull request #1242](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1242)

    1. Individual wall segments instead of polylines.
    * [BitBucket pull request #1246](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1246)
    * [Issue #389](https://github.com/osrf/gazebo/issues/389)
    * [Issue #415](https://github.com/osrf/gazebo/issues/415)

    1. Fix exiting and saving, exiting when there's nothing drawn, fix text on popups.
    * [BitBucket pull request #1296](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1296)

    1. Display measure for selected wall segment.
    * [BitBucket pull request #1291](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1291)
    * [Issue #366](https://github.com/osrf/gazebo/issues/366)

    1. Highlight selected item's 3D visual.
    * [BitBucket pull request #1292](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1292)

    1. Added color picker to inspector dialogs.
    * [BitBucket pull request #1298](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1298)

    1. Snapping on by default, off holding Shift. Improved snapping.
    * [BitBucket pull request #1304](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1304)

    1. Snap walls to length increments, moved scale to SegmentItem and added Get/SetScale, added SegmentItem::SnapAngle and SegmentItem::SnapLength.
    * [BitBucket pull request #1311](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1311)

    1. Make buildings available in "Insert Models" tab, improve save flow.
    * [BitBucket pull request #1312](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1312)

    1. Added EditorItem::SetHighlighted.
    * [BitBucket pull request #1308](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1308)

    1. Current level is transparent, lower levels opaque, higher levels invisible.
    * [BitBucket pull request #1303](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1303)

    1. Detach all child manips when item is deleted, added BuildingMaker::DetachAllChildren.
    * [BitBucket pull request #1316](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1316)

    1. Added texture picker to inspector dialogs.
    * [BitBucket pull request #1306](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1306)

    1. Measures for doors and windows. Added RectItem::angleOnWall and related Get/Set.
    * [BitBucket pull request #1322](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1322)
    * [Issue #370](https://github.com/osrf/gazebo/issues/370)

    1. Added Gazebo/BuildingFrame material to display holes for doors and windows on walls.
    * [BitBucket pull request #1338](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1338)

    1. Added Gazebo/Bricks material to be used as texture on the building editor.
    * [BitBucket pull request #1333](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1333)

    1. Pick colors from the palette and assign on 3D view. Added mouse and key event handlers to BuildingMaker, and events to communicate from BuildingModelManip to EditorItem.
    * [BitBucket pull request #1336](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1336)

    1. Pick textures from the palette and assign in 3D view.
    * [BitBucket pull request #1368](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1368)

1. Model editor updates
    1. Fix adding/removing event filters .
    * [BitBucket pull request #1279](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1279)

    1. Enabled multi-selection and align tool inside model editor.
    * [BitBucket pull request #1302](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1302)
    * [Issue #1323](https://github.com/osrf/gazebo/issues/1323)

    1. Enabled snap mode inside model editor.
    * [BitBucket pull request #1331](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1331)
    * [Issue #1318](https://github.com/osrf/gazebo/issues/1318)

    1. Implemented copy/pasting of links.
    * [BitBucket pull request #1330](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1330)

1. GUI publishes model selection information on ~/selection topic.
    * [BitBucket pull request #1318](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1318)

## Gazebo 4.0

### Gazebo 4.x.x (2015-xx-xx)

1. Fix build for Bullet 2.83, enable angle wrapping for BulletHingeJoint
    * [BitBucket pull request #1664](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1664)

### Gazebo 4.1.3 (2015-05-07)

1. Fix saving visual geom SDF values
    * [BitBucket pull request #1597](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1597)
1. Fix heightmap model texture loading.
    * [BitBucket pull request #1595](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1595)
1. Fix visual collision scale on separate client
    * [BitBucket pull request #1585](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1585)
1. Fix several clang compiler warnings
    * [BitBucket pull request #1594](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1594)
1. Fix blank save / browse dialogs
    * [BitBucket pull request #1544](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1544)

### Gazebo 4.1.2 (2015-03-20)

1. Fix quaternion documentation: target Gazebo_4.1
    * [BitBucket pull request #1525](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1525)
1. Speed up World::Step in loops
    * [BitBucket pull request #1492](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1492)
1. Reduce selection buffer updates -> 4.1
    * [BitBucket pull request #1494](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1494)
1. Fix loading of SimbodyPhysics parameters
    * [BitBucket pull request #1474](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1474)
1. Fix heightmap on OSX -> 4.1
    * [BitBucket pull request #1455](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1455)
1. Remove extra pose tag in a world file that should not be there
    * [BitBucket pull request #1458](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1458)
1. Better fix for #236 for IMU that doesn't require ABI changes
    * [BitBucket pull request #1448](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1448)
1. Fix regression of #236 for ImuSensor in 4.1
    * [BitBucket pull request #1446](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1446)
1. Preserve previous GAZEBO_MODEL_PATH values when sourcing setup.sh
    * [BitBucket pull request #1430](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1430)
1. issue #857: fix segfault for simbody screw joint when setting limits due to uninitialized limitForce.
    * [BitBucket pull request #1423](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1423)
1. Allow multiple contact sensors per link (#960)
    * [BitBucket pull request #1413](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1413)
1. Fix for issue #351, ODE World Step
    * [BitBucket pull request #1406](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1406)
1. Disable failing InelasticCollision/0 test (#1394)
    * [BitBucket pull request #1405](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1405)
1. Prevent out of bounds array access in SkidSteerDrivePlugin (found by cppcheck 1.68)
    * [BitBucket pull request #1379](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1379)

### Gazebo 4.1.1 (2015-01-15)

1. Fix BulletPlaneShape bounding box (#1265)
    * [BitBucket pull request #1367](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1367)
1. Fix dart linking errors on osx
    * [BitBucket pull request #1372](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1372)
1. Update to player interfaces
    * [BitBucket pull request #1324](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1324)
1. Handle GpuLaser name collisions (#1403)
    * [BitBucket pull request #1360](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1360)
1. Add checks for handling array's with counts of zero, and read specular values
    * [BitBucket pull request #1339](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1339)
1. Fix model list widget test
    * [BitBucket pull request #1327](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1327)
1. Fix ogre includes
    * [BitBucket pull request #1323](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1323)

### Gazebo 4.1.0 (2014-11-20)

1. Modified GUI rendering to improve the rendering update rate.
    * [BitBucket pull request #1487](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1487)
1. Add ArrangePlugin for arranging groups of models.
   Also add Model::ResetPhysicsStates to call Link::ResetPhysicsStates
   recursively on all links in model.
    * [BitBucket pull request #1208](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1208)
1. The `gz model` command line tool will output model info using either `-i` for complete info, or `-p` for just the model pose.
    * [BitBucket pull request #1212](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1212)
    * [DRCSim Issue #389](https://github.com/osrf/drcsim/issue/389)
1. Added SignalStats class for computing incremental signal statistics.
    * [BitBucket pull request #1198](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1198)
1. Add InitialVelocityPlugin to setting the initial state of links
    * [BitBucket pull request #1237](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1237)
1. Added Quaternion::Integrate function.
    * [BitBucket pull request #1255](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1255)
1. Added ConvertJointType functions, display more joint info on model list.
    * [BitBucket pull request #1259](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1259)
1. Added ModelListWidget::AddProperty, removed unnecessary checks on ModelListWidget.
    * [BitBucket pull request #1271](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1271)
1. Fix loading collada meshes with unsupported input semantics.
    * [BitBucket pull request #1319](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1319)

### Gazebo 4.0.2 (2014-09-23)

1. Fix and improve mechanism to generate pkgconfig libs
    * [BitBucket pull request #1207](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1207)
    * [Issue #1284](https://github.com/osrf/gazebo/issues/1284)
1. Added arat.world
    * [BitBucket pull request #1205](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1205)
1. Update gzprop to output zip files.
    * [BitBucket pull request #1197](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1197)
1. Make Collision::GetShape a const function
    * [BitBucket pull requset #1189](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1189)
1. Install missing physics headers
    * [BitBucket pull requset #1183](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1183)
1. Remove SimbodyLink::AddTorque console message
    * [BitBucket pull requset #1185](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1185)
1. Fix log xml
    * [BitBucket pull requset #1188](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1188)

### Gazebo 4.0.0 (2014-08-08)

1. Added lcov support to cmake
    * [BitBucket pull request #1047](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1047)
1. Fixed memory leak in image conversion
    * [BitBucket pull request #1057](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1057)
1. Removed deprecated function
    * [BitBucket pull request #1067](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1067)
1. Improved collada loading performance
    * [BitBucket pull request #1066](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1066)
    * [BitBucket pull request #1082](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1082)
    * [Issue #1134](https://github.com/osrf/gazebo/issues/1134)
1. Implemented a collada exporter
    * [BitBucket pull request #1064](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1064)
1. Force torque sensor now makes use of sensor's pose.
    * [BitBucket pull request #1076](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1076)
    * [Issue #940](https://github.com/osrf/gazebo/issues/940)
1. Fix Model::GetLinks segfault
    * [BitBucket pull request #1093](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1093)
1. Fix deleting and saving lights in gzserver
    * [BitBucket pull request #1094](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1094)
    * [Issue #1182](https://github.com/osrf/gazebo/issues/1182)
    * [Issue #346](https://github.com/osrf/gazebo/issues/346)
1. Fix Collision::GetWorldPose. The pose of a collision would not update properly.
    * [BitBucket pull request #1049](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1049)
    * [Issue #1124](https://github.com/osrf/gazebo/issues/1124)
1. Fixed the animate_box and animate_joints examples
    * [BitBucket pull request #1086](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1086)
1. Integrated Oculus Rift functionality
    * [BitBucket pull request #1074](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1074)
    * [BitBucket pull request #1136](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1136)
    * [BitBucket pull request #1139](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1139)
1. Updated Base::GetScopedName
    * [BitBucket pull request #1104](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1104)
1. Fix collada loader from adding duplicate materials into a Mesh
    * [BitBucket pull request #1105](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1105)
    * [Issue #1180](https://github.com/osrf/gazebo/issues/1180)
1. Integrated Razer Hydra functionality
    * [BitBucket pull request #1083](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1083)
    * [BitBucket pull request #1109](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1109)
1. Added ability to copy and paste models in the GUI
    * [BitBucket pull request #1103](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1103)
1. Removed unnecessary inclusion of gazebo.hh and common.hh in plugins
    * [BitBucket pull request #1111](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1111)
1. Added ability to specify custom road textures
    * [BitBucket pull request #1027](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1027)
1. Added support for DART 4.1
    * [BitBucket pull request #1113](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1113)
    * [BitBucket pull request #1132](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1132)
    * [BitBucket pull request #1134](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1134)
    * [BitBucket pull request #1154](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1154)
1. Allow position of joints to be directly set.
    * [BitBucket pull request #1097](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1097)
    * [Issue #1138](https://github.com/osrf/gazebo/issues/1138)
1. Added extruded polyline geometry
    * [BitBucket pull request #1026](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1026)
1. Fixed actor animation
    * [BitBucket pull request #1133](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1133)
    * [BitBucket pull request #1141](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1141)
1. Generate a versioned cmake config file
    * [BitBucket pull request #1153](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1153)
    * [Issue #1226](https://github.com/osrf/gazebo/issues/1226)
1. Added KMeans class
    * [BitBucket pull request #1147](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1147)
1. Added --summary-range feature to github pullrequest tool
    * [BitBucket pull request #1156](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1156)
1. Updated web links
    * [BitBucket pull request #1159](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1159)
1. Update tests
    * [BitBucket pull request #1155](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1155)
    * [BitBucket pull request #1143](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1143)
    * [BitBucket pull request #1138](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1138)
    * [BitBucket pull request #1140](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1140)
    * [BitBucket pull request #1127](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1127)
    * [BitBucket pull request #1115](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1115)
    * [BitBucket pull request #1102](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1102)
    * [BitBucket pull request #1087](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1087)
    * [BitBucket pull request #1084](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1084)

## Gazebo 3.0

### Gazebo 3.x.x (yyyy-mm-dd)

1. Fixed sonar and wireless sensor visualization
    * [BitBucket pull request #1254](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1254)
1. Update visual bounding box when model is selected
    * [BitBucket pull request #1280](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1280)

### Gazebo 3.1.0 (2014-08-08)

1. Implemented Simbody::Link::Set*Vel
    * [BitBucket pull request #1160](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1160)
    * [Issue #1012](https://github.com/osrf/gazebo/issues/1012)
1. Added World::RemoveModel function
    * [BitBucket pull request #1106](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1106)
    * [Issue #1177](https://github.com/osrf/gazebo/issues/1177)
1. Fix exit from camera follow mode using the escape key
    * [BitBucket pull request #1137](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1137)
    * [Issue #1220](https://github.com/osrf/gazebo/issues/1220)
1. Added support for SDF joint spring stiffness and reference positions
    * [BitBucket pull request #1117](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1117)
1. Removed the gzmodel_create script
    * [BitBucket pull request #1130](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1130)
1. Added Vector2 dot product
    * [BitBucket pull request #1101](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1101)
1. Added SetPositionPID and SetVelocityPID to JointController
    * [BitBucket pull request #1091](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1091)
1. Fix gzclient startup crash with ogre 1.9
    * [BitBucket pull request #1098](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1098)
    * [Issue #996](https://github.com/osrf/gazebo/issues/996)
1. Update the github_pullrequests tool
    * [BitBucket pull request #1108](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1108)
1. Light properties now remain in place after move by the user via the GUI.
    * [BitBucket pull request #1110](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1110)
    * [Issue #1211](https://github.com/osrf/gazebo/issues/1211)
1. Allow position of joints to be directly set.
    * [BitBucket pull request #1096](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1096)
    * [Issue #1138](https://github.com/osrf/gazebo/issues/1138)

### Gazebo 3.0.0 (2014-04-11)

1. Fix bug when deleting the sun light
    * [BitBucket pull request #1088](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1088)
    * [Issue #1133](https://github.com/osrf/gazebo/issues/1133)
1. Fix ODE screw joint
    * [BitBucket pull request #1078](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1078)
    * [Issue #1167](https://github.com/osrf/gazebo/issues/1167)
1. Update joint integration tests
    * [BitBucket pull request #1081](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1081)
1. Fixed false positives in cppcheck.
    * [BitBucket pull request #1061](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1061)
1. Made joint axis reference frame relative to child, and updated simbody and dart accordingly.
    * [BitBucket pull request #1069](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1069)
    * [Issue #494](https://github.com/osrf/gazebo/issues/494)
    * [Issue #1143](https://github.com/osrf/gazebo/issues/1143)
1. Added ability to pass vector of strings to SetupClient and SetupServer
    * [BitBucket pull request #1068](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1068)
    * [Issue #1132](https://github.com/osrf/gazebo/issues/1132)
1. Fix error correction in screw constraints for ODE
    * [BitBucket pull request #1070](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1070)
    * [Issue #1159](https://github.com/osrf/gazebo/issues/1159)
1. Improved pkgconfig with SDF
    * [BitBucket pull request #1062](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1062)
1. Added a plugin to simulate aero dynamics
    * [BitBucket pull request #905](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/905)
1. Updated bullet support
    * [Issue #1069](https://github.com/osrf/gazebo/issues/1069)
    * [BitBucket pull request #1011](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1011)
    * [BitBucket pull request #996](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/966)
    * [BitBucket pull request #1024](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1024)
1. Updated simbody support
    * [BitBucket pull request #995](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/995)
1. Updated worlds to SDF 1.5
    * [BitBucket pull request #1021](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1021)
1. Improvements to ODE
    * [BitBucket pull request #1001](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1001)
    * [BitBucket pull request #1014](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1014)
    * [BitBucket pull request #1015](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1015)
    * [BitBucket pull request #1016](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1016)
1. New command line tool
    * [BitBucket pull request #972](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/972)
1. Graphical user interface improvements
    * [BitBucket pull request #971](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/971)
    * [BitBucket pull request #1013](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1013)
    * [BitBucket pull request #989](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/989)
1. Created a friction pyramid class
    * [BitBucket pull request #935](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/935)
1. Added GetWorldEnergy functions to Model, Joint, and Link
    * [BitBucket pull request #1017](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1017)
1. Preparing Gazebo for admission into Ubuntu
    * [BitBucket pull request #969](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/969)
    * [BitBucket pull request #998](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/998)
    * [BitBucket pull request #1002](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1002)
1. Add method for querying if useImplicitStiffnessDamping flag is set for a given joint
    * [Issue #629](https://github.com/osrf/gazebo/issues/629)
    * [BitBucket pull request #1006](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1006)
1. Fix joint axis frames
    * [Issue #494](https://github.com/osrf/gazebo/issues/494)
    * [BitBucket pull request #963](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/963)
1. Compute joint anchor pose relative to parent
    * [Issue #1029](https://github.com/osrf/gazebo/issues/1029)
    * [BitBucket pull request #982](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/982)
1. Cleanup the installed worlds
    * [Issue #1036](https://github.com/osrf/gazebo/issues/1036)
    * [BitBucket pull request #984](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/984)
1. Update to the GPS sensor
    * [Issue #1059](https://github.com/osrf/gazebo/issues/1059)
    * [BitBucket pull request #978](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/978)
1. Removed libtool from plugin loading
    * [BitBucket pull request #981](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/981)
1. Added functions to get inertial information for a link in the world frame.
    * [BitBucket pull request #1005](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1005)

## Gazebo 2.0

### Gazebo 2.2.6 (2015-09-28)

1. Backport fixes to setup.sh from pull request #1430 to 2.2 branch
    * [BitBucket pull request 1889](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1889)
1. Fix heightmap texture loading (2.2)
    * [BitBucket pull request 1596](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1596)
1. Prevent out of bounds array access in SkidSteerDrivePlugin (found by cppcheck 1.68)
    * [BitBucket pull request 1379](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1379)
1. Fix build with boost 1.57 for 2.2 branch (#1399)
    * [BitBucket pull request 1358](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1358)
1. Fix manpage test failures by incrementing year to 2015
    * [BitBucket pull request 1361](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1361)
1. Fix build for OS X 10.10 (#1304, #1289)
    * [BitBucket pull request 1346](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1346)
1. Restore ODELink ABI, use Link variables instead (#1354)
    * [BitBucket pull request 1347](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1347)
1. Fix inertia_ratio test
    * [BitBucket pull request 1344](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1344)
1. backport collision visual fix -> 2.2
    * [BitBucket pull request 1343](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1343)
1. Fix two code_check errors on 2.2
    * [BitBucket pull request 1314](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1314)
1. issue #243 fix Link::GetWorldLinearAccel and Link::GetWorldAngularAccel for ODE
    * [BitBucket pull request 1284](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1284)

### Gazebo 2.2.3 (2014-04-29)

1. Removed redundant call to World::Init
    * [BitBucket pull request #1107](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1107)
    * [Issue #1208](https://github.com/osrf/gazebo/issues/1208)
1. Return proper error codes when gazebo exits
    * [BitBucket pull request #1085](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1085)
    * [Issue #1178](https://github.com/osrf/gazebo/issues/1178)
1. Fixed Camera::GetWorldRotation().
    * [BitBucket pull request #1071](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1071)
    * [Issue #1087](https://github.com/osrf/gazebo/issues/1087)
1. Fixed memory leak in image conversion
    * [BitBucket pull request #1073](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1073)

### Gazebo 2.2.1 (xxxx-xx-xx)

1. Fix heightmap model texture loading.
    * [BitBucket pull request #1596](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1596)

### Gazebo 2.2.0 (2014-01-10)

1. Fix compilation when using OGRE-1.9 (full support is being worked on)
    * [Issue #994](https://github.com/osrf/gazebo/issues/994)
    * [Issue #995](https://github.com/osrf/gazebo/issues/995)
    * [Issue #996](https://github.com/osrf/gazebo/issues/996)
    * [BitBucket pull request #883](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/883)
1. Added unit test for issue 624.
    * [Issue #624](https://github.com/osrf/gazebo/issues/624).
    * [BitBucket pull request #889](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/889)
1. Use 3x3 PCF shadows for smoother shadows.
    * [BitBucket pull request #887](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/887)
1. Update manpage copyright to 2014.
    * [BitBucket pull request #893](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/893)
1. Added friction integration test .
    * [BitBucket pull request #885](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/885)
1. Fix joint anchor when link pose is not specified.
    * [Issue #978](https://github.com/osrf/gazebo/issues/978)
    * [BitBucket pull request #862](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/862)
1. Added (ESC) tooltip for GUI Selection Mode icon.
    * [Issue #993](https://github.com/osrf/gazebo/issues/993)
    * [BitBucket pull request #888](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/888)
1. Removed old comment about resolved issue.
    * [Issue #837](https://github.com/osrf/gazebo/issues/837)
    * [BitBucket pull request #880](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/880)
1. Made SimbodyLink::Get* function thread-safe
    * [Issue #918](https://github.com/osrf/gazebo/issues/918)
    * [BitBucket pull request #872](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/872)
1. Suppressed spurious gzlog messages in ODE::Body
    * [Issue #983](https://github.com/osrf/gazebo/issues/983)
    * [BitBucket pull request #875](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/875)
1. Fixed Force Torque Sensor Test by properly initializing some values.
    * [Issue #982](https://github.com/osrf/gazebo/issues/982)
    * [BitBucket pull request #869](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/869)
1. Added breakable joint plugin to support breakable walls.
    * [BitBucket pull request #865](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/865)
1. Used different tuple syntax to fix compilation on OSX mavericks.
    * [Issue #947](https://github.com/osrf/gazebo/issues/947)
    * [BitBucket pull request #858](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/858)
1. Fixed sonar test and deprecation warning.
    * [BitBucket pull request #856](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/856)
1. Speed up test compilation.
    * Part of [Issue #955](https://github.com/osrf/gazebo/issues/955)
    * [BitBucket pull request #846](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/846)
1. Added Joint::SetEffortLimit API
    * [Issue #923](https://github.com/osrf/gazebo/issues/923)
    * [BitBucket pull request #808](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/808)
1. Made bullet output less verbose.
    * [BitBucket pull request #839](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/839)
1. Convergence acceleration and stability tweak to make atlas_v3 stable
    * [Issue #895](https://github.com/osrf/gazebo/issues/895)
    * [BitBucket pull request #772](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/772)
1. Added colors, textures and world files for the SPL RoboCup environment
    * [BitBucket pull request #838](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/838)
1. Fixed github_pullrequests tool to work with latest GitHub API.
    * [Issue #933](https://github.com/osrf/gazebo/issues/933)
    * [BitBucket pull request #841](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/841)
1. Fixed cppcheck warnings.
    * [BitBucket pull request #842](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/842)

### Gazebo 2.1.0 (2013-11-08)
1. Fix mainwindow unit test
    * [BitBucket pull request #752](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/752)
1. Visualize moment of inertia
    * Pull request [#745](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/745), [#769](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/769), [#787](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/787)
    * [Issue #203](https://github.com/osrf/gazebo/issues/203)
1. Update tool to count lines of code
    * [BitBucket pull request #758](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/758)
1. Implement World::Clear
    * Pull request [#785](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/785), [#804](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/804)
1. Improve Bullet support
    * [BitBucket pull request #805](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/805)
1. Fix doxygen spacing
    * [BitBucket pull request #740](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/740)
1. Add tool to generate model images for thepropshop.org
    * [BitBucket pull request #734](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/734)
1. Added paging support for terrains
    * [BitBucket pull request #707](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/707)
1. Added plugin path to LID_LIBRARY_PATH in setup.sh
    * [BitBucket pull request #750](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/750)
1. Fix for OSX
    * [BitBucket pull request #766](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/766)
    * [BitBucket pull request #786](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/786)
    * [Issue #906](https://github.com/osrf/gazebo/issues/906)
1. Update copyright information
    * [BitBucket pull request #771](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/771)
1. Enable screen dependent tests
    * [BitBucket pull request #764](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/764)
    * [Issue #811](https://github.com/osrf/gazebo/issues/811)
1. Fix gazebo command line help message
    * [BitBucket pull request #775](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/775)
    * [Issue #898](https://github.com/osrf/gazebo/issues/898)
1. Fix man page test
    * [BitBucket pull request #774](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/774)
1. Improve load time by reducing calls to RTShader::Update
    * [BitBucket pull request #773](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/773)
    * [Issue #877](https://github.com/osrf/gazebo/issues/877)
1. Fix joint visualization
    * [BitBucket pull request #776](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/776)
    * [BitBucket pull request #802](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/802)
    * [Issue #464](https://github.com/osrf/gazebo/issues/464)
1. Add helpers to fix NaN
    * [BitBucket pull request #742](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/742)
1. Fix model resizing via the GUI
    * [BitBucket pull request #763](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/763)
    * [Issue #885](https://github.com/osrf/gazebo/issues/885)
1. Simplify gzlog test by using sha1
    * [BitBucket pull request #781](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/781)
    * [Issue #837](https://github.com/osrf/gazebo/issues/837)
1. Enable cppcheck for header files
    * [BitBucket pull request #782](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/782)
    * [Issue #907](https://github.com/osrf/gazebo/issues/907)
1. Fix broken regression test
    * [BitBucket pull request #784](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/784)
    * [Issue #884](https://github.com/osrf/gazebo/issues/884)
1. All simbody and dart to pass tests
    * [BitBucket pull request #790](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/790)
    * [Issue #873](https://github.com/osrf/gazebo/issues/873)
1. Fix camera rotation from SDF
    * [BitBucket pull request #789](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/789)
    * [Issue #920](https://github.com/osrf/gazebo/issues/920)
1. Fix github pullrequest command line tool to match new API
    * [BitBucket pull request #803](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/803)
1. Fix transceiver spawn errors in tests
    * [BitBucket pull request #811](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/811)
    * [BitBucket pull request #814](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/814)

### Gazebo 2.0.0 (2013-10-08)
1. Refactor code check tool.
    * [BitBucket pull request #669](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/669)
1. Added pull request tool for GitHub.
    * [BitBucket pull request #670](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/670)
    * [BitBucket pull request #691](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/671)
1. New wireless receiver and transmitter sensor models.
    * [BitBucket pull request #644](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/644)
    * [BitBucket pull request #675](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/675)
    * [BitBucket pull request #727](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/727)
1. Audio support using OpenAL.
    * [BitBucket pull request #648](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/648)
    * [BitBucket pull request #704](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/704)
1. Simplify command-line parsing of gztopic echo output.
    * [BitBucket pull request #674](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/674)
    * Resolves: [Issue #795](https://github.com/osrf/gazebo/issues/795)
1. Use UNIX directories through the user of GNUInstallDirs cmake module.
    * [BitBucket pull request #676](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/676)
    * [BitBucket pull request #681](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/681)
1. New GUI interactions for object manipulation.
    * [BitBucket pull request #634](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/634)
1. Fix for OSX menubar.
    * [BitBucket pull request #677](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/677)
1. Remove internal SDF directories and dependencies.
    * [BitBucket pull request #680](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/680)
1. Add minimum version for sdformat.
    * [BitBucket pull request #682](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/682)
    * Resolves: [Issue #818](https://github.com/osrf/gazebo/issues/818)
1. Allow different gtest parameter types with ServerFixture
    * [BitBucket pull request #686](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/686)
    * Resolves: [Issue #820](https://github.com/osrf/gazebo/issues/820)
1. GUI model scaling when using Bullet.
    * [BitBucket pull request #683](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/683)
1. Fix typo in cmake config.
    * [BitBucket pull request #694](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/694)
    * Resolves: [Issue #824](https://github.com/osrf/gazebo/issues/824)
1. Remove gazebo include subdir from pkgconfig and cmake config.
    * [BitBucket pull request #691](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/691)
1. Torsional spring demo
    * [BitBucket pull request #693](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/693)
1. Remove repeated call to SetAxis in Joint.cc
    * [BitBucket pull request #695](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/695)
    * Resolves: [Issue #823](https://github.com/osrf/gazebo/issues/823)
1. Add test for rotational joints.
    * [BitBucket pull request #697](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/697)
    * Resolves: [Issue #820](https://github.com/osrf/gazebo/issues/820)
1. Fix compilation of tests using Joint base class
    * [BitBucket pull request #701](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/701)
1. Terrain paging implemented.
    * [BitBucket pull request #687](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/687)
1. Improve timeout error reporting in ServerFixture
    * [BitBucket pull request #705](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/705)
1. Fix mouse picking for cases where visuals overlap with the laser
    * [BitBucket pull request #709](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/709)
1. Fix string literals for OSX
    * [BitBucket pull request #712](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/712)
    * Resolves: [Issue #803](https://github.com/osrf/gazebo/issues/803)
1. Support for ENABLE_TESTS_COMPILATION cmake parameter
    * [BitBucket pull request #708](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/708)
1. Updated system gui plugin
    * [BitBucket pull request #702](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/702)
1. Fix force torque unit test issue
    * [BitBucket pull request #673](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/673)
    * Resolves: [Issue #813](https://github.com/osrf/gazebo/issues/813)
1. Use variables to control auto generation of CFlags
    * [BitBucket pull request #699](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/699)
1. Remove deprecated functions.
    * [BitBucket pull request #715](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/715)
1. Fix typo in `Camera.cc`
    * [BitBucket pull request #719](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/719)
    * Resolves: [Issue #846](https://github.com/osrf/gazebo/issues/846)
1. Performance improvements
    * [BitBucket pull request #561](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/561)
1. Fix gripper model.
    * [BitBucket pull request #713](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/713)
    * Resolves: [Issue #314](https://github.com/osrf/gazebo/issues/314)
1. First part of Simbody integration
    * [BitBucket pull request #716](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/716)

## Gazebo 1.9

### Gazebo 1.9.6 (2014-04-29)

1. Refactored inertia ratio reduction for ODE
    * [BitBucket pull request #1114](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1114)
1. Improved collada loading performance
    * [BitBucket pull request #1075](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1075)

### Gazebo 1.9.3 (2014-01-10)

1. Add thickness to plane to remove shadow flickering.
    * [BitBucket pull request #886](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/886)
1. Temporary GUI shadow toggle fix.
    * [Issue #925](https://github.com/osrf/gazebo/issues/925)
    * [BitBucket pull request #868](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/868)
1. Fix memory access bugs with libc++ on mavericks.
    * [Issue #965](https://github.com/osrf/gazebo/issues/965)
    * [BitBucket pull request #857](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/857)
    * [BitBucket pull request #881](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/881)
1. Replaced printf with cout in gztopic hz.
    * [Issue #969](https://github.com/osrf/gazebo/issues/969)
    * [BitBucket pull request #854](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/854)
1. Add Dark grey material and fix indentation.
    * [BitBucket pull request #851](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/851)
1. Fixed sonar sensor unit test.
    * [BitBucket pull request #848](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/848)
1. Convergence acceleration and stability tweak to make atlas_v3 stable.
    * [BitBucket pull request #845](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/845)
1. Update gtest to 1.7.0 to resolve problems with libc++.
    * [Issue #947](https://github.com/osrf/gazebo/issues/947)
    * [BitBucket pull request #827](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/827)
1. Fixed LD_LIBRARY_PATH for plugins.
    * [Issue #957](https://github.com/osrf/gazebo/issues/957)
    * [BitBucket pull request #844](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/844)
1. Fix transceiver sporadic errors.
    * Backport of [BitBucket pull request #811](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/811)
    * [BitBucket pull request #836](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/836)
1. Modified the MsgTest to be deterministic with time checks.
    * [BitBucket pull request #843](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/843)
1. Fixed seg fault in LaserVisual.
    * [Issue #950](https://github.com/osrf/gazebo/issues/950)
    * [BitBucket pull request #832](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/832)
1. Implemented the option to disable tests that need a working screen to run properly.
    * Backport of [BitBucket pull request #764](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/764)
    * [BitBucket pull request #837](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/837)
1. Cleaned up gazebo shutdown.
    * [BitBucket pull request #829](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/829)
1. Fixed bug associated with loading joint child links.
    * [Issue #943](https://github.com/osrf/gazebo/issues/943)
    * [BitBucket pull request #820](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/820)

### Gazebo 1.9.2 (2013-11-08)
1. Fix enable/disable sky and clouds from SDF
    * [BitBucket pull request #809](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/809])
1. Fix occasional blank GUI screen on startup
    * [BitBucket pull request #815](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/815])
1. Fix GPU laser when interacting with heightmaps
    * [BitBucket pull request #796](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/796])
1. Added API/ABI checker command line tool
    * [BitBucket pull request #765](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/765])
1. Added gtest version information
    * [BitBucket pull request #801](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/801])
1. Fix GUI world saving
    * [BitBucket pull request #806](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/806])
1. Enable anti-aliasing for camera sensor
    * [BitBucket pull request #800](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/800])
1. Make sensor noise deterministic
    * [BitBucket pull request #788](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/788])
1. Fix build problem
    * [Issue #901](https://github.com/osrf/gazebo/issues/901)
    * [BitBucket pull request #778](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/778])
1. Fix a typo in Camera.cc
    * [BitBucket pull request #720](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/720])
    * [Issue #846](https://github.com/osrf/gazebo/issues/846)
1. Fix OSX menu bar
    * [BitBucket pull request #688](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/688])
1. Fix gazebo::init by calling sdf::setFindCallback() before loading the sdf in gzfactory.
    * [BitBucket pull request #678](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/678])
    * [Issue #817](https://github.com/osrf/gazebo/issues/817)

### Gazebo 1.9.1 (2013-08-20)
* Deprecate header files that require case-sensitive filesystem (e.g. Common.hh, Physics.hh) [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/638/fix-for-775-deprecate-headers-that-require]
* Initial support for building on Mac OS X [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/660/osx-support-for-gazebo-19] [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/657/cmake-fixes-for-osx]
* Fixes for various issues [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/635/fix-for-issue-792/diff] [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/628/allow-scoped-and-non-scoped-joint-names-to/diff] [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/636/fix-build-dependency-in-message-generation/diff] [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/639/make-the-unversioned-setupsh-a-copy-of-the/diff] [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/650/added-missing-lib-to-player-client-library/diff] [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/656/install-gzmode_create-without-sh-suffix/diff]

### Gazebo 1.9.0 (2013-07-23)
* Use external package [sdformat](https://github.com/osrf/sdformat) for sdf parsing, refactor the `Element::GetValue*` function calls, and deprecate Gazebo's internal sdf parser [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/627]
* Improved ROS support ([[Tutorials#ROS_Integration |documentation here]]) [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/559]
* Added Sonar, Force-Torque, and Tactile Pressure sensors [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/557], [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/567]
* Add compile-time defaults for environment variables so that sourcing setup.sh is unnecessary in most cases [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/620]
* Enable user camera to follow objects in client window [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/603]
* Install protobuf message files for use in custom messages [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/614]
* Change default compilation flags to improve debugging [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/617]
* Change to supported relative include paths [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/594]
* Fix display of laser scans when sensor is rotated [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/599]

## Gazebo 1.8

### Gazebo 1.8.7 (2013-07-16)
* Fix bug in URDF parsing of Vector3 elements [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/613]
* Fix compilation errors with newest libraries [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/615]

### Gazebo 1.8.6 (2013-06-07)
* Fix inertia lumping in the URDF parser[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/554]
* Fix for ODEJoint CFM damping sign error [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/586]
* Fix transport memory growth[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/584]
* Reduce log file data in order to reduce buffer growth that results in out of memory kernel errors[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/587]

### Gazebo 1.8.5 (2013-06-04)
* Fix Gazebo build for machines without a valid display.[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/commits/37f00422eea03365b839a632c1850431ee6a1d67]

### Gazebo 1.8.4 (2013-06-03)
* Fix UDRF to SDF converter so that URDF gazebo extensions are applied to all collisions in a link.[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/579]
* Prevent transport layer from locking when a gzclient connects to a gzserver over a connection with high latency.[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/572]
* Improve performance and fix uninitialized conditional jumps.[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/571]

### Gazebo 1.8.3 (2013-06-03)
* Fix for gzlog hanging when gzserver is not present or not responsive[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/577]
* Fix occasional segfault when generating log files[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/575]
* Performance improvement to ODE[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/556]
* Fix node initialization[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/570]
* Fix GPU laser Hz rate reduction when sensor moved away from world origin[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/566]
* Fix incorrect lighting in camera sensors when GPU laser is subscribe to[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/563]

### Gazebo 1.8.2 (2013-05-28)
* ODE performance improvements[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/535][https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/537]
* Fixed tests[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/538][https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/541][https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/542]
* Fixed sinking vehicle bug[https://github.com/osrf/drcsim/issue/300] in pull-request[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/538]
* Fix GPU sensor throttling[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/536]
* Reduce string comparisons for better performance[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/546]
* Contact manager performance improvements[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/543]
* Transport performance improvements[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/548]
* Reduce friction noise[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/545]

### Gazebo 1.8.1 (2013-05-22)
* Please note that 1.8.1 contains a bug[https://github.com/osrf/drcsim/issue/300] that causes interpenetration between objects in resting contact to grow slowly.  Please update to 1.8.2 for the patch.
* Added warm starting[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/529]
* Reduced console output[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/533]
* Improved off screen rendering performance[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/530]
* Performance improvements [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/535] [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/537]

### Gazebo 1.8.0 (2013-05-17)
* Fixed slider axis [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/527]
* Fixed heightmap shadows [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/525]
* Fixed model and canonical link pose [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/519]
* Fixed OSX message header[https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/524]
* Added zlib compression for logging [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/515]
* Allow clouds to be disabled in cameras [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/507]
* Camera rendering performance [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/528]


## Gazebo 1.7

### Gazebo 1.7.3 (2013-05-08)
* Fixed log cleanup (again) [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/511/fix-log-cleanup-logic]

### Gazebo 1.7.2 (2013-05-07)
* Fixed log cleanup [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/506/fix-gzlog-stop-command-line]
* Minor documentation fix [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/488/minor-documentation-fix]

### Gazebo 1.7.1 (2013-04-19)
* Fixed tests
* IMU sensor receives time stamped data from links
* Fix saving image frames [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/466/fix-saving-frames/diff]
* Wireframe rendering in GUI [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/414/allow-rendering-of-models-in-wireframe]
* Improved logging performance [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/457/improvements-to-gzlog-filter-and-logging]
* Viscous mud model [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/448/mud-plugin/diff]

## Gazebo 1.6

### Gazebo 1.6.3 (2013-04-15)
* Fixed a [critical SDF bug](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/451)
* Fixed a [laser offset bug](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/449)

### Gazebo 1.6.2 (2013-04-14)
* Fix for fdir1 physics property [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/429/fixes-to-treat-fdir1-better-1-rotate-into/diff]
* Fix for force torque sensor [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/447]
* SDF documentation fix [https://github.com/osrf/gazebo/issues/494/joint-axis-reference-frame-doesnt-match]

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
    * [https://github.com/osrf/gazebo/issues/236 Issue #236]
    * [https://github.com/osrf/gazebo/issues/507 Issue #507]
    * [https://github.com/osrf/gazebo/issues/530 Issue #530]
    * [https://github.com/osrf/gazebo/issues/279 Issue #279]
    * [https://github.com/osrf/gazebo/issues/529 Issue #529]
    * [https://github.com/osrf/gazebo/issues/239 Issue #239]
    * [https://github.com/osrf/gazebo/issues/5 Issue #5]

## Gazebo 1.4

### Gazebo 1.4.0 (2013-02-01)
* New Features:
 * GUI elements to display messages from the server.
 * Multi-floor building editor and creator.
 * Improved sensor visualizations.
 * Improved mouse interactions

* Fixed the following issues:
    * [https://github.com/osrf/gazebo/issues/16 Issue #16]
    * [https://github.com/osrf/gazebo/issues/142 Issue #142]
    * [https://github.com/osrf/gazebo/issues/229 Issue #229]
    * [https://github.com/osrf/gazebo/issues/277 Issue #277]
    * [https://github.com/osrf/gazebo/issues/291 Issue #291]
    * [https://github.com/osrf/gazebo/issues/310 Issue #310]
    * [https://github.com/osrf/gazebo/issues/320 Issue #320]
    * [https://github.com/osrf/gazebo/issues/329 Issue #329]
    * [https://github.com/osrf/gazebo/issues/333 Issue #333]
    * [https://github.com/osrf/gazebo/issues/334 Issue #334]
    * [https://github.com/osrf/gazebo/issues/335 Issue #335]
    * [https://github.com/osrf/gazebo/issues/341 Issue #341]
    * [https://github.com/osrf/gazebo/issues/350 Issue #350]
    * [https://github.com/osrf/gazebo/issues/384 Issue #384]
    * [https://github.com/osrf/gazebo/issues/431 Issue #431]
    * [https://github.com/osrf/gazebo/issues/433 Issue #433]
    * [https://github.com/osrf/gazebo/issues/453 Issue #453]
    * [https://github.com/osrf/gazebo/issues/456 Issue #456]
    * [https://github.com/osrf/gazebo/issues/457 Issue #457]
    * [https://github.com/osrf/gazebo/issues/459 Issue #459]

## Gazebo 1.3

### Gazebo 1.3.1 (2012-12-14)
* Fixed the following issues:
    * [https://github.com/osrf/gazebo/issues/297 Issue #297]
* Other bugs fixed:
    * [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/164/ Fix light bounding box to disable properly when deselected]
    * [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/169/ Determine correct local IP address, to make remote clients work properly]
 * Various test fixes

### Gazebo 1.3.0 (2012-12-03)
* Fixed the following issues:
    * [https://github.com/osrf/gazebo/issues/233 Issue #233]
    * [https://github.com/osrf/gazebo/issues/238 Issue #238]
    * [https://github.com/osrf/gazebo/issues/2 Issue #2]
    * [https://github.com/osrf/gazebo/issues/95 Issue #95]
    * [https://github.com/osrf/gazebo/issues/97 Issue #97]
    * [https://github.com/osrf/gazebo/issues/90 Issue #90]
    * [https://github.com/osrf/gazebo/issues/253 Issue #253]
    * [https://github.com/osrf/gazebo/issues/163 Issue #163]
    * [https://github.com/osrf/gazebo/issues/91 Issue #91]
    * [https://github.com/osrf/gazebo/issues/245 Issue #245]
    * [https://github.com/osrf/gazebo/issues/242 Issue #242]
    * [https://github.com/osrf/gazebo/issues/156 Issue #156]
    * [https://github.com/osrf/gazebo/issues/78 Issue #78]
    * [https://github.com/osrf/gazebo/issues/36 Issue #36]
    * [https://github.com/osrf/gazebo/issues/104 Issue #104]
    * [https://github.com/osrf/gazebo/issues/249 Issue #249]
    * [https://github.com/osrf/gazebo/issues/244 Issue #244]

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
* Fixed a transport issue with the GUI. Fixed saving the world via the GUI. Added more documentation. ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/43/fixed-a-transport-issue-with-the-gui-fixed/diff pull request #43])
* Clean up mutex usage. ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/54/fix-mutex-in-modellistwidget-using-boost/diff pull request #54])
* Fix OGRE path determination ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/58/fix-ogre-paths-so-this-also-works-with/diff pull request #58], [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/68/fix-ogre-plugindir-determination/diff pull request #68])
* Fixed a couple of crashes and model selection/dragging problems ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/59/fixed-a-couple-of-crashes-and-model/diff pull request #59])

### Gazebo 1.2.5 (2012-10-22)
* Step increment update while paused fixed ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/45/fix-proper-world-stepinc-count-we-were/diff pull request #45])
* Actually call plugin destructors on shutdown ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/51/fixed-a-bug-which-prevent-a-plugin/diff pull request #51])
* Don't crash on bad SDF input ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/52/fixed-loading-of-bad-sdf-files/diff pull request #52])
* Fix cleanup of ray sensors on model deletion ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/53/deleting-a-model-with-a-ray-sensor-did/diff pull request #53])
* Fix loading / deletion of improperly specified models ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/56/catch-when-loading-bad-models-joint/diff pull request #56])

### Gazebo 1.2.4 (10-19-2012:08:00:52)
*  Style fixes ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/30/style-fixes/diff pull request #30]).
*  Fix joint position control ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/49/fixed-position-joint-control/diff pull request #49])

### Gazebo 1.2.3 (10-16-2012:18:39:54)
*  Disabled selection highlighting due to bug ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/44/disabled-selection-highlighting-fixed/diff pull request #44]).
*  Fixed saving a world via the GUI.

### Gazebo 1.2.2 (10-16-2012:15:12:22)
*  Skip search for system install of libccd, use version inside gazebo ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/39/skip-search-for-system-install-of-libccd/diff pull request #39]).
*  Fixed sensor initialization race condition ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/42/fix-sensor-initializaiton-race-condition pull request #42]).

### Gazebo 1.2.1 (10-15-2012:21:32:55)
*  Properly removed projectors attached to deleted models ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/37/remove-projectors-that-are-attached-to/diff pull request #37]).
*  Fix model plugin loading bug ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/31/moving-bool-first-in-model-and-world pull request #31]).
*  Fix light insertion and visualization of models prior to insertion ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/35/fixed-light-insertion-and-visualization-of/diff pull request #35]).
*  Fixed GUI manipulation of static objects ([https://github.com/osrf/gazebo/issues/63/moving-static-objects-does-not-move-the issue #63] [https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/38/issue-63-bug-patch-moving-static-objects/diff pull request #38]).
*  Fixed GUI selection bug ([https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/40/fixed-selection-of-multiple-objects-at/diff pull request #40])

### Gazebo 1.2.0 (10-04-2012:20:01:20)
*  Updated GUI: new style, improved mouse controls, and removal of non-functional items.
*  Model database: An online repository of models.
*  Numerous bug fixes
*  APT repository hosted at [http://osrfoundation.org OSRF]
*  Improved process control prevents zombie processes

