#ifndef _TEST_GENERATOR_HH
#define _TEST_GENERATOR_HH_

#define ODE_SUPPORT(testclass) INSTANTIATE_TEST_CASE_P(TestODE, testclass, ::testing::Values("ode"))
#define BULLET_SUPPORT(testclass)

#ifdef HAVE_BULLET
#undef BULLET_SUPPORT
#define BULLET_SUPPORT(testclass) INSTANTIATE_TEST_CASE_P(TestBullet, testclass, ::testing::Values("bullet"))
#endif

/// \brief Helper macro to instantiate gtest for using different physics engines
#define INSTANTIATE_PHYSICS_ENGINES_TEST(testclass) \
    ODE_SUPPORT(testclass); \
    BULLET_SUPPORT(testclass)

#endif
