#ifndef _TEST_GENERATOR_HH
#define _TEST_GENERATOR_HH_

#ifdef HAVE_BULLET
/// \brief Helper macro to instantiate gtest for using different physics engines
#define INSTANTIATE_PHYSICS_ENGINES_TEST(testclass) \
     INSTANTIATE_TEST_CASE_P(TestODE, testclass, ::testing::Values("ode"));  \
     INSTANTIATE_TEST_CASE_P(TestBullet, testclass, ::testing::Values("bullet"))
#else
/// \brief Helper macro to instantiate gtest for using different physics engines
#define INSTANTIATE_PHYSICS_ENGINES_TEST(testclass) \
  INSTANTIATE_TEST_CASE_P(TestODE, testclass, ::testing::Values("ode"))
#endif

#endif
