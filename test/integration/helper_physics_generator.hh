/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _TEST_GENERATOR_HH
#define _TEST_GENERATOR_HH_

#define ODE_SUPPORT(testclass) INSTANTIATE_TEST_CASE_P(TestODE, \
    testclass, ::testing::Values("ode"));
#define BULLET_SUPPORT(testclass)
#define DART_SUPPORT(testclass)

#ifdef HAVE_BULLET
#undef BULLET_SUPPORT
#define BULLET_SUPPORT(testclass) INSTANTIATE_TEST_CASE_P(TestBullet, \
    testclass, ::testing::Values("bullet"));
#endif

#ifdef HAVE_DART
#undef DART_SUPPORT
#define DART_SUPPORT(testclass) INSTANTIATE_TEST_CASE_P(TestDART, \
    testclass, ::testing::Values("dart"));
#endif

/// \brief Helper macro to instantiate gtest for using different physics engines
#define INSTANTIATE_PHYSICS_ENGINES_TEST(testclass) \
    ODE_SUPPORT(testclass) \
    BULLET_SUPPORT(testclass) \
    DART_SUPPORT(testclass)

#endif
