/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <string.h>

#include "test/accuracy/boxes.hh"
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;

#define MODELS_MIN 1
#define MODELS_MAX 105
#define MODELS_STEP 20
INSTANTIATE_TEST_CASE_P(OdeBoxes, BoxesTest,
  ::testing::Combine(::testing::Values("ode")
  , ::testing::Values(3.0e-4)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Bool()
  , ::testing::Values(true)));

INSTANTIATE_TEST_CASE_P(BulletBoxes, BoxesTest,
  ::testing::Combine(::testing::Values("bullet")
  , ::testing::Values(3.0e-4)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Bool()
  , ::testing::Values(true)));

INSTANTIATE_TEST_CASE_P(SimbodyBoxes, BoxesTest,
  ::testing::Combine(::testing::Values("simbody")
  , ::testing::Values(7.0e-4)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Bool()
  , ::testing::Values(true)));

INSTANTIATE_TEST_CASE_P(DartBoxes, BoxesTest,
  ::testing::Combine(::testing::Values("dart")
  , ::testing::Values(7.0e-4)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Bool()
  , ::testing::Values(true)));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
