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

#define DT_MIN 1e-4
#define DT_MAX 1.01e-3
#define DT_STEP 3.0e-4
INSTANTIATE_TEST_CASE_P(EnginesDtLinear, BoxesTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Range(DT_MIN, DT_MAX, DT_STEP)
  , ::testing::Values(1)
  , ::testing::Values(true)
  , ::testing::Values(false)));

INSTANTIATE_TEST_CASE_P(EnginesDtNonlinear, BoxesTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Range(DT_MIN, DT_MAX, DT_STEP)
  , ::testing::Values(1)
  , ::testing::Values(true)
  , ::testing::Values(true)));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
