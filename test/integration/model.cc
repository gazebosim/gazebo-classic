/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "ServerFixture.hh"

class ModelTest : public ServerFixture
{
};

/////////////////////////////////////////////////
// This tests getting links from a model.
TEST_F(ModelTest, GetLinksV)
{
  Load("worlds/joint_test.world");
  physics::ModelPtr model = GetModel("model_1");

  // This for-loop would cause a seg-fault in gazebo 2.2 and before.
  for (physics::Link_V::const_iterator iter = model->GetLinks().begin();
       iter != model->GetLinks().end(); ++iter)
  {
    EXPECT_TRUE(*iter);
    EXPECT_FALSE((*iter)->GetName().empty());
  }

  EXPECT_EQ(model->GetLinks().size(), 1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
