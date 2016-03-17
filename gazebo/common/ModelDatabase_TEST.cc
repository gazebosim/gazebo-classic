/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/common/ModelDatabase.hh"
#include "test/util.hh"

using namespace gazebo;

class ModelDatabaseTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(ModelDatabaseTest, Version)
{
  std::string uri = "model://coke_can";
  std::string model;
  model = gazebo::common::ModelDatabase::Instance()->GetModelFile(uri);

  // uncomment to check the actual value. This assumes that the correct value
  // is model.sdf, rather than model-1_2.sdf or something similar. This test
  // is at the mercy of changes in the model database and sdf parser versions

  // std::cout << "uri: " << uri << ", model file: " << model << std::endl;

  EXPECT_TRUE(model.find("model.sdf") != std::string::npos);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
