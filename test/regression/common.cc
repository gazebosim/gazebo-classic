/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "ServerFixture.hh"
#include "common/common.h"

using namespace gazebo;
class CommonTest : public ServerFixture
{
};

TEST_F(CommonTest, PoseAnimation)
{
  common::PoseAnimation anim("pose_test", 10, false);
  common::PoseKeyFrame *key = anim.CreateKeyFrame(0.0);

  key->SetTranslation(math::Vector3(0, 0, 0));
  EXPECT_TRUE(key->GetTranslation() == math::Vector3(0, 0, 0));

  key->SetRotation(math::Quaternion(0, 0, 0));
  EXPECT_TRUE(key->GetRotation() == math::Quaternion(0, 0, 0));

  key = anim.CreateKeyFrame(10.0);
  key->SetTranslation(math::Vector3(10, 20, 30));
  EXPECT_TRUE(key->GetTranslation() == math::Vector3(10, 20, 30));

  key->SetRotation(math::Quaternion(0.1, 0.2, 0.3));
  EXPECT_TRUE(key->GetRotation() == math::Quaternion(0.1, 0.2, 0.3));

  anim.AddTime(5.0);
  EXPECT_EQ(5.0, anim.GetTime());

  anim.SetTime(4.0);
  EXPECT_EQ(4.0, anim.GetTime());

  common::PoseKeyFrame interpolatedKey(0);
  anim.GetInterpolatedKeyFrame(interpolatedKey);
  std::cout << "Pose Int[" << interpolatedKey.GetTranslation() << "][" << interpolatedKey.GetRotation() << "]\n";
}

TEST_F(CommonTest, NumericAnimation)
{
  common::NumericAnimation anim("numeric_test", 10, false);
  common::NumericKeyFrame *key = anim.CreateKeyFrame(0.0);

  key->SetValue(0.0);
  EXPECT_EQ(0.0, key->GetValue());

  key = anim.CreateKeyFrame(10.0);
  key->SetValue(30);
  EXPECT_EQ(30, key->GetValue());

  anim.AddTime(5.0);
  EXPECT_EQ(5.0, anim.GetTime());

  anim.SetTime(4.0);
  EXPECT_EQ(4.0, anim.GetTime());

  common::NumericKeyFrame interpolatedKey(0);
  anim.GetInterpolatedKeyFrame(interpolatedKey);
  std::cout << "NUm interpolated[" << interpolatedKey.GetValue() << "]\n";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
