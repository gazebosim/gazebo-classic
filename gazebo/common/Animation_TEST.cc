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

#include <gtest/gtest.h>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/Animation.hh"

using namespace gazebo;

TEST(AnimationTest, PoseAnimation)
{
  {
    common::PoseAnimation anim("test", 1.0, true);
    anim.SetTime(-0.5);
    EXPECT_NEAR(0.5, anim.GetTime(), 1e-6);
  }

  {
    common::PoseAnimation anim("test", 1.0, false);
    anim.SetTime(-0.5);
    EXPECT_NEAR(0.0, anim.GetTime(), 1e-6);

    anim.SetTime(1.5);
    EXPECT_NEAR(1.0, anim.GetTime(), 1e-6);
  }


  common::PoseAnimation anim("pose_test", 5.0, false);
  common::PoseKeyFrame *key = anim.CreateKeyFrame(0.0);

  EXPECT_NEAR(5.0, anim.GetLength(), 1e-6);
  anim.SetLength(10.0);
  EXPECT_NEAR(10.0, anim.GetLength(), 1e-6);

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
  EXPECT_NEAR(5.0, anim.GetTime(), 1e-6);

  anim.SetTime(4.0);
  EXPECT_NEAR(4.0, anim.GetTime(), 1e-6);

  common::PoseKeyFrame interpolatedKey(-1.0);
  anim.GetInterpolatedKeyFrame(interpolatedKey);
  EXPECT_TRUE(interpolatedKey.GetTranslation() ==
      math::Vector3(3.76, 7.52, 11.28));
  EXPECT_TRUE(interpolatedKey.GetRotation() ==
      math::Quaternion(0.0302776, 0.0785971, 0.109824));
}

TEST(AnimationTest, NumericAnimation)
{
  common::NumericAnimation anim("numeric_test", 10, false);
  common::NumericKeyFrame *key = anim.CreateKeyFrame(0.0);

  key->SetValue(0.0);
  EXPECT_NEAR(0.0, key->GetValue(), 1e-6);

  key = anim.CreateKeyFrame(10.0);
  key->SetValue(30);
  EXPECT_NEAR(30, key->GetValue(), 1e-6);

  anim.AddTime(5.0);
  EXPECT_NEAR(5.0, anim.GetTime(), 1e-6);

  anim.SetTime(4.0);
  EXPECT_NEAR(4.0, anim.GetTime(), 1e-6);

  common::NumericKeyFrame interpolatedKey(0);
  anim.GetInterpolatedKeyFrame(interpolatedKey);
  EXPECT_NEAR(12, interpolatedKey.GetValue(), 1e-6);
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
