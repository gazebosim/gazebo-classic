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
  {
    common::PoseAnimation anim("test", 1.0, true);
    anim.SetTime(-0.5);
    EXPECT_EQ(0.5, anim.GetTime());
  }

  {
    common::PoseAnimation anim("test", 1.0, false);
    anim.SetTime(-0.5);
    EXPECT_EQ(0.0, anim.GetTime());

    anim.SetTime(1.5);
    EXPECT_EQ(1.0, anim.GetTime());
  }


  common::PoseAnimation anim("pose_test", 5.0, false);
  common::PoseKeyFrame *key = anim.CreateKeyFrame(0.0);

  EXPECT_EQ(5.0, anim.GetLength());
  anim.SetLength(10.0);
  EXPECT_EQ(10.0, anim.GetLength());

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

  common::PoseKeyFrame interpolatedKey(-1.0);
  anim.GetInterpolatedKeyFrame(interpolatedKey);
  EXPECT_TRUE(interpolatedKey.GetTranslation() ==
      math::Vector3(3.76, 7.52, 11.28));
  EXPECT_TRUE(interpolatedKey.GetRotation() ==
      math::Quaternion(0.0302776, 0.0785971, 0.109824));
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
  EXPECT_EQ(12, interpolatedKey.GetValue());
}

TEST_F(CommonTest, Color)
{
  common::Color clr(.1, .2, .3, 1.0);
  EXPECT_EQ(0.1f, clr.R());
  EXPECT_EQ(0.2f, clr.G());
  EXPECT_EQ(0.3f, clr.B());
  EXPECT_EQ(1.0f, clr.A());

  clr.Reset();
  EXPECT_EQ(0.0f, clr.R());
  EXPECT_EQ(0.0f, clr.G());
  EXPECT_EQ(0.0f, clr.B());
  EXPECT_EQ(0.0f, clr.A());

  clr.SetFromHSV(0, 0.5, 1.0);
  EXPECT_EQ(1.0f, clr.R());
  EXPECT_EQ(0.5f, clr.G());
  EXPECT_EQ(0.5f, clr.B());
  EXPECT_EQ(0.0f, clr.A());

  EXPECT_TRUE(clr.GetAsHSV() == math::Vector3(6, 0.5, 1));

  clr.SetFromYUV(0.5, 0.2, 0.8);
  EXPECT_TRUE(math::equal(0.00553f, clr.R(), 1e-3f));
  EXPECT_TRUE(math::equal(0.0f, clr.G()));
  EXPECT_TRUE(math::equal(0.9064f, clr.B(), 1e-3f));
  EXPECT_TRUE(math::equal(0.0f, clr.A()));

  EXPECT_TRUE(clr.GetAsYUV() == math::Vector3(0.104985, 0.95227, 0.429305));

  clr = common::Color(1.0, 0.0, 0.5, 1.0) + common::Color(0.1, 0.3, 0.4, 1.0);
  EXPECT_TRUE(math::equal(0.00431373f, clr.R()));
  EXPECT_TRUE(math::equal(0.3f, clr.G()));
  EXPECT_TRUE(math::equal(0.9f, clr.B()));
  EXPECT_TRUE(math::equal(2.0f, clr.A()));

  clr = common::Color(1.0, 0.0, 0.5, 1.0) - common::Color(0.1, 0.3, 0.4, 1.0);
  EXPECT_TRUE(math::equal(0.9f, clr.R()));
  EXPECT_TRUE(math::equal(0.0f, clr.G()));
  EXPECT_TRUE(math::equal(0.1f, clr.B()));
  EXPECT_TRUE(math::equal(0.0f, clr.A()));

  clr = common::Color(0.5, 0.2, 0.4, 0.6) / 2.0;
  EXPECT_TRUE(math::equal(0.25f, clr.R()));
  EXPECT_TRUE(math::equal(0.1f, clr.G()));
  EXPECT_TRUE(math::equal(0.2f, clr.B()));
  EXPECT_TRUE(math::equal(0.3f, clr.A()));
}

TEST_F(CommonTest, Time)
{
  /*
  common::Timer timer;
  timer.Start();
  usleep(100000);
  EXPECT_TRUE(timer.GetElapsed() > common::Time(0, 100000000));

  struct timeval tv;
  gettimeofday(&tv, NULL); 
  common::Time time(tv);
  EXPECT_EQ(time.sec, tv.tv_sec);
  EXPECT_EQ(time.nsec, tv.tv_usec * 1000);

  time.SetToWallTime();
  EXPECT_TRUE(common::Time::GetWallTime() - time < common::Time(0,1000000));

  time = common::Time(1,1000) + common::Time(1.5, 1000000000);
  EXPECT_TRUE(time == common::Time(3.5, 1000));

  time.Set(1, 1000);
  time += common::Time(1.5, 1000000000);
  EXPECT_TRUE(time == common::Time(3.5, 1000));

  tv.tv_sec = 2;
  tv.tv_usec = 1000000;
  time = common::Time(1,1000) + tv;
  EXPECT_TRUE(time == common::Time(4.0, 1000));

  time.Set(1, 1000);
  time += tv;
  EXPECT_TRUE(time == common::Time(4, 1000));

  time = common::Time(1,1000) - tv;
  EXPECT_TRUE(time == common::Time(-2, 1000));

  time.Set(1, 1000);
  time -= tv;
  EXPECT_TRUE(time == common::Time(-2, 1000));

  tv.tv_sec = 2;
  tv.tv_usec = 1000;
  time = common::Time(1, 1000) * tv;
  std::cout << "T1[" << time.sec << ", " << time.nsec << "]\n";
  EXPECT_TRUE(time == common::Time(3, 0));

  time.Set(1, 1000);
  time *= tv;
  std::cout << "T2[" << time.sec << ", " << time.nsec << "]\n";
  EXPECT_TRUE(time == common::Time(3, 0));

  time.Set(1, 1000);
  time = common::Time(1, 1000) * common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(2, 2000));


  time = common::Time(1,2000000) / tv;
  std::cout << "T3[" << time.sec << ", " << time.nsec << "][" << tv.tv_usec*1000 << "\n";
  EXPECT_TRUE(time == common::Time(0, 1000));

  time.Set(1, 2000000);
  time /= tv;
  std::cout << "T4[" << time.sec << ", " << time.nsec << "]\n";
  EXPECT_TRUE(time == common::Time(0, 1000));

  time.Set(1, 1000);
  time = common::Time(1, 1000) / common::Time(2, 2);
  std::cout << "T5[" << time.sec << ", " << time.nsec << "]\n";
  EXPECT_TRUE(time == common::Time(.5, 500));
  */
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
