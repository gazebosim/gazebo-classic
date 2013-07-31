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

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include "test_config.h"
#include "gazebo/common/Common.hh"
#include "gazebo/util/OpenAL.hh"

using namespace gazebo;

/////////////////////////////////////////////////
TEST(OpenAL, SourceInvalid)
{
  common::load();
  util::OpenALSourcePtr source;
  EXPECT_EQ(util::OpenAL::Instance()->CreateSource(sdf::ElementPtr()), source);

  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());
}

/////////////////////////////////////////////////
TEST(OpenAL, BadSDF)
{
  common::load();
  util::OpenALSourcePtr source;
  EXPECT_TRUE(util::OpenAL::Instance()->Load());

  // NULL SDF
  EXPECT_EQ(util::OpenAL::Instance()->CreateSource(sdf::ElementPtr()), source);

  sdf::SDFPtr sdf(new sdf::SDF);
  // Bad SDF
  EXPECT_EQ(util::OpenAL::Instance()->CreateSource(sdf->root), source);

  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());
}

/////////////////////////////////////////////////
TEST(OpenAL, BadValues)
{
  common::load();

  EXPECT_TRUE(util::OpenAL::Instance()->Load());

  util::OpenALSourcePtr source;

  sdf::SDFPtr sdf(new sdf::SDF);
  sdf::initFile("audio_source.sdf", sdf->root);

  std::string sdfString = "<sdf version='1.4'>"
    "<audio_source>"
    "<uri>file://media/audio/cheer.wav</uri>"
    "<pitch>2.0</pitch>"
    "<gain>1.0</gain>"
    "<loop>true</loop>"
    "</audio_source>"
    "</sdf>";

  EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

  source = util::OpenAL::Instance()->CreateSource(sdf->root);
  EXPECT_TRUE(source != NULL);

  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());

  EXPECT_FALSE(source->SetPitch(0));
  EXPECT_FALSE(source->SetGain(-1));
  EXPECT_FALSE(source->SetLoop(false));
}

/////////////////////////////////////////////////
TEST(OpenAL, SourcePlay)
{
  util::OpenALSourcePtr source;
  common::load();

  EXPECT_TRUE(util::OpenAL::Instance()->Load());

  sdf::SDFPtr sdf(new sdf::SDF);
  sdf::initFile("audio_source.sdf", sdf->root);

  std::string sdfString = "<sdf version='1.4'>"
    "<audio_source>"
    "<uri>file://media/audio/cheer.wav</uri>"
    "<pitch>2.0</pitch>"
    "<gain>1.0</gain>"
    "<loop>true</loop>"
    "</audio_source>"
    "</sdf>";

  EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

  source = util::OpenAL::Instance()->CreateSource(sdf->root);
  EXPECT_TRUE(source != NULL);

  EXPECT_NO_THROW(source->Play());
  EXPECT_TRUE(source->IsPlaying());
  EXPECT_NO_THROW(source->Play());
  EXPECT_TRUE(source->IsPlaying());

  EXPECT_NO_THROW(source->Rewind());

  EXPECT_NO_THROW(source->Play());
  EXPECT_TRUE(source->IsPlaying());

  EXPECT_NO_THROW(source->Pause());
  EXPECT_FALSE(source->IsPlaying());
  EXPECT_NO_THROW(source->Pause());
  EXPECT_FALSE(source->IsPlaying());

  EXPECT_NO_THROW(source->Play());
  EXPECT_TRUE(source->IsPlaying());
  EXPECT_NO_THROW(source->Stop());
  EXPECT_FALSE(source->IsPlaying());
  EXPECT_NO_THROW(source->Stop());
  EXPECT_FALSE(source->IsPlaying());

  EXPECT_NO_THROW(source->Rewind());

  EXPECT_NO_THROW(source->Play());
  EXPECT_TRUE(source->IsPlaying());

  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());
}

/////////////////////////////////////////////////
TEST(OpenAL, SourceVelPose)
{
  util::OpenALSourcePtr source;
  common::load();

  sdf::SDFPtr sdf(new sdf::SDF);
  sdf::initFile("audio_source.sdf", sdf->root);

  std::string sdfString = "<sdf version='1.4'>"
    "<audio_source>"
    "<uri>file://media/audio/cheer.wav</uri>"
    "<pitch>2.0</pitch>"
    "<gain>1.0</gain>"
    "<loop>true</loop>"
    "</audio_source>"
    "</sdf>";

  EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

  EXPECT_TRUE(util::OpenAL::Instance()->Load());
  source = util::OpenAL::Instance()->CreateSource(sdf->root);
  EXPECT_TRUE(source != NULL);

  EXPECT_TRUE(source->SetVelocity(math::Vector3(1,1,1)));
  EXPECT_TRUE(source->SetPose(math::Pose(1,1,1,0,0,0)));
}

/////////////////////////////////////////////////
TEST(OpenAL, Sourcevalid)
{
  common::load();

  EXPECT_TRUE(util::OpenAL::Instance()->Load());

  // Has pitch, gain, loop
  {
    sdf::SDFPtr sdf(new sdf::SDF);
    sdf::initFile("audio_source.sdf", sdf->root);

    std::string sdfString = "<sdf version='1.4'>"
      "<audio_source>"
      "<uri>file://media/audio/cheer.wav</uri>"
      "<pitch>2.0</pitch>"
      "<gain>1.0</gain>"
      "<loop>true</loop>"
      "</audio_source>"
      "</sdf>";

    EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

    EXPECT_TRUE(util::OpenAL::Instance()->CreateSource(sdf->root));
  }

  // No Pitch, gain, loop
  {
    sdf::SDFPtr sdf(new sdf::SDF);
    sdf::initFile("audio_source.sdf", sdf->root);

    std::string sdfString = "<sdf version='1.4'>"
      "<audio_source>"
      "<uri>file://media/audio/cheer.wav</uri>"
      "</audio_source>"
      "</sdf>";

    EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

    EXPECT_TRUE(util::OpenAL::Instance()->CreateSource(sdf->root));
  }

  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());

  // Calling Fini twice shouldn't cause a problem
  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());
}

/////////////////////////////////////////////////
TEST(OpenAL, SinkCreate)
{
  util::OpenALSinkPtr sink1;
  util::OpenALSinkPtr sink2;
  common::load();

  sink1 = util::OpenAL::Instance()->CreateSink(sdf::ElementPtr());
  sink2 = util::OpenAL::Instance()->CreateSink(sdf::ElementPtr());

  EXPECT_TRUE(sink1 != NULL);
  EXPECT_TRUE(sink2 == NULL);

  EXPECT_NO_THROW(util::OpenAL::Instance()->Fini());
  EXPECT_NO_THROW(sink1.reset());
}

/////////////////////////////////////////////////
TEST(OpenAL, SinkVelPose)
{
  util::OpenALSinkPtr sink;
  common::load();

  sink = util::OpenAL::Instance()->CreateSink(sdf::ElementPtr());
  EXPECT_TRUE(sink != NULL);

  EXPECT_FALSE(sink->SetVelocity(math::Vector3(1, 1, 1)));
  EXPECT_FALSE(sink->SetPose(math::Pose(1, 1, 1, 0, 0, 0)));

  EXPECT_TRUE(util::OpenAL::Instance()->Load());
  EXPECT_TRUE(sink->SetVelocity(math::Vector3(1, 1, 1)));
  EXPECT_TRUE(sink->SetPose(math::Pose(1, 1, 1, 0, 0, 0)));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
