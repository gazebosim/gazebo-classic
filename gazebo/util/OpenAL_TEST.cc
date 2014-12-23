/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/common/CommonIface.hh"
#include "gazebo/util/OpenAL.hh"
#include "gazebo/gazebo_config.h"
#include "test/util.hh"

using namespace gazebo;

class OpenAL : public gazebo::testing::AutoLogFixture { };

#ifdef HAVE_OPENAL
/////////////////////////////////////////////////
TEST_F(OpenAL, SourceInvalid)
{
  common::load();
  util::OpenALSourcePtr source;
  EXPECT_EQ(util::OpenAL::Instance()->CreateSource(sdf::ElementPtr()), source);

  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());
}
/////////////////////////////////////////////////
TEST_F(OpenAL, DefaultDevice)
{
  common::load();

  sdf::SDFPtr sdf(new sdf::SDF);
  sdf::initFile("world.sdf", sdf->root);

  std::string sdfString = "<sdf version='1.4'>"
    "<world name='default'>"
    "<audio>"
    "<device>default</device>"
    "</device>"
    "</world>"
    "</sdf>";

  EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

  EXPECT_TRUE(util::OpenAL::Instance()->Load(sdf->root));
  EXPECT_TRUE(util::OpenAL::Instance()->Load(sdf->root->GetElement("audio")));
}

/////////////////////////////////////////////////
TEST_F(OpenAL, NonDefaultDevice)
{
  common::load();

  sdf::SDFPtr sdf(new sdf::SDF);
  sdf::initFile("world.sdf", sdf->root);

  std::string sdfString = "<sdf version='1.4'>"
    "<world name='default'>"
    "<audio>"
    "<device>garbage</device>"
    "</device>"
    "</world>"
    "</sdf>";

  EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

  EXPECT_FALSE(util::OpenAL::Instance()->Load(sdf->root->GetElement("audio")));
}

/////////////////////////////////////////////////
TEST_F(OpenAL, BadSDF)
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
TEST_F(OpenAL, BadValues)
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

  EXPECT_FALSE(source->HasCollisionName("name2"));
}

/////////////////////////////////////////////////
TEST_F(OpenAL, SourcePlay)
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
    "<contact><collision>name</collision><collision>name2</collision></contact>"
    "</audio_source>"
    "</sdf>";


  EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

  source = util::OpenAL::Instance()->CreateSource(sdf->root);
  EXPECT_TRUE(source != NULL);

  EXPECT_TRUE(source->GetOnContact());

  EXPECT_EQ(source->GetCollisionNames().size(), 2u);
  EXPECT_EQ(source->GetCollisionNames()[0], "name");
  EXPECT_EQ(source->GetCollisionNames()[1], "name2");
  EXPECT_TRUE(source->HasCollisionName("name2"));
  EXPECT_TRUE(source->HasCollisionName("name"));

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
TEST_F(OpenAL, SourceVelPose)
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

  EXPECT_FALSE(source->GetOnContact());
  EXPECT_TRUE(source->SetVelocity(math::Vector3(1, 1, 1)));
  EXPECT_TRUE(source->SetPose(math::Pose(1, 1, 1, 0, 0, 0)));
}

/////////////////////////////////////////////////
TEST_F(OpenAL, Sourcevalid)
{
  common::load();

  EXPECT_TRUE(util::OpenAL::Instance()->Load());

  // Has pitch, gain, loop, contact
  {
    sdf::SDFPtr sdf(new sdf::SDF);
    sdf::initFile("audio_source.sdf", sdf->root);

    std::string sdfString = "<sdf version='1.4'>"
      "<audio_source>"
      "<uri>file://media/audio/cheer.wav</uri>"
      "<pitch>2.0</pitch>"
      "<gain>1.0</gain>"
      "<loop>true</loop>"
      "<contact>true</contact>"
      "</audio_source>"
      "</sdf>";

    EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

    EXPECT_TRUE(util::OpenAL::Instance()->CreateSource(sdf->root) != NULL);
  }

  // No Pitch, gain, loop, contact
  {
    sdf::SDFPtr sdf(new sdf::SDF);
    sdf::initFile("audio_source.sdf", sdf->root);

    std::string sdfString = "<sdf version='1.4'>"
      "<audio_source>"
      "<uri>file://media/audio/cheer.wav</uri>"
      "</audio_source>"
      "</sdf>";

    EXPECT_TRUE(sdf::readString(sdfString, sdf->root));

    EXPECT_TRUE(util::OpenAL::Instance()->CreateSource(sdf->root) != NULL);
  }

  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());

  // Calling Fini twice shouldn't cause a problem
  ASSERT_NO_THROW(util::OpenAL::Instance()->Fini());
}

/////////////////////////////////////////////////
TEST_F(OpenAL, SinkCreate)
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
TEST_F(OpenAL, SinkVelPose)
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
#endif

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
