/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/sensors/sensors.hh"

using namespace gazebo;
class ShadowCasterRenderBackFacesTest : public ServerFixture
{
  /// \brief Test rendering back faces
  public: void ShadowCasterBackFaces();

  /// \brief Test rendering without back face
  public: void ShadowCasterNoBackFaces();

  /// \brief Counter for the numbder of image messages received.
  public: unsigned int imageCount = 0u;

  /// \brief Depth data buffer.
  public: unsigned char* imageBuffer = nullptr;

  /// \brief Camera image callback
  /// \param[in] _msg Message with image data containing raw image values.
  public: void OnImage(ConstImageStampedPtr &_msg);
};

/////////////////////////////////////////////////
void ShadowCasterRenderBackFacesTest::OnImage(ConstImageStampedPtr &_msg)
{
  unsigned int imageSamples = _msg->image().width() *_msg->image().height() * 3;
  memcpy(this->imageBuffer, _msg->image().data().c_str(), imageSamples);
  this->imageCount++;
}

/////////////////////////////////////////////////
// \brief The shadow caster will render back faces; The plane in the world will
/// cast a shadow
void ShadowCasterRenderBackFacesTest::ShadowCasterBackFaces()
{
  this->Load("worlds/shadow_caster_back_faces.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  sensors::CameraSensorPtr sensor =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_normal"));
  EXPECT_TRUE(sensor != nullptr);
  EXPECT_TRUE(sensor->IsActive());

  unsigned int width  = 320;
  unsigned int height = 240;

  this->imageCount = 0;
  this->imageBuffer = new unsigned char[width * height*3];

  transport::NodePtr node(new transport::Node());
  node->Init();

  std::string topic = sensor->Topic();
  EXPECT_TRUE(!topic.empty());

  transport::SubscriberPtr sub = node->Subscribe(topic,
      &ShadowCasterRenderBackFacesTest::OnImage, this);

  // wait for a few images
  int i = 0;
  while (this->imageCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  for (unsigned int x = 0; x < 320 * 240; x += 600) {
    EXPECT_GT(50, this->imageBuffer[3 * x]);
    EXPECT_GT(50, this->imageBuffer[3 * x + 1]);
    EXPECT_GT(50, this->imageBuffer[3 * x + 2]);
  }

  delete this->imageBuffer;
}

/////////////////////////////////////////////////
TEST_F(ShadowCasterRenderBackFacesTest, ShadowCasterBackFaces)
{
  ShadowCasterBackFaces();
}

/////////////////////////////////////////////////
// \brief The shadow caster will not render back faces; The plane in the world
/// will not cast a shadow
void ShadowCasterRenderBackFacesTest::ShadowCasterNoBackFaces()
{
  this->Load("worlds/shadow_caster_no_back_faces.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  sensors::CameraSensorPtr sensor =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_normal"));
  EXPECT_TRUE(sensor != nullptr);
  EXPECT_TRUE(sensor->IsActive());

  unsigned int width  = 320;
  unsigned int height = 240;

  this->imageCount = 0;
  this->imageBuffer = new unsigned char[width * height*3];

  transport::NodePtr node(new transport::Node());
  node->Init();

  std::string topic = sensor->Topic();
  EXPECT_TRUE(!topic.empty());

  transport::SubscriberPtr sub = node->Subscribe(topic,
      &ShadowCasterRenderBackFacesTest::OnImage, this);

  // wait for a few images
  int i = 0;
  while (this->imageCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  for (unsigned int x = 0; x < 320 * 240; x += 600) {
    EXPECT_LT(50, this->imageBuffer[3 * x]);
    EXPECT_LT(50, this->imageBuffer[3 * x + 1]);
    EXPECT_LT(50, this->imageBuffer[3 * x + 2]);
  }

  delete this->imageBuffer;
}

/////////////////////////////////////////////////
TEST_F(ShadowCasterRenderBackFacesTest, ShadowCasterNoBackFaces)
{
  ShadowCasterNoBackFaces();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

