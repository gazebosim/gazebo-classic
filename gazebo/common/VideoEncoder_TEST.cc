/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/VideoEncoder.hh"
#include "test/util.hh"

using namespace gazebo;
using namespace common;

class VideoEncoderTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(VideoEncoderTest, StartStop)
{
  VideoEncoder video;
  EXPECT_FALSE(video.IsEncoding());
  EXPECT_STREQ(video.Format().c_str(), VIDEO_ENCODER_FORMAT_DEFAULT);
  EXPECT_EQ(video.BitRate(), static_cast<unsigned int>(
        VIDEO_ENCODER_BITRATE_DEFAULT));

#ifdef HAVE_FFMPEG
  video.Start();
  EXPECT_TRUE(video.IsEncoding());
  EXPECT_TRUE(common::exists(common::cwd() + "/TMP_RECORDING.mp4"));
  EXPECT_EQ(video.BitRate(), 920000u);

  video.Stop();
  EXPECT_FALSE(video.IsEncoding());
  EXPECT_FALSE(common::exists(common::cwd() + "/TMP_RECORDING.ogv"));

  video.Start(1024, 768, "ogv");
  EXPECT_TRUE(video.IsEncoding());
  EXPECT_STREQ(video.Format().c_str(), "ogv");
  EXPECT_TRUE(common::exists(common::cwd() + "/TMP_RECORDING.ogv"));

  video.Start(1024, 768, "mp4");
  EXPECT_TRUE(video.IsEncoding());
  EXPECT_STREQ(video.Format().c_str(), "ogv");

  video.Stop();
  EXPECT_FALSE(video.IsEncoding());
#endif
}

/////////////////////////////////////////////////
TEST_F(VideoEncoderTest, Exists)
{
  VideoEncoder video;
  EXPECT_FALSE(video.IsEncoding());
  EXPECT_STREQ(video.Format().c_str(), VIDEO_ENCODER_FORMAT_DEFAULT);

  EXPECT_FALSE(common::exists(common::cwd() + "/TMP_RECORDING.ogv"));
  EXPECT_FALSE(common::exists(common::cwd() + "/TMP_RECORDING.mp4"));

#ifdef HAVE_FFMPEG
  video.Start();
  EXPECT_TRUE(common::exists(common::cwd() + "/TMP_RECORDING.mp4"));

  video.Reset();
  EXPECT_FALSE(common::exists(common::cwd() + "/TMP_RECORDING.mp4"));
#endif
}
