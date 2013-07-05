/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/rendering/Camera.hh"

#include "gazebo/common/Animation.hh"
#include "gazebo/common/VideoMaker.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
VideoMaker::VideoMaker(const std::string &/*_name*/)
{
}

/////////////////////////////////////////////////
VideoMaker::~VideoMaker()
{
}

/////////////////////////////////////////////////
void VideoMaker::Load()
{
}

/////////////////////////////////////////////////
void VideoMaker::Load(const std::string &/*_log*/)
{
}

/////////////////////////////////////////////////
void VideoMaker::Start()
{
}

/////////////////////////////////////////////////
void VideoMaker::Pause()
{
}

/////////////////////////////////////////////////
void VideoMaker::Stop()
{
}

/////////////////////////////////////////////////
void VideoMaker::Save(const std::string &/*_filename*/)
{
}

/////////////////////////////////////////////////
void VideoMaker::Export(const std::string &/*_path*/,
    const std::string &/*_format*/)
{
}

/////////////////////////////////////////////////
void VideoMaker::Export(rendering::CameraPtr /*_camera*/,
          const std::string &/*_path*/, const std::string &/*_format*/)
{
}

/////////////////////////////////////////////////
void VideoMaker::AddCamera(rendering::CameraPtr /*_camera*/)
{
}

/////////////////////////////////////////////////
void VideoMaker::AddCamera(rendering::CameraPtr /*_camera*/,
    common::PoseAnimationPtr /*_path*/)
{
}

/////////////////////////////////////////////////
void VideoMaker::RemoveCamera(rendering::CameraPtr /*_camera*/)
{
}

/////////////////////////////////////////////////
void SetCameraDuration(rendering::CameraPtr /*_camera*/,
          double /*_start*/, double /*_duration*/)
{
}

/////////////////////////////////////////////////
double VideoMaker::GetElapsed()
{
  return 0;
}

/////////////////////////////////////////////////
double VideoMaker::GetElapsed(rendering::CameraPtr /*_camera*/)
{
  return 0;
}
