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

#include "gazebo/common/ffmpeg_inc.h"

#ifdef HAVE_FFMPEG
using namespace gazebo;

//////////////////////////////////////////////////
AVFrame *common::AVFrameAlloc(void)
{
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55, 28, 1)
  return av_frame_alloc();
#else
  return avcodec_alloc_frame();
#endif
}

//////////////////////////////////////////////////
void common::AVFrameUnref(AVFrame *_frame)
{
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55, 28, 1)
  av_frame_unref(_frame);
#else
  avcodec_get_frame_defaults(_frame);
#endif
}

//////////////////////////////////////////////////
void common::AVPacketUnref(AVPacket *_packet)
{
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 24, 102)
  av_packet_unref(_packet);
#else
  av_free_packet(_packet);
#endif
}

// ifdef HAVE_FFMPEG
#endif
