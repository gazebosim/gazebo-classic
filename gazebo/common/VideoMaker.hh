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
#ifndef _VIDEOMAKER_HH_
#define _VIDEOMAKER_HH_

#include <string>

#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace common
  {
    class Animation;

    /// \addtogroup gazebo_common
    /// \{

    /// \class VideoMaker VideoMaker.hh
    /// \brief Tool for making choreographed videos
    class VideoMaker
    {

      /// \brief Constructor
      public: VideoMaker(const std::string &_name);

      /// \brief Deconstructor
      public: virtual ~VideoMaker();

      /// \brief Load
      public: void Load();

      /// \brief Load a log file
      public: void Load(const std::string &_log);

      /// \brief Start the video making process.
      public: void Start();

      /// \brief Pause the video making process.
      public: void Pause();

      /// \brief Stop the video making process. All camera recordings will also
      // be stopped.
      public: void Stop();

      /// \brief Save the current video maker setup and configurations to file.
      /// \param[in] _filename Filename to save to.
      public: void Save(const std::string &_filename);

      /// \brief Export videos recorded by all cameras.
      /// \param[in] _path Path where the videos will be exported to.
      /// \param[in] _format Format of the exported videos.
      public: void Export(const std::string &_path, const std::string &_format);

      /// \brief Export video recording of a given camera.
      /// \param[in] _camera Pointer to camera.
      /// \param[in] _path Path where the videos will be exported to.
      /// \param[in] _format Format of the exported videos.
      public: void Export(rendering::CameraPtr _camera,
          const std::string &_path, const std::string &_format);

      /// \brief Add camera to video maker. Camera movement will be based on
      /// its camera mode (i.e. fixed, follow, track object). A video stream
      /// will be created for this camera.
      /// \param[in] _camera Pointer to camera
      public: void AddCamera(rendering::CameraPtr _camera);

      /// \brief Add camera to video maker and associate it with a
      /// keyframe-enabled animiated path. A video stream will be created for
      /// this camera.
      /// \param[in] _camera Pointer to the camera.
      /// \param[in] _path Pointer to animated path.
      public: void AddCamera(rendering::CameraPtr _camera,
          common::Animation *_path);

      /// \brief Remove a camera from the video maker. The recorded video stream
      /// associated with this camera will also be deleted.
      public: void RemoveCamera(rendering::CameraPtr _camera);

      /// \brief Set camera duration.
      /// \param[in] _camera Pointer to the camera.
      /// \param[in] _start Start time in sim time seconds.
      /// \param[in] _duration in sim time seconds.
      public: void SetCameraDuration(rendering::CameraPtr _camera,
          double _start, double _duration);

      /// \brief Get elapsed time of the video recording
      /// \return Sim time elapsed since starting the video maker
      public: double GetElapsed();

      /// \brief Get elapsed time of a camera recording
      /// \param[in] _camera Pointer to the camera
      /// \return Sim time elapsed since the start of the camera recording
      public: double GetElapsed(rendering::CameraPtr _camera);
    };
  }
}

#endif
