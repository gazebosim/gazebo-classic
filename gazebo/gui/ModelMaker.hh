/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_MODEL_MAKER_HH_
#define _GAZEBO_MODEL_MAKER_HH_

#include <list>
#include <string>
#include <sdf/sdf.hh>

#include "gazebo/gui/EntityMaker.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

  namespace gui
  {
    /// \brief
    class GAZEBO_VISIBLE ModelMaker : public EntityMaker
    {
      /// \brief Constructor
      public: ModelMaker();

      /// \brief Destructor
      public: virtual ~ModelMaker();

      /// \brief Initialize the model maker with an existing model
      /// \param[in] _modelName Name of existing model in the scene.
      /// \return True if initialization is successful.
      public: bool InitFromModel(const std::string &_modelName);

      public: bool InitFromSDFString(const std::string &_data);
      public: bool InitFromFile(const std::string &_filename);

      public: virtual void Start(const rendering::UserCameraPtr _camera);

      public: virtual void Stop();
      public: virtual bool IsActive() const;

      public: virtual void OnMouseMove(const common::MouseEvent &_event);

      /// \brief Internal init function.
      private: bool Init();

      private: virtual void CreateTheEntity();
      private: int state;
      private: bool leftMousePressed;
      private: math::Vector2i mousePushPos, mouseReleasePos;

      private: rendering::VisualPtr modelVisual;
      private: std::list<rendering::VisualPtr> visuals;
      private: sdf::SDFPtr modelSDF;

      private: bool clone;
    };
  }
}
#endif
