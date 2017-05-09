/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_MODEL_MAKER_PRIVATE_HH_
#define _GAZEBO_MODEL_MAKER_PRIVATE_HH_

#include <list>

#include "gazebo/gui/EntityMakerPrivate.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the ModelMaker class
    class ModelMakerPrivate : public EntityMakerPrivate
    {
      /// \brief The model visual being created.
      public: rendering::VisualPtr modelVisual;

      /// \brief A list of model visuals created by the model maker.
      public: std::list<rendering::VisualWeakPtr> visuals;

      /// \brief The SDF representation of the model.
      public: sdf::SDFPtr modelSDF;

      /// \brief True if the model is being created as a clone of an existing
      /// model.
      public: bool clone;

      /// \brief Publisher for factory messages.
      public: transport::PublisherPtr makerPub;
    };
  }
}
#endif
