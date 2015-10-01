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

#ifndef _VISUAL_PRIVATE_HH_
#define _VISUAL_PRIVATE_HH_

#include <string>
#include <utility>
#include <list>
#include <vector>

#include <boost/function.hpp>
#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/math/Box.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Vector2d.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/common/CommonTypes.hh"

namespace Ogre
{
  class MovableObject;
  class SceneNode;
  class StaticGeometry;
  class RibbonTrail;
  class AnimationState;
  class SkeletonInstance;
}

namespace gazebo
{
  namespace rendering
  {
    class WireBox;

    /// \brief Private data for the Visual class
    class VisualPrivate
    {
      /// \brief Pointer to the visual's scene.
      public: ScenePtr scene;

      /// \brief Pointer to the visual's scene node in Ogre.
      public: Ogre::SceneNode *sceneNode;

      /// \brief Parent visual.
      public: VisualPtr parent;

      /// \brief The SDF element for the visual.
      public: sdf::ElementPtr sdf;

      /// \brief The unique name for the visual's material.
      public: std::string myMaterialName;

      /// \brief The original name for the visual's material.
      public: std::string origMaterialName;

      /// \brief Transparency value.
      public: float transparency;

      /// \brief True if the visual is static, which allows Ogre to improve
      /// performance.
      public: bool isStatic;

      /// \brief Pointer to the static geometry.
      public: Ogre::StaticGeometry *staticGeom;

      /// \brief True if rendered.
      public: bool visible;

      /// \brief The ribbon train created by the visual.
      public: Ogre::RibbonTrail *ribbonTrail;

      /// \brief The visual's skeleton, used only for person simulation.
      public: Ogre::SkeletonInstance *skeleton;

      /// \brief Connection for the pre render event.
      public: event::ConnectionPtr preRenderConnection;

      /// \brief List of all the lines created.
      public: std::list<DynamicLines*> lines;

      /// \brief Lines and their vertices connected to this visual.
      public: std::list< std::pair<DynamicLines*, unsigned int> > lineVertices;

      /// \brief Name of the visual.
      public: std::string name;

      /// \brief Children visuals.
      public: std::vector<VisualPtr> children;

      /// \brief Used to animate the visual.
      public: Ogre::AnimationState *animState;

      /// \brief Time of the previous animation step.
      public: common::Time prevAnimTime;

      /// \brief Callback for the animation complete event.
      public: boost::function<void()> onAnimationComplete;

      /// \brief True to use RT shader system.
      public: bool useRTShader;

      /// \brief True if initialized.
      public: bool initialized;

      /// \brief A wire frame bounding box.
      public: WireBox *boundingBox;

      /// \brief Unique id of this visual.
      public: uint32_t id;

      /// \brief Counter used to create unique ids.
      public: static uint32_t visualIdCount;

      /// \brief Scale of visual.
      public: math::Vector3 scale;

      /// \brief True if lighting will be applied to this visual.
      public: bool lighting;

      /// \brief A list of visual plugins.
      public: std::vector<VisualPluginPtr> plugins;
    };
    /// \}
  }
}
#endif
