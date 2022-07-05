/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_RENDERING_VISUALPRIVATE_HH_
#define GAZEBO_RENDERING_VISUALPRIVATE_HH_

#include <map>
#include <string>
#include <utility>
#include <list>
#include <vector>
#include <functional>

#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/Visual.hh"
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
      /// \brief Constructor
      public: VisualPrivate()
              : sceneNode(NULL),
                transparency(0),
                castShadows(true),
                isStatic(false),
                staticGeom(NULL),
                visible(true),
                ribbonTrail(NULL),
                skeleton(NULL),
                animState(NULL),
                useRTShader(true),
                initialized(false),
                boundingBox(NULL),
                id(0),
                lighting(true),
                visibilityFlags(GZ_VISIBILITY_ALL),
                type(Visual::VT_ENTITY),
                layer(0),
                geomSize(ignition::math::Vector3d::One),
                inheritTransparency(true),
                wireframe(false)
      {
      }

      /// \brief Default destructor
      public: virtual ~VisualPrivate() = default;

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

      /// \brief True if visual casts shadows.
      public: bool castShadows;

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
      public: std::function<void()> onAnimationComplete;

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
      public: ignition::math::Vector3d scale;

      /// \brief True if lighting will be applied to this visual.
      public: bool lighting;

      /// \brief A list of visual plugins.
      public: std::vector<VisualPluginPtr> plugins;

      /// \brief The visual's mesh name.
      public: std::string meshName;

      /// \brief The visual's submesh name.
      public: std::string subMeshName;

      /// \brief Ambient color of the visual.
      public: ignition::math::Color ambient = ignition::math::Color(0, 0, 0, 0);

      /// \brief Diffuse color of the visual.
      public: ignition::math::Color diffuse = ignition::math::Color(0, 0, 0, 0);

      /// \brief Specular color of the visual.
      public: ignition::math::Color specular =
          ignition::math::Color(0, 0, 0, 0);

      /// \brief Specular exponent of the visual.
      public: double shininess = 0;

      /// \brief Emissive color of the visual.
      public: ignition::math::Color emissive =
          ignition::math::Color(0, 0, 0, 0);

      /// \brief Visibility flags of the visual.
      public: uint32_t visibilityFlags;

      /// \brief type
      public: Visual::VisualType type;

      /// \brief Index of the layer to which this visual belongs. Layers
      /// act similar to layers in photoshop.
      public: int32_t layer;

      /// \brief Size of attached geometry
      public: ignition::math::Vector3d geomSize;

      /// \brief True to inherit transparency from parent.
      public: bool inheritTransparency;

      /// \brief True if wireframe mode is enabled
      public: bool wireframe;

      /// \brief Stores the message for this visual according to the visual
      /// type. For example, VT_LINK will have gazebo::msgs::Link.
      public: google::protobuf::Message *typeMsg = nullptr;

      /// \brief Vector of visuals which will be generated on demand.
      public: std::vector<std::pair<Visual::VisualType,
          google::protobuf::Message *>> pendingChildren;

      /// \brief The initial pose of the visual.
      public: ignition::math::Pose3d initialRelativePose;

      /// \brief Original ogre materials used by the submeshes in the visual
      public: std::map<std::string, Ogre::MaterialPtr> submeshMaterials;
    };
    /// \}
  }
}
#endif
