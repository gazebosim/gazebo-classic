/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Contact Plugin
 * Author: Nate Koenig mod by John Hsu
 */

#ifndef GAZEBO_PROJECTOR_PLUGIN_HH
#define GAZEBO_PROJECTOR_PLUGIN_HH

#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreFrameListener.h>

#include <map>
#include <string>
#include <list>

#include "common/Plugin.hh"
#include "sensors/SensorTypes.hh"
#include "rendering/RenderTypes.hh"
#include "gazebo.h"

namespace Ogre
{
  class PlaneBoundedVolumeListSceneQuery;
  class Frustum;
  class Pass;
  class SceneNode;
}
namespace gazebo
{
  class ProjectorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model
    public: ProjectorPlugin();

    /// \brief Destructor
    public: virtual ~ProjectorPlugin();

    /// \brief Load the controller
    /// \param node XML config node
    protected: virtual void Load(physics::ModelPtr _parent,
      sdf::ElementPtr _sdf);

    /// \brief pointer to the world
    private: physics::WorldPtr world;

    /// \brief pointer to the world scene
    private: rendering::ScenePtr scene;

    /// \brief The parent Model
    private: physics::ModelPtr myParent;

    /// \brief The parent Model
    private: physics::LinkPtr myBody;  // Gazebo/ODE body

    rendering::VisualPtr myVisual;
    Ogre::SceneNode* mySceneNode;

    /// \brief Callback when a texture is published
    private: void LoadImage(const std::string &_texture_name);

    /// \brief Callbakc when a projector toggle is published
    private: void ToggleProjector(bool _projectorOn);

    /// \brief Utility method for accessing the root Ogre object
    private: Ogre::Root *getRootP();

    /// \brief A mutex to lock access to fields that are used in
    ///        plugin callbacks
    private: boost::mutex lock;

    // \brief Projector parameters
    private: std::string bodyName;
    private: std::string textureName;
    private: std::string filterTextureName;
    private: double fov;
    private: double nearClipDist;
    private: double farClipDist;
    private: math::Vector3 xyz;
    private: math::Quaternion rpy;

    private: std::string projectorNodeName;
    private: std::string projectorFilterNodeName;

    private: void UpdateShaders();
    private: event::ConnectionPtr add_model_event_;

    private:
      class Projector : public Ogre::FrameListener
      {
        public: Projector();
        public: virtual ~Projector();

        public: void init(Ogre::SceneNode *sceneNodePtr = NULL,
                          Ogre::SceneManager *sceneMgrPtr = NULL,
                          Ogre::String textureName =
                            "stereo_projection_pattern_alpha.png",
                          Ogre::String filterTextureName =
                            "stereo_projection_pattern_filter.png",
                          double nearDist = .5,
                          double farDist = 10,
                          double fov = 0.785398163,
                          std::string projectorNodeName =
                            "projectorNodeName",
                          std::string projectorFilterNodeName =
                            "projectorFilterNodeName");

        public: virtual bool frameStarted(const Ogre::FrameEvent &evt);
        public: virtual bool frameEnded(const Ogre::FrameEvent &evt);
        public: virtual bool frameRenderingQueued(const Ogre::FrameEvent &evt);

        public: void setEnabled(bool enabled);
        public: void setUsingShaders(bool usingShaders);
        public: void setSceneNode();
        public: void setTextureName(const Ogre::String& textureName);
        public: void setFilterTextureName(const Ogre::String& textureName);
        public: void setFrustumClipDistance(double nearDist, double farDist);
        public: void setFrustumFOV(double fovInRadians);
        public: void setPose(math::Vector3 xyz, math::Quaternion rpy);

        private: void addProjectorPassToVisibleMaterials();
        private: void addProjectorPassToAllMaterials();
        private: void addProjectorPassToMaterials(
                        std::list<std::string>& matList);
        private: void addProjectorPassToMaterial(std::string matName);
        private: void removeProjectorPassFromMaterials();
        private: void removeProjectorPassFromMaterial(std::string matName);

        private: Ogre::SceneNode* parentSceneNode;

        private: bool isEnabled;
        public:  bool isInit;
        private: bool isUsingShaders;

        private: Ogre::Frustum *projectorFrustum;
        private: Ogre::Frustum *projectorFilterFrustum;
        private: Ogre::PlaneBoundedVolumeListSceneQuery *projectorQuery;
        private: Ogre::SceneNode *projectorNode;
        private: Ogre::SceneNode *projectorFilterNode;
        private: Ogre::SceneManager *sceneMgr;

        private: Ogre::String projectedTextureName;
        private: Ogre::String projectedFilterTextureName;

        private: std::map<std::string, Ogre::Pass*> projectorTargets;

        private: std::string projectorNodeName;
        private: std::string projectorFilterNodeName;
      };

    private: Projector projector_;
  };
}

#endif

