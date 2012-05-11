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
 * Desc: Projector
 * Author: Jared Duke, (some maintainence by John Hsu)
 */

#ifndef __PROJECTOR_HH__
#define __PROJECTOR_HH__

#include <string>
#include <map>
#include <list>

#include "sdf/sdf.h"
#include "rendering/ogre.h"

#include "rendering/RenderTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief A Bumper controller
    class Projector
    {
      /// \brief Constructor
      public: Projector(VisualPtr _parent);

      /// \brief Destructor
      public: virtual ~Projector();

      /// \brief Load from an sdf pointer
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load from a message
      public: void Load(const msgs::Projector &_msg);

      public: void Load(const math::Pose &_pose = math::Pose(0, 0, 0, 0, 0, 0),
                     const std::string &_textureName = "",
                     double _nearClip = 0.25,
                     double _farClip = 15.0,
                     double _fov = M_PI * 0.25);

      /// \brief Load a texture into the projector
      public: void SetTexture(const std::string &_textureName);

      /// \brief Toggle the activation of the projector
      public: void Toggle();

      private: VisualPtr visual;

      private: class ProjectorFrameListener : public Ogre::FrameListener
      {
        public: ProjectorFrameListener();
        public: virtual ~ProjectorFrameListener();
        public: void Init(VisualPtr _visual,
                          const std::string &_textureName,
                          double _near = 0.5,
                          double _far = 10,
                          double _fov = 0.785398163);

        public: bool frameStarted(const Ogre::FrameEvent &_evt);

        public: void SetTexture(const std::string &_textureName);

        public: void SetEnabled(bool _enabled);
        public: void SetUsingShaders(bool _usingShaders);

        public: void SetPose(const math::Pose &_pose);

        private: void SetSceneNode();

        private: void SetFrustumClipDistance(double _near,
                                             double _far);
        private: void SetFrustumFOV(double _fov);
        private: void AddPassToAllMaterials();
        private: void AddPassToVisibleMaterials();
        private: void AddPassToMaterials(std::list<std::string> &_matList);
        private: void AddPassToMaterial(const std::string &_matName);
        private: void RemovePassFromMaterials();
        private: void RemovePassFromMaterial(const std::string &_matName);

        public: bool enabled;
        public:  bool initialized;
        private: bool usingShaders;

        private: std::string nodeName;
        private: std::string filterNodeName;

        private: std::string textureName;

        private: Ogre::Frustum *frustum;
        private: Ogre::Frustum *filterFrustum;
        private: Ogre::PlaneBoundedVolumeListSceneQuery *projectorQuery;

        private: VisualPtr visual;

        private: Ogre::SceneNode *node;
        private: Ogre::SceneNode *filterNode;
        private: Ogre::SceneManager *sceneMgr;
        private: std::map<std::string, Ogre::Pass*> projectorTargets;
      };

      private: ProjectorFrameListener projector;
    };
  }
}

#endif
