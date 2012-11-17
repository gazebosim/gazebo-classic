/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig
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

#ifndef _PROJECTOR_HH_
#define _PROJECTOR_HH_

#include <string>
#include <map>
#include <list>

#include "rendering/ogre_gazebo.h"

#include "msgs/msgs.hh"
#include "sdf/sdf.hh"
#include "transport/transport.hh"
#include "rendering/RenderTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Projector Projector.hh rendering/rendering.hh
    /// \brief Projects a material onto surface, light a light projector.
    class Projector
    {
      /// \brief Constructor.
      /// \param[in] _parent Name of the parent visual.
      public: Projector(VisualPtr _parent);

      /// \brief Destructor.
      public: virtual ~Projector();

      /// \brief Load from an sdf pointer.
      /// \param[in] _sdf Pointer to the SDF element.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load from a message.
      /// \param[in] _msg Load from a message.
      public: void Load(const msgs::Projector &_msg);

      /// \brief Load the projector.
      /// \param[in] _name Name of the projector.
      /// \param[in] _pos Pose of the projector.
      /// \param[in] _textureName Name of the texture to project.
      /// \param[in] _nearClip Near clip distance.
      /// \param[in] _farClip Far clip distance.
      /// \param[in] _fov Field of view.
      public: void Load(const std::string &_name,
                        const math::Pose &_pose = math::Pose(0, 0, 0, 0, 0, 0),
                        const std::string &_textureName = "",
                        double _nearClip = 0.25,
                        double _farClip = 15.0,
                        double _fov = M_PI * 0.25);

      /// \brief Load a texture into the projector.
      /// \param[in] _textureName Name of the texture to project.
      public: void SetTexture(const std::string &_textureName);

      /// \brief Toggle the activation of the projector.
      public: void Toggle();

      /// \brief Get the parent visual.
      /// \return Pointer ot the parent visual.
      public: VisualPtr GetParent();

      /// \brief Set whether the projector is enabled or disabled.
      /// \param[in] _enabled True to enable the projector.
      public: void SetEnabled(bool _enabled);

      private: void OnMsg(ConstProjectorPtr &_msg);

      private: VisualPtr visual;
      private: transport::NodePtr node;
      private: transport::SubscriberPtr controlSub;

      /// \cond
      /// \class ProjectorFrameListener Projector.hh rendering/rendering.hh
      /// \brief Frame listener, used to add projection materials when new
      /// textures are added to Ogre.
      private: class ProjectorFrameListener : public Ogre::FrameListener
      {
        /// \brief Constructor.
        public: ProjectorFrameListener();

        /// \brief Destructor.
        public: virtual ~ProjectorFrameListener();

        public: void Init(VisualPtr _visual,
                          const std::string &_textureName,
                          double _near = 0.5,
                          double _far = 10,
                          double _fov = 0.785398163);

        public: virtual bool frameStarted(const Ogre::FrameEvent &_evt);

        public: void SetTexture(const std::string &_textureName);

        public: void SetEnabled(bool _enabled);
        public: void SetUsingShaders(bool _usingShaders);

        public: void SetPose(const math::Pose &_pose);

        private: void SetSceneNode();

        private: void SetFrustumClipDistance(double _near, double _far);
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
      /// \endcond

      /// \brief The projection frame listener.
      private: ProjectorFrameListener projector;
    };
    /// \}
  }
}
#endif
