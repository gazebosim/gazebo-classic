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

namespace gazebo
{
  namespace rendering
  {
    /// \brief A Bumper controller
    class Projector
    {
      /// \brief Constructor
      public: Projector();

      /// \brief Destructor
      public: ~Projector();

      /// \brief Load the plugin
      /// \param take in SDF root element
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load a texture into the projector
      public: void LoadImage(const std::string &_textureName);

      /// \brief Toggle the activation of the projector
      public: void ToggleProjector(bool _projectorOn);

      private: double fov;
      private: double nearClip, farClip;

      private: class Projector : public Ogre::FrameListener
               {
               };
    };
  }
}

#endif
