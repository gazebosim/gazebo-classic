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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: A Light
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#ifndef LIGHT_HH
#define LIGHT_HH

#include <string>
#include <iostream>

#include "Pose3d.hh"
#include "Color.hh"
#include "Param.hh"
#include "Common.hh"

namespace Ogre
{
  class Light;
  class SceneManager;
}

namespace gazebo
{
  class Visual;
  class LightMsg;
  class XMLConfigNode;
  class OgreDynamicLines;
  class Scene;

  /// \brief Wrapper around an ogre light source
  class Light : public Common
  {
    /// \brief Constructor
    public: Light(Scene *scene);

    /// \brief Destructor
    public: virtual ~Light();

    /// \brief Load the light
    public: void Load(XMLConfigNode *node);

    /// \brief Load from a light message
    public: void LoadFromMsg(const LightMsg *msg);

    /// \brief Save a light
    public: void Save(const std::string &prefix, std::ostream &stream);

    /// \brief Set the position of the light
    public: void SetPosition(const Vector3 &p);

    /// \brief Set whether this entity has been selected by the user through  
    ///        the gui
    public: virtual bool SetSelected( bool s );

    // \brief Toggle light visual visibility
    public: void ToggleShowVisual();

    /// \brief Set whether to show the visual
    public: void ShowVisual(bool s);

    /// \brief Set the light type
    public: void SetLightType(const std::string &type);

    /// \brief Set the diffuse
    public: void SetDiffuseColor(const Color &color);

    /// \brief Set the specular color
    public: void SetSpecularColor(const Color &color);

    /// \brief Set the direction
    public: void SetDirection(const Vector3 &dir);

    /// \brief Set the attenuation
    public: void SetAttenuation(const Vector3 &att);

    /// \brief Set the spot light inner angle
    public: void SetSpotInnerAngle(const double &angle);

    /// \brief Set the spot light outter angle
    public: void SetSpotOuterAngle(const double &angle);

    /// \brief Set the spot light falloff
    public: void SetSpotFalloff(const double &angle);

    /// \brief Set the range
    public: void SetRange(const double &range);

    /// \brief Set cast shadowsj
    public: void SetCastShadows(const bool &cast);

    /// \private Helper node to create a visual representation of the light
    private: void CreateVisual();

    private: void SetupShadows();

    protected: virtual void OnPoseChange() {}

    /// The OGRE light source
    private: Ogre::Light *light;

    private: Visual *visual;
    private: OgreDynamicLines *line;

    private: ParamT<std::string> *lightTypeP;
    private: ParamT<Color> *diffuseP;
    private: ParamT<Color> *specularP;
    private: ParamT<Vector3> *directionP;
    private: ParamT<Vector3> *attenuationP;
    private: ParamT<double> *rangeP;
    private: ParamT<bool> *castShadowsP;
    private: ParamT<double> *spotInnerAngleP;
    private: ParamT<double> *spotOuterAngleP;
    private: ParamT<double> *spotFalloffP;

    private: static unsigned int lightCounter;
    private: Scene *scene;
  };
}
#endif
