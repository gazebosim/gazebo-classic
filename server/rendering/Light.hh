/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

/* Desc: A Light
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#ifndef LIGHT_HH
#define LIGHT_HH

#include <string>
#include <iostream>

#include "Event.hh"
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
    //public: void LoadFromMsg(const LightMsg *msg);

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

    private: ConnectionPtr showLightsConnection;
    private: static unsigned int lightCounter;
    private: Scene *scene;
  };
}
#endif
