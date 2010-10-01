#ifndef LIGHT_HH
#define LIGHT_HH

#include <string>
#include <iostream>

#include "Color.hh"
#include "Param.hh"
#include "Entity.hh"

namespace Ogre
{
  class Light;
  class SceneManager;
}

namespace gazebo
{
  class OgreVisual;
  class XMLConfigNode;
  class OgreDynamicLines;
  class Scene;

  /// \brief Wrapper around an ogre light source
  class Light : public Entity
  {
    /// \brief Constructor
    public: Light(Entity *parent, unsigned int sceneIndex);

    /// \brief Destructor
    public: virtual ~Light();

    /// \brief Load the light
    public: void Load(XMLConfigNode *node);

    /// \brief Save a light
    public: void Save(const std::string &prefix, std::ostream &stream);

    /// \brief Set whether this entity has been selected by the user through  
    ///        the gui
    public: virtual bool SetSelected( bool s );

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
    public: void SetSpotOutterAngle(const double &angle);

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

    /// Parent of the light
    private: OgreVisual *visual;
    private: OgreDynamicLines *line;

    private: ParamT<std::string> *lightTypeP;
    private: ParamT<Color> *diffuseP;
    private: ParamT<Color> *specularP;
    private: ParamT<Vector3> *directionP;
    private: ParamT<Vector3> *attenuationP;
    private: ParamT<double> *rangeP;
    private: ParamT<bool> *castShadowsP;
    private: ParamT<double> *spotInnerAngleP;
    private: ParamT<double> *spotOutterAngleP;
    private: ParamT<double> *spotFalloffP;

    private: static unsigned int lightCounter;
    private: Scene *scene;
  };
}
#endif
