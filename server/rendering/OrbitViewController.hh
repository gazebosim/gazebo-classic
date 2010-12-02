#ifndef ORBITVIEWCONTROLLER_HH
#define ORBITVIEWCONTROLLER_HH

#include "ViewController.hh"
#include "Vector3.hh"

namespace gazebo
{
  class OgreVisual;

  class OrbitViewController : public ViewController
  {
    /// \brief Constructor
    public: OrbitViewController(OgreCamera *camera);

    /// \brief Destructor
    public: virtual ~OrbitViewController();

    /// \brief Update
    public: virtual void Update();

    /// \brief Handle a mouse event
    public: virtual void HandleMouseEvent(const MouseEvent &event);

    /// \brief Get the type name of this view controller
    public: static std::string GetTypeString() {return "OrbitViewController";}

    /// \brief Translate the focal point
    private: void Translate(Vector3 vec);

    /// \brief Normalize yaw value
    private: void NormalizeYaw(float &v);

    /// \brief Normalize pitch value
    private: void NormalizePitch(float &v);

    private: float yaw, pitch;
    private: float distance;
    private: Vector3 focalPoint;

    private: OgreVisual *refVisual;
  };
}

#endif
