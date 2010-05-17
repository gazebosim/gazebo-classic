#ifndef ORBITVIEWCONTROLLER_HH
#define ORBITVIEWCONTROLLER_HH

#include "ViewController.hh"
#include "Vector3.hh"

namespace gazebo
{
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

    private: float yaw, pitch;
    private: float distance;
    private: Vector3 focalPoint;
  };
}

#endif
