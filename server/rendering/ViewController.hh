#ifndef VIEWCONTROLLER_HH
#define VIEWCONTROLLER_HH

namespace gazebo
{
  class OgreCamera;
  class MouseEvent;

  class ViewController
  {
    /// \brief Constructor
    public: ViewController(OgreCamera *camera);

    /// \brief Destructor
    public: virtual ~ViewController();

    public: virtual void Update() = 0;

    /// \brief Handle a mouse event
    public: virtual void HandleMouseEvent(const MouseEvent &event) = 0;

    protected: OgreCamera *camera; 
  };
}
#endif
