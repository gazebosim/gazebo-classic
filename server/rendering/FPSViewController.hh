#ifndef FPSVIEWCONTROLLER_HH
#define FPSVIEWCONTROLLER_HH

#include "ViewController.hh"

namespace gazebo
{
  class FPSViewController : public ViewController
  {
    /// \brief Constructor
    public: FPSViewController(OgreCamera *camera);

    /// \brief Destructor
    public: virtual ~FPSViewController();

    /// \brief Update
    public: virtual void Update();

    /// \brief Get the type name of this view controller
    public: static std::string GetTypeString() {return "FPSViewController";}

    /// \brief Handle a mouse event
    public: virtual void HandleMouseEvent(const MouseEvent &event);

  };
}
#endif
