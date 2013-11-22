#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class OculusWindow OculusWindow.hh gui/OculusWindow.hh
    /// \brief A widget that renders a camera view suitable for the Oculus
    /// Rift.
    class OculusWindow : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: OculusWindow(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~OculusWindow();

      protected: virtual void showEvent(QShowEvent *_e);
      protected: virtual void resizeEvent(QResizeEvent *_e);

      protected: void keyPressEvent(QKeyEvent *_event);

      private: std::string GetOgreHandle() const;

      private: QFrame *renderFrame;

      private: int windowId;

      private: rendering::OculusCameraPtr oculusCamera;
      private: rendering::ScenePtr scene;

      private: bool isFullScreen;
    };
  }
}
