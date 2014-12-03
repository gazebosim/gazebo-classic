#ifndef _COPYPASTEMANIP_HH_
#define _COPYPASTEMANIP_HH_

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/EntityMaker.hh"

namespace gazebo
{
  namespace gui
  {
    class CopyPasteManip
    {
      public: CopyPasteManip(rendering::UserCameraPtr _userCamera,
                             common::MouseEvent *_mouseEvent);
      public: void Paste(EntityMaker *entityMaker);

      private: rendering::UserCameraPtr userCamera;
      private: common::MouseEvent *mouseEvent;
    };
  }
}

#endif
