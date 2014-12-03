#ifndef _COPYPASTEMANIP_HH_
#define _COPYPASTEMANIP_HH_

#include <gazebo/common/MouseEvent.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gui/Events.hh>
#include <gazebo/gui/EntityMaker.hh>

class CopyPasteManip
{
  public: CopyPasteManip(rendering::UserCameraPtr _userCamera,
                         common::MouseEventPtr _mouseEvent);
  public: void Paste(EntityMaker *entityMaker);

  private: rendering::UserCameraPtr userCamera;
  private: common::MouseEventPtr mouseEvent;
};

#endif
