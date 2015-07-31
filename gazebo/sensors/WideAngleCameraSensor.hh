

#ifndef _WIDEANGLECAMERASENSOR_HH_
#define _WIDEANGLECAMERASENSOR_HH_

#include "CameraSensor.hh"

namespace gazebo
{
  namespace sensors
  {
    class WideAngleCameraSensor : public CameraSensor
    {
      public: WideAngleCameraSensor();

      public: virtual void Init();

      public: virtual void Load(const std::string &_worldName);

      protected: virtual bool UpdateImpl(bool _force) override;

      protected: void OnCtrlMessage(ConstCameraProjectionCmdPtr &_msg);

      protected: transport::PublisherPtr projPub;
      protected: transport::SubscriberPtr projSub;
    };
  }
}

#endif