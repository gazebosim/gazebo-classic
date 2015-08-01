

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

      public: virtual void Init() override;

      public: virtual void Load(const std::string &_worldName) override;

      protected: virtual void Fini() override;

      protected: virtual bool UpdateImpl(bool _force) override;

      protected: void OnCtrlMessage(ConstCameraLensCmdPtr &_msg);

      protected: transport::PublisherPtr lensPub;
      protected: transport::SubscriberPtr lensSub;
    };
  }
}

#endif