

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

      protected: transport::PublisherPtr projPub;
    };
  }
}

#endif