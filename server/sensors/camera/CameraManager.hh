#ifndef CAMERAMANAGER_HH
#define CAMERAMANAGER_HH

#include <deque>

class Camera;

class CameraManager
{
  private: CameraManager();
  private: ~CameraManager();

  public: static CameraManager *Instance();

  public: Camera *CreateCamera();
  public: Camera *GetCamera(int index);

  public: void SetActiveCamera(Camera *camera);
  public: Camera *GetActiveCamera();

  private: static CameraManager *myself;

  private: std::deque<Camera*> cameras;
  private: Camera *activeCamera;
};


#endif
