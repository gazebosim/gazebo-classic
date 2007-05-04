#include "Camera.hh"
#include "CameraManager.hh"

using namespace gazebo;

CameraManager *CameraManager::myself = NULL;

CameraManager::CameraManager()
{
  this->activeCamera = NULL;
}

CameraManager::~CameraManager()
{}

CameraManager *CameraManager::Instance()
{
  if (!myself)
  {
    myself = new CameraManager();
  }

  return myself;
}

Camera *CameraManager::CreateCamera()
{
  Camera *newCamera = new Camera();

  this->cameras.push_back(newCamera);

  return newCamera;
}

Camera *CameraManager::GetCamera(int index)
{
  return this->cameras[index];
}

void CameraManager::SetActiveCamera(Camera *camera)
{
  this->activeCamera = camera;
}

Camera *CameraManager::GetActiveCamera()
{
  return this->activeCamera;
}
