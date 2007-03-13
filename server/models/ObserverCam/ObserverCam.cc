#include <stdio.h>

#include "Global.hh"
#include "ModelFactory.hh"
#include "XMLConfig.hh"
#include "CameraManager.hh"
#include "Camera.hh"
#include "ObserverCam.hh"

GZ_REGISTER_STATIC("ObserverCam", ObserverCam);

ObserverCam::ObserverCam()
{
}

ObserverCam::~ObserverCam()
{
}

// Load the child model
int ObserverCam::LoadChild(XMLConfigNode *node)
{
  int imageSizeX, imageSizeY;
  double nearClip, farClip, hfov;

  // Create a new camera
  this->camera = CameraManager::Instance()->CreateCamera();

  // TODO: Fix so the camera sets whether it's active
  CameraManager::Instance()->SetActiveCamera(this->camera);

  // Get the image size
  imageSizeX = node->GetTupleInt("imageSize",0,640);
  imageSizeY = node->GetTupleInt("imageSize",1,480);

  // Near clip and far clip planes
  nearClip = node->GetDouble("nearClip",1.0);
  farClip = node->GetDouble("farClip",100.0);

  // Horizontal field of view
  hfov = node->GetDouble("hfov",60.0);

  //camera->Init(imageSizeX, imageSizeY, DTOR(hfov), nearClip, farClip, 16);
  camera->Init(640, 480, 60*M_PI/180.0, 1, 100, 16);

  return 0;
}

// Initialize the child model
int ObserverCam::InitChild()
{
  return 0;
}

// Update the child model
int ObserverCam::UpdateChild()
{
  return 0;
}

// Finilaize thie child model
int ObserverCam::FiniChild()
{
  return 0;
}
