#ifndef OBSERVERCAM_HH
#define OBSERVERCAM_HH

#include "Model.hh"

namespace gazebo
{
class Camera;
class XMLConfigNode;

class ObserverCam : public Model
{ 
  public: ObserverCam();

  public: virtual ~ObserverCam();

  // Load the child model
  protected: virtual int LoadChild(XMLConfigNode *node);

  // Initialize the child model
  protected: virtual int InitChild();

  // Update the child model
  protected: virtual int UpdateChild();

  // Finilaize thie child model
  protected: virtual int FiniChild();

  private: Camera *camera;
};
}
#endif
