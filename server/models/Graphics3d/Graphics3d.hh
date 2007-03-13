#ifndef GRAPHICS3D_HH
#define GRAPHICS3D_HH

#include "Model.hh"

class Body;
class Geom;
class XMLConfigNode;
//class Graphics3dIface;

class Graphics3d : public Model
{ 
  public: Graphics3d();

  public: virtual ~Graphics3d();

  // Load the child model
  protected: virtual int LoadChild(XMLConfigNode *node);

  // Initialize the child model
  protected: virtual int InitChild();

  // Update the child model
  protected: virtual int UpdateChild();

  // Finilaize thie child model
  protected: virtual int FiniChild();

 // private: Graphics3dIface *iface;
};
#endif
