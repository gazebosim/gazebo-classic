#ifndef LIGHTSOURCE_HH
#define LIGHTSOURCE_HH

#include "Model.hh"

class Body;
class Geom;
class XMLConfigNode;

class LightSource : public Model
{ 
  public: LightSource();

  public: virtual ~LightSource();

  // Load the child model
  protected: virtual int LoadChild(XMLConfigNode *node);

  // Initialize the child model
  protected: virtual int InitChild();

  // Update the child model
  protected: virtual int UpdateChild();

  // Finilaize thie child model
  protected: virtual int FiniChild();

};
#endif
