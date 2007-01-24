#ifndef GROUNDPLANE_HH
#define GROUNDPLANE_HH

#include "Model.hh"

class Body;
class Geom;
class XMLConfigNode;

class GroundPlane : public Model
{ 
  public: GroundPlane();

  public: virtual ~GroundPlane();

  // Load the child model
  protected: virtual int LoadChild(XMLConfigNode *node);

  // Initialize the child model
  protected: virtual int InitChild();

  // Update the child model
  protected: virtual int UpdateChild();

  // Finilaize thie child model
  protected: virtual int FiniChild();

  private: Body *body;
  private: Geom *geom;

};
#endif
