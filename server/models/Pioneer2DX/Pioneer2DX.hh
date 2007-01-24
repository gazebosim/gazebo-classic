#ifndef PIONEER2DX_HH
#define PIONEER2DX_HH

#include "Model.hh"

class Body;
class Geom;
class HingeJoint;
class XMLConfigNode;

typedef struct gz_position gz_position_t;
typedef struct gz_power gz_power_t;
typedef struct gz_sonar gz_sonar_t;


class Pioneer2DX : public Model
{ 
  public: Pioneer2DX();

  public: virtual ~Pioneer2DX();

  // Load the child model
  protected: virtual int LoadChild(XMLConfigNode *node);

  // Load the Bodies for the Pioneer
  private: int LoadBodies(XMLConfigNode *node);

  // Load the sonars
  private: int LoadSonar(XMLConfigNode *node);

  // Initialize the child model
  protected: virtual int InitChild();

  // Update the child model
  protected: virtual int UpdateChild();

  // Update parameters
  private: double updatePeriod, updateTime;

  // Finilaize thie child model
  protected: virtual int FiniChild();

  // Body and geometry for the chassis
  private: Body *chassisBody;
  private: Geom *chassisGeom;

  // Body, geom, and joints for the wheels
  private: Body *wheelBodies[2];
  private: Geom *wheelGeoms[2];
  private: HingeJoint *wheelJoints[2];

  // Radius of the drive wheels
  private: double wheelRadius;

  // Wheel separation distance
  private: double wheelSep;

  // Wheel speeds (left and right wheels are paired)
  private: int enableMotors;
  private: double wheelSpeed[2];

  // Odometric pose estimate
  private: double odomPose[3];

  // Odometric velocity estimate
  private: double odomVel[3];

  // Battery level
  private: double batteryLevel;

  // Coefficients of battery discharge curve
  private: double batteryCurve[2];

  // External interface
  private: gz_position_t *position_iface;
  private: gz_power_t *power_iface;
  private: gz_sonar_t *sonar_iface;


};
#endif
