/*****************************************************************************

 rimless-wheel example in ODE
 Evan Drumwright, 2015
 
 generates files cvio.dat (recorded constraint violation) and energy.dat
 (kinetic, potential, and total energy)

 requires environment variable RIMLESS_WHEEL_THETAD to be set to initial
 angular speed

******************************************************************************/

#include <cmath>
#include <limits>
#include <cstdlib>
#include <fstream>
#include <ode/ode.h>
#include <ode/odemath.h>
#include <ode/matrix.h>
#include <iostream>

dWorldID world;
dBodyID wheel;
unsigned iterations;
dJointGroupID contact_group;
const double RADIUS = 1.0;
const double ASQ = 10.0*10.0;
const double BSQ = 10.0*10.0;
const double MASS = 1.0;
const unsigned N_SPOKES = 6; 

void init()
{
  // init the world
  world = dWorldCreate();

  // set gravity is downhill already
  dWorldSetGravity(world, 0.099833, 0, -.995);

  // disable auto-disable
  dWorldSetAutoDisableFlag(world, 0);

  // create the contact group
  contact_group = dJointGroupCreate(0);

  // create the wheel
  wheel = dBodyCreate(world);

  // set the position and orientation of the wheel
  dBodySetPosition(wheel, 0.0, 0.0, 0.866025403784439);
  const double THETA = 0.0;
  dMatrix3 R;
  dACCESS33(R, 0, 0) = std::cos(THETA);
  dACCESS33(R, 0, 1) = 0.0;
  dACCESS33(R, 0, 2) = std::sin(THETA);
  dACCESS33(R, 1, 0) = 0.0;
  dACCESS33(R, 1, 1) = 1.0;
  dACCESS33(R, 1, 2) = 0.0;
  dACCESS33(R, 2, 0) = -std::sin(THETA);
  dACCESS33(R, 2, 1) = 0.0;
  dACCESS33(R, 2, 2) = std::cos(THETA);
  dBodySetRotation(wheel, R);

  // get and set the initial velocity
  char* theta_dot_str = getenv("RIMLESS_WHEEL_THETAD");
  if (!theta_dot_str) 
  {
    std::cerr << "RIMLESS_WHEEL_THETAD not defined!" << std::endl;
    exit(-1);
  }
  dBodySetAngularVel(wheel, 0.0, std::atof(theta_dot_str), 0.0);

  // set the mass
  dMass m;
  m.mass = (dReal) MASS;
  m.c[0] = m.c[1] = m.c[2] = 0.0;
  for (unsigned i=0; i< 3; i++)
    for (unsigned j=0; j< 3; j++)
      dACCESS33(m.I, i, j) = 0.0;
  dACCESS33(m.I, 0, 0) = 2.0;
  dACCESS33(m.I, 1, 1) = 1.0;
  dACCESS33(m.I, 2, 2) = 2.0;
  dBodySetMass(wheel, &m);
}

void step(double t, double step_size)
{
  // get position and orientation of the wheel 
  const dReal* pos = dBodyGetPosition(wheel);
  const dReal* R = dBodyGetRotation(wheel);

  // setup constraint violation
  double cvio = std::numeric_limits<double>::max();

  // loop over each spoke
  for (unsigned i=0; i< N_SPOKES; i++)
  {
    // get the value of theta
    double theta = M_PI * i * 2.0 / N_SPOKES;

    // setup the point of each spoke
    dVector3 p;
    p[0] = std::cos(theta)*RADIUS;
    p[1] = 0.0;
    p[2] = std::sin(theta)*RADIUS;
    dVector3 q;

    // transform it to the global frame
    dMultiply0(q, R, p, 3, 4, 1);
    q[0] += pos[0];
    q[1] += pos[1];
    q[2] += pos[2];

    // record constraint violation
    cvio = std::min(cvio, (double) q[2]);

    // only setup contact if position is at/below ground plane
    if (q[2] <= 0.0)
    {
      dContact c[1];
      c[0].surface.mu = 1000000.0;
      c[0].surface.mu2 = 1000000.0;
      c[0].surface.bounce = 0.0;
      c[0].surface.bounce_vel = 0.0;
      c[0].geom.pos[0] = pos[0];
      c[0].geom.pos[1] = pos[1];
      c[0].geom.pos[2] = 0.0;
      c[0].geom.depth = -q[2];
      c[0].geom.normal[0] = 0;
      c[0].geom.normal[1] = 1;
      c[0].geom.normal[2] = 1.0;
      dJointID contact_joint = dJointCreateContact(world, contact_group, c);
      dJointAttach(contact_joint, wheel, 0);
    }
  }

  if (iterations > 0)
  {
    dWorldSetQuickStepNumIterations(world, iterations);
    dWorldQuickStep(world, step_size);
  }
  else
    dWorldStep(world, step_size);

  // get the linear velocity and compute the linear component
  const dReal* xd = dBodyGetLinearVel(wheel);
  double ke = MASS * (xd[0]*xd[0] + xd[1]*xd[1] * xd[2]*xd[2]);

  // rotate the wheel
  dMatrix3 JRT, J;
  dMass m;
  dBodyGetMass(wheel, &m);
  dMultiply2(JRT, m.I, R, 3, 3, 3);
  dMultiply2(J,  R, JRT, 3, 3, 3);

  // get the angular velocity and compute the angular component
  const dReal* w = dBodyGetAngularVel(wheel);
  dVector3 v;
  dMultiply0(v, J, w, 3, 3, 1); 
  ke += w[0]*v[0] + w[1]*v[1] + w[2]*v[2];
  ke *= 0.5;

  // compute the potential energy
  double pe = 1.0 * MASS * pos[2];

  // compute energy
  std::ofstream out("energy.dat", std::ostream::app);
  out << ke << " " << pe << " " << (ke + pe) << std::endl;
  out.close();

  // compute constraint violation
  out.open("cvio.dat", std::ostream::app);
  out << cvio << std::endl;
  out.close();

  dJointGroupEmpty(contact_group);
}

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cerr << "syntax: rimless-wheel <step size> <quick step iterations>" << std::endl;
    return -1;
  }

  // wipe the files
  std::ofstream out("cvio.dat");
  out.close();
  out.open("energy.dat");
  out.close();

  // initialize ode
  dInitODE();

  // get the step size and the number of quick step iterations
  double step_size = std::atof(argv[1]);
  iterations = std::atoi(argv[2]);

  // initialize 
  init();

  // set CFM to smallest value s.t. we don't get a normalize error
  dWorldSetCFM(world, 1e-6);
  dWorldSetERP(world, 0.0);

  // step to 10s
  for (double t=0; t< 1000.0; t+= step_size)
    step(t, step_size);     

  // close ODE
  dJointGroupDestroy(contact_group);
  dWorldDestroy(world);
}

