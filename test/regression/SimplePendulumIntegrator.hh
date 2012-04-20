#include <math.h>

// integrate d^2 theta / dt^2 = - g * sin(theta) / l
// by discretizing it as
//   theta_2^2 - 2*theta_1 + theta_0 / dt^2 = -g * sin(theta_1) / l
//
// where thata_0 is theta(t_1 - 2*dt)
//       thata_1 is theta(t_1)
//       and
//       thata_f or theta(t_f) is the solution returned
//
// If pendulum starts out stationary, one can assume theta_0 = theta_1
//
double PendulumAngle( double g, double l, double theta_0, double theta_1, double t_1, double t_f , double dt)
{
  double theta_f = theta_1;
  int steps = ceil((t_f - t_1) / dt);
  double t = t_1;
  for (int i = 0 ; i < steps; i++)
  {
    t += dt;
    theta_f = -dt*dt*g*sin(theta_1)/l - theta_0 + 2.0*theta_1;
    // next step
    theta_0 = theta_1;
    theta_1 = theta_f;
    //printf("test %f %f %f\n",t, t_f,theta_f);
  }
  //printf("test %f %f\n",t, t_f,theta_f);

  return theta_f;
}
