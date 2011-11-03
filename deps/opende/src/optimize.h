#ifndef _ODE_OPTIMIZE_H
#define _ODE_OPTIMIZE_H

#include <ode/odemath.h>
#include <vector>
#include <boost/shared_array.hpp>

struct CvxOptParams
{
  int m;               // number of inequality constraints
  int nu_len;          // number of equality constraints (rows of A)
  dReal* A;            // equality constraint matrix
  dReal* b;            // equality constraint vector
  dReal mu;            // mu parameter for convex optimization
  void* data;          // user data passed to functions 
  int iterations;      // number of iterations performed (on exit)
  int max_iterations;  // maximum number of iterations to be performed
  dReal beta;          // exponential factor used in backtracking line search
  dReal alpha;         // tolerance factor used in backtracking line search
  dReal eps;           // tolerance to which the problem should be solved

  // user defined functions
  bool (*tcheck)(dReal*,int,void*);  // early exit testing function
  void (*fx)(dReal*,int,dReal*,int,void*);   // objective/constraint function
  void (*grad)(dReal*,int,int,dReal*,void*); // gradient function
  bool (*hess)(dReal*,int,int,dReal*,void*); // hessian function

  CvxOptParams()
  {
    m = nu_len = 0;
    A = b = NULL;
    data = NULL;
    iterations = 0;
    max_iterations = 0;
    eps = (dReal) 0.0;

    // setup reasonable values for alpha, beta, and mu
    alpha = (dReal) 0.05;
    beta = (dReal) 0.5;
    mu = (dReal) 10.0;

    tcheck = NULL;
    fx = NULL;
    grad = NULL;
    hess = NULL;
  }
};

bool dSolveLCPIP(dReal* M, dReal* q, dReal* x, int n, dReal eps_feas);
bool dMakeFeasibleConvex(CvxOptParams& cparams, dReal eps_feas, dReal* x, int n);
bool dOptimizeConvexPrimalDual(CvxOptParams& cparams, dReal eps_feas, dReal* x, int n);

#endif

