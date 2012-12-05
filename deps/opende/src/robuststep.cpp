/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include "step.h"
#include "collision_kernel.h"
#include "objects.h"
#include "joints/joint.h"
#include "joints/contact.h"
#include <ode/odeconfig.h>
#include "config.h"
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include "util.h"
#include "optimize.h"
#include <iostream>
#include <vector>
#include <limits>
#include <boost/shared_array.hpp>

using boost::shared_array;
using std::vector;

//****************************************************************************
// misc defines
#define DEBUG

// memory allocation system
#ifdef dUSE_MALLOC_FOR_ALLOCA
unsigned int dMemoryFlag;
#define REPORT_OUT_OF_MEMORY fprintf(stderr, "Insufficient memory to complete rigid body simulation.  Results will not be accurate.\n")

#define CHECK(p)                                \
  if (!p) {                                     \
    dMemoryFlag = d_MEMORY_OUT_OF_MEMORY;       \
    return;                                     \
  }

#define ALLOCA(t,v,s)                           \
  Auto<t> v(malloc(s));                         \
  CHECK(v)

#else // use alloca()

#define ALLOCA(t,v,s)                           \
  Auto<t> v( dALLOCA16(s) );

#endif

void dSetSubMat0(dReal* A, int ac, dReal* B, int br, int bc, int sr, int sc);
void dSetSubMat1(dReal* A, int ac, dReal* B, int br, int bc, int sr, int sc);
void printMatrix(dReal* A, int m, int n);
void dCopy(dReal* target, int tstride, dReal* source, int sstride, int n);
void dNPMultiply0(dReal *A, const dReal *B, const dReal *C, int p, int q, int r);
void dNPMultiply1(dReal *A, const dReal *B, const dReal *C, int p, int q, int r);
size_t dxEstimateStepMemoryRequirements(dxBody* const* body, int nb, dxJoint* const* _joint, int _nj);

/* This template should work almost like std::auto_ptr
 */
template<class T>
struct Auto {
  T *p;
  Auto(void * q) :
    p(reinterpret_cast<T*>(q))
  { }

  ~Auto()
  {
#ifdef dUSE_MALLOC_FOR_ALLOCA
    free(p);
#endif
  }

  operator T*() 
  {
    return p;
  }
  T& operator[] (int i)
  {
    return p[i];
  }
private:
  // intentionally undefined, don't use this
  template<class U>
  Auto& operator=(const Auto<U>&) const;
};





struct BodyPair
{
  int body1;
  int body2;
  BodyPair(int b1, int b2) { body1 = b1; body2 = b2; }
};

struct COptData
{
  int m;                     // number of bilateral constraint equations
  int ml;                    // number of lower-limit constraint equations
  int mh;                    // number of upper-limit constraint equations
  int nc;                    // number of contact constraints
  int nb;                    // number of rigid bodies
  dReal* I;                  // nb-stacked 3x3 inertia matrices
  dReal* invI;               // nb-stacked 3x3 inverse inertia matrices
  dReal* J;                  // the bilateral constraint Jacobian
  dReal* j_c;                // the r.h.s. of the equation J*v = j_c
  dReal* Jl;                 // lower-limit bilateral constraint Jacobian
  dReal* jl_c;               // the r.h.s. of the equation Jl*v >= jl_c
  dReal* Jh;                 // upper-limit bilateral constraint Jacobian
  dReal* jh_c;               // the r.h.s. of the equation Jh*v <= jh_c
  dReal* N;                  // the normal contact constraint Jacobian
  dReal* n_c;                // the r.h.s. of the equation N*v >= n_c
  dReal* T1;                 // first tangent direction constraint Jacobian
  dReal* T2;                 // second tangent direction constraint Jacobian
  dReal* hfext;              // external forces on the bodies * stepsize
  dReal* v;                  // velocities of the bodies
  dReal kappa;               // sum of contact forces for the frictionless prob
  dReal* mu;                 // vector of friction coefficients
  dReal* ohess1;             // objective function Hessian (phase I)
  dReal* ohess2;             // objective function Hessian (phase II)
  vector<BodyPair> biPairs;  // pairs of bodies for bilateral constraints
  vector<BodyPair> uniPairs; // pairs of bodies for contact constraints
  vector<BodyPair> loPairs;  // pairs of bodies for lo-limit bilat. constraints
  vector<BodyPair> hiPairs;  // pairs of bodies for hi-limit bilat. constraints
  dxBody* const* bodies;     // vector of bodies

  // work variables (alloc'd automatically)
  dReal* cn;                 // contact normal impulses
  dReal* ct1;                // contact tangential direction 1 impulses
  dReal* ct2;                // contact tangential direction 2 impulses
  dReal* cv;                 // bilateral joint constraint impulses
  dReal* cl;                 // impulses for enforcing lower joint limits
  dReal* ch;                 // impulses for enforcing upper joint limits
  dReal* Nv;                 // the vector N*v^{n+1}
  dReal* Mv;                 // the vector inv(M)*v^{n+1}
  dReal* Jlv;                // the vector Jl*v^{n+1}
  dReal* Jhv;                // the vector Jh*v^{n+1}
  dReal* vnp1;               // the vector v^{n+1}
  dReal* worknb6;            // nb*6-dimensional work vector
  dReal* worknb6_2;          // nb*6-dimensional work vector
  dReal* stack1;             // matrix stack: M^{-1}[N' J' Jl' Jh']
  dReal* stack2;             // matrix stack: M^{-1}[N' T1' T2' J' Jl' Jh']
  dReal* NStack;             // N*matrix stack

  COptData(int _m, int _ml, int _mh, int _nc, int _nb)
  {
    // store necessary variables
    this->m = _m;
    this->ml = _ml;
    this->mh = _mh;
    this->nc = _nc;
    this->nb = _nb;

    // create work variables
    cn = new dReal[_nc];
    ct1 = new dReal[_nc];
    ct2 = new dReal[_nc];
    cv = new dReal[_m];
    cl = new dReal[_ml];
    ch = new dReal[_mh];
    Nv = new dReal[_nc];
    Mv = new dReal[_nb*6];
    vnp1 = new dReal[_nb*6];
    worknb6 = new dReal[_nb*6];
    worknb6_2 = new dReal[_nb*6];
    Jlv = new dReal[_ml];
    Jhv = new dReal[_mh];
    stack1 = new dReal[(_nc + _m + _ml + _mh)*_nb*6];
    stack2 = new dReal[(_nc*3 + _m + _ml + _mh)*_nb*6];
    NStack = new dReal[_nc * (_nc*3 + _m + _ml + _mh)];
  }

  ~COptData()
  {
    delete [] cn;
    delete [] ct1;
    delete [] ct2;
    delete [] cv;
    delete [] cl;
    delete [] ch;
    delete [] Nv;
    delete [] Mv;
    delete [] Jlv;
    delete [] Jhv;
    delete [] vnp1;
    delete [] worknb6;
    delete [] worknb6_2;
    delete [] stack1;
    delete [] stack2;
    delete [] NStack;
  }
};

// estimates memory requirements for robust step
size_t dxEstimateRobustStepMemoryRequirements(dxBody* const* body, int nb, dxJoint* const* _joint, int _nj)
{
  return dxEstimateStepMemoryRequirements(body, nb, _joint, _nj);
}

// dReal dot product routine with stride
dReal dot(const dReal* x, int xstride, const dReal* y, int ystride, int n)
{
  dReal sum = (dReal) 0.0;
  for (int i=0; i< n; i++, x+= xstride, y+= ystride)
    sum += *x * *y;
  return sum;
}

// dReal addition routine
void dAdd(dReal* target, dReal* op1, dReal* op2, int n)
{
  for (int i=0; i< n; i++)
    *target++ = *op1++ + *op2++;
}

// dReal subtraction routine
void dSub(dReal* target, dReal* op1, dReal* op2, int n)
{
  for (int i=0; i< n; i++)
    *target++ = *op1++ - *op2++;
}

// dReal copying routine
void dCopy(dReal* target, dReal* source, int n)
{
  memcpy(target, source, n*sizeof(dReal));
}

// determines an orthonormal vector
static void det_orthonormal_vec(dReal* dir, dReal* v)
{
  dReal ones[3] = {1, 1, 1};
  dReal x = dFabs(dir[0]);
  dReal y = dFabs(dir[1]);
  dReal z = dFabs(dir[2]);
  if (x > y)
  {
    if (x > z)
      ones[0] = 0;
    else
      ones[2] = 0;
  }
  else
  {
    if (y > z)
      ones[1] = 0;
    else
      ones[2] = 0;
  }

  dCROSS(v, =, dir, ones);
} 

// verifies that all elements of a vector are finite (for debugging/testing)
static void verifyFinite(dReal* /*A*/, int /*n*/)
{
  /*for (int i=0; i< n; i++)
    dIASSERT(!isnan(A[i]) && !isinf(A[i]));
    */
}

// multiplies a nr x (nb*6) matrix (N, T1, or T2) by a (nb*6) x nc matrix
static void multSparseMatrix0(dReal* res, dReal* const M, dReal* const v, int nr, int nc, vector<BodyPair>& pairs)
{
  dIASSERT(nr == (int) pairs.size());

  // zero out results vector
  dSetZero(res, nr*nc);

  for (int i=0, j=0; i< nr; i++, j+= 12)
  {
    // check for non-existent second body
    if (pairs[i].body2 >= 0)
    {
      // get the two wrenches
      const dReal* w1 = M + j;
      const dReal* w2 = w1 + 6;

      for (int col=0; col < nc; col++)
      {
        // set the appropriate source pointers from v
        const dReal* v1 = v + pairs[i].body1*6*nc + col;
        const dReal* v2 = v + pairs[i].body2*6*nc + col;

        // compute the two dot products
        res[i*nc + col] = dot(w1, 1, v1, nc, 6) + dot(w2, 1, v2, nc, 6);
      }
    }
    else
    {
      const dReal* w = M + j;
      for (int col=0; col< nc; col++)
      {
        const dReal* vx = v + pairs[i].body1*6*nc + col;
        res[i*nc + col] = dot(w, 1, vx, nc, 6);
      }
    }
  }

  // verify that all elements are finite
  #ifdef DEBUG
  verifyFinite(res, nr*nc);
  #endif
}

// multiplies the transpose of a nr x (nb*6) matrix (N, T1, or T2) by a nb*6 x nc-dimensional matrix
static void multSparseMatrix1(dReal* res, dReal* const M, dReal* const v, int nr, int nb, int nc, vector<BodyPair>& pairs)
{
  dIASSERT(nr == (int) pairs.size());

  // zero out results vector
  dSetZero(res, nc*nb*6);

  for (int i=0, j=0; i< nr; i++, j+= 12)
  {
    // check for non-existent second body
    if (pairs[i].body2 >= 0)
    {
      // get the two wrenches
      const dReal* w1 = M + j;
      const dReal* w2 = w1 + 6;

      // determine the row the first wrench starts at
      //int res_row_skip = nc;
      int w1row = pairs[i].body1 * 6;
      int w2row = pairs[i].body2 * 6;

      for (int col=0; col < nc; col++)
      {
        // update the results vector
        int ridx = w1row*nc+col;
        int vidx = i*nc+col;
        res[ridx] += w1[0]*v[vidx]; ridx += nc;
        res[ridx] += w1[1]*v[vidx]; ridx += nc;
        res[ridx] += w1[2]*v[vidx]; ridx += nc;
        res[ridx] += w1[3]*v[vidx]; ridx += nc;
        res[ridx] += w1[4]*v[vidx]; ridx += nc;
        res[ridx] += w1[5]*v[vidx]; ridx = w2row*nc+col;
        res[ridx] += w2[0]*v[vidx]; ridx += nc;
        res[ridx] += w2[1]*v[vidx]; ridx += nc;
        res[ridx] += w2[2]*v[vidx]; ridx += nc;
        res[ridx] += w2[3]*v[vidx]; ridx += nc;
        res[ridx] += w2[4]*v[vidx]; ridx += nc;
        res[ridx] += w2[5]*v[vidx]; ridx += nc;
      }
    }
    else
    {
      const dReal* w = M + j;
      int wrow = pairs[i].body1 * 6;
      for (int col=0; col < nc; col++)
      {
        int ridx = wrow*nc+col;
        int vidx = i*nc+col;
        res[ridx] += w[0]*v[vidx]; ridx += nc;
        res[ridx] += w[1]*v[vidx]; ridx += nc;
        res[ridx] += w[2]*v[vidx]; ridx += nc;
        res[ridx] += w[3]*v[vidx]; ridx += nc;
        res[ridx] += w[4]*v[vidx]; ridx += nc;
        res[ridx] += w[5]*v[vidx]; ridx += nc;
      }
    }
  }

  // verify that all elements are finite
  #ifdef DEBUG
  verifyFinite(res, nb*6*nc);
  #endif
}

// determines the pairs of bodies, used for contact matrix-vector multiplication
static void determineBodyPairs(dxJoint* const* joints, int nj, vector<BodyPair>& contact_pairs, vector<BodyPair>& bilateral_pairs, vector<BodyPair>& lo_pairs, vector<BodyPair>& hi_pairs)
{
  dReal dummy[6*6], lo[6], hi[6];
  int findex[6];

  dxJoint::Info1 info;
  dxJoint::Info2 info2;
  info2.rowskip = 6;
  info2.fps = 0.0;
  info2.erp = 0.0;

  // setup all of info2 structure now, even though we won't really use it... 
  info2.J1l = dummy;
  info2.J1a = dummy;
  info2.J2l = dummy;
  info2.J2a = dummy;
  info2.c = dummy;
  info2.lo = lo;
  info2.hi = hi;
  info2.cfm = dummy;
  info2.findex = findex;

  // clear the vectors of body pairs
  contact_pairs.clear();
  bilateral_pairs.clear();
  lo_pairs.clear();
  hi_pairs.clear();

  // iterate through all constraints
  for (int i=0; i< nj; i++)
  {
    // get the info for the joint
    joints[i]->getInfo1(&info);

    // get the two bodies
    dxBody* body1 = joints[i]->node[0].body;
    dxBody* body2 = joints[i]->node[1].body;

    if (joints[i]->type() == dJointTypeContact)
    {
      if (body1 && body2)
        contact_pairs.push_back(BodyPair(body1->tag, body2->tag));
      else
      {
        dIASSERT(body1);
        contact_pairs.push_back(BodyPair(body1->tag, -1));
      }
    }
    else
    {
      // get info2 for the joint
      joints[i]->getInfo2(&info2);

      // add the pair nub times to bilateral constraints
      for (int j=0; j< info.nub; j++)
      {
        if (body1 && body2)
          bilateral_pairs.push_back(BodyPair(body1->tag, body2->tag));
        else
        {
          dIASSERT(body1);
          bilateral_pairs.push_back(BodyPair(body1->tag, -1));
        }
      }

      // add any lo / hi pairs
      for (int j=info.nub; j< info.m; j++)
      {
        if (lo[j] < dInfinity)
        {
          if (body1 && body2)
            lo_pairs.push_back(BodyPair(body1->tag, body2->tag));
          else
          {
            dIASSERT(body1);
            lo_pairs.push_back(BodyPair(body1->tag, -1));
          }
        }
        if (hi[j] > dInfinity)
        {
          if (body1 && body2)
            hi_pairs.push_back(BodyPair(body1->tag, body2->tag));
          else
          {
            dIASSERT(body1);
            hi_pairs.push_back(BodyPair(body1->tag, -1));
          }
        }
      }
    }
  }
}

// sets up a wrench at the given output
static void setWrench(dReal* out, dReal* p, dReal* x, dReal* dir, bool negate_dir)
{
  const int X = 0, Y = 1, Z = 2;

  // compute r
  dReal r[3];
  r[X] = p[X] - x[X];
  r[Y] = p[Y] - x[Y];
  r[Z] = p[Z] - x[Z];

  // compute r x dir
  dReal rxdir[3];
  dCROSS(rxdir, =, r, dir);

  // set the wrench
  if (!negate_dir)
  {
    (*out++) = dir[X];
    (*out++) = dir[Y];
    (*out++) = dir[Z];
    (*out++) = rxdir[X];
    (*out++) = rxdir[Y];
    (*out++) = rxdir[Z];
  }
  else
  {
    (*out++) = -dir[X];
    (*out++) = -dir[Y];
    (*out++) = -dir[Z];
    (*out++) = -rxdir[X];
    (*out++) = -rxdir[Y];
    (*out++) = -rxdir[Z];
  }
}

//****************************************************************************
// forms the stacked matrix [N' J' Jl' Jh']
//****************************************************************************
static void formStack1(dReal* A, dReal* N, dReal* J, dReal* Jl, dReal* Jh, int nb, int nc, int m, int ml, int mh, const vector<BodyPair>& uniPairs, const vector<BodyPair>& biPairs)
{
  const int NVARS = nc + m + ml + mh;

  // first, clear A
  dSetZero(A, nb*6*NVARS);

  // setup current column index of A
  int col = 0;

  // setup elements from N
  for (int i=0, j=0; i< nc; i++, col++, j+= 12)
  {
    int body1st = uniPairs[i].body1*6;
    int body2nd = uniPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, N + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, N + j + 6, 1, 6);
  }

  // setup elements from J
  for (int i=0, j=0; i< m; i++, col++, j+= 12)
  {
    int body1st = biPairs[i].body1*6;
    int body2nd = biPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, J + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, J + j + 6, 1, 6);
  }

  // setup elements from Jl
  for (int i=0, j=0; i< ml; i++, col++, j+= 12)
  {
    int body1st = biPairs[i].body1*6;
    int body2nd = biPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, Jl + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, Jl + j + 6, 1, 6);
  }

  // setup elements from Jh
  for (int i=0, j=0; i< mh; i++, col++, j+= 12)
  {
    int body1st = biPairs[i].body1*6;
    int body2nd = biPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, Jh + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, Jh + j + 6, 1, 6);
  }

}

//****************************************************************************
// forms the stacked matrix [N' T1' T2' J' Jl' Jh']
//****************************************************************************
static void formStack2(dReal* A, dReal* N, dReal* T1, dReal* T2, dReal* J, dReal* Jl, dReal* Jh, int nb, int nc, int m, int ml, int mh, const vector<BodyPair>& uniPairs, const vector<BodyPair>& biPairs)
{
  const int NVARS = nc*3 + m + ml + mh;

  // first, clear A
  dSetZero(A, nb*6*NVARS);

  // setup current column index of A
  int col = 0;

  // setup elements from N
  for (int i=0, j=0; i< nc; i++, col++, j+= 12)
  {
    int body1st = uniPairs[i].body1*6;
    int body2nd = uniPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, N + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, N + j + 6, 1, 6);
  }

  // setup elements from T1
  for (int i=0, j=0; i< nc; i++, col++, j+= 12)
  {
    int body1st = uniPairs[i].body1*6;
    int body2nd = uniPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, T1 + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, T1 + j + 6, 1, 6);
  }

  // setup elements from T2
  for (int i=0, j=0; i< nc; i++, col++, j+= 12)
  {
    int body1st = uniPairs[i].body1*6;
    int body2nd = uniPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, T2 + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, T2 + j + 6, 1, 6);
  }

  // setup elements from J
  for (int i=0, j=0; i< m; i++, col++, j+= 12)
  {
    int body1st = biPairs[i].body1*6;
    int body2nd = biPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, J + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, J + j + 6, 1, 6);
  }

  // setup elements from Jl
  for (int i=0, j=0; i< ml; i++, col++, j+= 12)
  {
    int body1st = biPairs[i].body1*6;
    int body2nd = biPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, Jl + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, Jl + j + 6, 1, 6);
  }

  // setup elements from Jh
  for (int i=0, j=0; i< mh; i++, col++, j+= 12)
  {
    int body1st = biPairs[i].body1*6;
    int body2nd = biPairs[i].body2*6;
    dCopy(A+body1st*NVARS+col, NVARS, Jh + j, 1, 6);
    if (body2nd >= 0)
      dCopy(A+body2nd*NVARS+col, NVARS, Jh + j + 6, 1, 6);
  }
}

//****************************************************************************
// sets up the bilateral constraint Jacobians
//****************************************************************************
static void getJ_c(dxWorld* world, dxJoint* const* joints, int nj, dReal stepsize1, int& m, shared_array<dReal>& J, shared_array<dReal>& j_c, int& ml, shared_array<dReal>& Jl, shared_array<dReal>& jl_c, int& mh, shared_array<dReal>& Jh, shared_array<dReal>& jh_c)
{
  // set all m's to zero
  m = ml = mh = 0;

  // prepare the info structures
  dxJoint::Info1 info1;
  dxJoint::Info2 info2;
  info2.rowskip = 6;
  info2.fps = stepsize1;
  info2.erp = world->global_erp;

  // I did this a bit foolishly: because Jacobians for both bodies are all on
  // the same line, so copying is necessary...
  dReal J1[6*6], J2[6*6], lo[6], hi[6], c[6];

  // setup all of info2 structure now, even though we won't really use it 
  // all until later
  info2.J1l = J1;
  info2.J1a = J1+3;
  info2.J2l = J2;
  info2.J2a = J2+3;
  info2.c = c;
  info2.lo = lo;
  info2.hi = hi;

  // use dummy variables for some parts of info2 structure 
  dReal dummy[6];
  int findex[6];
  info2.cfm = dummy;
  info2.findex = findex;

  // first, determine m, ml, and mh
  for (int i=0, j=0; i< nj; i++)
  {
    // skip contact constraints
    if (joints[i]->type() == dJointTypeContact)
    {
      joints[i]->tag = -1;
      continue;
    }

    // get the joint information
    joints[i]->getInfo1(&info1);
    joints[i]->getInfo2(&info2);    

    // tag the joint
    joints[i]->tag = (info1.m > 0) ? j++ : -1;

    // increase m by the number of unbounded constraints
    m += info1.nub;

    // only update ml/mh if info1.nub != info1.m
    if (info1.nub < info1.m)
    {
      for (int k=info1.nub; k< info1.m; k++)
      {
        if (info2.lo[k] > -dInfinity)
          ml++;
        if (info2.hi[k] < dInfinity)
          mh++;
      }
    }
  }

  // allocate memory for the Jacobians and rhs's
  J = shared_array<dReal>(new dReal[m*12]);
  j_c = shared_array<dReal>(new dReal[m]);
  Jl = shared_array<dReal>(new dReal[ml*12]);
  jl_c = shared_array<dReal>(new dReal[ml]);
  Jh = shared_array<dReal>(new dReal[mh*12]);
  jh_c = shared_array<dReal>(new dReal[ml]);

  // zero out the Jacobians and rhs's
  dSetZero(J.get(),m*12);
  dSetZero(j_c.get(),m);
  dSetZero(Jl.get(),ml*12);
  dSetZero(jl_c.get(),ml);
  dSetZero(Jh.get(),mh*12);
  dSetZero(jh_c.get(),mh);
  
  // now, setup the Jacobians and rhs's
  for (int i=0, jj=0, jl=0, jh=0; i<nj; i++) 
  {
    // skip non-tagged joints
    if (joints[i]->tag == -1)
      continue;

    // determine how many constraint equations and get the Jacobians
    joints[i]->getInfo1(&info1);
    joints[i]->getInfo2(&info2);

    // copy the standard Jacobian and rhs 
    dSetSubMat0(J.get(), 12, J1, info1.nub, 6, jj, 0);
    dSetSubMat0(J.get(), 12, J2, info1.nub, 6, jj, 6);
    dCopy(j_c.get()+jj, c, info1.nub);
    jj += info1.nub;

    // copy to the limit Jacobians
    if (info1.nub < info1.m)
    {
      for (int k=info1.nub; k< info1.m; k++)
      {
        if (info2.lo[k] > -dInfinity)
        {
          dSetSubMat0(Jl.get(), 12, J1+k*6, 1, 6, jl, 0);
          dSetSubMat0(Jl.get(), 12, J2+k*6, 1, 6, jl, 6);
          jl_c[jl] = c[k];
          jl++;
        } 
        if (info2.hi[k] < dInfinity)
        {
          dSetSubMat0(Jh.get(), 12, J1+k*6, 1, 6, jh, 0);
          dSetSubMat0(Jh.get(), 12, J2+k*6, 1, 6, jh, 6);
          jh_c[jh] = c[k];
          jh++;
        }
      }
    }
  }
}

//****************************************************************************
// calculates the relative velocity vector (for testing/debugging purposes)
//****************************************************************************
/*static void determineRvel(dxJoint* const* joints, int nj)
{
  for (int i=0; i< nj; i++)
  {
    if (joints[i]->type() == dJointTypeContact)
    {
      // cast it as a contact joint
      dxJointContact* joint = (dxJointContact*) joints[i];

      // get the contact info
      dContact contact = joint->contact;

      // get the two bodies
      dxBody* body1 = joints[i]->node[0].body;
      dxBody* body2 = joints[i]->node[1].body;

      // get the normal
      dReal n[3] = { contact.geom.normal[0], contact.geom.normal[1], contact.geom.normal[2] };

      // calculate the normal velocity for body 1
      dReal* pos1 = body1->posr.pos;
      dReal r[3];
      r[0] = contact.geom.pos[0] - pos1[0];
      r[1] = contact.geom.pos[1] - pos1[1];
      r[2] = contact.geom.pos[2] - pos1[2];
      dReal wxr[3];
      dCROSS(wxr, =, body1->avel, r);
      wxr[0] += body1->lvel[0];
      wxr[1] += body1->lvel[1];
      wxr[2] += body1->lvel[2];
      dReal rvel = n[0]*wxr[0] + n[1]*wxr[1] + n[2]*wxr[2]; 

      // calculate the normal velocity for body 2
      if (body2)
      {
        dReal* pos2 = body2->posr.pos;
        r[0] = contact.geom.pos[0] - pos2[0];
        r[1] = contact.geom.pos[1] - pos2[1];
        r[2] = contact.geom.pos[2] - pos2[2];
        dCROSS(wxr, =, body2->avel, r);
        wxr[0] += body2->lvel[0];
        wxr[1] += body2->lvel[1];
        wxr[2] += body2->lvel[2];
        rvel -= n[0]*wxr[0] + n[1]*wxr[1] + n[2]*wxr[2]; 
      }

      std::cout << "relative normal vel for contact " << i << ": " << rvel << std::endl;
    }
  }
}*/

//****************************************************************************
// sets up the "N" matrix (matrix of normal directions)
//****************************************************************************
static void getN_c(dxWorld* world, dxJoint* const* joints, int nj, dReal stepsize1, dReal* N, dReal* c)
{
  //const int X = 0, Y = 1, Z = 2;

  // space for dummy storage -- we don't use the contact Jacobian, lo, or hi
  dReal dummy[6*6];
  int findex[6];

  // prepare the info structure
  dxJoint::Info2 Jinfo;
  Jinfo.rowskip = 6;
  Jinfo.fps = stepsize1;
  Jinfo.erp = world->global_erp;
  Jinfo.J1l = dummy;
  Jinfo.J1a = dummy;
  Jinfo.J2l = dummy;
  Jinfo.J2a = dummy;
  Jinfo.lo = dummy;
  Jinfo.hi = dummy;
  Jinfo.cfm = dummy;
  Jinfo.findex = findex;

  for (int i=0, j=0, k=0; i< nj; i++)
  {
    if (joints[i]->type() == dJointTypeContact)
    {
      // cast it as a contact joint
      dxJointContact* joint = (dxJointContact*) joints[i];

      // get the contact info
      dContact contact = joint->contact;

      // determine the normal
      dReal normal[3];
      if (joint->flags & dJOINT_REVERSE)
      {
        normal[0] = -contact.geom.normal[0];
        normal[1] = -contact.geom.normal[1];
        normal[2] = -contact.geom.normal[2];
      }
      else
      {
        normal[0] = contact.geom.normal[0];
        normal[1] = contact.geom.normal[1];
        normal[2] = contact.geom.normal[2];
      }

      // get c 
      Jinfo.c = c + k++;
      joints[i]->getInfo2(&Jinfo);

      // get the two bodies
      dxBody* body1 = joints[i]->node[0].body;
      dxBody* body2 = joints[i]->node[1].body;

      // setup the wrench for the first body
      dReal* pos1 = body1->posr.pos;
      setWrench(N+j, contact.geom.pos, pos1, normal, false);
      j+= 6;

      // setup the wrench for the second body (if there is one)
      if (body2)
      {
        dReal* pos2 = body2->posr.pos;
        setWrench(N+j, contact.geom.pos, pos2, normal, true);
      }
      j+= 6;
    }
  }
}

//****************************************************************************
// sets up the "T1" matrix (matrix of first tangential directions)
//****************************************************************************
static void getT1(dxJoint* const* joints, int nj, dReal* T1)
{
  //const int X = 0, Y = 1, Z = 2;

  for (int i=0, j=0; i< nj; i++)
  {
    if (joints[i]->type() == dJointTypeContact)
    {
      // cast it as a contact joint
      dxJointContact* joint = (dxJointContact*) joints[i];

      // get the contact info
      dContact contact = joint->contact;

      // determine the first friction direction
      det_orthonormal_vec(contact.geom.normal, contact.fdir1);

      // get the two bodies
      dxBody* body1 = joints[i]->node[0].body;
      dxBody* body2 = joints[i]->node[1].body;

      // setup the wrench for the first body
      dReal* pos1 = body1->posr.pos;
      setWrench(T1+j, contact.geom.pos, pos1, contact.fdir1, false);
      for (int k=0; k< 6; k++)
        dIASSERT(!isnan(T1[j+k]));
      j+= 6;

      // setup the wrench for the second body (if there is one)
      if (body2)
      {
        dReal* pos2 = body2->posr.pos;
        setWrench(T1+j, contact.geom.pos, pos2, contact.fdir1, true);
      }
      j+= 6;
    }
  }
}

//****************************************************************************
// sets up the "T2" matrix (matrix of second tangential directions)
//****************************************************************************
static void getT2(dxJoint* const* joints, int nj, dReal* T2)
{
  //const int X = 0, Y = 1, Z = 2;

  for (int i=0, j=0; i< nj; i++)
  {
    if (joints[i]->type() == dJointTypeContact)
    {
      // cast it as a contact joint
      dxJointContact* joint = (dxJointContact*) joints[i];

      // get the contact info
      dContact contact = joint->contact;

      // determine the first friction direction (unsure why we must do this 2x)
      det_orthonormal_vec(contact.geom.normal, contact.fdir1);

      // compute the second tangential direction
      dReal t2[3];
      dCROSS(t2, =, contact.geom.normal, contact.fdir1);

      // get the two bodies
      dxBody* body1 = joints[i]->node[0].body;
      dxBody* body2 = joints[i]->node[1].body;

      // setup the wrench for the first body
      dReal* pos1 = body1->posr.pos;
      setWrench(T2+j, contact.geom.pos, pos1, t2, false);
      j+= 6;

      // setup the wrench for the second body (if there is one)
      if (body2)
      {
        dReal* pos2 = body2->posr.pos;
        setWrench(T2+j, contact.geom.pos, pos2, t2, true);
      }
      j+= 6;
    }
  }
}

//****************************************************************************
// sets up the vector of friction coefficients
//****************************************************************************
static void getMu(dxJoint* const* joints, int nj, dReal* mu)
{
  for (int i=0, j=0; i< nj; i++)
  {
    if (joints[i]->type() == dJointTypeContact)
    {
      // cast it as a contact joint
      dxJointContact* joint = (dxJointContact*) joints[i];

      // get the contact info
      dContact contact = joint->contact;

      // get mu
      mu[j++] = (contact.surface.mu < dInfinity) ? contact.surface.mu : 1000000;
    }
  }
}

//****************************************************************************
// sets up the vector of velocities
//****************************************************************************
static void getVelocities(dxBody* const* bodies, int nb, dReal* v)
{
  const int X = 0, Y = 1, Z = 2;

  for (int i=0, j=0; i< nb; i++)
  {
    v[j++] = bodies[i]->lvel[X];
    v[j++] = bodies[i]->lvel[Y];
    v[j++] = bodies[i]->lvel[Z];
    v[j++] = bodies[i]->avel[X];
    v[j++] = bodies[i]->avel[Y];
    v[j++] = bodies[i]->avel[Z];
  }
}

//****************************************************************************
// gets vector of external forces on the bodies
//****************************************************************************
static void getFExt(dxBody* const* bodies, int nb, dReal* f)
{
  const int X = 0, Y = 1, Z = 2;

  for (int i=0, j=0; i< nb; i++)
  {
    f[j++] = bodies[i]->facc[X];
    f[j++] = bodies[i]->facc[Y];
    f[j++] = bodies[i]->facc[Z];
    f[j++] = bodies[i]->tacc[X];
    f[j++] = bodies[i]->tacc[Y];
    f[j++] = bodies[i]->tacc[Z];
  }
}

//****************************************************************************
// sets the velocities of the bodies from a vector
//****************************************************************************
static void setVelocities(dReal* v, dxBody* const* bodies, int nb)
{
  const int X = 0, Y = 1, Z = 2;

  for (int i=0, j=0; i< nb; i++)
  {
    bodies[i]->lvel[X] = v[j++];
    bodies[i]->lvel[Y] = v[j++];
    bodies[i]->lvel[Z] = v[j++];
    bodies[i]->avel[X] = v[j++];
    bodies[i]->avel[Y] = v[j++];
    bodies[i]->avel[Z] = v[j++];
  }
}

//****************************************************************************
// carries out forward/inverse generalized mass matrix multiplication
//****************************************************************************
// invI: a stack of nb 3x3 inverse inertia matrices
// v: a nb*6 x vcol dimensional matrix
static void multGMass0(dReal* result, dReal* J, int nb, dxBody * const *body, dReal* v, int vcol, bool inverse)
{
  // clear the result vector
  dSetZero(result, nb*6*vcol);
  dReal tmp1[3];

  int rowskip = vcol;
  for (int i=0, j=0, k=0; i< nb; i++, j+= 6, k+= 9)
  {
    dReal scalar = (inverse) ? body[i]->invMass : body[i]->mass.mass;
    for (int col=0; col < vcol; col++)
    {
      int idx = j*vcol + col;
      result[idx] = scalar*v[idx];
      idx += rowskip;
      result[idx] = scalar*v[idx];
      idx += rowskip;
      result[idx] = scalar*v[idx];
      idx += rowskip;
      dReal tmp2[3] = {v[idx], v[idx+rowskip], v[idx+rowskip+rowskip]};
      dNPMultiply0(tmp1, J+k, tmp2, 3, 3, 1);
      result[idx] = tmp1[0];
      idx += rowskip;
      result[idx] = tmp1[1];
      idx += rowskip;
      result[idx] = tmp1[2];
      idx += rowskip;
    }
  }

  // verify that all elements are finite
  #ifdef DEBUG
  verifyFinite(result, nb*6*vcol);
  #endif
}

// forms the matrix X*inv(M)*X' used in Hessian calculation
static void formXiMXT(shared_array<dReal>& XMXT, dReal* X, int nb, int n, dReal* iJ, dxBody* const* body, vector<BodyPair>& pairs)
{
  // setup dense X' (6nb x n)
  shared_array<dReal> XT(new dReal[n * nb*6]);
  dSetZero(XT.get(), n*nb*6);
  for (int i=0, j=0; i< n; i++, j+= 12)
  {
    int body1row = 6 * pairs[i].body1;
    int body2row = 6 * pairs[i].body2;
    XT[body1row*n + i] = X[j];
    XT[(body1row+1)*n + i] = X[j+1];
    XT[(body1row+2)*n + i] = X[j+2];
    XT[(body1row+3)*n + i] = X[j+3];
    XT[(body1row+4)*n + i] = X[j+4];
    XT[(body1row+5)*n + i] = X[j+5];
    if (body2row >= 0)
    {
      XT[body2row*n + i] = X[j+6];
      XT[(body2row+1)*n + i] = X[j+7];
      XT[(body2row+2)*n + i] = X[j+8];
      XT[(body2row+3)*n + i] = X[j+9];
      XT[(body2row+4)*n + i] = X[j+10];
      XT[(body2row+5)*n + i] = X[j+11];
    }
  }

  // multiply iM * X'
  shared_array<dReal> iMXT(new dReal[6*nb*n]);
  multGMass0(iMXT.get(), iJ, nb, body, XT.get(), n, true);

  // multiply X * result
  XMXT = shared_array<dReal>(new dReal[n*n]);
  multSparseMatrix0(XMXT.get(), X, iMXT.get(), n, n, pairs);
}

//****************************************************************************
// convex optimization objective / constraint functions (phase I) 
//****************************************************************************
static void copt_fx1(dReal* x, int /*n*/, dReal* f, int /*m*/, void* data)
{
  COptData& cd = *((COptData*) data);
  const dReal S_BUFFER = dSqrt(std::numeric_limits<dReal>::epsilon());

  // determine cn, cv, cl, ch
  dReal* cn = cd.cn;
  dReal* cv = cd.cv;
  dReal* cl = cd.cl;
  dReal* ch = cd.ch;
  dCopy(cn, x, cd.nc);
  dCopy(cv, x+cd.nc, cd.m); 
  dCopy(cl, x+cd.nc+cd.m, cd.ml);
  dCopy(ch, x+cd.nc+cd.m+cd.ml, cd.mh);

  // calculate:
  // v = M^{-1}*(N'*cn + J'*cv + Jl'*cl + Jh'*ch + hk) + v^n
  dReal* vnp1 = cd.vnp1;
  dSetZero(cd.worknb6_2, cd.nb*6);
  multSparseMatrix1(cd.worknb6_2, cd.N, cn, cd.nc, cd.nb, 1, cd.uniPairs);
  multSparseMatrix1(cd.worknb6, cd.J, cv, cd.m, cd.nb, 1, cd.biPairs); 
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
  multSparseMatrix1(cd.worknb6, cd.Jl, cl, cd.ml, cd.nb, 1, cd.biPairs);
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
  multSparseMatrix1(cd.worknb6, cd.Jh, ch, cd.mh, cd.nb, 1, cd.biPairs);
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.hfext, cd.nb*6);
  multGMass0(vnp1, cd.invI, cd.nb, cd.bodies, cd.worknb6_2, 1, true);
  dAdd(vnp1, vnp1, cd.v, cd.nb*6);

  // multiply N*v^{n+1}
  dReal* Nv = cd.Nv;
  multSparseMatrix0(Nv, cd.N, vnp1, cd.nc, 1, cd.uniPairs); 

  // multiply Jl*v^{n+1}
  dReal* Jlv = cd.Jlv;
  multSparseMatrix0(Jlv, cd.Jl, vnp1, cd.ml, 1, cd.biPairs);

  // multiply Jh*v^{n+1}
  dReal* Jhv = cd.Jhv;
  multSparseMatrix0(Jhv, cd.Jh, vnp1, cd.mh, 1, cd.biPairs);

  // evaluate objective function: kinetic energy [index 0]
  int fidx = 0;
  dReal* Mv = cd.Mv;
  multGMass0(Mv, cd.I, cd.nb, cd.bodies, vnp1, 1, false);
  f[fidx++] = dDot(Mv, vnp1, cd.nb*6) * (dReal) 0.5; 
dIASSERT(!isnan(f[0]));

  // evaluate non-negativity constraints on cn [indices 1..nc]
  for (int i=0; i< cd.nc; i++)
    f[fidx++] = -cn[i] - S_BUFFER;

  // evaluate non-interpenetration constraints [indices nc+1..2*nc]
  for (int i=0; i< cd.nc; i++)
    f[fidx++] = cd.n_c[i] - Nv[i] - S_BUFFER;

  // evaluate lower joint limit constraints [indices 2*nc+1..2*nc+ml)
  for (int i=0; i< cd.ml; i++)
    f[fidx++] = cd.jl_c[i] - Jlv[i] - S_BUFFER;

  // evaluate upper joint limit constraints [indices 2*nc+ml+1..2*nc+ml+mh]
  for (int i=0; i< cd.mh; i++)
    f[fidx++] = cd.jh_c[i] - Jhv[i] - S_BUFFER;  
}

//****************************************************************************
// convex optimization objective / constraint functions (phase II) 
//****************************************************************************
static void copt_fx2(dReal* x, int /*n*/, dReal* f, int /*m*/, void* data)
{
  COptData& cd = *((COptData*) data);
  const dReal S_BUFFER = dSqrt(std::numeric_limits<dReal>::epsilon());

  // determine cn, ct1, ct2, cv, cl, ch
  dReal* cn = cd.cn;
  dReal* cv = cd.cv;
  dReal* cl = cd.cl;
  dReal* ch = cd.ch;
  dReal* ct1 = cd.ct1;
  dReal* ct2 = cd.ct2;
  dCopy(cn, x, cd.nc);
  dCopy(ct1, x+cd.nc, cd.nc);
  dCopy(ct2, x+cd.nc*2, cd.nc);
  dCopy(cv, x+cd.nc*3, cd.m); 
  dCopy(cl, x+cd.nc*3+cd.m, cd.ml);
  dCopy(ch, x+cd.nc*3+cd.m+cd.ml, cd.mh);

  // calculate:
  // v = M^{-1}*(N'*cn + T1'*ct1+ T2'*ct2 + J'*cv + Jl'*cl + Jh'*ch + hk) + v^n
  dReal* vnp1 = cd.vnp1;
  dSetZero(cd.worknb6_2, cd.nb*6);
  multSparseMatrix1(cd.worknb6_2, cd.N, cn, cd.nc, cd.nb, 1, cd.uniPairs);
  multSparseMatrix1(cd.worknb6, cd.T1, ct1, cd.nc, cd.nb, 1, cd.uniPairs);
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
  multSparseMatrix1(cd.worknb6, cd.T2, ct2, cd.nc, cd.nb, 1, cd.uniPairs);
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
  multSparseMatrix1(cd.worknb6, cd.J, cv, cd.m, cd.nb, 1, cd.biPairs); 
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
  multSparseMatrix1(cd.worknb6, cd.Jl, cl, cd.ml, cd.nb, 1, cd.biPairs);
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
  multSparseMatrix1(cd.worknb6, cd.Jh, ch, cd.mh, cd.nb, 1, cd.biPairs);
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
  dAdd(cd.worknb6_2, cd.worknb6_2, cd.hfext, cd.nb*6);
  multGMass0(vnp1, cd.invI, cd.nb, cd.bodies, cd.worknb6_2, 1, true);
  dAdd(vnp1, vnp1, cd.v, cd.nb*6);

  // multiply N*v^{n+1}
  dReal* Nv = cd.Nv;
  multSparseMatrix0(Nv, cd.N, vnp1, cd.nc, 1, cd.uniPairs); 

  // multiply Jl*v^{n+1}
  dReal* Jlv = cd.Jlv;
  multSparseMatrix0(Jlv, cd.Jl, vnp1, cd.ml, 1, cd.biPairs);

  // multiply Jh*v^{n+1}
  dReal* Jhv = cd.Jhv;
  multSparseMatrix0(Jhv, cd.Jh, vnp1, cd.mh, 1, cd.biPairs);

  // evaluate objective function: kinetic energy [index 0]
  int fidx = 0;
  dReal* Mv = cd.Mv;
  multGMass0(Mv, cd.I, cd.nb, cd.bodies, vnp1, 1, false);
  f[fidx++] = dDot(Mv, vnp1, cd.nb*6) * (dReal) 0.5; 
dIASSERT(!isnan(f[0]));

  // evaluate non-negativity constraints on cn [indices 1..nc]
  for (int i=0; i< cd.nc; i++)
    f[fidx++] = -cn[i] - S_BUFFER;

  // evaluate non-interpenetration constraints [indices nc+1..2*nc]
  for (int i=0; i< cd.nc; i++)
    f[fidx++] = cd.n_c[i] - Nv[i] - S_BUFFER;

  // evaluate lower joint limit constraints [indices 2*nc+1..2*nc+ml)
  for (int i=0; i< cd.ml; i++)
    f[fidx++] = cd.jl_c[i] - Jlv[i] - S_BUFFER;

  // evaluate upper joint limit constraints [indices 2*nc+ml+1..2*nc+ml+mh]
  for (int i=0; i< cd.mh; i++)
    f[fidx++] = cd.jh_c[i] - Jhv[i] - S_BUFFER;  
 
  // evaluate normal impulse constraint [index 2*nc+ml+mh+1]
  f[fidx] = -cd.kappa - 1e-1;
  for (int i=0; i< cd.nc; i++)
    f[fidx] += cn[i];
f[fidx] = -1000000;
  fidx++;

  // evaluate Coulomb friction constraints [indices 2*nc+ml+mh+2..3*nc+ml+mh+1]
  for (int i=0; i< cd.nc; i++)
//    f[fidx++] = (dReal) 0.5 * (ct1[i]*ct1[i] + ct2[i]*ct2[i] - cd.mu[i]*cd.mu[i]*cn[i]*cn[i]) - S_BUFFER;
f[fidx++] = -1000000;
}

//***************************************************************************
// numerical gradient functions for testing/debugging purposes
//****************************************************************************
/*static void copt_ngrad1(dReal* x, int n, int idx, dReal* g, void* data)
{
  COptData& cd = *((COptData*) data);

  // setup for numerical gradient
  const dReal H = (dReal) 1e-6;
  const dReal iH2 = (dReal) 0.5/H;
  shared_array<dReal> xx(new dReal[n]);
  dCopy(xx.get(), x, n);

  // determine how many constraints there are
  const unsigned M = cd.nc*2 + cd.m + cd.ml + cd.mh;

  // setup temporary arrays
  shared_array<dReal> v1(new dReal[M+1]);
  shared_array<dReal> v2(new dReal[M+1]);

  for (int i=0; i< n; i++)
  {
    xx[i] += H;
    copt_fx1(xx.get(), n, v1.get(), M, data);
    xx[i] -= H*2;
    copt_fx1(xx.get(), n, v2.get(), M, data);
    xx[i] += H;
    g[i] = (v1[idx] - v2[idx])*iH2;
  }
}*/
/*static void copt_ngrad2(dReal* x, int n, int idx, dReal* g, void* data)
{
  COptData& cd = *((COptData*) data);

  // setup for numerical gradient
  const dReal H = (dReal) 1e-6;
  const dReal iH2 = (dReal) 0.5/H;
  shared_array<dReal> xx(new dReal[n]);
  dCopy(xx.get(), x, n);

  // determine how many constraints there are
  const unsigned M = cd.nc*3 + cd.m + cd.ml + cd.mh + 1;

  // setup temporary arrays
  shared_array<dReal> v1(new dReal[M+1]);
  shared_array<dReal> v2(new dReal[M+1]);

  for (int i=0; i< n; i++)
  {
    xx[i] += H;
    copt_fx2(xx.get(), n, v1.get(), M, data);
    xx[i] -= H*2;
    copt_fx2(xx.get(), n, v2.get(), M, data);
    xx[i] += H;
    g[i] = (v1[idx] - v2[idx])*iH2;
  }
}*/


//****************************************************************************
// convex optimization gradient functions 
//****************************************************************************
static void copt_grad1(dReal* x, int n, int idx, dReal* g, void* data)
{
  COptData& cd = *((COptData*) data);
  //const dReal S_BUFFER = dSqrt(std::numeric_limits<dReal>::epsilon());

  // determine cv, cn, cl, ch 
  dReal* cn = cd.cn;
  dReal* cv = cd.cv;
  dReal* cl = cd.cl;
  dReal* ch = cd.ch;
  dCopy(cn, x, cd.nc);
  dCopy(cv, x+cd.nc, cd.m); 
  dCopy(cl, x+cd.nc+cd.m, cd.ml);
  dCopy(ch, x+cd.nc+cd.m+cd.ml, cd.mh);

  // evaluate objective function: kinetic energy
  // given: * is inverse
  // f0 = (cn'NM*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + cn'Nv +
  //       ct1'T1M*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + ct1'T1v +
  //       ct2'T2M*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + ct2'T2v +
  //       cv'JM*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + cv'Jvv +
  //       cl'JlM*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + cl'Jlv +
  //       ch'JhM*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + ch'Jhv +
  //       hfext'M*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + hfext'v +
  //       v'(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + v'v)*0.5
  // gf0/cn =  NM*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + Nv
  // gf0/cv =  JM*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + Jv
  // gf0/cl =  JlM*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + Jlv
  // gf0/ch =  JhM*(N'cn + J'cv + Jl'cl + Jh'ch + hfext) + Jhv

  if (idx == 0)
  {
    // calculate v^{n+1}
    dReal* vnp1 = cd.vnp1;
    dSetZero(cd.worknb6_2, cd.nb*6);
    multSparseMatrix1(cd.worknb6_2, cd.N, cn, cd.nc, cd.nb, 1, cd.uniPairs);
    multSparseMatrix1(cd.worknb6, cd.J, cv, cd.m, cd.nb, 1, cd.biPairs); 
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
    multSparseMatrix1(cd.worknb6, cd.Jl, cl, cd.ml, cd.nb, 1, cd.biPairs);
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
    multSparseMatrix1(cd.worknb6, cd.Jh, ch, cd.mh, cd.nb, 1, cd.biPairs);
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.hfext, cd.nb*6);
    multGMass0(vnp1, cd.invI, cd.nb, cd.bodies, cd.worknb6_2, 1, true);
    dAdd(vnp1, vnp1, cd.v, cd.nb*6);

    // calculate gradients
    dSetZero(g, n);
    multSparseMatrix0(g, cd.N, vnp1, cd.nc, 1, cd.uniPairs);
    multSparseMatrix0(g + cd.nc, cd.J, vnp1, cd.m, 1, cd.biPairs);
    multSparseMatrix0(g + cd.nc+cd.m, cd.Jl, vnp1, cd.ml, 1, cd.biPairs);
    multSparseMatrix0(g + cd.nc+cd.m+cd.ml, cd.Jh, vnp1, cd.mh, 1, cd.biPairs); 
  }
  // gradients for non-negativity constraints
  else if (idx <= cd.nc)
  {
    dSetZero(g, n);
    g[idx-1] = (dReal) -1.0;
  }
  // gradients for non-interpenetration constraints
  // fi = N(M*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + v) >= c
  // gfi/cn = NM*N'
  // gfi/ct1 = NM*T1'
  // gfi/ct2 = NM*T2'
  // gfi/cv = NM*J'
  // gfi/cl = NM*Jl'
  // gfi/ch = NM*Jh'
  else
  {
    // get the index of the non-interpenetration constraint
    int npidx = idx - cd.nc - 1;

    // multiply N by the stack
    multSparseMatrix0(cd.NStack, cd.N, cd.stack1, cd.nc, n, cd.uniPairs);
    for (int i=0; i< n; i++)
      g[i] = -cd.NStack[npidx*n + i];
  }
}

static void copt_grad2(dReal* x, int n, int idx, dReal* g, void* data)
{
  COptData& cd = *((COptData*) data);
  //const dReal S_BUFFER = dSqrt(std::numeric_limits<dReal>::epsilon());

  // determine cv, cn, ct1, ct2
  dReal* cn = cd.cn;
  dReal* cv = cd.cv;
  dReal* ct1 = cd.ct1;
  dReal* ct2 = cd.ct2;
  dReal* cl = cd.cl;
  dReal* ch = cd.ch;
  dCopy(cn, x, cd.nc);
  dCopy(ct1, x+cd.nc, cd.nc);
  dCopy(ct2, x+cd.nc*2, cd.nc);
  dCopy(cv, x+cd.nc*3, cd.m); 
  dCopy(cl, x+cd.nc*3+cd.m, cd.ml);
  dCopy(ch, x+cd.nc*3+cd.m+cd.ml, cd.mh);

  // evaluate objective function: kinetic energy
  // given: * is inverse
  // f0 = (cn'NM*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + 
  //       cn'Nv +
  //       ct1'T1M*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) +
  //       ct1'T1v +
  //       ct2'T2M*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) +
  //       ct2'T2v +
  //       cv'JM*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) +
  //       cv'Jvv +
  //       cl'JlM*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) +
  //       cl'Jlv +
  //       ch'JhM*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) +
  //       ch'Jhv +
  //       hfext'M*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) +
  //       hfext'v +
  //       v'(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + v'v)*0.5
  // gf0/cn =  NM*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + Nv
  // gf0/ct1 = T1M*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + T1v
  // gf0/ct2 = T2M*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + T2v
  // gf0/cv =  JM*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + Jv
  // gf0/cl =  JlM*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + Jlv
  // gf0/ch =  JhM*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + Jhv

  if (idx == 0)
  {
    // calculate v^{n+1}
    dReal* vnp1 = cd.vnp1;
    dSetZero(cd.worknb6_2, cd.nb*6);
    multSparseMatrix1(cd.worknb6_2, cd.N, cn, cd.nc, cd.nb, 1, cd.uniPairs);
    multSparseMatrix1(cd.worknb6, cd.T1, ct1, cd.nc, cd.nb, 1, cd.uniPairs);
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
    multSparseMatrix1(cd.worknb6, cd.T2, ct2, cd.nc, cd.nb, 1, cd.uniPairs);
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
    multSparseMatrix1(cd.worknb6, cd.J, cv, cd.m, cd.nb, 1, cd.biPairs); 
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
    multSparseMatrix1(cd.worknb6, cd.Jl, cl, cd.ml, cd.nb, 1, cd.biPairs);
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
    multSparseMatrix1(cd.worknb6, cd.Jh, ch, cd.mh, cd.nb, 1, cd.biPairs);
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.worknb6, cd.nb*6);
    dAdd(cd.worknb6_2, cd.worknb6_2, cd.hfext, cd.nb*6);
    multGMass0(vnp1, cd.invI, cd.nb, cd.bodies, cd.worknb6_2, 1, true);
    dAdd(vnp1, vnp1, cd.v, cd.nb*6);

    // calculate gradients
    dSetZero(g, n);
    multSparseMatrix0(g, cd.N, vnp1, cd.nc, 1, cd.uniPairs);
    multSparseMatrix0(g + cd.nc, cd.T1, vnp1, cd.nc, 1, cd.uniPairs);
    multSparseMatrix0(g + cd.nc*2, cd.T2, vnp1, cd.nc, 1, cd.uniPairs);
    multSparseMatrix0(g + cd.nc*3, cd.J, vnp1, cd.m, 1, cd.biPairs);
    multSparseMatrix0(g + cd.nc*3+cd.m, cd.Jl, vnp1, cd.ml, 1, cd.biPairs);
    multSparseMatrix0(g + cd.nc*3+cd.m+cd.ml, cd.Jh, vnp1, cd.mh, 1, cd.biPairs); 
  }
  // gradients for non-negativity constraints
  else if (idx <= cd.nc)
  {
    dSetZero(g, n);
    g[idx-1] = (dReal) -1.0;
  }
  // gradients for non-interpenetration constraints
  // fi = N(M*(N'cn + T1'ct1 + T2'ct2 + J'cv + Jl'cl + Jh'ch + hfext) + v) >= c
  // gfi/cn = NM*N'
  // gfi/ct1 = NM*T1'
  // gfi/ct2 = NM*T2'
  // gfi/cv = NM*J'
  // gfi/cl = NM*Jl'
  // gfi/ch = NM*Jh'
  else if (idx <= cd.nc*2)
  {
    // get the non-interpenetration index
    int npidx = idx - cd.nc - 1;

    // multiply N by the stack
    multSparseMatrix0(cd.NStack, cd.N, cd.stack2, cd.nc, n, cd.uniPairs);
    for (int i=0; i< n; i++)
      g[i] = -cd.NStack[npidx*n + i];
  }
  // gradients on normal impulse constraint
  else if (idx <= cd.nc*2+1)
  {
    dSetZero(g, n);
    for (int i=0; i< cd.nc; i++)
      g[i] = (dReal) 1.0;
  }
  // Coulomb friction constraints
  else
  {
    dSetZero(g, n);
    int i = idx - cd.nc*2 - cd.m*2 - 2;
    g[i] = -cd.mu[i]*cd.mu[i]*cn[i];
    g[i+cd.nc] = ct1[i];
    g[i+cd.nc*2] = ct2[i];
  }
}

//****************************************************************************
// numerical Hessian functions for testing/debugging purposes
//****************************************************************************
/*static bool copt_nhess1(dReal* x, int n, int idx, dReal* H, void* data)
{
  //COptData& cd = *((COptData*) data);

  // setup for numerical Hessian
  const dReal h = (dReal) 1e-6;
  const dReal ih2 = (dReal) 0.5/h;
  shared_array<dReal> xx(new dReal[n]);
  dCopy(xx.get(), x, n);

  // setup temporary arrays
  shared_array<dReal> v1(new dReal[n]), v2(new dReal[n]);

  // do numerical Hessian
  for (int i=0; i< n; i++)
  {
    xx[i] += h;
    copt_ngrad1(xx.get(), n, idx, v1.get(), data);
    xx[i] -= h*2;
    copt_ngrad1(xx.get(), n, idx, v2.get(), data);
    xx[i] += h;

    // set the i'th column of the Hessian
    for (int j=0; j< n; j++)
      H[j*n+i] = (v1[j] - v2[j])*ih2;
  }

  // average values of the Hessian
  for (int i=0; i< n; i++)
    for (int j=i+1; j< n; j++)
      H[j*n+i] = H[i*n+j] = (dReal) 0.5 * (H[j*n+i] + H[i*n+j]);

  return true;
}*/

/*static bool copt_nhess2(dReal* x, int n, int idx, dReal* H, void* data)
{
  //COptData& cd = *((COptData*) data);

  // setup for numerical Hessian
  const dReal h = (dReal) 1e-6;
  const dReal ih2 = (dReal) 0.5/h;
  shared_array<dReal> xx(new dReal[n]);
  dCopy(xx.get(), x, n);

  // setup temporary arrays
  shared_array<dReal> v1(new dReal[n]), v2(new dReal[n]);

  // do numerical Hessian
  for (int i=0; i< n; i++)
  {
    xx[i] += h;
    copt_ngrad2(xx.get(), n, idx, v1.get(), data);
    xx[i] -= h*2;
    copt_ngrad2(xx.get(), n, idx, v2.get(), data);
    xx[i] += h;

    // set the i'th column of the Hessian
    for (int j=0; j< n; j++)
      H[j*n+i] = (v1[j] - v2[j])*ih2;
  }

  // average values of the Hessians
  for (int i=0; i< n; i++)
    for (int j=i+1; j< n; j++)
      H[j*n+i] = H[i*n+j] = (dReal) 0.5 * (H[j*n+i] + H[i*n+j]);

  return true;
}*/

//****************************************************************************
// convex optimization Hessian functions 
//****************************************************************************
static bool copt_hess1(dReal* x, int n, int idx, dReal* H, void* data)
{
  COptData& cd = *((COptData*) data);
  //const dReal S_BUFFER = dSqrt(std::numeric_limits<dReal>::epsilon());

  // determine cv, cn, ct1, ct2
  dReal* cn = cd.cn;
  dReal* cv = cd.cv;
  dReal* ct1 = cd.ct1;
  dReal* ct2 = cd.ct2;
  dCopy(cn, x, cd.nc);
  dCopy(ct1, x+cd.nc, cd.nc);
  dCopy(ct2, x+cd.nc*2, cd.nc);
  dCopy(cv, x+cd.nc*3, cd.m); 

  // Hessian is unchanging for objective function 
  if (idx == 0)
  {
    dCopy(H, cd.ohess1, n*n);
    return true;
  }
  // Hessians for non-negativity constraints
  // Hessians for non-interpenetration constraints  
  // Hessians for inequality constraints on cv
  // Hessians on normal impulse constraint
  else
    return false;
}

static bool copt_hess2(dReal* x, int n, int idx, dReal* H, void* data)
{
  COptData& cd = *((COptData*) data);
  //const dReal S_BUFFER = dSqrt(std::numeric_limits<dReal>::epsilon());

  // determine cv, cn, ct1, ct2
  dReal* cn = cd.cn;
  dReal* cv = cd.cv;
  dReal* ct1 = cd.ct1;
  dReal* ct2 = cd.ct2;
  dCopy(cn, x, cd.nc);
  dCopy(ct1, x+cd.nc, cd.nc);
  dCopy(ct2, x+cd.nc*2, cd.nc);
  dCopy(cv, x+cd.nc*3, cd.m); 

  // evaluate objective function: kinetic energy
  if (idx == 0)
  {
    dCopy(H, cd.ohess2, n*n);
    return true;
  }
  // Hessians for non-negativity constraints
  // Hessians for non-interpenetration constraints  
  // Hessians for inequality constraints on cv
  // Hessians on normal impulse constraint
  else if (idx <= cd.nc*2+1)
  {
    return false;
  }
  // Hessians for Coulomb friction constraints
  else
  {
    dSetZero(H, n*n);
    int i = idx - cd.nc*2 - cd.m*2 - 2;
    H[i*n + i] = -cd.mu[i]*cd.mu[i];
    int j = i+cd.nc;
    int k = i+cd.nc*2;
    H[j*n + j] = (dReal) 1.0;
    H[k*n + k] = (dReal) 1.0;
    return true;
  }
}

//****************************************************************************
// a convex optimization version of dxInternalStepIsland
//****************************************************************************
void dxRobustStepIsland(dxWorldProcessContext* context, dxWorld *world, 
                        dxBody * const *body, int nb,
                        dxJoint * const *joint, int nj, dReal stepsize)
{
  const dReal EPS = world->rs.eps;
  const dReal EPS_FEAS = world->rs.eps_feas;
  const int MAX_ITERATIONS = world->rs.max_iterations;

  // get inverse of stepsize
  dReal stepsize1 = dRecip(stepsize);

  // determine number of contact constraints
  int nc = 0;
  for (int i=0; i< nj; i++)
    if (joint[i]->type() == dJointTypeContact)
      nc++; 

  // if there are no contact constraints, do the original island stepper
  if (nc == 0)
  {
    dInternalStepIsland(context, world, body, nb, joint, nj, stepsize);
    return;
  }

  // number all bodies in the body list - set their tag values
  for (int i=0; i<nb; i++) 
    body[i]->tag = i;

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator. invI are vertically stacked 3x4 matrices, one per body.
  shared_array<dReal> fwdJ(new dReal[3*3*nb]);
  ALLOCA(dReal,invI,3*nb*4*sizeof(dReal));
  for (int i=0; i<nb; i++) 
  {
    dReal tmp[12];

    // compute inverse inertia tensor in global frame
    dMULTIPLY2_333 (tmp,body[i]->invI,body[i]->posr.R);
    dMULTIPLY0_333 (invI+i*12,body[i]->posr.R,tmp);

    if (body[i]->flags & dxBodyGyroscopic) 
    {
        dMatrix3 I;

        // compute inertia tensor in global frame
        dMULTIPLY2_333 (tmp,body[i]->mass.I,body[i]->posr.R);
        dMULTIPLY0_333 (I,body[i]->posr.R,tmp);

        // copy inertia tensor
        fwdJ[i*9] = I[0];
        fwdJ[i*9+1] = I[1];
        fwdJ[i*9+2] = I[2];
        fwdJ[i*9+3] = I[4];
        fwdJ[i*9+4] = I[5];
        fwdJ[i*9+5] = I[6];
        fwdJ[i*9+6] = I[8];
        fwdJ[i*9+7] = I[9];
        fwdJ[i*9+8] = I[10];

        // compute rotational force
        dMULTIPLY0_331 (tmp,I,body[i]->avel);
        dCROSS (body[i]->tacc,-=,body[i]->avel,tmp);
    }
  }

  // copy inverse inertia tensors to 3x3 matrices; this is unnecessary,
  // but I don't want to tease apart the 4x3 matrix code...
  shared_array<dReal> invJ(new dReal[9*nb]);
  for (int i=0; i< nb; i++)
  {
    invJ[i*9+0] = invI[i*12+0];
    invJ[i*9+1] = invI[i*12+1];
    invJ[i*9+2] = invI[i*12+2];
    invJ[i*9+3] = invI[i*12+4];
    invJ[i*9+4] = invI[i*12+5];
    invJ[i*9+5] = invI[i*12+6];
    invJ[i*9+6] = invI[i*12+8];
    invJ[i*9+7] = invI[i*12+9];
    invJ[i*9+8] = invI[i*12+10];
  }

  // add the gravity force to all bodies
  for (int i=0; i<nb; i++) {
    if ((body[i]->flags & dxBodyNoGravity)==0) 
    {
      body[i]->facc[0] += body[i]->mass.mass * world->gravity[0];
      body[i]->facc[1] += body[i]->mass.mass * world->gravity[1];
      body[i]->facc[2] += body[i]->mass.mass * world->gravity[2];
    }
  }

  // setup vectors and matrices
  shared_array<dReal> mu(new dReal[nc]);
  shared_array<dReal> N(new dReal[nc*6*2]);
  shared_array<dReal> n_c(new dReal[nc]);
  shared_array<dReal> T1(new dReal[nc*6*2]);
  shared_array<dReal> T2(new dReal[nc*6*2]);
  shared_array<dReal> fext(new dReal[nb*6]);
  shared_array<dReal> v(new dReal[nb*6]);

  // determine the body pairs for unilateral and bilateral constraints
  vector<BodyPair> uniPairs, biPairs, loPairs, hiPairs;
  determineBodyPairs(joint, nj, uniPairs, biPairs, loPairs, hiPairs);

  // determine joint constraint Jacobian
  int m, ml, mh;
  shared_array<dReal> J, Jl, Jh, j_c, jl_c, jh_c;
  getJ_c(world, joint, nj, stepsize1, m, J, j_c, ml, Jl, jl_c, mh, Jh, jh_c);

  // setup vector of friction coefficients
  getMu(joint, nc, mu.get());

  // determine generalized normals
  getN_c(world, joint, nj, stepsize1, N.get(), n_c.get());

  // determine generalized tangential directions
  getT1(joint, nj, T1.get());
  getT2(joint, nj, T2.get());

  // determine generalized external forces and scale them by the step size
  getFExt(body, nb, fext.get());
  for (int i=0; i< nb*6; i++)
    fext[i] *= stepsize;

  // setup generalized velocities
  getVelocities(body, nb, v.get());

  // setup the convex optimization problem (w/o friction)
  COptData copt(m, ml, mh, nc, nb);
  copt.I = fwdJ.get();
  copt.invI = invJ.get();
  copt.J = J.get();
  copt.j_c = j_c.get();
  copt.Jl = Jl.get();
  copt.jl_c = jl_c.get();
  copt.Jh = Jh.get();
  copt.jh_c = jh_c.get();
  copt.N = N.get();
  copt.n_c = n_c.get();
  copt.T1 = T1.get();
  copt.T2 = T2.get();
  copt.hfext = fext.get();
  copt.v = v.get();
  copt.kappa = 0;
  copt.mu = mu.get();
  copt.biPairs = biPairs;
  copt.uniPairs = uniPairs;
  copt.bodies = body;

  // setup the convex optimization parameters
  CvxOptParams cparams;
  cparams.max_iterations = std::numeric_limits<int>::max();
  cparams.eps = EPS;
  cparams.data = (void*) &copt;
  cparams.fx = copt_fx1;
  cparams.grad = copt_grad1;
  cparams.hess = copt_hess1; 

  // setup the n+m+ml+mh and 3n+m+ml+mh - dimensional vectors of impulses to 
  // solve for
  int nvar1 = nc + m + ml + mh;
  int nvar2 = 3*nc + m + ml + mh;
  shared_array<dReal> x1(new dReal[nvar1]);
  shared_array<dReal> x2(new dReal[nvar2]);
  dSetZero(x1.get(), nvar1);
  dSetZero(x2.get(), nvar2);

  // setup # of inequality constraints
  cparams.m = 2*nc + ml + mh;

  // ****************************************************************
  // form the A matrix and b vector for the joint constraints
  // ****************************************************************
  shared_array<dReal> A1(new dReal[m*nvar1]);
  shared_array<dReal> b(new dReal[m]);
  shared_array<dReal> workm(new dReal[m]);
  shared_array<dReal> worknb6(new dReal[nb*6]);
  shared_array<dReal> worknb6n(new dReal[nb*6*nvar2]);
  shared_array<dReal> worknb6n2(new dReal[nb*6*nvar2]);
  // set b = j_c - J(v + M^{-1}hfext)
  dCopy(b.get(), j_c.get(), m);
  multGMass0(worknb6.get(), invJ.get(), nb, body, fext.get(), 1, true); 
  dAdd(worknb6.get(), worknb6.get(), v.get(), nb*6);
  multSparseMatrix0(workm.get(), J.get(), worknb6.get(), m, 1, biPairs);
  dSub(b.get(), b.get(), workm.get(), m);
  // set A = J*M^{-1}*[N' J' Jl' Jh']
  formStack1(worknb6n.get(), N.get(), J.get(), Jl.get(), Jh.get(), nb, nc, m, ml, mh, uniPairs, biPairs);
  multGMass0(copt.stack1, invJ.get(), nb, body, worknb6n.get(), nvar1, true);
  multSparseMatrix0(A1.get(), J.get(), copt.stack1, m, nvar1, biPairs); 
  cparams.A = A1.get();
  cparams.b = b.get();
  cparams.nu_len = 2*m;

  #ifdef DEBUG
  std::cout << "******** robust stepper **********" << std::endl;
  std::cout << "  -- N: " << std::endl;
  printMatrix(N.get(), nc, 12);
  std::cout << "-- about to find initial feasible point" << std::endl;
  #endif

  // determine Hessian of objective function for phase I and phase II
  // -- df0^2/dcncn = NM*N'
  // -- df0^2/dct1ct1 = T1M*T1'
  // -- df0^2/dct2ct2 = T2M*T2'
  // -- df0^2/dcvcv = JM*J'
  // -- df0^2/dclcl = JlM*Jl'
  // -- df0^2/dchch = JhM*Jh'
  shared_array<dReal> NMN, T1MT1, T2MT2, JMJ, JlMJl, JhMJh;
  shared_array<dReal> hess1(new dReal[nvar1*nvar1]);
  shared_array<dReal> hess2(new dReal[nvar2*nvar2]);
  dSetZero(hess1.get(), nvar1*nvar1);
  dSetZero(hess2.get(), nvar2*nvar2);
  formXiMXT(NMN, N.get(), nb, nc, invJ.get(), body, uniPairs);
  formXiMXT(T1MT1, T1.get(), nb, nc, invJ.get(), body, uniPairs);
  formXiMXT(T2MT2, T2.get(), nb, nc, invJ.get(), body, uniPairs);
  formXiMXT(JMJ, J.get(), nb, m, invJ.get(), body, biPairs);
  formXiMXT(JlMJl, Jl.get(), nb, ml, invJ.get(), body, loPairs);
  formXiMXT(JhMJh, Jh.get(), nb, mh, invJ.get(), body, hiPairs);
  dSetSubMat0(hess1.get(), nvar1, NMN.get(), nc, nc, 0, 0);
  dSetSubMat0(hess1.get(), nvar1, JMJ.get(), m, m, nc, nc);
  dSetSubMat0(hess1.get(), nvar1, JlMJl.get(), ml, ml, nc+m, nc+m);
  dSetSubMat0(hess1.get(), nvar1, JhMJh.get(), mh, mh, nc+m+ml, nc+m+ml);
  dSetSubMat0(hess2.get(), nvar2, NMN.get(), nc, nc, 0, 0);
  dSetSubMat0(hess2.get(), nvar2, T1MT1.get(), nc, nc, nc, nc);
  dSetSubMat0(hess2.get(), nvar2, T2MT2.get(), nc, nc, nc*2, nc*2);
  dSetSubMat0(hess2.get(), nvar2, JMJ.get(), m, m, nc*3, nc*3);
  dSetSubMat0(hess2.get(), nvar2, JlMJl.get(), ml, ml, nc*3+m, nc*3+m);
  dSetSubMat0(hess2.get(), nvar2, JhMJh.get(), mh, mh, nc*3+m+ml, nc*3+m+ml);
  copt.ohess1 = hess1.get();
  copt.ohess2 = hess2.get();

  // find an initial feasible point -- we constrain frictional impulses to be
  // zero
  if (!dMakeFeasibleConvex(cparams, EPS_FEAS, x1.get(), nvar1))
  {
    std::cout << "unable to find initial feasible point!" << std::endl;
    return;
  }

  #ifdef DEBUG
  std::cout << "  -- required " << cparams.iterations << " iterations" << std::endl;
  std::cout << "-- about to solve convex optimization problem w/o friction" << std::endl;
  #endif

  // solve the convex optimization problem (w/o friction) -- don't check that
  // we were successful; we will still have some sort of solution on failure
  dOptimizeConvexPrimalDual(cparams, EPS_FEAS, x1.get(), nvar1);

  // reset number of inequality constraints 
  cparams.m = 3*nc+ml+mh+1;

  // setup kappa
  copt.kappa = (dReal) 0.0;
  for (int i=0; i< nc; i++)
    copt.kappa += x1[i];

  // setup x2 from x
  memcpy(x2.get(), x1.get(), nc*sizeof(dReal));
  dSetZero(x2.get()+nc, nc*2);
  memcpy(x2.get()+nc*3, x1.get()+nc, (m+ml+mh)*sizeof(dReal));

  // setup A2 = J*M^{-1}*[N' T1' T2' J' Jl' Jh']
  shared_array<dReal> A2(new dReal[m*nvar2]);
  formStack2(worknb6n.get(), N.get(), T1.get(), T2.get(), J.get(), Jl.get(), Jh.get(), nb, nc, m, ml, mh, uniPairs, biPairs);
  multGMass0(copt.stack2, invJ.get(), nb, body, worknb6n.get(), nvar2, true);
  multSparseMatrix0(A2.get(), J.get(), copt.stack2, m, nvar2, biPairs); 
  cparams.A = A2.get();

  // resolve the convex optimization problem with friction; again, not 
  // necessary to check that we are successful
  #ifdef DEBUG
  std::cout << "  -- required " << cparams.iterations << " iterations" << std::endl;
  std::cout << "    solution: ";
  printMatrix(x1.get(), 1, nc+m+ml+mh);
  std::cout << "-- about to solve convex optimization problem w/ friction" << std::endl;
  #endif
  cparams.fx = copt_fx2;
  cparams.grad = copt_grad2;
  cparams.hess = copt_hess2; 
  cparams.max_iterations = MAX_ITERATIONS;
  dOptimizeConvexPrimalDual(cparams, EPS_FEAS, x2.get(), nvar2);

  // determine cn, ct1, ct2, cv, cl, ch
  dReal* cn = copt.cn;
  dReal* cv = copt.cv;
  dReal* cl = copt.cl;
  dReal* ch = copt.ch;
  dReal* ct1 = copt.ct1;
  dReal* ct2 = copt.ct2;
  dCopy(cn, x2.get(), nc);
  dCopy(ct1, x2.get()+nc, nc);
  dCopy(ct2, x2.get()+nc*2, nc);
  dCopy(cv, x2.get()+nc*3, m); 
  dCopy(cl, x2.get()+nc*3+m, ml);
  dCopy(ch, x2.get()+nc*3+m+ml, mh);

  // compute new velocities
  dReal* vnp1 = copt.vnp1;
  dSetZero(copt.worknb6_2, nb*6);
  multSparseMatrix1(copt.worknb6_2, N.get(), cn, nc, nb, 1, uniPairs);
  multSparseMatrix1(copt.worknb6, T1.get(), ct1, nc, nb, 1, uniPairs);
  dAdd(copt.worknb6_2, copt.worknb6_2, copt.worknb6, nb*6);
  multSparseMatrix1(copt.worknb6, T2.get(), ct2, nc, nb, 1, uniPairs);
  dAdd(copt.worknb6_2, copt.worknb6_2, copt.worknb6, nb*6);
  multSparseMatrix1(copt.worknb6, J.get(), cv, m, nb, 1, biPairs); 
  dAdd(copt.worknb6_2, copt.worknb6_2, copt.worknb6, nb*6);
  multSparseMatrix1(copt.worknb6, Jl.get(), cl, ml, nb, 1, biPairs);
  dAdd(copt.worknb6_2, copt.worknb6_2, copt.worknb6, nb*6);
  multSparseMatrix1(copt.worknb6, Jh.get(), ch, mh, nb, 1, biPairs);
  dAdd(copt.worknb6_2, copt.worknb6_2, copt.worknb6, nb*6);
  dAdd(copt.worknb6_2, copt.worknb6_2, fext.get(), nb*6);
  multGMass0(vnp1, invJ.get(), nb, body, copt.worknb6_2, 1, true);
  dAdd(vnp1, vnp1, v.get(), nb*6);

  #ifdef DEBUG
  std::cout << "  -- required " << cparams.iterations << " iterations" << std::endl;
  // add in inv(M)*h*fext to v
  multGMass0(copt.worknb6_2, invJ.get(), nb, body, fext.get(), 1, true);
  dAdd(v.get(), v.get(), copt.worknb6_2, nb*6);
  // compute energy before and after
  dReal* Mv = copt.Mv;
  multGMass0(Mv, fwdJ.get(), nb, body, v.get(), 1, false);
  dReal oke = dDot(Mv, v.get(), nb*6) * (dReal) 0.5; 
  multGMass0(Mv, fwdJ.get(), nb, body, vnp1, 1, false);
  dReal nke = dDot(Mv, vnp1, nb*6) * (dReal) 0.5; 
  std::cout << "old kinetic energy: " << oke << std::endl;
  std::cout << "new kinetic energy: " << nke << std::endl;
  if (nke > oke)
    std::cout << " *** ENERGY INCREASE ***" << std::endl;
  #endif

  // update body velocities
  setVelocities(vnp1, body, nb);

  #ifdef DEBUG
  std::cout << "new velocity vector: ";
  printMatrix(vnp1, 1, nb*6);
  dReal* Nv = copt.Nv;
  multSparseMatrix0(Nv, copt.N, vnp1, nc, 1, uniPairs);  
  std::cout << "N*v: ";
  printMatrix(Nv, 1, nc);
  #endif

  // TODO: determine impulses on bodies due to unilateral / bilateral constraints
  for (int i=0; i< nj; i++)
  {
    // see whether joint feedback is requested
    dJointFeedback *fb = joint[i]->feedback;
    if (fb)
    {
    }
  }

  // step bodies forward using new velocities
  for (int i=0; i<nb; i++) 
    dxStepBody (body[i],stepsize);

  // zero all force accumulators
  for (int i=0; i<nb; i++) 
  {
    body[i]->facc_last[0] = body[i]->facc[0];
    body[i]->facc_last[1] = body[i]->facc[1];
    body[i]->facc_last[2] = body[i]->facc[2];
    body[i]->tacc_last[0] = body[i]->tacc[0];
    body[i]->tacc_last[1] = body[i]->tacc[1];
    body[i]->tacc_last[2] = body[i]->tacc[2];
    body[i]->facc[0] = 0;
    body[i]->facc[1] = 0;
    body[i]->facc[2] = 0;
    body[i]->facc[3] = 0;
    body[i]->tacc[0] = 0;
    body[i]->tacc[1] = 0;
    body[i]->tacc[2] = 0;
    body[i]->tacc[3] = 0;
  }

/*

  // this will be set to the force due to the constraints
  ALLOCA(dReal,cforce,nb*8*sizeof(dReal));
  dSetZero (cforce,nb*8);

  // if there are constraints, compute cforce
  if (m > 0) {
    // create a constraint equation right hand side vector `c', a constraint
    // force mixing vector `cfm', and LCP low and high bound vectors, and an
    // 'findex' vector.

    // compute the constraint force `cforce'
#   ifdef TIMING
    dTimerNow ("compute constraint force");
#   endif
    // compute cforce = J'*lambda
    for (i=0; i<nj; i++) {
      dReal *JJ = J + 2*8*ofs[i];
      dxBody* b1 = joint[i]->node[0].body;
      dxBody* b2 = joint[i]->node[1].body;
      dJointFeedback *fb = joint[i]->feedback;

      if (fb) {
        // the user has requested feedback on the amount of force that this
        // joint is applying to the bodies. we use a slightly slower
        // computation that splits out the force components and puts them
        // in the feedback structure.
        dReal data[8];

        Multiply1_8q1 (data, JJ, lambda+ofs[i], info[i].m);
        dReal *cf1 = cforce + 8*b1->tag;
        cf1[0] += (fb->f1[0] = data[0]);
        cf1[1] += (fb->f1[1] = data[1]);
        cf1[2] += (fb->f1[2] = data[2]);
        cf1[4] += (fb->t1[0] = data[4]);
        cf1[5] += (fb->t1[1] = data[5]);
        cf1[6] += (fb->t1[2] = data[6]);
        if (b2){
          Multiply1_8q1 (data, JJ + 8*info[i].m, lambda+ofs[i], info[i].m);
          dReal *cf2 = cforce + 8*b2->tag;
          cf2[0] += (fb->f2[0] = data[0]);
          cf2[1] += (fb->f2[1] = data[1]);
          cf2[2] += (fb->f2[2] = data[2]);
          cf2[4] += (fb->t2[0] = data[4]);
          cf2[5] += (fb->t2[1] = data[5]);
          cf2[6] += (fb->t2[2] = data[6]);
	}
      }
      else {
	// no feedback is required, let's compute cforce the faster way
	MultiplyAdd1_8q1 (cforce + 8*b1->tag,JJ, lambda+ofs[i], info[i].m);
	if (b2) {
	  MultiplyAdd1_8q1 (cforce + 8*b2->tag,
			    JJ + 8*info[i].m, lambda+ofs[i], info[i].m);
	}
      }
    }
  }

  // add fe to cforce
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) cforce[i*8+j] += body[i]->facc[j];
    for (j=0; j<3; j++) cforce[i*8+4+j] += body[i]->tacc[j];
  }
  // multiply cforce by stepsize
  for (i=0; i < nb*8; i++) cforce[i] *= stepsize;
  // add invM * cforce to the body velocity
  for (i=0; i<nb; i++) {
    dReal body_invMass = body[i]->invMass;
    dReal *body_invI = invI + i*12;
    for (j=0; j<3; j++) body[i]->lvel[j] += body_invMass * cforce[i*8+j];
    dMULTIPLYADD0_331 (body[i]->avel,body_invI,cforce+i*8+4);
  }

  }
*/
}

//****************************************************************************

void dRobustStepIsland(dxWorldProcessContext* context, dxWorld *world, dxBody * const *body, int nb,
			  dxJoint * const *joint, int nj, dReal stepsize)
{

#ifdef dUSE_MALLOC_FOR_ALLOCA
  dMemoryFlag = d_MEMORY_OK;
#endif

  dxRobustStepIsland(context, world,body,nb,joint,nj,stepsize);

#ifdef dUSE_MALLOC_FOR_ALLOCA
    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY) {
      REPORT_OUT_OF_MEMORY;
      return;
    }
#endif
}

void TEST()
{
  dReal sparse[12*2] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 
                         12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 };
  dReal full[24*2] = { 0, 1, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 6, 7, 8, 9, 10, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 };
  dReal v[48] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47 };
  dReal v2[4] = {10, 20, 30, 40};
  vector<BodyPair> pairs;
  pairs.push_back(BodyPair(0, 2));
  pairs.push_back(BodyPair(2, 3));
  dReal result[24];
  multSparseMatrix0(result, sparse, v, 2, 2, pairs);
  std::cout << "result: ";
  printMatrix(result, 2, 2);
  dNPMultiply0(result, full, v, 2, 24, 2); 
  std::cout << "result: ";
  printMatrix(result, 2, 2);
  multSparseMatrix1(result, sparse, v2, 2, 4, 2, pairs);
  std::cout << "result: ";
  printMatrix(result, 2, 24);
  dNPMultiply1(result, full, v2, 24, 2, 2);
  printMatrix(result, 2, 24);

}

