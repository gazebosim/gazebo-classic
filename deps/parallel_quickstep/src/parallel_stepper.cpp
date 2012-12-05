#include <ode/objects.h>
#include <ode/ode.h>
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include <ode/misc.h>
#include "objects.h"
#include "config.h"
#include "joints/joint.h"
#include "lcp.h"
#include "util.h"

#include <parallel_common.h>
#include <parallel_utils.h>
#include <parallel_stepper.h>

#if defined(USE_CUDA)
#include <cuda_solver.h>
#ifdef CUDA_DOUBLESUPPORT
typedef parallel_ode::CudaPGSSolver<dReal,dReal> SolverType;
#else
typedef parallel_ode::CudaPGSSolver<float,dReal> SolverType;
#endif
#elif defined(USE_OPENCL)
#include <opencl_solver.h>
typedef parallel_ode::OpenCLPGSSolver<dReal> SolverType;
#elif defined(USE_OPENMP)
#include <openmp_solver.h>
typedef parallel_ode::OpenMPPGSSolver<dReal> SolverType;
#endif

#if PARALLEL_ENABLED
static SolverType parallelSolver;
#endif

typedef const dReal *dRealPtr;
typedef dReal *dRealMutablePtr;

template
void compute_invM_JT<dReal> (int m, dRealPtr J, dRealMutablePtr iMJ, int *jb,
                             dxBody * const *body, const dRealPtr invI);

template
void compute_Adcfm_b<dReal> (int m, dReal sor_w, dRealMutablePtr J, dRealPtr iMJ, int* jb, dRealPtr cfm,
                             dRealMutablePtr Adcfm, dRealMutablePtr b);

#define RANDOMLY_REORDER_CONSTRAINTS 1

#ifdef BENCHMARKING
static unsigned long totalConstraints = 0;
static unsigned long totalBodies = 0;
static const unsigned long maxBenchmarkIters = 300;
static const unsigned long minBenchmarkIters = 50;
static unsigned long benchmarkRoundsRemaining = 5;
static unsigned long benchmarkIters = 0;
static int sorItersMult = 2;
static dReal sorParamMult = .85;

static void benchmarkIteration( dxWorld *world) {
   ++benchmarkIters;

  if (benchmarkIters >= maxBenchmarkIters) {
    printf("Iterations = %d, Avg Constraints  = %f, Avg Bodies = %f\n",
           dWorldGetQuickStepNumIterations( world ),
           (dReal)totalConstraints / (dReal)(maxBenchmarkIters - minBenchmarkIters),
           (dReal)totalBodies / (dReal)(maxBenchmarkIters - minBenchmarkIters) );

    dTimerReport (stdout,1);

    --benchmarkRoundsRemaining;
    if( benchmarkRoundsRemaining == 0 ) exit(0);

    totalConstraints=0;
    totalBodies=0;
    benchmarkIters=0;
    dWorldSetQuickStepNumIterations( world, (int)(dWorldGetQuickStepNumIterations( world ) * sorItersMult) );
    dWorldSetQuickStepW( world, (dReal)(dWorldGetQuickStepW( world ) * sorParamMult) );
  }
}
#endif

struct IndexError {
  int index;		// row index
};

template<typename T>
void compute_invM_JT (int m, const T* J, T* iMJ, int *jb,
                      dxBody * const *body, const T* invI)
{
  IFTIMING( dTimerNow("compute invM/JT ") );

  // precompute iMJ = inv(M)*J'
  dRealMutablePtr iMJ_ptr = iMJ;
  dRealPtr J_ptr = J;
  for (int i=0; i<m; J_ptr += 12, iMJ_ptr += 12, i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    dReal k1 = body[b1]->invMass;
    for (int j=0; j<3; j++) iMJ_ptr[j] = k1*J_ptr[j];
    const dReal *invIrow1 = invI + 12*b1;
    dMultiply0_331 (iMJ_ptr + 3, invIrow1, J_ptr + 3);
    if (b2 >= 0) {
      dReal k2 = body[b2]->invMass;
      for (int j=0; j<3; j++) iMJ_ptr[j+6] = k2*J_ptr[j+6];
      const dReal *invIrow2 = invI + 12*b2;
      dMultiply0_331 (iMJ_ptr + 9, invIrow2, J_ptr + 9);
    }
  }
}

template<typename T>
void compute_Adcfm_b (int m, T sor_w, T* J, const T* iMJ, int* jb, const T* cfm,
                             T* Adcfm, T* b)
{
  IFTIMING( dTimerNow("compute Adcfm/b") );
  {
    // precompute 1 / diagonals of A
    dRealPtr iMJ_ptr = iMJ;
    dRealPtr J_ptr = J;
    for (int i=0; i<m; J_ptr += 12, iMJ_ptr += 12, i++) {
      dReal sum = 0;
      for (int j=0; j<6; j++) sum += iMJ_ptr[j] * J_ptr[j];
      if (jb[i*2+1] >= 0) {
        for (int k=6; k<12; k++) sum += iMJ_ptr[k] * J_ptr[k];
      }
      Adcfm[i] = sor_w / (sum + cfm[i]);
    }
  }

  {
    // scale J and b by Ad
    dRealMutablePtr J_ptr = J;
    for (int i=0; i<m; J_ptr += 12, i++) {
      dReal Ad_i = Adcfm[i];
      for (int j=0; j<12; j++) {
        J_ptr[j] *= Ad_i;
      }
      b[i] *= Ad_i;
      // scale Ad by CFM. N.B. this should be done last since it is used above
      Adcfm[i] = Ad_i * cfm[i];
    }
  }
}

static void worldSolve(dxWorldProcessContext *context,
  const int m, const int nb, dRealMutablePtr J, int *jb, dxBody * const *body,
  dRealPtr invI, dRealMutablePtr lambda, dRealMutablePtr fc, dRealMutablePtr b,
  dRealPtr lo, dRealPtr hi, dRealPtr cfm, dRealMutablePtr iMJ, dRealMutablePtr Ad, const int *findex,
  const dxQuickStepParameters *qs)
{
  IFVERBOSE( printf ("(# Bodies, #Constraints) = (%d, %d )\n",nb,m) );

  compute_invM_JT (m,J,iMJ,jb,body,invI);
  dSetZero (lambda,m);
  compute_Adcfm_b (m, qs->w, J, iMJ, jb, cfm, Ad, b);
  dSetZero (fc,nb*6);

  IFTIMING( dTimerNow ("solving LCP problem") );

  // order to solve constraint rows in
  IndexError *order = context->AllocateArray<IndexError> (m);

  {
    // make sure constraints with findex < 0 come first.
    IndexError *orderhead = order, *ordertail = order + (m - 1);

    // Fill the array from both ends
    for (int i=0; i<m; i++) {
      if (findex[i] < 0) {
        orderhead->index = i; // Place them at the front
        ++orderhead;
      } else {
        ordertail->index = i; // Place them at the end
        --ordertail;
      }
    }
    dIASSERT (orderhead-ordertail==1);
  }

#if COMPUTE_ERROR
  dReal *delta_error = context->AllocateArray<dReal> (m);
#endif

  const int num_iterations = qs->num_iterations;
  for (int iteration=0; iteration < num_iterations; iteration++) {

#ifdef RANDOMLY_REORDER_CONSTRAINTS
    if ((iteration & 7) == 0) {
      for (int i=1; i<m; i++) {
        int swapi = dRandInt(i+1);
        IndexError tmp = order[i];
        order[i] = order[swapi];
        order[swapi] = tmp;
      }
    }
#endif

#if COMPUTE_ERROR
    dReal rms_error = 0;
#endif

    for (int i=0; i<m; i++) {
      // @@@ potential optimization: we could pre-sort J and iMJ, thereby
      //     linearizing access to those arrays. hmmm, this does not seem
      //     like a win, but we should think carefully about our memory
      //     access pattern.

      int index = order[i].index;

      dRealMutablePtr fc_ptr1;
      dRealMutablePtr fc_ptr2;
      dReal delta;

      {
        int b1 = jb[index*2];
        int b2 = jb[index*2+1];
        fc_ptr1 = fc + 6*b1;
        fc_ptr2 = (b2 >= 0) ? fc + 6*b2 : NULL;
      }

      dReal old_lambda = lambda[index];

      {
        delta = b[index] - old_lambda*Ad[index];

        dRealPtr J_ptr = J + index*12;
        // @@@ potential optimization: SIMD-ize this and the b2 >= 0 case
        delta -=fc_ptr1[0] * J_ptr[0] + fc_ptr1[1] * J_ptr[1] +
          fc_ptr1[2] * J_ptr[2] + fc_ptr1[3] * J_ptr[3] +
          fc_ptr1[4] * J_ptr[4] + fc_ptr1[5] * J_ptr[5];
        // @@@ potential optimization: handle 1-body constraints in a separate
        //     loop to avoid the cost of test & jump?
        if (fc_ptr2) {
          delta -=fc_ptr2[0] * J_ptr[6] + fc_ptr2[1] * J_ptr[7] +
            fc_ptr2[2] * J_ptr[8] + fc_ptr2[3] * J_ptr[9] +
            fc_ptr2[4] * J_ptr[10] + fc_ptr2[5] * J_ptr[11];
        }
      }


      {
        dReal hi_act, lo_act;

        // set the limits for this constraint.
        // this is the place where the QuickStep method differs from the
        // direct LCP solving method, since that method only performs this
        // limit adjustment once per time step, whereas this method performs
        // once per iteration per constraint row.
        // the constraints are ordered so that all lambda[] values needed have
        // already been computed.
        if (findex[index] >= 0) {
          hi_act = dFabs (hi[index] * lambda[findex[index]]);
          lo_act = -hi_act;
        } else {
          hi_act = hi[index];
          lo_act = lo[index];
        }

        // compute lambda and clamp it to [lo,hi].
        // @@@ potential optimization: does SSE have clamping instructions
        //     to save test+jump penalties here?
        dReal new_lambda = old_lambda + delta;
        if (new_lambda < lo_act) {
          delta = lo_act-old_lambda;
          lambda[index] = lo_act;
        }
        else if (new_lambda > hi_act) {
          delta = hi_act-old_lambda;
          lambda[index] = hi_act;
        }
        else {
          lambda[index] = new_lambda;
        }
      }

#if COMPUTE_ERROR
      rms_error += delta*delta;
      delta_error[index] = dFabs(delta);
#endif

      //@@@ a trick that may or may not help
      //dReal ramp = (1-((dReal)(iteration+1)/(dReal)num_iterations));
      //delta *= ramp;

      {
        dRealPtr iMJ_ptr = iMJ + index*12;
        // update fc.
        // @@@ potential optimization: SIMD for this and the b2 >= 0 case
        fc_ptr1[0] += delta * iMJ_ptr[0];
        fc_ptr1[1] += delta * iMJ_ptr[1];
        fc_ptr1[2] += delta * iMJ_ptr[2];
        fc_ptr1[3] += delta * iMJ_ptr[3];
        fc_ptr1[4] += delta * iMJ_ptr[4];
        fc_ptr1[5] += delta * iMJ_ptr[5];
        // @@@ potential optimization: handle 1-body constraints in a separate
        //     loop to avoid the cost of test & jump?
        if (fc_ptr2) {
          fc_ptr2[0] += delta * iMJ_ptr[6];
          fc_ptr2[1] += delta * iMJ_ptr[7];
          fc_ptr2[2] += delta * iMJ_ptr[8];
          fc_ptr2[3] += delta * iMJ_ptr[9];
          fc_ptr2[4] += delta * iMJ_ptr[10];
          fc_ptr2[5] += delta * iMJ_ptr[11];
        }
      }
    }
#if COMPUTE_ERROR
    if ( iteration == num_iterations - 1 ) {
      // do we need to compute norm across entire solution space (0,m)?
      // since local convergence might produce errors in other nodes?
#ifdef RECOMPUTE_RMS
      // recompute rms_error to be sure swap is not corrupting arrays
      rms_error = 0;
#ifdef USE_1NORM
      for (int i=0; i<m; i++)
      {
        rms_error = dFabs(delta_error[order[i].index]) > rms_error ? dFabs(delta_error[order[i].index]) : rms_error; // 1norm test
      }
#else // use 2 norm
      for (int i=0; i<m; i++)  // use entire solution vector errors
        rms_error += delta_error[order[i].index]*delta_error[order[i].index]; ///(dReal)nRows;
      rms_error = sqrt(rms_error); ///(dReal)nRows;
#endif
#else
      rms_error = sqrt(rms_error); ///(dReal)nRows;
#endif
      printf("(Iteration, Error) = (%d, %20.18f)\n",iteration,rms_error);
    }
#endif
  }
}

void dxParallelProcessIslands (dxWorld *world, dReal stepsize, dstepper_fn_t stepper)
{
  const int sizeelements = 2;

  dxStepWorkingMemory *wmem = world->wmem;
  dIASSERT(wmem != NULL);

  dxWorldProcessContext *context = wmem->GetWorldProcessingContext();

  size_t const *islandreqs = NULL;
  int islandcount;
  int const *islandsizes;
  dxBody *const *body;
  dxJoint *const *joint;
  context->RetrievePreallocations(islandcount, islandsizes, body, joint, islandreqs);

  dxBody *const *bodystart = body;
  dxJoint *const *jointstart = joint;

  int const *const sizesend = islandsizes + islandcount * sizeelements;

  int bcount = 0;
  int jcount = 0;

  for (int const *sizescurr = islandsizes; sizescurr != sizesend; sizescurr += sizeelements) {
    bcount += sizescurr[0];
    jcount += sizescurr[1];
  }

  BEGIN_STATE_SAVE(context, stepperstate) {
    stepper (context,world,bodystart,bcount,jointstart,jcount,stepsize);
  } END_STATE_SAVE(context, stepperstate);

  context->CleanupContext();
  dIASSERT(context->IsStructureValid());
}

//***************************************************************************
// configuration

#define RANDOMLY_REORDER_CONSTRAINTS 1

//****************************************************************************
// special matrix multipliers

// multiply block of B matrix (q x 6) with 12 dReal per row with C vektor (q)
static void Multiply1_12q1 (dReal *A, const dReal *B, const dReal *C, int q)
{
  dIASSERT (q>0 && A && B && C);

  dReal a = 0;
  dReal b = 0;
  dReal c = 0;
  dReal d = 0;
  dReal e = 0;
  dReal f = 0;
  dReal s;

  for(int i=0, k = 0; i<q; k += 12, i++)
  {
    s = C[i]; //C[i] and B[n+k] cannot overlap because its value has been read into a temporary.

    //For the rest of the loop, the only memory dependency (array) is from B[]
    a += B[  k] * s;
    b += B[1+k] * s;
    c += B[2+k] * s;
    d += B[3+k] * s;
    e += B[4+k] * s;
    f += B[5+k] * s;
  }

  A[0] = a;
  A[1] = b;
  A[2] = c;
  A[3] = d;
  A[4] = e;
  A[5] = f;
}

//***************************************************************************
// various common computations involving the matrix J

// compute out = J*in.

static void multiply_J (int m, dRealPtr J, int *jb,
  dRealPtr in, dRealMutablePtr out)
{
  dRealPtr J_ptr = J;
  for (int i=0; i<m; i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    dReal sum = 0;
    dRealPtr in_ptr = in + b1*6;
    for (int j=0; j<6; j++) sum += J_ptr[j] * in_ptr[j];
    J_ptr += 6;
    if (b2 >= 0) {
      in_ptr = in + b2*6;
      for (int j=0; j<6; j++) sum += J_ptr[j] * in_ptr[j];
    }
    J_ptr += 6;
    out[i] = sum;
  }
}

struct dJointWithInfo1
{
  dxJoint *joint;
  dxJoint::Info1 info;
};

void dxParallelQuickStepper( dxWorldProcessContext *context,
                             dxWorld *world, dxBody * const *body, int nb,
                             dxJoint * const *_joint, int _nj, dReal stepsize)
{

  IFTIMING(dTimerStart("preprocessing"));
  const dReal stepsize1 = dRecip(stepsize);

  {
    // number all bodies in the body list - set their tag values
    for (int i=0; i<nb; i++) body[i]->tag = i;
  }

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
  dReal *invI = context->AllocateArray<dReal> (3*4*nb);

  {
    dReal *invIrow = invI;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow += 12, bodycurr++) {
      dMatrix3 tmp;
      dxBody *b = *bodycurr;

      // compute inverse inertia tensor in global frame
      dMultiply2_333 (tmp,b->invI,b->posr.R);
      dMultiply0_333 (invIrow,b->posr.R,tmp);

      if (b->flags & dxBodyGyroscopic) {
        dMatrix3 I;
        // compute inertia tensor in global frame
        dMultiply2_333 (tmp,b->mass.I,b->posr.R);
        dMultiply0_333 (I,b->posr.R,tmp);
        // compute rotational force
        dMultiply0_331 (tmp,I,b->avel);
        dSubtractVectorCross3(b->tacc,b->avel,tmp);
      }
    }
  }

  // get the masses for every body
  dReal *invM = context->AllocateArray<dReal> (nb);
  {
    dReal *invMrow = invM;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invMrow++, bodycurr++) {
      dxBody *b = *bodycurr;
      //*invMrow = b->mass.mass;
      *invMrow = b->invMass;

    }
  }

  {
    // add the gravity force to all bodies
    // since gravity does normally have only one component it's more efficient
    // to run three loops for each individual component
    dxBody *const *const bodyend = body + nb;
    dReal gravity_x = world->gravity[0];
    if (gravity_x) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
        dxBody *b = *bodycurr;
        if ((b->flags & dxBodyNoGravity)==0) {
          b->facc[0] += b->mass.mass * gravity_x;
        }
      }
    }
    dReal gravity_y = world->gravity[1];
    if (gravity_y) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
        dxBody *b = *bodycurr;
        if ((b->flags & dxBodyNoGravity)==0) {
          b->facc[1] += b->mass.mass * gravity_y;
        }
      }
    }
    dReal gravity_z = world->gravity[2];
    if (gravity_z) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
        dxBody *b = *bodycurr;
        if ((b->flags & dxBodyNoGravity)==0) {
          b->facc[2] += b->mass.mass * gravity_z;
        }
      }
    }
  }

  // get joint information (m = total constraint dimension, nub = number of unbounded variables).
  // joints with m=0 are inactive and are removed from the joints array
  // entirely, so that the code that follows does not consider them.
  dJointWithInfo1 *const jointiinfos = context->AllocateArray<dJointWithInfo1> (_nj);
  int nj;

  {
    dJointWithInfo1 *jicurr = jointiinfos;
    dxJoint *const *const _jend = _joint + _nj;
    for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; _jcurr++) {	// jicurr=dest, _jcurr=src
      dxJoint *j = *_jcurr;
      j->getInfo1 (&jicurr->info);
      dIASSERT (jicurr->info.m >= 0 && jicurr->info.m <= 6 && jicurr->info.nub >= 0 && jicurr->info.nub <= jicurr->info.m);
      if (jicurr->info.m > 0) {
        jicurr->joint = j;
        jicurr++;
      }
    }
    nj = jicurr - jointiinfos;
  }

  context->ShrinkArray<dJointWithInfo1>(jointiinfos, _nj, nj);

  int m;
  int mfb; // number of rows of Jacobian we will have to save for joint feedback

  {
    int mcurr = 0, mfbcurr = 0;
    const dJointWithInfo1 *jicurr = jointiinfos;
    const dJointWithInfo1 *const jiend = jicurr + nj;
    for (; jicurr != jiend; jicurr++) {
      int jm = jicurr->info.m;
      mcurr += jm;
      if (jicurr->joint->feedback)
        mfbcurr += jm;
    }

    m = mcurr;
    mfb = mfbcurr;
  }

  // if there are constraints, compute the constraint force
  dReal *J = NULL;
  int *jb = NULL;

  if (m > 0) {
    dReal *cfm, *lo, *hi, *rhs, *Jcopy;
    int *findex;

    {
      int mlocal = m;

      const unsigned jelements = mlocal*12;
      J = context->AllocateArray<dReal> (jelements);
      dSetZero (J,jelements);

      // create a constraint equation right hand side vector `c', a constraint
      // force mixing vector `cfm', and LCP low and high bound vectors, and an
      // 'findex' vector.
      cfm = context->AllocateArray<dReal> (mlocal);
      dSetValue (cfm,mlocal,world->global_cfm);

      lo = context->AllocateArray<dReal> (mlocal);
      dSetValue (lo,mlocal,-dInfinity);

      hi = context->AllocateArray<dReal> (mlocal);
      dSetValue (hi,mlocal,dInfinity);

      findex = context->AllocateArray<int> (mlocal);
      for (int i=0; i<mlocal; i++) findex[i] = -1;

      const unsigned jbelements = mlocal*2;
      jb = context->AllocateArray<int> (jbelements);

      rhs = context->AllocateArray<dReal> (mlocal);

      Jcopy = context->AllocateArray<dReal> (mfb*12);

    }

    BEGIN_STATE_SAVE(context, cstate) {
      dReal *c = context->AllocateArray<dReal> (m);
      dSetZero (c, m);

      {
        IFTIMING (dTimerNow ("create J"));
        // get jacobian data from constraints. an m*12 matrix will be created
        // to store the two jacobian blocks from each constraint. it has this
        // format:
        //
        //   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 \    .
        //   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2  )-- jacobian for joint 0, body 1 and body 2 (3 rows)
        //   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 /
        //   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 )--- jacobian for joint 1, body 1 and body 2 (3 rows)
        //   etc...
        //
        //   (lll) = linear jacobian data
        //   (aaa) = angular jacobian data
        //
        dxJoint::Info2 Jinfo;
        Jinfo.rowskip = 12;
        Jinfo.fps = stepsize1;
        Jinfo.erp = world->global_erp;

        dReal *Jcopyrow = Jcopy;
        unsigned ofsi = 0;
        const dJointWithInfo1 *jicurr = jointiinfos;
        const dJointWithInfo1 *const jiend = jicurr + nj;
        for (; jicurr != jiend; jicurr++) {
          dReal *const Jrow = J + ofsi * 12;
          Jinfo.J1l = Jrow;
          Jinfo.J1a = Jrow + 3;
          Jinfo.J2l = Jrow + 6;
          Jinfo.J2a = Jrow + 9;
          Jinfo.c = c + ofsi;
          Jinfo.cfm = cfm + ofsi;
          Jinfo.lo = lo + ofsi;
          Jinfo.hi = hi + ofsi;
          Jinfo.findex = findex + ofsi;


          // now write all information into J
          dxJoint *joint = jicurr->joint;
          joint->getInfo2 (&Jinfo);
          const int infom = jicurr->info.m;

          // we need a copy of Jacobian for joint feedbacks
          // because it gets destroyed by SOR solver
          // instead of saving all Jacobian, we can save just rows
          // for joints, that requested feedback (which is normally much less)
          if (joint->feedback) {
            const int rowels = infom * 12;
            memcpy(Jcopyrow, Jrow, rowels * sizeof(dReal));
            Jcopyrow += rowels;
          }

          // adjust returned findex values for global index numbering
          int *findex_ofsi = findex + ofsi;
          for (int j=0; j<infom; j++) {
            int fival = findex_ofsi[j];
            if (fival >= 0)
              findex_ofsi[j] = fival + ofsi;
          }

          ofsi += infom;
        }
      }

      {
        // create an array of body numbers for each joint row
        int *jb_ptr = jb;
        const dJointWithInfo1 *jicurr = jointiinfos;
        const dJointWithInfo1 *const jiend = jicurr + nj;
        for (; jicurr != jiend; jicurr++) {
          dxJoint *joint = jicurr->joint;
          const int infom = jicurr->info.m;

          int b1 = (joint->node[0].body) ? (joint->node[0].body->tag) : -1;
          int b2 = (joint->node[1].body) ? (joint->node[1].body->tag) : -1;
          for (int j=0; j<infom; j++) {
            jb_ptr[0] = b1;
            jb_ptr[1] = b2;
            jb_ptr += 2;
          }
        }
        dIASSERT (jb_ptr == jb+2*m);
      }

      BEGIN_STATE_SAVE(context, tmp1state) {
        IFTIMING (dTimerNow ("compute rhs"));
        // compute the right hand side `rhs'
        dReal *tmp1 = context->AllocateArray<dReal> (nb*6);
        // put v/h + invM*fe into tmp1
        dReal *tmp1curr = tmp1;
        const dReal *invIrow = invI;
        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body; bodycurr != bodyend; tmp1curr+=6, invIrow+=12, bodycurr++) {
          dxBody *b = *bodycurr;
          dReal body_invMass = b->invMass;
          for (int j=0; j<3; j++) tmp1curr[j] = b->facc[j] * body_invMass + b->lvel[j] * stepsize1;
          dMultiply0_331 (tmp1curr + 3,invIrow,b->tacc);
          for (int k=0; k<3; k++) tmp1curr[3+k] += b->avel[k] * stepsize1;
        }

        // put J*tmp1 into rhs
        multiply_J (m,J,jb,tmp1,rhs);
      } END_STATE_SAVE(context, tmp1state);

      // complete rhs
      for (int i=0; i<m; i++) rhs[i] = c[i]*stepsize1 - rhs[i];

      // scale CFM
      for (int j=0; j<m; j++) cfm[j] *= stepsize1;

    } END_STATE_SAVE(context, cstate);

    /*
    for( int i=0; i<m; i++ ) {
      lo[i] = CUDA_MIN > lo[i] ? CUDA_MIN : lo[i];//std::min(lo[i], (dReal)CUDA_MIN);//
      hi[i] = hi[i] > CUDA_MAX ? CUDA_MAX : hi[i];//std::min(hi[i], (dReal)CUDA_MAX);//
    }
    */

    // load lambda from the value saved on the previous iteration
    dReal *lambda = context->AllocateArray<dReal> (m);

    dReal *cforce = context->AllocateArray<dReal> (nb*6);

    dReal *iMJ = context->AllocateArray<dReal> (m*12);
    dReal *Ad = context->AllocateArray<dReal> (m);

    BEGIN_STATE_SAVE(context, lcpstate) {
      // solve the LCP problem and get lambda and invM*constraint_force
      IFBENCHMARKING( if( benchmarkIters > minBenchmarkIters ) totalConstraints+=m; totalBodies+=nb; dTimerStart(" LCP Solve - Parallel "););

#if PARALLEL_ENABLED
        // FIXMED: need to first cast params into float, right now, SolverType is float, but params contains dRal pointers
        SolverType::SolverParams params(context,&world->qs,m,nb,J,jb,body,invI,lambda,cforce,rhs,lo,hi,cfm,iMJ,Ad,findex,stepsize);
        parallelSolver.worldSolve( &params );
#else
        worldSolve(context,m,nb,J,jb,body,invI,lambda,cforce,rhs,lo,hi,cfm,iMJ,Ad,findex,&world->qs);
#endif

      IFBENCHMARKING (dTimerEnd());

    } END_STATE_SAVE(context, lcpstate);

    // note that the SOR method overwrites rhs and J at this point, so
    // they should not be used again.
    {
      IFTIMING (dTimerNow ("velocity update due to constraint forces"));
      // note that cforce is really not a force but an acceleration, hence there is
      // no premultiplying of invM here (compare to update due to external force 'facc' below)
      //
      // add stepsize * cforce to the body velocity
      const dReal *cforcecurr = cforce;
      dxBody *const *const bodyend = body + nb;
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; cforcecurr+=6, bodycurr++) {
        dxBody *b = *bodycurr;
        for (int j=0; j<3; j++) {
          b->lvel[j] += stepsize * cforcecurr[j];
          b->avel[j] += stepsize * cforcecurr[3+j];
        }
      }
    }

    if (mfb > 0) {
      // straightforward computation of joint constraint forces:
      // multiply related lambdas with respective J' block for joints
      // where feedback was requested
      dReal data[6];
      const dReal *lambdacurr = lambda;
      const dReal *Jcopyrow = Jcopy;
      const dJointWithInfo1 *jicurr = jointiinfos;
      const dJointWithInfo1 *const jiend = jicurr + nj;
      for (; jicurr != jiend; jicurr++) {
        dxJoint *joint = jicurr->joint;
        const int infom = jicurr->info.m;

        if (joint->feedback) {
          dJointFeedback *fb = joint->feedback;
          Multiply1_12q1 (data, Jcopyrow, lambdacurr, infom);
          fb->f1[0] = data[0];
          fb->f1[1] = data[1];
          fb->f1[2] = data[2];
          fb->t1[0] = data[3];
          fb->t1[1] = data[4];
          fb->t1[2] = data[5];

          if (joint->node[1].body)
          {
            Multiply1_12q1 (data, Jcopyrow+6, lambdacurr, infom);
            fb->f2[0] = data[0];
            fb->f2[1] = data[1];
            fb->f2[2] = data[2];
            fb->t2[0] = data[3];
            fb->t2[1] = data[4];
            fb->t2[2] = data[5];
          }

          Jcopyrow += infom * 12;
        }

        lambdacurr += infom;
      }
    }
  }

  {
    IFTIMING (dTimerNow ("compute velocity update"));
    // compute the velocity update:
    // add stepsize * invM * fe to the body velocity
    const dReal *invIrow = invI;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow += 12, bodycurr++) {
      dxBody *b = *bodycurr;
      dReal body_invMass_mul_stepsize = stepsize * b->invMass;
      for (int j=0; j<3; j++) {
        b->lvel[j] += body_invMass_mul_stepsize * b->facc[j];
        b->tacc[j] *= stepsize;
      }
      dMultiplyAdd0_331 (b->avel, invIrow, b->tacc);
    }
  }

  {
    // update the position and orientation from the new linear/angular velocity
    // (over the given timestep)
    IFTIMING (dTimerNow ("update position"));
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
      dxBody *b = *bodycurr;
      dxStepBody (b,stepsize);
    }
  }

  {
    IFTIMING (dTimerNow ("tidy up"));
    // zero all force accumulators
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
      dxBody *b = *bodycurr;
      b->facc_last[0] = b->facc[0];
      b->facc_last[1] = b->facc[1];
      b->facc_last[2] = b->facc[2];
      b->tacc_last[0] = b->tacc[0];
      b->tacc_last[1] = b->tacc[1];
      b->tacc_last[2] = b->tacc[2];
      dSetZero (b->facc,3);
      dSetZero (b->tacc,3);
    }
  }

  IFTIMING (dTimerEnd());
  IFTIMING (if (m > 0) dTimerReport (stdout,1));
  IFBENCHMARKING ( benchmarkIteration( world ) );
}

static size_t EstimateParallelSOR_LCPMemoryRequirements(int m, int n)
{
  size_t res = dEFFICIENT_SIZE(sizeof(dReal) * 4 * m) * 4; // for ij
  res += dEFFICIENT_SIZE(sizeof(dReal) * 4 * m) * 4; // for j
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for adcfm
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for rhs
  res += dEFFICIENT_SIZE(sizeof(dReal) * m) * 2; // for lohi
  res += dEFFICIENT_SIZE(sizeof(int) * m * 4); // for bodyIDs
  res += dEFFICIENT_SIZE(sizeof(int) * m); // for fIDs
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda

  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for iMass
  res += dEFFICIENT_SIZE(sizeof(dReal) * m * 4) * 3; // for i0
  res += dEFFICIENT_SIZE(sizeof(dReal) * m * 4); // for bodyFAcc
  res += dEFFICIENT_SIZE(sizeof(dReal) * m * 4); // for bodyTAcc
  res += dEFFICIENT_SIZE(sizeof(dReal) * m * 4) * (size_t)ParallelOptions::MAXBODYREPETITION; // for bodyFAccReduction
  res += dEFFICIENT_SIZE(sizeof(dReal) * m * 4) * (size_t)ParallelOptions::MAXBODYREPETITION; // for bodyTAccReduction

  return res;
}

size_t dxEstimateParallelStepMemoryRequirements (
  dxBody * const *body, int nb, dxJoint * const *_joint, int _nj)
{
  int nj, m, mfb;

  {
    int njcurr = 0, mcurr = 0, mfbcurr = 0;
    dxJoint::SureMaxInfo info;
    dxJoint *const *const _jend = _joint + _nj;
    for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; _jcurr++) {
      dxJoint *j = *_jcurr;
      j->getSureMaxInfo (&info);

      int jm = info.max_m;
      if (jm > 0) {
        njcurr++;

        mcurr += jm;
        if (j->feedback)
          mfbcurr += jm;
      }
    }
    nj = njcurr; m = mcurr; mfb = mfbcurr;
  }

  size_t res = 0;

  res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * nb); // for invI
  res += dEFFICIENT_SIZE(sizeof(dReal) * nb); // for invM

  {
    size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * _nj); // for initial jointiinfos
    size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * nj); // for shrunk jointiinfos
    if (m > 0) {
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for J
      sub1_res2 += 4 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for cfm, lo, hi, rhs
      sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * 12 * m); // for jb            FIXME: shoulbe be 2 not 12?
      sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for findex
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * mfb); // for Jcopy

      {
        size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for c
        {
          size_t sub3_res1 = dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for tmp1

          size_t sub3_res2 = 0;

          sub2_res1 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
        }

        size_t sub2_res2 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for cforce
        {
          size_t sub3_res1 = EstimateParallelSOR_LCPMemoryRequirements(m,nj); // for SOR_LCP
          size_t sub3_res2 = 0;
          sub2_res2 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
        }

        sub1_res2 += (sub2_res1 >= sub2_res2) ? sub2_res1 : sub2_res2;
      }
    }

    res += 3 * ((sub1_res1 >= sub1_res2) ? sub1_res1 : sub1_res2);
  }

  return res;
}

static size_t BuildIslandAndEstimateStepperMemoryRequirements(dxWorldProcessContext *context,
  dxWorld *world, dReal stepsize, dmemestimate_fn_t stepperestimate)
{
  const int sizeelements = 2;
  size_t maxreq = 0;

  // handle auto-disabling of bodies
  dInternalHandleAutoDisabling (world,stepsize);

  int nb = world->nb, nj = world->nj;
  // Make array for island body/joint counts
  int *islandsizes = context->AllocateArray<int>(sizeelements);

  // make arrays for body and joint lists (for a single island) to go into
  dxBody **body = context->AllocateArray<dxBody *>(nb);
  dxJoint **joint = context->AllocateArray<dxJoint *>(nj);

  BEGIN_STATE_SAVE(context, stackstate) {
    // allocate a stack of unvisited bodies in the island. the maximum size of
    // the stack can be the lesser of the number of bodies or joints, because
    // new bodies are only ever added to the stack by going through untagged
    // joints. all the bodies in the stack must be tagged!

    int stackalloc = (nj < nb) ? nj : nb;
    dxBody **stack = context->AllocateArray<dxBody *>(stackalloc);

    {
      // set all body/joint tags to 0
      for (dxBody *b=world->firstbody; b; b=(dxBody*)b->next) b->tag = 0;
      for (dxJoint *j=world->firstjoint; j; j=(dxJoint*)j->next) j->tag = 0;
    }

    int bcount = 0, jcount =0;

    dxBody **bodystart = body;
    dxJoint **jointstart = joint;
    for (dxBody *bb=world->firstbody; bb; bb=(dxBody*)bb->next) {
      // get bb = the next enabled, untagged body, and tag it
      if (!bb->tag) {
        if (!(bb->flags & dxBodyDisabled)) {
          bb->tag = 1;

          dxBody **bodycurr = bodystart;
          dxJoint **jointcurr = jointstart;

          // tag all bodies and joints starting from bb.
          *bodycurr++ = bb;

          int stacksize = 0;
          dxBody *b = bb;

          while (true) {
            // traverse and tag all body's joints, add untagged connected bodies
            // to stack
            for (dxJointNode *n=b->firstjoint; n; n=n->next) {
              dxJoint *njoint = n->joint;
              if (!njoint->tag) {
                if (njoint->isEnabled()) {
                  njoint->tag = 1;
                  *jointcurr++ = njoint;

                  dxBody *nbody = n->body;
                  // Body disabled flag is not checked here. This is how auto-enable works.
                  if (nbody && nbody->tag <= 0) {
                    nbody->tag = 1;
                    // Make sure all bodies are in the enabled state.
                    nbody->flags &= ~dxBodyDisabled;
                    stack[stacksize++] = nbody;
                  }
                } else {
                  njoint->tag = -1; // Used in Step to prevent search over disabled joints (not needed for QuickStep so far)
                }
              }
            }
            dIASSERT(stacksize <= world->nb);
            dIASSERT(stacksize <= world->nj);

            if (stacksize == 0) {
              break;
            }

            b = stack[--stacksize];	// pop body off stack
            *bodycurr++ = b;	// put body on body list
          }

          bcount += (bodycurr - bodystart);
          jcount += (jointcurr - jointstart);

          bodystart = bodycurr;
          jointstart = jointcurr;
        } else {
          bb->tag = -1; // Not used so far (assigned to retain consistency with joints)
        }
      }
    }

    islandsizes[0] = bcount;
    islandsizes[1] = jcount;

    size_t islandreq = stepperestimate(body, bcount, joint, jcount);
    maxreq = islandreq;

  } END_STATE_SAVE(context, stackstate);

# ifndef dNODEBUG
  // if debugging, check that all objects (except for disabled bodies,
  // unconnected joints, and joints that are connected to disabled bodies)
  // were tagged.
  {
    for (dxBody *b=world->firstbody; b; b=(dxBody*)b->next) {
      if (b->flags & dxBodyDisabled) {
        if (b->tag > 0) dDebug (0,"disabled body tagged");
      }
      else {
        if (b->tag <= 0) dDebug (0,"enabled body not tagged");
      }
    }
    for (dxJoint *j=world->firstjoint; j; j=(dxJoint*)j->next) {
      if ( (( j->node[0].body && (j->node[0].body->flags & dxBodyDisabled)==0 ) ||
        (j->node[1].body && (j->node[1].body->flags & dxBodyDisabled)==0) )
        &&
        j->isEnabled() ) {
          if (j->tag <= 0) dDebug (0,"attached enabled joint not tagged");
      }
      else {
        if (j->tag > 0) dDebug (0,"unattached or disabled joint tagged");
      }
    }
  }
# endif

  size_t const *islandreqs = NULL;
  int islandcount = 1;
  context->SavePreallocations(islandcount, islandsizes, body, joint, islandreqs);

  return maxreq;
}

static size_t EstimateIslandsProcessingMemoryRequirements(dxWorld *world, size_t &sesize)
{
  size_t res = 0;

  size_t islandcounts = dEFFICIENT_SIZE(world->nb * 2 * sizeof(int));
  res += islandcounts;

  size_t bodiessize = dEFFICIENT_SIZE(world->nb * sizeof(dxBody*));
  size_t jointssize = dEFFICIENT_SIZE(world->nj * sizeof(dxJoint*));
  res += bodiessize + jointssize;

  sesize = (bodiessize < jointssize) ? bodiessize : jointssize;
  return res;
}

static size_t AdjustArenaSizeForReserveRequirements(size_t arenareq, float rsrvfactor, unsigned rsrvminimum)
{
  float scaledarena = arenareq * rsrvfactor;
  size_t adjustedarena = (scaledarena < SIZE_MAX) ? (size_t)scaledarena : SIZE_MAX;
  size_t boundedarena = (adjustedarena > rsrvminimum) ? adjustedarena : (size_t)rsrvminimum;
  return dEFFICIENT_SIZE(boundedarena);
}

static dxWorldProcessContext *InternalReallocateWorldProcessContext (
  dxWorldProcessContext *oldcontext, size_t memreq,
  const dxWorldProcessMemoryManager *memmgr, float rsrvfactor, unsigned rsrvminimum)
{
  dxWorldProcessContext *context = oldcontext;
  bool allocsuccess = false;

  size_t oldarenasize;
  void *pOldArena;

  do {
    size_t oldmemsize = oldcontext ? oldcontext->GetMemorySize() : 0;
    if (!oldcontext || oldmemsize < memreq) {
      oldarenasize = oldcontext ? dxWorldProcessContext::MakeArenaSize(oldmemsize) : 0;
      pOldArena = oldcontext ? oldcontext->m_pArenaBegin : NULL;

      if (!dxWorldProcessContext::IsArenaPossible(memreq)) {
        break;
      }

      size_t arenareq = dxWorldProcessContext::MakeArenaSize(memreq);
      size_t arenareq_with_reserve = AdjustArenaSizeForReserveRequirements(arenareq, rsrvfactor, rsrvminimum);
      size_t memreq_with_reserve = memreq + (arenareq_with_reserve - arenareq);

      if (oldcontext) {

        if (oldcontext->m_pAllocCurrent != oldcontext->m_pAllocBegin) {

          // Save old efficient offset and meaningful data size for the case if
          // reallocation throws the block at different efficient offset
          size_t oldcontextofs = (size_t)oldcontext - (size_t)pOldArena;
          size_t datasize = (size_t)oldcontext->m_pAllocCurrent - (size_t)oldcontext;

          // Extra EFFICIENT_ALIGNMENT bytes might be needed after re-allocation with different alignment
          size_t shrunkarenasize = dEFFICIENT_SIZE(datasize + oldcontextofs) + EFFICIENT_ALIGNMENT;
          if (shrunkarenasize < oldarenasize) {

            void *pShrunkOldArena = oldcontext->m_pArenaMemMgr->m_fnShrink(pOldArena, oldarenasize, shrunkarenasize);
            if (!pShrunkOldArena) {
              break;
            }

            // In case if shrinking is not supported and memory manager had to allocate-copy-free
            if (pShrunkOldArena != pOldArena) {
              dxWorldProcessContext *shrunkcontext = (dxWorldProcessContext *)dEFFICIENT_PTR(pShrunkOldArena);

              // Preform data shift in case if efficient alignment of new block
              // does not match that of old block
              size_t shrunkcontextofs = (size_t)shrunkcontext - (size_t)pShrunkOldArena;
              size_t offsetdiff = oldcontextofs - shrunkcontextofs;
              if (offsetdiff != 0) {
                memmove(shrunkcontext, (void *)((size_t)shrunkcontext + offsetdiff), datasize);
              }

              // Make sure allocation pointers are valid - that is necessary to
              // be able to calculate size and free old arena later
              size_t shrunkdatasize = dxWorldProcessContext::MakeBufferSize(shrunkarenasize);
              void *blockbegin = dEFFICIENT_PTR(shrunkcontext + 1);
              void *blockend = dOFFSET_EFFICIENTLY(blockbegin, shrunkdatasize);
              shrunkcontext->m_pAllocBegin = blockbegin;
              shrunkcontext->m_pAllocEnd = blockend;
              shrunkcontext->m_pAllocCurrent = blockend; // -- set to end to prevent possibility of further allocation
              shrunkcontext->m_pArenaBegin = pShrunkOldArena;

              size_t stOffset = ((size_t)pShrunkOldArena - (size_t)pOldArena) - offsetdiff;
              shrunkcontext->OffsetPreallocations(stOffset);

              oldcontext = shrunkcontext;

              // Reassign to old arena variables for potential freeing at exit
              pOldArena = pShrunkOldArena;
            }

            // Reassign to old arena variables for potential freeing at exit
            oldarenasize = shrunkarenasize;
          }

        } else {
          oldcontext->m_pArenaMemMgr->m_fnFree(pOldArena, oldarenasize);
          oldcontext = NULL;

          // Zero variables to avoid another freeing on exit
          pOldArena = NULL;
          oldarenasize = 0;
        }
      }

      // Allocate new arena
      void *pNewArena = memmgr->m_fnAlloc(arenareq_with_reserve);
      if (!pNewArena) {
        break;
      }

      context = (dxWorldProcessContext *)dEFFICIENT_PTR(pNewArena);

      void *blockbegin = dEFFICIENT_PTR(context + 1);
      void *blockend = dOFFSET_EFFICIENTLY(blockbegin, memreq_with_reserve);

      context->m_pAllocBegin = blockbegin;
      context->m_pAllocEnd = blockend;
      context->m_pArenaBegin = pNewArena;
      context->m_pAllocCurrent = blockbegin;

      if (oldcontext) {
        context->CopyPreallocations(oldcontext);
      } else {
        context->ClearPreallocations();
      }

      context->m_pArenaMemMgr = memmgr;
      context->m_pPreallocationcContext = oldcontext;
    }

    allocsuccess = true;
  } while (false);

  if (!allocsuccess) {
    if (pOldArena) {
      dIASSERT(oldcontext);
      oldcontext->m_pArenaMemMgr->m_fnFree(pOldArena, oldarenasize);
    }
    context = NULL;
  }

  return context;
}

bool dxReallocateParallelWorldProcessContext (dxWorld *world, dReal stepsize, dmemestimate_fn_t stepperestimate)
{
  dxStepWorkingMemory *wmem = AllocateOnDemand(world->wmem);
  if (!wmem) return false;

  dxWorldProcessContext *oldcontext = wmem->GetWorldProcessingContext();
  dIASSERT (!oldcontext || oldcontext->IsStructureValid());

  const dxWorldProcessMemoryReserveInfo *reserveinfo = wmem->SureGetMemoryReserveInfo();
  const dxWorldProcessMemoryManager *memmgr = wmem->SureGetMemoryManager();

  dxWorldProcessContext *context = oldcontext;

  size_t sesize;
  size_t islandsreq = EstimateIslandsProcessingMemoryRequirements(world, sesize);
  dIASSERT(islandsreq == dEFFICIENT_SIZE(islandsreq));
  dIASSERT(sesize == dEFFICIENT_SIZE(sesize));

  size_t stepperestimatereq = islandsreq + sesize;
  context = InternalReallocateWorldProcessContext(context, stepperestimatereq, memmgr, 1.0f, reserveinfo->m_uiReserveMinimum);

  if (context)
  {
    size_t stepperreq = BuildIslandAndEstimateStepperMemoryRequirements(context, world, stepsize, stepperestimate);
    dIASSERT(stepperreq == dEFFICIENT_SIZE(stepperreq));

    size_t memreq = stepperreq + islandsreq;
    context = InternalReallocateWorldProcessContext(context, memreq, memmgr, reserveinfo->m_fReserveFactor, reserveinfo->m_uiReserveMinimum);
  }

  wmem->SetWorldProcessingContext(context);
  return context != NULL;
}


