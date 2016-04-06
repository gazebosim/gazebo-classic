#include <iostream>
#include <boost/shared_array.hpp>
#include <gazebo/ode/odemath.h>
#include <gazebo/ode/matrix.h>
#include <vector>
#include <limits>
#include "util.h"
#include "optimize.h"

using std::vector;
using boost::shared_array;
using std::cout;
using std::endl;

// misc defines
#define ALLOCA dALLOCA16
#define DEBUG

// LAPACK routine for solving a symmetric indefinite system
extern "C" {
int ssysv_(char* uplo, int* n, int* nrhs, dReal* A, int* lda, int* ipiv, dReal* xb, int* ldb, dReal* work, int* lwork, int* info, short uplo_len);
}

// routine for solving a symmetric indefinite system
void solveSymmetric(dReal* A, dReal* xb, int n)
{
  // form inputs to LAPACK
  char UPLO = 'U';
  int NRHS = 1;
  int LDA = n;
  int LDB = n;
  int LWORK = -1;
  dReal WORK_QUERY;
  int INFO;
  short UPLO_LEN = 1;

  // memory for pivots
  int* IPIV = new int[n];

  // first, determine workspace size
  ssysv_(&UPLO, &n, &NRHS, A, &LDA, IPIV, xb, &LDB, &WORK_QUERY, &LWORK, &INFO, UPLO_LEN);
  dIASSERT(INFO == 0);

  // setup workspace
  LWORK = (int) WORK_QUERY;
  dReal* WORK = new dReal[LWORK];

  // do the solve
  ssysv_(&UPLO, &n, &NRHS, A, &LDA, IPIV, xb, &LDB, WORK, &LWORK, &INFO, UPLO_LEN);
  dIASSERT(INFO == 0);

  delete [] IPIV;
  delete [] WORK;
}

// non-padded Cholesky factorization
int dNPFactorCholesky (dReal *A, int n)
{
// ODE's version of cholesky factorization seems to have a bug detecting when
// matrix is singular when n=2, substituting a check in here...
  if (n == 2)
  {
    if (dFabs(A[0]*A[3] - A[1]*A[2]) < dSqrt(std::numeric_limits<dReal>::epsilon()))
      return false;
  }

  int i,j,k,nskip;
  dReal sum,*a,*b,*aa,*bb,*cc,*recip;
  dAASSERT (n > 0 && A);
  nskip = n;
  recip = (dReal*) ALLOCA (n * sizeof(dReal));
  aa = A;
  for (i=0; i<n; i++) {
    bb = A;
    cc = A + i*nskip;
    for (j=0; j<i; j++) {
      sum = *cc;
      a = aa;
      b = bb;
      for (k=j; k; k--) sum -= (*(a++))*(*(b++));
      *cc = sum * recip[j];
      bb += nskip;
      cc++;
    }
    sum = *cc;
    a = aa;
    for (k=i; k; k--, a++) sum -= (*a)*(*a);
    if (sum <= REAL(0.0)) return 0;
    *cc = dSqrt(sum);
    recip[i] = dRecip (*cc);
    aa += nskip;
  }
  return 1;
}

// non-padded Cholesky solve
static void dNPSolveCholesky (const dReal *L, dReal *b, int n)
{
  int i,k,nskip;
  dReal sum,*y;
  dAASSERT (n > 0 && L && b);
  nskip = n;
  y = (dReal*) ALLOCA (n*sizeof(dReal));
  for (i=0; i<n; i++) {
    sum = 0;
    for (k=0; k < i; k++) sum += L[i*nskip+k]*y[k];
    y[i] = (b[i]-sum)/L[i*nskip+i];
  }
  for (i=n-1; i >= 0; i--) {
    sum = 0;
    for (k=i+1; k < n; k++) sum += L[k*nskip+i]*b[k];
    b[i] = (y[i]-sum)/L[i*nskip+i];
  }
}

// non-padded matrix multiplication
void dNPMultiply0 (dReal *A, const dReal *B, const dReal *C, int p, int q, int r)
{
  int i,j,k,qskip,rskip,rpad;
  if (p == 0 || q == 0 || r == 0)
    return;
  dAASSERT (A && B && C && p>0 && q>0 && r>0);
  qskip = q;
  rskip = r;
  rpad = rskip - r;
  dReal sum;
  const dReal *b,*c,*bb;
  bb = B;
  for (i=p; i; i--) {
    for (j=0 ; j<r; j++) {
      c = C + j;
      b = bb;
      sum = 0;
      for (k=q; k; k--, c+=rskip) sum += (*(b++))*(*c);
      *(A++) = sum; 
    }
    A += rpad;
    bb += qskip;
  }
}

// non-padded matrix multiplication
void dNPMultiply1 (dReal *A, const dReal *B, const dReal *C, int p, int q, int r)
{
  int i,j,k,pskip,rskip;
  dReal sum;
  if (p == 0 || q == 0 || r == 0)
    return;
  dAASSERT (A && B && C && p>0 && q>0 && r>0);
  pskip = p;
  rskip = r;
  for (i=0; i<p; i++) {
    for (j=0; j<r; j++) {
      sum = 0;
      for (k=0; k<q; k++) sum += B[i+k*pskip] * C[j+k*rskip];
      A[i*rskip+j] = sum;
    }
  }
}

void dNPMultiply2 (dReal *A, const dReal *B, const dReal *C, int p, int q, int r)
{
  int i,j,k,z,rpad,qskip;
  dReal sum;
  const dReal *bb,*cc;
  if (p == 0 || q == 0 || r == 0)
    return;
  dAASSERT (A && B && C && p>0 && q>0 && r>0);
  rpad = 0;
  qskip = q;
  bb = B;
  for (i=p; i; i--) {
    cc = C;
    for (j=r; j; j--) {
      z = 0;
      sum = 0;
      for (k=q; k; k--,z++) sum += bb[z] * cc[z];
      *(A++) = sum; 
      cc += qskip;
    }
    A += rpad;
    bb += qskip;
  }
}

// prints a matrix
void printMatrix(dReal* A, int m, int n)
{
  for (int i=0; i< m*n; i++)
  {
    if (i % n == 0 && i > 0)
      std::cout << std::endl;
    std::cout << A[i] << " ";
  }
  std::cout << std::endl;
}

// copies real arrays
void dCopy(dReal* target, int tstride, dReal* source, int sstride, int n)
{
  for (int i=0; i< n; i++, target += tstride, source += sstride)
    *target = *source;
}

// sets a submatrix of a matrix
// A: the matrix into which B will be copied
// ac: the number of columns of A
// B: the source matrix
// br: the number of rows of B
// bc: the number of columns of b
// sr: the starting row of A to which to copy
// sc: the starting column of A to which to copy
void dSetSubMat0(dReal* A, int ac, dReal* B, int br, int bc, int sr, int sc)
{
  int siA = sr*ac + sc;          // starting index for A
  const int Arowskip = ac;
  const int Browskip = bc;
  int siB = 0;                   // starting index for B

  for (int i=0; i< br; i++, siA += Arowskip, siB += Browskip)
    memcpy(A+siA, B+siB, bc*sizeof(dReal));
}

// sets a submatrix of a matrix to a transposed matrix
void dSetSubMat1(dReal* A, int ac, dReal* B, int br, int bc, int sr, int sc)
{
  int siA = sr*ac + sc;
  const int Arowskip = ac;
  int siB = 0;

  for (int i=0; i< bc; i++, siA += Arowskip, siB++)
    dCopy(A+siA, 1, B+siB, bc, br);
}

// Data structure for solving the LCP w = Mx + q for x
struct LCPData
{
  dReal const* M;  // the n x n LCP matrix
  dReal const* q;  // the n-dimensional LCP vector
  dReal* work;     // an n-dimensional work vector

  LCPData(int n) { work = new dReal[n]; }
  ~LCPData() { delete [] work; }
};

// data for computing feasible starting point for convex optimization
struct FeasibilityData
{
  int m;          // number of constraints in the original problem
  int n;          // size of x in the original problem
  void* data;     // data in the original problem
  dReal* workn;   // work array of size n
  dReal* workm;   // work array of size m+1
  dReal* worknxn; // work array of size n x n

  // functions in the original problem
  void (*fx)(dReal*, int, dReal*, int, void*);
  void (*grad)(dReal*, int, int, dReal*, void*);
  bool (*hess)(dReal*, int, int, dReal*, void*);

  FeasibilityData(int _m, int _n)
  {
    this->m = _m;
    this->n = _n;
    workn = new dReal[_n];
    workm = new dReal[_m+1];
    worknxn = new dReal[_n*_n];
  }

  ~FeasibilityData() { delete [] workm; delete [] workn; delete [] worknxn; }
};

// checks to see whether we can terminate early
static bool make_feasible_tcheck(dReal* y, int /*n*/, void* data)
{
  // get the feasibility data
  FeasibilityData& fdata = *((FeasibilityData*) data);
  //dIASSERT(n == fdata.n+1);
  const int OLDN = fdata.n;
  const int OLDM = fdata.m;

  // remove v from y to yield x
  memcpy(fdata.workn, y, OLDN*sizeof(dReal));

  // call the original objective / constraint function
  (*fdata.fx)(fdata.workn, OLDN, fdata.workm, OLDM, fdata.data);

  // determine the maximum constraint value
  dReal v = *std::max_element(fdata.workm+1, fdata.workm+1+OLDM);
  return (v <= (dReal) 0.0);
}

// evaluation function for making an initial point feasible
static void make_feasible_fx(dReal* y, int /*n*/, dReal* f, int /*m*/, void* data)
{
  // get the feasibility data
  FeasibilityData& fdata = *((FeasibilityData*) data);
  //dIASSERT(n == fdata.n+1);
  const int OLDN = fdata.n;
  const int OLDM = fdata.m;

  // remove v from y to yield x
  memcpy(fdata.workn, y, OLDN*sizeof(dReal));

  // call the original objective / constraint function
  (*fdata.fx)(fdata.workn, OLDN, fdata.workm, OLDM, fdata.data);

  // get v
  dReal v = y[OLDN];
  f[0] = v;

  // constraints of f are fdata.workm[i] <= v
  for (int i=0; i< OLDM; i++)
    f[i+1] = fdata.workm[i+1] - v;
}

// gradient function for making an initial point feasible
static void make_feasible_grad(dReal* y, int n, int idx, dReal* g, void* data)
{
  // get the feasibility data
  FeasibilityData& fdata = *((FeasibilityData*) data);
  dIASSERT(n == fdata.n+1);
  const int OLDN = fdata.n;

  // new objective function is simple
  if (idx == 0)
  {
    dSetZero(g, n);
    g[OLDN] = (dReal) 1.0;
    return;
  }

  // still here?  need to call original gradient function

  // remove v from y to yield x
  memcpy(fdata.workn, y, OLDN*sizeof(dReal));

  // call the original gradient function
  (*fdata.grad)(fdata.workn, OLDN, idx, g, fdata.data);
  g[OLDN] = (dReal) -1.0;
}

// Hessian function for making an initial point feasible
static bool make_feasible_hess(dReal* y, int n, int idx, dReal* H, void* data)
{
  // get the feasibility data
  FeasibilityData& fdata = *((FeasibilityData*) data);
  dIASSERT(n == fdata.n+1);
  const int OLDN = fdata.n;

  // new objective function is simple
  if (idx == 0)
  {
    dSetZero(H, n*n);
    return false;
  }

  // still here?  need to call original Hessian function

  // remove v from y to yield x
  memcpy(fdata.workn, y, OLDN*sizeof(dReal));

  // call the original Hessian function
  if (!(*fdata.hess)(fdata.workn, OLDN, idx, fdata.worknxn, fdata.data))
    return false;

  // set the submatrix
  dSetSubMat0(H, n, fdata.worknxn, OLDN, OLDN, 0, 0);

  // set the last column and row of H to zeros
  dSetZero(H+n*OLDN, n);
  int Hidx = OLDN;
  for (int i=0; i< OLDN; i++, Hidx+= n)
    H[Hidx] = (dReal) 0.0;

  return true;
}

// Hessian function for solving convex LCPs
static bool lcp_ip_hess(dReal* /*x*/, int n, int idx, dReal* H, void* data)
{
  // get the LCP data
  const LCPData& lcpdata = *((LCPData*) data);

  // get M and q
  const dReal* M = lcpdata.M;
  //const dReal* q = lcpdata.q;

  // copy precomputed Hessian
  if (idx == 0)
  {
    memcpy(H, M, n*n*sizeof(dReal));
    return true;
  }
  else
  {
    dSetZero(H, n*n);
    return false;
  }
}

// gradient function for solving convex LCPs
static void lcp_ip_grad(dReal* x, int n, int idx, dReal* g, void* data)
{
  // get the LCP data
  const LCPData& lcpdata = *((LCPData*) data);

  // get M and q
  const dReal* M = lcpdata.M;
  const dReal* q = lcpdata.q;

  // objective function gradient: M*x + 1/2q
  if (idx == 0)
  {
    dNPMultiply0(g, M, x, n, n, 1);
    for (int i=0; i< n; i++)
      g[i] += q[i]*0.5;  
  }
  // constraint x >= 0
  else if (idx <= n)
  {
    dSetZero(g, n);
    g[idx-1] = (dReal) -1.0;
  }
  // constraint w = Mx + q >= 0
  else
  {
    // copy and negate row idx - n - 1 of M
    memcpy(g, M+n*(idx-n-1), n*sizeof(dReal));
    for (int i=0; i< n; i++)
      g[i] = -g[i];
  }
}

// objective / constraint evaluation function for solving convex LCPs
static void lcp_ip_fx(dReal* x, int n, dReal* f, int /*m*/, void* data)
{
  const dReal S_BUFFER = std::numeric_limits<dReal>::epsilon();

  // get the LCP data
  const LCPData& lcpdata = *((LCPData*) data);

  // get M and q
  const dReal* M = lcpdata.M;
  const dReal* q = lcpdata.q;

  // get the work variable
  dReal* work = lcpdata.work;

  // evaluate the objective function
  dNPMultiply0(work, M, x, n, n, 1);
  for (int i=0; i< n; i++)
    work[i] += q[i];
  f[0] = dDot(x, work, n) * (dReal) 0.5;

  // evaluate the x >= 0 constraint functions
  int fidx = 1;
  for (int i=0; i< n; i++)
    if (!_dequal(x[i], (dReal) 0.0))
      f[fidx++] = -x[i];
    else
      f[fidx++] = -S_BUFFER;

  // evaluate the w >= 0 constraint functions
  for (int i=0; i< n; i++)
    if (!_dequal(work[i], (dReal) 0.0))
      f[fidx++] = -work[i];
    else
      f[fidx++] = -S_BUFFER;
}

// checks whether the inequality constraints are feasible
static bool feasible(dReal* f, int m)
{
  const dReal S_BUFFER = dSqrt(std::numeric_limits<dReal>::epsilon());

  for (int i=0; i< m; i++)
    if (!(f[i] < -S_BUFFER))
      return false;

  return true;
}

// makes a point feasible for solving a convex optimization problem
bool dMakeFeasibleConvex(CvxOptParams& cparams, dReal eps_feas, dReal* x, int n)
{
  // setup the feasibility data
  FeasibilityData fdata(cparams.m, n);
  fdata.fx = cparams.fx;
  fdata.grad = cparams.grad;
  fdata.hess = cparams.hess;
  fdata.data = cparams.data;

  // evaluate f at current x
  shared_array<dReal> f(new dReal[cparams.m+1]);
  (*cparams.fx)(x, n, f.get(), cparams.m, cparams.data);

  // verify that x is not already feasible (setup v simultaneously)
  dReal v = *std::max_element(f.get()+1, f.get()+cparams.m+1);
  if (v < (dReal) 0.0)
    return true;

  // setup y
  shared_array<dReal> y(new dReal[n+1]);
  memcpy(y.get(), x, n*sizeof(dReal));
  y[n] = v + (dReal) 1.0;

  // make new matrix A to account for increased size of y
  shared_array<dReal> Anew(new dReal[(n+1)*cparams.nu_len]);
  dSetSubMat0(Anew.get(), n+1, cparams.A, cparams.nu_len, n, 0, 0);
  for (int i=0; i< cparams.nu_len; i++)
    Anew[i*(n+1)+n] = (dReal) 0.0; 

  // setup convex optimization parameters
  CvxOptParams cp;
  cp.m = cparams.m;
  cp.nu_len = cparams.nu_len;
  cp.eps = cparams.eps;
  cp.tcheck = &make_feasible_tcheck;
  cp.fx = &make_feasible_fx;
  cp.grad = &make_feasible_grad;
  cp.hess = &make_feasible_hess;
  cp.data = &fdata;
  cp.A = Anew.get();
  cp.b = cparams.b;
  cp.max_iterations = std::max(1000 + (int) log(n), cparams.max_iterations);

  // do convex optimization with primal-dual method
  dOptimizeConvexPrimalDual(cp, eps_feas, y.get(), n+1);

  // get x out
  memcpy(x, y.get(), n*sizeof(dReal));

  // determine s
  (*cparams.fx)(x, n, f.get(), cparams.m, cparams.data);
  v = *std::max_element(f.get()+1, f.get()+cparams.m+1);
  return v < eps_feas;
}

// interior point method for solving convex LCPs
bool dSolveLCPIP(dReal* M, dReal* q, dReal* x, int n, dReal eps_feas)
{
  // create LCP data
  LCPData lcpd(n);
  lcpd.M = M;
  lcpd.q = q;

  // setup convex optimization parameters
  CvxOptParams cparams;
  cparams.m = n*2;
  cparams.fx = &lcp_ip_fx;
  cparams.grad = &lcp_ip_grad;
  cparams.hess = &lcp_ip_hess;
  cparams.eps = eps_feas;
  cparams.data = &lcpd;
  cparams.max_iterations = std::numeric_limits<int>::max();

  // first, make feasible
  if (!dMakeFeasibleConvex(cparams, eps_feas, x, n))
    return false;

  // now optimize
  if (!dOptimizeConvexPrimalDual(cparams, eps_feas, x, n))
    return false;

  // check that x >= 0
  for (int i=0; i< n; i++)
    if (x[i] < -eps_feas)
      return false;

  // check that w >= 0
  dNPMultiply0(lcpd.work, M, x, n, n, 1);
  for (int i=0; i< n; i++)
    if (lcpd.work[i] < -eps_feas)
      return false;

  // check complementarity condition
  return dDot(lcpd.work, x, n) < eps_feas;
}

static void setupM(dReal* M, dReal* H, int n, dReal* A, int nu_len)
{
  // first zero the matrix
  dSetZero(M, (n+nu_len)*(n+nu_len));

  // copy H to M, row by row
  dSetSubMat0(M, n+nu_len, H, n, n, 0, 0);
  dSetSubMat0(M, n+nu_len, A, nu_len, n, n, 0);
  dSetSubMat1(M, n+nu_len, A, nu_len, n, 0, n);
}

// conditions (makes positive-definite) the Hessian matrix
static void conditionHessian(dReal* M, dReal* tmp, int n)
{
  // Hessian _must_ be positive-definite for the optimization to work
  const dReal BETA = 1e-3;
  dReal tau = M[0];
  for (unsigned i=1; i< (unsigned)n; i++)
    tau = std::min(tau, M[i*n+i]);
  tau = (tau > (dReal) 0.0) ? BETA : -tau + BETA;

  // copy M to tmp
  memcpy(tmp, M, n*n*sizeof(dReal));

  if (!dNPFactorCholesky(tmp, n))
    while (true)
    {
      // copy M to tmp
      memcpy(tmp, M, n*n*sizeof(dReal));

      // update tmp += I*tau
      for (int i=0; i< n; i++)
        tmp[i*n+i] += tau;

      // try Cholesky factorization
      if (dNPFactorCholesky(tmp, n))
        break;

      // update tau
      tau = std::max(tau*(dReal) 2.0, BETA);
    }

  // augment M
  for (int i=0; i< n; i++)
    M[i*n+i] += tau;
}

bool dOptimizeConvexPrimalDual(CvxOptParams& cparams, dReal eps_feas, dReal* x, int n)
{
  const int m = cparams.m;
  const int nu_len = cparams.nu_len;
  dReal rdual_nrm, rpri_nrm, f0_best;
  const dReal NEAR_ZERO = dSqrt(std::numeric_limits<dReal>::epsilon());

  // see whether we can quit immediately
  if (cparams.tcheck && (*cparams.tcheck)(x, n, cparams.data))
    return true;

  // setup storage for best found
  shared_array<dReal> x_best(new dReal[n]);

  // setup vector of ones
  shared_array<dReal> ones_m(new dReal[m]);
  dSetValue(ones_m.get(), m, (dReal) 1.0);

  // setup mu*m
  const dReal mum = cparams.mu*m;

  // setup nu
  shared_array<dReal> nu(new dReal[nu_len]);
  dSetZero(nu.get(),nu_len);

  // determine f(x)
  shared_array<dReal> f(new dReal[m+1]);
  (*cparams.fx)(x, n, f.get(), m, cparams.data);
  f0_best = f[0];
  memcpy(x_best.get(), x, sizeof(dReal)*n);

  // verify that f is non-positive for constraint functions
  shared_array<dReal> fc(new dReal[m]);
  memcpy(fc.get(), f.get()+1, sizeof(dReal)*m);
  //dReal maxfc = *std::max_element(fc.get(), fc.get()+m);
  dUASSERT(maxfc <= 0.0, "initial point infeasible");

  // setup lambda: must be greater than zero
  shared_array<dReal> lambda(new dReal[m]);
  for (int i=0; i< m; i++)
    lambda[i] = (fc[i] < (dReal) 0.0) ? (dReal) -1.0/fc[i] : dRecip(NEAR_ZERO);

  // setup eta
  dReal eta = -dDot(fc.get(), lambda.get(), m);

  // init t
  dReal t = mum/eta;

  // init y and r
  shared_array<dReal> y(new dReal[n+m+nu_len]);
  shared_array<dReal> r(new dReal[n+m+nu_len]);

  // init vectors and matrices so we don't have to keep reallocating memory
  shared_array<dReal> x_plus(new dReal[n]);
  shared_array<dReal> dx(new dReal[n]);
  shared_array<dReal> lambda_plus(new dReal[m]);
  shared_array<dReal> dlambda(new dReal[m]);
  shared_array<dReal> nu_plus(new dReal[nu_len]);
  shared_array<dReal> dnu(new dReal[nu_len]);
  shared_array<dReal> Df(new dReal[m*n]);
  shared_array<dReal> inv_t(new dReal[m]);
  shared_array<dReal> l_fc(new dReal[m]);
  shared_array<dReal> tmp1(new dReal[n]);
  vector<shared_array<dReal> > g(m+1);
  for (int i=0; i< m+1; i++)
    g[i] = shared_array<dReal>(new dReal[n]);
  shared_array<dReal> H(new dReal[n*n]);
  shared_array<dReal> tmp2(new dReal[(n+nu_len)*(n+nu_len)]);
  shared_array<dReal> M(new dReal[(n+nu_len)*(n+nu_len)]);
  shared_array<dReal> rdual(new dReal[n]);
  shared_array<dReal> rcent(new dReal[m]);
  shared_array<dReal> rpri(new dReal[nu_len]);
  shared_array<dReal> rhs(new dReal[n+nu_len]);
  shared_array<dReal> tmp3(new dReal[m*m]);
  shared_array<dReal> tmp4(new dReal[m]);

  for (cparams.iterations = 0; cparams.iterations < cparams.max_iterations; cparams.iterations++)
  {
    // zero temporaries
    dSetZero(tmp1.get(), n);
    dSetZero(tmp2.get(), n*n);
    dSetZero(tmp3.get(), m*m);
    dSetZero(tmp4.get(), m);

    // reset t
    t = mum/eta;

    #ifdef DEBUG
    cout << "-------------------------------------------------------" << endl;
    cout << "iteration: " << cparams.iterations << endl;
    cout << " t: "  << t << endl;
    cout << " x: ";
    printMatrix(x, 1, n);
    cout << " lambda: ";
    printMatrix(lambda.get(), 1, m);
    #endif

    // compute the residual
    for (int i=0; i< m+1; i++) (*cparams.grad)(x, n, i, g[i].get(), cparams.data);
    (*cparams.fx)(x, n, f.get(), m, cparams.data);
    memcpy(fc.get(), f.get()+1, sizeof(dReal)*m);
    for (int i=0, k=0; i< m; i++)
      for (int j=0; j< n; j++)
        Df[k++] = g[i+1][j];
    dSetZero(r.get(), n+m+nu_len);
    dNPMultiply1(r.get(), Df.get(), lambda.get(), n, m, 1);
    dNPMultiply1(tmp1.get(), cparams.A, nu.get(), n, nu_len, 1);
    for (int i=0; i< n; i++) r[i] += tmp1[i] + g[0][i];
    dSetValue(inv_t.get(), m, (dReal) 1.0);
    for (int i=0; i< m; i++) inv_t[i] /= t;
    memcpy(l_fc.get(), fc.get(), m*sizeof(dReal));
    for (int i=0; i< m; i++)  l_fc[i] *= -lambda[i];
    memcpy(r.get()+n, l_fc.get(), m*sizeof(dReal));
    for (int i=0; i< m; i++)  r[n+i] -= inv_t[i];
    dNPMultiply0(r.get()+m+n, cparams.A, x, nu_len, n, 1);
    for (int i=0; i< nu_len; i++)  r[n+m+i] -= cparams.b[i];

    #ifdef DEBUG
    cout << "Df: ";
    printMatrix(Df.get(), m, n);
    cout << " f0: " << f[0] << endl;
    cout << " f: ";
    printMatrix(fc.get(), 1, m);
    cout << " rdual: ";
    printMatrix(r.get(), 1, n);
    cout << " rcent: ";
    printMatrix(r.get()+n, 1, m);
    #endif

    // solve KKT equations for dx, dlambda, dnu
    if (!(*cparams.hess)(x, n, 0, H.get(), cparams.data))
      dSetZero(H.get(), n*n);
    for (int i=0; i< m; i++)  { dIASSERT(fc[i] < 0.0); fc[i] = dRecip(fc[i]); }
    for (int i=0; i< m; i++)
    {
      if ((*cparams.hess)(x, n, i+1, tmp2.get(), cparams.data))
        for (int j=0; j< n*n; j++)  H[j] += tmp2[j] * lambda[i];
      dNPMultiply2(tmp2.get(), g[i+1].get(), g[i+1].get(), n, 1, n);
      for (int j=0; j< n*n; j++)  tmp2[j] *= lambda[i]*fc[i];
      for (int j=0; j< n*n; j++)  H[j] -= tmp2[j];
    }
    #ifdef DEBUG
    cout << "Hessian matrix [unconditioned]: " << std::endl;
    printMatrix(H.get(), n, n);
    #endif
    conditionHessian(H.get(), tmp2.get(), n);
    setupM(M.get(), H.get(), n, cparams.A, nu_len); 
    memcpy(rdual.get(), r.get(), n*sizeof(dReal));
    memcpy(rcent.get(), r.get()+n, m*sizeof(dReal));
    memcpy(rpri.get(), r.get()+n+m, nu_len*sizeof(dReal));
    dSetZero(tmp3.get(), m*m);
    for (int i=0; i< m; i++) tmp3[m*i+i] = fc[i];
    dNPMultiply0(tmp4.get(), tmp3.get(), rcent.get(), m, m, 1);    
    dNPMultiply1(tmp1.get(), Df.get(), tmp4.get(), n, m, 1);
    for (int i=0; i< n; i++) rhs[i] = -(rdual[i] + tmp1[i]);
    for (int i=0; i< nu_len; i++) rhs[i+n] = -rpri[i];
    #ifdef DEBUG
    cout << "M (KKT matrix) [conditioned]: " << std::endl;
    printMatrix(M.get(), n+nu_len, n+nu_len);
    cout << "rhs: ";
    printMatrix(rhs.get(), 1, n+nu_len);
    #endif
    if (nu_len > 0)
    {
      solveSymmetric(M.get(), rhs.get(), n+nu_len);
    }
    else
    {
      dNPFactorCholesky(M.get(), n);
      dNPSolveCholesky(M.get(), rhs.get(), n);
    }
for (int i=0; i< n+nu_len; i++)
dIASSERT(!isnan(rhs[i]));
    memcpy(dx.get(), rhs.get(), n*sizeof(dReal));
    memcpy(dnu.get(), rhs.get()+n, nu_len*sizeof(dReal));
    dSetZero(tmp3.get(), m*m);
    for (int i=0; i< m; i++) tmp3[i*m+i] = fc[i]*lambda[i];
    dNPMultiply0(tmp4.get(), Df.get(), dx.get(), m, n, 1);
    dNPMultiply0(dlambda.get(), tmp3.get(), tmp4.get(), m, m, 1);
    for (int i=0; i< m; i++)
      dlambda[i] = -dlambda[i] + fc[i]*rcent[i];
    #ifdef DEBUG
    cout << " dx: ";
    printMatrix(dx.get(), 1, n);
    cout << " dlambda: ";
    printMatrix(dlambda.get(), 1, m);
    #endif

    // compute primal-dual search direction
    const dReal dir_norm = dSqrt(dDot(rhs.get(), rhs.get(), n+nu_len));

    // prepare to do backtracking line search
    dReal smax = (dReal) 1.0;
    for (int i=0; i< m; i++)
      if (dlambda[i] < 0)
      {
        smax = std::min(smax, -lambda[i]/dlambda[i]);
        if (smax < std::numeric_limits<dReal>::epsilon())
          lambda[i] += NEAR_ZERO;
      }
    dReal s = (dReal) 0.99 * smax;
    #ifdef DEBUG
    cout << " maximum s: " << s << endl;
    #endif
    for (int i=0; i< n; i++) x_plus[i] = x[i] + dx[i]*s;
    for (int i=0; i< m; i++) lambda_plus[i] = lambda[i] + dlambda[i]*s;
    for (int i=0; i< nu_len; i++) nu_plus[i] = nu[i] + dnu[i]*s;

    // satisfy inequality constraints 
    (*cparams.fx)(x_plus.get(), n, f.get(), m, cparams.data);
    memcpy(fc.get(), f.get()+1, sizeof(dReal)*m);
    #ifdef DEBUG
    cout << " preparing to attempt to satisfy inequality constraints..." << endl;
    #endif
    while (!feasible(fc.get(), m))
    {
      #ifdef DEBUG
      cout << "    s: " << s << "  fc: ";
      printMatrix(fc.get(), 1, m);
      #endif
      s *= cparams.beta;
      if (s*dir_norm < std::numeric_limits<dReal>::epsilon())
      {
        #ifdef DEBUG
        cout << "*** s too small: exiting ***" << endl;
        #endif
        memcpy(x, x_best.get(), sizeof(dReal)*n);
        return false;
      }

      // update f
      for (int i=0; i< n; i++) x_plus[i] = x[i] + dx[i]*s;
      (*cparams.fx)(x_plus.get(), n, f.get(), m, cparams.data);
      memcpy(fc.get(), f.get()+1, sizeof(dReal)*m);
    }

    // update lambda and nu
    for (int i=0; i< m; i++) lambda_plus[i] = lambda[i] + dlambda[i]*s;
    for (int i=0; i< nu_len; i++) nu_plus[i] = nu[i] + dnu[i]*s;

    // re-evaluate the objective function
    (*cparams.fx)(x_plus.get(), n, f.get(), m, cparams.data);
    if (f[0] < f0_best)
    {
      f0_best = f[0];
      memcpy(x_best.get(), x, sizeof(dReal)*n);
    }

    // determine norm of r target
    const dReal rnorm = dSqrt(dDot(r.get(), r.get(), n+m+nu_len));
    #ifdef DEBUG
    cout << " current (old) residual: ";
    printMatrix(r.get(), 1, n+m+nu_len);
    #endif

    // compute new residual
    dSetZero(r.get(), n+m+nu_len);
    dSetZero(tmp1.get(), n);
    for (int i=0; i< m+1; i++) (*cparams.grad)(x_plus.get(), n, i, g[i].get(), cparams.data);
    (*cparams.fx)(x_plus.get(), n, f.get(), m, cparams.data);
    memcpy(fc.get(), f.get()+1, sizeof(dReal)*m);
    for (int i=0, k=0; i< m; i++)
      for (int j=0; j< n; j++)
        Df[k++] = g[i+1][j];
    dNPMultiply1(r.get(), Df.get(), lambda_plus.get(), n, m, 1);
    dNPMultiply1(tmp1.get(), cparams.A, nu_plus.get(), n, nu_len, 1);
    for (int i=0; i< n; i++) r[i] += tmp1[i] + g[0][i];
    dSetValue(inv_t.get(), m, (dReal) 1.0);
    for (int i=0; i< m; i++) inv_t[i] /= t;
    memcpy(l_fc.get(), fc.get(), m*sizeof(dReal));
    for (int i=0; i< m; i++)  l_fc[i] *= -lambda_plus[i];
    memcpy(r.get()+n, l_fc.get(), m*sizeof(dReal));
    for (int i=0; i< m; i++)  r[n+i] -= inv_t[i];
    dNPMultiply0(r.get()+m+n, cparams.A, x_plus.get(), nu_len, n, 1);
    for (int i=0; i< nu_len; i++)  r[n+m+i] -= cparams.b[i];

    // do backtracking line search
    #ifdef DEBUG 
    cout << " starting BLS..." << endl;
    #endif
    while (dSqrt(dDot(r.get(), r.get(), n+m+nu_len)) > (1-cparams.alpha*s)*rnorm)
    {
      #ifdef DEBUG
      cout << "  s: " << s << endl;
      #endif
      s *= cparams.beta;

      // verify that s is not too small
      if (s*dir_norm < std::numeric_limits<dReal>::epsilon())
      {
        memcpy(x, x_best.get(), sizeof(dReal)*n);
        #ifdef DEBUG
        cout << "  *** s is too small: exiting ***" << endl;
        #endif
        return false;
      }

      // update x+, lambda+, nu+
      for (int i=0; i< n; i++) x_plus[i] = x[i] + dx[i]*s;
      for (int i=0; i< m; i++) lambda_plus[i] = lambda[i] + dlambda[i]*s;
      for (int i=0; i< nu_len; i++) nu_plus[i] = nu[i] + dnu[i]*s;

      // re-valuate the objective function
      (*cparams.fx)(x_plus.get(), n, f.get(), m, cparams.data);
      if (f[0] < f0_best)
      {
        f0_best = f[0];
        memcpy(x_best.get(), x, sizeof(dReal)*n);
      }

      // recalculate the residual
      dSetZero(r.get(), n+m+nu_len);
      dSetZero(tmp1.get(), n);
      for (int i=0; i< m+1; i++) (*cparams.grad)(x_plus.get(), n, i, g[i].get(), cparams.data);
      (*cparams.fx)(x_plus.get(), n, f.get(), m, cparams.data);
      memcpy(fc.get(), f.get()+1, sizeof(dReal)*m);
      for (int i=0, k=0; i< m; i++)
        for (int j=0; j< n; j++)
          Df[k++] = g[i+1][j];
      dNPMultiply1(r.get(), Df.get(), lambda_plus.get(), n, m, 1);
      dNPMultiply1(tmp1.get(), cparams.A, nu_plus.get(), n, nu_len, 1);
      for (int i=0; i< n; i++) r[i] += tmp1[i] + g[0][i];
      dSetValue(inv_t.get(), m, (dReal) 1.0);
      for (int i=0; i< m; i++) inv_t[i] /= t;
      memcpy(l_fc.get(), fc.get(), m*sizeof(dReal));
      for (int i=0; i< m; i++)  l_fc[i] *= -lambda_plus[i];
      memcpy(r.get()+n, l_fc.get(), m*sizeof(dReal));
      for (int i=0; i< m; i++)  r[n+i] -= inv_t[i];
      dNPMultiply0(r.get()+m+n, cparams.A, x_plus.get(), nu_len, n, 1);
      for (int i=0; i< nu_len; i++)  r[n+m+i] -= cparams.b[i];
      #ifdef DEBUG
      cout << "   determined residual: ";
      printMatrix(r.get(), 1, n+m+nu_len);
      #endif
    }
    #ifdef DEBUG
    cout << "  new r: ";
    printMatrix(r.get(), 1, n+m+nu_len);
    #endif

    // get x, lambda, and nu
    memcpy(x, x_plus.get(), n*sizeof(dReal));
    std::swap(lambda, lambda_plus);
    std::swap(nu, nu_plus);
    #ifdef DEBUG
    printMatrix(x, 1, n);
    #endif

    // see whether we can quit
    if (cparams.tcheck && (*cparams.tcheck)(x, n, cparams.data))
    {
      #ifdef DEBUG
      cout << " *** termination check indicates we can quit ***" << endl;
      #endif
      return true;
    }

    // redetermine eta
    eta = -dDot(fc.get(), lambda.get(), m);

    // recompute primary and dual norms
    rdual_nrm = dSqrt(dDot(r.get(), r.get(), n));
    rpri_nrm = dSqrt(dDot(r.get()+n+m, r.get()+n+m, nu_len));

    // check for convergence
    if (rpri_nrm <= eps_feas && rdual_nrm <= eps_feas && eta < cparams.eps)
    {
      #ifdef DEBUG
      cout << "solution is within tolerances... exiting" << endl;
      #endif
      memcpy(x, x_best.get(), n*sizeof(dReal));
      return true;
    }
  }

  #ifdef DEBUG
  cout << " *** maximum number of iterations exceeded... exiting" << endl;
  #endif

  // if we're here, the max number of iterations has been exceeded
  memcpy(x, x_best.get(), n*sizeof(dReal));
  return false;
}


