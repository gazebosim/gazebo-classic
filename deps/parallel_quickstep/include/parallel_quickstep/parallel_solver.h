#ifndef PARALLEL_SOLVER_H
#define PARALLEL_SOLVER_H

#include "parallel_array.h"
#include "parallel_batch.h"
#include "parallel_math.h"
#include "parallel_reduce.h"
#include "parallel_timer.h"

#include "util.h"

namespace parallel_ode
{

using ::parallel_utils::ParallelHDArray;
using ::parallel_utils::MemManager;
using ::parallel_utils::CopyType;


namespace ParallelFlags
{
//! Flags for configuring the solver
enum ParallelFlag { PARALLEL_NONE       = 0x000,
                    PARALLEL_PREPROCESS = 0x001,
                    PARALLEL_ASYNC      = 0x002,
                    PARALLEL_RANDOMIZE  = 0x004,
                    PARALLEL_ATOMICS    = 0x008,
                    PARALLEL_ALIGN      = 0x010,
                    PARALLEL_PINNED     = 0x020,
                    PARALLEL_WC         = 0x040,
                    PARALLEL_REDUCE     = 0x080,
                    PARALLEL_COMPACT    = 0x100};
}
typedef ParallelFlags::ParallelFlag ParallelFlag;

//! Default configuration flags for the base solver
const int DEFAULT_FLAGS =
    ParallelFlags::PARALLEL_ALIGN
    | ParallelFlags::PARALLEL_PINNED
    | ParallelFlags::PARALLEL_REDUCE
    | ParallelFlags::PARALLEL_ASYNC
    | ParallelFlags::PARALLEL_RANDOMIZE
    ;//| PARALLEL_ATOMICS;

//! The base solver class
/*  ParallelPGSSolver is the abstract base class encapsulating functionality common to each of the parallel
 *  Quickstep solvers.  It takes the data from ODE, batches it appropriately, and relies on derived classes
 *  to actually drive the solver.  From there the data is transferred back to ODE.
 */
template<typename CudaT, typename ParamsT, ParallelType PType>
class ParallelPGSSolver
{
public:
  struct SolverParams;
  typedef typename vec4<CudaT>::Type Vec4T;
  typedef const CudaT* CudaTPtr;
  typedef CudaT* CudaTMutablePtr;
  typedef const ParamsT* ParamsTPtr;
  typedef ParamsT* ParamsTMutablePtr;
  typedef MemManager<CudaT,PType> PMemManager;
  typedef typename PMemManager::mem_flags MemFlags;

  /**
   * @brief Constructs a generic parallel quickstep solver
   *
   * @param parallelFlags Flags controller various parallel functionality
   * @param batchType The type of batch strategy to be used
   * @param reduceType The Type of reduction to be used
   * @param numBatches The maximum number of batches to be used, if any
   */
  ParallelPGSSolver( int parallelFlags = DEFAULT_FLAGS,
                     BatchType batchType = BatchTypes::DEFAULT_BATCH_TYPE,
                     ReduceType reduceType = ReduceTypes::DEFAULT_REDUCE_TYPE,
                     uint numBatches = ParallelOptions::MAXBATCHES )
      : parallelFlags_( parallelFlags ),
        parallelParams_( NULL ),
        batchStrategyType_( batchType ),
        batchStrategy_( NULL ),
        reduceStrategyType_( reduceType),
        reduceStrategy_( NULL ),
        batchRepetitionCount_( numBatches, 0 ),
        batchIndices_( numBatches, -1 ),
        batchSizes_( numBatches, 0 ),
        numBatches_( numBatches ),
        m_(0),
        constraintStride_(0),
        n_(0),
        bodyStride_(0),
        reduceStride_(0),
        bInit_(false) {
  }

  /**
   * @brief Destroys the parallel quickstep solver
   */
  virtual ~ParallelPGSSolver( ) {
    delete batchStrategy_;
    delete reduceStrategy_;
  }

  /**
   * @brief Solves the LCP given the data from ODE
   *
   * @param params Containts all pointers to data from ODE data structures necessary for the solve
   */
  virtual void worldSolve( SolverParams* params );

  /**
   * @brief Overloaded initialization of the solver, including batch and reduce strategies
   */
  virtual void initialize( );

protected:

  ///////////////////////////////////////////////////////////////////////////////////

  void syncODEToDevice( );
  void syncDeviceToODE( );
  void loadBatches( int* jb );

  void setConstraintSize( int m, bool bResizeArrays = true );
  void setBodySize( int n, bool bResizeArrays = true );

  ///////////////////////////////////////////////////////////////////////////////////

  virtual void preProcessHost( );
  virtual void preProcessDevice( const CudaT sorParam, const CudaT stepsize );
  virtual void solveAndReduce( const int offset, const int batchSize ) = 0;

  virtual void loadBodies( );
  virtual void loadConstraints( );
  virtual void loadConstants( );
  virtual void loadKernels( );
  virtual void loadSolution( );

  virtual void setMemFlags( MemFlags flags );

  ///////////////////////////////////////////////////////////////////////////////////

  inline void checkInit( ) { if(!bInit_) this->initialize(); }
  inline void setBodyStride( int bodyStride ) { bodyStride_ = bodyStride; }
  inline void setConstraintStride( int constraintStride ) { constraintStride_ = constraintStride; }
  inline void setReduceStride( int reduceStride ) { reduceStride_ = reduceStride; }

  inline int getNumBatches( ) const { return (int)batchSizes_.size(); }
  inline int getNumBodies( ) const { return n_; }
  inline int getBodyStride( ) const { return bodyStride_; }
  inline int getNumConstraints( ) const { return m_; }
  inline int getConstraintStride( ) const { return constraintStride_; }
  inline int getReduceStride( ) const { return reduceStride_; }
  inline int getDefaultAlign( ) const { return ParallelOptions::DEFAULTALIGN; }
  inline CopyType getCopyType( ) const { return asyncEnabled( ) ? parallel_utils::CopyTypes::COPY_ASYNC : parallel_utils::CopyTypes::COPY_SYNC; }

  inline int bID(const int index, const int offset) const { return index + offset * getBodyStride(); }
  inline int cID(const int index, const int offset) const { return index + offset * getConstraintStride(); }
  inline int rID(const int index, const int offset) const { return index + offset * getReduceStride(); }

  inline bool preprocessEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_PREPROCESS ); }
  inline bool asyncEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_ASYNC ); }
  inline bool randomizeEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_RANDOMIZE ); }
  inline bool atomicsEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_ATOMICS ); }
  inline bool alignEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_ALIGN ); }
  inline bool pinnedMemEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_PINNED ); }
  inline bool wcMemEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_WC ); }
  inline bool reduceEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_REDUCE ); }
  inline bool compactScalarsEnabled( ) const { return isEnabled( ParallelFlags::PARALLEL_COMPACT ); }

  inline bool isEnabled( const ParallelFlag flag ) const { return parallelFlags_ & flag; }

  void printConfig( );

  ///////////////////////////////////////////////////////////////////////////////////

  int parallelFlags_;                   /**< Flags that govern the behavior of the solver, e.g. atomics,alignment, etc... */
  SolverParams* parallelParams_;        /**< Parameter struct that contains the data from ODE */

  BatchType batchStrategyType_;         /**< The type of batching to use */
  BatchStrategy *batchStrategy_;        /**< The actual strategy type implementation */
  ReduceType reduceStrategyType_;       /**< The type of reduction to use */
  ReduceStrategy *reduceStrategy_;      /**< The actual reduction type implemention */

  IntVector constraintIndices_;         /**< Per Constraint: Contains the global reorded constraint indices */
  IntVector reductionOffsets0_;         /**< Per Constraint: Specifies how many times a body0 is repeated in a given batch  */
  IntVector reductionOffsets1_;         /**< Per Constraint: Specifies how many times a body1 is repeated in a given batch  */
  IntVector batchRepetitionCount_;      /**< Per Batch: The maximum number of times a body is repeated for each batch */
  IntVector batchIndices_;              /**< Per Batch: The starting index of each batch in the global constraint array */
  IntVector batchSizes_;                /**< Per Batch: The size of each batch */

  ///////////////////////////////////////////////////////////////////////////////////

  int numBatches_;                      /**< The (max) number of batches */

  int m_;                               /**< The number of constraints */
  int constraintStride_;                /**< The aligned number of constraints */
  int n_;                               /**< The number of bodies */
  int bodyStride_;                      /**< The aligned number of bodies */

  int reduceStride_;                    /**< The aligned number of reduction entries */

  bool bInit_;                          /**< Flag indiciating whether the solver has been explicitly initialized */

  ParallelHDArray<Vec4T,PType> j0;      /**< Host/Device array containg data for j,  m * 4 */
  ParallelHDArray<Vec4T,PType> ij0;     /**< Host/Device array containg data for ij, m * 4 */
  ParallelHDArray<CudaT,PType> lambda0;     /**< Host/Device array containing the solution */
  ParallelHDArray<CudaT,PType> adcfm;       /**< Host/Device array containing the computd Adcfm */
  ParallelHDArray<CudaT,PType> rhs;         /**< Host/Device array containing the rhs */
  ParallelHDArray<CudaT,PType> lohiD;       /**< Host/Device array containg both lo/hi constraints, */
  ParallelHDArray<int4,PType> bodyIDs;  /**< Host/Device array containing both body IDs and body reduction IDs */
  ParallelHDArray<int,PType> fIDs;      /**< Host/Device array containg the force IDs */

  ParallelHDArray<CudaT,PType> iMass;       /**< Host/Device array containing the inverse of the body mass */
  ParallelHDArray<Vec4T,PType> i0;      /**< Host/Device array containing the inertia, n * 3 */

  ParallelHDArray<Vec4T,PType> bodyFAcc; /**< Host/Device array containing the final body linear accelaration */
  ParallelHDArray<Vec4T,PType> bodyTAcc; /**< Host/Device array containing the final body rotational accelaration */
  ParallelHDArray<Vec4T,PType> bodyFAccReduction; /**< Host/Device array containing the reduction buffers for linear accelaration */
  ParallelHDArray<Vec4T,PType> bodyTAccReduction; /**< Host/Device array containing the reduction buffers for rotation acceleration */

public:

  //! Parameters containing data from ODE
  struct SolverParams {
    SolverParams( dxWorldProcessContext *contextIn, const dxQuickStepParameters *qsIn,
                  const int mIn, const int nbIn, ParamsTMutablePtr JIn, int *jbIn, dxBody * const *bodyIn,
                  ParamsTPtr invIIn, ParamsTMutablePtr lambdaIn, ParamsTMutablePtr fcIn, ParamsTMutablePtr bIn,
                  ParamsTPtr loIn, ParamsTPtr hiIn, ParamsTPtr cfmIn, ParamsTMutablePtr iMJIn, ParamsTMutablePtr AdcfmIn,
                  const int *findexIn, const ParamsT stepsizeIn )
    : context(contextIn), qs(qsIn), m(mIn),  nb(nbIn),  J(JIn), jb(jbIn), body(bodyIn),
      invI(invIIn),  lambda(lambdaIn),  fc(fcIn),  b(bIn),  lo(loIn),  hi(hiIn),  cfm(cfmIn),
      iMJ(iMJIn),  Adcfm(AdcfmIn), findex(findexIn),  stepsize(stepsizeIn) { }

    dxWorldProcessContext *context;
    const dxQuickStepParameters *qs;
    const int m;
    const int nb;
    ParamsTMutablePtr J;
    int *jb;
    dxBody * const *body;
    ParamsTPtr invI;
    ParamsTMutablePtr lambda;
    ParamsTMutablePtr fc;
    ParamsTMutablePtr b;
    ParamsTPtr lo;
    ParamsTPtr hi;
    ParamsTPtr cfm;
    ParamsTMutablePtr iMJ;
    ParamsTMutablePtr Adcfm;
    const int *findex;
    const ParamsT stepsize;
  };

};

}

#endif
