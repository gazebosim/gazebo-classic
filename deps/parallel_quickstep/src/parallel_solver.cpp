#include <parallel_solver.h>
#include <parallel_common.h>
#include <parallel_utils.h>
#include <parallel_stepper.h>

namespace parallel_ode
{

using ::parallel_utils::fillSequentialVector;
using ::parallel_utils::permuteVector;

// Forward template declarations
#if defined(USE_CUDA)
#ifdef CUDA_DOUBLESUPPORT
template class ParallelPGSSolver<dReal,dReal,ParallelTypes::PARALLEL_TYPE>;
#else
template class ParallelPGSSolver<float,dReal,ParallelTypes::PARALLEL_TYPE>;
#endif
#else
template class ParallelPGSSolver<dReal,dReal,ParallelTypes::PARALLEL_TYPE>;
#endif

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::initialize( )
{
  if(batchStrategy_) delete batchStrategy_;
  batchStrategy_ = BatchStrategyFactory::create( batchStrategyType_, numBatches_, alignEnabled(), getDefaultAlign() );

  if(reduceStrategy_) delete reduceStrategy_;
  reduceStrategy_ = ReduceStrategyFactory::create( reduceStrategyType_ );

  if( !bInit_ ) dxInitDevice();

  bInit_ = true;
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::worldSolve( SolverParams* params ) {

  checkInit();
  parallelParams_ = params;

  syncODEToDevice( );

  {
    IFTIMING( ParallelTimer timer( "solving LCP problem - parallel" ) );

    const int numIterations = parallelParams_->qs->num_iterations;
    const int numBatches = getNumBatches();

    IntVector batchOrder( numBatches );
    fillSequentialVector( batchOrder );

    for(int iteration = 0; iteration < numIterations; ++iteration) {

      if( randomizeEnabled( ) ) permuteVector( batchOrder );

      for(int batchCount = 0; batchCount < numBatches; ++batchCount) {

        const int batch = batchOrder[ batchCount ];
        const int offset = batchIndices_[ batch ];
        const int batchSize = batchSizes_[ batch ];

        if( batchSize == 0 ) continue;

        solveAndReduce( offset, batchSize );

      } // batch
    } // iteration
  }

  syncDeviceToODE( );

  parallelParams_ = NULL;
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::preProcessHost( )
{
  dSetZero (parallelParams_->lambda,parallelParams_->m);
  dSetZero (parallelParams_->fc,parallelParams_->nb*6);

  if( !preprocessEnabled( ) )
  {
    compute_invM_JT (parallelParams_->m,
                     parallelParams_->J,
                     parallelParams_->iMJ,
                     parallelParams_->jb,
                     parallelParams_->body,
                     parallelParams_->invI);
    compute_Adcfm_b (parallelParams_->m,
                     parallelParams_->qs->w,
                     parallelParams_->J,
                     parallelParams_->iMJ,
                     parallelParams_->jb,
                     parallelParams_->cfm,
                     parallelParams_->Adcfm,
                     parallelParams_->b);
  }
  else
  {
    parallelParams_->iMJ = NULL;

    //parallelParams_->Adcfm = const_cast<CudaTMutablePtr>(parallelParams_->cfm);
    for ( int i = 0; i < parallelParams_->m; i++)
      parallelParams_->Adcfm[i] = (CudaT)(parallelParams_->cfm[i]);
  }
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::preProcessDevice( const CudaT sorParam, const CudaT stepSize )
{
  IFTIMING( ParallelTimer timer(" -- preprocess parallel") );
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::syncODEToDevice( )
{
  preProcessHost( );

  // Defer array resizing until we've calculated necessary reduction sizes for body force accumulator
  setBodySize( parallelParams_->nb, false );
  setConstraintSize( parallelParams_->m, false );
  loadBatches( parallelParams_->jb );
  // With batch sizes computed we can now properly allocate the host/device buffers
  setBodySize( parallelParams_->nb, true );
  setConstraintSize( parallelParams_->m, true );

  loadBodies( );
  loadConstraints( );
  loadConstants( );
  loadKernels( );

  if( preprocessEnabled( ) ) {
    preProcessDevice( parallelParams_->qs->w, parallelParams_->stepsize );
  }

  IFVERBOSE( printConfig( ) );
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::syncDeviceToODE( )
{
  {
    IFTIMING( ParallelTimer timer(" -- sync Device to Host ") );
    CopyType copyType = getCopyType( );
    bodyFAcc.syncToHost( copyType );
    bodyTAcc.syncToHost( copyType );
    lambda0.syncToHost( copyType );
  }

  {
     IFTIMING( ParallelTimer timer(" -- load solution ") );
     loadSolution( );
  }
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::loadBatches( int* jb )
{
  IFTIMING( ParallelTimer timer("  -- load batches ") );
  int maxRepetitionCount = batchStrategy_->batch(jb,
                                                 getNumConstraints(),
                                                 getNumBodies(),
                                                 constraintIndices_,
                                                 reductionOffsets0_,
                                                 reductionOffsets1_,
                                                 batchRepetitionCount_,
                                                 batchIndices_,
                                                 batchSizes_);

  reduceStrategy_->initialize( getNumBodies( ), min(maxRepetitionCount, ParallelOptions::MAXBODYREPETITION), batchRepetitionCount_ );
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::loadBodies( )
{
  IFTIMING( ParallelTimer timer("  -- load bodies ") );

  const CopyType copyType = getCopyType( );

  Vec4T *bodyFAccP = bodyFAcc.getHostBuffer();
  Vec4T *bodyTAccP = bodyTAcc.getHostBuffer();
  Vec4T zeroVec = make_vec4( (CudaT)0.0 );

  for(size_t bodyID = 0; bodyID < bodyFAcc.getSize(); ++bodyID )
  {
    bodyFAccP[ bodyID ] = zeroVec;
    bodyTAccP[ bodyID ] = zeroVec;
  }
  bodyFAcc.syncToDevice( copyType );
  bodyTAcc.syncToDevice( copyType );

  if( preprocessEnabled( ) )
  {
    CudaT *iMassP = iMass.getHostBuffer();
    const ParamsT *invIP = parallelParams_->invI;
    Vec4T *iP = i0.getHostBuffer();

    for(int bodyID = 0; bodyID < getNumBodies(); ++bodyID, invIP += 12)
    {
      iMassP[ bodyID ] = parallelParams_->body[ bodyID ]->invMass;
      iP[ bID(bodyID,0) ] = make_vec4( (CudaT)invIP[0], (CudaT)invIP[1], (CudaT)invIP[2]);
      iP[ bID(bodyID,1) ] = make_vec4( (CudaT)invIP[4], (CudaT)invIP[5], (CudaT)invIP[6]);
      iP[ bID(bodyID,2) ] = make_vec4( (CudaT)invIP[8], (CudaT)invIP[9], (CudaT)invIP[10]);
    }
    iMass.syncToDevice( copyType );
    i0.syncToDevice( copyType );
  }
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::loadConstraints( )
{
  IFTIMING( ParallelTimer timer("  -- load constraints ") );
  const CopyType copyType = getCopyType( );

  CudaT *adcfmP = adcfm.getHostBuffer();
  CudaT *lambdaP = lambda0.getHostBuffer();
  CudaT *rhsP = rhs.getHostBuffer();
  CudaT *lohiP = lohiD.getHostBuffer();
  int4 *bodyIDsP = bodyIDs.getHostBuffer();
  int *fIDP = fIDs.getHostBuffer();
  Vec4T *jP = j0.getHostBuffer();

  for(size_t batchID = 0, hIndex = 0; batchID < batchSizes_.size(); ++batchID)
  {
    size_t dIndex = batchIndices_[ batchID ];
    for(int batchCount = 0; batchCount < batchSizes_[ batchID ]; ++batchCount,++hIndex,++dIndex)
    {
      const int scalarIndex = constraintIndices_[hIndex];
      const int vecIndex = scalarIndex * 12;

      const int body0ID = parallelParams_->jb[ scalarIndex*2   ];
      const int body1ID = parallelParams_->jb[ scalarIndex*2+1 ];

      const int body0ReductionID = reduceStrategy_->getFinalIndex( body0ID, (reductionOffsets0_[ hIndex ] %  ParallelOptions::MAXBODYREPETITION) );
      const int body1ReductionID = body1ID < 0 ? -1 : reduceStrategy_->getFinalIndex( body1ID, (reductionOffsets1_[ hIndex ] %  ParallelOptions::MAXBODYREPETITION) );

      bodyIDsP[dIndex]  = make_int4( body0ID, body1ID, body0ReductionID, body1ReductionID );

      fIDP[dIndex]   = parallelParams_->findex[ scalarIndex ];
      rhsP[dIndex]   = parallelParams_->b[ scalarIndex ];
      adcfmP[dIndex] = parallelParams_->Adcfm[ scalarIndex ];

      lohiP[ dIndex ]          = parallelParams_->lo[ scalarIndex ];
      lohiP[ cID(dIndex,1) ]   = parallelParams_->hi[ scalarIndex ];

      jP[ dIndex ]           = make_vec4( (CudaT)parallelParams_->J[vecIndex],   (CudaT)parallelParams_->J[vecIndex+1],  (CudaT)parallelParams_->J[vecIndex+2] );
      jP[ cID(dIndex,1) ]    = make_vec4( (CudaT)parallelParams_->J[vecIndex+3], (CudaT)parallelParams_->J[vecIndex+4],  (CudaT)parallelParams_->J[vecIndex+5] );
      jP[ cID(dIndex,2) ]    = make_vec4( (CudaT)parallelParams_->J[vecIndex+6], (CudaT)parallelParams_->J[vecIndex+7],  (CudaT)parallelParams_->J[vecIndex+8] );
      jP[ cID(dIndex,3) ]    = make_vec4( (CudaT)parallelParams_->J[vecIndex+9], (CudaT)parallelParams_->J[vecIndex+10], (CudaT)parallelParams_->J[vecIndex+11] );

      lambdaP[dIndex] = 0.0;
    }
  }

  adcfm.syncToDevice( copyType );
  lambda0.syncToDevice( copyType );
  rhs.syncToDevice( copyType );
  lohiD.syncToDevice( copyType );
  fIDs.syncToDevice( copyType );
  j0.syncToDevice( copyType );
  bodyIDs.syncToDevice( copyType );
  fIDs.syncToDevice( copyType );


  if( !preprocessEnabled( ) )
  {
    Vec4T* imjP = ij0.getHostBuffer();
    ParamsTPtr iMJ = parallelParams_->iMJ;
    for(size_t batchID = 0, hIndex = 0; batchID < batchSizes_.size(); ++batchID)
    {
      size_t dIndex = batchIndices_[ batchID ];
      for(int batchCount = 0; batchCount < batchSizes_[ batchID ]; ++batchCount,++hIndex,++dIndex)
      {
        const int vecIndex = constraintIndices_[hIndex] * 12;

        imjP[ dIndex ]         = make_vec4( (CudaT)iMJ[vecIndex],   (CudaT)iMJ[vecIndex+1],  (CudaT)iMJ[vecIndex+2] );
        imjP[ cID(dIndex,1) ]  = make_vec4( (CudaT)iMJ[vecIndex+3], (CudaT)iMJ[vecIndex+4],  (CudaT)iMJ[vecIndex+5] );
        imjP[ cID(dIndex,2) ]  = make_vec4( (CudaT)iMJ[vecIndex+6], (CudaT)iMJ[vecIndex+7],  (CudaT)iMJ[vecIndex+8] );
        imjP[ cID(dIndex,3) ]  = make_vec4( (CudaT)iMJ[vecIndex+9], (CudaT)iMJ[vecIndex+10], (CudaT)iMJ[vecIndex+11] );
      }
    }

    ij0.syncToDevice( copyType );
  }
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::loadConstants( )
{

}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::loadKernels( )
{

}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::loadSolution( )
{
  IFVERBOSE( lambda0.print("LAMBDA") );
  IFVERBOSE( bodyFAcc.print("BODYFACC") );
  IFVERBOSE( bodyTAcc.print("BODYTACC") );

  dxGlobalSync();

  CudaT *lambdaP = lambda0.getHostBuffer();
  Vec4T *fcP0 = bodyFAcc.getHostBuffer();
  Vec4T *fcP1 = bodyTAcc.getHostBuffer();

  for(size_t batchID = 0, hIndex = 0; batchID < batchSizes_.size(); ++batchID)
  {
    size_t dIndex = batchIndices_[ batchID ];
    for(int batchCount = 0; batchCount < batchSizes_[ batchID ]; ++batchCount,++hIndex,++dIndex)
    {
      parallelParams_->lambda[ constraintIndices_[ hIndex ] ] = lambdaP[ dIndex ];
    }
  }

  for(int bodyID = 0, fID = 0; bodyID < getNumBodies(); ++bodyID, fID+=6)
  {
    parallelParams_->fc[ fID    ] = fcP0[ bodyID ].x;
    parallelParams_->fc[ fID + 1] = fcP0[ bodyID ].y;
    parallelParams_->fc[ fID + 2] = fcP0[ bodyID ].z;
    parallelParams_->fc[ fID + 3] = fcP1[ bodyID ].x;
    parallelParams_->fc[ fID + 4] = fcP1[ bodyID ].y;
    parallelParams_->fc[ fID + 5] = fcP1[ bodyID ].z;
  }
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::setBodySize( int n, bool bResizeArrays )
{
  IFTIMING( ParallelTimer timer("  -- setting body size ") );
  n_ = n;

  if( bResizeArrays )
  {
    setBodyStride( alignDefaultSize( n_ ) );
    setReduceStride( alignDefaultSize( reduceStrategy_->getBodySizeWithReduction( ) ) );

    if( preprocessEnabled( ) )
    {
      iMass.setSize( n );
      i0.setSize( getBodyStride() * 3 );
    }

    bodyFAcc.setSize( getBodyStride() );
    bodyTAcc.setSize( getBodyStride() );

    if( reduceEnabled( ) ) {
      bodyFAccReduction.setSize( getReduceStride() );
      bodyTAccReduction.setSize( getReduceStride() );
    }
  }
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::setConstraintSize( int m, bool bResizeArrays )
{
  IFTIMING( ParallelTimer timer("  -- setting constraint size ") );
  m_ = m;

  constraintIndices_.resize(m);
  reductionOffsets0_.resize(m);
  reductionOffsets1_.resize(m);

  if( bResizeArrays )
  {
    setConstraintStride( parallel_utils::alignedSize( batchSizes_ ) );

    if( alignEnabled( ) ) {
      m = getConstraintStride( );
    }

    j0.setSize( m * 4 );
    ij0.setSize( m * 4 );

    if( compactScalarsEnabled( ) ) {
      adcfm.setSize( m * 4 );
    } else {
      adcfm.setSize( m );
      rhs.setSize( m );
      lohiD.setSize( m * 2 );
    }

    lambda0.setSize( m );
    bodyIDs.setSize( m );
    fIDs.setSize( m );
  }
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::setMemFlags( MemFlags flags )
{
  j0.setFlags( flags );
  ij0.setFlags( flags );
  lambda0.setFlags( flags );
  adcfm.setFlags( flags );
  rhs.setFlags( flags );
  lohiD.setFlags( flags );
  bodyIDs.setFlags( flags );
  fIDs.setFlags( flags );
  iMass.setFlags( flags );
  i0.setFlags( flags );
  bodyFAcc.setFlags( flags );
  bodyTAcc.setFlags( flags );
  bodyFAccReduction.setFlags( flags );
  bodyTAccReduction.setFlags( flags );
}

template<typename CudaT, typename ParamsT, ParallelType PType>
void ParallelPGSSolver<CudaT,ParamsT,PType>::printConfig( )
{
  if( getNumBodies() <= 0 || getNumConstraints() <= 0 ) return;

  IFVERBOSE( lambda0.print("j0") );
  IFVERBOSE( bodyFAcc.print("BODYFACC") );
  IFVERBOSE( bodyTAcc.print("BODYTACC") );

  int maxRepetitionCount = 0;
  for( ArrayIndex i = 0; i < batchRepetitionCount_.size(); ++i ) {
    maxRepetitionCount = max(maxRepetitionCount, batchRepetitionCount_[ i ]);
  }

  printf(" -- (CSize,BSize,AvgCPerB,MaxBRepetition)=(%d,%d,%f,%d) --\n",
         getNumConstraints(),
         getNumBodies(),
         (CudaT)getNumConstraints() / (CudaT)getNumBodies(),
         maxRepetitionCount);
}

}
