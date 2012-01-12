#ifndef PARALLEL_KERNELS_H
#define PARALLEL_KERNELS_H

#define C_ID(index,i)     (index+(cStride*(i)))
#define B_ID(index,i)     (index+(bStride*(i)))

static uint2 s_blockIdx, s_blockDim, s_threadIdx;

template <typename T> static inline dxDevice T       parallel_inf()         { return 0.0; }
template <typename T> static inline dxDevice T       parallel_zero()         { return 0.0; }

template <>                  inline dxDevice float   parallel_inf<float>()  { return dxParallelInfF; }
template <>                  inline dxDevice float   parallel_zero<float>()  { return  0.0f ; }
template <>                  inline dxDevice float4  parallel_zero<float4>()  { return make_float4( 0.0f ); }
#ifdef CUDA_DOUBLESUPPORT
template <>                  inline dxDevice double  parallel_inf<double>() { return dxParallelInfD; }
template <>                  inline dxDevice double  parallel_zero<double>()  { return 0.0; }
template <>                  inline dxDevice double4 parallel_zero<double4>()  { return make_double4( 0.0 ); }
#else
template <>                  inline dxDevice double  parallel_inf <double>()   { return dxParallelInfF; }
template <>                  inline dxDevice double  parallel_zero<double>()   { return 0.0; }
template <>                  inline dxDevice double4 parallel_zero<double4>()  { return make_fdouble4(0.0 ); }
#endif

template <typename T>
dxGlobal void
cudaZeroT( T *buffer, const int bufferSize )
{
  int index = dxBlockIdx.x * dxBlockDim.x + dxThreadIdx.x;

  if( index < bufferSize )
    buffer[ index ] = parallel_zero<T>();
}

//template<typename T, bool bUseAtomics, bool bUseProjection>
template<typename T>
dxGlobal void
cudaSORLCPT( typename vec4<T>::Type *fc0_reduction,
             typename vec4<T>::Type *fc1_reduction,
             T* lambda,
             const int4 *bodyIDs,
             const int *fIDs,
             const typename vec4<T>::Type *j,
             const typename vec4<T>::Type *ij,
             const typename vec4<T>::Type *fc0,
             const typename vec4<T>::Type *fc1,
             const T* adcfm,
             const T* rhs,
             const T* hilo,
             const int offset,
             const int numConstraints,
             const int bStride,
             const int cStride )
{
  typedef typename vec4<T>::Type Vec4T;

  int index = dxBlockIdx.x * dxBlockDim.x + dxThreadIdx.x;
  //int index = dxGlobalIdxX();

  if( index >= numConstraints )
    return;

  index += offset;

  T old_lambda = lambda[ index ];

  int4 bodyID = bodyIDs[ index ];

  Vec4T fc00 = fc0[ bodyID.x ];
  Vec4T fc01 = fc1[ bodyID.x ];
  Vec4T fc10 = make_vec4( (T)0.0 );
  Vec4T fc11 = make_vec4( (T)0.0 );

  Vec4T j0_temp = j[      index    ];
  Vec4T j1_temp = j[ C_ID(index,1) ];
  Vec4T j2_temp = j[ C_ID(index,2) ];
  Vec4T j3_temp = j[ C_ID(index,3) ];

  T  delta = rhs[ index ] - old_lambda * adcfm[ index ];

  if( bodyID.y >= 0 )  {
    fc10 = fc0[ bodyID.y ];
    fc11 = fc1[ bodyID.y ];
  }

  {
    delta -= dot( fc00, j0_temp );
    delta -= dot( fc01, j1_temp );
    if (bodyID.y >= 0) {
      delta -= dot( fc10, j2_temp );
      delta -= dot( fc11, j3_temp );
    }
  }

//  if( bUseProjection ) {
    T lo_act = hilo[ index ];
    T hi_act = hilo[ C_ID(index,1) ];
    int fID = fIDs[ index ];

    if (fID >= 0) {
      hi_act = abs( hi_act * lambda[ fID ]);
      lo_act = -hi_act;
    }

    T new_lambda = old_lambda + delta;
    T final_lambda = new_lambda;

    if (new_lambda < lo_act) {
      delta = lo_act-old_lambda;
      final_lambda = lo_act;
    }
    else if (new_lambda > hi_act) {
      delta = hi_act-old_lambda;
      final_lambda = hi_act;
    }
    lambda[ index ] = final_lambda;
//  } else {
//    lambda[ index ] = old_lambda + delta;
//  }

  j0_temp = ij[      index    ];
  j1_temp = ij[ C_ID(index,1) ];
  j2_temp = ij[ C_ID(index,2) ];
  j3_temp = ij[ C_ID(index,3) ];

  {
    j0_temp *= delta;
    j1_temp *= delta;

    // STORE
    //if( bUseAtomics ) {
    //  myAtomicVecAdd<T>(fc0[ bodyID.x ], j0_temp);
    //  myAtomicVecAdd<T>(fc1[ bodyID.x ], j1_temp);
    //} else {
      fc0_reduction[ bodyID.z ] = j0_temp;
      fc1_reduction[ bodyID.z ] = j1_temp;
    //}

    if( bodyID.y >= 0 ) {
      j2_temp *= delta;
      j3_temp *= delta;

      // STORE
      //if( bUseAtomics ) {
      //  myAtomicVecAdd<T>(fc0[ bodyID.y ], j2_temp);
      //  myAtomicVecAdd<T>(fc1[ bodyID.y ], j3_temp);
      //} else {
        fc0_reduction[ bodyID.w ] = j2_temp;
        fc1_reduction[ bodyID.w ] = j3_temp;
      //}
    }
  }
}

template<typename T>
dxGlobal void
cudaReduceIterativeCompactT( typename vec4<T>::Type *fc0_reduction,
                             typename vec4<T>::Type *fc1_reduction,
                             const int treePower )
{
  typedef typename vec4<T>::Type Vec4T;

  int index = dxBlockIdx.x * dxBlockDim.x + dxThreadIdx.x;

  // Should we do an atomiceExch here to zero out the old memory?
  Vec4T fc0 = fc0_reduction[ index ];
  Vec4T fc1 = fc1_reduction[ index ];

  // row2.w is repetition counter: will gather if >= 8,4,2,1. Do nothing if =0.
  if (fc1.w >= treePower)
  {
    fc1_reduction[ index ].w = fc1.w = 0; // mark as processed
    fc0_reduction[ index - treePower ] += fc0;
    fc1_reduction[ index - treePower ] += fc1;
  }
}

template<typename T>
dxGlobal void
cudaReduceLoopedCompactT( typename vec4<T>::Type *fc0_reduction,
                          typename vec4<T>::Type *fc1_reduction,
                          const int size,
                          const int step )
{
  typedef typename vec4<T>::Type Vec4T;

  int index = dxBlockIdx.x * dxBlockDim.x + dxThreadIdx.x;
  const uint stride = dxBlockIdx.x * dxBlockDim.x;

  while(index < size) {
    // Should we do an atomiceExch here to zero out the old memory?
    Vec4T fc0 = fc0_reduction[ index ];
    Vec4T fc1 = fc1_reduction[ index ];

    const uint offset = 0x1 << (step - 1);
    const uint next = index - offset;

    if( static_cast<int>(fc1.w) & offset) {
      fc1_reduction[ index ].w = fc1.w = 0;
      fc0_reduction[ next ] += fc0;
      fc1_reduction[ next ] += fc1;
    }
    index += stride;
  }
}

template<typename T>
dxGlobal void
cudaReduceStridedT( typename vec4<T>::Type *fc0,
                    typename vec4<T>::Type *fc1,
                    const typename vec4<T>::Type *fc0_reduction,
                    const typename vec4<T>::Type *fc1_reduction,
                    const int reductionStride,
                    const int bodySize,
                    const int reductionSize )
{
  typedef typename vec4<T>::Type Vec4T;

  const int index = dxBlockIdx.x * dxBlockDim.x + dxThreadIdx.x;
  if( index >= bodySize ) return;

  int nextIndex = index + reductionStride;

  Vec4T sum0 = fc0_reduction[ index ];
  Vec4T sum1 = fc1_reduction[ index ];

  while(nextIndex < reductionSize) {
    // Should we do an atomiceExch here to zero out the old memory?
    sum0 += fc0_reduction[ nextIndex ];
    sum1 += fc1_reduction[ nextIndex ];
    nextIndex += reductionStride;
  }

  fc0[ index ] += sum0;
  fc1[ index ] += sum1;
}

template <typename T, unsigned int blockSize>
dxGlobal void
cudaReduceSequentialT( typename vec4<T>::Type *fc0,
                       typename vec4<T>::Type *fc1,
                       typename vec4<T>::Type *fc0_reduction,
                       typename vec4<T>::Type *fc1_reduction,
                       int n )
{
  typedef typename vec3<T>::Type Vec3T;
  typedef typename vec4<T>::Type Vec4T;

  dxShared Vec3T sdata0[ blockSize * 2];
  dxShared Vec3T sdata1[ blockSize * 2];

  unsigned int tid = dxThreadIdx.x;
  unsigned int bid = dxBlockIdx.x;
  unsigned int i = dxBlockIdx.x*blockSize + dxThreadIdx.x;

  if(i >= n ) return;

  // Should we do an atomiceExch here to zero out the old memory?
  Vec3T fc0Sum = make_vec3( fc0_reduction[ i ] );
  Vec3T fc1Sum = make_vec3( fc1_reduction[ i ] );

  // each thread puts its local sum into shared memory
  sdata0[ tid ] = fc0Sum;
  sdata1[ tid ] = fc1Sum;

  dxSyncthreads();

  // do reduction in shared mem
  if (blockSize >= 512) {
    if (tid < 256) {
      sdata0[tid] = fc0Sum = fc0Sum + sdata0[tid + 256];
      sdata1[tid] = fc1Sum = fc1Sum + sdata1[tid + 256];
    }
    dxSyncthreads();
  }
  if (blockSize >= 256) {
    if (tid < 128) {
      sdata0[tid] = fc0Sum = fc0Sum + sdata0[tid + 128];
      sdata1[tid] = fc1Sum = fc1Sum + sdata1[tid + 128];
    }
    dxSyncthreads();
  }
  if (blockSize >= 128) {
    if (tid < 64) {
      sdata0[tid] = fc0Sum = fc0Sum + sdata0[tid + 64];
      sdata1[tid] = fc1Sum = fc1Sum + sdata1[tid + 64];
    }
    dxSyncthreads();
  }

  if (tid < 32)
  {
    // Within a warp we require the volatile specifier for correct storage behavior
    volatile Vec3T* smem0 = sdata0;
    volatile Vec3T* smem1 = sdata1;

    if (blockSize >=  64) { add_assign_volatile(smem0[tid],fc0Sum,smem0[tid+32]);
                            add_assign_volatile(smem1[tid],fc1Sum,smem1[tid+32]); }
    if (blockSize >=  32) { add_assign_volatile(smem0[tid],fc0Sum,smem0[tid+16]);
                            add_assign_volatile(smem1[tid],fc1Sum,smem1[tid+16]); }
    if (blockSize >=  16) { add_assign_volatile(smem0[tid],fc0Sum,smem0[tid+ 8]);
                            add_assign_volatile(smem1[tid],fc1Sum,smem1[tid+ 8]); }
    if (blockSize >=   8) { add_assign_volatile(smem0[tid],fc0Sum,smem0[tid+ 4]);
                            add_assign_volatile(smem1[tid],fc1Sum,smem1[tid+ 4]); }
    if (blockSize >=   4) { add_assign_volatile(smem0[tid],fc0Sum,smem0[tid+ 2]);
                            add_assign_volatile(smem1[tid],fc1Sum,smem1[tid+ 2]); }
    if (blockSize >=   2) { add_assign_volatile(smem0[tid],fc0Sum,smem0[tid+ 1]);
                            add_assign_volatile(smem1[tid],fc1Sum,smem1[tid+ 1]); }
  }

  // write result for this block to global mem
  if(tid == 0)
  {
    fc0[ bid ] += make_vec4( sdata0[0] );
    fc1[ bid ] += make_vec4( sdata1[0] );
  }
}

template<typename T>
dxGlobal void
cudaComputeInvMJTT( int4 *bodyIDs,
                    typename vec4<T>::Type *j0,
                    typename vec4<T>::Type *j1,
                    typename vec4<T>::Type *j2,
                    typename vec4<T>::Type *j3,
                    typename vec4<T>::Type *ij0,
                    typename vec4<T>::Type *ij1,
                    typename vec4<T>::Type *ij2,
                    typename vec4<T>::Type *ij3,
                    T *iMass,
                    int numConstraints,
                    typename vec4<T>::Type *ii0,
                    typename vec4<T>::Type *ii1,
                    typename vec4<T>::Type *ii2)
{
  typedef typename vec3<T>::Type Vec3T;

  int index = dxBlockIdx.x * dxBlockDim.x + dxThreadIdx.x;

  if (index >= numConstraints)
    return;

  Vec3T ij_temp, ii_temp, j_temp;

  int4 bodyID = bodyIDs[ index ];
  int body0ID = bodyID.x;
  int body1ID = bodyID.y;

  register T k = iMass[ body0ID ];

  // Store
  ij0[ index ] = j0[ index ] * k;

  ii_temp = make_vec3( ii0[ body0ID ] );
  j_temp = make_vec3( j1[ index] );
  ij_temp.x = dot( ii_temp, j_temp );

  ii_temp = make_vec3( ii1[ body0ID ] );
  j_temp = make_vec3( j1[ index] );
  ij_temp.y = dot( ii_temp, j_temp );

  ii_temp = make_vec3( ii2[ body0ID ] );
  j_temp = make_vec3( j1[ index] );
  ij_temp.z = dot( ii_temp, j_temp );

  // Store
  ij1[ index ] = make_vec4( ij_temp );

  if( body1ID >= 0 ) {
    k = iMass[ body1ID ];

    // Store
    ij2[ index ] = j2[ index ] * k;

    ii_temp = make_vec3( ii0[ body1ID ] );
    j_temp = make_vec3( j3[ index] );
    ij_temp.x = dot( ii_temp, j_temp );

    ii_temp = make_vec3( ii1[ body1ID ] );
    j_temp = make_vec3( j3[ index] );
    ij_temp.y = dot( ii_temp, j_temp );

    ii_temp = make_vec3( ii2[ body1ID ] );
    j_temp = make_vec3( j3[ index] );
    ij_temp.z = dot( ii_temp, j_temp );

    // Store
    ij3[ index ] = make_vec4( ij_temp );

  } else {
    // Store
    ij2[ index ] = make_vec4( (T)0.0 );
    ij3[ index ] = make_vec4( (T)0.0 );
  }
}

template<typename T>
dxGlobal void
cudaComputeAdcfmBT( int4 *bodyIDs,
                    typename vec4<T>::Type *j0,
                    typename vec4<T>::Type *j1,
                    typename vec4<T>::Type *j2,
                    typename vec4<T>::Type *j3,
                    typename vec4<T>::Type *ij0,
                    typename vec4<T>::Type *ij1,
                    typename vec4<T>::Type *ij2,
                    typename vec4<T>::Type *ij3,
                    T *adcfm,
                    T *rhs,
                    T sorParam, int numConstraints)
{
  typedef typename vec3<T>::Type Vec3T;

  int index = dxBlockIdx.x * dxBlockDim.x + dxThreadIdx.x;

  if (index >= numConstraints)
    return;

  int body1ID = bodyIDs[ index ].y;

  register T adcfm_i = 0.0;
  register T cfm_i = adcfm[ index ];
  register Vec3T j0_temp = make_vec3( j0[ index ] );
  register Vec3T j1_temp = make_vec3( j1[ index ] );
  register Vec3T ij0_temp = make_vec3( ij0[ index ] );
  register Vec3T ij1_temp = make_vec3( ij1[ index ] );

  {
    adcfm_i += dot( j0_temp, ij0_temp );
    adcfm_i += dot( j1_temp, ij1_temp );

    if(body1ID >= 0) {
      j0_temp = make_vec3( j2[ index ] );
      j1_temp = make_vec3( j3[ index ] );
      ij0_temp = make_vec3( ij2[ index ] );
      ij1_temp = make_vec3( ij3[ index ] );

      adcfm_i += dot( j0_temp, ij0_temp );
      adcfm_i += dot( j1_temp, ij1_temp );
    }
    adcfm_i = sorParam / (adcfm_i + cfm_i);
  }

  {
    j0[ index ] *= adcfm_i;
    j1[ index ] *= adcfm_i;
    j2[ index ] *= adcfm_i;
    j3[ index ] *= adcfm_i;
    rhs[ index ] *= adcfm_i;
    adcfm[ index ] = adcfm_i + cfm_i;
  }

}

template<typename T>
dxGlobal void
cudaIntegrateT( typename vec4<T>::Type* pos,
                typename vec4<T>::Type* lVel,
                typename vec4<T>::Type* aVel,
                float deltaTime,
                int numConstraints)
{
    int index = dxBlockIdx.x * dxBlockDim.x + dxThreadIdx.x;

    if (index >= numConstraints)
        return;

    //typename vec4<T>::Type position = pos[index];
    //typename vec3<T>::Type accel = computeBodyAccel<T, multithreadBodies>(position, oldPos, numBodies);
}

#endif
