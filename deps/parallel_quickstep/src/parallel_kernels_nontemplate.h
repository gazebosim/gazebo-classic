#ifndef PARALLEL_KERNELS_NONTEMPLATE_H
#define PARALLEL_KERNELS_NONTEMPLATE_H

#define C_ID(index,i)     (index+(cStride*(i)))
#define B_ID(index,i)     (index+(bStride*(i)))

dxGlobal void
parallelZero( dxDeviceData dReal *buffer, int bufferSize )
{
  int index = dxGlobalIdxX();

  if( index < bufferSize )
    buffer[ index ] = dParallelZero;
}

dxGlobal void
parallelZero4( dxDeviceData dReal4 *buffer, int bufferSize )
{
  int index = dxGlobalIdxX();

  if( index < bufferSize )
    buffer[ index ] = make_real4( dParallelZero );
}

dxGlobal void
parallelSORLCP( dxDeviceData dReal4 *fc0_reduction,
                dxDeviceData dReal4 *fc1_reduction,
                dxDeviceData dReal *lambda,
                dxDeviceData const int4 *bodyIDs,
                dxDeviceData const int *fIDs,
                dxDeviceData const dReal4 *j,
                dxDeviceData const dReal4 *ij,
                dxDeviceData const dReal4 *fc0,
                dxDeviceData const dReal4 *fc1,
                dxDeviceData const dReal *adcfm,
                dxDeviceData const dReal *rhs,
                dxDeviceData const dReal *lohi,
                const int offset,
                const int numConstraints,
                const int bStride,
                const int cStride )
{
  int index = dxGlobalIdxX();

  if( index >= numConstraints )
    return;

  index += offset;

  dReal old_lambda = lambda[ index ];

  int4 bodyID = bodyIDs[ index ];

  dReal4 fc00 = fc0[ bodyID.x ];
  dReal4 fc01 = fc1[ bodyID.x ];
  dReal4 fc10 = make_real4( dParallelZero );
  dReal4 fc11 = make_real4( dParallelZero );

  dReal4 j0_temp = j[      index    ];
  dReal4 j1_temp = j[ C_ID(index,1) ];

  dReal  delta = rhs[ index ] - old_lambda * adcfm[ index ];

  if( bodyID.y >= 0 )  {
    fc10 = fc0[ bodyID.y ];
    fc11 = fc1[ bodyID.y ];
  }

  {
    delta -= dot( fc00, j0_temp );
    delta -= dot( fc01, j1_temp );
    if (bodyID.y >= 0) {
      dReal4 j2_temp = j[ C_ID(index,2) ];
      dReal4 j3_temp = j[ C_ID(index,3) ];
      delta -= dot( fc10, j2_temp );
      delta -= dot( fc11, j3_temp );
    }
  }

  {
    dReal lo_act = lohi[ index ];
    dReal hi_act = lohi[ C_ID(index,1) ];

    int fID = fIDs[ index ];
    if (fID >= 0) {
      hi_act = fabs( hi_act * lambda[ fID ]);
      lo_act = -hi_act;
    }

    dReal new_lambda = old_lambda + delta;
    dReal final_lambda = new_lambda;

    if (new_lambda < lo_act) {
      delta = lo_act-old_lambda;
      final_lambda = lo_act;
    }
    else if (new_lambda > hi_act) {
      delta = hi_act-old_lambda;
      final_lambda = hi_act;
    }
    lambda[ index ] = final_lambda;
  }

  j0_temp = ij[      index    ];
  j1_temp = ij[ C_ID(index,1) ];

  {
    j0_temp *= delta;
    j1_temp *= delta;

    fc0_reduction[ bodyID.z ] += j0_temp;
    fc1_reduction[ bodyID.z ] += j1_temp;

    if( bodyID.y >= 0 ) {
      dReal4 j2_temp = ij[ C_ID(index,2) ];
      dReal4 j3_temp = ij[ C_ID(index,3) ];

      j2_temp *= delta;
      j3_temp *= delta;

      fc0_reduction[ bodyID.w ] += j2_temp;
      fc1_reduction[ bodyID.w ] += j3_temp;
    }
  }
}

dxGlobal void
parallelReduce( dxDeviceData dReal4 *fc0,
                dxDeviceData dReal4 *fc1,
                dxDeviceData const dReal4 *fc0_reduction,
                dxDeviceData const dReal4 *fc1_reduction,
                const int reductionStride,
                const int bodySize,
                const int reductionSize )
{
  const int index = dxGlobalIdxX();
  if( index >= bodySize ) return;

  int nextIndex = index + reductionStride;

  dReal4 sum0 = fc0_reduction[ index ];
  dReal4 sum1 = fc1_reduction[ index ];

  while(nextIndex < reductionSize) {
    sum0 += fc0_reduction[ nextIndex ];
    sum1 += fc1_reduction[ nextIndex ];
    nextIndex += reductionStride;
  }

  fc0[ index ] += sum0;
  fc1[ index ] += sum1;
}

#endif
