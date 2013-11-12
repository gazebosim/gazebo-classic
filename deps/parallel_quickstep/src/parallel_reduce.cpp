#include <parallel_reduce.h>
#include <parallel_utils.h>

namespace parallel_ode
{

using ::parallel_utils::fillSequentialVector;
using ::parallel_utils::fillStridedVector;
using ::parallel_utils::iPower2Up;

void ReduceStrategy::initialize( int bodySize, int maxBodyRepetitionCount, const IntVector& batchRepetitionCount )
{
  setBodySize( bodySize );
  setBodySizeWithReduction( bodySize );
  setBodyStride( 1 );
  setBodyOffsetStride( 0 );
}

//////////////////////////////////////////////////////////////////////////////////////////////

void SequentialReduceStrategy::initialize( int bodySize, int maxBodyRepetitionCount, const IntVector& batchRepetitionCount )
{
  setBodySize( bodySize );
  setBodyStride( iPower2Up( maxBodyRepetitionCount ) );
  setBodySizeWithReduction( getBodyStride( ) * getBodySize( ) );
  setBodyOffsetStride( 1 );
}

//////////////////////////////////////////////////////////////////////////////////////////////

void StridedReduceStrategy::initialize( int bodySize, int maxBodyRepetitionCount, const IntVector& batchRepetitionCount )
{
  setBodySize( bodySize );
  setBodyOffsetStride( alignDefaultSize( getBodySize( ) ) );
  setBodySizeWithReduction( getBodyOffsetStride( ) * maxBodyRepetitionCount );
  setBodyStride( 1 );
}

//////////////////////////////////////////////////////////////////////////////////////////////

void CompactReduceStrategy::initialize( int bodySize, int maxBodyRepetitionCount, const IntVector& batchRepetitionCount )
{
  setBodySize( bodySize );
  int tempReductionSize = 0;
  for(size_t batchID = 0; batchID < batchRepetitionCount.size( ); ++batchID) {
    tempReductionSize += batchRepetitionCount[ batchID ];
  }
  setBodySizeWithReduction( tempReductionSize );
  setBodyStride( -1 );
  setBodyOffsetStride( 1 );
}

}
