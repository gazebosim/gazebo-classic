#include <parallel_batch.h>
#include <parallel_utils.h>

#include <map>
#include <boost/unordered_map.hpp>

namespace parallel_ode
{

using ::parallel_utils::permuteVector;
using ::parallel_utils::fillSequentialVector;

typedef boost::unordered_map<int, int> BatchMap;
typedef std::multimap<int, int, std::greater<int> > BatchMultimap;
typedef BatchMultimap::iterator BatchMultimapIterator;

int BatchStrategy::baseBatch( const int* pairList,
                              const BatchVector& constraintIndices,
                              const BatchVector& batchSizes,
                              BatchVector& batchIndices,
                              BatchVector& bodyRepetitionCount0,
                              BatchVector& bodyRepetitionCount1,
                              BatchVector& maxBodyRepetitionCount )
{
  batchIndicesFromBatchSizes( batchSizes, batchIndices, isAligning( ), getAlignment( ) );

  return batchRepetitionCount( pairList, constraintIndices, batchSizes, bodyRepetitionCount0, bodyRepetitionCount1, maxBodyRepetitionCount );
}

void BatchStrategy::batchIndicesFromBatchSizes( const BatchVector& batchSizes, BatchVector& batchIndices, bool bAlign, int alignment )
{
  batchIndices.resize( batchSizes.size() );

  for( size_t batchID = 0, batchOffset = 0; batchID < batchSizes.size(); batchID++ )
  {
    if( bAlign ) alignOffset( batchOffset, alignment );
    batchIndices[ batchID ] = batchOffset;
    batchOffset += batchSizes[ batchID ];
  }
}

int BatchStrategy::batchRepetitionCount( const int *pairList,
                                         const BatchVector& constraintIndices,
                                         const BatchVector& batchSizes,
                                         BatchVector& bodyRepetitionCount0,
                                         BatchVector& bodyRepetitionCount1,
                                         BatchVector& maxBodyRepetitionCountInBatch )
{
  maxBodyRepetitionCountInBatch.resize( getMaxBatches() );

  int maxRepetitionCount = 0;
  BatchVector bodyRepetitionMap( bodyRepetitionCount0.size() );

  for( size_t batchID = 0, constraintID = 0; batchID < batchSizes.size(); batchID++ )
  {
    bodyRepetitionMap.assign( bodyRepetitionMap.size(), 0 );
    int maxRepetitionCountInBatch = 0;

    for( int iter = 0; iter < batchSizes[ batchID ]; ++iter,++constraintID )
    {
      int bodyID0 = pairList[ constraintIndices[ constraintID ]*2 ] ;
      int bodyID1 = pairList[ constraintIndices[ constraintID ]*2+1 ] ;

      if( bodyID0 >= 0 ) {
        bodyRepetitionCount0[ constraintID ] = bodyRepetitionMap[ bodyID0 ]++;
        maxRepetitionCountInBatch = std::max( maxRepetitionCountInBatch, bodyRepetitionMap[ bodyID0 ] );
      }
      if( bodyID1 >= 0 ) {
        bodyRepetitionCount1[ constraintID ] = bodyRepetitionMap[ bodyID1 ]++;
        maxRepetitionCountInBatch = std::max( maxRepetitionCountInBatch, bodyRepetitionMap[ bodyID1 ] );
      }
    }

    maxBodyRepetitionCountInBatch[ batchID ] = maxRepetitionCountInBatch;
    maxRepetitionCount = std::max( maxRepetitionCount, maxRepetitionCountInBatch );
  }

  return maxRepetitionCount;
}

int BatchStrategy::batch( const int* pairList, const int numConstraints, const int numBodies,
                          BatchVector& constraintIndices,
                          BatchVector& bodyRepetitionCount0,
                          BatchVector& bodyRepetitionCount1,
                          BatchVector& maxBodyRepetitionCountInBatch,
                          BatchVector& batchIndices,
                          BatchVector& batchSizes )
{
  batchSizes.resize( getMaxBatches() );
  batchSizes.assign( getMaxBatches(), 0 );

  // Place all constraints in the first batch
  fillSequentialVector( constraintIndices );
  permuteVector( constraintIndices );

  const int batchSize = numConstraints / getMaxBatches( );
  batchSizes[ 0 ] = batchSize + ( numConstraints % batchSize );
  for( int batchID = 1; batchID < getMaxBatches(); ++batchID ) {
    batchSizes[ batchID ] = batchSize;
  }

  return baseBatch( pairList, constraintIndices, batchSizes, batchIndices, bodyRepetitionCount0, bodyRepetitionCount1, maxBodyRepetitionCountInBatch );
}

int GreedyBatchStrategy::batch( const int* pairList, const int numConstraints, const int numBodies,
                                BatchVector& constraintIndices,
                                BatchVector& bodyRepetitionCount0,
                                BatchVector& bodyRepetitionCount1,
                                BatchVector& maxBodyRepetitionCountInBatch,
                                BatchVector& batchIndices,
                                BatchVector& batchSizes )
{
  int constraintsUsed = 0;
  std::vector<bool> constraintAssigned( numConstraints, false );
  std::vector<bool> bodyUsed( numBodies );

  batchSizes.resize( getMaxBatches() );
  batchSizes.assign( getMaxBatches(), 0 );
  constraintIndices.assign( numConstraints, -1 );

  int currentBatchSize = 0;

  // First we assign constraints to a batch, without assigning them particular ordered indices
  do {
    for(int batchID = 0; batchID < getMaxBatches(); ++batchID)
    {
      currentBatchSize = 0;
      bodyUsed.assign( numBodies, false );

      for(int constraintID = 0; constraintID < numConstraints; ++constraintID)
      {
        if( constraintAssigned[ constraintID ] ) continue;

        const int b1 = pairList[ constraintID*2 ];
        const int b2 = pairList[ constraintID*2 + 1];

        if((b1 < 0 || !bodyUsed[ b1 ]) && ( b2 < 0 || !bodyUsed[ b2 ]))
        {
          if( b1 >= 0 ) bodyUsed[ b1 ] = true;
          if( b2 >= 0 ) bodyUsed[ b2 ] = true;
          constraintAssigned[ constraintID ] = true;
          constraintIndices[ constraintID ] = batchID;
          ++constraintsUsed;
          ++currentBatchSize;
        }
      }

      batchSizes[ batchID ] += currentBatchSize;
    }
  } while( constraintsUsed < numConstraints);

  BatchVector batchSizesCopy = batchSizes;
  BatchVector constraintIndicesCopy = constraintIndices;

  // Now recover the ordered constraint indices from their batch assignment
  for(int constraintID = 0; constraintID < numConstraints; ++constraintID) {
    const int constraintBatchID = constraintIndicesCopy[ constraintID ];
    const int newConstraintID = --batchSizesCopy[ constraintBatchID ];
    constraintIndices[ constraintID ] = newConstraintID;
  }

  return baseBatch( pairList, constraintIndices, batchSizes, batchIndices, bodyRepetitionCount0, bodyRepetitionCount1, maxBodyRepetitionCountInBatch );
}

int ColoringBatchStrategy::batch( const int* pairList, const int numConstraints, const int numBodies,
                                  BatchVector& constraintIndices,
                                  BatchVector& bodyRepetitionCount0,
                                  BatchVector& bodyRepetitionCount1,
                                  BatchVector& maxBodyRepetitionCountInBatch,
                                  BatchVector& batchIndices,
                                  BatchVector& batchSizes )
{
  BatchMultimap bodyConstraintMap;
  BatchMultimap constraintHeap;

  std::vector<BatchMultimapIterator> pointers( numConstraints );
  BatchVector colors( numConstraints, std::numeric_limits<int>::max() );

  int maxDegree( 0 ), numColors( 1 ), degree;

  // Build body to constraint map
  for( int constraintID = 0; constraintID < numConstraints; ++constraintID )
  {
    bodyConstraintMap.insert( std::make_pair( pairList[ constraintID*2 ], constraintID ) );
    bodyConstraintMap.insert( std::make_pair( pairList[ constraintID*2 + 1 ], constraintID ) );
  }

  // Find the maximal degree, and degree, for each constraint
  for( int constraintID  = 0; constraintID < numConstraints; ++constraintID ) {
    const int b1( pairList[ constraintID*2 ] );
    const int b2( pairList[ constraintID*2 + 1 ] );

    int degree = 0;

    if( b1 >= 0 && b2 >= 0)
      degree = bodyConstraintMap.count( b1 ) + bodyConstraintMap.count( b2 ) - 1;
    else if( b1 >= 0 )
      degree = bodyConstraintMap.count( b1 );
    else if( b2 >= 0 )
      degree = bodyConstraintMap.count( b2 );

    if( degree > 0 ) {
      pointers[ constraintID ] = constraintHeap.insert( std::make_pair( degree, constraintID ) );
      maxDegree = std::max( maxDegree, degree );
    }
  }

  BatchVector occurrences( getMaxBatches() );
  BatchVector colorCount( numColors, 0 );

  while( !constraintHeap.empty() ) {
    int constraintID = constraintHeap.begin()->second;
    constraintHeap.erase( constraintHeap.begin() );
    occurrences.assign( numColors, 0 );

    const int bodyIDs[2] = { pairList[ constraintID*2 ], pairList[ constraintID*2 + 1 ] };

    for( int j = 0; j < 2; ++j) {

      BatchMultimapIterator it = bodyConstraintMap.find( bodyIDs[ j ] );
      if( it == bodyConstraintMap.end() ) continue;
      BatchMultimapIterator end = bodyConstraintMap.upper_bound( bodyIDs[ j ] );

      for(; it != end; ++it ) {
        int bodyConstraintID = it->second;
        if( bodyConstraintID == constraintID ) continue;

        if( colors[ bodyConstraintID ] < numColors ) {
          // Exclude color
          ++occurrences[ colors[ bodyConstraintID ] ];
        }
        else {
          // Decrement degree ( the constraint has been assigned )
          degree = pointers[ bodyConstraintID ]->first;
          if( pointers[bodyConstraintID] == constraintHeap.begin() ) {
            constraintHeap.erase( pointers[ bodyConstraintID ] );
            pointers[ bodyConstraintID ] = constraintHeap.insert( std::make_pair( degree - 1, bodyConstraintID ) );
          }
          else {
            constraintHeap.erase( pointers[ bodyConstraintID ]-- );
            pointers[ bodyConstraintID ] = constraintHeap.insert( pointers[ bodyConstraintID ], std::make_pair( degree - 1, bodyConstraintID ) );
          }
        }
      }
    }

    // Ensure creation of a well-balanced coloring
    int minColor = 0;
    for( int colorID = 1; colorID < numColors; ++colorID )
    {
      if( ( occurrences[ colorID ] <  occurrences[ minColor ] ) ||
          ( occurrences[ colorID ] == occurrences[ minColor ] && colorCount[ colorID ] < colorCount[ minColor ] ) )
      {
        minColor = colorID;
      }
    }

    if( occurrences[ minColor ] > 0 && numColors < getMaxBatches() )
    {
      colors[ constraintID ] = numColors++;
      colorCount.resize( numColors );
      colorCount[ colors[ constraintID ] ] = 1;
    }
    else
    {
      colors[ constraintID ] = minColor;
      colorCount[ colors[ constraintID ] ]++;
    }
  }

  batchSizes.resize( getMaxBatches(), 0 );
  batchSizes = colorCount;
  batchIndicesFromBatchSizes( batchSizes, batchIndices, isAligning( ), getAlignment( ) );

  // Assign indices from colors
  for( int constraintID = 0; constraintID < numConstraints; ++constraintID ) {
    const int constraintBatchID = colors[ constraintID ];
    constraintIndices[ constraintID ] = batchIndices[ constraintBatchID ] + ( batchSizes[ constraintBatchID ] - colorCount[ constraintBatchID ] );
    --colorCount[ constraintBatchID ];
  }

  return batchRepetitionCount( pairList, constraintIndices, batchSizes, bodyRepetitionCount0, bodyRepetitionCount1, maxBodyRepetitionCountInBatch );
}

}
