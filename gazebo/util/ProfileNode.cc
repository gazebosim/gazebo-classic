#include "gazebo/util/ProfileNode.hh"

namespace gazebo 
{
  namespace util 
  {
    ProfileNode::ProfileNode( const char * name, ProfileNode * parent ) :
      Name( name ),
      TotalCalls( 0 ),
      TotalTime( 0 ),
      StartTime( 0 ),
      RecursionCounter( 0 ),
      Parent( parent ),
      Child( nullptr ),
      Sibling( nullptr )
    {
      Reset();
    }

    void ProfileNode::CleanupMemory()
    {
      delete ( Child);
      Child = nullptr;
      delete ( Sibling);
      Sibling = nullptr;
    }

    ProfileNode::~ProfileNode( void )
    {
      CleanupMemory();
    }

    ProfileNode * ProfileNode::Get_Sub_Node( const char * name )
    {
      // Try to find this sub node
      ProfileNode * child = Child;
      while ( child ) {
        if ( child->Name == name ) {
          return child;
        }
        child = child->Sibling;
      }

      // We didn't find it, so add it

      ProfileNode * node = new ProfileNode( name, this );
      node->Sibling = Child;
      Child = node;
      return node;
    }

    void ProfileNode::Reset( void )
    {
      TotalCalls = 0;
      TotalTime = 0.0f;

      if ( Child ) {
        Child->Reset();
      }
      if ( Sibling ) {
        Sibling->Reset();
      }
    }

    void ProfileNode::Call( void )
    {
      TotalCalls++;
      if (RecursionCounter++ == 0) {
        StartTime.SetToWallTime();
      }
    }

    bool ProfileNode::Return( void )
    {
      if ( --RecursionCounter == 0 && TotalCalls != 0 ) {
        auto cur_time = gazebo::common::Time::GetWallTime();
        auto delta_t = cur_time - StartTime;
        TotalTime += delta_t;
      }
      return ( RecursionCounter == 0 );
    }
  }
}


